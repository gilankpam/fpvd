"""Single front-door HTTP API: /config /apply /reset /defaults /status /healthz.
Unified tree over the GS store + drone (via the LinkCoordinator and DroneClient
used internally by /apply and the facade). Transport-free `handle()`."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from .schema import SchemaError
from .config import deep_merge
from .facade import build_config_tree, split_patch, FacadeError
from .drone_client import DroneRejected, DroneUnreachable
from .dynlink.config_build import make_dl_snapshot
from .pixelpilot import render_pixelpilot_argv, render_pixelpilot_env
from .probe.config_build import make_probe_snapshot


class Api:
    def __init__(self, store, schema, render_mod, runner, drone, link,
                 status_fn, cfg_out, dynlink=None, pixelpilot=None, probe=None,
                 drone_cache=None):
        self.store = store
        self.schema = schema
        self.render_mod = render_mod
        self.runner = runner
        self.drone = drone
        self.link = link
        self.status_fn = status_fn
        self.cfg_out = cfg_out
        self.dynlink = dynlink
        self.pixelpilot = pixelpilot
        self.probe = probe
        self.drone_cache = drone_cache
        self._drone_dirty = False

    def _json(self, body: bytes) -> dict:
        return json.loads(body or b"{}")

    def handle(self, method, path, query, body):
        try:
            key = (method, path)
            if key == ("GET", "/healthz"):
                return 200, {"ok": True}
            if key == ("GET", "/defaults"):
                return 200, self.store.defaults()
            if key == ("GET", "/config"):
                pending = query.get("pending", ["false"])[0] == "true"
                gs_cfg = self.store.pending() if pending else self.store.effective()
                drone_cfg, meta = self.drone_cache.read()
                return 200, build_config_tree(gs_cfg, drone_cfg, meta)
            if key == ("PATCH", "/config"):
                return self._patch_config(self._json(body))
            if key == ("POST", "/apply"):
                return self._apply()
            if key == ("POST", "/reset"):
                self.store.reset()
                self.render_mod.write_cfg(self.cfg_out,
                                          self.render_mod.render_cfg(self.store.effective()))
                self.runner.restart()
                return 200, {"reset": True}
            if key == ("GET", "/status"):
                return 200, self.status_fn()
            return 404, {"error": "not found"}
        except SchemaError as e:
            return 400, {"error": str(e)}
        except Exception as e:  # surfaced, never silent
            return 500, {"error": str(e)}

    def _patch_config(self, patch):
        """Route a unified sparse PATCH to the GS pending store and the drone.

        Order matters: validate the GS portion (no mutation) -> proxy the drone
        portion -> patch the GS pending. A drone rejection leaves GS pending
        clean; a GS validation failure never touches the drone."""
        try:
            gs_sparse, drone_sparse, _ = split_patch(patch)
        except FacadeError as e:
            return 400, {"error": "bad_config", "message": str(e)}
        # 1) validate the GS portion locally (no mutation) by merging onto pending
        if gs_sparse:
            merged = deep_merge(self.store.pending(), gs_sparse)
            try:
                self.schema.validate_effective(merged)
            except SchemaError as e:
                return 400, {"error": "bad_config", "message": str(e)}
        # 2) proxy the drone portion (drone validates; a reject leaves GS untouched)
        if drone_sparse:
            try:
                self.drone.patch_config(drone_sparse)
                self._drone_dirty = True
            except DroneRejected as e:
                return 400, {"error": "drone_rejected", "message": e.message,
                             "details": e.body}
            except DroneUnreachable:
                return 502, {"error": "drone_unreachable"}
        # 3) patch the GS pending
        if gs_sparse:
            self.store.patch(gs_sparse)
        # return the unified pending tree
        drone_cfg, meta = self.drone_cache.read() if self.drone_cache else (
            None, {"droneReachable": False, "droneStale": True, "droneLastSeen": None})
        return 200, build_config_tree(self.store.pending(), drone_cfg, meta)

    @staticmethod
    def _without(cfg: dict, *keys) -> dict:
        return {k: v for k, v in cfg.items() if k not in keys}

    def _apply(self):
        pending = self.store.pending()
        effective = self.store.effective()

        # validate up front (idempotent with the coordinator's own validate)
        self.schema.validate_effective(pending)

        # hard-gate: a dynamicLink.enabled toggle requires the drone reachable (on AND off)
        en_old = bool(effective.get("dynamicLink", {}).get("enabled", False))
        en_new = bool(pending.get("dynamicLink", {}).get("enabled", False))
        if en_old != en_new and not self.drone.healthz():
            return 409, {"applied": False,
                         "error": "dynamicLink.enabled requires the drone reachable"}

        result = {"applied": True}

        # --- shared-link lane (coordinator): renders pending + retune/bounce + drone push; NO commit
        link_changed = pending.get("link") != effective.get("link")
        coord = None
        if link_changed:
            coord = self.link.apply_link(commit=False)
            result["sharedLink"] = coord
            if not coord.get("gsApplied"):
                return 500, {"applied": False, "sharedLink": coord,
                             "error": "link apply failed"}

        # --- GS-local wfb lane: render+bounce for non-link/dynamicLink/pixelpilot changes,
        #     unless the coordinator already bounced (it renders the whole pending) ---
        coord_bounced = bool(coord) and coord.get("mode") == "bounce"
        wfb_changed = (self._without(pending, "link", "dynamicLink", "pixelpilot")
                       != self._without(effective, "link", "dynamicLink", "pixelpilot"))
        wfb_bounced = False
        if wfb_changed and not coord_bounced:
            self.render_mod.write_cfg(self.cfg_out, self.render_mod.render_cfg(pending))
            if not self.runner.restart():
                self.render_mod.restore_bak(self.cfg_out)
                self.runner.restart()
                return 500, {"applied": False,
                             "error": "runner failed; rolled back to last-good cfg"}
            wfb_bounced = True

        # --- GS-local controllers (no runner bounce) ---
        self._route_dynamic_link(effective.get("dynamicLink", {}),
                                 pending.get("dynamicLink", {}), pending)
        self._route_pixelpilot(effective.get("pixelpilot", {}),
                               pending.get("pixelpilot", {}), pending)

        # --- commit the GS store ONCE (covers the link + GS-local lanes) ---
        self.store.commit()
        result["gs"] = {"applied": True, "wfbBounced": wfb_bounced or coord_bounced}

        # --- drone lane ---
        # NOTE: when the shared-link lane ran, the coordinator's drone.apply()
        # already committed the drone's whole pending (incl. any drone-section
        # change proxied at PATCH time), so this apply can be a harmless no-op
        # (the drone diffs internally and restarts nothing). Kept unconditional
        # so a GS-only or drone-only apply (no link change) still fires the drone.
        if self._drone_dirty:
            try:
                self.drone.apply()
                result["drone"] = {"fired": True, "applied": True}
            except DroneUnreachable:
                result["drone"] = {"fired": True, "applied": False, "reachable": False}
            except DroneRejected as e:
                result["drone"] = {"fired": True, "applied": False, "error": e.message}
            self._drone_dirty = False
        else:
            result["drone"] = {"fired": False, "applied": False}

        return 200, result

    def _route_dynamic_link(self, dl_old, dl_new, pending):
        """Start/stop/reconfigure the in-process controller AND the observe-only
        probe (they share a lifecycle). Never bounces the wfb runner."""
        if self.dynlink is None:
            return
        was, now = bool(dl_old.get("enabled")), bool(dl_new.get("enabled"))
        if not was and now:
            self.dynlink.set_config(make_dl_snapshot(pending))
            self.dynlink.start()
            if self.probe is not None:
                self.probe.set_config(make_probe_snapshot(pending))
                self.probe.start()
        elif was and not now:
            self.dynlink.stop()
            if self.probe is not None:
                self.probe.stop()
        elif was and now and dl_old != dl_new:
            self.dynlink.set_config(make_dl_snapshot(pending))
            if self.probe is not None:
                self.probe.set_config(make_probe_snapshot(pending))

    def _route_pixelpilot(self, pp_old, pp_new, pending):
        """Start/stop/restart the PixelPilot child. Never bounces the wfb
        runner. Mirrors _route_dynamic_link (set_argv ≈ set_config)."""
        if self.pixelpilot is None or pp_old == pp_new:
            return
        # enabled defaults to True here (unlike dynamicLink's False): pixelpilot
        # ships enabled in defaults.json and App boot-starts it under the same
        # default, so a missing key means "running", not "off".
        was, now = bool(pp_old.get("enabled", True)), bool(pp_new.get("enabled", True))
        if now:
            self.pixelpilot.set_argv(render_pixelpilot_argv(pending))
            self.pixelpilot.set_env(render_pixelpilot_env(pending))
            if was:
                self.pixelpilot.restart()
            else:
                self.pixelpilot.start()
        elif was and not now:
            self.pixelpilot.stop()


def make_http_server(api, host, port):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _dispatch(self, method):
            parsed = urlparse(self.path)
            n = int(self.headers.get("content-length", 0))
            body = self.rfile.read(n) if n else b""
            code, obj = api.handle(method, parsed.path, parse_qs(parsed.query), body)
            data = obj if isinstance(obj, (bytes, bytearray)) else json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self._dispatch("GET")

        def do_POST(self):
            self._dispatch("POST")

        def do_PATCH(self):
            self._dispatch("PATCH")

    return ThreadingHTTPServer((host, port), Handler)
