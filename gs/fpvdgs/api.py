"""Single front-door HTTP API: GS-local config under /gs/* (/gs/config /gs/apply
/gs/reset /gs/defaults /gs/status), an opaque /air/* drone proxy, and /healthz at
root. The GS-local link delta is applied here (live iw retune vs. runner bounce);
cross-device link orchestration moves to the client. Transport-free `handle()`."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from .schema import SchemaError
from .dynlink.config_build import make_dl_snapshot
from .pixelpilot import render_pixelpilot_argv, render_pixelpilot_env
from .probe.config_build import make_probe_snapshot


class Api:
    def __init__(self, store, schema, render_mod, runner, drone,
                 status_fn, cfg_out, dynlink=None, pixelpilot=None, probe=None,
                 retune=None, wlans_resolver=None, armer_tick=None,
                 idr_relay=None):
        self.store = store
        self.schema = schema
        self.render_mod = render_mod
        self.runner = runner
        self.drone = drone
        self.status_fn = status_fn
        self.cfg_out = cfg_out
        self.dynlink = dynlink
        self.pixelpilot = pixelpilot
        self.probe = probe
        self.retune = retune
        self.wlans_resolver = wlans_resolver
        self.armer_tick = armer_tick
        self.idr_relay = idr_relay

    def _json(self, body: bytes) -> dict:
        return json.loads(body or b"{}")

    def handle(self, method, path, query, body):
        try:
            if path.startswith("/air/"):
                return self._proxy(method, path, body)
            key = (method, path)
            if key == ("GET", "/healthz"):
                return 200, {"ok": True}
            if key == ("GET", "/gs/defaults"):
                return 200, self.store.defaults()
            if key == ("GET", "/gs/config"):
                pending = query.get("pending", ["false"])[0] == "true"
                return 200, (self.store.pending() if pending else self.store.effective())
            if key == ("PATCH", "/gs/config"):
                sparse = self._json(body)
                self.schema.validate_config_patch(sparse)
                self.store.patch(sparse)
                return 200, self.store.pending()
            if key == ("POST", "/gs/apply"):
                return self._apply_gs()
            if key == ("POST", "/gs/reset"):
                self.store.reset()
                self.render_mod.write_cfg(self.cfg_out,
                                          self.render_mod.render_cfg(self.store.effective()))
                self.runner.restart()
                return 200, {"reset": True}
            if key == ("GET", "/gs/status"):
                return 200, self.status_fn()
            return 404, {"error": "not found"}
        except SchemaError as e:
            return 400, {"error": str(e)}
        except Exception as e:  # surfaced, never silent
            return 500, {"error": str(e)}

    @staticmethod
    def _without(cfg: dict, *keys) -> dict:
        return {k: v for k, v in cfg.items() if k not in keys}

    def _apply_gs(self):
        pending = self.store.pending()
        effective = self.store.effective()
        self.schema.validate_effective(pending)

        link_changed = pending.get("link") != effective.get("link")
        wfb_changed = (self._without(pending, "dynamicLink", "pixelpilot",
                                     "idrForward", "link")
                       != self._without(effective, "dynamicLink", "pixelpilot",
                                        "idrForward", "link"))

        if link_changed or wfb_changed:
            # Render the cfg the runner reads on a (re)start, before applying.
            self.render_mod.write_cfg(self.cfg_out,
                                      self.render_mod.render_cfg(pending))
            if not self._apply_link_local(effective.get("link", {}),
                                          pending.get("link", {}),
                                          force_bounce=wfb_changed):
                self.render_mod.write_cfg(self.cfg_out,
                                          self.render_mod.render_cfg(effective))
                self.runner.restart()
                return 500, {"applied": False,
                             "error": "apply failed; rolled back to last-good cfg"}

        self._route_dynamic_link(effective.get("dynamicLink", {}),
                                 pending.get("dynamicLink", {}), pending)
        self._route_pixelpilot(effective.get("pixelpilot", {}),
                               pending.get("pixelpilot", {}), pending)
        self._route_idr_forward(effective.get("idrForward", {}),
                                pending.get("idrForward", {}), pending)
        self.store.commit()
        if self.armer_tick is not None:
            self.armer_tick()
        return 200, {"applied": True}

    @staticmethod
    def _bw_class(width):
        return 40 if width == 40 else 20

    def _can_retune_live(self, old, new):
        """Live iw retune is safe only when changes are limited to fields iw can
        apply on a running monitor card AND the radiotap BW class is unchanged.
        beamforming is reconciled by the armer, so it is excluded here."""
        if self.retune is None:
            return False
        changed = {k for k in set(old) | set(new)
                   if k != "beamforming" and old.get(k) != new.get(k)}
        if not changed <= {"channel", "width", "txPowerDbm", "region"}:
            return False
        return self._bw_class(old.get("width")) == self._bw_class(new.get("width"))

    def _apply_link_local(self, old_link, new_link, force_bounce=False):
        """Apply the GS-local link delta: live retune when possible, else bounce.
        No drone push (client orchestrates). Returns True on success."""
        non_bf_changed = any(k != "beamforming" and old_link.get(k) != new_link.get(k)
                             for k in set(old_link) | set(new_link))
        if not force_bounce and non_bf_changed and self._can_retune_live(old_link, new_link):
            if self.retune(new_link):
                return True
            # live retune failed -> fall back to a bounce
        return self.runner.restart()

    def _route_idr_forward(self, old, new, pending):
        """Start/stop the always-available IDR relay on idrForward changes.
        Independent of dynamicLink. Never bounces the wfb runner."""
        if self.idr_relay is None or old == new:
            return
        was = bool(old.get("enabled", True))
        now = bool(new.get("enabled", True))
        if now and not was:
            self.idr_relay.start()
        elif was and not now:
            self.idr_relay.stop()
        elif now and was and old.get("port") != new.get("port"):
            self.idr_relay.stop()
            self.idr_relay.start()

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

    def _proxy(self, method, path, body):
        sub = path[len("/air"):]  # "/config", "/apply", "/status"
        endpoint_method = {"GET": "GET", "PATCH": "PATCH", "POST": "POST"}.get(method)
        if endpoint_method is None:
            return 405, {"error": "method not allowed"}
        try:
            code, raw = self.drone._request(endpoint_method, sub,
                                            self._json(body) if body else None)
            return code, json.loads(raw or b"{}")
        except Exception as e:
            return 502, {"error": f"drone unreachable: {e}"}


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
