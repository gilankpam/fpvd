"""Single front-door HTTP API: /config /apply /reset /defaults /status /healthz,
opaque /air/* drone proxy, and /link coordinator. Transport-free `handle()`."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from .schema import SchemaError
from .dynlink.config_build import make_dl_snapshot


class Api:
    def __init__(self, store, schema, render_mod, runner, drone, link,
                 status_fn, cfg_out, dynlink=None):
        self.store = store
        self.schema = schema
        self.render_mod = render_mod
        self.runner = runner
        self.drone = drone
        self.link = link
        self.status_fn = status_fn
        self.cfg_out = cfg_out
        self.dynlink = dynlink

    def _json(self, body: bytes) -> dict:
        return json.loads(body or b"{}")

    def handle(self, method, path, query, body):
        try:
            if path.startswith("/air/"):
                return self._proxy(method, path, body)
            key = (method, path)
            if key == ("GET", "/healthz"):
                return 200, {"ok": True}
            if key == ("GET", "/defaults"):
                return 200, self.store.defaults()
            if key == ("GET", "/config"):
                pending = query.get("pending", ["false"])[0] == "true"
                return 200, (self.store.pending() if pending else self.store.effective())
            if key == ("PATCH", "/config"):
                sparse = self._json(body)
                self.schema.validate_config_patch(sparse)
                self.store.patch(sparse)
                return 200, self.store.pending()
            if key == ("POST", "/apply"):
                return self._apply_gs()
            if key == ("POST", "/reset"):
                self.store.reset()
                self.render_mod.write_cfg(self.cfg_out,
                                          self.render_mod.render_cfg(self.store.effective()))
                self.runner.restart()
                return 200, {"reset": True}
            if key == ("GET", "/status"):
                return 200, self.status_fn()
            if key == ("GET", "/link"):
                return 200, self._link_view()
            if key == ("PATCH", "/link"):
                sparse = self._json(body)
                self.schema.validate_link_patch(sparse)
                self.store.patch(sparse)
                return 200, self.store.pending().get("link", {})
            if key == ("POST", "/link/apply"):
                apply_to = self._json(body).get("applyTo", "both")
                return 200, self.link.apply_link(apply_to)
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
        # Guard: link drift must go through /link/apply (drone coordination).
        if pending.get("link") != effective.get("link"):
            return 409, {"error": "link changed; use POST /link/apply"}
        self.schema.validate_effective(pending)

        # Anything outside dynamicLink (link already equal) needs the runner.
        non_dl_changed = (self._without(pending, "dynamicLink")
                          != self._without(effective, "dynamicLink"))
        if non_dl_changed:
            self.render_mod.write_cfg(self.cfg_out,
                                      self.render_mod.render_cfg(pending))
            if not self.runner.restart():
                self.render_mod.restore_bak(self.cfg_out)
                self.runner.restart()
                return 500, {"applied": False,
                             "error": "runner failed; rolled back to last-good cfg"}

        self._route_dynamic_link(effective.get("dynamicLink", {}),
                                 pending.get("dynamicLink", {}), pending)
        self.store.commit()
        return 200, {"applied": True}

    def _route_dynamic_link(self, dl_old, dl_new, pending):
        """Start/stop/reconfigure the in-process controller. Never bounces
        the wfb runner."""
        if self.dynlink is None:
            return
        was, now = bool(dl_old.get("enabled")), bool(dl_new.get("enabled"))
        if not was and now:
            self.dynlink.set_config(make_dl_snapshot(pending))
            self.dynlink.start()
        elif was and not now:
            self.dynlink.stop()
        elif was and now and dl_old != dl_new:
            self.dynlink.set_config(make_dl_snapshot(pending))

    def _link_view(self):
        link = dict(self.store.effective().get("link", {}))
        reachable = self.drone.healthz()
        link["droneReachable"] = reachable
        return link

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
