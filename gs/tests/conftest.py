import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture(autouse=True)
def _reset_bf_capable():
    """build_app installs a process-global schema.set_bf_capable hook with no
    teardown; reset it around every test so a leaked probe can't shell out to
    `wfb-nics` or spuriously reject a later validate_effective."""
    from fpvdgs import schema
    schema.set_bf_capable(None)
    yield
    schema.set_bf_capable(None)


@pytest.fixture
def free_port():
    return _free_port()


@pytest.fixture
def fake_drone():
    """A stub drone fpvd. .calls records (method, path, body). .fail toggles 500s."""
    state = {"calls": [], "fail": False, "reject": None, "config": {"link": {"channel": 132}}}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _body(self):
            n = int(self.headers.get("content-length", 0))
            return self.rfile.read(n) if n else b""

        def _send(self, code, obj):
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            state["calls"].append(("GET", self.path, b""))
            if self.path == "/healthz":
                self._send(200, {"ok": True})
            elif self.path == "/status":
                self._send(200, {"link": {"channel": state["config"]["link"]["channel"]}})
            elif self.path == "/config":
                self._send(200, state["config"])
            else:
                self._send(404, {"error": "nf"})

        def do_PATCH(self):
            if state.get("reject") is not None:
                code, body = state["reject"]
                self._send(code, body)
                return
            body = self._body()
            state["calls"].append(("PATCH", self.path, body))
            if state["fail"]:
                self._send(500, {"error": "boom"})
                return
            patch = json.loads(body or b"{}")
            state["config"].setdefault("link", {}).update(patch.get("link", {}))
            self._send(200, state["config"])

        def do_POST(self):
            state["calls"].append(("POST", self.path, self._body()))
            self._send(500 if state["fail"] else 200, {"applied": not state["fail"]})

    port = _free_port()
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    state["endpoint"] = f"http://127.0.0.1:{port}"
    state["host"] = "127.0.0.1"
    state["port"] = port
    yield state
    srv.shutdown()
