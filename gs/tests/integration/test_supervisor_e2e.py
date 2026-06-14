import json
import socket
import threading
import time
import urllib.request

import pytest

from fpvdgs import supervisor


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _req(base, method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method=method)
    if data:
        req.add_header("content-type", "application/json")
    with urllib.request.urlopen(req, timeout=3) as r:
        return r.status, json.loads(r.read() or b"{}")


@pytest.fixture
def daemon(tmp_path, fake_drone):
    config_json = tmp_path / "config.json"
    cfg_out = tmp_path / "wifibroadcast.cfg"
    ready_port = _free_port()
    api_port = _free_port()
    config_json.write_text(json.dumps({
        "link": {"channel": 132, "width": 40, "region": "US",
                 "linkId": 7669206, "wlans": ["wlan0"]},
        "wfb": {"profile": "gs"},
        "drone": {"endpoint": fake_drone["endpoint"]},
        "pixelpilot": {"enabled": False},
    }))
    fake_runner = ["python3", "-c",
                   ("import socket,time;s=socket.socket();"
                    "s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);"
                    f"s.bind(('127.0.0.1',{ready_port}));s.listen(1);time.sleep(30)")]
    app = supervisor.build_app(
        str(config_json),
        cfg_out=str(cfg_out), host="127.0.0.1", port=api_port,
        runner_cmd=fake_runner, ready_port=ready_port, ready_timeout=5.0)
    app.start()
    t = threading.Thread(target=app.serve_forever, daemon=True)
    t.start()
    time.sleep(0.3)
    base = f"http://127.0.0.1:{api_port}"
    yield base, cfg_out, fake_drone
    app.shutdown()


def test_healthz_and_status(daemon):
    base, _, _ = daemon
    assert _req(base, "GET", "/healthz")[0] == 200
    code, st = _req(base, "GET", "/gs/status")
    assert code == 200
    assert st["runner"]["running"] is True


def test_cfg_rendered_on_boot(daemon):
    _, cfg_out, _ = daemon
    text = cfg_out.read_text()
    assert "wifi_channel = 132" in text


def test_gs_apply_link_change_rerenders_cfg(daemon):
    # A link change goes through /gs/config + /gs/apply and is a GS-local effect:
    # the cfg is re-rendered (and the runner retunes/bounces). The drone push is
    # now the client's job, not the GS's — so no drone /config call here.
    base, cfg_out, fake_drone = daemon
    _req(base, "PATCH", "/gs/config", {"link": {"channel": 100}})
    code, _ = _req(base, "POST", "/gs/apply", {})
    assert code == 200
    assert "wifi_channel = 100" in cfg_out.read_text()
    assert not any(p == "/config" for (_m, p, _b) in fake_drone["calls"])


def test_air_proxy_roundtrip(daemon):
    base, _, fake_drone = daemon
    code, _ = _req(base, "PATCH", "/air/config", {"video": {"bitrate": 9000}})
    assert code == 200
    assert any(p == "/config" for (_m, p, _b) in fake_drone["calls"])


def test_dynamiclink_assembled_into_status_and_controller_built(tmp_path, monkeypatch):
    """build_app constructs a controller; status_fn merges its state.
    Uses a stub controller via monkeypatch so no sockets/threads are needed."""
    import json
    from fpvdgs import supervisor

    class _StubController:
        def __init__(self, *a, **k):
            self.started = False
        def start(self): self.started = True
        def stop(self): self.started = False
        def set_config(self, snap): pass
        def status(self):
            return {"running": self.started, "statsConnected": False,
                    "decision": None, "lastEmitMs": None, "emitSeq": 0,
                    "reason": ""}

    monkeypatch.setattr(supervisor, "DynamicLinkController", _StubController)
    # Avoid spawning the real runner / radio probing. make_probe_snapshot
    # resolves wlans through its own module, so patch that too.
    monkeypatch.setattr(supervisor, "resolve_wlans", lambda cfg: ["wlan0"])
    from fpvdgs.probe import config_build as _probe_cb
    monkeypatch.setattr(_probe_cb, "resolve_wlans", lambda cfg: ["wlan0"])

    config_json = tmp_path / "config.json"
    config_json.write_text(json.dumps({
        "link": {"channel": 132, "width": 40, "region": "US"},
        "wfb": {"profile": "gs", "raw": {}},
        "drone": {"endpoint": "http://127.0.0.1:1"},
        "dynamicLink": {"enabled": False, "maxMcs": 5,
                        "radioProfile": "m8812eu2", "droneAddr": None,
                        "dronePort": 9999}}))
    cfg_out = tmp_path / "wfb.cfg"

    app = supervisor.build_app(str(config_json),
                               str(cfg_out), "127.0.0.1", 0,
                               runner_cmd=["true"])
    code, body = app.api.handle("GET", "/gs/status", {}, b"")
    assert code == 200
    assert "dynamicLink" in body
    assert "pixelpilot" in body
    assert body["dynamicLink"]["running"] is False
    # /gs/status is GS-local — it must NOT reach out to the drone (drone endpoint
    # here is unreachable :1). No drone-derived fields are reported.
    assert "droneReachable" not in body["link"]
    assert "drone" not in body["dynamicLink"]


def test_status_probe_tied_to_dynamiclink(tmp_path, monkeypatch):
    """The probe lifecycle + status follow dynamicLink.enabled (no probe config).
    A fake probe_spawn guarantees no real wfb_rx runs; a stub DynamicLinkController
    avoids sockets/threads. One probe wfb_rx is spawned when dynamicLink is on."""
    import json
    from fpvdgs import supervisor
    monkeypatch.setattr(supervisor, "resolve_wlans", lambda cfg: ["wlan0"])
    monkeypatch.setattr("fpvdgs.probe.config_build.resolve_wlans",
                        lambda cfg: ["wlan0"])

    class _StubDl:
        def __init__(self, *a, **k): self.started = False
        def start(self): self.started = True
        def stop(self): self.started = False
        def set_config(self, snap): pass
        def status(self): return {"running": self.started}
    monkeypatch.setattr(supervisor, "DynamicLinkController", _StubDl)

    spawned = []
    def fake_spawn(cmd):
        spawned.append(cmd)
        class _P:
            stdout = type("S", (), {"readline": staticmethod(
                lambda: __import__("asyncio").sleep(3600))})()
            def kill(self): pass
            async def wait(self): return 0
        return _P()

    config_json = tmp_path / "config.json"
    config_json.write_text(json.dumps({
        "link": {"channel": 132, "width": 40, "region": "US", "linkId": 7669206},
        "wfb": {"profile": "gs", "raw": {}},
        "drone": {"endpoint": "http://127.0.0.1:1"},
        "pixelpilot": {"enabled": False},
        "dynamicLink": {"enabled": True, "maxMcs": 5,
                        "radioProfile": "m8812eu2", "dronePort": 9999}}))
    cfg_out = tmp_path / "wfb.cfg"
    api_port = _free_port()
    app = supervisor.build_app(str(config_json),
                               str(cfg_out), "127.0.0.1", api_port,
                               runner_cmd=["true"], probe_spawn=fake_spawn)
    app.start()
    t = threading.Thread(target=app.serve_forever, daemon=True)
    t.start()
    time.sleep(0.3)
    try:
        code, body = app.api.handle("GET", "/gs/status", {}, b"")
        assert code == 200
        assert body["probe"]["enabled"] is True
        assert len(spawned) == 1            # one wfb_rx, started with dynamicLink
    finally:
        app.shutdown()
