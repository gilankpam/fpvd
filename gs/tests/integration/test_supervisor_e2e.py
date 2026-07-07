import asyncio
import json
import socket
import threading
import time
import urllib.request

import pytest

from fpvdgs import supervisor
from fpvdgs.wfb import engine as engine_mod


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


class _StubStatsSource:
    """A `client_factory()`-compatible stand-in that just idles until stopped
    -- these HTTP-wiring tests don't exercise the stats/dynlink data path."""

    def __init__(self, *a, **k):
        self._stop = None

    async def run(self):
        self._stop = asyncio.Event()
        await self._stop.wait()

    def stop(self):
        if self._stop is not None:
            self._stop.set()


class _FakeWfbEngine:
    """Stand-in for the real `WfbEngine` so these HTTP/wiring tests don't
    need real wfb_rx/wfb_tx binaries or a real wireless nic -- only the
    App/Api plumbing around the runner is under test here."""

    def __init__(self, config_provider=None, wlans_resolver=None, stats_port=None, reap_fn=None):
        self._running = False
        self.restarts = 0
        self.probe_feed = None  # 2026-07-06 Part B: _probe_status() reads runner.probe_feed

    def start(self) -> bool:
        self._running = True
        return True

    def restart(self, config=None) -> bool:
        self.restarts += 1
        self._running = True
        return True

    def stop(self) -> None:
        self._running = False

    def shutdown(self) -> None:
        self._running = False

    def state(self) -> dict:
        return {
            "running": self._running,
            "pid": None,
            "restarts": self.restarts,
            "autoRestarts": 0,
            "lastExit": None,
            "fault": False,
            "nodes": {},
        }

    def client_factory(self):
        return _StubStatsSource


def _req(base, method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method=method)
    if data:
        req.add_header("content-type", "application/json")
    with urllib.request.urlopen(req, timeout=3) as r:
        return r.status, json.loads(r.read() or b"{}")


@pytest.fixture
def daemon(tmp_path, fake_drone, monkeypatch):
    monkeypatch.setattr(engine_mod, "WfbEngine", _FakeWfbEngine)
    config_json = tmp_path / "config.json"
    api_port = _free_port()
    config_json.write_text(
        json.dumps(
            {
                "link": {
                    "channel": 132,
                    "width": 40,
                    "region": "US",
                    "linkId": 7669206,
                    "wlans": ["wlan0"],
                },
                "wfb": {"profile": "gs"},
                "drone": {"host": fake_drone["host"], "apiPort": fake_drone["port"]},
                "pixelpilot": {"enabled": False},
            }
        )
    )
    app = supervisor.build_app(
        str(config_json),
        host="127.0.0.1",
        port=api_port,
        ready_timeout=5.0,
    )
    app.start()
    t = threading.Thread(target=app.serve_forever, daemon=True)
    t.start()
    time.sleep(0.3)
    base = f"http://127.0.0.1:{api_port}"
    yield base, fake_drone
    app.shutdown()


def test_healthz_and_status(daemon):
    base, _ = daemon
    assert _req(base, "GET", "/healthz")[0] == 200
    code, st = _req(base, "GET", "/gs/status")
    assert code == 200
    assert st["runner"]["running"] is True


def test_gs_apply_link_change_applies_without_drone_push(daemon):
    # A link change goes through /gs/config + /gs/apply and is a GS-local effect
    # (the native engine retunes/bounces from its own config, no rendered cfg
    # file involved). The drone push is now the client's job, not the GS's --
    # so no drone /config call here.
    base, fake_drone = daemon
    _req(base, "PATCH", "/gs/config", {"link": {"channel": 100}})
    code, _ = _req(base, "POST", "/gs/apply", {})
    assert code == 200
    assert not any(p == "/config" for (_m, p, _b) in fake_drone["calls"])


def test_air_proxy_roundtrip(daemon):
    base, fake_drone = daemon
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

        def start(self):
            self.started = True

        def stop(self):
            self.started = False

        def set_config(self, snap):
            pass

        def status(self):
            return {
                "running": self.started,
                "statsConnected": False,
                "decision": None,
                "lastEmitMs": None,
                "emitSeq": 0,
                "reason": "",
            }

    monkeypatch.setattr(supervisor, "DynamicLinkController", _StubController)
    # Avoid spawning the real runner / radio probing.
    monkeypatch.setattr(supervisor, "resolve_wlans", lambda cfg: ["wlan0"])

    config_json = tmp_path / "config.json"
    config_json.write_text(
        json.dumps(
            {
                "link": {"channel": 132, "width": 40, "region": "US"},
                "wfb": {"profile": "gs", "raw": {}},
                "drone": {"host": "127.0.0.1", "apiPort": 1},
                "dynamicLink": {
                    "enabled": False,
                    "maxMcs": 5,
                    "dronePort": 9999,
                },
            }
        )
    )
    app = supervisor.build_app(str(config_json), "127.0.0.1", 0)
    code, body = app.api.handle("GET", "/gs/status", {}, b"")
    assert code == 200
    assert "dynamicLink" in body
    assert "pixelpilot" in body
    assert body["dynamicLink"]["running"] is False
    # /gs/status is GS-local — it must NOT reach out to the drone (drone endpoint
    # here is unreachable :1). No drone-derived fields are reported.
    assert "droneReachable" not in body["link"]
    assert "drone" not in body["dynamicLink"]


def test_status_probe_disabled_by_default_with_dynamiclink_enabled(tmp_path, monkeypatch):
    """dynamicLink.probe.enabled defaults false, so /gs/status.probe stays
    disabled even with dynamicLink enabled — the native engine only renders a
    probe_rx child when both dynamicLink AND the probe knob are on
    (`_probe_render_view`); no probe leg is spawned here."""
    import json

    from fpvdgs import supervisor
    from fpvdgs.wfb import engine as engine_mod

    monkeypatch.setattr(supervisor, "resolve_wlans", lambda cfg: ["wlan0"])
    monkeypatch.setattr(engine_mod, "WfbEngine", _FakeWfbEngine)

    class _StubDl:
        def __init__(self, *a, **k):
            self.started = False

        def start(self):
            self.started = True

        def stop(self):
            self.started = False

        def set_config(self, snap):
            pass

        def status(self):
            return {"running": self.started}

    monkeypatch.setattr(supervisor, "DynamicLinkController", _StubDl)

    config_json = tmp_path / "config.json"
    config_json.write_text(
        json.dumps(
            {
                "link": {"channel": 132, "width": 20, "region": "US", "linkId": 7669206},
                "wfb": {"profile": "gs", "raw": {}},
                "drone": {"host": "127.0.0.1", "apiPort": 1},
                "pixelpilot": {"enabled": False},
                "dynamicLink": {
                    "enabled": True,
                    "maxMcs": 5,
                    "dronePort": 9999,
                },
            }
        )
    )
    api_port = _free_port()
    app = supervisor.build_app(
        str(config_json),
        "127.0.0.1",
        api_port,
    )
    app.start()
    t = threading.Thread(target=app.serve_forever, daemon=True)
    t.start()
    time.sleep(0.3)
    try:
        code, body = app.api.handle("GET", "/gs/status", {}, b"")
        assert code == 200
        assert body["probe"] == {"enabled": False, "running": False}
    finally:
        app.shutdown()
