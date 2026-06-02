import json
import os

from fpvdgs import schema, render as render_mod
from fpvdgs.config import ConfigStore
from fpvdgs.api import Api
from fpvdgs.link import LinkCoordinator


class FakeRunner:
    def restart(self):
        return True

    def state(self):
        return {"running": True, "pid": 1, "restarts": 0, "lastExit": None, "fault": False}


class FakeDrone:
    def __init__(self):
        self.calls = []

    def healthz(self):
        return True

    def patch_config(self, d):
        self.calls.append(("PATCH", d))
        return {}

    def apply(self):
        self.calls.append(("POST", "/apply", None))
        return {}

    # opaque proxy hook (Api._proxy calls drone._request)
    def _request(self, method, path, body=None):
        self.calls.append((method, path, body))
        return 200, json.dumps({"proxied": path}).encode()


def _api():
    import tempfile
    cfg_out = os.path.join(tempfile.mkdtemp(), "wifibroadcast.cfg")
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"},
                         "wfb": {"profile": "gs"}, "drone": {"endpoint": "http://x"}},
                        overlay_path=None)
    drone = FakeDrone()
    runner = FakeRunner()
    link = LinkCoordinator(
        store,
        lambda cfg: render_mod.write_cfg(cfg_out, render_mod.render_cfg(cfg)),
        runner, drone)
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone, link=link, status_fn=lambda: {"ok": True}, cfg_out=cfg_out)
    return api, store, drone, cfg_out


def test_healthz():
    api, _, _, _ = _api()
    code, _ = api.handle("GET", "/healthz", {}, b"")
    assert code == 200


def test_get_config_returns_effective():
    api, _, _, _ = _api()
    code, obj = api.handle("GET", "/config", {}, b"")
    assert code == 200
    assert obj["link"]["channel"] == 132


def test_patch_config_rejects_link():
    api, _, _, _ = _api()
    code, obj = api.handle("PATCH", "/config", {},
                           json.dumps({"link": {"channel": 100}}).encode())
    assert code == 400
    assert "link" in obj["error"]


def test_patch_config_then_apply_renders_cfg():
    api, store, _, cfg_out = _api()
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"wfb": {"profile": "gs2"}}).encode())
    assert code == 200
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert store.effective()["wfb"]["profile"] == "gs2"
    assert os.path.exists(cfg_out)
    assert "wifi_channel" in open(cfg_out).read()


def test_apply_refuses_pending_link_change():
    api, store, _, _ = _api()
    api.handle("PATCH", "/link", {}, json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/apply", {}, b"")
    assert code == 409


def test_link_apply_both_renders_and_pushes():
    api, store, drone, cfg_out = _api()
    api.handle("PATCH", "/link", {}, json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/link/apply", {},
                           json.dumps({"applyTo": "both"}).encode())
    assert code == 200
    assert obj["gsApplied"] is True
    assert obj["droneApplied"] is True
    assert store.effective()["link"]["channel"] == 100
    assert "wifi_channel = 100" in open(cfg_out).read()


def test_air_config_is_proxied_opaquely():
    api, _, drone, _ = _api()
    code, raw = api.handle("PATCH", "/air/config", {}, b'{"video":{"bitrate":9000}}')
    assert code == 200
    assert any(c[0] == "PATCH" and c[1] == "/config" for c in drone.calls)
