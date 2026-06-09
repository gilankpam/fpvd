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
                         "wfb": {"profile": "gs"}, "droneLink": {"endpoint": "http://x"}},
                        overlay_path=None)
    drone = FakeDrone()
    runner = FakeRunner()
    link = LinkCoordinator(
        store,
        lambda cfg: render_mod.write_cfg(cfg_out, render_mod.render_cfg(cfg)),
        runner, drone, validate=schema.validate_effective)
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


def test_link_apply_rejects_bad_width():
    api, store, _, _ = _api()
    api.handle("PATCH", "/link", {}, json.dumps({"link": {"width": 80}}).encode())
    code, obj = api.handle("POST", "/link/apply", {},
                           json.dumps({"applyTo": "gs"}).encode())
    assert code == 400


def test_apply_rolls_back_on_runner_failure(tmp_path):
    from fpvdgs.api import Api as _Api

    class FailingRunner:
        def restart(self):
            return False
        def state(self):
            return {"running": False, "pid": None, "restarts": 0,
                    "lastExit": 1, "fault": False}

    cfg_out = str(tmp_path / "wifibroadcast.cfg")
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"},
                         "wfb": {"profile": "gs"}, "droneLink": {"endpoint": "http://x"}},
                        overlay_path=None)
    drone = FakeDrone()
    link = LinkCoordinator(
        store, lambda cfg: render_mod.write_cfg(cfg_out, render_mod.render_cfg(cfg)),
        FailingRunner(), drone, validate=schema.validate_effective)
    api = _Api(store=store, schema=schema, render_mod=render_mod, runner=FailingRunner(),
               drone=drone, link=link, status_fn=lambda: {}, cfg_out=cfg_out)
    # boot render of the good cfg
    render_mod.write_cfg(cfg_out, render_mod.render_cfg(store.effective()))
    api.handle("PATCH", "/config", {},
               json.dumps({"wfb": {"mavlink": {"peer": "connect://127.0.0.1:9999"}}}).encode())
    code, obj = api.handle("POST", "/apply", {}, b"")
    assert code == 500
    # cfg rolled back (no 9999) and overlay not committed
    assert "9999" not in open(cfg_out).read()
    assert store.effective().get("wfb", {}).get("mavlink", {}).get("peer") != "connect://127.0.0.1:9999"


# --- adaptiveLink apply routing ---
class _FakeController:
    def __init__(self):
        self.calls = []
    def start(self):
        self.calls.append(("start", None))
    def stop(self):
        self.calls.append(("stop", None))
    def set_config(self, snap):
        self.calls.append(("set_config", snap))


class _FakeRunner:
    def __init__(self):
        self.restarts = 0
    def restart(self):
        self.restarts += 1
        return True


def _api_with_dynlink(tmp_path):
    import json
    from fpvdgs import render, schema
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient

    defaults = {"link": {"channel": 132, "width": 40, "region": "US"},
                "wfb": {"profile": "gs", "raw": {}},
                "droneLink": {"endpoint": "http://10.5.0.10:8080"},
                "adaptiveLink": {"enabled": False,
                                 "controller": {"maxMcs": 5,
                                                "radioProfile": "m8812eu2",
                                                "droneAddr": None,
                                                "dronePort": 9999, "tuning": {}}}}
    store = ConfigStore(defaults)
    ctrl = _FakeController()
    runner = _FakeRunner()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render, runner=runner,
              drone=DroneClient("http://127.0.0.1:1"), link=None,
              status_fn=lambda: {}, cfg_out=cfg_out, dynlink=ctrl)
    return api, store, ctrl, runner


def test_enable_dynamiclink_starts_controller_without_bouncing_runner(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"adaptiveLink": {"enabled": True}})
    code, body = api.handle("POST", "/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert ("start", None) in ctrl.calls
    assert runner.restarts == 0          # adaptive-link-only change: no bounce
    assert store.effective()["adaptiveLink"]["enabled"] is True


def test_disable_dynamiclink_stops_controller(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"adaptiveLink": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    store.patch({"adaptiveLink": {"enabled": False}})
    api.handle("POST", "/apply", {}, b"")
    assert ("stop", None) in ctrl.calls
    assert runner.restarts == 0


def test_tuning_change_while_enabled_calls_set_config(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"adaptiveLink": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    store.patch({"adaptiveLink": {"controller": {"maxMcs": 3}}})
    api.handle("POST", "/apply", {}, b"")
    assert any(c[0] == "set_config" for c in ctrl.calls)
    assert runner.restarts == 0


def test_wfb_change_bounces_runner_and_leaves_controller_alone(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1          # non-adaptiveLink change: bounce
    assert ctrl.calls == []              # controller untouched (stayed disabled)


# --- pixelpilot apply routing ---
class _FakePP:
    def __init__(self):
        self.calls = []
    def set_argv(self, argv):
        self.calls.append(("set_argv", argv))
    def set_env(self, env):
        self.calls.append(("set_env", env))
    def start(self):
        self.calls.append(("start", None))
    def stop(self):
        self.calls.append(("stop", None))
    def restart(self):
        self.calls.append(("restart", None))


def _api_with_pp(tmp_path):
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient
    defaults = {"link": {"channel": 132, "width": 40, "region": "US"},
                "wfb": {"profile": "gs", "raw": {}},
                "droneLink": {"endpoint": "http://10.5.0.10:8080"},
                "pixelpilot": {"enabled": True, "screenMode": "1920x1080@60",
                               "videoScale": 1.0, "dvrFramerate": 60,
                               "extraArgs": []}}
    store = ConfigStore(defaults)
    runner = _FakeRunner()      # defined earlier in this file
    pp = _FakePP()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=DroneClient("http://127.0.0.1:1"), link=None,
              status_fn=lambda: {}, cfg_out=cfg_out, pixelpilot=pp)
    return api, store, pp, runner


def test_pixelpilot_change_restarts_pp_not_wfb(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"screenMode": "1280x720@60"}})
    code, body = api.handle("POST", "/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert any(c[0] == "set_argv" for c in pp.calls)
    assert ("restart", None) in pp.calls
    assert runner.restarts == 0      # PixelPilot-only change: radio untouched
    assert store.effective()["pixelpilot"]["screenMode"] == "1280x720@60"


def test_wfb_change_does_not_touch_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1
    assert pp.calls == []            # pixelpilot untouched


def test_pixelpilot_disable_then_enable(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"enabled": False}})
    api.handle("POST", "/apply", {}, b"")
    assert ("stop", None) in pp.calls
    pp.calls.clear()
    store.patch({"pixelpilot": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    assert ("start", None) in pp.calls     # off->on uses start(), not restart()
    assert ("restart", None) not in pp.calls


def test_combined_wfb_and_pixelpilot_change_bounces_both(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}},
                 "pixelpilot": {"screenMode": "1280x720@60"}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1               # wfb bounced
    assert ("restart", None) in pp.calls      # pixelpilot bounced too


def test_patch_config_accepts_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"pixelpilot": {"videoScale": 1.5}}).encode())
    assert code == 200
    assert store.pending()["pixelpilot"]["videoScale"] == 1.5


# --- probe lifecycle rides the adaptiveLink transition (no probe config) ---
class _FakeProbe:
    def __init__(self): self.started = False; self.cfgs = []
    def start(self): self.started = True
    def stop(self): self.started = False
    def set_config(self, snap): self.cfgs.append(snap)
    def status(self): return {"running": self.started, "streams": 1, "mcs": {}}


def _api_with_dl_and_probe(tmp_path):
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient
    # link.wlans is an explicit list so make_probe_snapshot's resolve_wlans
    # returns it directly (no wfb-nics / hardware probe).
    defaults = {"link": {"channel": 132, "width": 40, "region": "US",
                         "linkId": 7669206, "wlans": ["wlan0"]},
                "wfb": {"profile": "gs", "raw": {}},
                "droneLink": {"endpoint": "http://10.5.0.10:8080"},
                "adaptiveLink": {"enabled": False,
                                 "controller": {"maxMcs": 5,
                                                "radioProfile": "m8812eu2",
                                                "droneAddr": None,
                                                "dronePort": 9999, "tuning": {}}}}
    store = ConfigStore(defaults)
    ctrl = _FakeController()     # existing fake dynlink controller in this file
    probe = _FakeProbe()
    runner = _FakeRunner()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=DroneClient("http://127.0.0.1:1"), link=None,
              status_fn=lambda: {}, cfg_out=cfg_out, dynlink=ctrl, probe=probe)
    return api, store, ctrl, probe, runner


def test_enable_dynamiclink_starts_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    store.patch({"adaptiveLink": {"enabled": True}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert ("start", None) in ctrl.calls and probe.started is True
    assert runner.restarts == 0           # no video bounce


def test_disable_dynamiclink_stops_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    store.patch({"adaptiveLink": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    store.patch({"adaptiveLink": {"enabled": False}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert probe.started is False and runner.restarts == 0
