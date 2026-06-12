import json
import os
import tempfile

from fpvdgs import schema, render as render_mod
from fpvdgs.config import ConfigStore
from fpvdgs.api import Api


class FakeRunner:
    def __init__(self): self.restarts = 0
    def restart(self): self.restarts += 1; return True
    def state(self): return {"running": True, "pid": 1, "restarts": self.restarts,
                             "lastExit": None, "fault": False}


class FakeDrone:
    def __init__(self): self.calls = []
    def healthz(self): return True
    def _request(self, method, path, body=None):
        self.calls.append((method, path, body))
        return 200, json.dumps({"proxied": path}).encode()


def _api(retune_ok=True):
    cfg_out = os.path.join(tempfile.mkdtemp(), "wifibroadcast.cfg")
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"},
                         "wfb": {"profile": "gs"}, "drone": {"endpoint": "http://x"}},
                        overlay_path=None)
    drone = FakeDrone()
    runner = FakeRunner()
    retunes = []
    def retune(link): retunes.append(link); return retune_ok
    ticks = []
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone, status_fn=lambda: {"ok": True}, cfg_out=cfg_out,
              retune=retune, wlans_resolver=lambda cfg: ["wlan0"],
              armer_tick=lambda: ticks.append(1))
    return api, store, drone, runner, retunes, ticks, cfg_out


def test_gs_routes_answer_under_gs_prefix():
    api, *_ = _api()
    assert api.handle("GET", "/gs/config", {}, b"")[0] == 200
    assert api.handle("GET", "/gs/status", {}, b"")[0] == 200
    assert api.handle("GET", "/gs/defaults", {}, b"")[0] == 200


def test_healthz_stays_at_root():
    api, *_ = _api()
    assert api.handle("GET", "/healthz", {}, b"")[0] == 200


def test_link_endpoints_gone():
    api, *_ = _api()
    assert api.handle("GET", "/link", {}, b"")[0] == 404
    assert api.handle("POST", "/link/apply", {}, b"")[0] == 404


def test_air_still_proxies():
    api, _, drone, *_ = _api()
    code, obj = api.handle("GET", "/air/config", {}, b"")
    assert code == 200 and ("GET", "/config", None) in drone.calls


def test_link_change_retunes_live_no_bounce():
    api, store, _, runner, retunes, ticks, _ = _api()
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and obj["applied"] is True
    assert retunes and retunes[-1]["channel"] == 100   # retuned
    assert runner.restarts == 0                          # no bounce


def test_link_change_bounces_on_wlans():
    api, store, _, runner, retunes, ticks, _ = _api()
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"link": {"wlans": ["wlan1"]}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1 and retunes == []


def test_failed_retune_falls_back_to_bounce():
    api, store, _, runner, retunes, ticks, _ = _api(retune_ok=False)
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1   # retune failed -> bounced


def test_apply_fires_armer_tick():
    api, store, drone, runner, retunes, ticks, cfg_out = _api()
    api.handle("POST", "/gs/apply", {}, b"")
    assert ticks == [1]


# --- dynamicLink apply routing ---
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
                "drone": {"endpoint": "http://10.5.0.10:8080"},
                "dynamicLink": {"enabled": False, "maxMcs": 5,
                                "txpower": {"min": 18, "max": 28},
                                "radioProfile": "m8812eu2", "droneAddr": None,
                                "dronePort": 9999, "tuning": {}}}
    store = ConfigStore(defaults)
    ctrl = _FakeController()
    runner = _FakeRunner()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render, runner=runner,
              drone=DroneClient("http://127.0.0.1:1"),
              status_fn=lambda: {}, cfg_out=cfg_out, dynlink=ctrl)
    return api, store, ctrl, runner


def test_enable_dynamiclink_starts_controller_without_bouncing_runner(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    code, body = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert ("start", None) in ctrl.calls
    assert runner.restarts == 0          # dynamic-link-only change: no bounce
    assert store.effective()["dynamicLink"]["enabled"] is True


def test_disable_dynamiclink_stops_controller(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/gs/apply", {}, b"")
    store.patch({"dynamicLink": {"enabled": False}})
    api.handle("POST", "/gs/apply", {}, b"")
    assert ("stop", None) in ctrl.calls
    assert runner.restarts == 0


def test_tuning_change_while_enabled_calls_set_config(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/gs/apply", {}, b"")
    store.patch({"dynamicLink": {"maxMcs": 3}})
    api.handle("POST", "/gs/apply", {}, b"")
    assert any(c[0] == "set_config" for c in ctrl.calls)
    assert runner.restarts == 0


def test_wfb_change_bounces_runner_and_leaves_controller_alone(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}}})
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1          # non-dynamicLink change: bounce
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
                "drone": {"endpoint": "http://10.5.0.10:8080"},
                "pixelpilot": {"enabled": True, "screenMode": "1920x1080@60",
                               "videoScale": 1.0, "dvrFramerate": 60,
                               "extraArgs": []}}
    store = ConfigStore(defaults)
    runner = _FakeRunner()      # defined earlier in this file
    pp = _FakePP()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=DroneClient("http://127.0.0.1:1"),
              status_fn=lambda: {}, cfg_out=cfg_out, pixelpilot=pp)
    return api, store, pp, runner


def test_pixelpilot_change_restarts_pp_not_wfb(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"screenMode": "1280x720@60"}})
    code, body = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert any(c[0] == "set_argv" for c in pp.calls)
    assert ("restart", None) in pp.calls
    assert runner.restarts == 0      # PixelPilot-only change: radio untouched
    assert store.effective()["pixelpilot"]["screenMode"] == "1280x720@60"


def test_wfb_change_does_not_touch_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}}})
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1
    assert pp.calls == []            # pixelpilot untouched


def test_pixelpilot_disable_then_enable(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"enabled": False}})
    api.handle("POST", "/gs/apply", {}, b"")
    assert ("stop", None) in pp.calls
    pp.calls.clear()
    store.patch({"pixelpilot": {"enabled": True}})
    api.handle("POST", "/gs/apply", {}, b"")
    assert ("start", None) in pp.calls     # off->on uses start(), not restart()
    assert ("restart", None) not in pp.calls


def test_combined_wfb_and_pixelpilot_change_bounces_both(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}},
                 "pixelpilot": {"screenMode": "1280x720@60"}})
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1               # wfb bounced
    assert ("restart", None) in pp.calls      # pixelpilot bounced too


def test_patch_config_accepts_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    code, _ = api.handle("PATCH", "/gs/config", {},
                         json.dumps({"pixelpilot": {"videoScale": 1.5}}).encode())
    assert code == 200
    assert store.pending()["pixelpilot"]["videoScale"] == 1.5


# --- probe lifecycle rides the dynamicLink transition (no probe config) ---
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
                "drone": {"endpoint": "http://10.5.0.10:8080"},
                "dynamicLink": {"enabled": False, "maxMcs": 5,
                                "txpower": {"min": 18, "max": 28},
                                "radioProfile": "m8812eu2", "droneAddr": None,
                                "dronePort": 9999, "tuning": {}}}
    store = ConfigStore(defaults)
    ctrl = _FakeController()     # existing fake dynlink controller in this file
    probe = _FakeProbe()
    runner = _FakeRunner()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=DroneClient("http://127.0.0.1:1"),
              status_fn=lambda: {}, cfg_out=cfg_out, dynlink=ctrl, probe=probe)
    return api, store, ctrl, probe, runner


def test_enable_dynamiclink_starts_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert ("start", None) in ctrl.calls and probe.started is True
    assert runner.restarts == 0           # no video bounce


def test_disable_dynamiclink_stops_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/gs/apply", {}, b"")
    store.patch({"dynamicLink": {"enabled": False}})
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert probe.started is False and runner.restarts == 0


class FakeRelay:
    def __init__(self): self.events = []; self._running = False
    def start(self): self._running = True; self.events.append("start")
    def stop(self): self._running = False; self.events.append("stop")
    def status(self): return {"running": self._running, "listen": None}


def test_idr_forward_apply_starts_and_stops():
    cfg_out = os.path.join(tempfile.mkdtemp(), "wifibroadcast.cfg")
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"},
                         "wfb": {"profile": "gs"}, "drone": {"endpoint": "http://x"},
                         "idrForward": {"enabled": False, "port": 11223}},
                        overlay_path=None)
    relay = FakeRelay()
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=FakeRunner(),
              drone=FakeDrone(), status_fn=lambda: {}, cfg_out=cfg_out,
              retune=lambda l: True, wlans_resolver=lambda c: ["wlan0"],
              armer_tick=lambda: None, idr_relay=relay)
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"idrForward": {"enabled": True}}).encode())
    api.handle("POST", "/gs/apply", {}, b"")
    assert "start" in relay.events
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"idrForward": {"enabled": False}}).encode())
    api.handle("POST", "/gs/apply", {}, b"")
    assert relay.events[-1] == "stop"
