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
        self.patch_raises = None   # set to an exception to make patch_config raise
        self.apply_raises = None   # set to an exception to make apply() raise
        self._reachable = True     # toggle for healthz()

    def healthz(self):
        return self._reachable

    def patch_config(self, d):
        self.calls.append(("PATCH", d))
        if self.patch_raises is not None:
            raise self.patch_raises
        return {}

    def apply(self):
        self.calls.append(("POST", "/apply", None))
        if self.apply_raises is not None:
            raise self.apply_raises
        return {}

    def get_status(self):
        return {}

    # opaque proxy hook (Api._proxy calls drone._request)
    def _request(self, method, path, body=None):
        self.calls.append((method, path, body))
        return 200, json.dumps({"proxied": path}).encode()


class _FakeDroneCache:
    """Returns a fixed (drone_cfg, meta) pair for GET /config rendering.
    Defaults to a never-seen drone (None, droneStale) so callers that don't
    pass drone state still exercise the GS side of the unified tree."""
    def __init__(self, cfg=None, meta=None):
        self._cfg = cfg
        self._meta = meta or {"droneReachable": False, "droneStale": True,
                              "droneLastSeen": None}

    def read(self):
        return self._cfg, self._meta


_DRONE_CFG = {
    "link": {"channel": 132, "width": 40, "linkId": 7669206,
             "mcs": 3, "txpower": 25, "fec": {"k": 8, "n": 12}},
    "dynamicLink": {"enabled": False, "healthTimeoutMs": 10000,
                    "failsafe": {"mcs": 1}},
    "video": {"codec": "h265", "fps": 60},
    "telemetry": {"router": "msposd"},
}
_DRONE_META = {"droneReachable": True, "droneStale": False,
               "droneLastSeen": "2026-06-10T00:00:00Z"}


def _api(drone_cache=None):
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
              drone=drone, link=link, status_fn=lambda: {"ok": True}, cfg_out=cfg_out,
              drone_cache=drone_cache or _FakeDroneCache())
    return api, store, drone, cfg_out


def test_healthz():
    api, _, _, _ = _api()
    code, _ = api.handle("GET", "/healthz", {}, b"")
    assert code == 200


def test_get_config_returns_unified_tree():
    # GS link.region now lives under link.gs; shared keys (channel) stay flat.
    api, _, _, _ = _api(
        drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    code, body = api.handle("GET", "/config", {}, b"")
    assert code == 200
    assert body["_meta"]["droneReachable"] is True
    assert body["link"]["channel"] == 132
    assert body["link"]["gs"]["region"] == "US"
    assert body["link"]["drone"]["mcs"] == 3
    assert body["dynamicLink"]["controller"] == {}   # GS has no controller block here
    assert "applier" in body["dynamicLink"]
    assert body["dynamicLink"]["applier"]["healthTimeoutMs"] == 10000
    assert body["video"] == {"codec": "h265", "fps": 60}


def test_get_config_renders_gs_when_drone_never_seen():
    # default _FakeDroneCache: never-seen drone -> empty drone subtrees, GS renders.
    api, _, _, _ = _api()
    code, body = api.handle("GET", "/config", {}, b"")
    assert code == 200
    assert body["_meta"]["droneStale"] is True
    assert body["link"]["channel"] == 132
    assert body["link"]["gs"]["region"] == "US"
    assert body["link"]["drone"] == {}


def test_patch_config_routes_gs_section_not_drone():
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"pixelpilot": {"videoScale": 1.5}}).encode())
    assert code == 200
    assert store.pending()["pixelpilot"]["videoScale"] == 1.5
    assert drone.calls == []          # GS-only patch never touches the drone


def test_patch_config_routes_drone_section():
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"video": {"bitrate": 9000}}).encode())
    assert code == 200
    assert ("PATCH", {"video": {"bitrate": 9000}}) in drone.calls
    assert api._drone_dirty is True


def test_patch_config_link_gs_routes_to_pending():
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"link": {"gs": {"rxpower": 20}}}).encode())
    assert code == 200
    assert store.pending()["link"]["rxpower"] == 20
    assert drone.calls == []          # link.gs is GS-only


def test_patch_config_link_drone_proxies():
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"link": {"drone": {"mcs": 4}}}).encode())
    assert code == 200
    assert ("PATCH", {"link": {"mcs": 4}}) in drone.calls


def test_patch_config_shared_link_goes_to_gs_pending_only():
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"link": {"channel": 140}}).encode())
    assert code == 200
    assert store.pending()["link"]["channel"] == 140
    assert drone.calls == []          # shared keys pushed at apply, not now


def test_patch_config_rejects_meta():
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    code, obj = api.handle("PATCH", "/config", {},
                           json.dumps({"_meta": {"droneStale": False}}).encode())
    assert code == 400
    assert drone.calls == []


def test_patch_config_gs_validation_failure_leaves_pending_clean():
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    before = store.pending()
    code, obj = api.handle("PATCH", "/config", {},
                           json.dumps({"link": {"width": 80}}).encode())
    assert code == 400
    assert store.pending() == before          # GS pending unchanged
    assert drone.calls == []                  # drone never touched


def test_patch_config_drone_reject_leaves_gs_pending_clean():
    from fpvdgs.drone_client import DroneRejected
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    drone.patch_raises = DroneRejected(400, {"message": "bad mcs", "field": "mcs"})
    before = store.pending()
    # a patch that touches BOTH a GS section and a drone section
    code, obj = api.handle("PATCH", "/config", {},
                           json.dumps({"pixelpilot": {"videoScale": 1.5},
                                       "link": {"drone": {"mcs": 99}}}).encode())
    assert code == 400
    assert obj["error"] == "drone_rejected"
    assert obj["message"] == "bad mcs"
    assert obj["details"]["field"] == "mcs"
    assert store.pending() == before          # GS pending NOT mutated on drone reject


def test_patch_config_drone_unreachable():
    from fpvdgs.drone_client import DroneUnreachable
    api, store, drone, _ = _api(drone_cache=_FakeDroneCache(_DRONE_CFG, _DRONE_META))
    drone.patch_raises = DroneUnreachable("no route")
    code, obj = api.handle("PATCH", "/config", {},
                           json.dumps({"video": {"bitrate": 9000}}).encode())
    assert code == 502
    assert obj["error"] == "drone_unreachable"


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


def test_apply_shared_link_change_reachable_pushes_drone():
    api, store, drone, cfg_out = _api()
    # boot render of last-good so the coordinator has a cfg to roll back to
    render_mod.write_cfg(cfg_out, render_mod.render_cfg(store.effective()))
    api.handle("PATCH", "/config", {}, json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    # coordinator ran: shared link change pushed to the (reachable) drone
    assert obj["sharedLink"]["gsApplied"] is True
    assert obj["sharedLink"]["droneApplied"] is True
    assert store.effective()["link"]["channel"] == 100
    assert "wifi_channel = 100" in open(cfg_out).read()
    # link-only change must NOT also trigger the GS-local wfb lane
    assert obj["gs"]["wfbBounced"] is True   # the coordinator bounced (mode=bounce)


def test_apply_shared_link_change_unreachable_soft_degrades():
    api, store, drone, cfg_out = _api()
    render_mod.write_cfg(cfg_out, render_mod.render_cfg(store.effective()))
    drone._reachable = False
    api.handle("PATCH", "/config", {}, json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/apply", {}, b"")
    # drone DOWN still applies on the GS (soft-degrade), 200
    assert code == 200
    assert obj["sharedLink"]["gsApplied"] is True
    assert obj["sharedLink"]["droneApplied"] is False
    assert store.effective()["link"]["channel"] == 100


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


def _api_with_dynlink(tmp_path, drone=None):
    import json
    from fpvdgs import render, schema
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient

    defaults = {"link": {"channel": 132, "width": 40, "region": "US"},
                "wfb": {"profile": "gs", "raw": {}},
                "droneLink": {"endpoint": "http://10.5.0.10:8080"},
                "dynamicLink": {"enabled": False,
                                 "controller": {"maxMcs": 5,
                                                "radioProfile": "m8812eu2",
                                                "dronePort": 9999, "tuning": {}}}}
    store = ConfigStore(defaults)
    ctrl = _FakeController()
    runner = _FakeRunner()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render, runner=runner,
              drone=drone or DroneClient("http://127.0.0.1:1"), link=None,
              status_fn=lambda: {}, cfg_out=cfg_out, dynlink=ctrl)
    return api, store, ctrl, runner


def test_enable_dynamiclink_starts_controller_without_bouncing_runner(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path, drone=FakeDrone())
    store.patch({"dynamicLink": {"enabled": True}})
    code, body = api.handle("POST", "/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert ("start", None) in ctrl.calls
    assert runner.restarts == 0          # adaptive-link-only change: no bounce
    assert store.effective()["dynamicLink"]["enabled"] is True


def test_disable_dynamiclink_stops_controller(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path, drone=FakeDrone())
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    store.patch({"dynamicLink": {"enabled": False}})
    api.handle("POST", "/apply", {}, b"")
    assert ("stop", None) in ctrl.calls
    assert runner.restarts == 0


def test_tuning_change_while_enabled_calls_set_config(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path, drone=FakeDrone())
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    store.patch({"dynamicLink": {"controller": {"maxMcs": 3}}})
    api.handle("POST", "/apply", {}, b"")
    assert any(c[0] == "set_config" for c in ctrl.calls)
    assert runner.restarts == 0


def test_apply_enable_toggle_unreachable_hard_gates(tmp_path):
    # dynamicLink.enabled flips False->True but the drone is unreachable: hard-gate
    # with 409 and commit nothing.
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)   # DroneClient -> unreachable
    store.patch({"dynamicLink": {"enabled": True}})
    code, obj = api.handle("POST", "/apply", {}, b"")
    assert code == 409
    assert obj["applied"] is False
    assert ctrl.calls == []                                  # controller not started
    assert store.effective()["dynamicLink"]["enabled"] is False   # NOT committed


def test_apply_enable_toggle_reachable_ok(tmp_path):
    drone = FakeDrone()                                      # healthz() -> True
    api, store, ctrl, runner = _api_with_dynlink(tmp_path, drone=drone)
    store.patch({"dynamicLink": {"enabled": True}})
    code, obj = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert ("start", None) in ctrl.calls                    # controller started
    assert store.effective()["dynamicLink"]["enabled"] is True


def test_wfb_change_bounces_runner_and_leaves_controller_alone(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}}})
    code, _ = api.handle("POST", "/apply", {}, b"")
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


def test_apply_pixelpilot_only_fires_gs_local_not_drone(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"screenMode": "1280x720@60"}})
    code, body = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert body["gs"]["applied"] is True
    assert body["gs"]["wfbBounced"] is False       # pixelpilot-only: no wfb bounce
    assert body["drone"]["fired"] is False         # _drone_dirty False
    assert ("restart", None) in pp.calls
    assert store.effective()["pixelpilot"]["screenMode"] == "1280x720@60"


def test_apply_drone_dirty_fires_drone_lane():
    api, store, drone, _ = _api()
    api._drone_dirty = True                          # a prior drone PATCH marked it dirty
    code, body = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert body["drone"] == {"fired": True, "applied": True}
    assert ("POST", "/apply", None) in drone.calls
    assert api._drone_dirty is False                 # cleared after firing


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


# --- probe lifecycle rides the dynamicLink transition (no probe config) ---
class _FakeProbe:
    def __init__(self): self.started = False; self.cfgs = []
    def start(self): self.started = True
    def stop(self): self.started = False
    def set_config(self, snap): self.cfgs.append(snap)
    def status(self): return {"running": self.started, "streams": 1, "mcs": {}}


def _api_with_dl_and_probe(tmp_path, drone=None):
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient
    # link.wlans is an explicit list so make_probe_snapshot's resolve_wlans
    # returns it directly (no wfb-nics / hardware probe).
    defaults = {"link": {"channel": 132, "width": 40, "region": "US",
                         "linkId": 7669206, "wlans": ["wlan0"]},
                "wfb": {"profile": "gs", "raw": {}},
                "droneLink": {"endpoint": "http://10.5.0.10:8080"},
                "dynamicLink": {"enabled": False,
                                 "controller": {"maxMcs": 5,
                                                "radioProfile": "m8812eu2",
                                                "dronePort": 9999, "tuning": {}}}}
    store = ConfigStore(defaults)
    ctrl = _FakeController()     # existing fake dynlink controller in this file
    probe = _FakeProbe()
    runner = _FakeRunner()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone or DroneClient("http://127.0.0.1:1"), link=None,
              status_fn=lambda: {}, cfg_out=cfg_out, dynlink=ctrl, probe=probe)
    return api, store, ctrl, probe, runner


def test_enable_dynamiclink_starts_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path, drone=FakeDrone())
    store.patch({"dynamicLink": {"enabled": True}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert ("start", None) in ctrl.calls and probe.started is True
    assert runner.restarts == 0           # no video bounce


def test_disable_dynamiclink_stops_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path, drone=FakeDrone())
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    store.patch({"dynamicLink": {"enabled": False}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert probe.started is False and runner.restarts == 0
