import json

from fpvdgs import schema
from fpvdgs.api import Api
from fpvdgs.config import ConfigStore


class FakeRunner:
    def __init__(self):
        self.restarts = 0
        self.restart_configs = []  # the config handed to each restart() call
        self.restart_returns = []  # optional queued return values (pop-front)

    def restart(self, config=None):
        self.restarts += 1
        self.restart_configs.append(config)
        if self.restart_returns:
            return self.restart_returns.pop(0)
        return True

    def state(self):
        return {
            "running": True,
            "pid": 1,
            "restarts": self.restarts,
            "lastExit": None,
            "fault": False,
        }


class FakeDrone:
    def __init__(self):
        self.calls = []

    def healthz(self):
        return True

    def _request(self, method, path, body=None):
        self.calls.append((method, path, body))
        return 200, json.dumps({"proxied": path}).encode()


def _api(retune_ok=True, nodes_status_fn=None):
    store = ConfigStore(
        {
            "link": {"channel": 132, "width": 40, "region": "US"},
            "wfb": {"profile": "gs"},
            "drone": {"endpoint": "http://x"},
        },
        config_path=None,
    )
    drone = FakeDrone()
    runner = FakeRunner()
    retunes = []

    def retune(link):
        retunes.append(link)
        return retune_ok

    ticks = []
    api = Api(
        store=store,
        schema=schema,
        runner=runner,
        drone=drone,
        status_fn=lambda: {"ok": True},
        retune=retune,
        wlans_resolver=lambda cfg: ["wlan0"],
        armer_tick=lambda: ticks.append(1),
        nodes_status_fn=nodes_status_fn,
    )
    return api, store, drone, runner, retunes, ticks


def test_apply_does_not_touch_any_cfg_renderer():
    # Native engine reads no /etc/wifibroadcast.cfg; apply must not render one.
    api, *_ = _api()
    api.handle("PATCH", "/gs/config", {}, b'{"link":{"channel":140}}')
    status, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert status == 200
    assert not hasattr(api, "render_mod")


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


def test_nodes_endpoint_returns_injected_payload():
    calls = []

    def nodes_status_fn():
        calls.append(1)
        return {"nodes": [{"iface": "wlan0", "reachable": True}]}

    api, *_ = _api(nodes_status_fn=nodes_status_fn)
    code, obj = api.handle("GET", "/gs/nodes", {}, b"")
    assert code == 200
    assert obj == {"nodes": [{"iface": "wlan0", "reachable": True}]}
    assert calls == [1]


def test_nodes_endpoint_404_when_not_wired():
    api, *_ = _api()  # nodes_status_fn defaults to None: back-compat
    code, obj = api.handle("GET", "/gs/nodes", {}, b"")
    assert code == 404


def test_status_never_touches_nodes_status_fn():
    calls = []

    def nodes_status_fn():
        calls.append(1)
        return {"nodes": []}

    api, *_ = _api(nodes_status_fn=nodes_status_fn)
    code, obj = api.handle("GET", "/gs/status", {}, b"")
    assert code == 200
    assert calls == []  # the whole point: /gs/status never calls the node querier


def test_link_change_retunes_live_no_bounce():
    api, store, _, runner, retunes, ticks = _api()
    api.handle("PATCH", "/gs/config", {}, json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and obj["applied"] is True
    assert retunes and retunes[-1]["channel"] == 100  # retuned
    assert runner.restarts == 0  # no bounce


def test_link_change_bounces_on_wlans():
    api, store, _, runner, retunes, ticks = _api()
    api.handle("PATCH", "/gs/config", {}, json.dumps({"link": {"wlans": ["wlan1"]}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1 and retunes == []


def test_failed_retune_falls_back_to_bounce():
    api, store, _, runner, retunes, ticks = _api(retune_ok=False)
    api.handle("PATCH", "/gs/config", {}, json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1  # retune failed -> bounced


def test_video_encryption_change_bounces_no_live_retune():
    api, store, _, runner, retunes, ticks = _api()
    api.handle("PATCH", "/gs/config", {}, json.dumps({"link": {"videoEncryption": False}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and obj["applied"] is True
    assert runner.restarts == 1  # bounced (keypair is constructor-time)
    assert retunes == []  # NOT a live iw retune


def test_apply_bounce_hands_new_config_to_runner_restart():
    # The native WfbEngine rebuilds from the config handed to restart(); it must
    # be the PENDING (new) config, NOT the not-yet-committed store. Bench-caught
    # 2026-07-04: a link.cards/videoEncryption change applied cleanly but the
    # engine kept the OLD wiring because _apply_gs calls runner.restart() before
    # store.commit(), and the engine reads store.effective() (still old).
    api, store, _, runner, retunes, ticks = _api()
    api.handle("PATCH", "/gs/config", {}, json.dumps({"link": {"videoEncryption": False}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1
    assert runner.restart_configs[-1] is not None  # config was threaded through
    assert runner.restart_configs[-1]["link"]["videoEncryption"] is False  # the NEW value


def test_apply_rollback_hands_old_config_to_runner_restart():
    # On a failed apply-bounce the engine must rebuild from the OLD config so the
    # rollback restores the last-good wiring.
    store = ConfigStore(
        {"link": {"channel": 132, "width": 40, "region": "US", "videoEncryption": True}},
        config_path=None,
    )
    runner = FakeRunner()
    runner.restart_returns = [False, True]  # apply restart fails -> rollback restart
    api = Api(
        store=store,
        schema=schema,
        runner=runner,
        drone=FakeDrone(),
        status_fn=lambda: {"ok": True},
        retune=lambda link: False,
        wlans_resolver=lambda cfg: ["wlan0"],
        armer_tick=lambda: None,
    )
    api.handle("PATCH", "/gs/config", {}, json.dumps({"link": {"videoEncryption": False}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 500 and obj["applied"] is False
    assert len(runner.restart_configs) == 2
    assert runner.restart_configs[0]["link"]["videoEncryption"] is False  # apply attempt = new
    assert runner.restart_configs[1]["link"]["videoEncryption"] is True  # rollback = old


def test_apply_fires_armer_tick():
    api, store, drone, runner, retunes, ticks = _api()
    api.handle("POST", "/gs/apply", {}, b"")
    assert ticks == [1]


def test_apply_tap_port_change_bounces_runner():
    api, store, _, runner, retunes, ticks = _api()
    code, _ = api.handle("PATCH", "/gs/config", {}, b'{"dynamicLink": {"tap": {"port": 8111}}}')
    assert code == 200
    code, body = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1


def test_apply_tap_stale_ms_change_is_hot():
    api, store, _, runner, retunes, ticks = _api()
    code, _ = api.handle("PATCH", "/gs/config", {}, b'{"dynamicLink": {"tap": {"staleMs": 250}}}')
    assert code == 200
    code, body = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 0


def test_apply_probe_enable_bounces_runner():
    """dynamicLink.probe.enabled is baked into the engine graph at spawn
    time (2026-07-06 spec Part B) — flipping it (with dynamicLink enabled)
    must bounce the runner, exactly like the tap render view."""
    api, store, _, runner, retunes, ticks = _api()
    # Must set width to 20 because 40 MHz blocks dynamicLink.enabled=true (schema invariant)
    code, _ = api.handle(
        "PATCH",
        "/gs/config",
        {},
        b'{"link": {"width": 20}, "dynamicLink": {"enabled": true, "probe": {"enabled": true}}}',
    )
    assert code == 200
    code, body = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1


def test_apply_probe_enable_without_dl_enabled_is_hot():
    """probe.enabled=true with dynamicLink.enabled=false renders no probe
    leg (the conjunction is unchanged: false) — no runner bounce."""
    api, store, _, runner, retunes, ticks = _api()
    code, _ = api.handle(
        "PATCH", "/gs/config", {}, b'{"dynamicLink": {"probe": {"enabled": true}}}'
    )
    assert code == 200
    code, body = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 0


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

    def restart(self, config=None):
        self.restarts += 1
        return True


def _api_with_dynlink(tmp_path):
    from fpvdgs import schema
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient

    defaults = {
        "link": {"channel": 132, "width": 20, "region": "US"},
        "wfb": {"profile": "gs", "raw": {}},
        "drone": {"endpoint": "http://10.5.0.10:8080"},
        "dynamicLink": {
            "enabled": False,
            "maxMcs": 5,
            "dronePort": 9999,
        },
    }
    store = ConfigStore(defaults)
    ctrl = _FakeController()
    runner = _FakeRunner()
    api = Api(
        store=store,
        schema=schema,
        runner=runner,
        drone=DroneClient("http://127.0.0.1:1"),
        status_fn=lambda: {},
        dynlink=ctrl,
    )
    return api, store, ctrl, runner


def test_enable_dynamiclink_starts_controller_without_bouncing_runner(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    code, body = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert ("start", None) in ctrl.calls
    assert runner.restarts == 0  # dynamic-link-only change: no bounce
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
    assert runner.restarts == 1  # non-dynamicLink change: bounce
    assert ctrl.calls == []  # controller untouched (stayed disabled)


def test_width_change_while_enabled_reconfigures_controller(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/gs/apply", {}, b"")
    ctrl.calls.clear()
    store.patch({"link": {"width": 10}})  # 20 -> 10 while DL stays enabled
    api.handle("POST", "/gs/apply", {}, b"")
    # Controller rebuilt so it re-binds the prior to <adapter>__bw10 + resets.
    assert any(c[0] == "set_config" for c in ctrl.calls)


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

    def restart(self, config=None):
        self.calls.append(("restart", None))


def _api_with_pp(tmp_path):
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient

    defaults = {
        "link": {"channel": 132, "width": 40, "region": "US"},
        "wfb": {"profile": "gs", "raw": {}},
        "drone": {"endpoint": "http://10.5.0.10:8080"},
        "pixelpilot": {"enabled": True, "screenMode": "1920x1080@60", "extraArgs": []},
    }
    store = ConfigStore(defaults)
    runner = _FakeRunner()  # defined earlier in this file
    pp = _FakePP()
    api = Api(
        store=store,
        schema=schema,
        runner=runner,
        drone=DroneClient("http://127.0.0.1:1"),
        status_fn=lambda: {},
        pixelpilot=pp,
    )
    return api, store, pp, runner


def test_pixelpilot_change_restarts_pp_not_wfb(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"screenMode": "1280x720@60"}})
    code, body = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert any(c[0] == "set_argv" for c in pp.calls)
    assert ("restart", None) in pp.calls
    assert runner.restarts == 0  # PixelPilot-only change: radio untouched
    assert store.effective()["pixelpilot"]["screenMode"] == "1280x720@60"


def test_wfb_change_does_not_touch_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}}})
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1
    assert pp.calls == []  # pixelpilot untouched


def test_pixelpilot_disable_then_enable(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"enabled": False}})
    api.handle("POST", "/gs/apply", {}, b"")
    assert ("stop", None) in pp.calls
    pp.calls.clear()
    store.patch({"pixelpilot": {"enabled": True}})
    api.handle("POST", "/gs/apply", {}, b"")
    assert ("start", None) in pp.calls  # off->on uses start(), not restart()
    assert ("restart", None) not in pp.calls


def test_combined_wfb_and_pixelpilot_change_bounces_both(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch(
        {"wfb": {"raw": {"common": {"foo": 1}}}, "pixelpilot": {"screenMode": "1280x720@60"}}
    )
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1  # wfb bounced
    assert ("restart", None) in pp.calls  # pixelpilot bounced too


def test_patch_config_accepts_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    code, _ = api.handle(
        "PATCH",
        "/gs/config",
        {},
        json.dumps({"pixelpilot": {"screenMode": "1280x720@60"}}).encode(),
    )
    assert code == 200
    assert store.pending()["pixelpilot"]["screenMode"] == "1280x720@60"


# --- probe lifecycle rides the dynamicLink transition (no probe config) ---
class _FakeProbe:
    def __init__(self):
        self.started = False
        self.cfgs = []

    def start(self):
        self.started = True

    def stop(self):
        self.started = False

    def set_config(self, snap):
        self.cfgs.append(snap)

    def status(self):
        return {"running": self.started, "streams": 1, "mcs": {}}


def _api_with_dl_and_probe(tmp_path):
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient

    # link.wlans is an explicit list so make_probe_snapshot's resolve_wlans
    # returns it directly (no wfb-nics / hardware probe).
    defaults = {
        "link": {
            "channel": 132,
            "width": 20,
            "region": "US",
            "linkId": 7669206,
            "wlans": ["wlan0"],
        },
        "wfb": {"profile": "gs", "raw": {}},
        "drone": {"endpoint": "http://10.5.0.10:8080"},
        "dynamicLink": {
            "enabled": False,
            "maxMcs": 5,
            "dronePort": 9999,
        },
    }
    store = ConfigStore(defaults)
    ctrl = _FakeController()  # existing fake dynlink controller in this file
    probe = _FakeProbe()
    runner = _FakeRunner()
    api = Api(
        store=store,
        schema=schema,
        runner=runner,
        drone=DroneClient("http://127.0.0.1:1"),
        status_fn=lambda: {},
        dynlink=ctrl,
        probe=probe,
    )
    return api, store, ctrl, probe, runner


def test_enable_dynamiclink_starts_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert ("start", None) in ctrl.calls and probe.started is True
    assert runner.restarts == 0  # no video bounce


def test_disable_dynamiclink_stops_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/gs/apply", {}, b"")
    store.patch({"dynamicLink": {"enabled": False}})
    code, _ = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200
    assert probe.started is False and runner.restarts == 0


def test_width_only_change_does_not_bounce_probe(tmp_path):
    """Regression guard: a width-only change rebuilds the DL controller (so it
    re-keys the learned prior) but must NOT call probe.set_config — the probe
    snapshot is width-agnostic and bouncing it would interrupt the uplink."""
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    # Enable DL first so the controller is running.
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/gs/apply", {}, b"")
    # Clear both controllers' recorded calls before the width change.
    ctrl.calls.clear()
    probe.cfgs.clear()
    # Apply a width-only change (link.width 20 -> 10; dynamicLink unchanged).
    store.patch({"link": {"width": 10}})
    api.handle("POST", "/gs/apply", {}, b"")
    # Controller must have been reconfigured (prior re-key + selector reset).
    assert any(c[0] == "set_config" for c in ctrl.calls)
    # Probe must NOT have been reconfigured.
    assert probe.cfgs == []


class FakeRelay:
    def __init__(self):
        self.events = []
        self._running = False

    def start(self):
        self._running = True
        self.events.append("start")

    def stop(self):
        self._running = False
        self.events.append("stop")

    def status(self):
        return {"running": self._running, "listen": None}


def test_idr_forward_apply_starts_and_stops():
    store = ConfigStore(
        {
            "link": {"channel": 132, "width": 40, "region": "US"},
            "wfb": {"profile": "gs"},
            "drone": {"endpoint": "http://x"},
            "idrForward": {"enabled": False, "port": 11223},
        },
        config_path=None,
    )
    relay = FakeRelay()
    api = Api(
        store=store,
        schema=schema,
        runner=FakeRunner(),
        drone=FakeDrone(),
        status_fn=lambda: {},
        retune=lambda _link: True,
        wlans_resolver=lambda c: ["wlan0"],
        armer_tick=lambda: None,
        idr_relay=relay,
    )
    api.handle("PATCH", "/gs/config", {}, json.dumps({"idrForward": {"enabled": True}}).encode())
    api.handle("POST", "/gs/apply", {}, b"")
    assert "start" in relay.events
    api.handle("PATCH", "/gs/config", {}, json.dumps({"idrForward": {"enabled": False}}).encode())
    api.handle("POST", "/gs/apply", {}, b"")
    assert relay.events[-1] == "stop"
