from fpvdgs.supervisor import App
from fpvdgs.config import ConfigStore


class _Fake:
    def __init__(self, name=None, log=None):
        self.calls = []
        self._name = name
        self._log = log  # shared list[(name, event)] for cross-object ordering
    def start(self):
        self.calls.append("start")
        if self._log is not None:
            self._log.append((self._name, "start"))
    def stop(self):
        self.calls.append("stop")
        if self._log is not None:
            self._log.append((self._name, "stop"))
    def shutdown(self):
        self.calls.append("shutdown")
        if self._log is not None:
            self._log.append((self._name, "shutdown"))
    def serve_forever(self):
        pass


def _app(pp_enabled, log=None):
    store = ConfigStore({"pixelpilot": {"enabled": pp_enabled},
                         "dynamicLink": {"enabled": False}})
    runner = _Fake("runner", log)
    http = _Fake("http", log)
    dynlink = _Fake("dynlink", log)
    pp = _Fake("pp", log)
    return App(store, runner, http, api=None, dynlink=dynlink, pixelpilot=pp), pp, runner


def test_app_starts_pixelpilot_when_enabled():
    app, pp, runner = _app(True)
    app.start()
    assert "start" in pp.calls
    assert "start" in runner.calls


def test_app_skips_pixelpilot_when_disabled():
    app, pp, runner = _app(False)
    app.start()
    assert pp.calls == []
    assert "start" in runner.calls


def test_app_shutdown_stops_pixelpilot():
    app, pp, runner = _app(True)
    app.start()
    app.shutdown()
    assert "shutdown" in pp.calls


def test_app_shutdown_order_pixelpilot_before_runner():
    log = []
    app, pp, runner = _app(True, log=log)
    app.start()
    app.shutdown()
    # pixelpilot consumes wfb's video, so it must stop before the wfb runner
    events = [name for name, event in log if event == "shutdown"]
    assert events.index("pp") < events.index("runner")


def test_build_app_wires_api_collaborators(tmp_path, monkeypatch):
    """build_app must wire the new Api collaborators (retune, wlans_resolver,
    armer_tick) so /gs/apply can retune the GS and reconcile the beamformee."""
    import fpvdgs.supervisor as sup

    monkeypatch.setattr(sup.render_mod, "write_cfg", lambda *a, **k: None)
    monkeypatch.setattr(sup.render_mod, "render_cfg", lambda eff: "")

    # Explicit wlans so every resolve_wlans() (supervisor AND probe.config_build)
    # short-circuits without shelling out to `wfb-nics`.
    config = tmp_path / "config.json"
    config.write_text('{"link": {"region": "US", "channel": 132, "width": 20, '
                      '"wlans": ["wlan0"]}}')

    app = sup.build_app(str(config), str(tmp_path / "out.cfg"),
                        "127.0.0.1", 0, runner_cmd=["true"])
    assert app.api.retune is not None
    assert app.api.wlans_resolver is not None
    assert app.api.armer_tick is not None


def test_app_starts_and_stops_connection_monitor():
    store = ConfigStore({"dynamicLink": {"enabled": False}})
    runner = _Fake("runner")
    mon = _Fake("mon")
    app = App(store, runner, _Fake("http"), api=None, dynlink=_Fake("dynlink"),
              connection_monitor=mon)
    app.start()
    assert "start" in mon.calls          # always-on, regardless of dynamicLink
    app.shutdown()
    assert "stop" in mon.calls


def test_build_app_wires_connection_monitor_and_bus(tmp_path, monkeypatch):
    import fpvdgs.supervisor as sup
    monkeypatch.setattr(sup.render_mod, "write_cfg", lambda *a, **k: None)
    monkeypatch.setattr(sup.render_mod, "render_cfg", lambda eff: "")
    config = tmp_path / "config.json"
    config.write_text('{"link": {"region": "US", "channel": 132, "width": 20, '
                      '"wlans": ["wlan0"]}}')
    app = sup.build_app(str(config), str(tmp_path / "out.cfg"),
                        "127.0.0.1", 0, runner_cmd=["true"])
    assert app.connection_monitor is not None
    assert app.bus is not None
