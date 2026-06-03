from fpvdgs.supervisor import App
from fpvdgs.config import ConfigStore


class _Fake:
    def __init__(self):
        self.calls = []
    def start(self):
        self.calls.append("start")
    def stop(self):
        self.calls.append("stop")
    def shutdown(self):
        self.calls.append("shutdown")
    def serve_forever(self):
        pass


def _app(pp_enabled):
    store = ConfigStore({"pixelpilot": {"enabled": pp_enabled},
                         "dynamicLink": {"enabled": False}})
    runner, http, dynlink, pp = _Fake(), _Fake(), _Fake(), _Fake()
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
    app.shutdown()
    assert "shutdown" in pp.calls
