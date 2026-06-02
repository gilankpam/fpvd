import socket
import time

from fpvdgs.runner_supervisor import RunnerSupervisor, resolve_wlans


def test_resolve_wlans_explicit_list():
    assert resolve_wlans({"link": {"wlans": ["wlan0", "wlan1"]}}) == ["wlan0", "wlan1"]


def test_resolve_wlans_auto_uses_wfb_nics(monkeypatch):
    import fpvdgs.runner_supervisor as rs
    monkeypatch.setattr(rs, "_wfb_nics", lambda: ["wlxAAA", "wlxBBB"])
    assert resolve_wlans({"link": {"wlans": "auto"}}) == ["wlxAAA", "wlxBBB"]


def _listener_cmd(port):
    # a fake runner that opens the readiness port and sleeps
    code = (
        "import socket,time;"
        "s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);"
        f"s.bind(('127.0.0.1',{port}));s.listen(1);"
        "time.sleep(30)"
    )
    return ["python3", "-c", code]


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_start_reaches_ready_then_stop():
    port = _free_port()
    sup = RunnerSupervisor(_listener_cmd(port), cfg_out="/tmp/ignored.cfg",
                           profile="gs", wlans=["wlan0"], ready_port=port,
                           ready_timeout=5.0)
    assert sup.start() is True
    assert sup.state()["running"] is True
    assert sup.state()["pid"] > 0
    sup.stop()
    time.sleep(0.2)
    assert sup.state()["running"] is False


def test_restart_increments_counter():
    port = _free_port()
    sup = RunnerSupervisor(_listener_cmd(port), cfg_out="/tmp/ignored.cfg",
                           profile="gs", wlans=["wlan0"], ready_port=port,
                           ready_timeout=5.0)
    sup.start()
    sup.restart()
    assert sup.state()["restarts"] >= 1
    sup.stop()
