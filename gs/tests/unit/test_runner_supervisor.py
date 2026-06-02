import os
import signal
import socket
import time

from fpvdgs.runner_supervisor import RunnerSupervisor, resolve_wlans


def test_resolve_wlans_explicit_list():
    assert resolve_wlans({"link": {"wlans": ["wlan0", "wlan1"]}}) == ["wlan0", "wlan1"]


def test_resolve_wlans_auto_uses_wfb_nics(monkeypatch):
    import fpvdgs.runner_supervisor as rs
    monkeypatch.setattr(rs, "_wfb_nics", lambda: ["wlxAAA", "wlxBBB"])
    assert resolve_wlans({"link": {"wlans": "auto"}}) == ["wlxAAA", "wlxBBB"]


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _listener_cmd(port):
    code = (
        "import socket,time;"
        "s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);"
        f"s.bind(('127.0.0.1',{port}));s.listen(1);"
        "time.sleep(30)"
    )
    return ["python3", "-c", code]


def _mk(port, **kw):
    return RunnerSupervisor(_listener_cmd(port), cfg_out="/tmp/ignored.cfg",
                            profile="gs", wlans=["wlan0"], ready_port=port,
                            ready_timeout=5.0, poll_interval=0.05, backoff=0.05, **kw)


def test_start_reaches_ready_then_stop():
    port = _free_port()
    sup = _mk(port)
    try:
        assert sup.start() is True
        assert sup.state()["running"] is True
        assert sup.state()["pid"] > 0
        sup.stop()
        time.sleep(0.2)
        assert sup.state()["running"] is False
    finally:
        sup.shutdown()


def test_restart_increments_counter():
    port = _free_port()
    sup = _mk(port)
    try:
        sup.start()
        sup.restart()
        assert sup.state()["restarts"] >= 1
    finally:
        sup.shutdown()


def test_watcher_auto_restarts_on_crash():
    port = _free_port()
    sup = _mk(port)
    try:
        assert sup.start() is True
        pid1 = sup.state()["pid"]
        os.killpg(os.getpgid(pid1), signal.SIGKILL)  # simulate a crash
        deadline = time.time() + 6
        while time.time() < deadline:
            st = sup.state()
            if st["running"] and st["pid"] != pid1:
                break
            time.sleep(0.05)
        st = sup.state()
        assert st["running"] is True
        assert st["pid"] != pid1
        assert st["autoRestarts"] >= 1
    finally:
        sup.shutdown()


def test_operator_restart_does_not_trip_fault():
    port = _free_port()
    sup = _mk(port, max_restarts=2)
    try:
        sup.start()
        for _ in range(5):
            assert sup.restart() is True
        st = sup.state()
        assert st["fault"] is False
        assert st["running"] is True
    finally:
        sup.shutdown()


def test_crash_loop_sets_fault():
    sup = RunnerSupervisor(["python3", "-c", "import sys; sys.exit(1)"],
                           cfg_out="/tmp/ignored.cfg", profile="gs", wlans=["wlan0"],
                           ready_port=_free_port(), ready_timeout=0.5,
                           poll_interval=0.05, backoff=0.05, max_restarts=2)
    try:
        sup.start()  # exits immediately; never binds the port
        deadline = time.time() + 6
        while time.time() < deadline and not sup.state()["fault"]:
            time.sleep(0.05)
        assert sup.state()["fault"] is True
    finally:
        sup.shutdown()


def test_shutdown_stops_watcher():
    port = _free_port()
    sup = _mk(port)
    sup.start()
    sup.shutdown()
    assert sup.state()["running"] is False
    assert sup._watcher is None or not sup._watcher.is_alive()
