import os
import signal
import time

from fpvdgs.runner_supervisor import ProcessSupervisor, resolve_wlans


def test_resolve_wlans_explicit_list():
    assert resolve_wlans({"link": {"wlans": ["wlan0", "wlan1"]}}) == ["wlan0", "wlan1"]


def test_resolve_wlans_auto_uses_wfb_nics(monkeypatch):
    import fpvdgs.runner_supervisor as rs

    monkeypatch.setattr(rs, "_wfb_nics", lambda: ["wlxAAA", "wlxBBB"])
    assert resolve_wlans({"link": {"wlans": "auto"}}) == ["wlxAAA", "wlxBBB"]


def test_resolve_wlans_sees_only_local_cards():
    # resolve_wlans is a shim over the flat card model (Phase 2 remote
    # cards): every existing caller must keep seeing only LOCAL ifaces.
    cfg = {
        "link": {
            "cards": [
                "wlan0",
                {"host": "10.0.0.5", "iface": "wlan1"},
            ]
        }
    }
    assert resolve_wlans(cfg) == ["wlan0"]


def _fake_supervisor(tmp_path, ready=True, **kw):
    # A tiny child that stays alive; readiness is a toggled flag, not a TCP port.
    script = tmp_path / "child.sh"
    script.write_text("#!/bin/sh\nsleep 30\n")
    script.chmod(0o755)
    state = {"ready": ready}
    return ProcessSupervisor(
        argv=["/bin/sh", str(script)],
        ready_check=lambda: state["ready"],
        ready_timeout=2.0,
        ready_on_timeout=False,
        poll_interval=0.05,
        backoff=0.05,
        **kw,
    ), state


def test_start_reaches_ready_then_stop(tmp_path):
    sup, state = _fake_supervisor(tmp_path)
    try:
        assert sup.start() is True
        assert sup.state()["running"] is True
        assert sup.state()["pid"] > 0
        sup.stop()
        time.sleep(0.2)
        assert sup.state()["running"] is False
    finally:
        sup.shutdown()


def test_restart_increments_counter(tmp_path):
    sup, state = _fake_supervisor(tmp_path)
    try:
        sup.start()
        sup.restart()
        assert sup.state()["restarts"] >= 1
    finally:
        sup.shutdown()


def test_watcher_auto_restarts_on_crash(tmp_path):
    sup, state = _fake_supervisor(tmp_path)
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


def test_operator_restart_does_not_trip_fault(tmp_path):
    sup, state = _fake_supervisor(tmp_path, max_restarts=2)
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
    sup = ProcessSupervisor(
        argv=["python3", "-c", "import sys; sys.exit(1)"],
        ready_check=lambda: False,
        ready_timeout=0.5,
        ready_on_timeout=False,
        poll_interval=0.05,
        backoff=0.05,
        max_restarts=2,
    )
    try:
        sup.start()  # exits immediately; never becomes ready
        deadline = time.time() + 6
        while time.time() < deadline and not sup.state()["fault"]:
            time.sleep(0.05)
        assert sup.state()["fault"] is True
    finally:
        sup.shutdown()


def test_shutdown_stops_watcher(tmp_path):
    sup, state = _fake_supervisor(tmp_path)
    sup.start()
    sup.shutdown()
    assert sup.state()["running"] is False
    assert sup._watcher is None or not sup._watcher.is_alive()
