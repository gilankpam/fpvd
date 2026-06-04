import os
import time

from fpvdgs.runner_supervisor import ProcessSupervisor

FIXTURE = os.path.join(os.path.dirname(__file__), "..", "fixtures",
                       "fake_pixelpilot.sh")


def _sup(argv, **kw):
    return ProcessSupervisor(argv, ready_check=None, ready_timeout=0.4,
                             ready_on_timeout=True, poll_interval=0.05,
                             backoff=0.05, **kw)


def test_set_argv_changes_what_gets_spawned(tmp_path, monkeypatch):
    argv_file = tmp_path / "argv.log"
    monkeypatch.setenv("PP_ARGV_FILE", str(argv_file))
    sup = _sup([FIXTURE, "--screen-mode", "MODE_A"])
    try:
        assert sup.start() is True
        sup.set_argv([FIXTURE, "--screen-mode", "MODE_B"])
        sup.restart()
        time.sleep(0.1)
        logged = argv_file.read_text()
        assert "MODE_A" in logged and "MODE_B" in logged
    finally:
        sup.shutdown()


def test_crash_recovery_then_fault(tmp_path):
    sup = _sup([FIXTURE, "--die"], max_restarts=2)
    try:
        assert sup.start() is False           # exits 7 immediately
        deadline = time.time() + 6
        while time.time() < deadline and not sup.state()["fault"]:
            time.sleep(0.05)
        assert sup.state()["fault"] is True
        assert sup.state()["lastExit"] == 7
    finally:
        sup.shutdown()
