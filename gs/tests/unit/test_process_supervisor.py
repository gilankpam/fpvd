import time

from fpvdgs.runner_supervisor import ProcessSupervisor


def _settle(argv, **kw):
    # readiness = "still alive at end of a short window" (no port probe)
    return ProcessSupervisor(argv, ready_check=None, ready_timeout=0.4,
                             ready_on_timeout=True, poll_interval=0.05,
                             backoff=0.05, **kw)


def test_settle_readiness_start_succeeds_for_living_process():
    sup = _settle(["sleep", "30"])
    try:
        assert sup.start() is True
        st = sup.state()
        assert st["running"] is True and st["pid"] > 0
    finally:
        sup.shutdown()


def test_settle_readiness_immediate_exit_is_failed_start():
    sup = _settle(["python3", "-c", "import sys; sys.exit(3)"], max_restarts=2)
    try:
        assert sup.start() is False
        # crash-loop guard trips after the budget
        deadline = time.time() + 6
        while time.time() < deadline and not sup.state()["fault"]:
            time.sleep(0.05)
        assert sup.state()["fault"] is True
        assert sup.state()["lastExit"] == 3
    finally:
        sup.shutdown()


def test_set_argv_is_used_on_restart():
    sup = _settle(["sleep", "30"])
    try:
        sup.start()
        sup.set_argv(["sleep", "31"])
        sup.restart()
        # the live process now runs the swapped argv
        import subprocess
        pid = sup.state()["pid"]
        out = subprocess.run(["ps", "-o", "args=", "-p", str(pid)],
                             capture_output=True, text=True).stdout
        assert "31" in out
    finally:
        sup.shutdown()
