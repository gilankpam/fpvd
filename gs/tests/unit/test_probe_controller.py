import asyncio
from fpvdgs.probe.controller import ProbeController

def _snap(**over):
    s = {"port": 50, "rxL": 50, "key": "/etc/gs.key",
         "linkId": 7669206, "wlans": ["wlanA", "wlanB"]}
    s.update(over)
    return s

class _FakeProc:
    """Emits scripted stdout lines then idles until killed."""
    def __init__(self, lines):
        self._lines = list(lines)
        self.stdout = self
        self.killed = False
    async def readline(self):
        if self._lines:
            return (self._lines.pop(0) + "\n").encode()
        await asyncio.sleep(3600)
    def kill(self): self.killed = True
    async def wait(self): return 0

def _wait_until(pred, timeout=2.0):
    import time
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False

def test_builds_one_wfb_rx_on_fixed_port():
    cmds = []
    def spawn(cmd):
        cmds.append(cmd)
        return _FakeProc([])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        assert len(cmds) == 1
        cmd = cmds[0]
        assert "/usr/bin/wfb_rx" in cmd[0]
        assert "-p" in cmd and "50" in cmd
        assert "-K" in cmd and "/etc/gs.key" in cmd
        assert "-i" in cmd and "7669206" in cmd
        assert "wlanA" in cmd and "wlanB" in cmd
        assert c.status()["running"] is True and c.status()["streams"] == 1
    finally:
        c.stop()
    assert c.status()["running"] is False

def test_measures_per_mcs_from_stdout():
    def spawn(cmd):
        return _FakeProc(["1\tRX_ANT\t5805:5:20\t0\t1:-80:-80:-80:8:8:8",
                          "1\tPKT\t10:0:0:0:1:1:0:9:0:1:0:0:0:0"])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        assert _wait_until(lambda: "5" in c.status()["mcs"])
        mcs = c.status()["mcs"]
        assert abs(mcs["5"]["per"] - 0.9) < 1e-9 and mcs["5"]["snr"] == 8
    finally:
        c.stop()

def test_retune_followed_via_rx_ant_key():
    # The same fixed port carries a different MCS after the drone retunes; the
    # aggregator keys by RX_ANT mcs, so a new slot appears.
    def spawn(cmd):
        return _FakeProc(["1\tRX_ANT\t5805:3:20\t0\t9:-55:-55:-55:28:28:28",
                          "1\tPKT\t9:0:0:0:9:9:0:0:0:9:0:0:0:0",
                          "2\tRX_ANT\t5805:4:20\t0\t9:-56:-56:-56:27:27:27",
                          "2\tPKT\t9:0:0:0:9:9:0:0:0:9:0:0:0:0"])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        assert _wait_until(lambda: {"3", "4"} <= set(c.status()["mcs"]))
        snap = c.status()["mcs"]
        assert snap["3"]["per"] == 0.0 and snap["4"]["per"] == 0.0
    finally:
        c.stop()

def test_snapshot_reads_tuning_from_config():
    from fpvdgs.probe.config_build import make_probe_snapshot
    eff = {"link": {"linkId": 7, "wlans": ["wlanA"]},
           "probe": {"rxL": 800, "blackoutWindows": 2, "ewmaAlpha": 0.1}}
    snap = make_probe_snapshot(eff)
    assert snap["rxL"] == 800
    assert snap["blackoutWindows"] == 2
    assert snap["ewmaAlpha"] == 0.1


def test_snapshot_tuning_defaults():
    from fpvdgs.probe.config_build import make_probe_snapshot
    eff = {"link": {"linkId": 7, "wlans": ["wlanA"]}}
    snap = make_probe_snapshot(eff)
    assert snap["rxL"] == 50
    assert snap["blackoutWindows"] == 10
    assert snap["ewmaAlpha"] == 0.25


def test_controller_honors_blackout_windows_from_snapshot():
    # blackoutWindows=2 → a run of 2 empty windows pins per=1.0
    def spawn(cmd):
        return _FakeProc(["1\tRX_ANT\t5805:5:20\t0\t5:-60:-60:-60:20:20:20",
                          "1\tPKT\t5:0:0:0:5:5:0:0:0:5:0:0:0:0",   # real: per 0
                          "2\tPKT\t0:0:0:0:0:0:0:0:0:0:0:0:0:0",   # empty 1
                          "2\tPKT\t0:0:0:0:0:0:0:0:0:0:0:0:0:0"])  # empty 2 -> blackout
    c = ProbeController(_snap(blackoutWindows=2), spawn=spawn)
    c.start()
    try:
        assert _wait_until(lambda: c.status()["mcs"].get("5", {}).get("per") == 1.0)
    finally:
        c.stop()


def test_status_exposes_age_ms_freshness():
    def spawn(cmd):
        return _FakeProc(["1\tRX_ANT\t5805:5:20\t0\t1:-60:-60:-60:20:20:20",
                          "1\tPKT\t10:0:0:0:10:10:0:0:0:10:0:0:0:0"])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        assert _wait_until(lambda: "5" in c.status()["mcs"])
        age = c.status()["mcs"]["5"]["ageMs"]
        assert age is not None and age >= 0.0 and age < 2000.0
    finally:
        c.stop()
