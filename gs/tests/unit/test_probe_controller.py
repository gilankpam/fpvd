import asyncio
from fpvdgs.probe.controller import ProbeController

def _snap(**over):
    # rxL=20 deliberately avoids colliding with the radio_ports (50/51) or the
    # throwaway sink ports (7000/7001) so the "50"/"51" routing in the tests
    # below matches only the -p port token.
    s = {"enabled": True, "basePort": 50, "maxStreams": 2, "rxL": 20,
         "key": "/etc/gs.key", "linkId": 7669206, "wlans": ["wlanA", "wlanB"]}
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

def test_builds_one_wfb_rx_cmd_per_stream():
    cmds = []
    def spawn(cmd):
        cmds.append(cmd)
        return _FakeProc([])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        # port 50 and 51, sharing key/linkId/wlans
        assert any("-p" in cmd and "50" in cmd for cmd in cmds)
        assert any("-p" in cmd and "51" in cmd for cmd in cmds)
        c0 = cmds[0]
        assert "/usr/bin/wfb_rx" in c0[0]
        assert "-K" in c0 and "/etc/gs.key" in c0
        assert "-i" in c0 and "7669206" in c0
        assert "wlanA" in c0 and "wlanB" in c0
    finally:
        c.stop()

def test_measures_per_mcs_from_stdout():
    def spawn(cmd):
        # the -p 50 stream carries mcs 3 (clean), the -p 51 stream mcs 5 (lossy)
        if "50" in cmd:
            return _FakeProc(["1\tRX_ANT\t5805:3:20\t0\t9:-55:-55:-55:28:28:28",
                              "1\tPKT\t9:0:0:0:9:9:0:0:0:9:0:0:0:0"])
        return _FakeProc(["1\tRX_ANT\t5805:5:20\t0\t1:-80:-80:-80:8:8:8",
                          "1\tPKT\t10:0:0:0:1:1:0:9:0:1:0:0:0:0"])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        # let the reader tasks drain the scripted lines
        import time; time.sleep(0.4)
        st = c.status()
        assert st["enabled"] is True and st["running"] is True
        mcs = st["mcs"]
        assert mcs["3"]["per"] == 0.0 and mcs["3"]["rssi"] == -55
        assert abs(mcs["5"]["per"] - 0.9) < 1e-9 and mcs["5"]["snr"] == 8
    finally:
        c.stop()
    assert c.status()["running"] is False

def test_disabled_spawns_nothing():
    spawned = []
    c = ProbeController(_snap(enabled=False), spawn=lambda cmd: spawned.append(cmd) or _FakeProc([]))
    c.start()
    try:
        assert spawned == []
        assert c.status()["running"] is False
    finally:
        c.stop()


def _wait_until(pred, timeout=2.0, interval=0.01):
    """Poll pred() until truthy or timeout. Returns pred()'s last value."""
    import time
    deadline = time.monotonic() + timeout
    val = pred()
    while not val and time.monotonic() < deadline:
        time.sleep(interval)
        val = pred()
    return val


def test_spawn_failure_cleans_up_prior_procs():
    # Fix-1 regression: a spawn that succeeds for stream 0 (-p 50) but RAISES on
    # stream 1 (-p 51) must not orphan the stream-0 wfb_rx. The _run finally
    # block kills it; the loop thread tears down and start() must not hang.
    made = []
    def spawn(cmd):
        if "51" in cmd:
            raise RuntimeError("boom")
        p = _FakeProc([]); made.append(p); return p
    c = ProbeController(_snap(), spawn=spawn)
    c.start()   # must return (not hang) even though _run raised
    try:
        # the stream-0 proc is killed by _run's finally as the loop unwinds
        assert _wait_until(lambda: made and all(p.killed for p in made))
    finally:
        c.stop()
    assert made and all(p.killed for p in made)   # stream 0 killed, not orphaned
    assert c.status()["running"] is False


def test_hot_reconfig_changes_stream_count():
    # Fix-2 regression (restart path): a running controller restarts on
    # set_config and reflects the new maxStreams.
    c = ProbeController(_snap(maxStreams=2), spawn=lambda cmd: _FakeProc([]))
    c.start()
    try:
        assert _wait_until(lambda: c.status()["streams"] == 2)
        c.set_config(_snap(maxStreams=3))
        assert _wait_until(lambda: c.status()["streams"] == 3)
        assert c.status()["running"] is True
    finally:
        c.stop()


def test_disable_enable_round_trip_respawns():
    # Fix-2 trap regression: disable clears _thread; the OLD guard
    # (`running and enabled`) computed running=False on the later enable and
    # never restarted. The unconditional `if running` guard must respawn.
    spawned = []
    def spawn(cmd):
        spawned.append(cmd)
        return _FakeProc([])
    c = ProbeController(_snap(maxStreams=2), spawn=spawn)
    c.start()
    try:
        assert _wait_until(lambda: c.status()["streams"] == 2)
        c.set_config(_snap(enabled=False))
        assert _wait_until(lambda: c.status()["running"] is False)
        before = len(spawned)
        c.set_config(_snap(enabled=True, maxStreams=2))
        assert _wait_until(lambda: c.status()["running"] is True)
        assert _wait_until(lambda: c.status()["streams"] == 2)
        assert len(spawned) == before + 2   # genuinely respawned after re-enable
    finally:
        c.stop()
