import asyncio
from fpvdgs.probe.controller import ProbeController

def _snap(**over):
    s = {"enabled": True, "basePort": 50, "maxStreams": 2, "rxL": 50,
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
