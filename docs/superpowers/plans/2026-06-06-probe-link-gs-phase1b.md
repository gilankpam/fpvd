# Probe Link — GS Measurement (Phase 1b) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The fpvd ground-station daemon measures the drone's probe streams — spawning a FEC-off `wfb_rx` per probe `radio_port`, parsing each one's stdout for per-MCS on-air PER + RSSI/SNR, and surfacing it in `/status` — observe-only, hot-toggleable, without ever bouncing the video runner.

**Architecture:** A `ProbeController` (threaded asyncio, mirroring `DynamicLinkController`) owns the probe `wfb_rx` subprocesses. When `probe.enabled`, it spawns `wfb_rx -p <basePort+i>` for `i` in `0..maxStreams-1`, reads each process's **stdout** (the `RX_ANT`/`PKT` IPC lines wfb-ng's own runner parses), and maintains an EWMA-smoothed per-MCS view (MCS read from the radiotap `RX_ANT` key — *not* a static list, so it survives Phase 2's adaptive probing). A pure `parser` module turns stdout lines into per-MCS measurements; the controller manages lifecycle + status. Independent of the wfb-ng runner, so enable/disable never restarts video.

**Tech Stack:** Python 3.13, asyncio, pytest (vendored config in `gs/pyproject.toml`). GS is `aarch64` OpenIPC with `wfb_ng` 25.5.1 installed.

**Prereq context:** Phase 1a (drone injector) is merged-to-PR (#12) and hardware-validated — the drone, when `probe.enabled`, transmits FEC-off probe streams on `radio_port` `basePort+i` (`basePort=50` default). This GS side receives + measures them. The two `basePort`s must match (both default `50`).

**Run tests:** `cd /home/gilankpam/Projects/drone/fpvd/gs && python -m pytest tests/ -q` (pyproject sets `pythonpath=["."]`, `testpaths=["tests"]`). Run one: `python -m pytest tests/unit/test_probe_parser.py -q`.

---

## wfb_rx stdout format (ground truth, from live `wfb_ng` 25.5.1 + `../wfb-ng/src/rx.cpp`)

Each `wfb_rx` prints tab-separated IPC lines every `-l` ms:

- **`RX_ANT`** — per `(freq,mcs,bw)`/antenna:
  `<ts>\tRX_ANT\t<freq>:<mcs>:<bw>\t<ant_id>\t<count>:<rssi_min>:<rssi_avg>:<rssi_max>:<snr_min>:<snr_avg>:<snr_max>`
  e.g. `5952334\tRX_ANT\t5805:3:20\t0\t2:-58:-57:-57:25:25:26` → mcs=3, rssi_avg=-57 (last-field idx **2**), snr_avg=25 (idx **5**).
- **`PKT`** — per-instance window counters (cleared each interval):
  `<ts>\tPKT\t<all>:<all_bytes>:<dec_err>:<session>:<data>:<uniq>:<fec_rec>:<lost>:<bad>:<out>:<out_bytes>:<bursts_rec>:<holdoff>:<late_deadline>`
  e.g. `…\tPKT\t6:8568:0:0:6:3:0:0:0:3:4200:0:0:0` → **data=idx 4**, **fec_rec=idx 6**, **lost=idx 7**. (Older wfb-ng emits 11 fields; the needed indices 4/6/7 are identical — new fields are appended.)
- **`SESSION`** — on FEC-param change; ignore in Phase 1b.

Each probe `wfb_rx` receives ONE drone MCS (one MCS per `radio_port` in Phase 1a), so that instance's `RX_ANT` mcs key = its MCS, its `PKT` `data`/`lost` = that MCS's window, raw PER = `lost/(data+lost)` (FEC off ⇒ `fec_rec`≈0).

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `gs/fpvdgs/probe/__init__.py` | create | package marker |
| `gs/fpvdgs/probe/parser.py` | create | pure: parse `RX_ANT`/`PKT` lines + per-MCS EWMA aggregation |
| `gs/fpvdgs/probe/controller.py` | create | `ProbeController` (threaded asyncio; spawns/reads probe `wfb_rx`) |
| `gs/fpvdgs/schema.py` | modify | `probe` in `CONFIG_TOP_KEYS` + `_validate_probe` |
| `gs/etc/defaults.json` | modify | add `probe` block (disabled, basePort 50) |
| `gs/fpvdgs/status.py` | modify | `build_status(..., probe=None)` |
| `gs/fpvdgs/supervisor.py` | modify | build/start/stop `ProbeController`; `_probe_status()` |
| `gs/tests/unit/test_probe_parser.py` | create | parser + aggregator tests |
| `gs/tests/unit/test_probe_controller.py` | create | controller lifecycle + fake-spawner tests |
| `gs/tests/unit/test_probe_schema.py` | create | probe validation tests |

`basePort` = the wfb **radio_port** (must match the drone's `probe.basePort`); the probe `wfb_rx` forwards decoded packets to a throwaway localhost UDP port (we only consume stdout).

---

## Task 1: Probe config schema + validation

**Files:** Modify `gs/fpvdgs/schema.py`, `gs/etc/defaults.json`; Test `gs/tests/unit/test_probe_schema.py` (create)

- [ ] **Step 1: Write the failing test** — `gs/tests/unit/test_probe_schema.py`:

```python
import pytest
from fpvdgs import schema

def test_probe_in_top_keys():
    schema.validate_config_patch({"probe": {"enabled": True}})  # no raise

def test_probe_defaults_valid():
    schema.validate_effective({
        "link": {"width": 20, "region": "US", "channel": 161},
        "probe": {"enabled": False, "basePort": 50, "maxStreams": 4, "rxL": 50},
    })

@pytest.mark.parametrize("bad", [
    {"enabled": "yes"},
    {"enabled": True, "basePort": 0},
    {"enabled": True, "basePort": 70000},
    {"enabled": True, "maxStreams": 0},
    {"enabled": True, "rxL": -1},
])
def test_probe_rejects(bad):
    cfg = {"link": {"width": 20, "region": "US", "channel": 161}, "probe": bad}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)
```

- [ ] **Step 2: Run to verify it fails** — `cd gs && python -m pytest tests/unit/test_probe_schema.py -q` → FAIL (`probe` not in `CONFIG_TOP_KEYS`; no `_validate_probe`).

- [ ] **Step 3: Implement.** In `gs/fpvdgs/schema.py`: add `"probe"` to the `CONFIG_TOP_KEYS` set. Add the validator and call it from `validate_effective` (next to the `dynamicLink`/`pixelpilot` calls):

```python
def _validate_probe(probe: dict) -> None:
    if not isinstance(probe.get("enabled", False), bool):
        raise SchemaError("probe.enabled must be a bool")
    base_port = probe.get("basePort", 50)
    if isinstance(base_port, bool) or not isinstance(base_port, int) or not 1 <= base_port <= 255:
        raise SchemaError("probe.basePort must be an int in 1..255 (a wfb radio_port)")
    max_streams = probe.get("maxStreams", 4)
    if isinstance(max_streams, bool) or not isinstance(max_streams, int) or not 1 <= max_streams <= 8:
        raise SchemaError("probe.maxStreams must be an int in 1..8")
    if base_port + max_streams - 1 > 255:
        raise SchemaError("probe.basePort + maxStreams exceeds radio_port 255")
    rx_l = probe.get("rxL", 50)
    if isinstance(rx_l, bool) or not isinstance(rx_l, int) or not 1 <= rx_l <= 1000:
        raise SchemaError("probe.rxL must be an int in 1..1000 (ms)")
```

In `validate_effective`, after the `pixelpilot` check:
```python
    probe = cfg.get("probe")
    if probe is not None:
        _validate_probe(probe)
```

In `gs/etc/defaults.json`, add a top-level block:
```json
  "probe": { "enabled": false, "basePort": 50, "maxStreams": 4, "rxL": 50 }
```

- [ ] **Step 4: Run to verify it passes** — `python -m pytest tests/unit/test_probe_schema.py -q` → PASS. Then full `python -m pytest tests/ -q` → no regressions.

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/schema.py gs/etc/defaults.json gs/tests/unit/test_probe_schema.py
git commit -m "feat(gs/probe): probe config schema + validation"
```

---

## Task 2: RX_ANT/PKT parser + per-MCS aggregator

**Files:** Create `gs/fpvdgs/probe/__init__.py` (empty), `gs/fpvdgs/probe/parser.py`; Test `gs/tests/unit/test_probe_parser.py`

- [ ] **Step 1: Write the failing test** — `gs/tests/unit/test_probe_parser.py`:

```python
from fpvdgs.probe import parser

def test_parse_rx_ant():
    ev = parser.parse_line("5952334\tRX_ANT\t5805:3:20\t0\t2:-58:-57:-57:25:25:26")
    assert ev == ("RX_ANT", {"mcs": 3, "rssi": -57, "snr": 25})

def test_parse_pkt():
    ev = parser.parse_line("99\tPKT\t6:8568:0:0:6:3:0:0:0:3:4200:0:0:0")
    assert ev == ("PKT", {"data": 6, "fec_rec": 0, "lost": 0})

def test_parse_pkt_11field_legacy():
    ev = parser.parse_line("99\tPKT\t10:5000:0:0:8:8:1:1:0:8:4000")
    assert ev == ("PKT", {"data": 8, "fec_rec": 1, "lost": 1})

def test_parse_ignores_session_and_garbage():
    assert parser.parse_line("9\tSESSION\t0:1:1:1") is None
    assert parser.parse_line("not a stats line") is None

def test_aggregator_ewma_and_per():
    agg = parser.McsAggregator(alpha=0.5)
    agg.on_rx_ant(mcs=5, rssi=-60, snr=20)
    agg.on_pkt(mcs=5, data=90, lost=10)   # window PER = 10/100 = 0.10
    agg.on_pkt(mcs=5, data=98, lost=2)    # window PER = 2/100 = 0.02; EWMA -> 0.06
    snap = agg.snapshot()
    assert snap[5]["rssi"] == -60
    assert snap[5]["snr"] == 20
    assert abs(snap[5]["per"] - 0.06) < 1e-9
    assert snap[5]["windows"] == 2

def test_aggregator_blackout_window_is_full_loss():
    agg = parser.McsAggregator(alpha=1.0)
    agg.on_pkt(mcs=7, data=0, lost=0)     # nothing decoded this window
    assert agg.snapshot()[7]["per"] == 1.0
```

- [ ] **Step 2: Run to verify it fails** — `python -m pytest tests/unit/test_probe_parser.py -q` → FAIL (no module).

- [ ] **Step 3: Implement.** Create `gs/fpvdgs/probe/__init__.py` (empty file). Create `gs/fpvdgs/probe/parser.py`:

```python
"""Pure parsing of wfb_rx stdout IPC lines + per-MCS EWMA aggregation.

Line formats (tab-separated), per ../wfb-ng/src/rx.cpp dump_stats():
  RX_ANT: <ts>\tRX_ANT\t<freq>:<mcs>:<bw>\t<ant>\t<count>:<rssi_min>:<rssi_avg>:<rssi_max>:<snr_min>:<snr_avg>:<snr_max>
  PKT:    <ts>\tPKT\t<all>:<all_bytes>:<dec_err>:<session>:<data>:<uniq>:<fec_rec>:<lost>:<bad>:<out>:... (>=11 fields)
The needed PKT indices (data=4, fec_rec=6, lost=7) are stable across versions
(newer wfb-ng appends fields). FEC is off on the probe, so fec_rec≈0 and the raw
on-air PER for a window is lost/(data+lost).
"""
from __future__ import annotations


def parse_line(line: str):
    """Return ('RX_ANT', {mcs,rssi,snr}) | ('PKT', {data,fec_rec,lost}) | None."""
    cols = line.rstrip("\n").split("\t")
    if len(cols) < 3:
        return None
    kind = cols[1]
    try:
        if kind == "RX_ANT" and len(cols) >= 5:
            freq, mcs, bw = (int(x) for x in cols[2].split(":"))
            vals = [int(x) for x in cols[4].split(":")]
            if len(vals) < 7:
                return None
            # vals = [count, rssi_min, rssi_avg, rssi_max, snr_min, snr_avg, snr_max]
            return ("RX_ANT", {"mcs": mcs, "rssi": vals[2], "snr": vals[5]})
        if kind == "PKT":
            f = [int(x) for x in cols[2].split(":")]
            if len(f) < 8:
                return None
            return ("PKT", {"data": f[4], "fec_rec": f[6], "lost": f[7]})
    except ValueError:
        return None
    return None


class McsAggregator:
    """Per-MCS EWMA of raw on-air PER + latest RSSI/SNR.

    Each probe wfb_rx receives one MCS, so RX_ANT supplies the MCS label (and
    rssi/snr) and the following PKT lines supply that MCS's window data/lost.
    Callers route on_rx_ant/on_pkt with the mcs from the latest RX_ANT.
    """

    def __init__(self, alpha: float = 0.25):
        self.alpha = alpha
        self._m: dict[int, dict] = {}

    def _slot(self, mcs: int) -> dict:
        return self._m.setdefault(
            mcs, {"per": None, "rssi": None, "snr": None, "windows": 0})

    def on_rx_ant(self, mcs: int, rssi: int, snr: int) -> None:
        s = self._slot(mcs)
        s["rssi"], s["snr"] = rssi, snr

    def on_pkt(self, mcs: int, data: int, lost: int) -> None:
        s = self._slot(mcs)
        denom = data + lost
        win_per = (lost / denom) if denom > 0 else 1.0   # no decodes ⇒ blackout
        s["per"] = win_per if s["per"] is None else (
            self.alpha * win_per + (1 - self.alpha) * s["per"])
        s["windows"] += 1

    def snapshot(self) -> dict[int, dict]:
        return {mcs: dict(s) for mcs, s in self._m.items()}
```

- [ ] **Step 4: Run to verify it passes** — `python -m pytest tests/unit/test_probe_parser.py -q` → PASS.

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/probe/__init__.py gs/fpvdgs/probe/parser.py gs/tests/unit/test_probe_parser.py
git commit -m "feat(gs/probe): wfb_rx stdout parser + per-MCS EWMA aggregator"
```

---

## Task 3: ProbeController (spawn + read probe wfb_rx)

**Files:** Create `gs/fpvdgs/probe/controller.py`; Test `gs/tests/unit/test_probe_controller.py`

The controller mirrors `DynamicLinkController`'s threaded-asyncio lifecycle (`gs/fpvdgs/dynlink/controller.py`) — read it for the exact `_lock`/`_lifecycle`/`_thread_main`/`_started` scaffolding. A `spawn` callable is injected so tests don't launch real `wfb_rx`.

- [ ] **Step 1: Write the failing test** — `gs/tests/unit/test_probe_controller.py`:

```python
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
```

- [ ] **Step 2: Run to verify it fails** — `python -m pytest tests/unit/test_probe_controller.py -q` → FAIL (no module).

- [ ] **Step 3: Implement.** Create `gs/fpvdgs/probe/controller.py`. Mirror the `DynamicLinkController` threading scaffold (`gs/fpvdgs/dynlink/controller.py`): `_lock` (RLock) guards `_thread/_loop/_stop_event/_status`; `_lifecycle` (RLock) serializes start/stop; `start()` launches `_thread_main` and waits on `_started`; `_thread_main` makes a new event loop and runs `_run()`; `stop()` does `loop.call_soon_threadsafe(stop_event.set)` then joins. Specifics:

```python
"""GS-side probe measurement: spawn a FEC-off wfb_rx per probe radio_port,
parse each one's stdout for per-MCS PER/RSSI. Threaded asyncio, mirroring
DynamicLinkController. Observe-only; independent of the wfb-ng runner."""
from __future__ import annotations

import asyncio
import logging
import threading

from .parser import McsAggregator, parse_line

log = logging.getLogger(__name__)

WFB_RX = "/usr/bin/wfb_rx"


async def _default_spawn(cmd):
    return await asyncio.create_subprocess_exec(
        *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)


class ProbeController:
    def __init__(self, snapshot, *, spawn=None, ewma_alpha: float = 0.25):
        self._snap = dict(snapshot)
        self._spawn = spawn or _default_spawn   # cmd(list[str]) -> proc (await or sync in tests)
        self._alpha = ewma_alpha
        self._lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None
        self._started = threading.Event()
        self._agg = McsAggregator(alpha=ewma_alpha)
        self._status = {"running": False, "streams": 0}

    # ---- public, thread-safe ----
    def start(self):
        with self._lifecycle:
            with self._lock:
                if self._thread and self._thread.is_alive():
                    return
                self._started.clear()
                self._thread = threading.Thread(target=self._thread_main,
                                                name="probe-controller", daemon=True)
                self._thread.start()
            self._started.wait(timeout=5.0)

    def stop(self):
        with self._lifecycle:
            with self._lock:
                loop, stop, thread = self._loop, self._stop_event, self._thread
            if loop is not None and stop is not None:
                try:
                    loop.call_soon_threadsafe(stop.set)
                except RuntimeError:
                    pass
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=5.0)
            with self._lock:
                self._thread = None

    def set_config(self, snapshot):
        with self._lifecycle:
            running = self._thread is not None and self._thread.is_alive()
            if running:
                self.stop()
            with self._lock:
                self._snap = dict(snapshot)
                self._agg = McsAggregator(alpha=self._alpha)
            if running and snapshot.get("enabled"):
                self.start()

    def status(self):
        with self._lock:
            st = dict(self._status)
        st["enabled"] = bool(self._snap.get("enabled"))
        st["mcs"] = {str(m): v for m, v in self._agg.snapshot().items()}
        return st

    def _set(self, **kw):
        with self._lock:
            self._status.update(kw)

    # ---- loop thread ----
    def _thread_main(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        with self._lock:
            self._loop = loop
        try:
            loop.run_until_complete(self._run())
        except Exception:
            log.exception("probe controller loop crashed")
        finally:
            try:
                loop.close()
            finally:
                with self._lock:
                    self._loop = None
                    self._status.update(running=False)
                self._started.set()

    def _build_cmd(self, port: int, sink: int) -> list[str]:
        snap = self._snap
        return [WFB_RX, "-K", str(snap["key"]), "-i", str(snap["linkId"]),
                "-p", str(port), "-c", "127.0.0.1", "-u", str(sink),
                "-l", str(snap.get("rxL", 50)), *list(snap["wlans"])]

    async def _read_stream(self, proc):
        agg, cur_mcs = self._agg, None
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            ev = parse_line(line.decode("utf-8", "replace"))
            if ev is None:
                continue
            kind, d = ev
            if kind == "RX_ANT":
                cur_mcs = d["mcs"]
                with self._lock:
                    agg.on_rx_ant(d["mcs"], d["rssi"], d["snr"])
            elif kind == "PKT" and cur_mcs is not None:
                with self._lock:
                    agg.on_pkt(cur_mcs, d["data"], d["lost"])

    async def _run(self):
        self._stop_event = asyncio.Event()
        snap = self._snap
        procs, tasks = [], []
        if snap.get("enabled"):
            base, n = int(snap["basePort"]), int(snap.get("maxStreams", 4))
            for i in range(n):
                cmd = self._build_cmd(base + i, 7000 + i)   # 7000+i = throwaway sink (discarded)
                res = self._spawn(cmd)
                proc = await res if asyncio.iscoroutine(res) else res
                procs.append(proc)
                tasks.append(asyncio.ensure_future(self._read_stream(proc)))
            self._set(streams=len(procs))
        self._set(running=True)
        self._started.set()
        try:
            await self._stop_event.wait()
        finally:
            self._set(running=False, streams=0)
            for p in procs:
                try:
                    p.kill()
                except Exception:
                    pass
            for t in tasks:
                t.cancel()
            for p in procs:
                try:
                    await p.wait()
                except Exception:
                    pass
```

- [ ] **Step 4: Run to verify it passes** — `python -m pytest tests/unit/test_probe_controller.py -q` → PASS (3 cases). Then full `python -m pytest tests/ -q`.

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/probe/controller.py gs/tests/unit/test_probe_controller.py
git commit -m "feat(gs/probe): ProbeController spawns + measures probe wfb_rx"
```

---

## Task 4: Surface probe in /status

**Files:** Modify `gs/fpvdgs/status.py`; Test: append to `gs/tests/unit/test_status.py` (or create if absent)

- [ ] **Step 1: Write the failing test** — append to `gs/tests/unit/test_status.py`:

```python
from fpvdgs import status as status_mod

def test_build_status_includes_probe_when_present():
    drone_probe = {"reachable": True, "linkId": 7669206, "inSync": True}
    j = status_mod.build_status("vX", {"running": True}, {}, drone_probe,
                                probe={"enabled": True, "running": True,
                                       "streams": 2, "mcs": {"3": {"per": 0.0}}})
    assert j["probe"]["enabled"] is True
    assert j["probe"]["mcs"]["3"]["per"] == 0.0

def test_build_status_omits_probe_when_none():
    j = status_mod.build_status("vX", {"running": True}, {},
                                {"reachable": False, "linkId": 1, "inSync": None})
    assert "probe" not in j
```

- [ ] **Step 2: Run to verify it fails** — `python -m pytest tests/unit/test_status.py -q` → FAIL (`build_status` has no `probe` param).

- [ ] **Step 3: Implement.** In `gs/fpvdgs/status.py`, add a keyword param `probe: dict | None = None` to `build_status(...)`, and before `return out`:
```python
    if probe is not None:
        out["probe"] = probe
```

- [ ] **Step 4: Run to verify it passes** — `python -m pytest tests/unit/test_status.py -q` → PASS. Full `python -m pytest tests/ -q`.

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/status.py gs/tests/unit/test_status.py
git commit -m "feat(gs/probe): add probe block to /status"
```

---

## Task 5: Wire ProbeController into the App

**Files:** Modify `gs/fpvdgs/supervisor.py`; Test: append to `gs/tests/integration/test_supervisor_e2e.py` (mirror its existing build_app idiom; if testing build_app is awkward, a focused unit test of the wiring is acceptable — read the file first).

- [ ] **Step 1: Write the failing test.** READ `gs/fpvdgs/supervisor.py` (`build_app`, `App`, `status_fn`) and `gs/tests/integration/test_supervisor_e2e.py` for the harness. Add a test that builds the app with a probe-enabled config + a stub `wfb_rx` spawner and asserts `/status` (via `status_fn`/the API) contains a `probe` block with `enabled: true`. Match the existing e2e idiom; if `build_app` doesn't accept an injectable spawner, add one (`build_app(..., probe_spawn=None)` threaded into `ProbeController(spawn=probe_spawn)`), defaulting to the real spawner.

```python
def test_status_has_probe_block(tmp_path):
    # --- mirror the existing build_app harness in this file ---
    # write defaults+overlay enabling probe, build_app(..., probe_spawn=fake),
    # call the app's status_fn(), assert "probe" in it with enabled True.
    ...
    st = app.api.status_fn()   # or however the harness reads status
    assert st["probe"]["enabled"] is True
```

- [ ] **Step 2: Run to verify it fails** — FAIL (no probe in status / no ProbeController wired).

- [ ] **Step 3: Implement.** In `gs/fpvdgs/supervisor.py`:
- import: `from .probe.controller import ProbeController`
- in `build_app`, after the `dynlink = DynamicLinkController(...)` line, construct the probe snapshot + controller:
```python
    def _probe_snapshot(eff):
        p = dict(eff.get("probe", {}))
        p["key"] = "/etc/gs.key"
        p["linkId"] = eff.get("link", {}).get("linkId")
        p["wlans"] = resolve_wlans(eff)
        return p

    probe = ProbeController(_probe_snapshot(effective), spawn=probe_spawn)
```
  (add `probe_spawn=None` to `build_app`'s signature and pass it through; `ProbeController` defaults to the real spawner when `spawn=None`.)
- add `probe` to the `App(...)` constructor call + `App.__init__` params (store as `self.probe`).
- in `App.start()`:
```python
        if self.store.effective().get("probe", {}).get("enabled"):
            self.probe.start()
```
- in `App.shutdown()`: `self.probe.stop()` (before `self.runner.shutdown()`).
- add a status helper + pass it through:
```python
    def _probe_status():
        if not store.effective().get("probe", {}).get("enabled"):
            return {"enabled": False, "running": False}
        return probe.status()
```
  and in `status_fn()`'s `build_status(...)` call add `probe=_probe_status()`.

- [ ] **Step 4: Run to verify it passes** — the new test PASS; full `python -m pytest tests/ -q` green.

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/supervisor.py gs/tests/integration/test_supervisor_e2e.py
git commit -m "feat(gs/probe): wire ProbeController into the GS app + status"
```

---

## Task 6: Deploy + on-hardware smoke (needs live GS + drone)

**Files:** none (verification). Requires the drone (running the Phase-1a build) reachable and the GS reachable.

- [ ] **Step 1: Deploy the GS build** — `./deploy/gs/deploy.sh --host 10.18.0.1` → `[done]`.
- [ ] **Step 2: Enable the drone probe** (over `/air` or LAN): `PATCH http://<drone>:8080/config -d '{"probe":{"enabled":true,"mcsList":[3,5,7]}}'` + `POST /apply`.
- [ ] **Step 3: Enable GS probe + verify** — `PATCH http://10.18.0.1:8080/config -d '{"probe":{"enabled":true,"basePort":50,"maxStreams":4}}'` + `POST /apply`. Then `curl http://10.18.0.1:8080/status` → confirm a `probe` block with per-MCS `per`/`rssi`/`snr` for mcs 3/5/7, and confirm the **video** stream is unaffected (no runner bounce: `runner` state unchanged, no video glitch). Cross-check the per-MCS PER against the throwaway MVP rig if desired.
- [ ] **Step 4: Disable both** (`probe.enabled=false` on GS + drone) and confirm the probe `wfb_rx` are gone and video is healthy.

---

## Self-Review

**Spec coverage (§4.3 GS measurement / §5 Phase 1b):**
- "probe wfb_rx per radio_port, FEC-off, raw PER" → Tasks 2–3 (parser PER from `PKT lost/data`; controller spawns per `basePort+i`).
- "per-MCS RSSI/SNR from RX_ANT, MCS labeled from radiotap not a static list" → Task 2 (`RX_ANT` → mcs/rssi/snr) + Task 3 (`cur_mcs` from `RX_ANT`, used to route `PKT`).
- "EWMA-smoothed, surfaced in status, observe-only, hot-toggleable, independent of runner" → Tasks 2 (EWMA), 4/5 (status + App start/stop), controller is standalone subprocesses (no runner bounce).
- GS config = port range not mcsList → Task 1 (`basePort`+`maxStreams`).
- **Deferred (Phase 2):** per-MCS attribution when a port's MCS *changes over time* (Phase 1a fixes one MCS per port, so per-instance `PKT` = that MCS; Phase 2's adaptive remap will need RX_ANT-gated PKT routing — already structured via `cur_mcs`, but unvalidated for mid-stream MCS change).

**Placeholder scan:** Tasks 5's test references the existing `test_supervisor_e2e.py` harness for the build_app idiom (its construction is non-trivial/codebase-specific) and notes the possible `probe_spawn` injection — the implementer reads that file for the exact seam. Every code step otherwise contains complete code.

**Type consistency:** `parse_line` returns `("RX_ANT", {mcs,rssi,snr})` / `("PKT", {data,fec_rec,lost})` / `None` — consumed exactly so in Task 3 `_read_stream`. `McsAggregator.on_rx_ant(mcs,rssi,snr)` / `on_pkt(mcs,data,lost)` / `snapshot()` signatures match between Task 2 (def) and Task 3 (use). `ProbeController(snapshot, *, spawn=, ewma_alpha=)` + `.start/.stop/.status/.set_config` consistent across Task 3 and Task 5. Status shape `{enabled, running, streams, mcs:{<mcs str>:{per,rssi,snr,windows}}}` consistent Task 3 → 4 → 5.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-06-probe-link-gs-phase1b.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — fresh subagent per task, review between tasks.

**2. Inline Execution** — execute here with checkpoints.

**Which approach?**
