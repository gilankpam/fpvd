# Flight-Bounded Logs (link-gap roll + DVR dir) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Segment the GS flight log per *flight* — roll a new JSONL file when the video link returns healthy after being gone longer than `flight_gap_s` (default 15 s) — and relocate the logs to `/media/dvr/log/dynamic-link/`.

**Architecture:** `Policy.tick` already computes per-tick link health (`signals.link_starved_w`) and now tracks the monotonic time of the last *healthy* tick; on the first healthy tick after a gap > `flight_gap_s` it calls a new `FlightLog.roll()` (close current file, open a fresh one) before writing. `flight_gap_s` and the new `dir` default are config knobs under `tuning.learned_prior.flightlog`. The learned-prior curve is untouched (stays on `/etc/fpvd/learned/`); only the log file segments.

**Tech Stack:** Python 3.13 + pytest (GS). No new deps.

**Spec:** `docs/superpowers/specs/2026-06-08-flight-bounded-logs-design.md`.

**Test command:** `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/ -q` (use the venv). One file: `… tests/unit/test_dl_flightlog.py -q`. **Baseline: 244 passed.** Git from repo root `/home/gilankpam/Projects/drone/fpvd`.

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `gs/fpvdgs/dynlink/flightlog.py` | modify | `FlightLogConfig`: `dir` default → `/media/dvr/log/dynamic-link/`, add `flight_gap_s: float = 15.0`. `FlightLog`: extract `_open(start_ms)`, add `roll()`. |
| `gs/fpvdgs/dynlink/config_build.py` | modify | parse `flightlog.flight_gap_s` (default 15.0) + new `dir` default. |
| `gs/fpvdgs/dynlink/policy.py` | modify | `Policy.__init__`: `self._last_healthy_mono = None`. `Policy.tick`: detect link-gap recovery and call `self.flightlog.roll()` before `self.flightlog.write(...)`. |
| `gs/tests/unit/test_dl_flightlog.py` | modify | `roll()` unit tests. |
| `gs/tests/unit/test_dl_config_build.py` | modify | assert `flight_gap_s` + `dir` defaults/overrides. |
| `gs/tests/unit/test_dl_policy_learned.py` | modify | Policy gap-roll integration tests (deterministic monotonic clock). |

Order: Task 1 (config) → Task 2 (`roll()`) → Task 3 (Policy wiring). Each task is one commit; the GS suite stays green at every commit.

---

## Task 1: Config — DVR dir default + `flight_gap_s`

**Files:** Modify `gs/fpvdgs/dynlink/flightlog.py`, `gs/fpvdgs/dynlink/config_build.py`; Test `gs/tests/unit/test_dl_config_build.py`, `gs/tests/unit/test_dl_flightlog.py`.

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_dl_config_build.py`:
```python
def test_flightlog_dir_default_is_dvr_and_gap_default():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {}})
    assert cfg.flightlog.dir == "/media/dvr/log/dynamic-link/"
    assert cfg.flightlog.flight_gap_s == 15.0


def test_flightlog_flight_gap_s_parsed():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {"learned_prior": {"flightlog": {
        "flight_gap_s": 8.0, "dir": "/tmp/fl"}}}})
    assert cfg.flightlog.flight_gap_s == 8.0
    assert cfg.flightlog.dir == "/tmp/fl"
```

Append to `gs/tests/unit/test_dl_flightlog.py`:
```python
def test_config_defaults_dvr_dir_and_gap():
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    c = FlightLogConfig()
    assert c.dir == "/media/dvr/log/dynamic-link/"
    assert c.flight_gap_s == 15.0
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog.py::test_config_defaults_dvr_dir_and_gap tests/unit/test_dl_config_build.py -k flightlog -q`
Expected: FAIL — `dir` is still `/etc/fpvd/flightlog`, no `flight_gap_s` attribute.

- [ ] **Step 3a: Update `FlightLogConfig`** — in `gs/fpvdgs/dynlink/flightlog.py`, change the dataclass to:
```python
@dataclass
class FlightLogConfig:
    enabled: bool = True
    dir: str = "/media/dvr/log/dynamic-link/"
    max_files: int = 8
    max_mb: float = 4.0
    flight_gap_s: float = 15.0   # link gone > this (s) => next healthy tick = new flight file
```

- [ ] **Step 3b: Parse it in `config_build.py`** — in `_build_policy_config`, the `flightlog` block currently builds `FlightLogConfig(enabled=..., dir=..., max_files=..., max_mb=...)`. Update the `dir` default and add `flight_gap_s`:
```python
    flightlog = FlightLogConfig(
        enabled=bool(fl_raw.get("enabled", True)),
        dir=str(fl_raw.get("dir", "/media/dvr/log/dynamic-link/")),
        max_files=int(fl_raw.get("max_files", 8)),
        max_mb=float(fl_raw.get("max_mb", 4.0)),
        flight_gap_s=float(fl_raw.get("flight_gap_s", 15.0)),
    )
```
(Read the actual `flightlog = FlightLogConfig(...)` block in `config_build.py` and edit those lines in place — only the `dir` default changes and the `flight_gap_s` line is added.)

- [ ] **Step 4: Run to verify they pass**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog.py tests/unit/test_dl_config_build.py -q`
Expected: PASS. Then full suite `.venv/bin/python -m pytest tests/ -q 2>&1 | tail -3` → 0 failed. **If any existing test asserted the old `/etc/fpvd/flightlog` default, update it to `/media/dvr/log/dynamic-link/`.**

- [ ] **Step 5: Commit**
```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/flightlog.py gs/fpvdgs/dynlink/config_build.py gs/tests/unit/test_dl_flightlog.py gs/tests/unit/test_dl_config_build.py
git commit -m "feat(gs/dynlink): flightlog DVR dir default + flight_gap_s knob"
```

---

## Task 2: `FlightLog.roll()` (+ `_open` refactor)

**Files:** Modify `gs/fpvdgs/dynlink/flightlog.py`; Test `gs/tests/unit/test_dl_flightlog.py`.

`roll()` ends the current flight file and begins a new one (fresh monotonic-ms name), pruning to `max_files`. Refactor the constructor's open logic into `_open(start_ms)` so both `__init__` and `roll()` share it.

- [ ] **Step 1: Write the failing tests** — append to `gs/tests/unit/test_dl_flightlog.py`:
```python
def test_roll_starts_new_file_and_both_persist(tmp_path):
    import time
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=8), start_ms=1000)
    fl.write({"ts": 1.0, "mcs": 5})
    fl.roll()
    fl.write({"ts": 9.0, "mcs": 2})
    fl.close()
    files = sorted(tmp_path.glob("*.jsonl"))
    assert len(files) == 2                      # rolled into a second file
    # the post-roll record is in the newest file, the pre-roll record in the older
    newest = max(files, key=lambda p: p.stat().st_mtime)
    assert '"mcs":2' in newest.read_text()


def test_roll_prunes_to_max_files(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=2), start_ms=1000)
    fl.write({"ts": 1.0})
    fl.roll(); fl.write({"ts": 2.0})
    fl.roll(); fl.write({"ts": 3.0})            # 3 flights, cap 2
    fl.close()
    assert len(list(tmp_path.glob("*.jsonl"))) == 2


def test_roll_is_noop_when_disabled(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), enabled=False), start_ms=1)
    fl.roll()
    fl.write({"ts": 1.0})
    fl.close()
    assert list(tmp_path.iterdir()) == []
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog.py -k roll -q`
Expected: FAIL — `AttributeError: 'FlightLog' object has no attribute 'roll'`.

- [ ] **Step 3: Implement** — in `gs/fpvdgs/dynlink/flightlog.py`:

3a. Add `import time` to the imports (alongside `import json`, `import os`).

3b. Replace the `FlightLog.__init__` body's open logic with a call to a new `_open` helper, and add `roll()`. The current `__init__` is:
```python
    def __init__(self, cfg: FlightLogConfig, *, start_ms: int) -> None:
        self.cfg = cfg
        self._fh = None
        self._bytes = 0
        self._max_bytes = int(cfg.max_mb * 1024 * 1024)
        if not cfg.enabled:
            return
        try:
            os.makedirs(cfg.dir, exist_ok=True)
            self._path = os.path.join(cfg.dir, f"{start_ms}.jsonl")
            self._fh = open(self._path, "w")
        except OSError as e:
            log.warning("flightlog: open failed in %s: %s", cfg.dir, e)
            self._fh = None
```
Change it to:
```python
    def __init__(self, cfg: FlightLogConfig, *, start_ms: int) -> None:
        self.cfg = cfg
        self._fh = None
        self._bytes = 0
        self._max_bytes = int(cfg.max_mb * 1024 * 1024)
        self._open(start_ms)

    def _open(self, start_ms: int) -> None:
        if not self.cfg.enabled:
            return
        try:
            os.makedirs(self.cfg.dir, exist_ok=True)
            self._path = os.path.join(self.cfg.dir, f"{start_ms}.jsonl")
            self._fh = open(self._path, "w")
            self._bytes = 0
        except OSError as e:
            log.warning("flightlog: open failed in %s: %s", self.cfg.dir, e)
            self._fh = None
```
And add `roll()` (place it after `close()`):
```python
    def roll(self) -> None:
        """End the current flight file and begin a new one (a new flight).
        No-op if disabled. Re-attempts the open even if the previous one
        failed (e.g. the DVR mount came back)."""
        if not self.cfg.enabled:
            return
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError:
                pass
            self._fh = None
        self._prune()
        self._open(int(time.monotonic() * 1000))
```
(`close()`, `write()`, `_prune()` are unchanged. The `max_files=0` guard already in `_prune` still holds.)

- [ ] **Step 4: Run to verify they pass**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog.py -q`
Expected: PASS (existing + 3 new). Full suite green.

- [ ] **Step 5: Commit**
```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/flightlog.py gs/tests/unit/test_dl_flightlog.py
git commit -m "feat(gs/dynlink): FlightLog.roll() — new file per flight"
```

---

## Task 3: Policy — roll on link-gap recovery

**Files:** Modify `gs/fpvdgs/dynlink/policy.py`; Test `gs/tests/unit/test_dl_policy_learned.py`.

`Policy` tracks the monotonic time of the last *healthy* tick; on the first healthy tick after a gap > `flight_gap_s`, it rolls the flight log before writing this tick's record (so the post-gap record lands in the new file). Uses raw `signals.link_starved_w` as the health signal and `time.monotonic()` (already imported in policy.py).

- [ ] **Step 1: Write the failing tests** — append to `gs/tests/unit/test_dl_policy_learned.py`:
```python
def _cfg_fl(tmp_path, flight_gap_s=15.0):
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path / "lp")),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl"), flight_gap_s=flight_gap_s),
    )


def _sig_starved(starved, ts=1.0, rssi=-55.0):
    return Signals(rssi=rssi, residual_loss_w=0.0, fec_work=0.0,
                   link_starved_w=starved, timestamp=ts)


def test_flight_rolls_on_link_gap_recovery(tmp_path, monkeypatch):
    from fpvdgs.dynlink import policy as policy_mod, flightlog as fl_mod
    clock = {"t": 1000.0}
    monkeypatch.setattr(policy_mod.time, "monotonic", lambda: clock["t"])
    monkeypatch.setattr(fl_mod.time, "monotonic", lambda: clock["t"])
    p = Policy(_cfg_fl(tmp_path, flight_gap_s=15.0), _profile())
    p.tick(_sig_starved(False, ts=1.0)); clock["t"] += 0.1     # baseline (no roll on 1st)
    p.tick(_sig_starved(False, ts=1.1))
    clock["t"] += 20.0                                          # link gone 20 s
    p.tick(_sig_starved(True, ts=2.0))                         # starved: baseline frozen
    p.tick(_sig_starved(False, ts=3.0))                        # healthy: 20 s > 15 s -> ROLL
    p.close()
    assert len(list((tmp_path / "fl").glob("*.jsonl"))) == 2


def test_brief_gap_does_not_roll(tmp_path, monkeypatch):
    from fpvdgs.dynlink import policy as policy_mod, flightlog as fl_mod
    clock = {"t": 1000.0}
    monkeypatch.setattr(policy_mod.time, "monotonic", lambda: clock["t"])
    monkeypatch.setattr(fl_mod.time, "monotonic", lambda: clock["t"])
    p = Policy(_cfg_fl(tmp_path, flight_gap_s=15.0), _profile())
    p.tick(_sig_starved(False, ts=1.0)); clock["t"] += 5.0     # only 5 s gap
    p.tick(_sig_starved(True, ts=2.0))
    p.tick(_sig_starved(False, ts=3.0))                        # 5 s < 15 s -> no roll
    p.close()
    assert len(list((tmp_path / "fl").glob("*.jsonl"))) == 1


def test_first_healthy_tick_does_not_roll(tmp_path, monkeypatch):
    from fpvdgs.dynlink import policy as policy_mod, flightlog as fl_mod
    clock = {"t": 9999.0}     # large value: would exceed any gap if baseline weren't None
    monkeypatch.setattr(policy_mod.time, "monotonic", lambda: clock["t"])
    monkeypatch.setattr(fl_mod.time, "monotonic", lambda: clock["t"])
    p = Policy(_cfg_fl(tmp_path, flight_gap_s=15.0), _profile())
    p.tick(_sig_starved(False, ts=1.0))                        # 1st healthy: None baseline -> no roll
    p.close()
    assert len(list((tmp_path / "fl").glob("*.jsonl"))) == 1
```
(`_profile()` and `Signals`/`Policy`/`PolicyConfig` imports already exist at the top of this test file from Phase 4.)

- [ ] **Step 2: Run to verify they fail**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py -k "flight or gap or first_healthy" -q`
Expected: FAIL — no roll happens (single file) because the gap-detection isn't wired yet.

- [ ] **Step 3: Implement** — in `gs/fpvdgs/dynlink/policy.py`:

3a. In `Policy.__init__`, add (next to the other Phase-4 fields, after `self._predict_demote_count = 0`):
```python
        self._last_healthy_mono = None   # monotonic ts of last non-starved tick (flight-gap roll)
```

3b. In `Policy.tick`, immediately BEFORE the `self.flightlog.write({...})` call (after `reason` is composed), add the gap-detection roll:
```python
        # Flight-boundary roll: a new flight = the link returning healthy after
        # being gone (starved) longer than flight_gap_s. Monotonic time so the
        # unreliable GS wall-clock can't break it; raw link_starved_w as health.
        if not signals.link_starved_w:
            _now_mono = time.monotonic()
            if (self._last_healthy_mono is not None
                    and (_now_mono - self._last_healthy_mono)
                    > self.cfg.flightlog.flight_gap_s):
                self.flightlog.roll()
            self._last_healthy_mono = _now_mono
        self.flightlog.write({
            ...   # UNCHANGED existing record
        })
```
(Place the block so the `roll()` happens before the existing `self.flightlog.write(...)`, so the post-gap record lands in the new file. Do not change the `write(...)` record contents or anything else in `tick()`.)

- [ ] **Step 4: Run to verify they pass**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py tests/unit/test_dl_flightlog.py -q`
Expected: PASS (3 new + existing). Full suite `.venv/bin/python -m pytest tests/ -q 2>&1 | tail -3` → 0 failed.

- [ ] **Step 5: Commit**
```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_policy_learned.py
git commit -m "feat(gs/dynlink): roll flight log on link-gap recovery (one file per flight)"
```

---

## Task 4: Deploy note (operator-run, GS-only)

**Files:** none. GS-only, no drone/wire change — `./deploy/gs/deploy.sh --host 10.18.0.1`. At runtime `FlightLog` `os.makedirs` creates `/media/dvr/log/dynamic-link/` on the DVR mount; if `/media/dvr` is not mounted, logging no-ops with a warning (the link + learned curve are unaffected). Hand the deploy to the operator. Verify: leave dynamicLink enabled, fly + land + fly again (or simulate a >15 s link drop), and confirm two `*.jsonl` files appear under `/media/dvr/log/dynamic-link/`, one per flight; `pull` + `flightlog_analyze.py` each.

---

## Self-Review

**Spec coverage (`2026-06-08-flight-bounded-logs-design.md`):**
- §2 `flight_gap_s` default 15 s + `dir` → `/media/dvr/log/dynamic-link/` → Task 1. ✓
- §3 detection (monotonic gap since last healthy tick; roll on first healthy after gap > T; `None` baseline; raw `link_starved_w`) → Task 3. ✓
- §4 `FlightLog.roll()` (close+reopen, prune to max_files, disabled no-op, re-attempt open) → Task 2. ✓
- §5 learned curve untouched → no task touches `learned_prior` persistence (the `_cfg_fl` test helper puts it under a separate tmp dir; production stays `/etc/fpvd/learned/`). ✓
- §6 config delta + `/media/dvr` unmounted → graceful no-op → Task 1 (config) + the existing `_open` OSError path reused by Task 2. ✓
- §7 testing (roll unit; Policy gap-roll with deterministic monotonic; brief-gap-no-roll; first-tick-no-roll; config defaults; regression) → Tasks 1–3. ✓

**Placeholder scan:** every code step shows the full edit; the one `write({...})` ellipsis is explicitly marked UNCHANGED with the instruction not to alter it. No TBDs.

**Type/name consistency:** `FlightLogConfig.flight_gap_s` (Task 1) ↔ `self.cfg.flightlog.flight_gap_s` read in Policy (Task 3); `FlightLog.roll()` (Task 2) ↔ `self.flightlog.roll()` call (Task 3); `_open(start_ms)` shared by `__init__` + `roll()`; `signals.link_starved_w` is the existing per-window field; `time` is already imported in policy.py and added to flightlog.py (Task 2 step 3a).

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-08-flight-bounded-logs.md`.** Two execution options:

1. **Subagent-Driven (recommended)** — fresh subagent per task, spec + quality review between. GS-only; keep the suite green at every commit (Tasks 1–3 each commit; Task 4 is the operator deploy — STOP before it unless told to deploy).
2. **Inline Execution** — execute via executing-plans with checkpoints.
