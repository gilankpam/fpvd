# Phase 2 — Probe-Driven MCS Selector Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the GS SNR-floor MCS selector with a probe-driven one — promote on a clean `current+1` probe (EWMA + debounce), keep the existing reactive Channel-B demote, run the video at the ceiling. GS-only; no drone/wire/HELLO change.

**Architecture:** `LeadingSelector.select()` (in `fpvdgs/dynlink/policy.py`) is rewritten from SNR-margin/hysteresis to: emergency demote (kept `_emergency_active`) + probe-driven single-step promote. `Policy.tick()` receives the probe snapshot (from `ProbeController.status()`) per tick; the `DynamicLinkController` is given a `probe_status` callable, wired from the supervisor. The SNR machinery (margin/hysteresis/slope/confidence + the SNR fields in `SignalAggregator`) is removed; RSSI stays only as a cold-start hint. Bitrate/FEC/wire/predictor/Channel-B are untouched.

**Tech Stack:** Python 3.13, pytest. GS is `fpvdgs`.

**Spec:** `docs/superpowers/specs/2026-06-07-probe-driven-mcs-selector-design.md`.

**Run tests:** `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/ -q` (one file: `.venv/bin/python -m pytest tests/unit/test_dl_policy_leading.py -q`). pyproject sets `pythonpath=["."]`, `testpaths=["tests"]`.

**Probe snapshot shape** (`ProbeController.status()`, after Task 1): `{"running": bool, "streams": int, "mcs": {"<mcs>": {"per": float|None, "rssi": int|None, "snr": int|None, "windows": int, "ageMs": float|None}}}`. `ageMs` = wall-clock ms since that rung was last updated (added in Task 1); `None` if never seen.

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `fpvdgs/probe/controller.py` | modify | add per-MCS `ageMs` freshness to `status()` |
| `fpvdgs/dynlink/policy.py` | modify | `GateConfig` knob swap; rewrite `LeadingSelector.select()` (probe promote + emergency demote); `Policy.__init__`/`tick()` take + pass the probe snapshot; cold-start from RSSI; delete SNR helpers |
| `fpvdgs/dynlink/signals.py` | modify | drop SNR fields/EWMA/slope from `Signals`/`consume()` (keep rssi, loss, fec, starvation) |
| `fpvdgs/dynlink/config_build.py` | modify | parse the new probe knobs; deprecate the `snr_*` gate knobs |
| `fpvdgs/dynlink/controller.py` | modify | `__init__(probe_status=None)`; pass the snapshot into `Policy.tick()` |
| `fpvdgs/supervisor.py` | modify | construct `probe_ctrl` before `dynlink`; pass `probe_status=probe_ctrl.status` |
| `tests/unit/test_probe_controller.py` | modify | assert `ageMs` freshness |
| `tests/unit/test_dl_policy_leading.py` | modify | replace SNR-mechanism tests with probe promote/demote/cold-start tests |
| `tests/unit/test_dl_signals.py` | modify | remove the snr/snr_slope tests |
| `tests/unit/test_dl_config_build.py` | modify | probe-knob parse + snr-deprecation |
| `tests/unit/test_dl_controller.py` | modify | controller forwards probe snapshot into the policy |

Execute in order: Task 1 (freshness) → 2 (config knobs) → 3 (signals SNR removal) → 4 (selector rewrite) → 5 (wiring) → 6 (cold-start). Each is a self-contained commit.

---

## Task 1: Per-MCS freshness (`ageMs`) on probe status

**Files:** Modify `fpvdgs/probe/controller.py`; Test `tests/unit/test_probe_controller.py`

The selector needs to reject stale probe rungs. `status()` currently exposes only a `windows` count (not wall-clock). Add a per-MCS last-update monotonic timestamp in the controller (keeps `parser.py` pure) and surface `ageMs`.

- [ ] **Step 1: Write the failing test** — append to `tests/unit/test_probe_controller.py`:

```python
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
```

- [ ] **Step 2: Run to verify it fails** — `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_probe_controller.py::test_status_exposes_age_ms_freshness -q`
Expected: FAIL — `KeyError: 'ageMs'`.

- [ ] **Step 3: Implement.** In `fpvdgs/probe/controller.py`:

Add `import time` (top, with the other imports). In `__init__`, after `self._agg = McsAggregator(...)`, add:
```python
        self._last_update = {}   # mcs(int) -> monotonic seconds of last sample
```
In `_read_stream`, stamp the time whenever a line updates an mcs. Replace the existing `RX_ANT`/`PKT` handling block with:
```python
            kind, d = ev
            if kind == "RX_ANT":
                cur_mcs = d["mcs"]
                with self._lock:
                    self._agg.on_rx_ant(d["mcs"], d["rssi"], d["snr"])
                    self._last_update[d["mcs"]] = time.monotonic()
            elif kind == "PKT" and cur_mcs is not None:
                with self._lock:
                    self._agg.on_pkt(cur_mcs, d["data"], d["lost"])
                    self._last_update[cur_mcs] = time.monotonic()
```
In `status()`, add `ageMs` to each mcs entry:
```python
    def status(self):
        now = time.monotonic()
        with self._lock:
            st = dict(self._status)
            snap = self._agg.snapshot()
            last = dict(self._last_update)
        mcs = {}
        for m, v in snap.items():
            entry = dict(v)
            t = last.get(m)
            entry["ageMs"] = None if t is None else (now - t) * 1000.0
            mcs[m] = entry
        st["mcs"] = {str(m): v for m, v in mcs.items()}
        return st
```

- [ ] **Step 4: Run to verify it passes** — `.venv/bin/python -m pytest tests/unit/test_probe_controller.py -q`
Expected: PASS (all, including the existing ones — the other mcs entries now also carry `ageMs`).

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/probe/controller.py gs/tests/unit/test_probe_controller.py
git commit -m "feat(gs/probe): per-MCS ageMs freshness on probe status"
```

---

## Task 2: `GateConfig` knob swap + config parsing

**Files:** Modify `fpvdgs/dynlink/policy.py`, `fpvdgs/dynlink/config_build.py`; Test `tests/unit/test_dl_config_build.py`

Replace the SNR gate knobs with the probe-selector knobs. Keep the emergency + bounds knobs.

- [ ] **Step 1: Write the failing test** — append to `tests/unit/test_dl_config_build.py`:

```python
def test_gate_parses_probe_knobs():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {"gate": {
        "probe_viable_threshold": 0.97,
        "probe_freshness_ms": 400,
        "promote_debounce_windows": 2,
        "video_demote_per": 0.04,
    }}})
    g = cfg.gate
    assert g.probe_viable_threshold == 0.97
    assert g.probe_freshness_ms == 400
    assert g.promote_debounce_windows == 2
    assert g.video_demote_per == 0.04
    # emergency + bounds kept
    assert g.emergency_loss_rate == 0.05 and g.max_mcs == 7
```

(Adapt the `build_policy_config` call to the real entry point used by the existing tests in this file — check the top of `test_dl_config_build.py`; it may use `make_dl_snapshot`/`_build_policy_config`. Match it.)

- [ ] **Step 2: Run to verify it fails** — `.venv/bin/python -m pytest tests/unit/test_dl_config_build.py::test_gate_parses_probe_knobs -q`
Expected: FAIL — `GateConfig` has no `probe_viable_threshold`.

- [ ] **Step 3: Implement.** In `fpvdgs/dynlink/policy.py`, replace the `GateConfig` dataclass body (the SNR fields) with:
```python
@dataclass
class GateConfig:
    """Probe-driven promote + emergency (Channel-B) demote.

    Promote: the `current+1` probe rung must read clean (EWMA success
    >= probe_viable_threshold) and fresh (within probe_freshness_ms) for
    promote_debounce_windows consecutive ticks. Demote: the kept Channel-B
    emergency (loss/fec/starvation) plus a video on-air PER breach
    (video_demote_per on (lost+fec_rec)/(out+lost)).
    """
    # Probe-driven promote
    probe_viable_threshold: float = 0.99   # min EWMA success (1 - per) to climb
    probe_freshness_ms: float = 500.0      # max age of the probed rung's sample
    promote_debounce_windows: int = 3      # consecutive clean ticks before a climb
    # Reactive demote
    video_demote_per: float = 0.05         # (lost+fec_rec)/(out+lost) demote breach
    emergency_loss_rate: float = 0.05
    emergency_fec_pressure: float = 0.80
    # MCS bounds
    max_mcs: int = 7
    max_mcs_step_up: int = 1
```

In `fpvdgs/dynlink/config_build.py`, replace the `GateConfig(...)` construction (the `snr_*`/`hysteresis_*` lines) with:
```python
    gate = GateConfig(
        probe_viable_threshold=float(gate_raw.get("probe_viable_threshold", 0.99)),
        probe_freshness_ms=float(gate_raw.get("probe_freshness_ms", 500.0)),
        promote_debounce_windows=int(gate_raw.get("promote_debounce_windows", 3)),
        video_demote_per=float(gate_raw.get("video_demote_per", 0.05)),
        emergency_loss_rate=float(gate_raw.get("emergency_loss_rate", 0.05)),
        emergency_fec_pressure=float(gate_raw.get("emergency_fec_pressure", 0.80)),
        max_mcs=int(gate_raw.get("max_mcs", 7)),
        max_mcs_step_up=int(gate_raw.get("max_mcs_step_up", 1)),
    )
```
Add the now-removed gate knobs to the deprecated-key warning. Extend `_DEPRECATED_LEADING_KEYS` (or add a gate-level equivalent next to it) with a warn for any of: `snr_ema_alpha`, `snr_slope_alpha`, `snr_predict_horizon_ticks`, `snr_safety_margin`, `loss_margin_weight`, `fec_margin_weight`, `hysteresis_up_db`, `hysteresis_down_db` present in `gate_raw`. Mirror the existing warn pattern:
```python
    _DEPRECATED_GATE_KEYS = {
        "snr_ema_alpha", "snr_slope_alpha", "snr_predict_horizon_ticks",
        "snr_safety_margin", "loss_margin_weight", "fec_margin_weight",
        "hysteresis_up_db", "hysteresis_down_db",
    }
    dep_gate = sorted(k for k in _DEPRECATED_GATE_KEYS if k in gate_raw)
    if dep_gate:
        log.warning("gate has deprecated SNR knobs (ignored): %s. "
                    "MCS is now probe-driven.", ", ".join(dep_gate))
```
(Place `_DEPRECATED_GATE_KEYS` at module scope next to `_DEPRECATED_LEADING_KEYS`; put the warn inside `_build_policy_config` after `gate_raw` is read.) Also, in `_build_aggregator` (config_build.py ~line 295-307), remove the `ewma_alpha_snr_slope=gate.get("snr_slope_alpha", 0.3)` argument (the aggregator no longer takes it — see Task 3).

- [ ] **Step 4: Run to verify it passes** — `.venv/bin/python -m pytest tests/unit/test_dl_config_build.py -q`
Expected: PASS. (Update any existing config-build test that asserted on the removed `snr_*` gate fields — replace those assertions with the probe-knob ones or delete them.)

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/dynlink/policy.py gs/fpvdgs/dynlink/config_build.py gs/tests/unit/test_dl_config_build.py
git commit -m "feat(gs/dynlink): probe-selector gate knobs; deprecate SNR gate knobs"
```

---

## Task 3: Remove SNR from `SignalAggregator`

**Files:** Modify `fpvdgs/dynlink/signals.py`; Test `tests/unit/test_dl_signals.py`

Drop the SNR raw aggregation, the smoothed `snr`, and `snr_slope` (and the `ewma_alpha_snr_slope`/`_prev_snr` state). Keep rssi, residual_loss, fec_work, starvation, burst/holdoff (trailing loop).

- [ ] **Step 1: Update the tests first.** In `tests/unit/test_dl_signals.py`, DELETE the snr-slope tests (`test_snr_slope_initialises_zero_on_first_window`, `test_snr_slope_tracks_per_tick_delta_with_alpha`, `test_snr_slope_stable_under_constant_snr`, `test_snr_slope_tracks_negative_trend`) and `test_snr_max_is_max_of_avgs_across_antennas`; in `test_ewma_smoothes_rssi_max_not_min` remove the `assert s.snr == ...` line (keep the rssi assertions). Add a guard test:
```python
def test_signals_has_no_snr_fields():
    from fpvdgs.dynlink.signals import Signals
    s = Signals()
    assert not hasattr(s, "snr") and not hasattr(s, "snr_slope")
    assert not hasattr(s, "snr_max_w")
```

- [ ] **Step 2: Run to verify it fails** — `.venv/bin/python -m pytest tests/unit/test_dl_signals.py::test_signals_has_no_snr_fields -q`
Expected: FAIL — `Signals` still has `snr`.

- [ ] **Step 3: Implement.** In `fpvdgs/dynlink/signals.py`:
- From the `Signals` dataclass remove: `snr_min_w`, `snr_avg_w`, `snr_max_w`, `snr`, `snr_slope`. Keep `rssi_*`, `rssi`, `residual_loss_w`, `fec_work_rate_w`, `fec_work`, `packet_rate_w`, `link_starved_w`, burst/holdoff/late, `session`, `timestamp`, `windows_seen`, `ant_count`.
- In `SignalAggregator.__init__`, remove the `ewma_alpha_snr_slope` parameter and the `self._prev_snr = None` field (and drop `ewma_alpha_snr_slope` from its signature — update `build_aggregator` in config_build per Task 2).
- In `consume()`: remove the `snr_mins`/`snr_avgs`/`s.snr_min_w`/`s.snr_avg_w`/`s.snr_max_w` lines from the antenna block (keep the rssi ones); remove the `s.snr` EWMA (the `if s.snr_max_w is not None: s.snr = _ewma(...)`) and the entire `snr_slope` block. Keep the rssi EWMA, `fec_work`, burst/holdoff/late EWMA.

- [ ] **Step 4: Run to verify** — `.venv/bin/python -m pytest tests/unit/test_dl_signals.py -q`
Expected: PASS. (The full suite will still fail — `policy.py` references `signals.snr`; fixed in Task 4. That's expected.)

- [ ] **Step 5: Commit (with Task 4)** — defer; `policy.py` won't import-run cleanly until Task 4 removes the `signals.snr` references. Proceed to Task 4, commit together.

---

## Task 4: Rewrite `LeadingSelector.select()` — probe promote + emergency demote

**Files:** Modify `fpvdgs/dynlink/policy.py`; Test `tests/unit/test_dl_policy_leading.py`

The new selector: emergency demote (kept `_emergency_active`) + video-PER demote + single-step probe promote (clean `current+1`, EWMA threshold, debounce, fresh). Climb stops at the ceiling (a cliffed `current+1` never promotes). Keep `_row`/`current_row`/`_compute_tx_power`/`_emergency_active`/`__init__` row-table. Delete `_pick_mcs`/`_margin`/`_stress_margin_dB`/`_try_confidence_feed` and the SNR `select` body.

- [ ] **Step 1: Write the failing tests** — in `tests/unit/test_dl_policy_leading.py`, keep `_selector(...)` and `_drive_to_mcs`, but replace `_select(...)` and the SNR-mechanism tests. New `_select` + tests:

```python
def _probe(viable_mcs, *, per=0.0, age_ms=0.0):
    """Probe snapshot where every rung up to viable_mcs reads clean+fresh,
    and rungs above it are cliffed (per=1.0)."""
    mcs = {}
    for m in range(0, 8):
        p = per if m <= viable_mcs else 1.0
        mcs[str(m)] = {"per": p, "rssi": -60, "snr": 20, "windows": 50,
                       "ageMs": age_ms}
    return {"running": True, "streams": 1, "mcs": mcs}

def _select(s, *, probe=None, loss=0.0, fec=0.0, link_starved=False, ts_ms=0.0):
    return s.select(probe=probe if probe is not None else _probe(7),
                    loss_rate=loss, fec_pressure=fec,
                    link_starved=link_starved, ts_ms=ts_ms)

def test_promotes_one_step_when_next_rung_clean_after_debounce():
    s = _selector(max_mcs=5, promote_debounce_windows=3,
                  probe_viable_threshold=0.99, probe_freshness_ms=500)
    start = s.state.current_mcs
    ts = 0.0
    last = start
    # need debounce windows of clean current+1, with rate limit satisfied
    for _ in range(8):
        ts += 1000.0
        mcs, _, _ = _select(s, probe=_probe(5), ts_ms=ts)
        last = mcs
    assert last == start + 1 or last > start   # climbed at least one rung

def test_does_not_promote_on_single_clean_blip():
    s = _selector(max_mcs=5, promote_debounce_windows=3)
    start = s.state.current_mcs
    mcs, _, changed = _select(s, probe=_probe(5), ts_ms=1000.0)  # 1 window only
    assert mcs == start and not changed

def test_stops_climbing_at_ceiling():
    s = _selector(max_mcs=3, promote_debounce_windows=1)
    ts = 0.0
    for _ in range(20):
        ts += 1000.0
        mcs, _, _ = _select(s, probe=_probe(3), ts_ms=ts)
    assert s.state.current_mcs == 3   # cliffed above 3, won't exceed

def test_no_promote_when_probe_stale():
    s = _selector(max_mcs=5, promote_debounce_windows=1, probe_freshness_ms=500)
    start = s.state.current_mcs
    ts = 0.0
    for _ in range(5):
        ts += 1000.0
        mcs, _, _ = _select(s, probe=_probe(5, age_ms=999.0), ts_ms=ts)  # stale
    assert s.state.current_mcs == start

def test_emergency_loss_still_demotes_one_step():
    s = _selector(emergency_loss_rate=0.05, max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, _, changed = _select(s, loss=0.06, ts_ms=99999.0)
    assert changed and mcs == pre - 1

def test_video_per_breach_demotes():
    s = _selector(video_demote_per=0.05, max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    # (lost+fec_rec)/(out+lost) modeled via loss_rate input >= video_demote_per
    mcs, _, changed = _select(s, loss=0.06, ts_ms=99999.0)
    assert changed and mcs == pre - 1
```

Add a probe-based climb helper near `_drive_to_mcs`:
```python
def _drive_to_mcs_probe(s, target, max_ticks=400):
    ts = 0.0
    for _ in range(max_ticks):
        ts += 1000.0
        s.select(probe=_probe(7), loss_rate=0.0, fec_pressure=0.0,
                 link_starved=False, ts_ms=ts)
        if s.state.current_mcs >= target:
            break
    return s.state.current_mcs
```

Remove the SNR-mechanism tests listed in the spec §8 (hysteresis, stress-margin, confidence-loop, asymmetric-SNR, `_pick_mcs`, `snr_slope`). Keep the emergency tests, `test_starts_at_safe_default_mcs1`, `test_initial_tx_power_at_max`, `test_max_mcs_too_low_raises`, `test_tx_power_at_mcs0_is_max`, and `test_policy_emits_safe_defaults_until_drone_synced`.

- [ ] **Step 2: Run to verify it fails** — `.venv/bin/python -m pytest tests/unit/test_dl_policy_leading.py -q`
Expected: FAIL — `select()` has the old SNR signature.

- [ ] **Step 3: Implement.** In `fpvdgs/dynlink/policy.py`:

(a) Add promote-debounce state to `LeadingSelector.__init__` (after the row-table/state setup):
```python
        self._promote_clean = 0   # consecutive clean current+1 windows
```

(b) Delete the methods `_stress_margin_dB`, `_margin`, `_pick_mcs`, `_try_confidence_feed`, and the old `select` body. Keep `_row`, `current_row`, `_compute_tx_power`, `_emergency_active`, `reasons`, `__init__`.

(c) Add the new `select` (uses the existing timing helpers/fields — `min_between_changes_ms` and `hold_modes_down_ms` live in `self.sel` (`ProfileSelectionConfig`); reuse the same last-change timestamp the old code tracked in `self.state`. If the old code used `self.state.last_change_ms`, reuse it; otherwise add `self._last_change_ms = 0.0`):
```python
    def select(self, *, probe, loss_rate, fec_pressure, link_starved, ts_ms):
        """Probe-driven promote + reactive demote. Returns (mcs, tx_power, changed)."""
        prev = self.state.current_mcs
        self.reasons.clear() if hasattr(self.reasons, "clear") else None
        reasons = []

        def commit(new_mcs, why):
            new_mcs = max(0, min(new_mcs, self._cap_mcs))
            if new_mcs != self.state.current_mcs:
                self.state.current_mcs = new_mcs
                self.state.tx_power_dBm = self._compute_tx_power(new_mcs)
                self._last_change_ms = ts_ms
                self._promote_clean = 0
                reasons.append(why)

        # --- Demote: emergency (Channel B) or video-PER breach (reactive) ---
        if self._emergency_active(loss_rate, fec_pressure, link_starved):
            commit(prev - 1, f"emergency loss={loss_rate:.3f} fec={fec_pressure:.3f} starved={link_starved}")
            self._reasons = reasons
            return self.state.current_mcs, self.state.tx_power_dBm, (self.state.current_mcs != prev)
        if loss_rate >= self.gate.video_demote_per:
            commit(prev - 1, f"video_per_demote loss={loss_rate:.3f}")
            self._reasons = reasons
            return self.state.current_mcs, self.state.tx_power_dBm, (self.state.current_mcs != prev)

        # --- Rate limit (promotes only; emergencies above bypass it) ---
        within_hold = (ts_ms - self._last_change_ms) < self.sel.hold_modes_down_ms
        within_rate = (ts_ms - self._last_change_ms) < self.sel.min_between_changes_ms
        if within_hold or within_rate:
            self._reasons = reasons
            return self.state.current_mcs, self.state.tx_power_dBm, False

        # --- Promote: clean+fresh current+1 for promote_debounce_windows ticks ---
        target = self.state.current_mcs + 1
        rung = (probe or {}).get("mcs", {}).get(str(target)) if target <= self._cap_mcs else None
        fresh = (rung is not None and rung.get("ageMs") is not None
                 and rung["ageMs"] <= self.gate.probe_freshness_ms)
        clean = (fresh and rung.get("per") is not None
                 and (1.0 - rung["per"]) >= self.gate.probe_viable_threshold)
        if clean:
            self._promote_clean += 1
            if self._promote_clean >= self.gate.promote_debounce_windows:
                commit(target, f"probe_promote mcs{target} per={rung['per']:.4f}")
        else:
            self._promote_clean = 0

        self._reasons = reasons
        return self.state.current_mcs, self.state.tx_power_dBm, (self.state.current_mcs != prev)
```
Update the `reasons` property to return `self._reasons` (init `self._reasons = []` in `__init__`). Add `self._last_change_ms = 0.0` in `__init__` if not already present. Confirm `self.sel` is the `ProfileSelectionConfig` with `min_between_changes_ms`/`hold_modes_down_ms` (it's passed to `__init__` as `sel`); if those live elsewhere, read them from the right config object.

(d) In `Policy.tick()` (policy.py ~811-819), replace the `leading.select(snr_ema=..., ...)` call with:
```python
        new_mcs, tx_power, mcs_changed = self.leading.select(
            probe=self._probe_status() if self._probe_status else None,
            loss_rate=signals.residual_loss_w,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            ts_ms=ts_ms,
        )
```
Remove the `signals.snr`/`snr_max_w`/`snr_slope` references in the `signals_snapshot` dict at the bottom of `tick()` (delete the `"snr"`, `"snr_min_w"`, `"snr_max_w"`, `"snr_slope"` keys; keep rssi/loss/fec/etc.).

(e) `Policy.__init__`: add `probe_status=None` kwarg and store it:
```python
    def __init__(self, cfg, profile, *, drone_config=None, probe_status=None):
        ...
        self._probe_status = probe_status
```

- [ ] **Step 4: Run to verify** — `.venv/bin/python -m pytest tests/unit/test_dl_policy_leading.py tests/unit/test_dl_signals.py -q`
Expected: PASS. Then `.venv/bin/python -m pytest tests/ -q` — the controller test may still pass `Policy` without `probe_status` (defaults to None → no promote, emergency-only) which is fine; if `test_dl_controller` asserts a non-None mcs it still gets one (cold-start/safe). Fix any remaining SNR references the import surfaces.

- [ ] **Step 5: Commit (Tasks 3 + 4)**
```bash
git add gs/fpvdgs/dynlink/signals.py gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_signals.py gs/tests/unit/test_dl_policy_leading.py
git commit -m "feat(gs/dynlink): probe-driven MCS select; remove SNR machinery"
```

---

## Task 5: Wire the probe snapshot through the controller + supervisor

**Files:** Modify `fpvdgs/dynlink/controller.py`, `fpvdgs/supervisor.py`; Test `tests/unit/test_dl_controller.py`

- [ ] **Step 1: Write the failing test** — append to `tests/unit/test_dl_controller.py`:

```python
def test_controller_forwards_probe_snapshot_to_policy():
    seen = {}
    def fake_probe_status():
        seen["called"] = seen.get("called", 0) + 1
        return {"running": True, "streams": 1, "mcs": {}}
    c = DynamicLinkController(_snapshot(_free_port_like := 46990),
                             stats_client_factory=_OneShotStatsClient,
                             gs_listen_port=0, probe_status=fake_probe_status)
    c.start()
    try:
        import time as _t
        _t.sleep(0.4)
        assert seen.get("called", 0) >= 1   # tick loop pulled the probe snapshot
    finally:
        c.stop()
```
(Match the file's existing controller-construction idiom and the `_OneShotStatsClient`/`_snapshot` helpers; the port arg just needs to be the existing positional snapshot+factory shape.)

- [ ] **Step 2: Run to verify it fails** — `.venv/bin/python -m pytest tests/unit/test_dl_controller.py::test_controller_forwards_probe_snapshot_to_policy -q`
Expected: FAIL — `DynamicLinkController.__init__` has no `probe_status`.

- [ ] **Step 3: Implement.** In `fpvdgs/dynlink/controller.py`:
- Add `probe_status=None` to `__init__` and store `self._probe_status = probe_status`.
- In `_run`, pass it into the policy: `policy = Policy(build_policy_config(snap), profile, drone_config=drone_cfg, probe_status=self._probe_status)`.

In `fpvdgs/supervisor.py` `build_app`, **reorder** so `probe_ctrl` is built before `dynlink`, then pass it:
```python
    probe_ctrl = ProbeController(make_probe_snapshot(effective), spawn=probe_spawn)

    dynlink = DynamicLinkController(make_dl_snapshot(effective),
                                    probe_status=probe_ctrl.status)
```
(Leave the rest of `build_app` — `App(...)`/`Api(...)` already receive both controllers.)

- [ ] **Step 4: Run to verify** — `.venv/bin/python -m pytest tests/ -q`
Expected: PASS (full suite green).

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/dynlink/controller.py gs/fpvdgs/supervisor.py gs/tests/unit/test_dl_controller.py
git commit -m "feat(gs/dynlink): wire probe snapshot into the policy tick"
```

---

## Task 6: Cold-start MCS from link RSSI

**Files:** Modify `fpvdgs/dynlink/policy.py`; Test `tests/unit/test_dl_policy_leading.py`

Before any probe data exists, seed the operating MCS from the single link-RSSI via a coarse table (so the first ticks aren't stuck at the safe floor while the probe warms up). Conservative: only used until the probe promotes.

- [ ] **Step 1: Write the failing test** — append to `tests/unit/test_dl_policy_leading.py`:
```python
def test_cold_start_seeds_mcs_from_rssi():
    from fpvdgs.dynlink.policy import coarse_mcs_for_rssi
    assert coarse_mcs_for_rssi(-50) >= coarse_mcs_for_rssi(-80)
    assert coarse_mcs_for_rssi(None) == 0   # unknown -> floor
    assert 0 <= coarse_mcs_for_rssi(-65) <= 7
```

- [ ] **Step 2: Run to verify it fails** — `.venv/bin/python -m pytest tests/unit/test_dl_policy_leading.py::test_cold_start_seeds_mcs_from_rssi -q`
Expected: FAIL — `coarse_mcs_for_rssi` not defined.

- [ ] **Step 3: Implement.** In `fpvdgs/dynlink/policy.py`, add a module-level coarse table + helper:
```python
# Coarse RSSI -> initial MCS, ONLY for cold-start before probe data exists.
# Intentionally conservative; the probe takes over and refines from here.
# (Phase 4 replaces this with the learned per-card prior.)
_COLD_START_RSSI_DBM = [(-55, 5), (-65, 3), (-75, 1), (-200, 0)]

def coarse_mcs_for_rssi(rssi):
    if rssi is None:
        return 0
    for floor, mcs in _COLD_START_RSSI_DBM:
        if rssi >= floor:
            return mcs
    return 0
```
In `Policy.tick()`, before the `select(...)` call, seed once when cold (no promote has happened and the probe has no data yet):
```python
        if not self._cold_started and signals.rssi is not None:
            seed = coarse_mcs_for_rssi(signals.rssi)
            if seed > self.leading.state.current_mcs:
                self.leading.state.current_mcs = min(seed, self.leading._cap_mcs)
                self.leading.state.tx_power_dBm = self.leading._compute_tx_power(
                    self.leading.state.current_mcs)
            self._cold_started = True
```
Add `self._cold_started = False` in `Policy.__init__`.

- [ ] **Step 4: Run to verify** — `.venv/bin/python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_policy_leading.py
git commit -m "feat(gs/dynlink): RSSI cold-start seed for the probe selector"
```

---

## Task 7: On-hardware smoke (needs live GS + drone)

**Files:** none (verification). Deploy the GS only (no drone change).

- [ ] **Step 1:** `./deploy/gs/deploy.sh --host 10.18.0.1`.
- [ ] **Step 2:** Enable dynamicLink (drone via `/air`, GS direct). Confirm the selector **climbs into headroom** the old SNR floor left (e.g. it reaches MCS where the probe reads `current+1` clean), and **stops at the ceiling** (doesn't promote when `current+1` cliffs).
- [ ] **Step 3:** Force a degradation (move the drone to the edge); confirm reactive demote fires (video-PER/Channel-B), and that promotes resume when conditions recover. Confirm no probe/selector change ever bounces the video runner (`runner.restarts` unchanged).
- [ ] **Step 4:** Disable dynamicLink; confirm clean teardown.

---

## Self-Review

**Spec coverage (`2026-06-07-probe-driven-mcs-selector-design.md`):**
- §4 probe-driven promote (EWMA + debounce, stops at ceiling) → Task 4 (`select` promote branch) + Task 1 (freshness). ✓
- §4 reactive demote (Channel-B kept + video-PER) → Task 4 (emergency + `video_demote_per`). ✓
- §4 cold-start RSSI → Task 6. ✓
- §4 probe-stale ⇒ no promote → Task 4 (`fresh` gate) + Task 1 (`ageMs`). ✓
- §5 wiring (probe snapshot → `Policy.tick`) → Task 5 (controller `probe_status` + supervisor reorder) + Task 4(e)/(d). ✓
- §6 remove SNR machinery → Task 2 (gate knobs), Task 3 (signals), Task 4(b) (selector helpers). ✓ Kept: Channel-B, predictor, bitrate, FEC, wire, HELLO (untouched). ✓
- §7 config delta → Task 2. ✓
- §8 tests → Tasks 1-6 each rewrite/extend their test file. ✓

**Placeholder scan:** Task 2 Step 1 and Task 5 Step 1 say "match the existing harness/entry point" — these are real seams the implementer reads (the test files' construction idioms); the new code/assertions are given in full. Task 4(c) notes "reuse the last-change timestamp the old code tracked, or add `self._last_change_ms`" — the implementer confirms which exists; both the field and its use are specified. No TBDs.

**Type/name consistency:** `select(probe=, loss_rate=, fec_pressure=, link_starved=, ts_ms=)` consistent Task 4 def ↔ Task 4(d) call. `probe_status` callable consistent: supervisor (`probe_ctrl.status`) → `DynamicLinkController(probe_status=)` → `Policy(probe_status=)` → `self._probe_status()` in `tick`. `ageMs` consistent Task 1 (producer) ↔ Task 4 (`fresh` consumer). `GateConfig` fields consistent Task 2 (def) ↔ Task 4 (use: `probe_viable_threshold`, `probe_freshness_ms`, `promote_debounce_windows`, `video_demote_per`, `emergency_*`, `max_mcs`). `coarse_mcs_for_rssi` consistent Task 6.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-07-probe-driven-mcs-selector.md`.**

**Recommended:** execute via **superpowers:subagent-driven-development** (fresh subagent per task, spec + quality review between). Given this session's length, a fresh session is the cleaner place to run it — the plan + spec on disk are the complete handoff.
