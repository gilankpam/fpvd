# Predictive-Demote Flapping Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the learned-prior predictive demote from flapping the link by de-noising its RSSI slope (least-squares over a window) and gating it to fire only on a genuine projected fade.

**Architecture:** Pure GS-side change in `gs/fpvdgs/dynlink/`. Add a pure `lsq_slope` helper to `learned_prior.py`; in `policy.py` replace the single-tick RSSI delta with a least-squares slope over a rolling window, and add a slope-direction gate so a predictive demote only fires when the projected RSSI drop over the horizon is meaningful. Add a `predict_gated` flight-log field and surface it in the offline analyzer. No wire/drone/config-schema change; the prior data, probe, warm-start, and reactive demote are untouched.

**Tech Stack:** Python ≥3.11, pytest. Run the GS suite from `gs/` with `.venv/bin/python -m pytest`.

**Spec:** `docs/superpowers/specs/2026-06-15-predictive-demote-flapping-fix-design.md`

---

## File Structure

- **Modify** `gs/fpvdgs/dynlink/learned_prior.py` — add the pure module-level `lsq_slope()` helper; add two `LearnedPriorConfig` fields (`predictive_slope_window_ticks`, `predictive_min_drop_db`).
- **Modify** `gs/fpvdgs/dynlink/policy.py` — import `lsq_slope` and `deque`; swap `_prev_rssi` for a rolling RSSI window; compute the least-squares slope; add the slope-direction gate + `predict_gated`; log `predict_gated`.
- **Modify** `gs/tools/flightlog_analyze.py` — count + print `predict_gated` (the suppressed-demote diagnostic).
- **Tests:** `gs/tests/unit/test_dl_learned_prior.py` (helper), `gs/tests/unit/test_dl_flightlog_debug_fields.py` (logged slope + `predict_gated`), `gs/tests/unit/test_dl_policy_learned.py` (gate behavior), `gs/tests/unit/test_dl_flightlog_analyze.py` (analyzer).

**Note on existing tests (verified, should stay green):** a 2-sample least-squares slope equals the single-tick delta, so `test_record_carries_pc_and_slope` (asserts `slope == -2.0`) and `test_record_pc_and_slope_none_when_prior_cold_or_no_rssi` are unaffected; `test_predictive_demote_on_confident_fade` uses a genuine −3 dB/tick fade whose projected drop (6 dB) clears the gate. If any of these fail, stop and reconcile before continuing.

---

## Task 1: `lsq_slope` pure helper

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py` (add module-level function after `MAX_MCS`, line 20)
- Test: `gs/tests/unit/test_dl_learned_prior.py`

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_dl_learned_prior.py`:

```python
def test_lsq_slope_flat_is_zero():
    from fpvdgs.dynlink.learned_prior import lsq_slope
    assert lsq_slope([-50.0, -50.0, -50.0]) == 0.0


def test_lsq_slope_linear_ramp_is_exact():
    from fpvdgs.dynlink.learned_prior import lsq_slope
    # -0.5 dBm per tick ramp
    assert abs(lsq_slope([-50.0, -50.5, -51.0, -51.5, -52.0]) - (-0.5)) < 1e-9


def test_lsq_slope_rejects_lone_spike():
    from fpvdgs.dynlink.learned_prior import lsq_slope
    # a -0.5/tick ramp with one +10 dB spike stays a clear downtrend;
    # a single-tick delta at the spike would read ~+7.5
    ramp = [-0.5 * i for i in range(10)]
    ramp[5] += 10.0
    assert -0.6 < lsq_slope(ramp) < -0.35


def test_lsq_slope_under_two_samples_is_zero():
    from fpvdgs.dynlink.learned_prior import lsq_slope
    assert lsq_slope([]) == 0.0
    assert lsq_slope([-50.0]) == 0.0
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q -k lsq_slope`
Expected: FAIL with `ImportError: cannot import name 'lsq_slope'`.

- [ ] **Step 3: Implement the helper**

In `gs/fpvdgs/dynlink/learned_prior.py`, after the `MAX_MCS = 7` line (line 20), add:

```python


def lsq_slope(samples) -> float:
    """Least-squares gradient (dBm per tick) over an evenly-spaced sample
    sequence (x = 0, 1, ..., n-1). Unlike a single-tick delta, a lone spike
    barely moves the fit. Fewer than 2 samples → 0.0 (no trend yet)."""
    n = len(samples)
    if n < 2:
        return 0.0
    mean_x = (n - 1) / 2.0
    mean_y = sum(samples) / n
    num = sum((i - mean_x) * (y - mean_y) for i, y in enumerate(samples))
    den = sum((i - mean_x) ** 2 for i in range(n))
    return num / den if den else 0.0
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q -k lsq_slope`
Expected: 4 passed.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "dl: add lsq_slope least-squares RSSI gradient helper"
```

---

## Task 2: Least-squares slope in Policy

Replace the single-tick delta with a least-squares slope over a rolling RSSI window. Behavior of the demote *decision* is unchanged in this task (the gate comes in Task 3); only the slope input changes.

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py` (add `predictive_slope_window_ticks` to `LearnedPriorConfig`, after line 34)
- Modify: `gs/fpvdgs/dynlink/policy.py:14-21` (imports), `:263` (init), `:297-314` (slope computation)
- Test: `gs/tests/unit/test_dl_flightlog_debug_fields.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_flightlog_debug_fields.py`:

```python
def test_logged_slope_is_least_squares_not_single_tick(tmp_path):
    """A lone RSSI spike barely moves the logged slope (least-squares over a
    window) — the old single-tick delta would log the full +5 dB jump."""
    p = Policy(_cfg(tmp_path), _profile())
    for rssi, ts in [(-50.0, 1.0), (-50.0, 1.1), (-50.0, 1.2),
                     (-50.0, 1.3), (-45.0, 1.4)]:
        p.tick(_sig(rssi, ts=ts))
    p.close()
    last = _records(tmp_path)[-1]
    # lsq over [-50,-50,-50,-50,-45] = +1.0  (single-tick delta would be +5.0)
    assert abs(last["slope"] - 1.0) < 1e-6
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_debug_fields.py::test_logged_slope_is_least_squares_not_single_tick -q`
Expected: FAIL — `assert abs(5.0 - 1.0) < 1e-6` (old code logs the +5.0 single-tick delta).

- [ ] **Step 3a: Add the config field**

In `gs/fpvdgs/dynlink/learned_prior.py`, in `LearnedPriorConfig`, immediately after `predictive_debounce_windows: int = 3` (line 34), add:

```python
    predictive_slope_window_ticks: int = 10   # least-squares RSSI window (1.0 s @ 10 Hz)
```

- [ ] **Step 3b: Import `deque` and `lsq_slope` in policy.py**

In `gs/fpvdgs/dynlink/policy.py`, change the imports block (lines 14-21) so it reads:

```python
import logging
import time
from collections import deque
from dataclasses import dataclass, field

from .decision import Decision
from .flightlog import FlightLog, FlightLogConfig
from .learned_prior import LearnedPrior, LearnedPriorConfig, lsq_slope
from .signals import Signals
```

- [ ] **Step 3c: Swap `_prev_rssi` for a rolling window**

In `gs/fpvdgs/dynlink/policy.py`, replace line 263:

```python
        self._prev_rssi: float | None = None
```

with:

```python
        self._rssi_window: deque = deque(
            maxlen=cfg.learned_prior.predictive_slope_window_ticks)
```

- [ ] **Step 3d: Compute the least-squares slope**

In `gs/fpvdgs/dynlink/policy.py`, replace the predictive-demote block (lines 297-314, from `predict_reason = ""` through `self._prev_rssi = signals.rssi`) with:

```python
        predict_reason = ""
        if signals.rssi is None:
            slope = None
        else:
            self._rssi_window.append(signals.rssi)
            slope = lsq_slope(self._rssi_window)
        pc = None
        if signals.rssi is not None:
            pc = self.learned_prior.predictive_ceiling(signals.rssi, slope)
            cur = self.leading.state.current_mcs
            if pc is not None and pc < cur:
                self._predict_demote_count += 1
                if (self._predict_demote_count
                        >= self.cfg.learned_prior.predictive_debounce_windows):
                    self.leading.state.current_mcs = max(pc, 0)
                    self.leading._promote_clean = 0
                    predict_reason = f"predict_demote mcs{cur}->{pc}"
            else:
                self._predict_demote_count = 0
```

(The trailing `self._prev_rssi = signals.rssi` line is removed — the window is appended inside the block.)

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_debug_fields.py tests/unit/test_dl_policy_learned.py -q`
Expected: all pass (new slope test passes; existing `test_record_carries_pc_and_slope`, `test_record_pc_and_slope_none_when_prior_cold_or_no_rssi`, and `test_predictive_demote_on_confident_fade` still pass).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/learned_prior.py gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_flightlog_debug_fields.py
git commit -m "dl: use least-squares RSSI slope for predictive demote (de-noise)"
```

---

## Task 3: Slope-direction gate + `predict_gated`

Only predict-demote when the projected RSSI drop over the horizon is meaningful, and log when a would-be demote was suppressed.

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py` (add `predictive_min_drop_db` to `LearnedPriorConfig`)
- Modify: `gs/fpvdgs/dynlink/policy.py` (gate logic in the predictive block; `predict_gated` in the record)
- Test: `gs/tests/unit/test_dl_policy_learned.py`, `gs/tests/unit/test_dl_flightlog_debug_fields.py`

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_dl_policy_learned.py`:

```python
def test_predictive_demote_blocked_when_rssi_flat(tmp_path):
    """Static prior-vs-probe disagreement at flat RSSI must NOT demote: the
    probe legitimately climbed above the prior's learned ceiling, but with no
    fade the slope-direction gate suppresses the predictive demote. This is the
    000010/000012 flapping root cause."""
    prof = _profile()
    p = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=10_000,
                    min_samples_predictive=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=2), prof)
    # learned ceiling at -50 = 2 (rungs 0..2 clean; rung 3 dirty)
    for _ in range(5):
        for rung in range(3):
            p.learned_prior.ingest(rssi=-50.0, probed_rung=rung, probe_clean=True,
                                   operating_mcs=rung, operating_clean=True)
        p.learned_prior.ingest(rssi=-50.0, probed_rung=3, probe_clean=False,
                               operating_mcs=2, operating_clean=True)
    p.leading.state.current_mcs = 5      # probe pushed above the learned ceiling
    dec = None
    for ts in (1.0, 1.1, 1.2, 1.3, 1.4):
        dec = p.tick(_sig(-50.0, ts=ts))
    assert dec.mcs == 5                   # flat RSSI → never predict-demoted
    p.close()


def test_predictive_demote_blocked_when_fade_too_shallow(tmp_path):
    """A real but shallow downtrend whose projected drop is below
    predictive_min_drop_db must NOT demote — only a genuine fade should."""
    prof = _profile()
    p = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=10_000,
                    min_samples_predictive=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=2, predictive_min_drop_db=1.0), prof)

    def prime(rssi, ceiling, n=50):
        b = p.learned_prior.rssi_bin(rssi)
        for rung in range(ceiling + 1):
            p.learned_prior._cells[b][rung] = [1.0, float(n)]

    # Ceiling 2 across a band so the small fade stays in the ceiling-2 region.
    for r in (-52.0, -51.0, -50.0, -49.0, -48.0):
        prime(r, 2)
    p.leading.state.current_mcs = 5
    # shallow -0.2 dBm/tick fade: projected drop over horizon = 0.6 dB < 1.0
    dec = None
    for rssi, ts in [(-50.0, 1.0), (-50.2, 1.1), (-50.4, 1.2),
                     (-50.6, 1.3), (-50.8, 1.4)]:
        dec = p.tick(_sig(rssi, ts=ts))
    assert dec.mcs == 5                   # shallow fade gated out
    p.close()
```

Append to `gs/tests/unit/test_dl_flightlog_debug_fields.py`:

```python
def test_record_carries_predict_gated_flag(tmp_path):
    """predict_gated is True when pc < cur but the slope-direction gate blocks
    the demote (flat RSSI = no real fade); the reason carries no predict_demote."""
    prof = _profile()
    p = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=10_000,
                    min_samples_predictive=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=2), prof)
    for _ in range(5):
        for rung in range(3):
            p.learned_prior.ingest(rssi=-50.0, probed_rung=rung, probe_clean=True,
                                   operating_mcs=rung, operating_clean=True)
        p.learned_prior.ingest(rssi=-50.0, probed_rung=3, probe_clean=False,
                               operating_mcs=2, operating_clean=True)
    p.leading.state.current_mcs = 5
    p.tick(_sig(-50.0, ts=1.0))
    p.tick(_sig(-50.0, ts=1.1))
    p.close()
    last = _records(tmp_path)[-1]
    assert last["predict_gated"] is True
    assert "predict_demote" not in last["reason"]


def test_record_predict_gated_false_when_no_demote_intent(tmp_path):
    # Cold prior → pc None → no demote intent → predict_gated False.
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig(-50.0))
    p.close()
    assert _records(tmp_path)[-1]["predict_gated"] is False
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py::test_predictive_demote_blocked_when_rssi_flat tests/unit/test_dl_policy_learned.py::test_predictive_demote_blocked_when_fade_too_shallow tests/unit/test_dl_flightlog_debug_fields.py::test_record_carries_predict_gated_flag tests/unit/test_dl_flightlog_debug_fields.py::test_record_predict_gated_false_when_no_demote_intent -q`
Expected: FAIL — the two policy tests demote to 2 (old code ignores slope direction); the `predict_gated` tests `KeyError` (field not in record yet).

- [ ] **Step 3a: Add the config field**

In `gs/fpvdgs/dynlink/learned_prior.py`, in `LearnedPriorConfig`, immediately after the `predictive_slope_window_ticks` line added in Task 2, add:

```python
    predictive_min_drop_db: float = 1.0       # min projected RSSI drop over the horizon to demote
```

- [ ] **Step 3b: Add the gate and `predict_gated`**

In `gs/fpvdgs/dynlink/policy.py`, replace the predictive-demote block (the version produced in Task 2, from `predict_reason = ""` through the `else: self._predict_demote_count = 0`) with:

```python
        predict_reason = ""
        predict_gated = False
        if signals.rssi is None:
            slope = None
        else:
            self._rssi_window.append(signals.rssi)
            slope = lsq_slope(self._rssi_window)
        pc = None
        if signals.rssi is not None:
            pc = self.learned_prior.predictive_ceiling(signals.rssi, slope)
            cur = self.leading.state.current_mcs
            projected_drop = -slope * self.cfg.learned_prior.predictive_horizon_ticks
            if pc is not None and pc < cur:
                if projected_drop >= self.cfg.learned_prior.predictive_min_drop_db:
                    self._predict_demote_count += 1
                    if (self._predict_demote_count
                            >= self.cfg.learned_prior.predictive_debounce_windows):
                        self.leading.state.current_mcs = max(pc, 0)
                        self.leading._promote_clean = 0
                        predict_reason = f"predict_demote mcs{cur}->{pc}"
                else:
                    # pc says demote but RSSI isn't genuinely falling: a static
                    # prior-vs-probe disagreement, not a fade. Suppress (the
                    # flapping fix) and log it.
                    predict_gated = True
                    self._predict_demote_count = 0
            else:
                self._predict_demote_count = 0
```

- [ ] **Step 3c: Log `predict_gated` in the record**

In `gs/fpvdgs/dynlink/policy.py`, in the `self.flightlog.write({...})` dict, add a line immediately after `"slope": slope,`:

```python
            "predict_gated": predict_gated,
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py tests/unit/test_dl_flightlog_debug_fields.py -q`
Expected: all pass (new gate + `predict_gated` tests pass; the existing genuine-fade test still demotes).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/learned_prior.py gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_policy_learned.py gs/tests/unit/test_dl_flightlog_debug_fields.py
git commit -m "dl: gate predictive demote on a genuine projected fade (fix flapping)"
```

---

## Task 4: Surface `predict_gated` in the offline analyzer

**Files:**
- Modify: `gs/tools/flightlog_analyze.py` (`summarize` + `_print_summary`)
- Test: `gs/tests/unit/test_dl_flightlog_analyze.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_flightlog_analyze.py`:

```python
def test_summarize_counts_gated_demotes(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    with open(log, "w") as f:
        f.write(json.dumps({"ts": 1.0, "mcs": 5, "reason": "",
                            "predict_gated": True}) + "\n")
        f.write(json.dumps({"ts": 1.1, "mcs": 5, "reason": "",
                            "predict_gated": False}) + "\n")
        f.write(json.dumps({"ts": 1.2, "mcs": 5, "reason": ""}) + "\n")  # pre-field log
    s = mod.summarize(str(log))
    assert s["gated_demotes"] == 1
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_analyze.py::test_summarize_counts_gated_demotes -q`
Expected: FAIL — `KeyError: 'gated_demotes'`.

- [ ] **Step 3a: Count gated demotes in `summarize`**

In `gs/tools/flightlog_analyze.py`, in `summarize`, change the counter init line:

```python
    predictive = reactive = warm_fallback = 0
```

to:

```python
    predictive = reactive = warm_fallback = gated = 0
```

Then inside the `for r in recs:` loop, after the `reactive` counting block, add:

```python
        if r.get("predict_gated"):
            gated += 1
```

And in the returned dict, add a key after `"reactive_demotes": reactive,`:

```python
        "gated_demotes": gated,
```

- [ ] **Step 3b: Print it**

In `gs/tools/flightlog_analyze.py`, in `_print_summary`, after the `reactive demotes:` print line, add:

```python
    print(f"gated demotes:      {s['gated_demotes']}")
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_analyze.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add gs/tools/flightlog_analyze.py gs/tests/unit/test_dl_flightlog_analyze.py
git commit -m "tools: report gated (suppressed) predictive demotes in flightlog_analyze"
```

---

## Task 5: Full-suite verification + replay on the diagnosed logs

- [ ] **Step 1: Run the entire GS suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: all pass (the suite must be green as a whole — config_build/import coupling means partial refactors go red).

- [ ] **Step 2: Sanity-replay the analyzer on the original logs**

Run: `cd gs && .venv/bin/python tools/flightlog_analyze.py ../logs/000010.jsonl && .venv/bin/python tools/flightlog_analyze.py ../logs/000012.jsonl`
Expected: prints summaries without error, including a `gated demotes:` line (0 for these pre-field logs, since they predate `predict_gated` — confirms the field is tolerated when absent). These logs predate the fix, so their `predictive demotes` counts are unchanged; this step only confirms the analyzer still parses them.

- [ ] **Step 3: No commit** (verification only).

---

## Self-Review (completed at authoring time)

- **Spec coverage:** §4.1 least-squares slope → Task 1 (helper) + Task 2 (wiring); §4.2 slope-direction gate → Task 3; §4.3 `predict_gated` diagnostic → Task 3 (field) + Task 4 (analyzer); §6 testing → tests in every task + Task 5 full-suite; §1 count-decay → explicitly out of scope, no task (correct).
- **Placeholder scan:** none — every code/command step shows concrete content.
- **Type/name consistency:** `lsq_slope` (Task 1) imported/called identically in Task 2; config fields `predictive_slope_window_ticks` (Task 2) and `predictive_min_drop_db` (Task 3) referenced via `self.cfg.learned_prior.*` matching the existing `predictive_debounce_windows`/`predictive_horizon_ticks` access; `predict_gated` key consistent across policy record (Task 3) and analyzer (Task 4); `_rssi_window` defined in Task 2 init and used in Task 2/3 tick.
