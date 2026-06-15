# Reactive Loss-Demote Hysteresis Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Stop the reactive loss demote from firing on single-window transients by adding consecutive-window hysteresis, and unify the redundant `emergency_loss_rate`/`video_demote_per` loss paths into one gated decision.

**Architecture:** Pure GS-side change in `gs/fpvdgs/dynlink/`. Task 1 is a behavior-preserving refactor: `select()` takes a `loss_demote: bool` (loss removed from `_emergency_active`, which becomes fec-or-starved), and `emergency_loss_rate` is removed (`video_demote_per` is the single loss threshold). Task 2 adds the hysteresis: `Policy.tick` counts consecutive breaching windows (mirroring `starvation_windows`) and only sets `loss_demote` once sustained, plus a `loss_gated` diagnostic. No wire/drone change; `selector` config gains `lossWindows` and drops `emergencyLossRate`.

**Tech Stack:** Python ≥3.11, pytest. Run the GS suite from `gs/` with `.venv/bin/python -m pytest`.

**Spec:** `docs/superpowers/specs/2026-06-15-loss-hysteresis-design.md`

---

## File Structure

- **Modify** `gs/fpvdgs/dynlink/policy.py` — `SelectorConfig` (drop `emergency_loss_rate`, add `loss_windows`); `_emergency_active` (fec/starved only); `select()` (`loss_demote` param + restructured demote); `Policy.__init__`/`Policy.tick` (loss hysteresis counter, `loss_gated`, record field).
- **Modify** `gs/fpvdgs/dynlink/config_build.py` — drop `emergencyLossRate` read, add `lossWindows` read.
- **Modify** `gs/fpvdgs/config_defaults.py` — drop `emergencyLossRate`, add `lossWindows`.
- **Modify** `gs/tools/flightlog_analyze.py` — count + print `loss_gated`.
- **Tests:** `gs/tests/unit/test_dl_policy_leading.py` (select interface), `gs/tests/unit/test_dl_config_build.py` (config field swap), new `gs/tests/unit/test_dl_loss_hysteresis.py` (Policy-level hysteresis + `loss_gated`), `gs/tests/unit/test_dl_flightlog_analyze.py` (analyzer).

---

## Task 1: Unify the loss path (behavior-preserving)

Remove the redundant `emergency_loss_rate`; loss demote becomes a `loss_demote: bool` the caller computes (single-window for now — identical behavior to today). The emergency path becomes fec-or-starved. Reasons cleanly separate (loss → `video_per_demote`, fec/starved → `emergency`).

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py` (`SelectorConfig`, `_emergency_active`, `select`, `Policy.tick`)
- Modify: `gs/fpvdgs/dynlink/config_build.py`
- Modify: `gs/fpvdgs/config_defaults.py`
- Test: `gs/tests/unit/test_dl_policy_leading.py`, `gs/tests/unit/test_dl_config_build.py`

- [ ] **Step 1: Update the failing tests first**

In `gs/tests/unit/test_dl_policy_leading.py`:

(a) Remove `emergency_loss_rate` from the `_selector` helper — delete the parameter line `              emergency_loss_rate: float = 0.05,` and the kwarg line `        emergency_loss_rate=emergency_loss_rate,`.

(b) Replace the `_select` helper with one that passes `loss_demote`:

```python
def _select(s, *, probe=None, loss=0.0, loss_demote=False, fec=0.0,
            link_starved=False, ts_ms=0.0):
    return s.select(probe=probe if probe is not None else _probe(7),
                    loss_rate=loss, loss_demote=loss_demote, fec_pressure=fec,
                    link_starved=link_starved, ts_ms=ts_ms)
```

(c) Replace `test_emergency_loss_still_demotes_one_step`, `test_emergency_below_threshold_no_demote`, and `test_video_per_breach_demotes` with these three (the dual-threshold test is obsolete; the loss-threshold semantics move to Task 2's Policy-level tests):

```python
def test_loss_demote_one_step():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, changed = _select(s, loss=0.06, loss_demote=True, ts_ms=99999.0)
    assert changed and mcs == pre - 1


def test_loss_not_demoted_when_loss_demote_false():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, changed = _select(s, loss=0.06, loss_demote=False, ts_ms=99999.0)
    assert not changed and mcs == pre


def test_loss_demote_reason_is_video_per_not_emergency():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    _select(s, loss=0.07, loss_demote=True, ts_ms=99999.0)
    assert any("video_per_demote" in r for r in s._reasons)
    assert not any("emergency" in r for r in s._reasons)
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_leading.py -q`
Expected: FAIL — `select()` got an unexpected keyword `loss_demote` (and `_selector` no longer accepts `emergency_loss_rate`).

- [ ] **Step 3a: `SelectorConfig` — drop `emergency_loss_rate`**

In `gs/fpvdgs/dynlink/policy.py`, in `SelectorConfig`, delete the line:

```python
    emergency_loss_rate: float = 0.05
```

- [ ] **Step 3b: `_emergency_active` — fec/starved only**

In `gs/fpvdgs/dynlink/policy.py`, replace the whole `_emergency_active` method:

```python
    def _emergency_active(self, fec_pressure: float, link_starved: bool) -> bool:
        return (
            fec_pressure >= self.cfg.emergency_fec_pressure
            or link_starved
        )
```

- [ ] **Step 3c: `select()` — add `loss_demote`, restructure demote**

In `gs/fpvdgs/dynlink/policy.py`, change the `select` signature to add `loss_demote` (keyword-only, default False):

```python
    def select(
        self,
        *,
        probe: dict | None,
        loss_rate: float,
        loss_demote: bool = False,
        fec_pressure: float,
        link_starved: bool,
        ts_ms: float,
    ) -> tuple[int, bool]:
```

Then replace the demote block (currently the `if self._emergency_active(loss_rate, fec_pressure, link_starved):` block through the `video_per_demote` block) with:

```python
        # --- Demote (reactive, bypasses the promote rate limit) ---
        # Emergency: FEC pressure or sustained starvation. Loss is a separate,
        # caller-hysteresis-gated trigger (loss_demote) handled just below.
        if self._emergency_active(fec_pressure, link_starved):
            commit(
                prev - 1,
                f"emergency fec={fec_pressure:.3f} starved={link_starved}",
            )
            self._reasons = reasons
            return (st.current_mcs, st.current_mcs != prev)
        if loss_demote:
            commit(prev - 1, f"video_per_demote loss={loss_rate:.3f}")
            self._reasons = reasons
            return (st.current_mcs, st.current_mcs != prev)
```

- [ ] **Step 3d: `Policy.tick` — compute and pass `loss_demote`**

In `gs/fpvdgs/dynlink/policy.py`, find the `self.leading.select(` call in `tick` and replace it with (adds the `loss_demote` line + arg; single-window here — hysteresis comes in Task 2):

```python
        loss_demote = signals.residual_loss_w >= self.cfg.selector.video_demote_per
        new_mcs, _changed = self.leading.select(
            probe=probe_snap,
            loss_rate=signals.residual_loss_w,
            loss_demote=loss_demote,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            ts_ms=ts_ms,
        )
```

- [ ] **Step 3e: `config_build.py` — drop the `emergencyLossRate` read**

In `gs/fpvdgs/dynlink/config_build.py`, delete the line:

```python
        emergency_loss_rate=float(sel.get("emergencyLossRate", d.emergency_loss_rate)),
```

- [ ] **Step 3f: `config_defaults.py` — drop `emergencyLossRate`**

In `gs/fpvdgs/config_defaults.py`, delete the line:

```python
            "emergencyLossRate": sel.emergency_loss_rate,
```

- [ ] **Step 3g: `test_dl_config_build.py` — drop the `emergency_loss_rate` assertion**

In `gs/tests/unit/test_dl_config_build.py`, delete the line:

```python
    assert s.emergency_loss_rate == 0.05
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_leading.py tests/unit/test_dl_config_build.py -q`
Expected: all pass (loss demotes via `loss_demote`; fec/starved emergency unchanged).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/policy.py gs/fpvdgs/dynlink/config_build.py gs/fpvdgs/config_defaults.py gs/tests/unit/test_dl_policy_leading.py gs/tests/unit/test_dl_config_build.py
git commit -m "dl: unify loss demote into a gated loss_demote (drop emergency_loss_rate)"
```

---

## Task 2: Add loss hysteresis + `loss_gated`

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py` (`SelectorConfig.loss_windows`, `Policy.__init__`, `Policy.tick`)
- Modify: `gs/fpvdgs/dynlink/config_build.py`, `gs/fpvdgs/config_defaults.py`
- Test: new `gs/tests/unit/test_dl_loss_hysteresis.py`, `gs/tests/unit/test_dl_config_build.py`

- [ ] **Step 1: Write the failing tests**

Create `gs/tests/unit/test_dl_loss_hysteresis.py`:

```python
"""Reactive loss-demote hysteresis: require N consecutive breaching windows
(residual_loss_w >= video_demote_per) before a loss demote, mirroring the
starvation hysteresis. A single transient window must not demote."""
from __future__ import annotations

import json

from fpvdgs.dynlink.policy import Policy, PolicyConfig, SelectorConfig
from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.signals import Signals


def _cfg(tmp_path, **sel):
    return PolicyConfig(
        selector=SelectorConfig(**sel),
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path)),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def _sig(loss, rssi=-50.0, ts=1.0):
    return Signals(rssi=rssi, residual_loss_w=loss, fec_work=0.0,
                   link_starved_w=False, timestamp=ts)


def _records(tmp_path):
    files = sorted((tmp_path / "fl").glob("*.jsonl"))
    assert files, "expected a flight-log file"
    with open(files[-1]) as f:
        return [json.loads(line) for line in f if line.strip()]


def test_single_loss_window_does_not_demote(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    dec = p.tick(_sig(0.06, ts=1.0))
    assert dec.mcs == 5             # one breaching window → held
    p.close()


def test_two_consecutive_loss_windows_demote(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.06, ts=1.0))
    dec = p.tick(_sig(0.06, ts=1.1))
    assert dec.mcs == 4             # sustained → demote
    p.close()


def test_clean_window_resets_loss_count(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.06, ts=1.0))      # count 1
    p.tick(_sig(0.0, ts=1.1))       # clean → reset
    dec = p.tick(_sig(0.06, ts=1.2))  # count 1 again
    assert dec.mcs == 5             # not sustained → held
    p.close()


def test_sustained_loss_demotes_each_window_after_latch(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.06, ts=1.0))         # count 1, no demote
    d2 = p.tick(_sig(0.06, ts=1.1))    # count 2 → demote to 4
    d3 = p.tick(_sig(0.06, ts=1.2))    # still breaching → demote to 3
    assert d2.mcs == 4 and d3.mcs == 3
    p.close()


def test_loss_gated_true_when_suppressed(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.06, ts=1.0))         # breach but not sustained
    p.close()
    last = _records(tmp_path)[-1]
    assert last["loss_gated"] is True
    assert "video_per_demote" not in last["reason"]


def test_loss_gated_false_when_clean(tmp_path):
    p = Policy(_cfg(tmp_path, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.0, ts=1.0))
    p.close()
    assert _records(tmp_path)[-1]["loss_gated"] is False
```

Append to `gs/tests/unit/test_dl_config_build.py`:

```python
def test_loss_windows_reads_and_defaults():
    from fpvdgs.dynlink.config_build import build_policy_config
    over = build_policy_config(_block(selector={"lossWindows": 4}))
    assert over.selector.loss_windows == 4
    assert build_policy_config(_block()).selector.loss_windows == 2  # default
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_loss_hysteresis.py tests/unit/test_dl_config_build.py::test_loss_windows_reads_and_defaults -q`
Expected: FAIL — `SelectorConfig` has no `loss_windows` (TypeError), and records have no `loss_gated` key.

- [ ] **Step 3a: `SelectorConfig` — add `loss_windows`**

In `gs/fpvdgs/dynlink/policy.py`, in `SelectorConfig`, immediately after the `starvation_windows: int = 5` line, add:

```python
    # Loss hysteresis: consecutive breaching windows (residual_loss_w >=
    # video_demote_per) before a loss demote — filters single-window transients.
    loss_windows: int = 2
```

- [ ] **Step 3b: `Policy.__init__` — add the counter**

In `gs/fpvdgs/dynlink/policy.py`, in `Policy.__init__`, immediately after `self._starvation_count: int = 0`, add:

```python
        self._loss_count: int = 0
```

- [ ] **Step 3c: `Policy.tick` — compute hysteresis + `loss_gated`**

In `gs/fpvdgs/dynlink/policy.py`, find the starvation hysteresis block in `tick` (ends with `sustained_starved = (...)`) and immediately after it add:

```python
        # Loss hysteresis (mirrors starvation): residual_loss_w is raw and
        # spikes on a single bad window. Require loss_windows consecutive
        # breaching windows before a loss demote; flag suppressed ones.
        if signals.residual_loss_w >= self.cfg.selector.video_demote_per:
            self._loss_count += 1
        else:
            self._loss_count = 0
        sustained_loss = self._loss_count >= self.cfg.selector.loss_windows
        loss_gated = (
            signals.residual_loss_w >= self.cfg.selector.video_demote_per
            and not sustained_loss
        )
```

Then replace the Task-1 `loss_demote = signals.residual_loss_w >= self.cfg.selector.video_demote_per` line (just above the `self.leading.select(` call) so the call uses `sustained_loss`:

```python
        new_mcs, _changed = self.leading.select(
            probe=probe_snap,
            loss_rate=signals.residual_loss_w,
            loss_demote=sustained_loss,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            ts_ms=ts_ms,
        )
```

(Delete the now-unused `loss_demote = ...` assignment line from Task 1 Step 3d.)

- [ ] **Step 3d: `Policy.tick` — log `loss_gated`**

In `gs/fpvdgs/dynlink/policy.py`, in the `self.flightlog.write({...})` dict, add a line immediately after `"link_starved": sustained_starved,`:

```python
            "loss_gated": loss_gated,
```

- [ ] **Step 3e: `config_build.py` — read `lossWindows`**

In `gs/fpvdgs/dynlink/config_build.py`, in the `SelectorConfig(...)` construction, immediately after the `starvation_windows=...` line, add:

```python
        loss_windows=int(sel.get("lossWindows", d.loss_windows)),
```

- [ ] **Step 3f: `config_defaults.py` — add `lossWindows`**

In `gs/fpvdgs/config_defaults.py`, in the `"selector": { ... }` dict, immediately after the `"starvationWindows": sel.starvation_windows,` line, add:

```python
            "lossWindows": sel.loss_windows,
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_loss_hysteresis.py tests/unit/test_dl_config_build.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/policy.py gs/fpvdgs/dynlink/config_build.py gs/fpvdgs/config_defaults.py gs/tests/unit/test_dl_loss_hysteresis.py gs/tests/unit/test_dl_config_build.py
git commit -m "dl: add consecutive-window loss hysteresis + loss_gated diagnostic"
```

---

## Task 3: Surface `loss_gated` in the analyzer

**Files:**
- Modify: `gs/tools/flightlog_analyze.py` (`summarize` + `_print_summary`)
- Test: `gs/tests/unit/test_dl_flightlog_analyze.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_flightlog_analyze.py`:

```python
def test_summarize_counts_loss_gated(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    with open(log, "w") as f:
        f.write(json.dumps({"ts": 1.0, "mcs": 5, "reason": "",
                            "loss_gated": True}) + "\n")
        f.write(json.dumps({"ts": 1.1, "mcs": 5, "reason": "",
                            "loss_gated": False}) + "\n")
        f.write(json.dumps({"ts": 1.2, "mcs": 5, "reason": ""}) + "\n")  # pre-field
    s = mod.summarize(str(log))
    assert s["loss_gated_demotes"] == 1
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_analyze.py::test_summarize_counts_loss_gated -q`
Expected: FAIL — `KeyError: 'loss_gated_demotes'`.

- [ ] **Step 3a: Count `loss_gated` in `summarize`**

In `gs/tools/flightlog_analyze.py`, in `summarize`, change the counter-init line `predictive = reactive = warm_fallback = gated = 0` to:

```python
    predictive = reactive = warm_fallback = gated = loss_gated = 0
```

Inside the `for r in recs:` loop, after the existing `if r.get("predict_gated"):` block, add:

```python
        if r.get("loss_gated"):
            loss_gated += 1
```

In the returned dict, after `"gated_demotes": gated,` add:

```python
        "loss_gated_demotes": loss_gated,
```

- [ ] **Step 3b: Print it**

In `gs/tools/flightlog_analyze.py`, in `_print_summary`, after the `gated demotes:` print line, add:

```python
    print(f"loss-gated demotes: {s['loss_gated_demotes']}")
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_analyze.py -q`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add gs/tools/flightlog_analyze.py gs/tests/unit/test_dl_flightlog_analyze.py
git commit -m "tools: report loss-gated (suppressed) demotes in flightlog_analyze"
```

---

## Task 4: Full-suite verification

- [ ] **Step 1: Run the entire GS suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: all pass (the suite must be green as a whole — config_build/import coupling).

- [ ] **Step 2: Sanity-replay the analyzer on a recent log**

Run: `cd gs && .venv/bin/python tools/flightlog_analyze.py ../logs/000014.jsonl`
Expected: prints summaries without error, including a `loss-gated demotes:` line (0 for this pre-field log — confirms the field is tolerated when absent).

- [ ] **Step 3: No commit** (verification only).

---

## Self-Review (completed at authoring time)

- **Spec coverage:** §4.1 hysteresis → Task 2; §4.2 unify (`loss_demote`, remove `emergency_loss_rate`, `_emergency_active` fec/starved) → Task 1; §4.3 `loss_gated` → Task 2 (field) + Task 3 (analyzer); §4.4 config migration → Task 1 (drop `emergencyLossRate`) + Task 2 (add `lossWindows`); §6 testing → tests in every task + Task 4; §1 FEC threshold deferred → no task (correct).
- **Placeholder scan:** none — every code/command step is concrete.
- **Type/name consistency:** `loss_demote` (bool) consistent across `select()` (Task 1) and the `tick` call (Tasks 1→2); `loss_windows`/`lossWindows` consistent across `SelectorConfig`, `config_build`, `config_defaults`, tests; `loss_gated` consistent across `Policy.tick` record (Task 2) and analyzer `loss_gated_demotes` (Task 3); `_loss_count` defined in `__init__` (Task 2) and used in `tick` (Task 2); `_emergency_active(fec_pressure, link_starved)` new signature used only at its one call site in `select()`.
