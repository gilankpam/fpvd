# GS RSSI Normalization for Dynamic TX Power — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** EIRP-normalize the GS video-link RSSI by the drone's known per-MCS TX power so `signals.rssi` becomes distance-linear again, before the EWMA, fixing learned-prior bin corruption and the predictive-demote-vs-promote misfire.

**Architecture:** The drone now sets TX power per-MCS from a static backoff curve, so `RSSI_rx ≈ curve[mcs] − pathloss`. The GS holds a mirror of that curve and de-trends each stats window by its received MCS at the signals layer (`rssi_norm = rssi_raw + (P_ref − curve[mcs])`), *before* the EWMA smoother. Every downstream RSSI consumer (predictive-demote slope, learned-prior bins, flight log) then reads the corrected value with no per-consumer logic change. The fragile raw-RSSI cold-start seed is **removed** (the probe + learned prior cover startup), so it can't over-seed on the dynamic-power scale. GS-only — no wire/drone change. A config `enabled` flag makes normalization identity-when-off for rollback.

**Tech Stack:** Python 3 (GS service `gs/fpvdgs/`), pytest, dataclasses. Spec: `docs/superpowers/specs/2026-06-08-gs-rssi-normalization-design.md`.

**Test runner (all tasks):** `cd gs && .venv/bin/python -m pytest tests/ -q` (baseline: 262 passing). Run the whole suite after every task; run the named test for the red/green steps.

**Coupling note (must stay in sync):** The GS mirror curve `tx_power_dbm_by_mcs` MUST equal the drone's `kTxPowerDbmByMcs` in `drone/src/dynlink/txpower_curve.hpp` = `[29, 28, 25, 23, 19, 19, 19, 19]` for MCS0–7. Both are static calibration constants.

---

## File Structure

- `gs/fpvdgs/dynlink/signals.py` — **modified.** Add `RssiNormConfig` dataclass + `normalize_rssi()` pure function; add `Signals.mcs_w` and `Signals.rssi_raw` fields; add `SignalAggregator.rssi_norm` field; normalize in `consume()` before the RSSI EWMA and track `rssi_raw`.
- `gs/fpvdgs/dynlink/policy.py` — **modified.** **Remove** the RSSI cold-start (`_COLD_START_RSSI_DBM` + `coarse_mcs_for_rssi`) and simplify the warm-start to the learned prior only; record `rssi_raw` in the flight log and decision snapshot.
- `gs/fpvdgs/dynlink/config_build.py` — **modified.** Parse `tuning.rssi_norm` into `RssiNormConfig` for the aggregator.
- `gs/tests/unit/test_dl_rssi_norm.py` — **created.** Normalization math, clamps, identity-when-disabled.
- `gs/tests/unit/test_dl_signals.py` — **modified.** Add `mcs` param to the `_rx` helper (default `0` → zero offset, keeps existing RSSI assertions valid); add aggregator normalization + mixed-MCS-EWMA + `rssi_raw` tests.
- `gs/tests/unit/test_dl_policy_leading.py` — **modified.** Remove `test_cold_start_seeds_mcs_from_rssi`.
- `gs/tests/unit/test_dl_config_build.py` — **modified** (or created if absent — verify path first). `rssi_norm` config wiring tests.
- `gs/tests/unit/test_dl_policy_learned.py` — **modified.** Rewrite `test_unknown_curve_falls_back_to_cold_start` (empty store now stays at boot MCS — no seed); add the predictive-demote-misfire regression test.

---

## Task 1: `RssiNormConfig` + `normalize_rssi` pure function

**Files:**
- Modify: `gs/fpvdgs/dynlink/signals.py` (add near the top, after the imports / `WINDOW_S`, before `@dataclass class Signals`)
- Test: `gs/tests/unit/test_dl_rssi_norm.py` (create)

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_dl_rssi_norm.py`:

```python
"""Tests for EIRP RSSI normalization math (GS RSSI-norm design §Approach)."""
from __future__ import annotations

from fpvdgs.dynlink.signals import RssiNormConfig, normalize_rssi


def test_default_curve_mirrors_drone():
    # Must equal drone kTxPowerDbmByMcs in txpower_curve.hpp.
    cfg = RssiNormConfig()
    assert cfg.tx_power_dbm_by_mcs == (29, 28, 25, 23, 19, 19, 19, 19)
    assert cfg.p_ref_dbm == 29
    assert cfg.enabled is True


def test_normalize_adds_pref_minus_curve_per_mcs():
    cfg = RssiNormConfig()
    # MCS0: curve 29, offset 0 → unchanged.
    assert normalize_rssi(-60.0, 0, cfg) == -60.0
    # MCS5: curve 19, offset +10 → raised 10 dB.
    assert normalize_rssi(-70.0, 5, cfg) == -60.0
    # MCS3: curve 23, offset +6.
    assert normalize_rssi(-70.0, 3, cfg) == -64.0


def test_normalize_clamps_mcs_out_of_range():
    cfg = RssiNormConfig()
    # mcs > 7 clamps to 7 (curve 19, offset +10).
    assert normalize_rssi(-70.0, 9, cfg) == -60.0
    # mcs < 0 clamps to 0 (curve 29, offset 0).
    assert normalize_rssi(-60.0, -3, cfg) == -60.0


def test_normalize_identity_when_disabled():
    cfg = RssiNormConfig(enabled=False)
    assert normalize_rssi(-70.0, 5, cfg) == -70.0


def test_normalize_none_safe():
    cfg = RssiNormConfig()
    assert normalize_rssi(None, 5, cfg) is None
    assert normalize_rssi(-70.0, None, cfg) == -70.0
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_rssi_norm.py -q`
Expected: FAIL with `ImportError: cannot import name 'RssiNormConfig'`.

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/dynlink/signals.py`, after the `WINDOW_S = 0.1` line and before `@dataclass class Signals`, add:

```python
@dataclass(frozen=True)
class RssiNormConfig:
    """EIRP-normalization of the video-link RSSI by the drone's per-MCS TX
    power. `tx_power_dbm_by_mcs` MIRRORS the drone's kTxPowerDbmByMcs curve
    (drone/src/dynlink/txpower_curve.hpp) — both are static calibration
    constants and MUST stay in sync. When `enabled` is False, normalization
    is identity (raw RSSI), for rollback / back-compat."""
    enabled: bool = True
    p_ref_dbm: int = 29
    tx_power_dbm_by_mcs: tuple[int, ...] = (29, 28, 25, 23, 19, 19, 19, 19)


def normalize_rssi(rssi_raw, mcs, cfg: RssiNormConfig):
    """EIRP-normalize one RSSI reading: rssi_raw + (P_ref − curve[mcs]).
    Clamps mcs into the curve's index range. None-safe (returns rssi_raw
    when disabled, or when rssi_raw / mcs is None)."""
    if not cfg.enabled or rssi_raw is None or mcs is None:
        return rssi_raw
    n = len(cfg.tx_power_dbm_by_mcs)
    m = max(0, min(n - 1, int(mcs)))
    return rssi_raw + (cfg.p_ref_dbm - cfg.tx_power_dbm_by_mcs[m])
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_rssi_norm.py -q`
Expected: PASS (5 passed).

- [ ] **Step 5: Run the full suite (no regressions)**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: 267 passed (262 baseline + 5 new).

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/dynlink/signals.py gs/tests/unit/test_dl_rssi_norm.py
git commit -m "feat(gs/dynlink): RssiNormConfig + normalize_rssi pure function"
```

---

## Task 2: Normalize in `SignalAggregator.consume` before the EWMA

**Files:**
- Modify: `gs/fpvdgs/dynlink/signals.py` (`Signals` dataclass; `SignalAggregator` field; `consume`)
- Test: `gs/tests/unit/test_dl_signals.py`

- [ ] **Step 1: Add the `mcs` param to the shared `_rx` helper (test infra)**

In `gs/tests/unit/test_dl_signals.py`, change the `_rx` signature and the `RxAnt` construction so a window's received MCS is settable. **Change the default to `0`** (curve[0]=29 → zero offset → all existing RSSI assertions stay valid):

Replace the signature line:

```python
def _rx(
    ts: float,
    *,
    out: int = 0,
    lost: int = 0,
    fec_rec: int = 0,
    data: int = 0,
    bursts_rec: int = 0,
    holdoff: int = 0,
    late_deadline: int = 0,
    mcs: int = 0,
    ants: list[tuple[int, int, int, int]] | None = None,
) -> RxEvent:
```

And in the `RxAnt(...)` construction inside `_rx`, change `mcs=7,` to `mcs=mcs,`.

- [ ] **Step 2: Write the failing tests**

Append to `gs/tests/unit/test_dl_signals.py`:

```python
def test_rssi_normalized_by_received_mcs():
    """A window at MCS5 (curve 19, P_ref 29) raises signals.rssi by +10
    vs the raw value; rssi_raw keeps the measured value."""
    agg = SignalAggregator(ewma_alpha_rssi=1.0)  # no smoothing → see one window
    s = agg.consume(_rx(0.1, mcs=5, ants=[(-70, -70, 10, 10)]))
    assert s.rssi == -60.0       # -70 + (29 - 19)
    assert s.rssi_raw == -70.0   # measured, un-normalized
    assert s.rssi_max_w == -70.0


def test_rssi_norm_disabled_is_identity():
    from fpvdgs.dynlink.signals import RssiNormConfig
    agg = SignalAggregator(
        ewma_alpha_rssi=1.0, rssi_norm=RssiNormConfig(enabled=False)
    )
    s = agg.consume(_rx(0.1, mcs=5, ants=[(-70, -70, 10, 10)]))
    assert s.rssi == -70.0       # raw, unchanged
    assert s.rssi_raw == -70.0


def test_rssi_ewma_removes_power_step_across_mcs_climb():
    """Fixed distance, promote MCS0→MCS5: drone power drops 29→19 so the
    measured RSSI drops ~10 dB. Normalized signals.rssi stays flat (the
    power step is removed before the EWMA); rssi_raw shows the step down."""
    agg = SignalAggregator(ewma_alpha_rssi=0.2)
    # Window 1: MCS0 @ raw -60  → normalized -60.
    s = agg.consume(_rx(0.1, mcs=0, ants=[(-60, -60, 20, 20)]))
    assert s.rssi == -60.0
    assert s.rssi_raw == -60.0
    # Window 2: MCS5 @ raw -70 (power dropped 10) → normalized -60.
    s = agg.consume(_rx(0.2, mcs=5, ants=[(-70, -70, 12, 12)]))
    assert s.rssi == -60.0            # flat — power step removed
    assert s.rssi_raw < -60.0         # raw EWMA steps down toward -70


def test_rssi_norm_uses_best_antenna_mcs():
    """The window's MCS comes from the best (max rssi_avg) antenna."""
    agg = SignalAggregator(ewma_alpha_rssi=1.0)
    # Best antenna (rssi_avg -55) carries MCS5 → offset +10 on -55.
    s = agg.consume(_rx(0.1, mcs=5, ants=[(-55, -55, 20, 20),
                                          (-72, -70, 15, 17)]))
    assert s.rssi == -45.0   # -55 + 10
    assert s.rssi_max_w == -55.0
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_signals.py -q`
Expected: FAIL — `AttributeError: 'Signals' object has no attribute 'rssi_raw'` / TypeError on `rssi_norm=` kwarg.

- [ ] **Step 4: Implement — `Signals` fields**

In `gs/fpvdgs/dynlink/signals.py`, in the `Signals` dataclass, add `mcs_w` next to the raw `_w` fields (after `rssi_max_w`):

```python
    rssi_max_w: float | None = None       # max(rssi_avg) — best-antenna operating point
    mcs_w: int | None = None              # received MCS of the best antenna this window
```

And add `rssi_raw` next to the EWMA-smoothed `rssi` field:

```python
    # EWMA-smoothed controller inputs
    rssi: float | None = None             # EIRP-normalized (consumer-facing)
    rssi_raw: float | None = None         # EWMA of the un-normalized RSSI (observability)
```

- [ ] **Step 5: Implement — `SignalAggregator` field**

In the `SignalAggregator` dataclass, add the config field (after `starvation_threshold_pps`, before `signals`):

```python
    starvation_threshold_pps: float = 50.0
    rssi_norm: RssiNormConfig = field(default_factory=RssiNormConfig)

    signals: Signals = field(default_factory=Signals)
```

- [ ] **Step 6: Implement — capture the window MCS in `consume`**

In `consume`, in the `if ev.rx_ant_stats:` block, replace the `s.rssi_max_w = float(max(rssi_avgs))` line and capture the best antenna's MCS:

```python
        if ev.rx_ant_stats:
            rssi_mins = [a.rssi_min for a in ev.rx_ant_stats]
            rssi_avgs = [a.rssi_avg for a in ev.rx_ant_stats]
            s.rssi_min_w = float(min(rssi_mins))
            s.rssi_avg_w = float(sum(rssi_avgs) / len(rssi_avgs))
            best_ant = max(ev.rx_ant_stats, key=lambda a: a.rssi_avg)
            s.rssi_max_w = float(best_ant.rssi_avg)
            s.mcs_w = int(best_ant.mcs)
            s.ant_count = len(ev.rx_ant_stats)
```

- [ ] **Step 7: Implement — normalize before the RSSI EWMA**

In `consume`, replace the RSSI EWMA block:

```python
        if s.rssi_max_w is not None:
            s.rssi = _ewma(s.rssi, s.rssi_max_w, self.ewma_alpha_rssi)
```

with:

```python
        if s.rssi_max_w is not None:
            # Normalize per-window by the received MCS BEFORE smoothing, so a
            # promote's power drop never enters the EWMA as a fake fade.
            rssi_norm_w = normalize_rssi(s.rssi_max_w, s.mcs_w, self.rssi_norm)
            s.rssi = _ewma(s.rssi, rssi_norm_w, self.ewma_alpha_rssi)
            s.rssi_raw = _ewma(s.rssi_raw, s.rssi_max_w, self.ewma_alpha_rssi)
```

- [ ] **Step 8: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_signals.py -q`
Expected: PASS (all, including the 4 new tests).

- [ ] **Step 9: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: all green (271 passed: 267 + 4 new).

- [ ] **Step 10: Commit**

```bash
git add gs/fpvdgs/dynlink/signals.py gs/tests/unit/test_dl_signals.py
git commit -m "feat(gs/dynlink): EIRP-normalize RSSI per-window before EWMA"
```

---

## Task 3: Remove the RSSI cold-start fallback

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py` (delete `_COLD_START_RSSI_DBM` + `coarse_mcs_for_rssi`; simplify the warm-start in `Policy.tick`)
- Test: `gs/tests/unit/test_dl_policy_leading.py` (delete the obsolete cold-start test), `gs/tests/unit/test_dl_policy_learned.py` (rewrite the fallback test)

**Why:** The probe owns promotes (it safely scans `current_mcs + 1`) and the learned prior is the smart warm-start accelerant. The coarse raw-RSSI hand-table is the weakest mechanism and, under per-MCS dynamic TX power, RSSI is no longer a reliable *absolute* MCS predictor — keeping it risks over-seeding (jumping to an unsustainable MCS) on the dynamic-power scale, especially on a GS restart mid-flight when the drone is at a high MCS. Drop it: when the prior is cold, the probe simply climbs from the boot MCS (1).

- [ ] **Step 1: Update the tests to the new behavior**

In `gs/tests/unit/test_dl_policy_leading.py`, delete the `# ── Cold-start …` comment header (the `# ── Cold-start: seed MCS from link RSSI before any probe data exists ──` line) and the entire `test_cold_start_seeds_mcs_from_rssi` function.

In `gs/tests/unit/test_dl_policy_learned.py`, replace `test_unknown_curve_falls_back_to_cold_start` with:

```python
def test_unknown_curve_no_seed_stays_at_boot(tmp_path):
    # Empty store → warm-start unknown → NO RSSI cold-start seed (dropped).
    # With no probe data the MCS stays at the boot default (1); in production
    # the probe climbs from there.
    p = Policy(_cfg(tmp_path, min_samples_warmstart=100), _profile())
    dec = p.tick(_sig(-50.0))
    assert dec.mcs == 1
    p.close()
```

- [ ] **Step 2: Run tests to verify the rewrite fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py -q -k no_seed_stays_at_boot`
Expected: FAIL — currently the coarse table still seeds, so `dec.mcs == 5`, and `assert dec.mcs == 1` fails.

- [ ] **Step 3: Implement — delete the cold-start table + function**

In `gs/fpvdgs/dynlink/policy.py`, delete this entire block (the comment lines, `_COLD_START_RSSI_DBM`, and `coarse_mcs_for_rssi`):

```python
# Coarse RSSI -> initial MCS, ONLY for cold-start before probe data exists.
# Intentionally conservative; the probe takes over and refines from here.
# (Phase 4 replaces this with the learned per-card prior.)
# Floors must stay in descending order: coarse_mcs_for_rssi returns the first match.
_COLD_START_RSSI_DBM = [(-55, 5), (-65, 3), (-75, 1), (-200, 0)]


def coarse_mcs_for_rssi(rssi):
    if rssi is None:
        return 0
    for floor, mcs in _COLD_START_RSSI_DBM:
        if rssi >= floor:
            return mcs
    return 0
```

- [ ] **Step 4: Implement — simplify the warm-start in `Policy.tick`**

Replace the warm-start block:

```python
        # Warm-start seed (one-shot). Prefer the learned per-card curve; fall
        # back to the coarse hand-table when it's unknown/unconfident. Only
        # raises the boot MCS, runs before select().
        if not self._cold_started and signals.rssi is not None:
            seed = None
            if self.learned_prior is not None:
                seed = self.learned_prior.warmstart_seed(signals.rssi)
            if seed is None:
                seed = coarse_mcs_for_rssi(signals.rssi)
            if seed is not None and seed > self.leading.state.current_mcs:
                self.leading.state.current_mcs = min(seed, self.leading._cap_mcs)
                self.leading.state.tx_power_dBm = self.leading._compute_tx_power(
                    self.leading.state.current_mcs)
            self._cold_started = True
```

with:

```python
        # Warm-start seed (one-shot). Uses the learned per-card curve ONLY —
        # there is no RSSI hand-table fallback. Under per-MCS dynamic TX power
        # RSSI is not a reliable absolute MCS predictor, so when the prior is
        # cold the probe climbs safely from the boot MCS. Only raises the boot
        # MCS, runs before select().
        if not self._cold_started and signals.rssi is not None:
            seed = (self.learned_prior.warmstart_seed(signals.rssi)
                    if self.learned_prior is not None else None)
            if seed is not None and seed > self.leading.state.current_mcs:
                self.leading.state.current_mcs = min(seed, self.leading._cap_mcs)
                self.leading.state.tx_power_dBm = self.leading._compute_tx_power(
                    self.leading.state.current_mcs)
            self._cold_started = True
```

- [ ] **Step 5: Run the rewritten test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py -q -k no_seed_stays_at_boot`
Expected: PASS.

- [ ] **Step 6: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: all green (the deleted `test_cold_start_seeds_mcs_from_rssi` is gone; the fallback test now asserts boot MCS).

- [ ] **Step 7: Commit**

```bash
git add gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_policy_leading.py gs/tests/unit/test_dl_policy_learned.py
git commit -m "feat(gs/dynlink): drop fragile RSSI cold-start seed (probe + prior cover it)"
```

---

## Task 4: Wire `tuning.rssi_norm` config into the aggregator

**Files:**
- Modify: `gs/fpvdgs/dynlink/config_build.py` (`_build_aggregator`, imports)
- Test: `gs/tests/unit/test_dl_config_build.py` (verify the path first; if no such file, create it)

- [ ] **Step 1: Confirm the config-build test file path**

Run: `ls gs/tests/unit/ | grep -i config`
If `test_dl_config_build.py` exists, append to it. If not, create it with this header:

```python
"""Tests for config_build: tuning passthrough → policy/aggregator objects."""
from __future__ import annotations

from fpvdgs.dynlink.config_build import build_aggregator
```

(If the file already exists, ensure `build_aggregator` is imported.)

- [ ] **Step 2: Write the failing tests**

Append:

```python
def test_rssi_norm_defaults_enabled_full_curve():
    agg = build_aggregator({})
    assert agg.rssi_norm.enabled is True
    assert agg.rssi_norm.p_ref_dbm == 29
    assert agg.rssi_norm.tx_power_dbm_by_mcs == (29, 28, 25, 23, 19, 19, 19, 19)


def test_rssi_norm_parsed_from_tuning_block():
    block = {"tuning": {"rssi_norm": {
        "enabled": False, "p_ref_dbm": 30,
        "tx_power_dbm_by_mcs": [30, 29, 26, 24, 20, 20, 20, 20],
    }}}
    agg = build_aggregator(block)
    assert agg.rssi_norm.enabled is False
    assert agg.rssi_norm.p_ref_dbm == 30
    assert agg.rssi_norm.tx_power_dbm_by_mcs == (30, 29, 26, 24, 20, 20, 20, 20)
```

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_config_build.py -q`
Expected: FAIL — `test_rssi_norm_parsed_from_tuning_block` fails because `_build_aggregator` does not yet read `rssi_norm` (the aggregator keeps the default enabled curve regardless of the block).

- [ ] **Step 4: Implement — import `RssiNormConfig`**

In `gs/fpvdgs/dynlink/config_build.py`, extend the signals import:

```python
from .signals import RssiNormConfig, SignalAggregator
```

- [ ] **Step 5: Implement — parse in `_build_aggregator`**

Replace `_build_aggregator`:

```python
def _build_aggregator(raw: dict) -> SignalAggregator:
    s = raw.get("smoothing", {})
    starv = s.get("starvation_threshold_pps", 50.0)
    rn = raw.get("rssi_norm", {}) or {}
    rssi_norm = RssiNormConfig(
        enabled=bool(rn.get("enabled", True)),
        p_ref_dbm=int(rn.get("p_ref_dbm", 29)),
        tx_power_dbm_by_mcs=tuple(
            int(x) for x in rn.get(
                "tx_power_dbm_by_mcs", (29, 28, 25, 23, 19, 19, 19, 19))
        ),
    )
    return SignalAggregator(
        ewma_alpha_rssi=float(s.get("ewma_alpha_rssi", 0.2)),
        ewma_alpha_fec=float(s.get("ewma_alpha_fec", 0.2)),
        ewma_alpha_burst=float(s.get("ewma_alpha_burst", 0.1)),
        starvation_threshold_pps=float(starv),
        rssi_norm=rssi_norm,
    )
```

- [ ] **Step 6: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_config_build.py -q`
Expected: PASS.

- [ ] **Step 7: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: all green.

- [ ] **Step 8: Commit**

```bash
git add gs/fpvdgs/dynlink/config_build.py gs/tests/unit/test_dl_config_build.py
git commit -m "feat(gs/dynlink): parse tuning.rssi_norm into the signal aggregator"
```

---

## Task 5: Record `rssi_raw` in the flight log + decision snapshot

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py` (`Policy.tick` — `self.flightlog.write({...})` and `Decision.signals_snapshot`)
- Test: `gs/tests/unit/test_dl_policy_learned.py`

- [ ] **Step 1: Write the failing test**

First check how existing policy tests construct a `Policy` and feed a `Signals` (look at `tests/unit/test_dl_policy_learned.py` for the existing fixture/builder, e.g. a `_signals(...)` or `_policy(...)` helper). Append a test using the same construction style; the assertion is the flight-log record and snapshot carry `rssi_raw`. Template (adapt the Policy/Signals construction to match the file's existing helpers):

```python
def test_decision_snapshot_carries_rssi_raw():
    from fpvdgs.dynlink.signals import Signals
    pol = _make_policy()          # reuse this file's existing policy builder
    sig = Signals()
    sig.timestamp = 1.0
    sig.rssi = -55.0              # normalized
    sig.rssi_raw = -65.0         # measured
    sig.session = _make_session()  # reuse existing session helper if needed
    decision = pol.tick(sig)
    assert decision.signals_snapshot["rssi"] == -55.0
    assert decision.signals_snapshot["rssi_raw"] == -65.0
```

> NOTE for the implementer: if `test_dl_policy_learned.py` lacks reusable `_make_policy`/`_make_session` helpers, instead add this assertion onto an existing passing test in that file that already builds a `Policy` and calls `tick` — set `sig.rssi_raw` on its input `Signals` and assert the snapshot key. The behavior under test is purely "the `rssi_raw` field is threaded into the snapshot + flight-log dict."

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py -q -k rssi_raw`
Expected: FAIL — `KeyError: 'rssi_raw'`.

- [ ] **Step 3: Implement — add `rssi_raw` to the flight-log write**

In `Policy.tick`, in the `self.flightlog.write({...})` dict, add the `rssi_raw` key after `"rssi"`:

```python
        self.flightlog.write({
            "ts": signals.timestamp,
            "rssi": signals.rssi,
            "rssi_raw": signals.rssi_raw,
            "mcs": new_mcs,
            "reason": reason,
            "residual_loss_w": signals.residual_loss_w,
            "fec_work": signals.fec_work,
            "link_starved": sustained_starved,
            "ceiling": (self.learned_prior.ceiling(signals.rssi)
                        if self.learned_prior and signals.rssi is not None else None),
        })
```

- [ ] **Step 4: Implement — add `rssi_raw` to the decision snapshot**

In the `return Decision(...)` `signals_snapshot` dict, add `rssi_raw` after `"rssi"`:

```python
            signals_snapshot={
                "rssi": signals.rssi,
                "rssi_raw": signals.rssi_raw,
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "link_starved": sustained_starved,
                "mcs": new_mcs,
            },
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py -q -k rssi_raw`
Expected: PASS.

- [ ] **Step 6: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: all green (277 passed).

- [ ] **Step 7: Commit**

```bash
git add gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_policy_learned.py
git commit -m "feat(gs/dynlink): record rssi_raw in flight log + decision snapshot"
```

---

## Task 6: Predictive-demote misfire regression test

**Files:**
- Test only: `gs/tests/unit/test_dl_policy_learned.py` (no production code — this proves the consumer behaves correctly on the de-trended scale)

**What it proves:** Given the same learned prior, a de-trended (flat) RSSI input does NOT produce a predictive demote, whereas the raw (stepped-down on a promote) input WOULD. `predictive_ceiling` is the input to the demote decision (`pc < current_mcs` fires the demote); we assert directly on it.

- [ ] **Step 1: Write the test**

Append to `gs/tests/unit/test_dl_policy_learned.py`:

```python
def test_predictive_demote_does_not_misfire_on_detrended_rssi():
    """Promote MCS->MCS+ drops the drone's power → raw RSSI steps down →
    negative raw slope → predictive_ceiling at the projected raw RSSI lands
    in a low-ceiling bin and would demote. After EIRP-normalization the RSSI
    is flat (slope ~0), so predictive_ceiling stays at the operating ceiling
    and does NOT demote."""
    from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig

    cfg = LearnedPriorConfig(enabled=True, persist_dir="/nonexistent-no-write")
    lp = LearnedPrior("test-misfire", cfg)

    def prime(rssi_lo, ceiling, n=50):
        b = lp.rssi_bin(rssi_lo)
        for rung in range(ceiling + 1):
            lp._cells[b][rung] = [1.0, float(n)]

    # Low-RSSI region tops out at MCS1; high-RSSI region supports MCS5.
    prime(-80.0, 1)
    prime(-50.0, 5)

    current_mcs = 5
    # Raw path: rssi -62, slope -6/tick → projected -80 → ceiling 1 < 5 → DEMOTE.
    pc_raw = lp.predictive_ceiling(-62.0, -6.0)
    assert pc_raw == 1
    assert pc_raw < current_mcs            # raw RSSI WOULD have demoted

    # Normalized path: rssi -50, slope 0 (power step removed) → projected -50
    # → ceiling 5, not below current → NO demote.
    pc_norm = lp.predictive_ceiling(-50.0, 0.0)
    assert pc_norm == 5
    assert not (pc_norm < current_mcs)     # normalized RSSI does NOT demote
```

- [ ] **Step 2: Run the test**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py -q -k misfire`
Expected: PASS (this is a behavioral assertion on already-shipped `predictive_ceiling`; it documents and locks the fix's payoff).

- [ ] **Step 3: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: all green (278 passed).

- [ ] **Step 4: Commit**

```bash
git add gs/tests/unit/test_dl_policy_learned.py
git commit -m "test(gs/dynlink): predictive-demote no longer misfires on de-trended RSSI"
```

---

## Rollout / Migration (post-merge, GS-only — NOT a code task)

Perform after the branch merges and is ready to deploy. These are operational steps; do not run them during implementation.

1. **Deploy:** `./deploy/gs/deploy.sh --host 10.18.0.1`
2. **Reset the learned store** — it has been ingesting raw RSSI under dynamic power and is corrupted; it auto-rebuilds empty:
   ```bash
   ssh root@10.18.0.1 'rm -f /etc/fpvd/learned/*.json'
   ```
3. **Widen the learned-prior bin range** so the normalized (P_ref=29) scale, which shifts values up by up to +10 dB, stays in range. In the live `/etc/fpvd/config.json`, under `dynamicLink.tuning.learned_prior`, set `rssi_max` to `-20` (default is `-30`). This is additive — **do NOT clobber the existing tuned probe knobs** (`rxL=800`, `gate.probe_viable_threshold=0.85`, etc.). A changed bin signature also forces the clean rebuild we want.
4. **Hardware verify:**
   - Predictive-demote no longer fires on a promote (watch the flight log `reason` field for `predict_demote` immediately following a promote — should be gone).
   - Learned-prior bins fill on the normalized scale (check `/status` learned-prior bins populate at the shifted RSSI values).
   - Flight-log records carry both `rssi` (normalized) and `rssi_raw` (measured), differing by `(29 − curve[mcs])`.

---

## Self-Review (completed during planning)

**Spec coverage:**
- §Components 1 (config `tuning.rssi_norm`) → Task 4. ✅
- §Components 2 (`SignalAggregator.consume` normalize before EWMA, `rssi_raw`, None-safe) → Tasks 1–2. ✅
- §Components 3 (consumers unchanged) → no code change; verified `signals.rssi` is the single seam (Task 2). ✅
- §Components 4 (cold-start table shift) → **superseded by an approved design decision:** Task 3 *removes* the RSSI cold-start entirely (Option B) rather than shifting it onto the normalized scale. Rationale: the probe owns promotes and the learned prior is the smart warm-start; the raw-RSSI hand-table is fragile under dynamic power and risks over-seeding. ✅
- §Components 5 (flight log + snapshot `rssi` + `rssi_raw`) → Task 5. ✅
- §Migration (reset store, widen bins) → Rollout section. ✅
- §Edge cases (clamp MCS, mixed-MCS per-window-before-EWMA, curve sync) → Tasks 1–2 + coupling note. ✅
- §Testing (norm math, +10 @ MCS5, mixed-MCS EWMA flat, predictive no-misfire, flight log both, identity-when-disabled) → Tasks 1, 2, 5, 6. ✅

**Design decision (post-spec, approved by user):** The spec's §Components 4 cold-start *table shift* is replaced by *removing* the RSSI cold-start (Task 3). The probe (safe `current_mcs+1` scan) and the learned prior cover startup; on a cold prior the probe climbs from the boot MCS (1). This also drops the need for a `PolicyConfig.rssi_normalized` flag.

**Design deviation noted:** The spec's §Testing line "feed windows at MCS0 then MCS5 at the *same* raw RSSI" is loose wording — at the *same* raw RSSI the normalized value would *step*, not stay flat. The physically meaningful invariant (and what the feature exists for) is: at a *fixed distance*, a promote drops power so raw RSSI drops ~10 dB while normalized stays flat. Task 2's `test_rssi_ewma_removes_power_step_across_mcs_climb` implements this correct interpretation (MCS0@-60 → MCS5@-70 → normalized flat at -60).

**Design deviation noted:** Spec §Migration frames the bin-range widening as operational. To keep `enabled=false` (the normalization path) identity, the learned-prior `rssi_max` default is left unchanged in code and widened via live config at rollout (Rollout step 3) rather than changing the code default.

**Placeholder scan:** none — every code step shows complete code; every test step shows complete test code. ✅

**Type consistency:** `RssiNormConfig` (fields `enabled`/`p_ref_dbm`/`tx_power_dbm_by_mcs`), `normalize_rssi(rssi_raw, mcs, cfg)`, `Signals.mcs_w`/`Signals.rssi_raw`, `SignalAggregator.rssi_norm` are used identically across all tasks. No `coarse_mcs_for_rssi` / `rssi_normalized` remain (Task 3 removed them). ✅
