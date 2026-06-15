# Predictive-Demote Flapping Fix Design

**Date:** 2026-06-15
**Status:** Draft for review
**Target:** GS `fpvdgs.dynlink` (GS-local; no drone change, no wire change)
**Refines:** `2026-06-07-phase4-learned-rssi-prior-design.md` (the learned RSSI→ceiling prior + predictive demote)
**Diagnosed from:** flight logs `000010.jsonl` (5.2 min) + `000012.jsonl` (2.9 min)

---

## 1. Purpose & scope

The learned-prior **predictive demote** flaps the link. Across two flights it demotes
the operating MCS, the probe immediately re-promotes to the same rung, and the cycle
repeats roughly once per second. This degrades the very thing it exists to protect —
a stable, high link.

This is a **pure GS-side trigger-logic fix** to the predictive-demote path in
`policy.py` + `learned_prior.py`. The v3 `{mcs}`-only wire, the drone, the probe, the
reactive demote, warm-start, and the learned-prior *data* are all unchanged. The
learned ceilings are fine; the bug is in *when* we act on them.

**Out of scope (noted follow-up):**
- **The sample count never decays.** `LearnedPrior._update` does `cell[1] += 1.0`
  with no decay or cap, despite the module docstring claiming "a decaying sample
  count." Once a bin crosses `min_samples_predictive` it is "confident" forever, so
  stale ceilings (antenna swap, new location, interference) cannot age out and the
  adapting `clean_ewma` is the only corrective signal. Real latent issue, recorded
  here, **not addressed in this change**.

## 2. Evidence (the diagnosis)

Both flights, same lens (`gs/tools/flightlog_analyze.py` + ad-hoc):

| metric | 000010 | 000012 |
|---|---|---|
| duration / cadence | 5.2 min / 9.99 Hz | 2.9 min / 10.0 Hz |
| MCS changes | 262 (50/min) | 191 (65/min) |
| predict_demotes | 97 | 61 |
| …fired with **video clean** (`residual_loss_w < 1%`, not starved) | 93% | 85% |
| …fired with **slope ≥ 0** (RSSI flat/rising) | 34% | 39% |
| …**re-promoted to same rung ≤ 3 s** (the flap loop) | 82% | 80% |
| a least-squares slope would neutralize (near-flat projection) | 69% | 62% |

Two independent root causes:

1. **Noisy slope.** The trigger uses `slope = signals.rssi − self._prev_rssi` (a
   single-tick delta) projected `× predictive_horizon_ticks` (3). A lone noisy tick
   is amplified ×3 across a rung boundary. ~65% of demotes are this jitter.

2. **Static enforcement, not fade prediction.** The trigger fires whenever
   `pc < cur` (`policy.py:305`) **regardless of slope sign**. When the probe has
   legitimately climbed above the prior's static learned ceiling, the prior demotes
   even at flat/rising RSSI — a steady-state fight between prior and probe that *no*
   slope smoothing can fix (34–39% of demotes, slope already ≥ 0).

Supporting findings:
- The flap loop (demote → probe re-promote ≤ 3 s) is reproducible at ~81% in both flights.
- The 392 "starved" ticks in `000010` are a **landing artifact** (all at −11 dBm / MCS 0
  at end of flight, only 3% follow an MCS change) — not link failure, not flap-induced.
- Even *genuine* downtrend demotes (least-squares projected drop ≥ 1 dB) are followed by
  real degradation only ~40–47% of the time → predictive demote is inherently imprecise,
  so the gate must be conservative and the reactive path stays the true safety net.
- **Emptying the prior is not a fix:** with zero samples `predictive_ceiling` returns
  `None` (no rung has `n ≥ min_samples_predictive`), so predictive demote is dormant —
  but `ingest()` refills the bins in ~4 s of dwell each (and the isotonic ladder
  extrapolates from the first confident bin), so flapping returns within tens of seconds,
  minus the warm-start benefit.

## 3. Locked design decisions (from brainstorming)

- **Fix in the trigger, keep the feature.** Predictive demote stays a pre-fade
  accelerant; we de-noise its input and gate it to fire only on a genuine downtrend.
- **Least-squares slope** over a rolling RSSI window — the only estimator that rejects a
  lone spike (vs EWMA-the-delta, which leaks it proportionally, or an N-tick baseline,
  which is at the mercy of its two endpoint samples).
- **Slope-direction gate** — require a real projected drop, not just `pc < cur`.
- **Knobs are code constants** in `LearnedPriorConfig` defaults (the learned-prior block
  is `# frozen: always-on, internal defaults` in `config_build.py`; not exposed in
  `config.json`).
- **Count-decay deferred** (§1).

## 4. The changes

### 4.1 Least-squares slope (Change 1)

Replace the single-tick delta in `Policy.tick` with a least-squares gradient over a
rolling window of recent smoothed RSSI.

- **State:** replace `Policy._prev_rssi: float | None` with a fixed-size buffer,
  `collections.deque(maxlen=predictive_slope_window_ticks)`, appended each tick that
  has a non-`None` `signals.rssi`.
- **Estimator:** a pure module-level helper
  `lsq_slope(samples: Sequence[float]) -> float` returning dBm/tick. Evenly-spaced x
  (0,1,…,n−1) → closed-form `Σ(x−x̄)(y−ȳ) / Σ(x−x̄)²`. `< 2` samples → `0.0`.
- **Config:** `LearnedPriorConfig.predictive_slope_window_ticks: int = 10` (1.0 s @ 10 Hz).
- **Logging:** the computed slope continues to be logged under the existing `slope`
  flight-log key, so `flightlog_analyze.py` is unaffected.

Helper home: the pure function lives next to the prediction math it feeds
(`learned_prior.py`); the rolling buffer stays in `Policy` (where `_prev_rssi` is today).

### 4.2 Slope-direction gate (Change 2)

Demote only when a genuine fade is projected. The current condition
(`policy.py:305`):

```python
if pc is not None and pc < cur:
    self._predict_demote_count += 1
    ...
else:
    self._predict_demote_count = 0
```

becomes:

```python
projected_drop = -slope * self.cfg.learned_prior.predictive_horizon_ticks
if pc is not None and pc < cur and projected_drop >= self.cfg.learned_prior.predictive_min_drop_db:
    self._predict_demote_count += 1
    ...
else:
    self._predict_demote_count = 0
```

- **Config:** `LearnedPriorConfig.predictive_min_drop_db: float = 1.0` — minimum
  projected RSSI drop over `predictive_horizon_ticks` to permit a demote (≈ slope ≤
  −0.33 dBm/tick ≈ −3.3 dB/s; from the "genuine downtrend" cut in both logs).
- The debounce (`predictive_debounce_windows = 3`) now counts consecutive ticks meeting
  **both** conditions; any non-qualifying tick resets it via the existing `else`.

This eliminates the 34–39% flat/rising demotes outright; with §4.1 it leaves only
genuine-downtrend demotes (~20% of today's volume).

### 4.3 Diagnostics

Add one flight-log field, `predict_gated: bool` — `True` when `pc is not None and
pc < cur` but `projected_drop < predictive_min_drop_db` (the gate suppressed a demote
the old code would have issued). Lets the next flight directly quantify suppressed
demotes and validate the fix in the field. Fits the existing
`test_dl_flightlog_debug_fields.py`.

## 5. What stays the same

`predictive_ceiling`, `_confident_ceiling`, the isotonic ladder, the prior data on
disk, `warmstart_seed`, the probe, `LeadingSelector` promote/demote, and the reactive
emergency/video-PER path are all unchanged. No wire change, no drone change, no
`config.json` schema change (new fields are frozen dataclass defaults).

## 6. Testing (TDD)

**Pure `lsq_slope` (`test_dl_learned_prior.py` or a new module test):**
- flat sequence → `0.0`
- linear ramp (e.g. −0.5/tick) → exact gradient
- linear ramp + one large spike inside a 10-window → near the ramp gradient (robustness)
- `< 2` samples → `0.0`

**Gate in `Policy.tick` (`test_dl_policy_learned.py`):**
- flat/rising RSSI with `pc < cur` → **no** predictive demote (regression for the 34–39% case)
- sustained downtrend past `predictive_min_drop_db` with `pc < cur` → demote after `predictive_debounce_windows`
- downtrend just under threshold → no demote
- `predict_gated` set when `pc < cur` but the gate blocks

**Suite:** update existing `test_dl_policy_learned.py` assertions that depend on the old
single-tick slope; the whole GS suite (`config_build`/import coupling) must stay green.

## 7. Defaults summary

| knob (`LearnedPriorConfig`) | default | meaning |
|---|---|---|
| `predictive_slope_window_ticks` | `10` | least-squares window (1.0 s @ 10 Hz) |
| `predictive_min_drop_db` | `1.0` | min projected RSSI drop over the horizon to allow a demote |
| `predictive_horizon_ticks` (existing) | `3` | projection horizon (unchanged) |
| `predictive_debounce_windows` (existing) | `3` | consecutive qualifying ticks before demote (unchanged) |
