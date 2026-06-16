# Learned-Prior Knee-Model Redesign

**Date:** 2026-06-16
**Status:** Draft for review
**Target:** GS `fpvdgs.dynlink` (`learned_prior.py`, with thin `policy.py` / `config_build.py` edits). GS-local; no drone change, no wire change.
**Supersedes:** the binned RSSI→ceiling table (`learned_prior.py` as of `main`) and the abandoned `fix/learned-prior-low-rssi` extrapolation patch.
**Refines:** `2026-06-15-predictive-demote-flapping-fix-design.md` (keeps its least-squares slope + slope-direction gate; resolves its "sample count never decays" follow-up).
**Diagnosed from:** flight `000013.jsonl` + the persisted `m8812eu2.json` prior.

---

## 1. Purpose & scope

The learned prior exists to drive the **down-only predictive demote**: as RSSI fades, lower the operating MCS *before* the link eats loss. In flight `000013` it failed exactly when needed — during a RSSI fade from −69 to −80 dBm the prior's ceiling stayed pinned at 4 the whole way down, the predictive demote never fired, and the rung came down only reactively by eating 0.4–0.9 loss bursts at each step.

Root cause is the **learning model**, not a single bug:

1. **No recency.** `_update` does `cell[1] += 1.0` — the "decaying sample count" the docstring promises was never implemented. Cells learned once stay confident forever; cells no longer visited freeze. The prior cannot track recent flights.
2. **Selection bias.** A rung's cell only fills when the controller parked the link there — low rungs mid-fade/mid-demote (dirty), high rungs near the GS (clean) — producing physically-impossible non-monotonicity (MCS4 reads *cleaner* than MCS2 at the same RSSI). Empirically, the prior records MCS1 at −82 dBm as 85% clean while the flight shows it was 99.4% clean — the gap is **arrival-transient contamination** (the just-after-demote tick where loss is still draining is recorded against the new rung).
3. **Probe-optimism contamination.** The FEC-off observe-only probe feeds `probe_clean` votes into the cells, inflating high rungs.
4. **Flat low-RSSI extrapolation.** Below the lowest confident anchor the ceiling held flat (`ladder[0][1]`) instead of dropping — the read-side symptom the `fix/learned-prior-low-rssi` band-aid targeted.

This redesign replaces the binned table with a **per-rung RSSI-knee model** that is monotone by construction, recency-weighted, and learns only from settled stretches — so a bad prior self-heals over flights and a deleted one relearns correctly.

**FPV operating context (shapes the learning rule):** RSSI swings rapidly — drone goes behind trees, egresses fast, enters buildings. On the way down the link eats loss at *every* rung faster than it can demote; those losses are **transition artifacts, not rung-unviability at that RSSI**. The model must not learn from them.

**Out of scope (explicit, so we don't over-claim):**
- **Co-channel interference bursts at good RSSI.** Not RSSI-predictable; the reactive demote remains the handler. The demote-settle freeze stays removed.
- **Instant drops** (tree at speed) outrun any slope predictor; reactive demote is the backstop.
- **Keyed on RSSI, not SNR.** The user flies one location, so noise-floor invariance is unnecessary (YAGNI). EIRP-normalized RSSI is kept.

## 2. The model

Replace the 30×8 clean-rate table with **one learned RSSI knee per MCS rung**:

- `knee[K]` — the EIRP-normalized RSSI below which rung K stops carrying clean video **in steady state**; `None` until learned.
- `count[K]` — a confidence counter that **decays** with observations (recency). Knee K is *confident* once `count[K] >= min_samples`.

**Invariants enforced by construction** (selection-bias non-monotonicity becomes impossible):
- **Monotone in rung:** effective knees are non-decreasing in K (a higher rung never needs less RSSI). Enforced on read via cumulative-max over confident knees.
- **Derived ceiling:** `ceiling(rssi)` = highest confident K with `knee[K] <= rssi`, defined for *all* RSSI — no extrapolation holes, no `ladder[0][1]`. `None` when no knee is confident.

Persisted state is ~8 knees + 8 counts (schema v2).

## 3. Learning rule

**When (the FPV-critical gate):** ingest a sample only when the **operating rung has been unchanged for `settle_ticks`** (loss from the last change has drained). This discards behind-a-tree / fast-egress transients. RSSI is *allowed* to be moving — only the rung must be settled — so the model still learns abundantly while cruising. During a demote cascade the rung is never settled, so nothing is learned (correct).

**How** (settled sample at rung K, RSSI r, `clean = residual_loss_w < viable_loss`):
- **clean** → K provably works at r ⇒ knee should be `<= r`. If `knee[K] > r`, pull **down slowly** by `alpha_relax`. First-ever sample seeds `knee[K] = r`.
- **dirty** → K provably fails at r even settled ⇒ knee should be `>= r`. If `knee[K] < r`, pull **up fast** by `alpha_tighten`. First-ever sample seeds `knee[K] = r`.

The pull is a single EWMA step `knee[K] += alpha * (r - knee[K])`, applied only on the "wrong side" (clean with `r < knee`, or dirty with `r > knee`); a consistent sample (clean above / dirty below the knee) leaves the knee put. `alpha_tighten > alpha_relax` — **pessimistic asymmetry**: quick to distrust a rung that fails, slow to re-trust it.

Every settled ingest at rung K bumps `count[K]` (building confidence) and applies recency: counts are scaled by `recency_decay` (<1) per observation so old evidence fades and a stale/bad knee that stops being reinforced ages back below `min_samples` and self-heals. The pulls themselves are EWMA, so recency is also built into the knee value.

**Probe votes are dropped from learning** (defect 3). The probe stays a promote gate, not a teacher.

## 4. Query & policy integration

The public interface is unchanged, so `policy.tick` barely changes:

- `ceiling(rssi)` — highest confident knee `<= rssi`; `None` if none confident.
- `predictive_ceiling(rssi, slope)` = `ceiling(rssi + slope * predictive_horizon_ticks)`.
- `warmstart_seed(rssi)` = `ceiling(rssi)` (one-shot boot seed).
- `ingest(rssi, operating_mcs, operating_clean, settled)` — new signature (drops `probed_rung`/`probe_clean`; adds `settled`, computed in `policy.tick` from the selector's last-change time).

The predictive-demote block in `policy.tick` is **unchanged**: `pc = predictive_ceiling(rssi, slope); if pc is not None and pc < cur: (slope-gate + debounce) → demote toward pc`. This stays correct because monotonicity makes `ceiling(projected) < cur` *positive evidence* that `cur` is unviable (a confident lower knee above the projected RSSI implies the higher rung fails too). The PR-#19 least-squares slope + slope-direction gate + debounce are preserved verbatim — they protect against false demotes on noisy slope and on flat-RSSI prior-vs-reality disagreement.

## 5. Config & persistence

`LearnedPriorConfig` rewritten:
- **New:** `settle_ticks` (≈5), `alpha_tighten` (≈0.25), `alpha_relax` (≈0.05), `min_samples`, `recency_decay`, `viable_loss` (reuse selector `video_demote_per`=0.05 semantics).
- **Kept:** `predictive_horizon_ticks`, `predictive_slope_window_ticks`, `predictive_min_drop_db`, `predictive_debounce_windows`, `persist_dir`, `flush_interval_observations`.
- **Deleted:** `bin_width_db`, `rssi_min`, `rssi_max`, `viable_threshold`, `ewma_alpha`, `min_samples_warmstart`/`min_samples_predictive` split, `warmstart_margin` (and the never-shipped `extrapolation_db_per_rung`).

`config_build` exposes the learning knobs (`settleTicks`, `alphaTighten`, `alphaRelax`, `minSamples`, `recencyDecay`) for in-flight tuning, replacing the lone `extrapolationDbPerRung`. The rest of the learned-prior block stays frozen.

**Persistence:** schema v2 = `{schema: 2, key, knees: [...], counts: [...]}`. On load, any non-v2 file (the v1 bin table) is ignored and rebuilt — the existing `m8812eu2.json` retrains from scratch. Deploy path unchanged.

**Flightlog:** keep `ceiling` / `pc`; add a compact `knees` snapshot and a `prior_learn` boolean (did this tick feed learning) so knee formation and transient-rejection are visible offline. `flightlog_analyze.py` gains a knee summary.

## 6. Components & boundaries

- **`KneeModel`** (new, in `learned_prior.py`) — owns `knees`/`counts`, the asymmetric-pull update, monotone `ceiling`, decay, and v2 persistence. Pure, unit-testable in isolation.
- **`LearnedPrior`** — thin facade keeping the `ceiling`/`predictive_ceiling`/`warmstart_seed`/`ingest`/`flush`/`to_status` interface `policy.py` depends on; delegates to `KneeModel`. `lsq_slope` stays here (model-agnostic util).
- **`Policy`** — computes `settled` and passes it to `ingest`; predictive-demote logic unchanged.
- **`config_build`** — maps the new camelCase knobs.

## 7. Testing (TDD)

**KneeModel units:** clean lowers / dirty raises the knee; `alpha_tighten > alpha_relax` asymmetry; monotonicity (cumulative-max) holds even after an out-of-order inversion; confidence gating; recency decay ages out a stale knee; **learn-gate rejects unsettled-rung samples** (transient-rejection guarantee); `ceiling`/`predictive_ceiling` correctness incl. `None` when cold; v2 persistence round-trip and v1-ignored/corrupt-ignored.

**Policy integration:** predictive demote fires on a projected fade below the current knee, still slope-gated + debounced; no learning mid-cascade (rung changing); warmstart seeds from knees; cold prior → no predictive demote (probe owns it).

**Offline validation:** train `KneeModel` on `000013`'s settled samples and confirm it would have predictive-demoted on the −69→−80 fade (the originating failure).

The existing bin-based `test_dl_learned_prior.py` is largely replaced — expected for a redesign. Policy/flightlog tests touching `pc`/`ceiling`/`ingest` are updated.

## 8. Risks

- **Knee learning-rate tuning.** `alpha_tighten`/`alpha_relax`/`settle_ticks` need in-flight tuning; exposed as config knobs. Pessimistic defaults bias safe (demote a touch early rather than late).
- **Cold-start gap.** Until knees are confident, predictive demote is silent and the probe + reactive demote carry the link (same as today's cold prior). Acceptable.
- **One-rung-per-fade pacing.** Predictive demote steps one rung at a time with debounce; a very steep fade may need a couple of ticks to walk down. The reactive demote remains the fast backstop.
