# Phase 4 — Learned RSSI→Ceiling Prior + Flight Logging Design

**Date:** 2026-06-07
**Status:** Draft for review
**Target:** GS `fpvdgs.dynlink` (GS-local; no drone change, no wire change)
**Refines:** `2026-06-06-probe-driven-link-control-design.md` §4.8 (the learned RSSI/SNR prior)
**Builds on:** Phase 2 (the GS `LeadingSelector`) + Phase 3a/3b (drone-local compute, the `{mcs}`-only v3 wire) — all deployed + hardware-validated 2026-06-07.

---

## 1. Purpose & scope

Replace the hand-authored cold-start RSSI table (`coarse_mcs_for_rssi` / `_COLD_START_RSSI_DBM`) with a **learned, per-card `link-RSSI → viable-ceiling-MCS` curve**, accumulated **passively** across flights, that:

- **warm-starts** the operating MCS at `dynamicLink` enable (instead of climbing from MCS 1 — a ~15–20 s climb observed live), and
- **predictively demotes** ahead of an RSSI fade (before the reactive video-PER/emergency path catches it).

The learned curve is a **prior / accelerant, never the authority**: the live probe still gates every promote and the reactive Channel-B demote still backstops everything. RSSI is blind to interference (which is *why* the SNR-floor selector was abandoned, parent §2.2); the probe sees it. When the curve is unknown or unconfident, the selector behaves **exactly as it does today**.

This phase also adds **structured per-flight logging + an offline analysis script** so the curve and the selector knobs can be understood and hand-tuned from real flights.

**Pure GS-side addition.** The v3 wire stays `{mcs}`-only; the drone is unchanged. The drone already publishes `radio.adapterId` in `/status`, used here only as an opportunistic logged cross-check.

**Out of scope (deferred):**
- **GS auto-profile-select** (parent §4.8: drone `adapter_id` → GS auto-picks `radioProfile`, default `"auto"`). The operator still hand-sets `radioProfile`; we key the learned curve by it. Auto-select is a small later pass, worth doing once a second card profile exists.
- **Probe-pps reduction** (parent §4.8 "lower probe pps when confident"). It would require GS→drone control of the probe rate — the v3 wire is `{mcs}`-only by design — and the probe is < 1 % airtime at 20 MHz, so the payoff doesn't justify reintroducing that control path. Revisit as its own phase only if probe airtime ever proves worth reclaiming.
- The Phase-4 "higher-pps calibration pass" idea (parent §5) — we chose **pure passive** accumulation.

## 2. Locked design decisions (from brainstorming)

- **GS-local, no wire/drone change.** Warm-start changes the *initial* `{mcs}`; predictive-demote *lowers* the commanded `{mcs}` — both already carried by the v3 wire.
- **Keyed by `radioProfile`** (operator-set, always available, no `/air` dependency — `/air` was `droneReachable:false` during the 3b flag-day). The drone's `radio.adapterId` is read **opportunistically** when `/air` is reachable, purely to **log a warning** on mismatch with `radioProfile`. Never a hard dependency.
- **Pure passive calibration** — learn only from normal operation; no deliberate probing. The curve sharpens flight-over-flight.
- **Representation: binned viability table** (`{clean_ewma, n}` per `(RSSI bin, rung)`) + a **derived isotonic floor ladder** for extrapolation into unflown RSSI. (Not a parametric-only ladder — its floor-from-noise fit is the fragile part; not regression — overkill.)
- **Two uses only: warm-start + predictive-demote.** Probe-savings dropped (see §1).
- **Confidence-gated everywhere.** Each use is off until its RSSI region has enough samples; both stay subordinate to the live probe + the reactive demote.
- **Drift, not invalidation.** Recency-weighted updates pull the curve toward new reality (antenna swap, environment) without explicit invalidation.

## 3. The model

Per card (= per `radioProfile`), in memory and persisted to one JSON file:

- **Bins:** link-RSSI bucketed into fixed `bin_width_db` (default **2 dB**) bins over `[rssi_min, rssi_max]` (default ≈ −90…−30 dBm → ~30 bins).
- **Cells:** each `(bin, rung)` holds `clean_ewma` (recency-weighted clean-rate, clean = 1 / cliff = 0) and `n` (a decaying sample count → confidence). ≈ 30 bins × 8 rungs × 2 numbers ≈ a few hundred values, a few-KB JSON, dumpable in `/status`.

**Rung-monotonicity** carries the model: a lower MCS is strictly more robust than a higher one, so `clean` at rung R implies all rungs `< R` are clean at that bin. We therefore only need the *boundary* observations the probe (`current+1`) and the operating rung naturally provide — never an exhaustive per-rung sweep.

**Derived quantities:**
- `ceiling(bin)` = highest rung with `clean_ewma ≥ viable_threshold` **and** `n ≥ min_samples`; `UNKNOWN` if none qualify.
- `confidence(bin)` = whether `n` at/around the ceiling clears the use-specific threshold (`min_samples_warmstart` vs the stricter `min_samples_predictive`).
- **Derived floor ladder:** an isotonic (monotone-in-RSSI) step function `rssi_floor[rung]` fit over the per-bin ceilings. It (a) **denoises** by enforcing "higher RSSI ⇒ ≥ ceiling" and (b) answers `ceiling(RSSI)` for an RSSI not directly/confidently binned (extrapolation). Query resolution: confident bin → its `ceiling`; else the derived ladder; else `UNKNOWN` (→ today's behavior).

## 4. Observations (what gets logged each tick)

Both sources already exist in the selector loop — no new measurement, just ingestion:

1. **Probe rung** (the boundary): the probe sits at `current+1`; its verdict (`clean = 1−per ≥ probe_viable_threshold`, the selector's existing criterion) updates cell `(bin, current+1)`.
2. **Operating rung** (health): video running at the operating MCS is a live `clean` label for `(bin, operating_mcs)`; a **demote event** (video-PER / emergency) flips it to a `cliff` for that cell, exactly when the rung fails.

Update per observed cell: `clean_ewma ← α·verdict + (1−α)·clean_ewma` (default `α ≈ 0.1`); `n` incremented with decay (recency). Ticks with invalid/missing RSSI are not ingested.

## 5. Use 1 — warm-start (one-shot, at enable)

On the first tick after `dynamicLink` enable, query the curve at the current link-RSSI → `seed = ceiling(RSSI)`. Set the operating MCS to `clamp(seed − warmstart_margin, mcs_min, maxMcs)` (default `margin = 0`; a 1-rung cushion is tunable). `mcs_min`/`maxMcs` are the selector's existing rung bounds (the profile row table's `mcs_min` and the `dynamicLink.maxMcs` cap) — note Phase 3b deleted the GS-side `PolicyConfig.safe`, so the low bound is the selector's `mcs_min`, not a `safe` floor. The live probe then validates `current+1` and takes over exactly as today.

**Guards:** seeds only if the region is confident (`n ≥ min_samples_warmstart`); otherwise falls back to today's `coarse_mcs_for_rssi` (now the *unconfident* fallback, not the primary). Never seeds above `maxMcs` or below `mcs_min`. After the seed, the live selector owns the MCS. Worst case — a too-high seed under interference RSSI can't see — the reactive demote corrects it within a window; strictly no worse than a too-high manual MCS today.

## 6. Use 2 — predictive demote (each tick, down-only)

Smooth RSSI (short EWMA + slope). If `ceiling(projected-RSSI) < operating_mcs` in a **confident** region, pre-demote the operating MCS toward that ceiling — before video-PER or the probe (which sits *above* the operating rung) would catch the operating rung degrading.

**Guards** (RSSI is single-sample-noisy at ~20 pps, so deliberately conservative):
- confident region required (`min_samples_predictive`, stricter than warm-start);
- **debounce** — condition sustained `predictive_debounce_windows` consecutive ticks (default 3);
- **down-only** — predictive can never *raise* MCS (promotion stays the live probe's job);
- never below the selector's `mcs_min`;
- the existing **reactive Channel-B demote (video-PER / emergency) is unchanged and remains the backstop** — predictive is an *additional, earlier* trigger;
- on RSSI recovery the probe re-promotes normally.

Net: trims the brief glitch on a fast fade (where reactive fires *after* loss starts), at the cost of an occasional early demote on a confident cliff, bounded by the guards.

## 7. Persistence & keying

- **Store:** one JSON file per card at `<persist_dir>/<radioProfile>.json` (default `persist_dir = /etc/fpvd/learned/` — the `/etc/fpvd` overlay persists across reboots; configurable). Loaded on `dynamicLink` enable; flushed periodically (`flush_interval`) and on disable; written atomically (temp + rename).
- **Versioning:** the file carries a schema version + the bin config (`bin_width_db`, `rssi_min/max`). A mismatch → the stale file is discarded and rebuilt rather than mis-read.
- **Keying & cross-check:** filename = sanitized `radioProfile`. When `/air` is reachable, compare the drone's `radio.adapterId` to the configured `radioProfile`; **log a warning on mismatch** (e.g. drone reports `bl-m8731bu4` but `radioProfile=m8812eu2`). Soft, never blocks.
- **Drift:** the EWMA + decaying counts pull the curve toward new reality; no explicit invalidation.

## 8. Flight logging & offline analysis

**GS-side flight logger** (`gs/fpvdgs/dynlink/flightlog.py`, owned by `Policy`): a structured **JSONL, one record per selector tick**, reusing data the selector + store already compute. A file opens on `dynamicLink` enable and closes on disable (one per session). Each record:
- `ts`, link-RSSI (raw + smoothed);
- probe per-rung `{per, rssi, snr, ageMs}`;
- operating MCS, the emitted `{mcs}` + `reason`, demote flags (video-PER / emergency / starvation);
- learned-prior state: `ceiling`/`confidence` for the current RSSI bin, the **warm-start seed** (on the seed tick), and any **predictive-demote trigger** (with the projected RSSI/ceiling that fired it).

**Persistence & bounds:** `<flightlog.dir>/<startMs>.jsonl` (default `/etc/fpvd/flightlog/`, overlay-persistent — survives the reboots seen during the 3b flag-day), **size-capped + rotated** (`max_files`, `max_mb`; ~10 Hz × 600 s ≈ ~2 MB/flight). GS side stays dependency-free (plain JSON append); files pulled via `scp` / the existing API.

**Repo-side analysis script** (`gs/tools/flightlog_analyze.py`, dev-machine, offline — `matplotlib` optional, degrades to text/CSV):
- **Timeline** — RSSI vs operating-MCS vs probe-PER over the flight, with promote/demote/warm-start/predictive events marked.
- **Curve dump** — the learned RSSI→ceiling table + derived floor ladder (from the persisted `<radioProfile>.json` or reconstructed from the log), with per-bin sample counts/confidence.
- **Summary stats** — time-at-each-MCS, demote counts (reactive vs predictive), glitch episodes, warm-start hit-vs-fallback rate, mean operating-MCS vs the curve's ceiling (head-room on the table).

The operator reads this and hand-tunes the knobs. The analysis tool is **not deployed** (dev-machine only); the logger ships in `fpvdgs`.

## 9. Config delta

New, under `tuning.learned_prior` (all defaulted; no existing knobs removed):
- `enabled` (kill-switch, default on), `bin_width_db` (2), `rssi_min`, `rssi_max`, `ewma_alpha` (0.1), `viable_threshold`, `min_samples_warmstart`, `min_samples_predictive` (> warmstart), `warmstart_margin` (0), `predictive_debounce_windows` (3) + RSSI-smoothing params, `flush_interval`, `persist_dir`.
- `tuning.learned_prior.flightlog.{enabled (on), dir, max_files, max_mb}`.

`coarse_mcs_for_rssi` / `_COLD_START_RSSI_DBM` and the `profile.py` `rssi_floor` rows are **kept** as the unconfident fallback. No deprecations.

## 10. Bootstrap & failure modes

- **First flight per card:** no file → empty table → warm-start `UNKNOWN` → today's `coarse_mcs_for_rssi` + climb; predictive off. The table fills passively; the next enable warm-starts wherever flown.
- All failure modes **degrade to today's behavior and never crash the controller:** corrupt/unreadable store → caught, empty, logged, rebuilt; invalid/missing RSSI → tick not ingested, query `UNKNOWN`; probe stale/dead → unchanged from today (the prior never gates promotes, so it can't make this worse), predictive still works off RSSI bounded by its guards; `radioProfile` changed → re-keys on next enable; disk full / write failure → log + continue in-memory.

## 11. Integration

A new GS module `gs/fpvdgs/dynlink/learned_prior.py` (store + model + queries) and `flightlog.py` (logger), both owned by `Policy`. `Policy.__init__` loads the store (given `radioProfile` + `persist_dir`); `Policy.tick()` ingests one observation, opens/writes/closes the flight log on the lifecycle, injects the warm-start seed (first tick) + predictive-demote signal into the `LeadingSelector`, and queries the curve. The `LeadingSelector` decision logic is otherwise unchanged; `coarse_mcs_for_rssi` is retained as the fallback.

## 12. Testing

- **Unit (engine):** binning (incl. out-of-range/missing RSSI); ingest/update (clean raises, cliff lowers, `n` decays); `ceiling(bin)` (threshold + min_samples, `UNKNOWN`); confidence gates (warmstart vs predictive); derived floor ladder (isotonic monotonicity, denoise non-monotone input, extrapolate an unflown bin); persistence round-trip (write→read identical; schema/bin-config mismatch → discarded; corrupt → empty + no crash; atomic temp+rename); keying (filename from `radioProfile`; adapter_id mismatch → warning, no behavior change); kill-switch (`enabled=false` → ingest no-op, queries `UNKNOWN`).
- **Unit (flightlog):** record schema serialization; rotation/size-cap; lifecycle open/close on enable/disable; disabled → no-op. Analysis script: smoke test against a sample JSONL (parses → summary).
- **Integration (simulated flights through `Policy`):** bootstrap→learn→warm-start (feed an RSSI-sweep flight, persist, fresh `Policy` warm-starts to the learned ceiling on tick 1 instead of climbing from 1); predictive demote (RSSI fade → demote ahead of video-PER; a single noisy dip does **not** trigger; reactive still fires when the prior is unconfident/off); probe stays authoritative (a confident curve never promotes; a live probe cliff overrides). **Regression:** with an empty/disabled prior, the existing Phase-2 selector tests stay byte-for-byte green.
- **Hardware (multi-flight, operator-run):** flight 1 (empty store) behaves like today **and** the store fills (dump in `/status`); flight 2 at a known RSSI warm-starts to the learned ceiling (faster than the climb), probe still validates; a forced RSSI fade triggers predictive demote ahead of video loss (no glitch), a stable-but-noisy link does not false-demote; confirm no runner bounce, wire still `{mcs}`-only (no new fields), no drone change, and the adapter_id cross-check warning on a deliberate `radioProfile` mismatch.

## 13. Relationship to other phases

- **Phases 1–3 (done):** probe link, GS probe-driven selector, drone-local compute, `{mcs}`-only v3 wire. Phase 4 layers the learned prior onto the Phase-2 selector — purely additive.
- **Deferred siblings:** GS auto-profile-select (drone `adapter_id` → profile) and probe-pps reduction (§1). Each is its own small later phase.
- After Phase 4 the cold-start is warm, fades are anticipated, and every flight makes the per-card curve sharper — while the live probe remains the authority and the wire/drone are untouched.

## 14. Self-review

- **Spec coverage (parent §4.8):** learned RSSI→ceiling prior ✓ (§3–§7, passive, per-card, prior-not-authority); RSSI as a single MCS-independent link value ✓ (1-D curve, not a per-MCS matrix). **Deviations from parent §4.8 (documented §1):** (a) keyed by `radioProfile` not drone `adapter_id` — GS-local, `/air`-independent, adapter_id kept as a cross-check; (b) auto-profile-select deferred; (c) probe-pps reduction dropped (would breach the `{mcs}`-only wire for <1 % airtime); (d) pure-passive (no higher-pps calibration pass). Logging+analysis is an addition beyond the parent (§8).
- **Ambiguity:** confidence gating is explicit and per-use (`min_samples_warmstart` < `min_samples_predictive`); predictive is down-only + debounced + backstopped by the unchanged reactive path; the curve never promotes.
- **Scope:** one cohesive GS-local plan (learning engine + two uses + logging). The deferred siblings keep it focused.
- **Placeholders:** constants given as defaults (bin 2 dB, α 0.1, debounce 3, paths) — final values tuned against the live config + first flights during planning. No TBDs.
