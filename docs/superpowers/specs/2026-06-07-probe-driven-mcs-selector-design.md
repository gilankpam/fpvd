# Phase 2 — Probe-Driven MCS Selector (GS) Design

**Date:** 2026-06-07
**Status:** Draft for review
**Target:** GS `fpvdgs.dynlink`; drone untouched
**Refines:** `2026-06-06-probe-driven-link-control-design.md` §4.4 / §5 (Phase 2)
**Builds on:** Phase 1 (probe link, observe-only) + the fixed-stream plumbing (`2026-06-07-probe-fixed-stream-plumbing-design.md`), both deployed + hardware-validated.

---

## 1. Purpose & scope

Make the probe **drive the MCS**, replacing the SNR-floor selector. Today the probe measures per-rung PER observe-only and the GS picks MCS from an SNR-margin/hysteresis ladder (`LeadingSelector` Channel A). That ladder is the unsound thing the redesign set out to remove — it's conservative (leaves viable rungs on the table; observed live: probe found MCS 3 clean while the SNR selector sat at MCS 2) and SNR is misleading at the cliff (survivor-biased).

**In scope:** a **GS-only selector swap** — replace the SNR-based promote/demote (`LeadingSelector` Channel A + the SNR signals feeding it) with a **probe-driven promote**, keeping the existing **Channel-B reactive demote**. Wire the probe snapshot into the policy. Remove the SNR machinery.

**Out of scope (unchanged this phase):** the GS→drone wire, HELLO/sync-gate, GS-side bitrate/FEC computation (all Phase 3); the drone (no change); the probe plumbing (the deployed **single** `current+1` stream is exactly what's needed). The Phase-4 learned RSSI/SNR prior (parent §4.8).

## 2. Locked design decisions (from brainstorming)

- **One probe stream** (`current+1`), unchanged from the deployed plumbing. No drone/plumbing change.
- **Video runs at the ceiling** (the highest viable MCS) — max throughput, no margin sacrificed.
- **Probe = the promote signal only.** Promote when `current+1` reads clean (EWMA + debounce); the climb naturally stops at the ceiling (once `current+1` cliffs, you stay).
- **Demote = reactive, the kept Channel-B path** (live video PER / FEC / starvation). Accepted tradeoff: a brief (~1-window, ~50–100 ms, FEC-cushioned) glitch is possible when the ceiling drops out from under the operating rung — the price of 1-stream + max-throughput. (Predictive/glitch-free demote needed a 2nd probe stream + running at ceiling−1; explicitly not taken.)
- **RSSI = cold-start hint only** (a single MCS-independent link value read off the video stream; validated 2026-06-07). No per-MCS RSSI, no RSSI in the running control loop.
- **Learning engine = Phase 4** (parent §4.8).

## 3. Architecture

The selector lives where `LeadingSelector` does (`fpvdgs/dynlink/policy.py`), inside `Policy.tick(signals) -> Decision`. Two signal sources feed it each tick:

```
ProbeController.status()  ──per-rung PER──┐
  (probe wfb_rx, current+1)               ▼
SignalAggregator (video :8103 stats) ──▶ Selector.tick(M, probe, video) ──▶ Decision(mcs=M')
  residual_loss, fec_work, starvation,                                          │
  link-RSSI (cold-start)                                                        ▼
                                                          bitrate/k/n (unchanged) → wire → drone
```

The probe controller and the dynlink controller are separate objects (both built in `supervisor.py`). Wiring: give the dynlink controller a handle to read `ProbeController.status()` and pass that snapshot into `Policy.tick()` alongside `signals`. (Injected accessor, mirroring how `signals` is already supplied — see §5.)

## 4. Selector logic

State: operating MCS `M` (∈ `[min_mcs, maxMcs]`); ceiling `C` = highest viable rung. Probe reports raw PER for the probed rung (`current+1` ⇒ `M+1`).

**Promote (probe-driven, NEW):**
- Read `probe[M+1]`. If it is **fresh** (a sample within `probe_freshness_ms`) and its EWMA success `1 − per ≥ probe_viable_threshold` for `promote_debounce_windows` consecutive fresh windows, and `M+1 ≤ maxMcs`, and the rate limit (`min_between_changes_ms`, and not within `hold_modes_down_ms` of a demote) allows → `M := M+1`.
- Climb stops at the ceiling automatically: at `M = C`, `probe[M+1]` cliffs (`per` high) ⇒ no promote. When `C` later rises, `probe[M+1]` goes clean ⇒ resume climbing.
- The EWMA (probe-side, `probe_ewma_alpha`) smooths noise; the `promote_debounce_windows` count guards the sharp cliff against a transient clean blip.

**Demote (reactive, KEPT Channel-B):**
- Trigger on the live video stream: on-air PER `(lost + fec_rec)/(out + lost) ≥ video_demote_per`, OR FEC exhaustion / `fec_pressure ≥ emergency_fec_pressure`, OR starvation (`link_starved` sustained `starvation_windows`). → step down (one step; emergency may step multiple), bypassing the promote rate-limit, then `hold_modes_down_ms`.
- This is the existing `_emergency_active()` path, retained as-is (optionally tuning its loss trigger to the combined `(lost+fec_rec)` metric). It is the *only* demote — the probe (cliffed above the ceiling) gives no demote signal in this design.

**Cold-start:** before any probe/video data (no session yet), seed `M` from the single link-RSSI via a coarse default `rssi → mcs` table (or `safe.mcs` if RSSI unavailable). Once probe/video data flows, the loop above takes over.

**Probe stale/dead** (no fresh `M+1` sample within `probe_freshness_ms`): **no promote** (hold `M`); demote stays fully active (it's video-driven, independent of the probe). Logged.

**Oscillation:** none by construction — promote needs sustained-clean `M+1`; at the ceiling `M+1` is cliffed so it can't promote, and demote only fires on real video degradation. There is no SNR hysteresis to tune.

## 5. Wiring (probe → selector)

- `supervisor.py` builds both `DynamicLinkController` and `ProbeController`. Give the dynlink controller read access to the probe snapshot — pass a `probe_status` callable (default `probe_ctrl.status`) into `DynamicLinkController`, which forwards the snapshot into `Policy.tick(signals, probe=...)`. Null-safe: if no probe handle (or probe disabled), `Policy` runs cold-start + reactive-demote only (no promote).
- `Policy.tick(signals, probe_snapshot)` extracts `probe_snapshot["mcs"][str(M+1)]` (per, windows, freshness) for the promote test. The probe snapshot's per-MCS RSSI/SNR are ignored for control (logged only).

## 6. Removed vs kept

**Removed (SNR machinery, `policy.py`/`signals.py`/`config_build.py`):** `LeadingSelector._pick_mcs`/`_margin`/`_stress_margin` (SNR-floor lookup), the upgrade/downgrade hysteresis gates, `snr_slope` + the predictive horizon, the upward confidence loop, the per-control SNR/RSSI EWMA in `SignalAggregator` (keep link-RSSI for cold-start + logging), and the gate knobs `snr_safety_margin`, `snr_ema_alpha`, `snr_slope_alpha`, `snr_predict_horizon_ticks`, `hysteresis_up_db`, `hysteresis_down_db`, `upward_confidence_loops`. The radio-profile `snr_floor_dB` table is no longer read (leave the file/back-compat; mark deprecated).

**Kept:** the Channel-B emergency path (`_emergency_active`), the latency predictor (`PredictorConfig`/`fit_or_degrade` — SNR-independent), bitrate (`bitrate.py`), dynamic-FEC (`dynamic_fec.py`), the `Decision` shape + wire (`wire.py`), HELLO/sync-gate (`drone_config.py`/`tunnel_listener.py`), the dispatch/apply, `min_between_changes_ms`/`hold_modes_down_ms`/`max_mcs`/`starvation_windows`.

## 7. Config delta (`dynamicLink.tuning.gate` + selector)

- **Add:** `probe_viable_threshold` (e.g. 0.99 success), `probe_freshness_ms` (e.g. 500), `promote_debounce_windows` (e.g. 3), `video_demote_per` (e.g. 0.05 on the combined metric). Reuse the probe's `ewma_alpha`.
- **Remove:** `snr_safety_margin`, `snr_ema_alpha`, `snr_slope_alpha`, `snr_predict_horizon_ticks`, `hysteresis_up_db`, `hysteresis_down_db`, `upward_confidence_loops` (parse-and-ignore for back-compat, like the already-deprecated keys).
- **Kept:** `emergency_loss_rate`, `emergency_fec_pressure`, `max_mcs`, `min_between_changes_ms`, `hold_modes_down_ms`, `starvation_windows`/threshold.

## 8. Testing

- **Unit (new):** promote climbs to the ceiling on sustained-clean `current+1`; stops when `current+1` cliffs; `promote_debounce_windows` rejects a single clean blip; probe-stale ⇒ no promote; cold-start picks an MCS from RSSI. Drive `Policy.tick` with a stubbed probe snapshot + synthetic signals.
- **Unit (kept):** Channel-B emergency (loss / fec / starvation) still demotes; latency predictor; bitrate; dynamic-FEC.
- **Unit (removed):** the SNR-hysteresis / margin / confidence-loop / `snr_slope` tests in `test_dl_policy_leading.py` and the `snr_slope` test in `test_dl_signals.py`.
- **Replay:** drive the selector against captured probe+video traces (promote into headroom on the clean trace; reactive-demote on the cliff trace).
- **Controller:** `test_dl_controller.py` — the dynlink controller forwards the probe snapshot into `Policy.tick`.

## 9. Relationship to other phases

Phase 2 is GS-only and leaves the wire/HELLO/bitrate for **Phase 3** (drone bitrate calc + `{mcs}`-only wire + HELLO removal). The **Phase 4** learned RSSI/SNR prior (parent §4.8) layers on top later as the cold-start/predictor accelerant, subordinate to the probe.

## 10. Self-review

- **Spec coverage (parent §4.4/§5):** promote-on-probe ✓ (§4); demote-on-video-PER + Channel-B ✓ (§4, kept); RSSI prior cold-start ✓; SNR removed ✓ (§6); rate limits kept ✓. Deviation from parent §5: the wire-shrink + HELLO removal are deferred to Phase 3 (they require the drone bitrate calc) — documented in §1/§9. `current+2`/predictive-demote from the parent's fuller vision is explicitly dropped (1-stream, ceiling, reactive demote) — §2.
- **Ambiguity:** demote is the single kept Channel-B path (not a separate graduated tier) — §4. Probe snapshot is read-only into the selector; per-MCS RSSI ignored — §5.
- **Scope:** single GS implementation plan; no drone/plumbing/wire change.
- **Placeholders:** threshold defaults are starting values to tune on hardware, not blockers.
