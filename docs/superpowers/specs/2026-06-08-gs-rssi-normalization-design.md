# GS RSSI normalization for dynamic TX power — design

**Date:** 2026-06-08
**Branch:** `feat/probe-driven-link`
**Scope:** GS-only. No drone change, no wire change.

## Problem

The drone now sets TX power per-MCS from a backoff curve (see
`2026-06-08-drone-dynamic-txpower-design.md`): `tx_power = curve[operating_mcs]`,
ranging 29 dBm at MCS0 down to 19 dBm at MCS4–7. So the RSSI the GS measures is

```
RSSI_rx ≈ curve[operating_mcs] − pathloss(distance) + const
```

— it now conflates distance with the current MCS's power. Every GS RSSI consumer
reads `signals.rssi` (the EWMA of the video link's best-antenna RSSI, measured at
the operating power):

- cold-start seed — `coarse_mcs_for_rssi(rssi)` / `learned_prior.warmstart_seed(rssi)`
- predictive-demote — `learned_prior.predictive_ceiling(rssi, slope)`, with
  `slope = rssi − prev_rssi`
- learned prior — `ingest(rssi=…)` and `ceiling(rssi)`, **binned by raw RSSI**
- flight log + decision snapshot

Two concrete breakages:

1. **Learned RSSI→ceiling bins mix distances.** A given RSSI bin now corresponds
   to different ranges depending on which MCS produced it (e.g. MCS0 at 29 dBm and
   MCS5 at 19 dBm at the same distance differ by ~10 dB), so the RSSI→ceiling map
   is corrupted.
2. **Predictive-demote misfires against promotes.** A promote drops the MCS's
   power → an RSSI step-down that looks like a fast fade → `slope` goes negative →
   the predictive-demote can fire against the very promote that caused it.

## Approach

**EIRP-normalize RSSI by the known per-MCS power**, so it is distance-linear
again. The GS decided the MCS and can hold a copy of the curve, so it can de-trend
locally — no wire/back-channel change:

```
rssi_norm = rssi_raw + (P_ref − curve[received_mcs])
```

Normalize **at the signals layer, per stats window, by the received MCS** (each
`RxAnt` window carries both `rssi_avg` and `mcs` = the MCS the video was received
at = the drone's power that window), **before** the EWMA. Then `signals.rssi`
becomes EIRP-normalized and every downstream consumer gets the corrected value
with no per-consumer change. Normalizing before the EWMA removes the power step at
the source, so a promote's power drop never enters the EWMA as a fake fade.

## Components / changes (GS)

1. **Config** — new `tuning.rssi_norm` block, parsed in `config_build.py`:
   - `tx_power_dbm_by_mcs`: list[int], default `[29, 28, 25, 23, 19, 19, 19, 19]`
     — **mirror of the drone's `TXPWR_DBM` curve**. Must stay in sync with the
     drone (both are static calibration constants; document the coupling).
   - `p_ref_dbm`: int, default `29` (normalize to full EIRP).
   - `enabled`: bool, default `true`. When false → identity (raw RSSI), for
     rollback/back-compat.
   Carried into a small `RssiNormConfig` consumed by the `SignalAggregator`.

2. **`SignalAggregator` (`signals.py`)** — in `consume`, after computing the
   best-antenna window RSSI (`rssi_max_w`):
   - take the window's received MCS from the `RxAnt` stats (all antennas in a
     window share it; use the best-antenna's `mcs`),
   - `rssi_norm_w = rssi_max_w + (p_ref − curve[clamp(mcs,0,7)])` when enabled,
     else `rssi_max_w`,
   - EWMA `rssi_norm_w` into `signals.rssi` (the consumer-facing value, now
     normalized),
   - also expose `signals.rssi_raw` = EWMA (or latest) of the un-normalized
     `rssi_max_w`, for the flight log / observability.
   No `rx_ant_stats` in a window → `signals.rssi` unchanged (None-safe), as today.

3. **Policy consumers — no logic change.** `coarse_mcs_for_rssi`,
   `predictive_ceiling`/slope, `learned_prior.ingest`/`ceiling`/`warmstart_seed`
   all keep reading `signals.rssi`, now normalized. (The slope is computed from
   the normalized `signals.rssi`, so a promote no longer steps it.)

4. **Cold-start table** — shift `coarse_mcs_for_rssi`'s thresholds onto the
   normalized (P_ref = 29) scale. It's a conservative one-shot fallback the
   learned prior supersedes quickly, so exact values are not critical; just move
   them so they're sane against full-EIRP-normalized RSSI.

5. **Flight log + decision snapshot** — record both `rssi` (normalized) and
   `rssi_raw`, so offline analysis sees the de-trended value and the measured one.

## Migration

The live learned-prior store (`/etc/fpvd/learned/*.json`) has been ingesting raw
RSSI under dynamic power since the txpower feature deployed, so it is already
partly corrupted. **Reset it** on rollout (`rm /etc/fpvd/learned/*.json`; it
auto-rebuilds empty). The store's bin signature is `[bin_width, rssi_min,
rssi_max]`; the normalized scale (P_ref = 29) shifts values up vs the old raw
scale, so confirm the configured `rssi_min`/`rssi_max` bin range still covers the
normalized range (widen if needed) — a bin-signature change also forces a clean
rebuild.

## Edge cases / assumptions

- **MCS out of curve range:** clamp to `[0,7]` before the curve lookup.
- **Mixed-MCS during a transition:** handled correctly — each window is normalized
  by its own received MCS before the EWMA, so the EWMA never blends powers.
- **Curve sync:** the GS `tx_power_dbm_by_mcs` must equal the drone `TXPWR_DBM`.
  Documented coupling; both static. (A future option is the drone reporting its
  applied power, but that needs a back-channel removed in Phase 3b — out of scope.)
- **Probe-status RSSI** (per-rung, in `/status`) is observability only and not
  consumed by the policy; leave it raw (out of scope).

## Testing (TDD, GS pytest — `.venv/bin/python -m pytest tests/ -q`)

- Normalization math: `rssi_norm = raw + (P_ref − curve[mcs])` per MCS; clamps
  out-of-range MCS; identity when `enabled=false`.
- `SignalAggregator.consume`: a window at MCS5 (curve 19, P_ref 29) raises RSSI by
  +10 vs the raw; `signals.rssi_raw` keeps the measured value.
- **EWMA across mixed-MCS windows:** feed windows at MCS0 then MCS5 at the *same*
  raw RSSI; `signals.rssi` stays ~flat (the power step is removed), whereas
  `signals.rssi_raw` shows the ~10 dB step.
- **Predictive-demote no longer misfires on a promote:** a simulated MCS climb
  (power drop) produces no negative normalized slope, so no predictive demote
  (where the raw-RSSI version would have demoted).
- Flight log carries both `rssi` and `rssi_raw`.
- Regression: with `enabled=false`, behavior is byte-identical to today.

## Out of scope

- Drone reporting its applied TX power back to the GS (needs a back-channel).
- Adaptive/online estimation of the power offset (we mirror the static curve).
- Normalizing the probe-status display RSSI.
- Auto-syncing the GS curve from the drone.
