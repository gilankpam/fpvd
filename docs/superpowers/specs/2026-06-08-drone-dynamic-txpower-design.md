# Drone dynamic per-MCS TX power — design

**Date:** 2026-06-08
**Branch:** `feat/probe-driven-link`
**Scope:** drone-only (Phase 3a — the drone owns its own bitrate/FEC/depth/power compute). GS unchanged.

## Problem

The link could not climb past MCS4. Investigation (see session history) found:

- The rtl88x2eu driver applies a **flat TX power across every modulation** — `tx_power_idx`
  shows MCS0 (BPSK) and MCS5 (64-QAM) both driven at the same index (idx 69 @ 20 dBm,
  no regulatory limit, not thermal). 64-QAM has ~3–4 dB higher peak-to-average ratio, so at
  the same average power its peaks hit the PA's compression region while lower-order rates stay
  linear → 64-QAM distorts (high EVM) and fails while MCS0–4 are fine.
- Confirmed empirically: at close range, `link.txpower=52` (~26 dBm) → MCS5 = 100% loss;
  dropping to `40` (~20 dBm) → MCS5 clean. Forcing the video link to MCS5 at high power
  produced sustained `link_starved` (total blackout); the probe agreed (MCS5 undecodable).
- The drone has **no TX-power compute today**: `applyLocalCompute` sets k/n/bitrate/depth/fps
  only; `dispatchTxApply` never touches power; `RadioTxpower` exists but is vestigial (never
  called, comment marks it for Phase 3b removal); `Decision.txPowerDbm` is unused in the climb.
  The only power applied is the static `link.txpower` via `radio-up.sh`, flat across all MCS.

So high MCS overdrives the PA because nothing backs power off as the modulation order rises.
The fix is a per-MCS power backoff the drone applies itself.

## Approach

Use the pre-calibrated per-MCS power table for this exact card from OpenIPC
adaptive-link (`wlan_adapters.yaml`, `bl-m8812eu2`), **level 4** column. Values are `iw`
mBm (the `iw set txpower fixed <mBm>` argument), which matches our existing path
(`link.txpower * 50` → mBm; `40 → 2000 mBm → "20 dBm"` as the driver reports).

Reference (mBm) and the whole-dBm curve we bake (the existing `RadioTxpower::apply` takes
int8 dBm and is already tested; reusing it avoids a wider field, a wire change, and rewriting
7 tests — at the cost of ≤0.5 dB rounding on two mid rungs, negligible for an uncalibrated
table described upstream as "not tested with instruments yet"):

```
reference mBm: 2900  2750  2500  2250  1900  1900  1900  1900
reference dBm:   29  27.5    25  22.5    19    19    19    19
TXPWR_DBM[8] = {  29,   28,   25,   23,   19,   19,   19,   19 }   // round-half-up
//        MCS:    0     1     2     3     4     5     6     7
```

Full power (29 dBm) at MCS0 for range; backed off to 19 dBm for the 64-QAM rungs (MCS4–7).
The 19 dBm for MCS5 matches the ~20 dBm found empirically.

### Coupling: power follows the OPERATING rung (`d.mcs`)

`power = TXPWR_DBM[d.mcs]`. Considered (and rejected) coupling power to the *probed* rung
(current+1): it would under-power the operating video and cost ~1.5 dB of range at the
furthest (MCS0 would run at MCS1's power). Operating-rung coupling has no downside here, and
it keeps the probe honest **for free** because the backoff region is flat:

| operating | probe (op+1) | interface power = TXPWR[op] | probe's correct power = TXPWR[probe] | Δ | note |
|:--:|:--:|:--:|:--:|:--:|---|
| 0 BPSK | 1 | 29 | 27.5 | +1.5 | full power (furthest); QPSK probe immune to overdrive |
| 1 QPSK | 2 | 27.5 | 25 | +2.5 | harmless |
| 2 QPSK | 3 | 25 | 22.5 | +2.5 | harmless |
| 3 16-QAM | 4 | 22.5 | 19 | +3.5 | harmless (16-QAM held at 26 dBm in tests) |
| 4 16-QAM | **5 64-QAM** | **19** | **19** | **0** | **critical climb — exact** |
| 5 64-QAM | 6 | 19 | 19 | 0 | exact |
| 6 64-QAM | 7 | 19 | 19 | 0 | exact |
| 7 64-QAM | 7 (clamp) | 19 | 19 | 0 | at ceiling |

Whenever the *probe* is a 64-QAM rung (the overdrive-prone case), the operating rung is
already in the flat-19 region, so the probe measures MCS5/6/7 at exactly its real power. The
only over-powering is on BPSK/QPSK/16-QAM probes, which do not overdrive at these levels.

The shared-interface-power constraint (probe + video are two `wfb_tx` streams on the same
`wlan0`; `iw txpower` is per-netdev; `setRadio` carries no power) is what forces a single
power for both — operating-rung coupling is the correct resolution given the table shape.

### Fixed level (no runtime level selection)

Level 4 is baked. No 5-level selector, no RSSI-adaptive level, no per-tick power computation.
A future enhancement could make the level RSSI-adaptive for high-MCS-at-medium-range, but
that is out of scope.

## Components / changes (drone)

1. **Power curve constant** — `TXPWR_DBM[8]` (int8) in new `src/dynlink/txpower_curve.{hpp,cpp}`.
   One lookup `txpowerDbmForMcs(int mcs)` clamping `mcs` to `[0,7]`, returning `int8_t` dBm.
2. **Field** — reuse the existing `Decision.txPowerDbm` (`int8`). No new field, no wire change
   (txpower is a drone-local field, not in the 11-byte wire payload).
3. **`applyLocalCompute`** (`local_compute.cpp`) — set `d.txPowerDbm = txpowerDbmForMcs(d.mcs)`
   (previously left untouched). Update the header comment that says txPowerDbm is left alone.
4. **`RadioTxpower`** (`radio_txpower.{hpp,cpp}`) — **unchanged** (already does diff-based
   `iw dev <iface> set txpower fixed <dBm*100>`). It was only vestigial in that nothing
   *called* it; this feature is the caller. Its existing tests stay green.
5. **`dispatchTxApply`** (`controller.cpp`) — in the existing MCS-change block (where
   `setRadio` and the probe retune already fire), call `radio_->apply(d.txPowerDbm)`.
   Co-located so power changes exactly when MCS changes. Always on when dynamic-link is
   running (no separate enable flag — no new config).
6. **`dispatchTxSafe`** — `radio_->applySafe(txpowerDbmForMcs(cfg.safe.mcs))` (unconditional,
   matching the other safe sub-commands; low safe MCS → high power → good for recovery).
7. **Config lock** (`src/config/lock.cpp`) — add `{"link","txpower"}` to `kLockedPaths` so a
   `PATCH /config` of `link.txpower` while `dynamicLink.enabled` returns `400
   dynamic_link_locked` (same as `link.mcs`). The curve now owns tx power per-decision, so a
   manual value would be silently overridden — reject it instead. Update the stale NOTE
   comment (lines ~12–20) that says `link.txpower` is deliberately unlocked / constant; keep
   `link.stbc` / `link.ldpc` unlocked (still config-preserved, not DL decisions). The boot
   path is unaffected — `link.txpower` is still read at radio bring-up; to change it the
   operator disables DL first, exactly like `link.mcs`.

## Dynamics

- Promote (MCS↑) → power ↓ (backoff for the higher rung's linearity).
- Demote (MCS↓) → power ↑ (more robustness for recovery).
- Video always runs at its own rung's calibrated power (no under-power, no range loss).

## Edge cases / assumptions

- **Bandwidth:** the table is the 20 MHz reference; the link runs 20 MHz. 10 MHz would need
  its own column (out of scope).
- **Startup / non-DL:** unchanged — `link.txpower` via `radio-up.sh` is the boot default; the
  curve overrides per-decision once dynamic-link is running.
- **Clamp:** `d.mcs` is always 0–7; lookup clamps defensively.
- **GS:** no change.

## Testing (TDD, drone C++ — `./build/fpvd_tests`, run from `drone/`)

- `txpowerDbmForMcs(mcs)` returns the expected dBm for MCS0–7 and clamps out-of-range input.
- `applyLocalCompute` sets `d.txPowerDbm` from the curve for representative MCS values.
- `RadioTxpower` already has tests (apply/applySafe/diff/iface) — unchanged; no new test there.
- `dispatchTxApply`/`dispatchTxSafe` radio wiring is verified by build + on-hardware (no clean
  unit seam: `radio_` runs `iw` via posix_spawn and isn't injectable).
- Config lock: `PATCH link.txpower` while `dynamicLink.enabled` → rejected with
  `dynamic_link_locked` (`lockedPaths` contains `link.txpower`); accepted when DL disabled;
  `link.stbc`/`link.ldpc` remain accepted while DL enabled (not newly locked).

## Out of scope

- 5-level selection / operator power presets.
- RSSI-adaptive power level (high-MCS-at-medium-range).
- Multi-card / multi-bandwidth power tables (parsing `wlan_adapters.yaml`).
- Wider-than-int8 / mBm power fidelity (we round the table to whole dBm by design).
