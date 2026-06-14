# Dynamic-Link Config Cleanup, Merge & Tiering — Design

**Date:** 2026-06-14
**Status:** Approved (pending spec review) → implementation plan next
**Builds on:** `refactor/decouple-idr-osd` (commit `b9a81b4`) — the IDR and OSD
decouplings were the first slices of this same effort.

## Motivation

The dynamic-link subsystem accreted ~80 individual config fields across both
daemons. Several are dead (defined + built but never read), several duplicate
each other, and the GS side exposes a large `dynamicLink.tuning` subtree as an
**opaque, unvalidated passthrough** — every knob is settable, none is checked or
documented, and a typo silently no-ops. This design removes the dead knobs,
merges the redundant ones, and sorts the survivors into three explicit tiers
(operational / advanced / static calibration), giving the advanced GS knobs the
same first-class validation the drone already has.

## Decisions (locked)

1. **Full execution** — remove dead, do the merges, and restructure the tiers in
   code now (not a recommendation doc).
2. **Clean break** — no back-compat shims. Renamed/removed keys just stop
   working; the live GS overlay is updated by hand (see Migration).
3. **Freeze calibration to constants** — hardware-calibration knobs become
   compile-time / module constants, not runtime config. (The IDR `idr_constants`
   and OSD `osd_constants` from the prior commit are the precedent.)
4. **Strict GS `tuning` validation** — the new validator rejects unknown keys at
   load, matching the existing GS validators (`beamforming`, top-level config).
   Boot validation is fatal: `build_app` runs `validate_effective` unguarded
   (`supervisor.py:86`) and `main()` (`:188`) doesn't catch it — no last-good
   fallback, so a stale key blocks boot until removed. This is the intended
   forcing function for a clean overlay.

## Non-goals

- No change to the adaptive-link *algorithm* (selector, learned prior, probe
  cadence behavior). This is config plumbing only.
- No GS↔drone wire change (v3 `{mcs}` packet is untouched).
- `safe.*` stays a self-contained failsafe (see B5).
- Renaming `osdUpdateIntervalMs` (misleadingly named control-loop tick cadence)
  is out of scope — noted for a future pass.

## Current inventory (post IDR/OSD)

### Drone `dynamicLink.*` (19 fields)

| Block | Fields | Where | Disposition |
|---|---|---|---|
| scalars | `enabled, healthTimeoutMs, applyStaggerMs, applySubPaceMs` | defaults.json | Tier 1 `enabled`; Tier 2 rest |
| `roiQp` | `thresholdKbps, lowAnchorKbps, floor, step` | defaults.json | Tier 2 |
| `safe` | `mcs, k, n, overheadPct, deadlineMs, bandwidth, txPowerDbm, bitrateKbps` | defaults.json | Tier 1 (keep self-contained) |
| `bitrate` | `minBitrateKbps, maxBitrateKbps` | schema-only | **merge → `compute`** (Tier 2) |
| `fec` | `baseRedundancyRatio, blocksPerFrame, kMin, kMax` | schema-only | **merge → `compute`** (Tier 2) |

Top-level `osd.enabled` (Tier 1, already lifted). Drone static constants:
`txpower_curve.hpp`, `probe_constants.hpp`, `idr_constants.hpp`,
`osd_constants.hpp`, OpenIPC rate table — Tier 3 (already frozen).

### GS top-level `dynamicLink.*`

`enabled, maxMcs, radioProfile, droneAddr, dronePort, videoStreamId` (Tier 1),
`tuning{}` (opaque passthrough → to be validated).

### GS `dynamicLink.tuning.*` (~39 fields, unvalidated)

| Sub-block | Live fields | Dead / frozen |
|---|---|---|
| `gate` | `probe_viable_threshold, probe_freshness_ms, promote_debounce_windows, video_demote_per, emergency_loss_rate, emergency_fec_pressure, max_mcs` | **`max_mcs_step_up` (dead)** |
| `profile_selection` | `hold_modes_down_ms, min_between_changes_ms` | **`hold_fallback_mode_ms, fast_downgrade, upward_confidence_loops` (dead)** → block **merges into `gate`** |
| `policy` | `starvation_windows` | — |
| `learned_prior` | 13 fields + `flightlog{5}` | — |
| `smoothing` | `ewma_alpha_rssi, ewma_alpha_fec, starvation_threshold_pps` | **`ewma_alpha_burst` (dead)** |
| `rssi_norm` | — | `enabled, p_ref_dbm, tx_power_dbm_by_mcs` → **freeze to constant** |

GS `probe.*` (top-level, overlay-only): `rxL, ewmaAlpha, blackoutWindows`
→ **move under `dynamicLink.tuning.probe`** (Tier 2).

GS deprecated-key detector sets: `_DEPRECATED_LEADING_KEYS`,
`_DEPRECATED_GATE_KEYS`, `_DEPRECATED_PHASE3A_KEYS` → **remove** (clean break,
no migration window).

## The tier model

| Tier | Mechanism | Drone | GS |
|---|---|---|---|
| **1 — Operational** | in `defaults.json`, validated, operator-documented | `dynamicLink.enabled`, `dynamicLink.safe.*`, `osd.enabled` | `dynamicLink.{enabled, maxMcs, radioProfile, droneAddr, dronePort, videoStreamId}`, `tuning.learned_prior.enabled`, `tuning.learned_prior.flightlog.{enabled,dir}` |
| **2 — Advanced** | schema-defaulted + **validated**, *not* in defaults.json, in a tuning-reference doc | `dynamicLink.{healthTimeoutMs, applyStaggerMs, applySubPaceMs, roiQp.*, compute.*}` | `tuning.{gate.*, smoothing.*, policy.starvation_windows, learned_prior.*(internals), probe.*}` |
| **3 — Static/calibration** | compile-time / module constants, no config path | `txpower_curve`, `probe_constants`, `idr_constants`, `osd_constants`, OpenIPC rate table | rssi_norm curve (**freeze**) |

## Section A — Delete dead (GS only; drone has none)

1. `gate.max_mcs_step_up` — promote is hardcoded `current+1`; never read.
2. `profile_selection.hold_fallback_mode_ms` (already deprecation-warned),
   `fast_downgrade`, `upward_confidence_loops` — none read by `select()`.
3. `smoothing.ewma_alpha_burst` + the `burst_rate / holdoff_rate / late_rate`
   signal chain in `signals.py` (per-window `_w` + EWMA fields) — computed every
   window, consumed by nothing (not even logged).
4. The three `_DEPRECATED_*` detector sets in `config_build.py` — they warn on
   ~47 long-removed keys; the strict validator (C1) subsumes their purpose.

## Section B — Merge

**B1. Drone `bitrate` + `fec` → `dynamicLink.compute`**
`{minBitrateKbps, maxBitrateKbps, baseRedundancyRatio, blocksPerFrame, kMin,
kMax}` — one block feeding `BitrateEngineConfig`. Rename the two schema structs
into one `DynamicLinkCompute`; update `runtime_config.cpp` mapping + tests.

**B2. GS `profile_selection` → fold into `gate`**
Move the two survivors (`hold_modes_down_ms`, `min_between_changes_ms`) into
`GateConfig`; delete `ProfileSelectionConfig` and its `config_build` wiring.

**B3. GS `probe.*` → `dynamicLink.tuning.probe.*`**
The probe measurement window is dynamic-link tuning; the orphan top-level `probe`
block goes away. `probe/config_build.py` reads from the new nested location.

**B4. Freeze the rssi_norm curve + drift-guard test**
`RssiNormConfig.tx_power_dbm_by_mcs` becomes a module constant (the override path
from `tuning.rssi_norm` is removed). Add a pytest that parses
`drone/src/dynlink/txpower_curve.hpp`, extracts `kTxPowerDbmByMcs`, and asserts
the GS tuple matches — replacing the "keep in sync by hand" coupling with a test.
(`rssi_norm.enabled` rollback toggle is dropped; flip the constant + redeploy if
rollback is ever needed.)

**B5. `safe` — considered, keep as-is**
A failsafe must not derive values from the curve / `link.fec`, so `safe` stays
self-contained and fully explicit. (The only arguable cut — deriving
`safe.txPowerDbm` from the frozen curve at `safe.mcs` — is rejected here.)

## Section C — Retier + formalize the GS surface

**C1. GS: replace the opaque `tuning` passthrough with a validated nested
schema.** A `_validate_tuning(tuning)` recurses the known structure
(`gate, policy, learned_prior(.flightlog), smoothing, probe`), type/range-checks
each value, and **rejects unknown keys at every level** (strict). Wire it into
both `validate_effective` (boot) and `validate_config_patch` (PATCH). Extend
`_validate_dynamic_link` to also reject unknown `dynamicLink.*` keys. Silent
typo-noops and the deleted dead keys (A) now become explicit load-time errors.

**C2. Drone: shrink `defaults.json` to Tier 1.** Move `healthTimeoutMs,
applyStaggerMs, applySubPaceMs, roiQp, compute` out of defaults.json (they keep
their schema defaults). The drone `dynamicLink` block in defaults becomes
`{enabled, safe}`; top-level `osd.enabled` stays.

**C3. Docs.** New `docs/dynamic-link-tuning.md` enumerating the Tier-2 advanced
knobs (no longer discoverable via defaults.json), grouped by daemon.

## Migration (live GS overlay — clean break)

On the deployed GS overlay, before/with the deploy:

- `probe.{rxL, ewmaAlpha, blackoutWindows}` → `dynamicLink.tuning.probe.{…}`
  (the live `rxL=800`). The old top-level `probe` block must be removed.
- `profile_selection.{hold_modes_down_ms, min_between_changes_ms}` →
  `dynamicLink.tuning.gate.{…}`; drop any other `profile_selection.*`.
- Drop `gate.max_mcs_step_up`, any `rssi_norm` override, any `_DEPRECATED_*`-era
  keys, and any `smoothing.ewma_alpha_burst`.

Because the strict validator + fatal boot validation means **a leftover key
blocks GS boot**, the overlay must be clean before the GS daemon restarts. The
deploy step verifies the overlay parses + validates against the new schema.

## Testing

- **Drone doctest:** `compute`-block round-trip (B1); defaults.json-shrink still
  deep-merges (C2); `safe` fallback unchanged.
- **GS pytest (suite must stay green):** tuning validation good/bad/unknown (C1);
  dead-key removal (A); `profile_selection`→`gate` fold preserves selector
  behavior (B2); frozen rssi_norm + curve drift-guard test (B4); probe-block
  move (B3).
- The GS suite is import/config-coupled, so all changes land together.

## Risks

- **Boot-loop on a forgotten overlay key** — accepted (decision 4); mitigated by
  a deploy-time overlay validation step and the Migration checklist.
- **Drift between the GS rssi_norm constant and the drone curve** — mitigated by
  the B4 drift-guard test.
- **Removing `_DEPRECATED_*` warnings** removes the friendly migration hint, but
  the strict validator gives a precise "unknown key X" error instead.
