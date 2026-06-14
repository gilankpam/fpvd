# Dynamic-Link Config Cleanup & Defaults Dedup — Design

**Date:** 2026-06-14
**Status:** Approved (pending spec review) → implementation plan next
**Builds on:** `refactor/decouple-idr-osd` (commit `b9a81b4`) — the IDR and OSD
decouplings were the first slices of this same effort.

## Motivation

The dynamic-link subsystem accreted ~80 config fields across both daemons: dead
knobs (built, never read), duplicated knobs, and an opaque, unvalidated GS
`dynamicLink.tuning` passthrough where a typo silently no-ops.

On top of that, **defaults are maintained twice.** The code already holds every
default — nlohmann `..._WITH_DEFAULT` structs on the drone (~61 of them),
dataclasses + scattered `.get(k, literal)` on the GS — and the shipped
`defaults.json` files mirror those values, so every change touches two places
(this very cleanup keeps editing `schema.hpp` *and* `defaults.json` in lockstep).

This design (1) deletes the dead knobs, (2) merges the redundant ones, (3)
freezes hardware-calibration values to constants, and (4) **makes code the single
source of defaults** — dropping the drone's shipped `defaults.json` so a default
lives in exactly one place. The result is a **two-bucket** config model —
*tunable* vs *frozen* — that replaces the earlier three-tier, file-based split.

## Decisions (locked)

1. **Full execution** — remove dead, merge, and restructure in code now.
2. **Clean break** — no back-compat shims; the live GS overlay is hand-updated
   (see Migration).
3. **Freeze calibration to constants** — `txpower_curve`, the probe/idr/osd
   constants, and the GS rssi_norm curve. (The IDR/OSD constants from the prior
   commit are the precedent.)
4. **Strict GS `tuning` validation** — reject unknown keys at load. Boot
   validation is fatal: `build_app` runs `validate_effective` unguarded
   (`supervisor.py:86`) and `main()` (`:188`) doesn't catch it — no last-good
   fallback, so a stale key blocks boot until removed. Intended forcing function
   for a clean overlay.
5. **Code is the single default source** — drop the drone's shipped
   `defaults.json`; the defaults baseline becomes the serialized code struct.
   (GS defaults-centralization is a named follow-up — see Section C — because the
   GS's top-level defaults aren't in a code schema yet.)

## Non-goals

- No change to the adaptive-link *algorithm* (selector, learned prior, probe
  cadence behavior). This is config plumbing only.
- No GS↔drone wire change (v3 `{mcs}` packet is untouched).
- **GS defaults-centralization** (moving top-level `link/wfb/drone/pixelpilot`
  defaults into code + dropping `gs/etc/defaults.json`) is a named follow-up, not
  in this spec — only the drone defaults and the GS `tuning` subtree are de-duped
  here.
- Renaming `osdUpdateIntervalMs` (the misleadingly-named control-loop tick
  cadence) — out of scope, noted for a future pass.

## The config model (two buckets)

| Bucket | Mechanism | Discovery |
|---|---|---|
| **Tunable** | code default + sparse `config.json` overlay, deep-merged; validated | `GET /config` returns the full effective config (defaults materialized); reference doc explains each knob |
| **Frozen** | compile-time / module constant; no config path | the reference doc + the header |

There is **no `defaults.json` to be "in"** anymore (drone now; GS after the
follow-up), so the old Tier-1-vs-Tier-2 distinction dissolves — everything
tunable is discovered the same way. "Operational vs advanced" survives only as
*documentation grouping* in the reference doc, not as a config mechanism.

**Why a sparse overlay (not a full `config.json`):** with the file holding only
your deviations, a new build's added keys and changed defaults flow in
automatically, and the file doubles as the "what did I change" diff. A full file
would freeze old defaults and hide overrides. New-build behavior:

| New build… | Result |
|---|---|
| adds a key | absent from your overlay → picks up the new code default automatically |
| changes a default | inherited automatically unless you explicitly overrode it |
| removes/renames a key | drone: nlohmann ignores it; GS: strict validator rejects it (clean the overlay — decision 4) |

**Discovery mechanism.** `GET /config` is the source of truth for "what can I set
and what is it now":
- **Drone** already returns `nlohmann::json(effective())` — the full struct with
  all defaults materialized (`handlers.cpp:21`). No change needed.
- **GS** currently returns the raw stored dict, so dataclass `tuning` defaults are
  invisible. This spec adds **default-materialization** (C3) so `GET /gs/config`
  renders the effective tuning with defaults filled in, matching the drone.

## Current inventory (post IDR/OSD)

### Drone `dynamicLink.*` (17 fields, all tunable)

| Block | Fields | Disposition |
|---|---|---|
| scalars | `enabled, healthTimeoutMs, applyStaggerMs, applySubPaceMs` | keep |
| `roiQp` | `thresholdKbps, lowAnchorKbps, floor, step` | keep |
| `safe` | `mcs, k, n, overheadPct, deadlineMs, bitrateKbps` | keep; `bandwidth`/`txPowerDbm` **removed → derived** (B5) |
| `bitrate` | `minBitrateKbps, maxBitrateKbps` | **merge → `compute`** (B1) |
| `fec` | `baseRedundancyRatio, blocksPerFrame, kMin, kMax` | **merge → `compute`** (B1) |

Top-level `osd.enabled` (tunable, already lifted). Frozen constants:
`txpower_curve.hpp`, `probe_constants.hpp`, `idr_constants.hpp`,
`osd_constants.hpp`, OpenIPC rate table.

### GS top-level `dynamicLink.*`

`enabled, maxMcs, radioProfile, droneAddr, dronePort, videoStreamId` (tunable),
`tuning{}` (opaque passthrough → validated, C2).

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
→ **move under `dynamicLink.tuning.probe`**.

GS deprecated-key detector sets: `_DEPRECATED_LEADING_KEYS`,
`_DEPRECATED_GATE_KEYS`, `_DEPRECATED_PHASE3A_KEYS` → **remove** (clean break).

## Section A — Delete dead (GS only; drone has none)

1. `gate.max_mcs_step_up` — promote is hardcoded `current+1`; never read.
2. `profile_selection.hold_fallback_mode_ms` (already deprecation-warned),
   `fast_downgrade`, `upward_confidence_loops` — none read by `select()`.
3. `smoothing.ewma_alpha_burst` + the `burst_rate / holdoff_rate / late_rate`
   signal chain in `signals.py` (per-window `_w` + EWMA fields) — computed every
   window, consumed by nothing (not even logged).
4. The three `_DEPRECATED_*` detector sets in `config_build.py` — they warn on
   ~47 long-removed keys; the strict validator (C2) subsumes their purpose.

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

**B5. Slim `safe`: derive `bandwidth` and `txPowerDbm`**
Drop `dynamicLink.safe.bandwidth` and `dynamicLink.safe.txPowerDbm` from config
(schema + `validate.cpp` checks).

- `txPowerDbm` is **already dead**: `dispatchTxSafe` pushes
  `txpowerDbmForMcs(safe.mcs)` (the frozen curve) and never reads the configured
  value — so removal is zero behavior change.
- `bandwidth` is derived from the operating `link.width`
  (`modulationWidth(link.width)`, identical to `linkBandwidth`). The safe rung
  must not change bandwidth — a NIC retune drops the link — so sourcing it from
  the operating width is both dedup and a correctness fix.

`SafeDefaults` shrinks to `{mcs, k, n, overheadPct, deadlineMs, bitrateKbps}`;
`dispatchTxSafe` uses `cfg.linkBandwidth` + `txpowerDbmForMcs(cfg.safe.mcs)`.
The remaining fields stay explicit — a failsafe's k/n/overhead/deadline/bitrate
are deliberate recovery values, not derived.

## Section C — Defaults dedup, GS validation & discovery

**C1. Drone: code is the single default source; drop `defaults.json`.**
- `loadEffective` and `computeOverlay` use the serialized code struct
  (`nlohmann::json(Config{})`) as the defaults baseline instead of parsing
  `defaults.json`.
- The loader tolerates an absent defaults file (today it hard-fails — the `/rom`
  shadow gotcha) and falls back to `Config{}`.
- Remove `drone/etc/defaults.json` and its CMake `install(FILES …)` rule.
- `GET /config` still returns the full effective config; `PATCH` still writes a
  sparse overlay diffed against the code defaults. One source: the struct.

**C2. GS: replace the opaque `tuning` passthrough with a validated nested
schema.** A `_validate_tuning(tuning)` recurses the known structure
(`gate, policy, learned_prior(.flightlog), smoothing, probe`), type/range-checks
each value, and **rejects unknown keys at every level**. Wire into
`validate_effective` (boot) and `validate_config_patch` (PATCH); extend
`_validate_dynamic_link` to reject unknown `dynamicLink.*` keys too. Dead keys
(A) and typos become explicit load-time errors.

**C3. GS: materialize tuning defaults into `GET /gs/config`.**
Render the effective tuning with dataclass defaults filled in so the advanced
knobs are discoverable via the API (matching the drone). Single source for the
tuning defaults = the dataclasses.

**C4. Docs.** New `docs/dynamic-link-tuning.md` enumerating every tunable knob
(grouped operational/advanced for readability) with default + valid range, both
daemons. With no `defaults.json`, this + `GET /config` are the discovery surface.

**Follow-up (out of scope here): GS defaults-centralization.**
The GS keeps `gs/etc/defaults.json` for now — its top-level
`link/wfb/drone/pixelpilot/idrForward` defaults live only in the file (plus
scattered `.get(k, literal)` literals), not in a code schema. A later phase
centralizes those into one code layer and drops the GS `defaults.json` the same
way C1 does on the drone. Tracked here, not implemented in this spec.

## Migration (clean break)

**GS overlay** (`/etc/fpvd/config.json`), before/with the deploy:
- `probe.{rxL, ewmaAlpha, blackoutWindows}` → `dynamicLink.tuning.probe.{…}`
  (the live `rxL=800`). The old top-level `probe` block must be removed.
- `profile_selection.{hold_modes_down_ms, min_between_changes_ms}` →
  `dynamicLink.tuning.gate.{…}`; drop any other `profile_selection.*`.
- Drop `gate.max_mcs_step_up`, any `rssi_norm` override, any `_DEPRECATED_*`-era
  keys, and `smoothing.ewma_alpha_burst`.

The strict validator + fatal boot validation means **a leftover key blocks GS
boot**, so the overlay must be clean before the GS daemon restarts; the deploy
step validates the overlay against the new schema first.

**Drone overlay:** nlohmann-tolerant, so stale `safe.bandwidth`/`safe.txPowerDbm`
(or any removed key) are silently ignored — no boot risk; clean them out anyway.
Dropping the shipped `defaults.json` doesn't touch the operator overlay.

## Testing

- **Drone doctest:** loading with **no defaults file** yields a fully-defaulted
  `Config` (== `Config{}` ⊕ overlay); `computeOverlay` diffs against `Config{}`;
  `GET /config` returns the full effective struct; `compute`-block round-trip
  (B1); `safe` fallback unchanged with derived bandwidth/txpower (B5).
- **GS pytest (suite must stay green):** tuning validation good/bad/unknown (C2);
  `GET /gs/config` materializes tuning defaults (C3); dead-key removal (A);
  `profile_selection`→`gate` fold preserves selector behavior (B2); frozen
  rssi_norm + curve drift-guard test (B4); probe-block move (B3).
- The GS suite is import/config-coupled, so all changes land together.

## Risks

- **Boot-loop on a forgotten GS overlay key** — accepted (decision 4); mitigated
  by a deploy-time overlay validation step + the Migration checklist.
- **Drift between the GS rssi_norm constant and the drone curve** — mitigated by
  the B4 drift-guard test.
- **No human-readable drone baseline file** after C1 — mitigated by `GET /config`
  (full effective) + the C4 reference doc; optionally ship a commented
  `config.example.json` as docs only (not loaded).
- **GS keeps its defaults duplication** until the follow-up centralization — the
  scattered `.get(k, literal)` literals + `gs/etc/defaults.json` remain in sync by
  hand for top-level config; this spec only de-dups the drone and the GS `tuning`
  subtree.
