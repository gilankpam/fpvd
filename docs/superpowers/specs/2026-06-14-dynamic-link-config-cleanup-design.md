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
2. **Clean break** — no back-compat shims; the live GS `config.json` is
   hand-migrated to the new key names (see Migration).
3. **Freeze calibration to constants** — `txpower_curve`, the probe/idr/osd
   constants, and the GS rssi_norm curve. (The IDR/OSD constants from the prior
   commit are the precedent.)
4. **Tolerant load — warn, never fatal.** The loader merges `config.json` onto
   the code defaults: a key present in the file wins; a missing key falls back to
   the code default; an **unknown / deprecated / renamed key logs a warning and
   is ignored** (a renamed key resets to the code default while its old name
   warns); an **invalid value for a known key also warns and falls back to the
   code default**. Config problems never block boot. (Reverses an earlier
   strict-reject idea — removes the boot-loop risk entirely; the cost is that a
   typo'd key silently uses the default and only surfaces as a log line.)
5. **Single full `config.json`; code is the default source.** No separate
   `defaults.json`, no sparse overlay. `config.json` holds the full set of
   tunable knobs, and the code struct / dataclasses are the single source of
   default *values* — used as the fallback for missing keys and to **generate**
   the shipped full file (so it is a code-derived artifact, never a
   hand-maintained second copy of the defaults). (GS top-level defaults need the
   centralization follow-up to be code-sourced; see Section C.)

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
| **Tunable** | full `config.json` (all knobs), merged onto code defaults; tolerant load (warn, never fatal) | the file is itself the full list; `GET /config` also returns the full effective config |
| **Frozen** | compile-time / module constant; no config path | the reference doc + the header |

There is **no `defaults.json`** and **no sparse overlay** — one full file plus
code defaults as the fallback. The old Tier-1-vs-Tier-2 "in the file or not"
distinction dissolves; "operational vs advanced" survives only as *documentation
grouping*, not a config mechanism.

**The file is generated, not hand-written.** Defaults are defined once in code;
the full `config.json` is materialized from them (`--dump-config` / first boot),
so the file is a code-derived artifact rather than a second hand-maintained copy
of the defaults. The operator then edits the deployed copy.

**Load = tolerant merge** (decision 4): file value → else code default for a
missing key → unknown/invalid keys warn and fall back. New-build behavior:

| New build… | Result |
|---|---|
| adds a key | missing from your file → code default fills it (re-dump to write it in) |
| changes a default | your file pins whatever it holds; a changed code default reaches only keys *not* in your file — re-dump to adopt |
| renames a key | old name → warning, ignored; new name → code default |
| deprecated key present | warning log, ignored |

**Discovery.** The full `config.json` *is* the inventory of tunable knobs. `GET
/config` corroborates it with live values:
- **Drone** already returns `nlohmann::json(effective())` — the full struct
  (`handlers.cpp:21`).
- **GS** currently returns the raw stored dict, so dataclass `tuning` defaults are
  invisible. C3 adds **default-materialization** so `GET /gs/config` (and the
  generated file) render the full effective config, matching the drone.

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
   ~47 long-removed keys; the tolerant loader's generic warn-on-unknown (C2)
   subsumes their purpose with one consistent path.

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

## Section C — Single full config, tolerant load & discovery

**C1. Drone: single full `config.json`; code is the default source.**
- Drop `drone/etc/defaults.json` and its CMake `install(FILES …)` rule.
- Load: parse `config.json` if present and deep-merge it onto `Config{}` (the
  code defaults) so any missing key defaults; an absent file → run on `Config{}`
  (today the loader hard-fails on a missing defaults file — the `/rom` shadow
  gotcha — that path disappears).
- Warn-on-unknown: nlohmann silently drops unknown keys, so add an explicit pass
  that diffs the parsed JSON's keys against the schema and logs a warning per
  unknown/deprecated key. An invalid value for a known key → warn + keep the code
  default (never abort).
- `PATCH` merges + persists the **full** effective config back to `config.json`
  (no sparse diff — remove `computeOverlay`). `GET /config` unchanged.
- Generate the initial full file from `nlohmann::json(Config{})` (a
  `--dump-config` flag / install step) so it's code-derived, not hand-written.

**C2. GS: replace the opaque `tuning` passthrough with a *tolerant* validated
schema.** `_validate_tuning(tuning)` recurses the known structure (`gate, policy,
learned_prior(.flightlog), smoothing, probe`), type/range-checks known keys
(**warn + fall back to the dataclass default** on a bad value), and **warns on
unknown keys** (ignored, not rejected). Same treatment in `_validate_dynamic_link`
for unknown `dynamicLink.*`. One consistent warn path replaces both the
`_DEPRECATED_*` detector sets and the silent-typo no-ops — and, unlike a strict
reject, never blocks boot.

**C3. GS: materialize the full effective config.** Render the effective config
with all dataclass defaults filled in — used both for `GET /gs/config` and to
generate the full `config.json`. Single source for the tuning defaults = the
dataclasses.

**C4. Docs.** `docs/dynamic-link-tuning.md` documents each tunable knob (meaning +
valid range, grouped operational/advanced). The generated full `config.json` is
the canonical inventory; the doc adds the semantics.

**Follow-up (out of scope here): GS defaults-centralization.**
For "missing key → code default" and clean full-file generation, the GS needs
code defaults for its top-level `link/wfb/drone/pixelpilot/idrForward` keys.
Today those are the scattered `.get(k, literal)` literals (which *do* serve as
fallbacks, so tolerant load works now) plus `gs/etc/defaults.json`. A later phase
centralizes them into one code layer and drops `gs/etc/defaults.json` the way C1
does on the drone. Tracked here, not implemented in this spec.

## Migration (tolerant — warns, never bricks)

Tolerant load means a non-migrated key never blocks boot — but a **renamed key
resets to the code default**, so your tuned value silently reverts (with a
warning). Migration therefore still matters for *behavior*, just not for boot.

**GS `config.json`** — rename to the new locations so your values keep applying:
- `probe.{rxL, ewmaAlpha, blackoutWindows}` → `dynamicLink.tuning.probe.{…}`
  (otherwise the old top-level `probe` warns and `rxL` falls back to 50 — your
  `800` won't apply).
- `profile_selection.{hold_modes_down_ms, min_between_changes_ms}` →
  `dynamicLink.tuning.gate.{…}`.
- Drop `gate.max_mcs_step_up`, `rssi_norm`, `smoothing.ewma_alpha_burst`,
  `profile_selection.*` leftovers, `_DEPRECATED_*`-era keys (all warn + ignored).

Easiest path: **regenerate** the full `config.json` from code defaults, then
re-apply your handful of real overrides under the new key names.

**Drone:** stale `safe.bandwidth`/`safe.txPowerDbm` (or any removed key) warn +
ignored. Dropping the shipped `defaults.json` doesn't touch `config.json`.

## Testing

- **Drone doctest:** tolerant load — a missing key → code default; an **unknown
  key → warning** (captured) and ignored; an **invalid known value → warning +
  code default**; **no config file → `Config{}`**; `PATCH` persists the **full**
  effective config; `GET /config` returns the full struct; `--dump-config`
  emits `nlohmann::json(Config{})`; `compute`-block round-trip (B1); `safe`
  fallback unchanged with derived bandwidth/txpower (B5).
- **GS pytest (suite must stay green):** tolerant tuning validation — good value
  applies, bad value warns + defaults, unknown key warns + ignored (C2);
  `GET /gs/config` materializes the full effective config (C3); dead-key removal
  (A); `profile_selection`→`gate` fold preserves selector behavior (B2); frozen
  rssi_norm + curve drift-guard test (B4); probe-block move (B3).
- The GS suite is import/config-coupled, so all changes land together.

## Risks

- **A typo'd / renamed key silently uses the default** (warning only) — the
  intended setting doesn't apply unless the operator notices the log. This is the
  accepted cost of tolerant-over-strict (decision 4); the regenerate-the-file
  workflow avoids stale key names.
- **Changed code defaults don't propagate** to keys already present in the full
  `config.json` — re-dump to adopt new defaults.
- **Drift between the GS rssi_norm constant and the drone curve** — mitigated by
  the B4 drift-guard test.
- **GS keeps its top-level defaults duplication** until the follow-up
  centralization — the scattered `.get(k, literal)` literals + `gs/etc/defaults.json`
  stay hand-synced; this spec de-dups only the drone and the GS `tuning` subtree.
