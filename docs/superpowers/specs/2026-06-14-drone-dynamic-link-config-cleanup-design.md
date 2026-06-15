# Drone Dynamic-Link Config Cleanup & Defaults Dedup — Design

**Date:** 2026-06-14
**Status:** Approved (pending spec review) → implementation plan next
**Scope:** **Drone (`fpvd`) only.** The GS-side cleanup — dead-knob removal,
`profile_selection`→`gate`, `probe`→`tuning`, rssi_norm freeze, tolerant `tuning`
validation, `GET /config` materialization, and GS defaults-centralization — is
deferred to its own spec.
**Builds on:** `refactor/decouple-idr-osd` (commit `b9a81b4`) — the IDR and OSD
decouplings were the first slices of this effort.

## Motivation

The drone's `dynamicLink` config has two issues this spec fixes:

1. **Defaults are maintained twice.** The code already holds every default
   (nlohmann `..._WITH_DEFAULT` structs, ~61 of them), and the shipped
   `drone/etc/defaults.json` mirrors those values — so each change touches two
   places (this very cleanup kept editing `schema.hpp` *and* `defaults.json`).
2. **Redundant / derivable knobs.** `bitrate` and `fec` are two schema blocks
   feeding one engine; `safe.bandwidth`/`safe.txPowerDbm` duplicate values that
   are already derivable (`safe.txPowerDbm` is in fact dead).

The fix: merge the redundant blocks, slim `safe`, and make **code the single
source of defaults** behind a **single full `config.json`** with a **tolerant
loader**. The result is a two-bucket model — *tunable* vs *frozen* — with one
place to define a default.

## Decisions (locked)

1. **Full execution** — make the changes in code now.
2. **Tolerant of stale keys (warn + default).** The loader merges `config.json`
   onto the code defaults: a present key wins; a **missing key falls back to the
   code default**; an **unknown / deprecated / renamed key logs a warning and is
   ignored** (a renamed key's new name is absent → it takes the default). This is
   what makes upgrades safe — new / removed / renamed keys never break a load.
   **Value validation is unchanged**: a present key with a wrong-typed or
   out-of-range value is still a normal config error (a deliberate mistake, not
   an upgrade artifact). (Cost: a renamed/typo'd key silently uses the default,
   surfacing only as a warning.)
3. **Single full `config.json`; code is the default source.** No `defaults.json`,
   no sparse overlay. `config.json` holds the full set of tunable knobs and is
   **generated from the code defaults** (`--dump-config` / first boot), so it's a
   code-derived artifact, never a hand-maintained second copy.

Calibration constants are already frozen on the drone (`txpower_curve.hpp`,
`probe_constants.hpp`, `idr_constants.hpp`, `osd_constants.hpp`, OpenIPC rate
table) — they stay as-is; this spec only documents them.

## Non-goals

- No adaptive-link *algorithm* change (config plumbing only).
- No GS↔drone wire change (v3 `{mcs}` packet untouched).
- **All GS-side config work** — its own spec (see Scope).
- Renaming `osdUpdateIntervalMs` (misnamed control-loop tick cadence) — future pass.

## The config model (two buckets)

| Bucket | Mechanism | Discovery |
|---|---|---|
| **Tunable** | full `config.json` (all knobs), merged onto code defaults; tolerant load | the file is itself the full list; `GET /config` returns the full effective config |
| **Frozen** | compile-time / module constant; no config path | the reference doc + the header |

Defaults are defined once in code; the full `config.json` is materialized from
them. The operator edits the deployed copy. New-build behavior under tolerant
load:

| New build… | Result |
|---|---|
| adds a key | missing from your file → code default fills it (re-dump to write it in) |
| changes a default | your file pins whatever it holds; a changed code default reaches only keys *not* in your file — re-dump to adopt |
| renames a key | old name → warning, ignored; new name → code default |
| deprecated key present | warning log, ignored |

`GET /config` already returns `nlohmann::json(effective())` — the full struct
with all defaults materialized (`handlers.cpp:21`) — so discovery needs no
change on the drone.

## Current `dynamicLink.*` inventory (post IDR/OSD; 17 fields, all tunable)

| Block | Fields | Disposition |
|---|---|---|
| scalars | `enabled, healthTimeoutMs, applyStaggerMs, applySubPaceMs` | keep |
| `roiQp` | `thresholdKbps, lowAnchorKbps, floor, step` | keep |
| `safe` | `mcs, k, n, overheadPct, deadlineMs, bitrateKbps` | keep; `bandwidth`/`txPowerDbm` **removed → derived** (Change 2) |
| `bitrate` | `minBitrateKbps, maxBitrateKbps` | **merge → `compute`** (Change 1) |
| `fec` | `baseRedundancyRatio, blocksPerFrame, kMin, kMax` | **merge → `compute`** (Change 1) |

Top-level `osd.enabled` (tunable, already lifted). The DL loop also *reads*
`link.{width, stbc, ldpc, fec.*, mtu}` and `video.fps` — borrowed inputs, not
owned by `dynamicLink`. No dead knobs on the drone (all 17 are consumed).

## Changes

### Change 1 — merge `bitrate` + `fec` → `dynamicLink.compute`

`{minBitrateKbps, maxBitrateKbps, baseRedundancyRatio, blocksPerFrame, kMin,
kMax}` in one `DynamicLinkCompute` struct feeding `BitrateEngineConfig`. Update
the `runtime_config.cpp` mapping, add `validate.cpp` ranges (`min>0`,
`max>min`, `baseRedundancyRatio>0`, `blocksPerFrame>0`, `1<=kMin<=kMax`), and
tests. (These blocks are schema-only today, so no defaults.json change beyond
the dedup in Change 3.)

### Change 2 — slim `safe`: derive `bandwidth` and `txPowerDbm`

Drop `dynamicLink.safe.bandwidth` and `dynamicLink.safe.txPowerDbm` from schema +
`validate.cpp`.

- `txPowerDbm` is **already dead**: `dispatchTxSafe` pushes
  `txpowerDbmForMcs(safe.mcs)` (the frozen curve) and never reads the configured
  value — removal is zero behavior change.
- `bandwidth` is derived from the operating `link.width`
  (`modulationWidth(link.width)`, identical to `linkBandwidth`); the safe rung
  must not change bandwidth (a NIC retune drops the link).

`SafeDefaults` shrinks to `{mcs, k, n, overheadPct, deadlineMs, bitrateKbps}`;
`dispatchTxSafe` uses `cfg.linkBandwidth` + `txpowerDbmForMcs(cfg.safe.mcs)`. The
remaining fields stay explicit — a failsafe's k/n/overhead/deadline/bitrate are
deliberate recovery values.

### Change 3 — single full `config.json` + tolerant load; drop `defaults.json`

- Remove `drone/etc/defaults.json` and its CMake `install(FILES …)` rule.
- **Load:** parse `config.json` if present and deep-merge onto `Config{}` (the
  code defaults) so any missing key defaults; an absent file → run on `Config{}`
  (today the loader hard-fails on a missing defaults file — the `/rom` shadow
  gotcha — that path disappears).
- **Warn-on-unknown:** nlohmann drops unknown keys silently, so add a pass that
  walks the file's JSON and logs a warning on any key not in the schema's
  known-key set (derive the known set by serializing `Config{}`). Value
  validation via `validate.cpp` is unchanged — a wrong-typed or out-of-range
  value of a known key is still a normal config error.
- **`PATCH`** merges + persists the **full** effective config back to
  `config.json` (no sparse diff — remove `computeOverlay`). `GET /config`
  unchanged. The existing `dynamic_link_locked` PATCH guard is unaffected.
- **`--dump-config`** writes `nlohmann::json(Config{})` so the initial full file
  is code-generated.

### Change 4 — docs

`docs/dynamic-link-tuning.md`: each drone tunable knob (purpose + valid range,
grouped operational/advanced) and the frozen constants. The generated
`config.json` is the canonical inventory; the doc adds semantics.

## Migration (tolerant — warns, never bricks)

The drone overlay is nlohmann-tolerant and the new loader warns rather than
fails, so nothing bricks. A removed key (`safe.bandwidth`, `safe.txPowerDbm`, the
already-gone `minIdrIntervalMs`, `osd.*`) just warns and is ignored. Dropping the
shipped `defaults.json` doesn't touch `config.json`. Cleanest path on deploy:
**regenerate** `config.json` via `--dump-config`, then re-apply real overrides.

## Testing (drone doctest, run from `drone/`)

- **Tolerant load:** missing key → code default; unknown key → warning
  (captured) + ignored; **no config file → `Config{}`**; an out-of-range known
  value still fails `validate()` as today.
- **Persistence:** `PATCH` writes the full effective config; round-trips.
- **`--dump-config`** emits `nlohmann::json(Config{})`.
- **Change 1:** `compute` round-trip + the new validation ranges.
- **Change 2:** `safe` fallback unchanged — `dispatchTxSafe` uses `linkBandwidth`
  + `txpowerDbmForMcs(safe.mcs)`; positional `SafeDefaults` inits updated.
- Build host-only; `./build/fpvd_tests` from `drone/` (not `ctest`).

## Risks

- **A typo'd / renamed key silently uses the default** (warning only) — accepted
  cost of tolerant-over-strict (decision 2); the regenerate-the-file workflow
  avoids stale names.
- **Changed code defaults don't propagate** to keys already in the full
  `config.json` — re-dump to adopt.
- **No human-readable baseline file** after dropping `defaults.json` — mitigated
  by `GET /config` (full effective), `--dump-config`, and the Change 4 doc.
