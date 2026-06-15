# GS Dynamic-Link Config Cleanup & Defaults Dedup — Design

**Date:** 2026-06-15
**Status:** Approved (pending spec review) → implementation plan next
**Scope:** **GS (`fpvdgs`) only.** The drone-side cleanup is already done on this
branch (`2026-06-14-drone-dynamic-link-config-cleanup-design.md`). This is the
deferred GS phase that drone spec called out: dead-knob removal,
`profile_selection`→`selector` merge, `tuning` de-opacification, probe/learned-prior
freeze, code-as-default-source + tolerant load + `--dump-config`.
**Builds on:** `refactor/decouple-idr-osd` — the drone config cleanup commits.

## Motivation

The GS `dynamicLink` config has three issues this spec fixes:

1. **The knobs are invisible and unvalidated.** Every advanced knob rides in an
   opaque `dynamicLink.tuning` passthrough whose defaults live *only* in the
   Python dataclasses, so `defaults.json` ships `tuning: {}` and
   `GET /gs/config` shows nothing. An operator cannot discover a knob, see its
   effective value, or get a typo rejected — a mistyped `tuning` key is silently
   ignored.
2. **Dead and deprecated knobs everywhere.** `config_build` carries three
   `_DEPRECATED_*` lists plus dead parse branches; several "live" dataclass
   fields are parsed but never consumed; the shipped `deploy/gs/config.json` is
   ~80% dead/deprecated keys; a top-level `probe` block is read at boot but is
   API-unreachable (orphaned).
3. **Defaults are split between a data file and code.** Top-level defaults live
   in `gs/etc/defaults.json`; `tuning` defaults live in dataclasses. The drone
   just resolved the equivalent split by making code the single default source
   behind a single full `config.json` with a tolerant loader. The GS should
   match for cross-daemon consistency.

The fix mirrors the drone: **code is the single source of defaults** behind a
**single full `config.json`** with a **tolerant loader**; the opaque `tuning`
block is replaced by explicit, validated, materialized sub-blocks; dead knobs are
removed; and calibration/internal knobs are **frozen** as code constants. The
result is the same two-bucket model — *tunable* vs *frozen* — with one place to
define a default.

## Decisions (locked)

1. **Mirror the drone config model, daemon-wide.** The model parts are one-file /
   one-loader by nature (`ConfigStore` loads one config tree), so they apply to
   the whole GS config: code = single default source; single full `config.json`
   (no sparse overlay); tolerant warn-on-unknown load; `--dump-config`; drop
   `defaults.json`; `GET /gs/config` returns the full effective config.
2. **DL-only knob restructuring.** Other blocks (`link`, `wfb`, `drone`,
   `idrForward`, `pixelpilot`) come along for the *model* migration (they get
   code defaults, mechanically moved from `defaults.json`) but their knobs are
   untouched.
3. **`learnedPrior` is always-on and fully frozen.** `enabled` is hard-wired
   true (the controller constructs the prior unconditionally whenever dynamic
   link runs); the 12 internals become code constants. The block leaves
   `config.json` entirely.
4. **Probe knobs frozen.** `rxL` / `ewmaAlpha` / `blackoutWindows` become code
   constants; the orphaned `effective['probe']` read is deleted. `rxL` is frozen
   at **50 ms** (the current code constant) — consistent with the exposed
   `selector.probeFreshnessMs` = 500 ms, so a probed rung never reads stale
   between `wfb_rx` stats batches.
5. **`flightlog` and `rssiNorm` expose only their `enabled` toggle**, freezing
   the rest (storage paths/caps for flightlog; the EIRP curve for rssiNorm).

## Non-goals

- No adaptive-link *algorithm* change (config plumbing only).
- No GS↔drone wire change (v3 `{mcs}` packet untouched).
- No knob-level audit of non-DL blocks (`pixelpilot` etc.) — they get code
  defaults only.
- **No `radioProfile` rename.** It is no longer a "profile" (it keys the
  learned-prior persistence file + the drone adapter-match warning), but renaming
  it would orphan persisted `/etc/fpvd/learned/<name>.json` history under the
  tolerant loader. It stays as-is; only its docs are corrected.

## The config model (two buckets)

| Bucket | Mechanism | Discovery |
|---|---|---|
| **Tunable** | full `config.json` (all exposed knobs), merged onto code defaults; tolerant load | the file itself + `GET /gs/config` (full effective) |
| **Frozen** | module / dataclass-default constant; no config path | the reference doc + the source |

Defaults are defined once in code; the full `config.json` is materialized from
them via `--dump-config`. New-build behavior under tolerant load matches the
drone: a missing key takes the code default; an unknown/deprecated/renamed key
logs a warning and is ignored; a present key with a wrong-typed or out-of-range
value is still a normal config error (value validation is unchanged).

## Current GS `dynamicLink` inventory & disposition

Curated top-level keys (consumed by `make_dl_snapshot`, validated in `schema.py`):
`enabled, maxMcs, radioProfile, droneAddr, dronePort, videoStreamId` + the opaque
`tuning`. `tuning` is reshaped by `config_build._raw_from_block` into a vestigial
"gs.yaml-shaped" dict (no `gs.yaml` file exists — historical layout from the
retired standalone `S99dynamic-link-gs`) and fed to the policy/aggregator
dataclasses.

| Disposition | Knobs |
|---|---|
| **EXPOSE** (in `config.json`, validated, GET-visible) | `enabled, maxMcs, radioProfile, droneAddr, dronePort`; **`selector`{}** = `probeViableThreshold, probeFreshnessMs, promoteDebounceWindows, videoDemotePer, emergencyLossRate, emergencyFecPressure, holdModesDownMs, minBetweenChangesMs, starvationWindows`; **`smoothing`{}** = `ewmaAlphaRssi, ewmaAlphaFec, ewmaAlphaBurst, starvationThresholdPps`; **`flightlog`{}** = `enabled`; **`rssiNorm`{}** = `enabled` |
| **FREEZE** (code constant; documented in the reference doc) | `videoStreamId` = `"video"`; `rssiNorm` curve (`pRefDbm` = 29, `txPowerDbmByMcs` = `(29,28,25,23,19,19,19,19)` — drone-mirror calibration); **all `learnedPrior`** (`enabled` = true + the 12 internals: `binWidthDb, rssiMin, rssiMax, ewmaAlpha, viableThreshold, minSamplesWarmstart, minSamplesPredictive, warmstartMargin, predictiveHorizonTicks, predictiveDebounceWindows, flushIntervalObservations, persistDir`); **all `probe`** (`rxL` = 50, `ewmaAlpha` = 0.25, `blackoutWindows` = 10, `port` = 50); `flightlog.{dir, maxFiles, maxMb, flightGapS}`; `stats_endpoint` = `tcp://127.0.0.1:8103`, `WINDOW_S` = 0.1, `MAX_MCS` = 7 |
| **DELETE** (dead/deprecated → gone) | the opaque `tuning` block + `_raw_from_block` reshape; the 3 `_DEPRECATED_*` lists + their warning code; dead parse branches (`leading_loop`, `policy.bitrate`, `fec`, `video`, `cooldown`, `safe_defaults`); dead dataclass fields (`GateConfig.max_mcs_step_up`, `ProfileSelectionConfig.{hold_fallback_mode_ms, fast_downgrade, upward_confidence_loops}`); dead seeded-overlay keys (`dynamicLink.{bandwidth, txpower, idrForward, idrPort}`); stale `api.md` DL section + stale `gs/build/lib/.../profiles/*.json` artifact |
| **MERGE** | `ProfileSelectionConfig` → folded into a single `SelectorConfig` (its two live timing knobs `holdModesDownMs`/`minBetweenChangesMs`) + `starvationWindows` absorbed from `PolicyConfig`; the `selection` and `policy` blocks vanish. Net: the `tuning`
sub-block sprawl collapses to **2** multi-knob config blocks (`selector`,
`smoothing`) + two enable-only toggles (`flightlog`, `rssiNorm`). |

Dead-field verification: `max_mcs_step_up`, `fast_downgrade`, `upward_confidence_loops`,
and `hold_fallback_mode_ms` each appear only at their dataclass-default line — no
consumer in the probe-driven selector reads them (confirmed by grep over
`fpvdgs/dynlink/*.py`). `gate.max_mcs` is always overridden by the curated
`maxMcs`, so it is not a separate exposed knob.

## Resulting `config.json` `dynamicLink`

```jsonc
"dynamicLink": {
  "enabled": false, "maxMcs": 5, "radioProfile": "m8812eu2",
  "droneAddr": null, "dronePort": 9999,
  "selector": { "probeViableThreshold": 0.99, "probeFreshnessMs": 500,
    "promoteDebounceWindows": 3, "videoDemotePer": 0.05,
    "emergencyLossRate": 0.05, "emergencyFecPressure": 0.80,
    "holdModesDownMs": 2000, "minBetweenChangesMs": 200, "starvationWindows": 5 },
  "smoothing": { "ewmaAlphaRssi": 0.2, "ewmaAlphaFec": 0.2,
    "ewmaAlphaBurst": 0.1, "starvationThresholdPps": 50 },
  "flightlog": { "enabled": true },
  "rssiNorm": { "enabled": true }
}
```
Everything frozen or dead is simply absent. JSON keys are camelCase (matching the
GS surface, e.g. `maxMcs`); `config_build` maps them to the dataclasses' snake_case
fields exactly as `maxMcs`→`max_mcs` does today.

## Changes

### Change 1 — model migration (daemon-wide)

- **Code = default source.** New `fpvdgs/config_defaults.py::default_config() -> dict`
  builds the full default tree: the non-DL blocks (`link/wfb/drone/idrForward/
  pixelpilot`) as a literal moved verbatim from `defaults.json`; the
  `dynamicLink` subtree assembled from the dataclasses so its defaults stay DRY
  with the code that consumes them.
- **`ConfigStore` rewrite** (`config.py`): `load(config_path)` (no `defaults_path`)
  parses `config.json` if present and deep-merges it onto `default_config()`;
  an absent file runs on pure defaults. `effective()` = `deep_merge(defaults,
  loaded)`. `commit()` persists the **full** effective config (atomic temp +
  `os.replace`, already the pattern) — no sparse diff. `reset()` clears overrides
  → persists the full defaults.
- **Tolerant warn-on-unknown.** On load, walk the file's JSON against the
  known-key set derived from `default_config()` (recursively); log a warning on
  any unknown key and ignore it. This **replaces** the bespoke `_DEPRECATED_*`
  lists. Value validation via `schema.validate_effective` is unchanged.
- **`--dump-config`** (`supervisor.main`): print `default_config()` as JSON to
  stdout; drop the `--defaults` arg; keep `--config`.
- **`GET /gs/config`** is unchanged (returns `store.effective()`) — it now
  materializes every exposed knob, closing the discoverability gap for free.
  `GET /gs/defaults` returns `default_config()`.

### Change 2 — de-opacify `dynamicLink` (flatten `tuning`)

- Replace the opaque `tuning` passthrough with explicit `selector` / `smoothing`
  / `flightlog` / `rssiNorm` sub-blocks (shape above). Delete `_raw_from_block`
  and the "gs.yaml-shaped" indirection; `config_build` maps the explicit blocks
  straight onto the dataclasses.
- **Validate** the new blocks in `schema._validate_dynamic_link` with ranges:
  probabilities/PER in `[0,1]`; EWMA alphas in `(0,1]`; positive ints for
  windows/ms; `maxMcs` in `0..7` (kept). Two paths handle unknown keys
  differently, by design: **on PATCH** (interactive edit) an unknown
  `dynamicLink` sub-key is rejected so typos surface immediately (extends the
  existing top-level key-set check down into the block); **on load** (boot /
  upgrade) the tolerant loader warns + ignores, so a config from an older/newer
  build never bricks startup.

### Change 3 — merges

- **`ProfileSelectionConfig` → `SelectorConfig`.** Rename `GateConfig` to
  `SelectorConfig`, absorbing the two live `ProfileSelectionConfig` timing knobs
  (`hold_modes_down_ms`, `min_between_changes_ms`) and `PolicyConfig.starvation_windows`.
  Drop the dead fields (`max_mcs_step_up`, `hold_fallback_mode_ms`,
  `fast_downgrade`, `upward_confidence_loops`). `LeadingSelector(gate, sel)` →
  `LeadingSelector(cfg)`. The `selection` and `policy` config blocks disappear.

### Change 4 — freezes

- **`learnedPrior`:** delete the `learned_prior` parse in `config_build`; the
  `LearnedPriorConfig` dataclass defaults are the frozen source. `Policy`
  constructs the prior unconditionally (drop the `if cfg.learned_prior.enabled
  … else None` branch and collapse the `learned_prior is not None` guards).
- **`probe`:** delete the `effective.get("probe")` read in
  `probe/config_build.make_probe_snapshot`; `rxL`/`ewmaAlpha`/`blackoutWindows`
  come from the `PROBE_*` module constants (`PROBE_RX_L` stays 50). `port` is
  already `PROBE_PORT`.
- **`flightlog` internals / `rssiNorm` curve:** `config_build` reads only
  `flightlog.enabled` and `rssiNorm.enabled`; the rest of `FlightLogConfig` /
  `RssiNormConfig` defaults are the frozen source.
- **`videoStreamId`:** drop from the schema and `make_dl_snapshot`; the
  `DynamicLinkController` video-stream filter uses the constant `"video"`.

### Change 5 — deploy + docs

- **`deploy/gs/deploy.sh`:** stop shipping `gs/etc/defaults.json`; seed
  `/etc/fpvd/config.json` (when absent) from the installed `fpvd --dump-config`
  atomically (temp + `mv`, after the package is copied), mirroring the drone
  deploy. Preserve an existing operator `config.json`.
- Delete `gs/etc/defaults.json` and the stale checked-in `deploy/gs/config.json`
  (regenerated from `--dump-config`); remove the stale
  `gs/build/lib/.../profiles/*.json` artifact (verify it is not tracked/shipped).
- **`docs/api.md`:** rewrite the GS `dynamicLink` section to the flat schema;
  drop the `tuning`/`gs.yaml`/`profiles/<name>.json` references.
- New **`docs/gs-dynamic-link-tuning.md`** reference: each exposed knob (purpose +
  valid range, grouped operational/advanced) and the frozen constants
  (selector-internal, learned-prior, probe, rssi-norm curve, flightlog storage).

## Migration (tolerant — warns, never bricks)

The new loader warns rather than fails, so nothing bricks. The bench's stale
`config.json` (dead `tuning.*`, dead `dynamicLink.{bandwidth,txpower,idrForward,
idrPort}`, and any hand-added top-level `probe` block) loads with warnings and the
removed keys ignored; the surviving exposed knobs still take effect. Cleanest path
on deploy: **regenerate** `config.json` via `--dump-config`, then re-apply real
overrides. The learned-prior persistence (`/etc/fpvd/learned/<radioProfile>.json`)
is untouched (no `radioProfile` rename).

## Testing (GS pytest, full suite must stay green)

The GS suite must pass **as a whole** — config_build / import coupling means
partial refactors go red. Run `cd gs && .venv/bin/python -m pytest tests/ -q`.

- **Model:** `default_config()` materialization (full tree, every block present);
  tolerant load (missing key → default; unknown key → warning captured + ignored;
  no file → defaults); an out-of-range known value still fails validation;
  `commit`/`reset` persist the full effective config and round-trip;
  `--dump-config` emits `default_config()`.
- **Flatten/validate (Change 2):** explicit `selector`/`smoothing`/`flightlog`/
  `rssiNorm` parse into the dataclasses; the new validation ranges; `GET
  /gs/config` shows the materialized knobs.
- **Merge (Change 3):** `SelectorConfig` carries the timing + starvation knobs;
  `LeadingSelector` takes one config; behavior unchanged at defaults.
- **Freeze (Change 4):** `learnedPrior` is always constructed (no enabled knob);
  `probe`/`flightlog`-internals/`rssiNorm`-curve come from constants regardless of
  config; a stray `tuning`/`probe` block in input only warns.
- **Rework:** `test_dl_config_build.py` (tuning passthrough + deprecation tests
  removed/replaced), `test_schema.py`, `test_api.py`, `test_app_wiring.py`,
  `test_dl_policy_learned.py` (drop the disabled-path case).

## Risks

- **A typo'd / renamed exposed key silently uses the default** (warning only) —
  accepted cost of tolerant-over-strict, same as the drone; the
  regenerate-the-file workflow avoids stale names.
- **Frozen knobs need a code change + redeploy to tune** — accepted; the
  learned-prior internals, probe window, rssi-norm curve, and flightlog storage
  are calibration/internal, and freezes are reversible (re-expose later) if field
  experience demands.
- **No human-readable shipped baseline file** after dropping `defaults.json` —
  mitigated by `GET /gs/config` (full effective), `--dump-config`, and the Change
  5 reference doc.
