# waybeam Resilience preset — design

**Date:** 2026-06-15
**Status:** approved (design); implementation pending
**Scope:** drone (`fpvd`) only

## Summary

Expose waybeam's single `video0.resilience` knob through fpvd's config as
`video.resilience`. The preset selects an error-resilience profile; waybeam
internally derives intra-refresh (rolling GDR stripe), the SVC-T reference
pyramid, and GOP length from it — there are no per-feature knobs to plumb. fpvd
validates the value against the known preset list, writes it into the generated
`/etc/waybeam.json`, and applies a change by bouncing the waybeam process (the
existing "restart" field path).

Reference: <https://github.com/OpenIPC/waybeam_venc#resilience-preset-star6e--maruko>

### waybeam contract (as documented)

- `video0.resilience` is a string, mutability **reboot**, accepted values:
  `off | rescue | quality | sprint | racing | endurance | patrol | rally |
  range | fpv`, default `off`.
- A single field picks the profile; `intra_refresh_*` and `ref_*` are derived
  automatically and are **not** user-settable.
- `video0.gop_size` is honored **only** when `resilience == "off"`.
- waybeam reports a resilience change requires a reboot on Star6E/Maruko and
  returns `{"reboot_required": true}` from its own HTTP API.

## Decisions (resolved during brainstorming)

1. **Apply semantics — process restart, verify on bench.** fpvd rewrites
   `/etc/waybeam.json` and bounces the waybeam process (the existing venc
   teardown + ~700 ms settle). We bet the pipeline rebuild applies intra-refresh
   without a full device reboot. This is a **verification risk** (see below):
   waybeam's own API claims a reboot is required. If the bench shows the change
   does not take effect without a full reboot, the follow-up is to add an
   auto-reboot-on-resilience-change path. Out of scope for this spec.

2. **Validation — strict enum.** fpvd knows the 10 presets and rejects unknown
   values at `PATCH /config` time, matching how `rcMode`/`codec` validate.

3. **DL coupling — operator-owned, never locked.** Resilience is a flight-profile
   choice. The dynamic-link controller never mutates it, so it stays `PATCH`-able
   even while `dynamicLink.enabled`. The IDR relay and DL controller are
   untouched.

## Architecture & changes

All changes are in the drone daemon. Touch points map onto the existing waybeam
config pipeline (struct → translate → diff/apply).

### 1. Schema (`drone/src/config/schema.hpp`)

Add to `struct Video`:

```cpp
std::string resilience{"off"};   // off|rescue|quality|sprint|racing|endurance|patrol|rally|range|fpv
```

Default `"off"` preserves current GOP-based behavior exactly.

### 2. Validation (config validator)

- Define the allowed set once, as a single constant (single source of truth for
  the enum):
  `{"off","rescue","quality","sprint","racing","endurance","patrol","rally","range","fpv"}`.
- Reject an out-of-set `video.resilience` with a clear `400` (consistent with the
  existing `rcMode`/`codec` validation messages), before the value reaches
  `pending_`.

### 3. Translate (`drone/src/translate/waybeam.cpp` — `toWaybeamJson`)

- Emit `video0.resilience = c.video.resilience`.
- Leave the existing `video0.gop_size` mapping unchanged. Add a code comment
  noting waybeam ignores `gop_size` when `resilience != "off"` (per its
  contract), so no lock/hide logic is warranted.

### 4. Diff / apply classification (`drone/src/translate/waybeam.cpp` — `waybeamConfigDiff`)

- A change to `resilience` goes in the **restart** bucket under key
  `video0.resilience`. Never live (no `/api/v1/set` push).
- The existing apply flow then rewrites `/etc/waybeam.json` and calls
  `orch_.restart("waybeam", waybeamRestartSettleMs)`.
- **No** `restartOsd()` — resolution is unchanged, so the msposd canvas is
  unaffected.

### 5. DL lock (`drone/src/config/lock.cpp`)

- No change. `video.resilience` is deliberately **absent** from the lock list so
  it remains settable while `dynamicLink.enabled`.

## Data flow

```
PATCH /config {video:{resilience:"fpv"}}
  → validate enum (reject unknown → 400)
  → deep-merge into pending_
POST /apply
  → waybeamConfigDiff(effective_, pending_) → restart bucket has video0.resilience
  → persist full config.json + rewrite /etc/waybeam.json (toWaybeamJson)
  → orch_.restart("waybeam", waybeamRestartSettleMs)   # ~700 ms venc settle
  → effective_ = pending_; report "waybeam" in restarted[]
```

## Error handling

- **Unknown preset:** `400` at PATCH, before apply. No partial state.
- **waybeam restart failure:** handled by the existing orchestrator restart path
  (unchanged); surfaced in the apply result like any other restart failure.

## Scope boundaries (explicitly out)

- **GS:** no change. Resilience is drone-only video config, reached opaquely via
  `/air/*`. No `deploy/gs/deploy.sh` scp-list change (no new subpackage).
- **Device reboot:** not implemented. Treated as a verification risk per
  decision (1).
- **OSD-artifact safety:** some presets are not "OSD-safe" per waybeam's table.
  This is a waybeam/operator concern; documented, not gated by fpvd.
- **gopSize behavior:** unchanged; documented as inert when `resilience != "off"`.
- **Dynamic-link / IDR relay logic:** untouched.

## Testing (doctest, TDD)

In `drone/tests/`:

- `toWaybeamJson` emits `video0.resilience`; a default `Video` round-trips as
  `"off"`.
- `waybeamConfigDiff`: a resilience change lands in the `restart` map (key
  `video0.resilience`), and **not** in `live`.
- Validator: accepts each of the 10 presets; rejects an unknown value with the
  expected error.
- Lock regression: a `video.resilience` PATCH succeeds while
  `dynamicLink.enabled` (guards that resilience is not locked).
- Config (de)serialization includes `resilience` and preserves its value.

Run from `drone/`: `./build/fpvd_tests` (not `ctest`).

## Verification on hardware (bench)

After deploy to the camera:

1. `PATCH /air/config {video:{resilience:"<preset>"}}` then `POST /air/apply`;
   confirm `waybeam` appears in `restarted[]`.
2. Confirm the generated `/etc/waybeam.json` carries `video0.resilience`.
3. Confirm the encoder actually adopts intra-refresh **after the process bounce
   alone** (no full device reboot) — e.g. via waybeam logs / observed recovery
   behavior. This is the open risk from decision (1); if it fails, open a
   follow-up for an auto-reboot path.
