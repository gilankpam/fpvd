# Beamforming Hardening — Design

**Date:** 2026-06-09
**Status:** Approved design, pending implementation plan
**Context:** Beamforming was verified working end-to-end live (drone `cbr_rssi`
−48/−67 dBm), but getting there exposed three real defects that make
enable-via-`/link/apply` either silently no-op or unreliable. This spec fixes
them. Companion: `2026-06-09-drone-bf-osd-realstate-design.md` (the OSD).

Scope: **#2** drone (C++), **#3** + **#4** GS (Python). The drone fix and the
OSD share `daemon.cpp`/`beamforming.cpp`, so they are implemented together in a
drone plan; #3 + #4 are a separate GS plan.

## #2 — Drone: `reconcileBeamforming()` missing from the hot apply path

### Problem
`Daemon::reconcileBeamforming()` (`drone/src/daemon.cpp`) is called in only two
places: `start()` (boot, ~line 65) and the **full-rebuild** branch of `apply()`
(~line 368). A beamforming config change goes through the **hot** apply path
(`reallyRestart && !needsRebuild`), which handles txpower/mtu/fec/`stbc`/channel
but **never calls `reconcileBeamforming()`**. So `apply()` commits
`beamforming.enabled=true` (and reports `"beamforming"` in `restarted`, which is
cosmetic — derived from `bfChanged`) but the controller is **never armed**:
registers stay zero, `/status` stays `disabled`. This is why BF only armed after
a full `fpvd` restart. `diff.cpp:54` already documents that beamforming "is
reconciled separately" — the separate reconcile is simply missing from the hot
path.

### Fix
Call `reconcileBeamforming()` in the hot apply path, placed **after** the
`videoRadiotap`/`setRadio` block (so any `stbc`/radio retune has settled) and
**before** the `nicChannel` deferred-return block (so it runs whether or not the
apply also changes channel). It runs whenever `bfChanged || subs.radio` (see the
re-arm nuance below); calling it is cheap and idempotent otherwise.

### Re-arm nuance (radio reset)
A `stbc`/radio retune (`WfbControlClient::setRadio`) can reset the
`REG_TXBF_*`/`bf_monitor` registers. The drone `BeamformingController::reconcile`
has an idempotency guard (skip the proc write when already running with identical
params + `Active`). After a radio reset that guard would wrongly skip the
re-write, leaving BF disarmed while the controller still believes it is `Active`.

So: when the radio was touched in this apply (`subs.radio`) and BF is enabled,
the reconcile must **force a re-write** of `bf_monitor_conf` even if params are
unchanged. Implementation: `reconcileBeamforming(bool force)` — `force=true` when
`subs.radio`; the controller's reconcile rewrites the conf node when `force` is
set, bypassing the idempotency skip. Boot and full-rebuild paths pass
`force=true` as well (the card was just brought up).

### Cosmetic cleanup (optional, low priority)
A BF-only change currently reports `"radio"` in `restarted` because
`diff.cpp:10` sets `d.radio` for any `link` delta. Not harmful; out of scope
unless trivial during implementation.

### Verified (2026-06-09): `setRadio` does NOT reset the TXBF registers
A final review raised a concern that under DL-on, a `stbc`/`ldpc` retune is
applied asynchronously by the DL control thread (`dl_.setConfig` → `setRadio`)
*after* the synchronous `reconcileBeamforming(force=true)` — an inverted order
that would re-arm BF before the reset. **Hardware verification refuted the
premise:** the `bf_monitor` registers (`ENABLE_NDPA=1`, Remote MAC set) survived
an extended active-DL session (~7 min, `soundingCount` 4421, MCS adapting, so
`setRadio` was called many times). `setRadio` is a wfb_tx radiotap-header change,
not a card re-init, so it does not touch `REG_TXBF_*`. The `force` flag is
therefore only genuinely needed for the full-rebuild card bring-up (already
`force=true`); the DL-on ordering is a non-issue and needs no fix.

## #2b — Sounding-loop resilience (real bug found during verification)

The same verification found the live drone's BF loop **dead**: `state=error`,
`reason="bf_monitor_trig write failed"`, `soundingCount` + `rfinfo` token frozen.
The loop's error path does `status_.state = Error; return;` — so a **single
transient `bf_monitor_trig` write failure kills the sounding loop permanently**
(until a reconcile), even though the registers stay armed. BF silently stops.

### Fix
The loop must tolerate transient write failures instead of exiting:
- On a `bf_monitor_trig` write failure: set `state=Error` with the reason but
  **do not `return`** — sleep and retry on the next tick.
- On a subsequent successful sounding: if the state was `Error`, recover it to
  `Active` and clear the reason (self-heal).
- Only write `bf_monitor_en "1"` after a *successful* sounding write (don't
  enable off a failed trig).
The loop still exits only on `stopFlag_` (reconcile/stop). A persistently broken
node leaves the loop in `Error` but alive and retrying — never a silent death.

## #3 — GS: auto-manage STBC + surface drone validation errors

The drone schema (`drone/src/config/validate.cpp:64-77`) enforces:
beamforming requires `link.stbc=false`, a valid `remoteMac`, `ackTimeout` in
33..255, `intervalMs>=1`. STBC and TX beamforming are mutually exclusive.

### 3a. Auto-manage STBC
The coordinator (`gs/fpvdgs/link.py`) already builds a `push` dict for the drone.
When beamforming changes, bundle the STBC requirement:
- enabling (`bf_enabled`): `push["stbc"] = False` alongside the beamforming block.
- disabling: `push["stbc"] = True` (symmetric restore to the drone's default).

So a single `/link/apply` flips STBC and beamforming together — the operator
never sees `requires link.stbc=false`. `stbc` is a drone-only TX param (not in
the GS config), so it is pushed, never stored on the GS.

### 3b. Surface validation errors (not "unreachable")
Today `DroneClient._ok_json` raises `DroneUnreachable` on **any** `code >= 400`,
so a drone **validation rejection** (400) is indistinguishable from a genuine
connectivity failure — the coordinator reports `droneReachable=false`, which is
misleading.

Fix: distinguish a 4xx **rejection** from a connection failure.
- Add `DroneRejected(Exception)` to `drone_client.py`, carrying the status code
  and parsed error body. `_ok_json` raises `DroneRejected` for 400–499 and
  `DroneUnreachable` only for connection/transport errors (and 5xx).
- In `apply_link`, catch `DroneRejected` separately: do **not** flip
  `droneReachable=false`; instead surface it in the result, e.g.
  `res["droneError"] = {"code": ..., "message": ...}` and leave
  `droneApplied=false`. The GS still applies locally (best-effort semantics
  unchanged). `DroneUnreachable` keeps its current meaning (flaky link).

This makes a future drone-side rejection (e.g. a bad `remoteMac`) visible as a
real error instead of a phantom "drone unreachable."

## #4 — GS: DroneClient timeout 4s → 10s

The drone `/status` is ~3 ms on its LAN but the FPV management link intermittently
stalls up to ~8 s; the coordinator's `get_status` raced the 4 s timeout. Raise
the default `DroneClient` timeout from `4.0` to `10.0` (`drone_client.py:13`,
also reflected at the `supervisor.py:85` construction). A single call now rides
through an 8 s stall. Accepted trade-off: a genuinely-down drone makes drone
calls (e.g. the `/air` proxy, status polling) hang up to 10 s.

## Error handling summary

| Condition | Behavior |
|---|---|
| Drone 4xx validation rejection | `DroneRejected` → `res.droneError`, `droneApplied=false`, GS still applies |
| Drone connection/transport failure or 5xx | `DroneUnreachable` → `droneReachable=false` (unchanged) |
| 8 s link stall | absorbed by the 10 s timeout |
| `stbc`/radio retune in same apply (drone) | forced BF re-arm (`force=true`) |

## Testing

**Drone** (`drone/tests/integration/test_beamforming.cpp` / daemon tests; run
`./build/fpvd_tests` from `drone/`, NOT ctest):
- Hot-path apply with `beamforming.enabled=true` arms the controller (registers
  written) without a full rebuild.
- A `stbc` change with BF enabled forces a re-arm (conf re-written) even though
  BF params are unchanged.
- Boot/full-rebuild paths still arm (force=true).

**GS** (`gs/tests/unit/`):
- `test_link.py`: enabling BF puts `stbc=false` in the drone push; disabling puts
  `stbc=true`; a `DroneRejected` from the drone surfaces as `res["droneError"]`
  with `droneReachable` NOT set false, and the GS still commits/applies locally.
- `test_drone_client.py`: default timeout is 10.0; a 400 raises `DroneRejected`
  (with code+body), a connection error raises `DroneUnreachable`, a 5xx raises
  `DroneUnreachable`.

## Out of scope
- The OSD indicator (companion spec).
- Boot-arming the GS beamformee (separate follow-up).
- Coordinator-level retries (chose a static 10 s timeout instead).
- Improving the FPV management-link quality itself.
