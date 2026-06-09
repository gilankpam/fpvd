# GS Beamformee Boot Re-Arm — Design

**Date:** 2026-06-09
**Status:** Approved design
**Context:** The GS `BeamformingController` only arms on a `/link/apply`. After a
GS restart/reboot (which also reloads the `8812eu` driver, clearing the TXBF
registers), `link.beamforming.enabled` stays `true` in config but the beamformee
is never re-armed — BF silently stays off until an operator re-applies. Observed
live (GS uptime 6.5 min, config enabled, card `ENABLE_NDPA:0`). This adds a small
background re-arm loop so the GS arms itself at boot once the drone is reachable.

## Component: `BeamformingArmer` (`gs/fpvdgs/beamforming_armer.py`)

One job: keep the GS beamformee armed to match config. Injected deps:
- `beamforming` — the `BeamformingController`
- `drone` — the `DroneClient`
- `wlans_resolver(cfg) -> list[str]`
- `config_provider() -> dict` (effective cfg)
- `interval` (default 5.0s)

A background thread (started by `App.start()`, stopped by `App.shutdown()`) runs
`_tick()` every `interval`:

```
cfg = config_provider()
bf  = cfg["link"]["beamforming"]            # tolerate missing
if bf.enabled and controller.status()["state"] != "active":
    primary = (wlans_resolver(cfg) or [None])[0]
    if primary and controller.supported(primary) and drone.healthz():
        try:
            mac = drone.get_status().get("beamforming", {}).get("localMac", "")
        except (DroneUnreachable, DroneRejected):
            mac = ""
        if mac:
            controller.reconcile(True, primary, mac)   # arm
# disabled OR already active OR drone down -> no-op, retry next tick
```

`_tick()` is a pure method (no thread) so it is unit-testable; the thread just
calls it on a loop with a stop event.

## Behavior / error handling
- **Boot/restart:** controller is fresh (`state != "active"`) → arms as soon as
  the drone is reachable, retrying every 5s until then.
- **Idempotent with `/link/apply`:** if an apply already armed it, `_tick` sees
  `active` → no-op. No contention (both call the same idempotent `reconcile`).
- **Never crashes the app:** all drone calls wrapped; `DroneUnreachable`/
  `DroneRejected` caught → retry next tick.
- **Disable:** when config flips to disabled, `_tick` is a no-op (it does not
  disarm — `/link/apply` owns disable, which pushes stbc restore etc.).

## Wiring (`gs/fpvdgs/supervisor.py`)
`build_app` constructs the armer from the already-built `beamforming`, `drone`,
and `resolve_wlans` + a `config_provider = lambda: store.effective()`. `App`
gains `armer`; `App.start()` calls `armer.start()`, `App.shutdown()` calls
`armer.stop()` (before the runner stops, mirroring the other children).

## Testing (`gs/tests/unit/test_beamforming_armer.py`)
Unit-test `_tick()` against fakes:
- enabled + controller not active + drone up + MAC present → `reconcile(True, primary, mac)` called.
- already `active` → no `reconcile`.
- disabled in config → no `reconcile`.
- drone `healthz()` False → no `reconcile`.
- `get_status()` raises `DroneUnreachable` → no crash, no `reconcile`.
- unsupported primary → no `reconcile`.

## Out of scope
- Detecting an external TXBF-register reset while the controller still reports
  `active` (stale status; harder problem).
- Disarm-on-config-disable from the armer (owned by `/link/apply`).
