# GS dynamic-link fold-in (in-process, hot config reload)

**Date:** 2026-06-02
**Status:** Design — approved, pending spec review
**Branch:** `gs-dynamic-link-fold-in`

## Summary

Fold the **ground-station side** of the standalone `dynamic-link` project
(`../dynamic-link/gs`) into `fpvd`'s GS daemon (`gs/fpvdgs`), the same way the
drone side was already folded into the C++ daemon (commit `8701ee0`). The GS
dynamic-link is the *brain*: it reads wfb-ng link stats at 10 Hz, runs the
adaptive control policy, and emits decision packets to the drone, which applies
them via its already-folded in-process controller.

After this work, configuring/enabling/tuning the GS dynamic-link happens through
fpvd's existing config API (`PATCH /config` + `POST /apply`) and is applied **at
runtime with no wfb restart** — matching how every other GS config change is
moving (live `iw` retunes in `LinkCoordinator`, in-process hot reload on the
drone).

## Context: who computes, who applies

```
  GS (fpvdgs)                                     Drone (fpvd C++)
  ───────────                                     ────────────────
  wfb-ng stats :8103  ──►  DynamicLinkController
   (10 Hz RxEvents)         (policy: dual-gate MCS,
                             trailing FEC/depth/bitrate,
                             latency predictor)
                                   │
                                   │  DLK1 v2 decision (31 B)   UDP →  :9999  ──►  DynamicLinkController
                                   │  HELLO-ACK (DLHA)          UDP →  :9999       (applies: wfb_tx, iw txpower,
                                   │                                                waybeam encoder)
                                   ◄── HELLO (DLHE) keepalive   UDP ◄  10.5.0.1:5801
```

The drone side is **done** and unchanged by this work. The GS side currently
runs as a separate process (`dynamic_link.service`) configured by a ~100-key
`gs.yaml`. We are migrating that brain in-process into `fpvdgs`.

## Decisions (locked during brainstorming)

1. **Run model:** in-process **asyncio loop on a dedicated daemon thread** inside
   `fpvdgs`. The policy core (~2000 lines, flight-critical, well-tested) is lifted
   verbatim; only its lifecycle and config source change. `asyncio` is stdlib, so
   `fpvdgs` stays pure-stdlib; the supervisor proper stays thread-based.
2. **Config surface:** **curated keys + opaque `tuning` passthrough**. The
   operationally meaningful knobs are validated in fpvd's schema; the ~100
   fine-tuning knobs stay as code defaults, overridable via `dynamicLink.tuning`.
3. **Scope:** **core adaptive control only.** Phase-3 forensics
   (timesync, latency sink, video tap, MAVLink status, flight log) are out of
   scope — keeps fpvdgs free of PyYAML and the `wfb_ng` MAVLink dependency.
4. **GS/drone arming:** **independent + status visibility.** GS config arms only
   the GS brain; the drone is armed separately (its own config, reachable via
   fpvd's `/air` proxy). `/status` reports drone reachability, the drone's
   dynamic-link state, and the HELLO-handshake state so mismatches are visible.

## Architecture

### New package: `gs/fpvdgs/dynlink/`

Lifted **as-is** from `dynamic-link/gs/dynamic_link/` (core control):

| Module | Role |
| --- | --- |
| `policy.py` | dual-gate MCS selector, trailing FEC/depth loop, composite Policy |
| `signals.py` | signal aggregator, EWMA smoothing, window metrics |
| `stats_client.py` | async TCP client for wfb-ng JSON stats (`:8103`) |
| `dynamic_fec.py` | `(k, n)` computation, `NEscalator`, `EmitGate` |
| `bitrate.py` | wire-target → encoder bitrate |
| `predictor.py` | latency-budget defensive gate |
| `profile.py` | radio profile loader (MCS-vs-SNR floor table) |
| `wire.py` | DLK1 v2 encoder (31-byte decision packets) |
| `return_link.py` | non-blocking UDP sender to the drone |
| `drone_config.py` | P4a HELLO handshake state (mtu/fps/generationId), gates emit |
| `decision.py` | Decision dataclass |

**Dropped** (Phase-3 forensics): `timesync.py`, `tunnel_listener.py` (PONG only),
`latency_sink.py`, `video_tap.py`, `mavlink_status.py`, `flight_log.py`,
`debug_config.py`, `observed.py`, `sinks.py`, `tools/`.

> The HELLO **receive** path (drone → GS DLHE, GS → drone DLHA) is *core* and
> comes along with `drone_config.py`; only the timesync PONG listener and the
> forensic sinks are dropped. During implementation, confirm the HELLO receive
> currently living in `tunnel_listener.py`/`service.py` is separated from the
> dropped PONG/timesync code.

### New wrapper: `DynamicLinkController` (`gs/fpvdgs/dynlink/controller.py`)

The GS analog of the drone's in-process controller. Owns one daemon thread that
runs `asyncio.run(self._run())`. `_run()` builds the stats client, return-link
sender, policy, and HELLO handshake from a config **snapshot**, then ticks at the
stats cadence until stopped. Thread-safe surface for the HTTP server thread:

- `start()` — spawn the thread (only when `enabled`). No-op if already running.
- `stop()` — `loop.call_soon_threadsafe` an asyncio stop event; join the thread.
  Idempotent.
- `set_config(snapshot)` — push a new snapshot. Tunable changes (gate weights,
  fec thresholds, maxMcs, txpower range, bandwidth) are swapped **live** on the
  next tick. Only an **endpoint** change (stats host/port, drone addr/port, GS
  HELLO-listen port) tears down and rebuilds I/O **inside the loop** — the wfb
  runner is never touched.
- `status()` — `{running, statsConnected, decision{mcs,k,n,depth,txpowerDbm,
  bitrateKbps}, lastEmitMs, emitSeq, reason, hello}` (a thread-safe copy of the
  last published state).

This wrapper is the only surface the rest of fpvdgs sees — mirroring the drone's
`DlStatus` published-by-loop / read-by-HTTP-thread split.

### Lifecycle wiring (`gs/fpvdgs/supervisor.py`)

`build_app()` constructs the controller next to the runner and hands it to `App`:

- `App.start()` → `runner.start()` then, if `dynamicLink.enabled`,
  `controller.start()`.
- `App.shutdown()` → `controller.stop()` then `runner.shutdown()` (stop the brain
  before tearing down the stats source it consumes).

## Config schema

Added to `gs/etc/defaults.json` and validated in `gs/fpvdgs/schema.py`:

```json
"dynamicLink": {
  "enabled": false,
  "maxMcs": 5,
  "bandwidth": 20,
  "txpower": { "min": 18, "max": 28 },
  "radioProfile": "m8812eu2",
  "droneAddr": null,
  "dronePort": 9999,
  "tuning": {}
}
```

- **Curated keys** are validated by `schema.validate_effective` (ranges: `maxMcs`
  0–7, `bandwidth` ∈ {20, 40}, `txpower.min ≤ txpower.max`, `radioProfile` is a
  known profile, `dronePort` 1–65535).
- **`droneAddr: null`** → default to the host parsed from `drone.endpoint`
  (e.g. `10.5.0.10`), with `dronePort` 9999.
- **`tuning`** is an opaque object, deep-merged over the policy's code defaults.
  Not strictly validated (advanced knob; mistakes surface as controller-build
  errors reported through `/apply`).

The controller's internal policy config is built as
**code defaults ⊕ curated keys ⊕ `tuning`**. `dynamic_link.service`'s
`_build_policy_config()` is refactored to consume this fpvd block instead of
parsing `gs.yaml`.

## Apply / hot-reload flow

`dynamicLink` changes ride the existing `PATCH /config` + `POST /apply`. Today
fpvd's GS `/apply` bounces the runner for *every* change; it is refined to **diff
subsystems** first:

```
POST /apply  (after validate + commit pending→effective):
  dl_old, dl_new = old.dynamicLink, new.dynamicLink
  non_dl_changed = (old minus dynamicLink) != (new minus dynamicLink)

  if non_dl_changed:   <existing runner path: render cfg, bounce runner, readiness>
  # dynamic-link routing (never touches the runner):
  if  !dl_old.enabled and  dl_new.enabled:  controller.start()
  elif dl_old.enabled and !dl_new.enabled:  controller.stop()
  elif dl_old.enabled and  dl_new.enabled and dl_changed:
                                            controller.set_config(snapshot(new))
```

**Invariant: no `dynamicLink` change ever bounces the wfb runner.** The
controller is a stats *client* of the runner's `:8103`; arming/disarming/tuning
the brain is independent of the radio process.

Routing lives in a small helper alongside the apply handler (symmetric with
`LinkCoordinator`'s `/link` handling). `enabled` true→false stops emitting; the
**drone's own watchdog** (`healthTimeoutMs`) then falls back to safe defaults —
no explicit teardown packet needed.

### Stats cadence coupling (the one runner-side dependency)

The policy expects **10 Hz** stats, i.e. wfb-ng `log_interval = 100`. To keep
enable/disable fully hot, `gs/fpvdgs/render.py` emits `log_interval = 100`
**unconditionally** (not gated on `dynamicLink.enabled`). That way arming the
brain never requires a cfg change or runner bounce. (Cost: marginally more stats
logging when dynamic-link is off — acceptable.)

## Status & coordination

`/status` gains a `dynamicLink` section, assembled in `supervisor.status_fn`:

```json
"dynamicLink": {
  "enabled": true,
  "running": true,
  "statsConnected": true,
  "decision": { "mcs": 4, "k": 8, "n": 12, "depth": 1,
                "txpowerDbm": 22, "bitrateKbps": 9000 },
  "lastEmitMs": 1234, "emitSeq": 4567, "reason": "snr_margin",
  "drone": { "reachable": true, "dynamicLinkActive": true, "hello": "acked" }
}
```

- `running / statsConnected / decision / lastEmit / reason` ← `controller.status()`.
- `drone.reachable` ← `DroneClient.healthz()` (already present).
- `drone.dynamicLinkActive` ← `/air/status` proxy to the drone's status.
- `drone.hello` ← GS `drone_config` handshake state (`none | waiting | acked`).

This delivers option-1 coordination: independent arming, full visibility of a
GS-armed/drone-not mismatch, no cross-link enable coupling.

## Compatibility fixes (must-do)

Concrete facts from the drone build
(`drone/src/dynlink/runtime_config.hpp`, `hello.hpp`):

- **Decision + HELLO-ACK target → drone `:9999`** (not the dynamic-link sample's
  `5800`; `wfb_tun` owns `5800`). The lifted `return_link` and the DLHA sender
  point at `droneAddr:dronePort` (default drone host : 9999).
- **HELLO from drone arrives at GS `10.5.0.1:5801`** (`gsTunnelAddr:gsTunnelPort`
  in the drone build). The GS HELLO listener binds this; surface it as a
  (rarely-changed) endpoint in the controller config.
- **Wire-contract test:** keep a test asserting the GS `DLK1` v2 encoder output
  matches the drone decoder's expectation (magic `0x444C4B31`, version 2, 31
  bytes, CRC32). The drone's `dynlink/wire.cpp` is the authority.
- **Radio profiles → JSON** (or embedded Python dicts) under
  `gs/fpvdgs/dynlink/profiles/`, replacing `conf/radios/*.yaml`, so fpvdgs needs
  no PyYAML. Selected by `dynamicLink.radioProfile`.
- **Stats reconnect:** confirm `stats_client` reconnects to `:8103` across a
  runner bounce (a real link change can restart the runner under the brain).
- **Stats-schema parity:** confirm fpvd's runner (wfb-ng) emits the per-antenna
  RSSI/SNR and FEC counters the policy reads. Both projects build on the same
  wfb-ng (`../wfb-ng`); a quick confirm de-risks it.

## Testing

Port the core unit tests from `dynamic-link/tests`:

- `test_policy_leading.py` — dual-gate selector.
- `test_policy_dynamic_fec_e2e.py` — `(k, n)` logic.
- `test_wire_contract.py` — GS encoder hex vs the drone decoder (now fpvd's).
- Relevant slices of `test_phase2_e2e.py` — policy → wire → decision.

New tests in `gs/tests`:

- **Controller lifecycle:** `start` → running; `stop` → joined; `set_config`
  swaps tunables live; a tunable change does **not** restart I/O.
- **Apply routing:** a `dynamicLink`-only change leaves the runner untouched
  (assert the runner is not bounced); a mixed change bounces the runner **and**
  reconfigures the controller; enable/disable transitions call `start`/`stop`.
- **Status shape:** `/status.dynamicLink` is populated and merges drone-side
  fields.

## Out of scope

- Phase-3 forensics (timesync, latency sink, video tap, MAVLink status, flight
  log) and the `tools/` post-flight suite.
- One-switch GS+drone arming (coordination option 2) — layers cleanly on top of
  this later.
- Any drone-side change beyond what is already folded in.
- Retiring the standalone `dynamic-link` repo — a follow-up once in-flight parity
  is confirmed.

## Open items to confirm during implementation

1. Exact module boundary of the HELLO-receive path vs the dropped timesync PONG
   listener in the current `tunnel_listener.py`/`service.py`.
2. GS HELLO-listen bind (`10.5.0.1:5801`) — confirm against the live drone build
   and decide whether it is operator-configurable or pinned.
3. wfb-ng stats JSON schema parity between `fpvd` and `dynamic-link`.
