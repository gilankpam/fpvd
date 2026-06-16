# GS Drone Connection Events — Design

**Date:** 2026-06-17
**Branch:** `feat/drone-connection-events` (off `feat/learned-prior-knee-model`)
**Status:** Approved design, pre-implementation

## 1. Motivation

"Drone connected / disconnected" is not a first-class concept on the GS today. The
only thing resembling it is buried inside `Policy.tick()`
(`gs/fpvdgs/dynlink/policy.py:433-442`): it tracks `link_starved_w` from the
**video** stats feed and, when the link returns healthy after a `>15 s` gap, calls
`flightlog.roll()`. The `FlightLog` is owned by the `Policy`, so this connect/disconnect
proxy only exists while `dynamicLink.enabled`, is derived from a video-specific
signal, and cannot be observed or reused by any other subsystem.

We want a **global, subscribable connection event** on the GS: a single authoritative
source of "the drone is connected / disconnected" that any subsystem can react to.
The flight-log roll/sync is the first consumer; this design also lifts two
correctness/durability behaviors onto the same signal.

## 2. Decisions

| Decision | Choice | Rationale |
|---|---|---|
| **Source of truth** | The wfb **tunnel** stream on the `:8103` stats feed, with the drone HTTP API on top of it | The tunnel is the bidirectional management channel — independent of video and `dynamicLink` — and is the path the `/air/*` HTTP API actually traverses, so "tunnel up + HTTP answering" genuinely means "drone reachable." |
| **Connected gate** | Tunnel-gated **+ HTTP-confirmed** | Tunnel rx traffic *arms* the monitor; `connected` only fires once `GET /status` (via `DroneClient`) succeeds, and the event payload carries drone identity/version. Disconnect fires on tunnel loss **or** sustained HTTP failure. |
| **Placement** | A new **always-on top-level `App` subsystem**, not under dynlink | Makes the event truly global — it runs regardless of `dynamicLink.enabled`. |
| **Delivery** | A generic thread-safe **`EventBus`** with synchronous, exception-isolated dispatch; subscribers marshal onto their own thread | Fits the established thread-per-subsystem stdlib supervisor; no single-asyncio rearchitecture. |
| **Subscribers wired now** | flight-log roll/sync, **selector reset on reconnect**, **learned-prior flush on disconnect** | The latter two close a real correctness gap and a durability gap, and live in the same dynlink subscriber as the flight log. |

Rejected alternatives: extending the dynlink controller to emit the events (couples the
"global" event to `dynamicLink.enabled`); unifying the whole supervisor onto one asyncio
loop with an async-native bus (large rearchitecture, YAGNI).

## 3. Architecture

### New files (both top-level `fpvdgs/` modules — not subpackages)

- `gs/fpvdgs/events.py` — generic thread-safe `EventBus`, event-name constants
  (`DRONE_CONNECTED`, `DRONE_DISCONNECTED`), and a `ConnectionEvent` payload dataclass.
- `gs/fpvdgs/connection_monitor.py` — `ConnectionMonitor` + `ConnectionMonitorConfig`.
  Owns a daemon thread + asyncio loop (mirrors `DynamicLinkController`). Watches the
  tunnel stream via a second `:8103` `StatsClient`, confirms via a short-timeout
  `DroneClient`, runs the state machine, publishes to the bus.

### Modified files

- `gs/fpvdgs/supervisor.py` — build/own/start/stop the bus + monitor; add a `connection`
  block to `status_fn`.
- `gs/fpvdgs/dynlink/controller.py` — take the bus; subscribe on loop-up, unsubscribe on
  stop; drive flight-log roll/sync, selector reset, prior flush (all marshaled onto its
  own loop).
- `gs/fpvdgs/dynlink/policy.py` — retire the inline gap-roll; add `reset_for_new_session()`.
- `gs/fpvdgs/dynlink/flightlog.py` — add `begin_flight()` + `sync()`; drop the now-unused `flight_gap_s` field.
- `gs/fpvdgs/config_defaults.py`, `gs/fpvdgs/schema.py` — new `connectionMonitor` block.

**No `deploy/gs/deploy.sh` change:** line 34 already ships all top-level `fpvdgs/*.py`
via a glob, so the two new modules deploy automatically. (The CLAUDE.md deploy gotcha is
specifically about new *subpackages*, which need their own `mkdir` + `scp` line.)

### Data flow

```
wfb :8103 ──(tunnel rx records)──► ConnectionMonitor
                                      │  StatsClient on_event: stamp last_tunnel_rx (monotonic)
                                      ▼
                              eval task (asyncio, every evalIntervalS)
                                ├─ tunnel fresh? ──► DroneClient.get_status()  [run_in_executor, short timeout]
                                │                      └─ ok ─► CONNECTED ─► publish DRONE_CONNECTED(snapshot)
                                │                              (then healthz() heartbeat every httpPollS)
                                └─ tunnel stale OR healthz failing ─► publish DRONE_DISCONNECTED(payload)
                                      │
                                      ▼  EventBus.publish  (synchronous, in monitor thread)
                              ┌───────┴────────┐
                       dynlink controller   (future: BF re-arm, probe idle, …)
                       (call_soon_threadsafe onto its own loop)
                              │
                       FlightLog.begin_flight()/sync()  +  Policy.reset_for_new_session()  +  learned_prior.flush()
```

### Ownership / wiring

The bus is created in `build_app`, owned by `App`, and passed to both the monitor
(publisher) and the dynlink controller (subscriber). The monitor is started
**unconditionally** in `App.start()` and stopped in `App.shutdown()`. `build_app`
constructs the monitor's `DroneClient` with the **short** `httpTimeoutS` (the existing
`drone` client keeps its 10 s timeout for the `/air` proxy).

## 4. Connection state machine

Three states (ARMED is transient):

| State | Meaning | HTTP polling |
|---|---|---|
| **DISCONNECTED** | no fresh tunnel rx | none (passive — waits for tunnel) |
| **ARMED** | tunnel rx seen, app not yet confirmed | actively poll `get_status()` |
| **CONNECTED** | tunnel fresh **and** drone confirmed | heartbeat `healthz()` every `httpPollS` |

`get_status()` confirms the connect (it both proves the app is up and supplies the
payload); the cheaper boolean `healthz()` is the ongoing heartbeat. Transitions are
evaluated every `evalIntervalS`. Let `tunnel_fresh = (now - last_tunnel_rx) < tunnelStaleS`:

- **DISCONNECTED → ARMED:** `tunnel_fresh`.
- **ARMED → CONNECTED:** `get_status()` succeeds → **publish `DRONE_CONNECTED`** with that snapshot.
- **ARMED → DISCONNECTED:** `not tunnel_fresh` before confirmation (no event — never announced).
- **CONNECTED → DISCONNECTED:** `not tunnel_fresh` **OR** `httpFailCount` consecutive
  heartbeat (`healthz()`) failures → **publish `DRONE_DISCONNECTED`** (with the
  distinguishing `reason`).

### The idle-tunnel invariant

The tunnel only shows *rx* traffic when the drone sends IP packets back; when otherwise
idle, the main thing generating that return traffic is *our own* healthz responses. So
while CONNECTED the heartbeat keeps the tunnel "fresh" itself. This gives a hard
invariant: **`tunnelStaleS` must exceed `httpPollS`** (defaults 4.0 s > 1.5 s) or a
quiet-but-healthy link would false-disconnect. The schema enforces this.

To keep disconnect detection fast and prevent a slow call from stalling the eval loop,
the monitor's `DroneClient` uses a short timeout (`httpTimeoutS` ≈ 1.5 s) and every
`healthz()`/`get_status()` runs in `loop.run_in_executor`. A powered-off drone is then
detected within roughly `max(tunnelStaleS, httpPollS·httpFailCount)` ≈ 3–4 s.

### Event payloads (`ConnectionEvent`)

- `DRONE_CONNECTED`: `{state: "connected", at_mono, drone: <get_status snapshot: version, radioProfile, mcs, channel, …>}`
- `DRONE_DISCONNECTED`: `{state: "disconnected", at_mono, reason: "tunnel_lost" | "http_failed", last_seen_mono}`

Monotonic timestamps drive all logic (the GS wall clock is unreliable — see the flight-log
notes). Wall time may be included for display only, clearly labeled.

## 5. `EventBus` contract

```python
class EventBus:
    def subscribe(self, event: str, cb: Callable[[dict], None]) -> None
    def unsubscribe(self, event: str, cb) -> None
    def publish(self, event: str, payload: dict | None = None) -> None
    def state(self, key: str, default=None)   # latest cached payload, e.g. bus.state("drone")
```

Delivery / threading contract:

- `publish()` invokes each subscriber **synchronously, in the publisher's (monitor's)
  thread**, in subscription order.
- A callback that raises is **caught and logged**; it never breaks sibling callbacks or
  the publisher.
- **Callbacks must be quick, non-blocking, and thread-safe.** Real work or touching
  another thread's state → the subscriber marshals (`loop.call_soon_threadsafe`, or
  enqueue). The bus spawns no threads and holds one `RLock`.
- The bus **caches the latest payload per event key** (e.g. `"drone"`). A late-starting
  subsystem reads `bus.state("drone")` to learn current status immediately. **No
  auto-replay on subscribe** — avoids spurious double-fires; subscribers opt in by reading
  state explicitly.

Generic string-keyed events so future signals (`video.lost`, `config.applied`, …) reuse
it with no new machinery.

## 6. Subscribers

All three live in `DynamicLinkController`, which subscribes on loop-up and unsubscribes on
`stop()`. Each bus callback fires on the **monitor's** thread and immediately marshals onto
the dynlink loop via `call_soon_threadsafe`, so the in-loop handlers touch
`Policy`/`FlightLog`/`learned_prior` on the **same thread** as the per-tick writes — no
new locks; single-threaded access is preserved.

1. **Flight-log roll/sync** (baseline).
   - `on connected` → `flightlog.begin_flight()` — rolls to a new flight file if the current
     one has records, keeps an already-open empty file, or (re)opens one. (Avoids an empty
     file on the first connect right after start.)
   - `on disconnected` → `flightlog.sync()` (flush + fsync) — makes the flight **durable**
     the moment the link drops, without closing it (no write-after-close reopen; a tunnel-only
     blip while video still flows keeps appending to the same flight).
   - **Retire** `_last_healthy_mono` and the `flight_gap_s` roll block in `policy.py`. The
     flight boundary is now the connection event (tunnel + HTTP), a better signal than
     video-starvation.

2. **Selector reset on (re)connect.** New `Policy.reset_for_new_session()` resets the
   **volatile selector state** to boot — `leading.state.current_mcs` → boot MCS,
   `_cold_started` → `False` (so the learned-prior warm-start re-runs), promote/demote
   counters → 0, `_rssi_window` cleared, `_ticks_at_mcs` → 0, `_last_ingest_mcs` → `None`.
   It **does not** touch `learned_prior` knees (persistent cross-session knowledge).
   Prevents the GS from resuming a stale climbed-up MCS at a freshly-booted drone (which
   has fallen back to `dynamicLink.safe`). Invoked on the `connected` edge.

3. **Learned-prior flush on disconnect.** `learned_prior.flush()` on the `disconnected`
   edge — same durability trigger as the flight-log `sync()`. Hardens the per-card knee
   model against the GS reboot-on-video-loss, instead of only flushing on controller
   teardown.

### Future subscribers (the bus makes these one-liners; not in this spec)

- **Beamforming re-arm on connect** — `armer.tick()` on the connect edge for responsive BF
  gain at flight start.
- **Probe idle on disconnect** — stop the probe's TX/RX airtime + CPU when no drone is
  connected; resume on connect.

Explicitly **out of scope**: pixelpilot restart/OSD on connect (touches the video pipeline;
the external reboot-on-video-loss watchdog already handles loss) and the stateless IDR relay.

## 7. Observability

`ConnectionMonitor.status()` (consistent with `dynlink.status()` / `probe_ctrl.status()`)
feeds a new `connection` block in `GET /status`:

```json
{"state": "connected|armed|disconnected", "sinceMs": 12345, "reason": "", "drone": {"version": "...", "radioProfile": "...", "mcs": 4}}
```

Each edge logs at INFO under a `fpvdgs.connection` logger
(`drone connected: version=… mcs=…` / `drone disconnected: reason=tunnel_lost`).

## 8. Configuration

New top-level `connectionMonitor` block (code-default, tolerant loader, schema-validated —
the established daemon-wide pattern; read inline in `build_app` like `idrForward`):

```json
"connectionMonitor": {
  "enabled": true,
  "tunnelStaleS": 4.0,
  "httpPollS": 1.5,
  "httpTimeoutS": 1.5,
  "httpFailCount": 2,
  "evalIntervalS": 0.5
}
```

- `flightlog.flight_gap_s` is **removed** (superseded). It is not read in `config_build`
  (only `enabled` is), so removal is a clean two-line change (the dataclass field + the
  `policy.tick` usage). The tolerant loader strips the stale key from any on-disk config —
  no boot-brick.
- Schema enforces **`tunnelStaleS > httpPollS`** (warn/clamp otherwise) and that
  `enabled`/counts/intervals are well-typed and positive.

## 9. Testing (TDD throughout)

- `gs/tests/unit/test_events.py` — subscribe/publish/unsubscribe, exception isolation,
  state cache, dispatch order.
- `gs/tests/unit/test_connection_monitor.py` — inject a fake `stats_client_factory`
  (synthetic tunnel rx events), an injectable clock (`time_fn`), and a scriptable fake
  `DroneClient` (programmable `healthz`/`get_status`). Cover: ARMED→CONNECTED (event +
  payload from `get_status`); tunnel-lost and http-fail disconnects (distinct `reason`);
  ARMED→DISCONNECTED with **no** event; **idle-tunnel kept fresh by the heartbeat** (no
  false disconnect when `httpPollS < tunnelStaleS`); flap suppression; `enabled: false` →
  no thread. No real sockets — mirrors the dynlink `StatsClient`/`ReplayClient` injection
  pattern.
- Extend the dynlink controller tests — connect edge ⇒ begin_flight + `reset_for_new_session`;
  disconnect edge ⇒ sync + `learned_prior.flush`; assert the inline `flight_gap_s` path is
  gone. Driven through a real bus + synthetic publishes.
- The **full GS suite stays green** (config_build / schema round-trip for the new block —
  partial refactors go red on the import/config coupling).

## 10. Risks & verification

1. **Two `:8103` clients.** The monitor opens a second subscriber on wfb-ng's stats server
   alongside dynlink. Believed supported (the web UI + OSD already co-subscribe), but
   **bench-verify early**. Fallback if single-client: one shared fan-out reader feeding both
   the monitor and dynlink.
2. **Idle-tunnel false disconnect.** Mitigated by the `tunnelStaleS > httpPollS` invariant,
   the short HTTP timeout, and `run_in_executor`. Validate on the bench with a quiet link.
3. **Threading.** All `FlightLog`/`Policy`/`learned_prior` access stays on the dynlink loop
   via `call_soon_threadsafe`; bus callbacks are quick + marshaling; the monitor's HTTP runs
   in an executor so it cannot stall the eval loop.
4. **HTTP load.** DISCONNECTED is fully passive (tunnel-gated): a powered-off drone draws
   zero HTTP polling.
5. **Selector reset correctness.** `reset_for_new_session()` must reset only volatile
   selector state and never the persistent `learned_prior` knees — covered by a targeted
   unit test.

## 11. Out of scope / future

- BF re-arm and probe-idle subscribers (documented hooks above).
- A shared per-flight "session id" derived on connect that subsystems tag their artifacts
  with (flight log, prior snapshots) for cross-correlation.
- Using the wfb session `epoch` as a finer "drone rebooted" signal than tunnel+HTTP.
