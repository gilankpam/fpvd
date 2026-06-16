# GS Drone Connection Events Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a global, subscribable "drone connected / disconnected" event on the GS, sourced from the wfb tunnel stream + HTTP confirmation, with the flight log, dynlink selector reset, and learned-prior flush as its first subscribers.

**Architecture:** A new always-on `ConnectionMonitor` subsystem (daemon thread + asyncio loop, mirroring `DynamicLinkController`) watches the tunnel stream on a second `:8103` `StatsClient`, confirms reachability via a short-timeout `DroneClient`, and publishes to a generic thread-safe `EventBus`. The dynlink controller subscribes and drives flight-log roll/sync, selector reset, and prior flush — marshaling each onto its own loop.

**Tech Stack:** Python ≥3.11, stdlib only (`threading`, `asyncio`, `urllib`), pytest, doctest-free GS side.

**Spec:** `docs/superpowers/specs/2026-06-17-gs-drone-connection-events-design.md`

**Conventions:**
- Run the full GS suite from `gs/`: `.venv/bin/python -m pytest tests/ -q`. Run one file: `.venv/bin/python -m pytest tests/unit/test_x.py -q`.
- All commits end with the trailer (shown in each commit step):
  `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`
- TDD: write the failing test, watch it fail, implement minimally, watch it pass, commit.

---

## File Structure

**New files:**
- `gs/fpvdgs/events.py` — generic thread-safe `EventBus` + event-name constants (`DRONE_CONNECTED`, `DRONE_DISCONNECTED`). One responsibility: in-process pub/sub.
- `gs/fpvdgs/connection_monitor.py` — `ConnectionMonitor` + `ConnectionMonitorConfig`. One responsibility: derive drone connect/disconnect from tunnel-stream + HTTP and publish it.
- `gs/tests/unit/test_events.py`, `gs/tests/unit/test_connection_monitor.py`.

**Modified files:**
- `gs/fpvdgs/config_defaults.py` — add the `connectionMonitor` default block.
- `gs/fpvdgs/schema.py` — accept + validate `connectionMonitor` (incl. the `tunnelStaleS > httpPollS` invariant).
- `gs/fpvdgs/status.py` — `build_status()` gains a `connection` block.
- `gs/fpvdgs/supervisor.py` — `build_app` creates the bus + monitor; `App` owns/starts/stops them; `status_fn` reports `connection`.
- `gs/fpvdgs/dynlink/flightlog.py` — add `begin_flight()` + `sync()`; remove the dead `flight_gap_s` field.
- `gs/fpvdgs/dynlink/policy.py` — add `reset_for_new_session()`; remove the inline gap-roll + `_last_healthy_mono` + unused `import time`.
- `gs/fpvdgs/dynlink/controller.py` — take a `bus`; subscribe; drive flight-log/selector/prior on connect/disconnect.
- Tests touched: `test_dl_flightlog.py`, `test_dl_policy_learned.py` (delete relocated gap-roll tests), `test_schema.py`, `test_status.py`, `test_app_wiring.py`, `test_dl_controller.py`.

**No `deploy/gs/deploy.sh` change** — line 34 (`scp "$GS/fpvdgs"/*.py`) already ships top-level modules.

---

## Task 1: EventBus

**Files:**
- Create: `gs/fpvdgs/events.py`
- Test: `gs/tests/unit/test_events.py`

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_events.py`:

```python
from fpvdgs.events import EventBus, DRONE_CONNECTED, DRONE_DISCONNECTED


def test_subscribe_receives_published_payload():
    bus = EventBus()
    got = []
    bus.subscribe("e", got.append)
    bus.publish("e", {"x": 1})
    assert got == [{"x": 1}]


def test_publish_with_no_payload_delivers_empty_dict():
    bus = EventBus()
    got = []
    bus.subscribe("e", got.append)
    bus.publish("e")
    assert got == [{}]


def test_unsubscribe_stops_delivery():
    bus = EventBus()
    got = []

    def cb(p):
        got.append(p)

    bus.subscribe("e", cb)
    bus.publish("e", {"n": 1})
    bus.unsubscribe("e", cb)
    bus.publish("e", {"n": 2})
    assert got == [{"n": 1}]


def test_subscriber_exception_is_isolated():
    bus = EventBus()
    got = []

    def bad(p):
        raise RuntimeError("boom")

    def good(p):
        got.append(p)

    bus.subscribe("e", bad)
    bus.subscribe("e", good)
    bus.publish("e", {"n": 1})       # bad raises; good must still run
    assert got == [{"n": 1}]


def test_dispatch_order_is_subscription_order():
    bus = EventBus()
    order = []
    bus.subscribe("e", lambda p: order.append("a"))
    bus.subscribe("e", lambda p: order.append("b"))
    bus.publish("e")
    assert order == ["a", "b"]


def test_state_caches_latest_drone_payload():
    bus = EventBus()
    assert bus.state("drone") is None
    bus.publish(DRONE_CONNECTED, {"state": "connected"})
    assert bus.state("drone") == {"state": "connected"}
    bus.publish(DRONE_DISCONNECTED, {"state": "disconnected", "reason": "tunnel_lost"})
    assert bus.state("drone")["state"] == "disconnected"
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_events.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.events'`

- [ ] **Step 3: Write minimal implementation**

Create `gs/fpvdgs/events.py`:

```python
"""Generic in-process pub/sub event bus for cross-subsystem GS events.

Thread-safe, synchronous, exception-isolated dispatch. Publishers and
subscribers may live on different threads; each callback runs on the
PUBLISHER's thread, so a callback must be quick, non-blocking, and thread-safe
(marshal real work onto its own loop). The bus caches the latest payload per
event so a late subscriber can read current state via state()."""
from __future__ import annotations

import logging
import threading
from typing import Callable

log = logging.getLogger("fpvdgs.events")

DRONE_CONNECTED = "drone.connected"
DRONE_DISCONNECTED = "drone.disconnected"

# event -> the state() cache key it updates
_STATE_KEY = {
    DRONE_CONNECTED: "drone",
    DRONE_DISCONNECTED: "drone",
}


class EventBus:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._subs: dict[str, list[Callable[[dict], None]]] = {}
        self._state: dict[str, dict] = {}

    def subscribe(self, event: str, cb: Callable[[dict], None]) -> None:
        with self._lock:
            self._subs.setdefault(event, []).append(cb)

    def unsubscribe(self, event: str, cb: Callable[[dict], None]) -> None:
        with self._lock:
            subs = self._subs.get(event)
            if subs and cb in subs:
                subs.remove(cb)

    def publish(self, event: str, payload: dict | None = None) -> None:
        payload = payload or {}
        # Snapshot subscribers + update the state cache under the lock, then
        # dispatch OUTSIDE it so a callback can safely re-enter the bus.
        with self._lock:
            subs = list(self._subs.get(event, ()))
            key = _STATE_KEY.get(event)
            if key is not None:
                self._state[key] = payload
        for cb in subs:
            try:
                cb(payload)
            except Exception:
                log.exception("event subscriber for %s raised", event)

    def state(self, key: str, default=None):
        with self._lock:
            v = self._state.get(key, default)
            return dict(v) if isinstance(v, dict) else v
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_events.py -q`
Expected: PASS (6 passed)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/events.py gs/tests/unit/test_events.py
git commit -m "feat(gs): generic thread-safe EventBus for cross-subsystem events" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: ConnectionMonitor

**Files:**
- Create: `gs/fpvdgs/connection_monitor.py`
- Test: `gs/tests/unit/test_connection_monitor.py`

Mirrors `DynamicLinkController`'s thread/loop lifecycle. Tests drive it through the public `start()/stop()/status()` API with a fake `stats_client_factory` and a fake `DroneClient`, using short intervals + deadline loops (the established `test_dl_controller.py` style).

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_connection_monitor.py`:

```python
import time

from fpvdgs.connection_monitor import ConnectionMonitor, ConnectionMonitorConfig
from fpvdgs.events import EventBus, DRONE_CONNECTED, DRONE_DISCONNECTED
from fpvdgs.dynlink.stats_client import RxEvent


def _tunnel_rx(stream_id="tunnel rx"):
    return RxEvent(timestamp=1.0, id=stream_id, packets_window={"data": 1})


def _stats_factory(control):
    """Factory whose client emits a tunnel rx each loop while control['emit'];
    control['id'] selects the stream id so a test can emit non-tunnel records."""
    class _Stats:
        def __init__(self, endpoint, on_event):
            self._on = on_event
            self._stop = False

        async def run(self):
            import asyncio
            while not self._stop:
                if control.get("emit"):
                    self._on(_tunnel_rx(control.get("id", "tunnel rx")))
                await asyncio.sleep(0.01)

        def stop(self):
            self._stop = True

    return _Stats


class _FakeDrone:
    def __init__(self, status_ok=True, healthz_ok=True, version="d1"):
        self.status_ok = status_ok
        self.healthz_ok = healthz_ok
        self.version = version

    def get_status(self):
        if not self.status_ok:
            raise RuntimeError("unreachable")
        return {"version": self.version}

    def healthz(self):
        return self.healthz_ok


def _fast_cfg(**over):
    base = dict(tunnel_stale_s=0.2, http_poll_s=0.02,
                http_timeout_s=0.5, http_fail_count=2, eval_interval_s=0.02)
    base.update(over)
    return ConnectionMonitorConfig(**base)


def _wait(pred, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


def test_connects_when_tunnel_and_http_ok():
    control = {"emit": True}
    bus = EventBus()
    got = []
    bus.subscribe(DRONE_CONNECTED, got.append)
    m = ConnectionMonitor(bus, _FakeDrone(version="d-9"), _fast_cfg(),
                          stats_client_factory=_stats_factory(control))
    m.start()
    try:
        assert _wait(lambda: bool(got)), "expected DRONE_CONNECTED"
        assert got[0]["drone"]["version"] == "d-9"
        assert m.status()["state"] == "connected"
    finally:
        m.stop()


def test_disconnect_on_tunnel_loss():
    control = {"emit": True}
    bus = EventBus()
    events = []
    bus.subscribe(DRONE_DISCONNECTED, events.append)
    m = ConnectionMonitor(bus, _FakeDrone(), _fast_cfg(),
                          stats_client_factory=_stats_factory(control))
    m.start()
    try:
        assert _wait(lambda: m.status()["state"] == "connected")
        control["emit"] = False                      # tunnel goes silent
        assert _wait(lambda: bool(events)), "expected DRONE_DISCONNECTED"
        assert events[0]["reason"] == "tunnel_lost"
    finally:
        m.stop()


def test_disconnect_on_http_failure():
    control = {"emit": True}
    bus = EventBus()
    events = []
    bus.subscribe(DRONE_DISCONNECTED, events.append)
    drone = _FakeDrone()
    m = ConnectionMonitor(bus, drone, _fast_cfg(),
                          stats_client_factory=_stats_factory(control))
    m.start()
    try:
        assert _wait(lambda: m.status()["state"] == "connected")
        drone.healthz_ok = False                     # heartbeat starts failing
        assert _wait(lambda: bool(events)), "expected DRONE_DISCONNECTED"
        assert events[0]["reason"] == "http_failed"
    finally:
        m.stop()


def test_armed_without_http_never_announces_connected():
    control = {"emit": True}
    bus = EventBus()
    got = []
    bus.subscribe(DRONE_CONNECTED, got.append)
    m = ConnectionMonitor(bus, _FakeDrone(status_ok=False), _fast_cfg(),
                          stats_client_factory=_stats_factory(control))
    m.start()
    try:
        time.sleep(0.5)
        assert got == []                             # tunnel up but HTTP never confirms
        assert m.status()["state"] != "connected"
    finally:
        m.stop()


def test_only_tunnel_stream_arms_the_monitor():
    control = {"emit": True, "id": "video rx"}       # video, not tunnel
    bus = EventBus()
    got = []
    bus.subscribe(DRONE_CONNECTED, got.append)
    m = ConnectionMonitor(bus, _FakeDrone(), _fast_cfg(),
                          stats_client_factory=_stats_factory(control))
    m.start()
    try:
        time.sleep(0.5)
        assert got == []                             # never armed -> never connected
    finally:
        m.stop()


def test_disabled_does_not_start_a_thread():
    bus = EventBus()
    got = []
    bus.subscribe(DRONE_CONNECTED, got.append)
    m = ConnectionMonitor(bus, _FakeDrone(), _fast_cfg(enabled=False),
                          stats_client_factory=_stats_factory({"emit": True}))
    m.start()
    try:
        time.sleep(0.3)
        assert got == []
        assert m.status()["enabled"] is False
    finally:
        m.stop()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_connection_monitor.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.connection_monitor'`

- [ ] **Step 3: Write minimal implementation**

Create `gs/fpvdgs/connection_monitor.py`:

```python
"""Always-on drone connection monitor.

Watches the wfb tunnel stream on the :8103 stats feed and confirms drone
reachability via the HTTP API, publishing drone.connected / drone.disconnected
on an EventBus. Owns a daemon thread + asyncio loop (mirrors
DynamicLinkController). Independent of dynamicLink — it runs whenever fpvd runs.

State machine (evaluated every eval_interval_s):
  DISCONNECTED -> ARMED      : tunnel rx seen within tunnel_stale_s
  ARMED -> CONNECTED         : get_status() succeeds (publishes, carries payload)
  ARMED -> DISCONNECTED      : tunnel goes stale before confirmation (no event)
  CONNECTED -> DISCONNECTED  : tunnel stale OR http_fail_count heartbeat failures

Invariant: tunnel_stale_s > http_poll_s, so the heartbeat's own HTTP return
traffic keeps the tunnel 'fresh' on an otherwise-idle but healthy link."""
from __future__ import annotations

import asyncio
import logging
import threading
import time
from dataclasses import dataclass

from .dynlink.stats_client import RxEvent, StatsClient
from .events import DRONE_CONNECTED, DRONE_DISCONNECTED

log = logging.getLogger("fpvdgs.connection")


@dataclass
class ConnectionMonitorConfig:
    enabled: bool = True
    tunnel_stale_s: float = 4.0
    http_poll_s: float = 1.5
    http_timeout_s: float = 1.5
    http_fail_count: int = 2
    eval_interval_s: float = 0.5


class ConnectionMonitor:
    def __init__(self, bus, drone_client, cfg=None, *,
                 stats_endpoint="tcp://127.0.0.1:8103",
                 stats_client_factory=StatsClient,
                 time_fn=time.monotonic):
        self._bus = bus
        self._drone = drone_client
        self._cfg = cfg or ConnectionMonitorConfig()
        self._stats_endpoint = stats_endpoint
        self._make_stats = stats_client_factory
        self._time = time_fn
        self._lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None
        self._started = threading.Event()
        # state machine
        self._state = "disconnected"
        self._since = 0.0
        self._reason = ""
        self._drone_info = None
        self._last_tunnel_rx = -1.0e9   # monotonic; far past => stale at boot
        self._fail = 0
        self._last_http = -1.0e9

    # ---- public thread-safe API ----
    def start(self):
        if not self._cfg.enabled:
            return
        with self._lifecycle:
            with self._lock:
                if self._thread and self._thread.is_alive():
                    return
                self._started.clear()
                self._thread = threading.Thread(target=self._thread_main,
                                                name="conn-monitor", daemon=True)
                self._thread.start()
            self._started.wait(timeout=5.0)

    def stop(self):
        with self._lifecycle:
            with self._lock:
                loop, stop, thread = self._loop, self._stop_event, self._thread
            if loop is not None and stop is not None:
                try:
                    loop.call_soon_threadsafe(stop.set)
                except RuntimeError:
                    pass
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=5.0)
            with self._lock:
                self._thread = None

    def status(self):
        with self._lock:
            since_ms = None
            if self._state == "connected":
                since_ms = int((self._time() - self._since) * 1000)
            return {
                "enabled": bool(self._cfg.enabled),
                "state": self._state,
                "reason": self._reason,
                "sinceMs": since_ms,
                "drone": dict(self._drone_info) if self._drone_info else None,
            }

    # ---- internals ----
    def _thread_main(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        with self._lock:
            self._loop = loop
        try:
            loop.run_until_complete(self._run())
        except Exception:
            log.exception("conn-monitor loop crashed")
        finally:
            try:
                loop.close()
            finally:
                with self._lock:
                    self._loop = None
                self._started.set()

    async def _run(self):
        self._stop_event = asyncio.Event()
        self._started.set()
        client = self._make_stats(self._stats_endpoint, self._on_stats_event)
        run_task = asyncio.ensure_future(client.run())
        eval_task = asyncio.ensure_future(self._eval_loop())
        stop_task = asyncio.ensure_future(self._stop_event.wait())
        try:
            await asyncio.wait({run_task, eval_task, stop_task},
                               return_when=asyncio.FIRST_COMPLETED)
        finally:
            client.stop()
            for t in (run_task, eval_task, stop_task):
                t.cancel()

    def _on_stats_event(self, ev):
        # Only the tunnel stream marks reachability; video/mavlink are ignored.
        if isinstance(ev, RxEvent) and ev.id and "tunnel" in ev.id.lower():
            with self._lock:
                self._last_tunnel_rx = self._time()

    async def _eval_loop(self):
        while not self._stop_event.is_set():
            try:
                await self._evaluate()
            except Exception:
                log.exception("conn-monitor evaluate failed")
            try:
                await asyncio.wait_for(self._stop_event.wait(),
                                       timeout=self._cfg.eval_interval_s)
            except asyncio.TimeoutError:
                pass

    async def _evaluate(self):
        now = self._time()
        with self._lock:
            last_rx = self._last_tunnel_rx
            state = self._state
        fresh = (now - last_rx) < self._cfg.tunnel_stale_s

        if state == "disconnected":
            if not fresh:
                return
            state = "armed"

        if state == "armed":
            snap = await self._call(self._drone.get_status)
            if snap is not None:
                self._enter_connected(snap, now)
            else:
                with self._lock:
                    self._state = "armed" if fresh else "disconnected"
            return

        # state == "connected"
        if (now - self._last_http) >= self._cfg.http_poll_s:
            self._last_http = now
            ok = await self._call_bool(self._drone.healthz)
            self._fail = 0 if ok else self._fail + 1
        if not fresh or self._fail >= self._cfg.http_fail_count:
            self._enter_disconnected("tunnel_lost" if not fresh else "http_failed", now)

    async def _call(self, fn):
        loop = asyncio.get_event_loop()
        try:
            return await loop.run_in_executor(None, fn)
        except Exception:
            return None

    async def _call_bool(self, fn):
        loop = asyncio.get_event_loop()
        try:
            return bool(await loop.run_in_executor(None, fn))
        except Exception:
            return False

    def _enter_connected(self, snap, now):
        info = {"version": snap.get("version")} if isinstance(snap, dict) else {}
        with self._lock:
            self._state = "connected"
            self._since = now
            self._reason = ""
            self._drone_info = info
            self._fail = 0
            self._last_http = now
        log.info("drone connected: %s", info)
        self._bus.publish(DRONE_CONNECTED,
                          {"state": "connected", "at_mono": now, "drone": info})

    def _enter_disconnected(self, reason, now):
        with self._lock:
            last_seen = self._since
            self._state = "disconnected"
            self._reason = reason
            self._drone_info = None
            self._fail = 0
        log.info("drone disconnected: reason=%s", reason)
        self._bus.publish(DRONE_DISCONNECTED,
                          {"state": "disconnected", "at_mono": now,
                           "reason": reason, "last_seen_mono": last_seen})
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_connection_monitor.py -q`
Expected: PASS (6 passed). If a timing test flakes, it is a real ordering bug — do not just bump timeouts; confirm the state machine logic.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/connection_monitor.py gs/tests/unit/test_connection_monitor.py
git commit -m "feat(gs): tunnel-gated, HTTP-confirmed ConnectionMonitor subsystem" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Config defaults + schema validation

**Files:**
- Modify: `gs/fpvdgs/config_defaults.py`
- Modify: `gs/fpvdgs/schema.py:5` (CONFIG_TOP_KEYS), `:64-93` (validate_effective), add `_validate_connection_monitor`
- Test: `gs/tests/unit/test_schema.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_schema.py`:

```python
def test_connection_monitor_accepts_shipped_defaults():
    from fpvdgs.config_defaults import default_config
    validate_effective(default_config())          # includes connectionMonitor; must pass


def test_connection_monitor_invariant_rejects_stale_le_poll():
    from fpvdgs.config_defaults import default_config
    cfg = default_config()
    cfg["connectionMonitor"]["tunnelStaleS"] = 1.0
    cfg["connectionMonitor"]["httpPollS"] = 1.5    # stale must be > poll
    with pytest.raises(SchemaError):
        validate_effective(cfg)


def test_connection_monitor_rejects_bad_fail_count():
    from fpvdgs.config_defaults import default_config
    cfg = default_config()
    cfg["connectionMonitor"]["httpFailCount"] = 0  # must be a positive int
    with pytest.raises(SchemaError):
        validate_effective(cfg)


def test_config_patch_accepts_connection_monitor():
    validate_config_patch({"connectionMonitor": {"enabled": False}})   # no raise
```

Confirm the test file already imports `pytest`, `validate_effective`, `validate_config_patch`, `SchemaError` (it does — see the top of the file).

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_schema.py -q`
Expected: FAIL — `test_connection_monitor_accepts_shipped_defaults` raises `KeyError`/`SchemaError` (no `connectionMonitor` in defaults) and `test_config_patch_accepts_connection_monitor` raises `SchemaError` (unknown top-level key).

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/config_defaults.py`, add the import near the other dynlink imports (top of file):

```python
from .connection_monitor import ConnectionMonitorConfig
```

In `default_config()`, add a `connectionMonitor` entry to the returned dict (DRY — sourced from the dataclass defaults). Insert it after the `"idrForward"` entry:

```python
        "idrForward": {"enabled": True, "port": 11223},
        "connectionMonitor": _connection_monitor_defaults(),
```

And add this helper above `default_config()`:

```python
def _connection_monitor_defaults() -> dict:
    cm = ConnectionMonitorConfig()
    return {
        "enabled": cm.enabled,
        "tunnelStaleS": cm.tunnel_stale_s,
        "httpPollS": cm.http_poll_s,
        "httpTimeoutS": cm.http_timeout_s,
        "httpFailCount": cm.http_fail_count,
        "evalIntervalS": cm.eval_interval_s,
    }
```

In `gs/fpvdgs/schema.py`, add `connectionMonitor` to `CONFIG_TOP_KEYS` (line 5):

```python
CONFIG_TOP_KEYS = {"link", "wfb", "drone", "dynamicLink", "pixelpilot",
                   "idrForward", "connectionMonitor"}
```

Add a key set near the other `*_KEYS` constants:

```python
CONNECTION_MONITOR_KEYS = {"enabled", "tunnelStaleS", "httpPollS",
                           "httpTimeoutS", "httpFailCount", "evalIntervalS"}
```

In `validate_effective()`, after the `idrForward` block (around line 90), add:

```python
    cm = cfg.get("connectionMonitor")
    if cm is not None:
        _validate_connection_monitor(cm)
```

Add the validator (place it near `_validate_idr_forward`). It is lenient on unknown keys — like `pixelpilot`/`idrForward`, and unlike the strict `dynamicLink` — so a stale removed knob in an on-disk config can never brick boot:

```python
def _validate_connection_monitor(cm: dict) -> None:
    if not isinstance(cm, dict):
        raise SchemaError("connectionMonitor must be an object")
    if not isinstance(cm.get("enabled", True), bool):
        raise SchemaError("connectionMonitor.enabled must be a bool")
    for k in ("tunnelStaleS", "httpPollS", "httpTimeoutS", "evalIntervalS"):
        v = cm.get(k)
        if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0):
            raise SchemaError(f"connectionMonitor.{k} must be a positive number")
    _validate_pos_int("connectionMonitor.httpFailCount", cm.get("httpFailCount"))
    # Invariant: the heartbeat's own HTTP return traffic keeps the tunnel 'fresh',
    # so tunnelStaleS must exceed httpPollS or a quiet healthy link false-disconnects.
    stale = cm.get("tunnelStaleS", 4.0)
    poll = cm.get("httpPollS", 1.5)
    if isinstance(stale, (int, float)) and isinstance(poll, (int, float)) and not stale > poll:
        raise SchemaError("connectionMonitor.tunnelStaleS must be > httpPollS")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_schema.py -q`
Expected: PASS (all schema tests, including the 4 new ones)

Then run the full suite — adding a default-config key must not break the config tests (`test_config.py:70` asserts `effective() == default_config()`, which stays true since both gain the block):
Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS (no regressions)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/config_defaults.py gs/fpvdgs/schema.py gs/tests/unit/test_schema.py
git commit -m "feat(gs): connectionMonitor config block + schema validation" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Supervisor wiring + /status connection block

**Files:**
- Modify: `gs/fpvdgs/status.py:33-65` (build_status)
- Modify: `gs/fpvdgs/supervisor.py` (imports, `App`, `build_app`, `status_fn`)
- Test: `gs/tests/unit/test_status.py`, `gs/tests/unit/test_app_wiring.py`

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_status.py`:

```python
def test_build_status_includes_connection_when_passed():
    from fpvdgs.status import build_status
    out = build_status("v", {}, {}, {"linkId": 1},
                       connection={"state": "connected", "drone": {"version": "d1"}})
    assert out["connection"]["state"] == "connected"


def test_build_status_omits_connection_when_absent():
    from fpvdgs.status import build_status
    out = build_status("v", {}, {}, {"linkId": 1})
    assert "connection" not in out
```

Append to `gs/tests/unit/test_app_wiring.py`:

```python
def test_app_starts_and_stops_connection_monitor():
    store = ConfigStore({"dynamicLink": {"enabled": False}})
    runner = _Fake("runner")
    mon = _Fake("mon")
    app = App(store, runner, _Fake("http"), api=None, dynlink=_Fake("dynlink"),
              connection_monitor=mon)
    app.start()
    assert "start" in mon.calls          # always-on, regardless of dynamicLink
    app.shutdown()
    assert "stop" in mon.calls


def test_build_app_wires_connection_monitor_and_bus(tmp_path, monkeypatch):
    import fpvdgs.supervisor as sup
    monkeypatch.setattr(sup.render_mod, "write_cfg", lambda *a, **k: None)
    monkeypatch.setattr(sup.render_mod, "render_cfg", lambda eff: "")
    config = tmp_path / "config.json"
    config.write_text('{"link": {"region": "US", "channel": 132, "width": 20, '
                      '"wlans": ["wlan0"]}}')
    app = sup.build_app(str(config), str(tmp_path / "out.cfg"),
                        "127.0.0.1", 0, runner_cmd=["true"])
    assert app.connection_monitor is not None
    assert app.bus is not None
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_status.py tests/unit/test_app_wiring.py -q`
Expected: FAIL — `build_status() got an unexpected keyword argument 'connection'`; `App.__init__() got an unexpected keyword argument 'connection_monitor'`

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/status.py`, extend `build_status` — add the `connection` parameter and emit it:

```python
def build_status(version: str, runner_state: dict, wlans: dict,
                 link_info: dict, link_stats: dict | None = None,
                 uptime_ms: int | None = None,
                 dynamic_link: dict | None = None,
                 pixelpilot: dict | None = None,
                 probe: dict | None = None,
                 beamforming: dict | None = None,
                 connection: dict | None = None) -> dict:
```

and just before `return out` add:

```python
    if connection is not None:
        out["connection"] = connection
```

In `gs/fpvdgs/supervisor.py`:

Add imports near the other local imports (after `from .config_defaults import default_config`):

```python
from .connection_monitor import ConnectionMonitor, ConnectionMonitorConfig
from .events import EventBus
```

Extend `App.__init__` to own the bus + monitor:

```python
    def __init__(self, store, runner, http_server, api, dynlink,
                 pixelpilot=None, probe=None, armer=None, idr_relay=None,
                 connection_monitor=None, bus=None):
        self.store = store
        self.runner = runner
        self.http = http_server
        self.api = api
        self.dynlink = dynlink
        self.pixelpilot = pixelpilot
        self.probe = probe
        self.armer = armer
        self.idr_relay = idr_relay
        self.connection_monitor = connection_monitor
        self.bus = bus
```

In `App.start()`, start the monitor right after the runner (the runner brings up `:8103`; `.start()` is a no-op when the monitor is disabled):

```python
    def start(self):
        self.runner.start()
        if self.connection_monitor is not None:
            self.connection_monitor.start()
        if self.armer is not None:
            self.armer.start()   # boot re-arm: keeps the GS beamformee armed to config
```

In `App.shutdown()`, stop the monitor (it is independent — stop it early):

```python
    def shutdown(self):
        self.http.shutdown()
        if self.connection_monitor is not None:
            self.connection_monitor.stop()
        if self.armer is not None:
            self.armer.stop()
```

In `build_app`, create the bus + monitor. Insert after the `drone = DroneClient(...)` line (which defines `drone_host` and `drone_cfg`):

```python
    bus = EventBus()
    cm_cfg = effective.get("connectionMonitor", {})
    dcm = ConnectionMonitorConfig()
    mon_cfg = ConnectionMonitorConfig(
        enabled=bool(cm_cfg.get("enabled", dcm.enabled)),
        tunnel_stale_s=float(cm_cfg.get("tunnelStaleS", dcm.tunnel_stale_s)),
        http_poll_s=float(cm_cfg.get("httpPollS", dcm.http_poll_s)),
        http_timeout_s=float(cm_cfg.get("httpTimeoutS", dcm.http_timeout_s)),
        http_fail_count=int(cm_cfg.get("httpFailCount", dcm.http_fail_count)),
        eval_interval_s=float(cm_cfg.get("evalIntervalS", dcm.eval_interval_s)),
    )
    mon_drone = DroneClient(
        f"http://{drone_host}:{int(drone_cfg.get('apiPort', 8080))}",
        timeout=mon_cfg.http_timeout_s)
    connection_monitor = ConnectionMonitor(bus, mon_drone, mon_cfg)
```

Leave the dynlink controller construction **unchanged** here — Task 7 adds its `bus=` parameter and will pass `bus=bus` at this line. (Passing `bus=bus` now, before Task 7, raises `TypeError` because the controller does not yet accept it.)

In `status_fn`, add the connection block to the `build_status(...)` call:

```python
        return status_mod.build_status(__version__, runner.state(), wlan_info, link_info,
                                       uptime_ms=uptime_ms,
                                       dynamic_link=_dynamic_link_status(),
                                       pixelpilot=_pixelpilot_status(),
                                       probe=_probe_status(),
                                       beamforming=beamforming.status_with_primary(primary),
                                       connection=connection_monitor.status())
```

Extend the `App(...)` construction in `build_app`'s return:

```python
    http_server = make_http_server(api, host, port)
    return App(store, runner, http_server, api, dynlink,
               pixelpilot=pixelpilot, probe=probe_ctrl, armer=armer,
               idr_relay=idr_relay, connection_monitor=connection_monitor, bus=bus)
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_status.py tests/unit/test_app_wiring.py -q`
Expected: PASS

Run the full suite to catch wiring fallout:
Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS (no regressions)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/status.py gs/fpvdgs/supervisor.py \
        gs/tests/unit/test_status.py gs/tests/unit/test_app_wiring.py
git commit -m "feat(gs): wire EventBus + ConnectionMonitor into App; report /status connection" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: FlightLog begin_flight() + sync()

**Files:**
- Modify: `gs/fpvdgs/dynlink/flightlog.py` (add two methods)
- Test: `gs/tests/unit/test_dl_flightlog.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_flightlog.py`:

```python
def test_begin_flight_keeps_fresh_empty_file(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    path1 = fl._path
    fl.begin_flight()                  # nothing written yet -> keep the same file
    assert fl._path == path1
    fl.close()
    assert len(list(tmp_path.glob("*.jsonl"))) == 1


def test_begin_flight_rolls_when_file_has_records(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=8))
    fl.write({"ts": 1.0})
    fl.begin_flight()                  # has records -> start a new flight file
    fl.write({"ts": 2.0})
    fl.close()
    assert len(list(tmp_path.glob("*.jsonl"))) == 2


def test_begin_flight_noop_when_disabled(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), enabled=False))
    fl.begin_flight()
    fl.close()
    assert list(tmp_path.iterdir()) == []


def test_sync_fsyncs_open_file(tmp_path, monkeypatch):
    import fpvdgs.dynlink.flightlog as mod
    calls = []
    monkeypatch.setattr(mod.os, "fsync", lambda fd: calls.append(fd))
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    fl.write({"ts": 1.0})
    fl.sync()
    assert calls, "sync() must fsync the open file"
    fl.close()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog.py -q`
Expected: FAIL — `AttributeError: 'FlightLog' object has no attribute 'begin_flight'`

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/dynlink/flightlog.py`, add these two methods to `FlightLog` (e.g. right after `roll()`):

```python
    def sync(self) -> None:
        """Flush + fsync the open flight file now — durability on demand, e.g.
        at a link-loss edge. No-op if no file is open."""
        self._sync()

    def begin_flight(self) -> None:
        """Ensure a fresh file is open for a new flight: roll to a new file if
        the current one already holds records, keep an already-open empty file,
        or (re)open one if none is open. No-op if disabled. Driven by the
        drone-connected event."""
        if not self.cfg.enabled:
            return
        if self._fh is not None and self._bytes == 0:
            return                      # already on a fresh, empty flight file
        self.roll()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/flightlog.py gs/tests/unit/test_dl_flightlog.py
git commit -m "feat(gs): FlightLog.begin_flight() + sync() for event-driven boundaries" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Policy.reset_for_new_session() + retire the inline gap-roll

This removes the video-starvation gap-roll (relocated to the ConnectionMonitor) and adds the selector reset. It deletes the three gap-roll tests in `test_dl_policy_learned.py` (the behavior is re-tested via the connection event in Task 7) and drops the dead `flight_gap_s` field.

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py` (add `reset_for_new_session`; remove gap block + `_last_healthy_mono` + unused `import time`)
- Modify: `gs/fpvdgs/dynlink/flightlog.py:22` (remove `flight_gap_s` field)
- Modify: `gs/tests/unit/test_dl_flightlog.py:57-61` (drop the gap assertion)
- Modify: `gs/tests/unit/test_dl_policy_learned.py` (delete 3 gap tests + `_sig_starved` + `flight_gap_s` param)
- Test: `gs/tests/unit/test_dl_policy_learned.py` (new reset test)

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_policy_learned.py` (uses the existing `_cfg`, `_profile` helpers in that file):

```python
def test_reset_for_new_session_resets_selector_keeps_prior(tmp_path):
    p = Policy(_cfg(tmp_path), _profile())
    prior_before = p.learned_prior
    # Simulate a session that climbed + accumulated hysteresis state.
    p.leading.state.current_mcs = 5
    p._cold_started = True
    p._loss_count = 3
    p._starvation_count = 4
    p._snr_demote_count = 2
    p.leading._promote_clean = 3

    p.reset_for_new_session()

    assert p.leading.state.current_mcs == 1     # back to the boot MCS
    assert p._cold_started is False             # warm-start will re-run
    assert p._loss_count == 0
    assert p._starvation_count == 0
    assert p._snr_demote_count == 0
    assert p.leading._promote_clean == 0
    assert p.learned_prior is prior_before      # persistent knees preserved
    p.close()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py::test_reset_for_new_session_resets_selector_keeps_prior -q`
Expected: FAIL — `AttributeError: 'Policy' object has no attribute 'reset_for_new_session'`

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/dynlink/policy.py`, add the method to `Policy` (e.g. just before `close()`):

```python
    def reset_for_new_session(self) -> None:
        """Reset volatile selector + hysteresis state to boot. A confirmed drone
        reconnect is a new session, so re-run the learned-prior warm-start and
        re-climb from the boot MCS instead of resuming a stale climbed-up rung.
        The persistent learned_prior knees are kept (cross-session knowledge)."""
        self.leading = LeadingSelector(self.cfg.selector)
        self._cold_started = False
        self._starvation_count = 0
        self._loss_count = 0
        self._ticks_at_mcs = 0
        self._last_ingest_mcs = None
        self._predict_demote_count = 0
        self._snr_demote_count = 0
        self._rssi_window.clear()
```

Remove the inline gap-roll. Delete line 295 (`self._last_healthy_mono = None ...`) from `__init__`, and delete the gap-roll block in `tick()` (currently lines 433-442):

```python
        # Flight-boundary roll: a new flight = the link returning healthy after
        # being gone (starved) longer than flight_gap_s. Monotonic time so the
        # unreliable GS wall-clock can't break it; raw link_starved_w as health.
        if not signals.link_starved_w:
            _now_mono = time.monotonic()
            if (self._last_healthy_mono is not None
                    and (_now_mono - self._last_healthy_mono)
                    > self.cfg.flightlog.flight_gap_s):
                self.flightlog.roll()
            self._last_healthy_mono = _now_mono
```

`time.monotonic()` was the only `time.` use in the file. Remove the now-unused `import time` (line 15). Verify:

Run: `cd gs && grep -n "time\." fpvdgs/dynlink/policy.py`
Expected: no output → safe to delete `import time`.

In `gs/fpvdgs/dynlink/flightlog.py`, remove the dead field (line 22):

```python
    flight_gap_s: float = 15.0   # link gone > this (s) => next healthy tick = new flight file
```

Update the affected tests. In `gs/tests/unit/test_dl_flightlog.py`, replace `test_config_defaults_dvr_dir_and_gap` (lines 57-61) with:

```python
def test_config_defaults_dvr_dir():
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    c = FlightLogConfig()
    assert c.dir == "/media/dvr/log/dynamic-link/"
```

In `gs/tests/unit/test_dl_policy_learned.py`:
- Change the `_cfg_fl` helper (line 64) to drop the `flight_gap_s` parameter:

```python
def _cfg_fl(tmp_path):
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path / "lp")),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )
```

- Delete the three relocated gap-roll tests in their entirety: `test_flight_rolls_on_link_gap_recovery`, `test_brief_gap_does_not_roll`, `test_first_healthy_tick_does_not_roll` (lines 78-111).
- Delete the now-unused `_sig_starved` helper (lines 73-75). Confirm it is unused first:

Run: `cd gs && grep -n "_sig_starved" tests/unit/test_dl_policy_learned.py`
Expected: no output after the three tests are deleted → safe to remove the helper.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py tests/unit/test_dl_flightlog.py -q`
Expected: PASS (new reset test passes; gap-roll tests gone; no `flight_gap_s` references)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/policy.py gs/fpvdgs/dynlink/flightlog.py \
        gs/tests/unit/test_dl_policy_learned.py gs/tests/unit/test_dl_flightlog.py
git commit -m "refactor(gs): selector reset for new session; retire inline flight-gap roll" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: DynamicLinkController subscribes to connection events

Wire the three subscribers: connect → `flightlog.begin_flight()` + `policy.reset_for_new_session()`; disconnect → `flightlog.sync()` + `learned_prior.flush()`. Bus callbacks run on the monitor's thread and marshal onto the controller's own loop.

**Files:**
- Modify: `gs/fpvdgs/dynlink/controller.py`
- Test: `gs/tests/unit/test_dl_controller.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_controller.py` (the `_RepeatStatsClient`, `_snapshot`, `_free_udp_port` helpers already exist in the file):

```python
def test_connect_event_resets_selector_and_begins_flight():
    from fpvdgs.events import EventBus, DRONE_CONNECTED
    bus = EventBus()
    drone_sock, drone_port = _free_udp_port()
    c = DynamicLinkController(_snapshot(drone_port),
                              stats_client_factory=_RepeatStatsClient, bus=bus)
    c.start()
    try:
        # wait for the policy to exist, then simulate a climbed-up session
        deadline = time.monotonic() + 1.5
        while c._policy is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert c._policy is not None
        c._policy.leading.state.current_mcs = 5

        bus.publish(DRONE_CONNECTED, {"state": "connected", "drone": {}})

        deadline = time.monotonic() + 1.5
        while c._policy.leading.state.current_mcs != 1 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert c._policy.leading.state.current_mcs == 1   # reset to boot on reconnect
    finally:
        c.stop()
        drone_sock.close()


def test_disconnect_event_flushes_prior():
    from fpvdgs.events import EventBus, DRONE_DISCONNECTED
    bus = EventBus()
    drone_sock, drone_port = _free_udp_port()
    c = DynamicLinkController(_snapshot(drone_port),
                              stats_client_factory=_RepeatStatsClient, bus=bus)
    c.start()
    try:
        deadline = time.monotonic() + 1.5
        while c._policy is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert c._policy is not None
        flushed = []
        c._policy.learned_prior.flush = lambda: flushed.append(True)  # spy

        bus.publish(DRONE_DISCONNECTED, {"state": "disconnected", "reason": "tunnel_lost"})

        deadline = time.monotonic() + 1.5
        while not flushed and time.monotonic() < deadline:
            time.sleep(0.02)
        assert flushed, "disconnect must flush the learned prior"
    finally:
        c.stop()
        drone_sock.close()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_controller.py -q`
Expected: FAIL — `DynamicLinkController.__init__() got an unexpected keyword argument 'bus'`

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/dynlink/controller.py`, add the events import near the top:

```python
from ..events import DRONE_CONNECTED, DRONE_DISCONNECTED
```

Extend `__init__` to take a `bus` and subscribe (and initialize `self._policy`):

```python
    def __init__(self, snapshot, *, stats_endpoint="tcp://127.0.0.1:8103",
                 stats_client_factory=StatsClient, probe_status=None, bus=None):
        self._snapshot = dict(snapshot)
        self._stats_endpoint = stats_endpoint
        self._make_stats = stats_client_factory
        self._probe_status = probe_status
        self._bus = bus
        self._policy = None
        self._lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None         # asyncio.Event, created in-loop
        self._started = threading.Event()
        self._status = {"running": False, "statsConnected": False,
                        "decision": None, "lastEmitMs": None, "emitSeq": 0,
                        "reason": ""}
        if bus is not None:
            bus.subscribe(DRONE_CONNECTED, self._on_drone_connected)
            bus.subscribe(DRONE_DISCONNECTED, self._on_drone_disconnected)
```

In `_run`, publish the live `policy` to the instance (so the marshaled handlers can reach it) right after it is created:

```python
        policy = Policy(build_policy_config(snap), profile_name,
                        probe_status=self._probe_status)
        with self._lock:
            self._policy = policy
```

and clear it in the `finally` block of `_run`:

```python
        try:
            await self._stats_loop(on_event)
        finally:
            policy.close()
            return_link.close()
            with self._lock:
                self._policy = None
            self._set(running=False, statsConnected=False)
```

Add the bus handlers + the in-loop work. The `_on_*` methods run on the monitor's thread and marshal onto our loop; the `_inloop` methods run on the dl loop thread (same thread as the per-tick writes), so they touch `FlightLog`/`Policy` without a lock:

```python
    # ---- connection-event subscribers (called on the monitor's thread) ----
    def _marshal(self, fn):
        with self._lock:
            loop = self._loop
        if loop is None:
            return                       # loop down (dynlink disabled/stopped)
        try:
            loop.call_soon_threadsafe(fn)
        except RuntimeError:
            pass                         # loop tearing down

    def _on_drone_connected(self, payload):
        self._marshal(self._connected_inloop)

    def _on_drone_disconnected(self, payload):
        self._marshal(self._disconnected_inloop)

    def _connected_inloop(self):
        p = self._policy
        if p is None:
            return
        p.reset_for_new_session()        # new session: re-warm-start, re-climb
        p.flightlog.begin_flight()       # start a fresh flight file

    def _disconnected_inloop(self):
        p = self._policy
        if p is None:
            return
        p.flightlog.sync()               # make the flight durable at the loss edge
        p.learned_prior.flush()          # persist the session's learning
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_controller.py -q`
Expected: PASS (existing controller tests + the 2 new ones)

If Task 4 deferred adding `bus=bus` to the controller construction in `build_app`, add it now:

```python
    dynlink = DynamicLinkController(make_dl_snapshot(effective),
                                    probe_status=probe_ctrl.status, bus=bus)
```

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/controller.py gs/fpvdgs/supervisor.py \
        gs/tests/unit/test_dl_controller.py
git commit -m "feat(gs): dynlink subscribes to drone connect/disconnect events" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: Full-suite verification + import-coupling guard

**Files:** none (verification only)

- [ ] **Step 1: Run the full GS suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS — entire suite green. The GS suite must stay green as a whole (config_build/import coupling). If `test_dl_imports.py` or `test_config.py` fail, a default-config or import wiring is off — fix before proceeding.

- [ ] **Step 2: Verify the materialized default config includes the new block**

Run: `cd gs && .venv/bin/python -m fpvdgs.supervisor --dump-config | .venv/bin/python -c "import sys, json; c = json.load(sys.stdin); print('connectionMonitor' in c, c.get('connectionMonitor'))"`
Expected: `True {'enabled': True, 'tunnelStaleS': 4.0, 'httpPollS': 1.5, 'httpTimeoutS': 1.5, 'httpFailCount': 2, 'evalIntervalS': 0.5}`

- [ ] **Step 3: Verify a stale config without the block still loads (deep-merge backfill)**

Run: `cd gs && .venv/bin/python -c "from fpvdgs.config import ConfigStore; from fpvdgs.schema import validate_effective; s = ConfigStore({'link': {'region': 'US', 'channel': 132, 'width': 20}}); validate_effective(s.effective()); print('connectionMonitor' in s.effective())"`
Expected: `True` (defaults backfill the missing block; validate_effective passes)

- [ ] **Step 4: Commit (if any verification fixes were needed; otherwise skip)**

```bash
git add -A
git commit -m "test(gs): verify connection-events wiring + default-config backfill" \
  -m "Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Bench verification (post-merge, on hardware — not part of the automated suite)

These are the spec's risk-list items; validate on the GS bench after deploying (`./deploy/gs/deploy.sh`):

1. **Two `:8103` clients** — confirm wfb-ng's stats server serves both the ConnectionMonitor and the dynlink controller simultaneously (check `/status` shows `connection.state` advancing AND dynlink emitting). If the second client is refused, fall back to one shared fan-out stats reader feeding both.
2. **Connect/disconnect timing** — power the drone on: `/status` `connection.state` should go `disconnected → connected` within a few seconds; power it off: `→ disconnected` with `reason: "tunnel_lost"` within ~`tunnelStaleS`.
3. **Idle-tunnel stability** — with a healthy but quiet link, confirm no spurious `tunnel_lost` (the heartbeat keeps the tunnel fresh; `tunnelStaleS > httpPollS`).
4. **Flight boundaries** — confirm a fresh flight file appears on reconnect and the prior flight is fsynced.
```
