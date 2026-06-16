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
