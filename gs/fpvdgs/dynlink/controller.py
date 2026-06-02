# gs/fpvdgs/dynlink/controller.py
"""In-process GS dynamic-link controller.

Owns one daemon thread running an asyncio event loop: stats client →
SignalAggregator → Policy → wire encode → ReturnLink, plus the P4a HELLO
listener. Thread-safe surface for the (thread-based) supervisor:
start/stop/set_config/status. A config change while running rebuilds the
loop from the new snapshot; the wfb runner is never touched."""
from __future__ import annotations

import asyncio
import logging
import threading
import time

from .config_build import build_aggregator, build_policy_config, resolve_profile
from .drone_config import DroneConfigState
from .policy import Policy
from .return_link import ReturnLink
from .stats_client import RxEvent, SessionEvent, StatsClient
from .tunnel_listener import TunnelListener
from .wire import Encoder as WireEncoder, encode_hello_ack

log = logging.getLogger("fpvdgs.dynlink")


class DynamicLinkController:
    def __init__(self, snapshot, *, stats_endpoint="tcp://127.0.0.1:8103",
                 gs_listen_addr="0.0.0.0", gs_listen_port=5801,
                 stats_client_factory=StatsClient):
        self._snapshot = dict(snapshot)
        self._stats_endpoint = stats_endpoint
        self._gs_listen = (gs_listen_addr, gs_listen_port)
        self._make_stats = stats_client_factory
        self._lock = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None         # asyncio.Event, created in-loop
        self._started = threading.Event()
        self._status = {"running": False, "statsConnected": False,
                        "decision": None, "lastEmitMs": None, "emitSeq": 0,
                        "reason": "", "hello": "none"}

    # ---- thread-safe public API -----------------------------------------
    def start(self):
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._started.clear()
            self._thread = threading.Thread(target=self._thread_main,
                                            name="dl-controller", daemon=True)
            self._thread.start()
        self._started.wait(timeout=5.0)

    def stop(self):
        with self._lock:
            loop, stop, thread = self._loop, self._stop_event, self._thread
        if loop is not None and stop is not None:
            try:
                loop.call_soon_threadsafe(stop.set)
            except RuntimeError:
                pass  # loop already closed/closing — teardown in progress
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)
        with self._lock:
            self._thread = None

    def set_config(self, snapshot):
        """Apply a new snapshot. If running, rebuild the loop (stop+start)
        from the new config — the wfb runner is untouched."""
        running = self._thread is not None and self._thread.is_alive()
        if running:
            self.stop()
        with self._lock:
            self._snapshot = dict(snapshot)
        if running:
            self.start()

    def status(self):
        with self._lock:
            st = dict(self._status)
            st["decision"] = dict(st["decision"]) if st["decision"] else None
            return st

    # ---- internals ------------------------------------------------------
    def _set(self, **kw):
        with self._lock:
            self._status.update(kw)

    def _thread_main(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        with self._lock:
            self._loop = loop
        try:
            loop.run_until_complete(self._run())
        except Exception:
            log.exception("dl-controller loop crashed")
        finally:
            try:
                loop.close()
            finally:
                with self._lock:
                    self._loop = None
                    self._status.update(running=False, statsConnected=False)
                self._started.set()   # unblock start() even on early failure

    async def _run(self):
        with self._lock:
            snap = dict(self._snapshot)
        profile = resolve_profile(snap)
        drone_cfg = DroneConfigState()
        policy = Policy(build_policy_config(snap), profile, drone_config=drone_cfg)
        aggregator = build_aggregator(snap)
        return_link = ReturnLink(snap["droneAddr"], int(snap["dronePort"]))
        encoder = WireEncoder(seq=1)

        def _on_hello(h):
            drone_cfg.on_hello(h)
            ack = drone_cfg.build_ack()
            if ack is not None:
                return_link.send_hello_ack(encode_hello_ack(ack))
                self._set(hello="acked")

        listener = TunnelListener(self._gs_listen[0], self._gs_listen[1],
                                  on_pong=None, on_hello=_on_hello)
        try:
            await listener.start()
        except OSError as e:
            log.warning("dl: HELLO listener bind %s failed: %s", self._gs_listen, e)
            listener = None

        def on_event(ev):
            if isinstance(ev, SessionEvent):
                aggregator.update_session(ev.session)
                return
            if isinstance(ev, RxEvent):
                signals = aggregator.consume(ev)
                decision = policy.tick(signals)
                return_link.send(encoder.encode(decision))
                self._set(
                    decision={"mcs": decision.mcs, "k": decision.k,
                              "n": decision.n, "depth": decision.depth,
                              "txpowerDbm": decision.tx_power_dBm,
                              "bitrateKbps": decision.bitrate_kbps},
                    reason=decision.reason,
                    lastEmitMs=int(time.monotonic() * 1000),
                    emitSeq=self._status["emitSeq"] + 1,
                )

        self._stop_event = asyncio.Event()
        self._set(running=True)
        self._started.set()

        try:
            await self._stats_loop(on_event)
        finally:
            if listener is not None:
                listener.stop()
            return_link.close()
            self._set(running=False, statsConnected=False)

    async def _stats_loop(self, on_event):
        """Run the stats client, reconnecting across runner bounces until
        stop is requested."""
        while not self._stop_event.is_set():
            client = self._make_stats(self._stats_endpoint, on_event)
            run_task = asyncio.ensure_future(client.run())
            stop_task = asyncio.ensure_future(self._stop_event.wait())
            self._set(statsConnected=True)
            try:
                await asyncio.wait({run_task, stop_task},
                                   return_when=asyncio.FIRST_COMPLETED)
            finally:
                client.stop()
                for t in (run_task, stop_task):
                    t.cancel()
                self._set(statsConnected=False)
            if self._stop_event.is_set():
                break
            try:
                await asyncio.wait_for(self._stop_event.wait(), timeout=1.0)
            except asyncio.TimeoutError:
                pass   # backoff elapsed → reconnect
