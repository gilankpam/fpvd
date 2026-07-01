# gs/fpvdgs/dynlink/controller.py
"""In-process GS dynamic-link controller.

Owns one daemon thread running an asyncio event loop: stats client →
SignalAggregator → Policy → wire encode → ReturnLink. Thread-safe
surface for the (thread-based) supervisor: start/stop/set_config/status.
A config change while running rebuilds the loop from the new snapshot;
the wfb runner is never touched."""

from __future__ import annotations

import asyncio
import logging
import threading
import time

from ..events import DRONE_CONNECTED, DRONE_DISCONNECTED
from .config_build import build_aggregator, build_policy_config
from .policy import Policy
from .return_link import ReturnLink
from .stats_client import RxEvent, SessionEvent, StatsClient
from .wire import Encoder as WireEncoder

log = logging.getLogger("fpvdgs.dynlink")


class DynamicLinkController:
    def __init__(
        self,
        snapshot,
        *,
        stats_endpoint="tcp://127.0.0.1:8103",
        stats_client_factory=StatsClient,
        probe_status=None,
        bus=None,
    ):
        self._snapshot = dict(snapshot)
        self._stats_endpoint = stats_endpoint
        self._make_stats = stats_client_factory
        self._probe_status = probe_status
        self._bus = bus
        self._policy = None
        self._aggregator = None
        self._pending_cal = None
        self._lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None  # asyncio.Event, created in-loop
        self._started = threading.Event()
        self._status = {
            "running": False,
            "statsConnected": False,
            "decision": None,
            "lastEmitMs": None,
            "emitSeq": 0,
            "reason": "",
        }
        if bus is not None:
            bus.subscribe(DRONE_CONNECTED, self._on_drone_connected)
            bus.subscribe(DRONE_DISCONNECTED, self._on_drone_disconnected)

    # ---- thread-safe public API -----------------------------------------
    def start(self):
        with self._lifecycle:
            with self._lock:
                if self._thread and self._thread.is_alive():
                    return
                self._started.clear()
                self._thread = threading.Thread(
                    target=self._thread_main, name="dl-controller", daemon=True
                )
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
                    pass  # loop already closed/closing — teardown in progress
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=5.0)
            with self._lock:
                self._thread = None

    def set_config(self, snapshot):
        """Apply a new snapshot. If running, rebuild the loop (stop+start)
        from the new config — the wfb runner is untouched."""
        with self._lifecycle:
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
                self._started.set()  # unblock start() even on early failure

    async def _run(self):
        with self._lock:
            snap = dict(self._snapshot)
        policy = Policy(build_policy_config(snap), probe_status=self._probe_status)
        with self._lock:
            self._policy = policy
        aggregator = build_aggregator(snap)
        with self._lock:
            self._aggregator = aggregator
        return_link = ReturnLink(snap["droneAddr"], int(snap["dronePort"]))
        encoder = WireEncoder(seq=1)

        # The wfb stats feed (:8103) interleaves rx records for every service
        # (video / mavlink / tunnel). Only the VIDEO stream may drive the policy:
        # the low/zero-rate uplink streams would trip link_starved and pin MCS at
        # the floor, and pollute the session/loss/fec signals. Match the video
        # stream by id substring (records are e.g. "video rx").
        video_id = "video"  # frozen: the wfb video stream id

        def _is_video(ev):
            return ev.id is not None and video_id in ev.id.lower()

        def on_event(ev):
            if isinstance(ev, SessionEvent):
                if _is_video(ev):
                    aggregator.update_session(ev.session)
                return
            if isinstance(ev, RxEvent):
                if not _is_video(ev):
                    return
                signals = aggregator.consume(ev)
                decision = policy.tick(signals)
                return_link.send(encoder.encode(decision))
                self._set(
                    decision={"mcs": decision.mcs},
                    reason=decision.reason,
                    lastEmitMs=int(time.monotonic() * 1000),
                    emitSeq=self._status["emitSeq"] + 1,
                )

        self._stop_event = asyncio.Event()
        self._set(running=True)
        self._started.set()
        self._seed_from_cached_connection()

        try:
            await self._stats_loop(on_event)
        finally:
            policy.close()
            return_link.close()
            with self._lock:
                self._policy = None
                self._aggregator = None
            self._set(running=False, statsConnected=False)

    # ---- connection-event subscribers (called on the monitor's thread) ----
    def _marshal(self, fn):
        with self._lock:
            loop = self._loop
        if loop is None:
            return  # loop down (dynlink disabled/stopped)
        try:
            loop.call_soon_threadsafe(fn)
        except RuntimeError:
            pass  # loop tearing down

    def _seed_from_cached_connection(self):
        """Already-connected seed (called from _run, on the loop thread). If the
        drone connected before this loop started — e.g. dynamicLink toggled on
        mid-session while the link is already up — the DRONE_CONNECTED event
        already fired and won't repeat. Bind calibration from the bus's cached
        connection state so the curve + learned-prior key bind immediately,
        instead of running un-normalized until the next reconnect."""
        if self._bus is None:
            return
        st = self._bus.state("drone")
        if isinstance(st, dict) and st.get("state") == "connected":
            with self._lock:
                self._pending_cal = (st.get("drone") or {}).get("radio")
            self._connected_inloop()

    def _on_drone_connected(self, payload):
        radio = ((payload or {}).get("drone") or {}).get("radio")
        with self._lock:
            self._pending_cal = radio
        self._marshal(self._connected_inloop)

    def _on_drone_disconnected(self, payload):
        self._marshal(self._disconnected_inloop)

    def _connected_inloop(self):
        p = self._policy
        if p is None:
            return
        with self._lock:
            radio = self._pending_cal
        self._bind_calibration(radio)  # bind prior key before reset so first climb uses it
        p.reset_for_new_session()  # new session: reset selector, re-climb from start MCS via probe
        p.flightlog.begin_flight()  # start a fresh flight file
        log.info("dynlink: drone connected — bound calibration + began new flight")

    def _bind_calibration(self, radio):
        agg = self._aggregator
        adapter = (radio or {}).get("adapterId")
        # New session: clear stale smoothed signals. Raw SNR is the control axis
        # — no txpower-curve normalization (2026-07-02 spec).
        if agg is not None:
            agg.reset_smoothed()
        # Learned-prior key binds on a present adapter id + the channel width
        # (10 MHz and 20 MHz keep independent knees), independent of the curve.
        if adapter:
            width = int(self._snapshot.get("linkWidth", 20))
            self._policy.bind_learned_prior(str(adapter), width)

    def _disconnected_inloop(self):
        p = self._policy
        if p is None:
            return
        p.flightlog.sync()  # make the flight durable at the loss edge
        p.learned_prior.flush()  # persist the session's learning
        log.info("dynlink: drone disconnected — synced flight + flushed prior")

    async def _stats_loop(self, on_event):
        """Run the stats client, reconnecting across runner bounces until
        stop is requested."""
        while not self._stop_event.is_set():
            client = self._make_stats(self._stats_endpoint, on_event)
            run_task = asyncio.ensure_future(client.run())
            stop_task = asyncio.ensure_future(self._stop_event.wait())
            self._set(statsConnected=True)
            try:
                await asyncio.wait({run_task, stop_task}, return_when=asyncio.FIRST_COMPLETED)
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
                pass  # backoff elapsed → reconnect
