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

from .config_build import build_aggregator, build_policy_config, resolve_profile
from .policy import Policy
from .return_link import ReturnLink
from .stats_client import RxEvent, SessionEvent, StatsClient
from .wire import Encoder as WireEncoder

log = logging.getLogger("fpvdgs.dynlink")


class _IdrRelay(asyncio.DatagramProtocol):
    """Bridge PixelPilot IDR/keyframe tokens to the drone.

    The local video player sends IDR tokens to 127.0.0.1:<idrPort> (wfb_rx
    relays RTP through localhost, so the player aims its requests there). We
    forward each datagram over the tunnel to the drone's idr_listen at
    <droneAddr>:<idrPort>. Replaces the standalone `socat` idr-forwarder that
    shipped with the old dynamic-link-gs service."""

    def __init__(self, dest):
        self._dest = dest          # (droneAddr, idrPort)
        self._transport = None

    def connection_made(self, transport):
        self._transport = transport

    def datagram_received(self, data, addr):
        if self._transport is not None:
            try:
                self._transport.sendto(data, self._dest)
            except OSError:
                pass               # drone momentarily unreachable — drop, keep relaying


class DynamicLinkController:
    def __init__(self, snapshot, *, stats_endpoint="tcp://127.0.0.1:8103",
                 stats_client_factory=StatsClient, probe_status=None):
        self._snapshot = dict(snapshot)
        self._stats_endpoint = stats_endpoint
        self._make_stats = stats_client_factory
        self._probe_status = probe_status
        self._lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None         # asyncio.Event, created in-loop
        self._started = threading.Event()
        self._status = {"running": False, "statsConnected": False,
                        "decision": None, "lastEmitMs": None, "emitSeq": 0,
                        "reason": "", "idrListen": None}

    # ---- thread-safe public API -----------------------------------------
    def start(self):
        with self._lifecycle:
            with self._lock:
                if self._thread and self._thread.is_alive():
                    return
                self._started.clear()
                self._thread = threading.Thread(target=self._thread_main,
                                                name="dl-controller", daemon=True)
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
                self._started.set()   # unblock start() even on early failure

    async def _run(self):
        with self._lock:
            snap = dict(self._snapshot)
        profile = resolve_profile(snap)
        policy = Policy(build_policy_config(snap), profile,
                        probe_status=self._probe_status)
        aggregator = build_aggregator(snap)
        return_link = ReturnLink(snap["droneAddr"], int(snap["dronePort"]))
        encoder = WireEncoder(seq=1)

        # IDR-token relay: 0.0.0.0:idrPort -> droneAddr:idrPort. Non-fatal if
        # the local port is taken (e.g. a leftover socat); the controller runs on.
        #
        # The listen address MUST be 0.0.0.0 (INADDR_ANY), never 127.0.0.1: we
        # reuse this same socket to forward each token on to the (non-loopback)
        # drone, and a socket bound to 127.0.0.1 cannot send off-loopback — the
        # sendto() fails with EINVAL, which _IdrRelay swallows, so every IDR
        # request gets dropped silently. INADDR_ANY still accepts the player's
        # loopback tokens and lets the kernel pick the source for the drone route.
        idr_transport = None
        self._set(idrListen=None)
        if snap.get("idrForward", True):
            idr_port = int(snap.get("idrPort", 11223))
            try:
                idr_transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
                    lambda: _IdrRelay((snap["droneAddr"], idr_port)),
                    local_addr=("0.0.0.0", idr_port))
                sa = idr_transport.get_extra_info("sockname")
                self._set(idrListen="%s:%d" % (sa[0], sa[1]) if sa else None)
            except OSError as e:
                log.warning("dl: IDR relay bind 0.0.0.0:%d failed: %s", idr_port, e)
                idr_transport = None

        # The wfb stats feed (:8103) interleaves rx records for every service
        # (video / mavlink / tunnel). Only the VIDEO stream may drive the policy:
        # the low/zero-rate uplink streams would trip link_starved and pin MCS at
        # the floor, and pollute the session/loss/fec signals. Match the video
        # stream by id substring (records are e.g. "video rx").
        video_id = (snap.get("videoStreamId") or "video").lower()

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

        try:
            await self._stats_loop(on_event)
        finally:
            policy.close()
            if idr_transport is not None:
                idr_transport.close()
            return_link.close()
            self._set(running=False, statsConnected=False, idrListen=None)

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
