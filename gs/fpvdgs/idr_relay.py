"""Always-on IDR/keyframe relay: PixelPilot -> drone encoder.

PixelPilot sends IDR (keyframe) request tokens to the GS on 0.0.0.0:11223;
this relay forwards each datagram over the tunnel to the drone's idr_listen at
<droneHost>:11223. It is standing GS data-plane infrastructure, decoupled from
the adaptive-link controller, so keyframe forwarding works on static *and*
adaptive links. Replaces the standalone socat idr-forwarder that shipped with
the old dynamic-link-gs service.

The listen address MUST be 0.0.0.0 (INADDR_ANY), never 127.0.0.1: the same
socket is reused to forward each token to the (non-loopback) drone, and a socket
bound to 127.0.0.1 cannot send off-loopback (sendto fails EINVAL, which the
relay swallows -> every IDR request silently dropped). INADDR_ANY still accepts
the player's loopback tokens and lets the kernel pick the drone-route source."""

from __future__ import annotations

import asyncio
import logging
import threading

log = logging.getLogger("fpvdgs.idr_relay")

IDR_PORT = 11223


class _IdrRelay(asyncio.DatagramProtocol):
    """Forward every received datagram to a fixed drone destination."""

    def __init__(self, dest):
        self._dest = dest  # (droneHost, IDR_PORT)
        self._transport = None

    def connection_made(self, transport):
        self._transport = transport

    def datagram_received(self, data, addr):
        if self._transport is not None:
            try:
                self._transport.sendto(data, self._dest)
            except OSError:
                pass  # drone momentarily unreachable — drop, keep relaying


class IdrRelay:
    """Always-on owner of the IDR relay: a daemon thread running an asyncio loop
    that binds 0.0.0.0:<port> and forwards to (drone_host, <port>) until stopped.
    Thread-safe start/stop/status for the (thread-based) supervisor."""

    def __init__(self, drone_host, *, port=IDR_PORT):
        self._dest = (drone_host, port)
        self._port = port
        self._lock = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None  # asyncio.Event, created in-loop
        self._started = threading.Event()
        self._status = {"running": False, "listen": None}

    # ---- thread-safe public API -----------------------------------------
    def start(self):
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._started.clear()
            self._thread = threading.Thread(target=self._thread_main, name="idr-relay", daemon=True)
            self._thread.start()
        self._started.wait(timeout=5.0)

    def stop(self):
        with self._lock:
            loop, stop, thread = self._loop, self._stop_event, self._thread
        if loop is not None and stop is not None:
            try:
                loop.call_soon_threadsafe(stop.set)
            except RuntimeError:
                pass  # loop already closed/closing
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)
        with self._lock:
            self._thread = None

    def status(self):
        with self._lock:
            return dict(self._status)

    # ---- internals ------------------------------------------------------
    def _thread_main(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        with self._lock:
            self._loop = loop
        try:
            loop.run_until_complete(self._run())
        except Exception:
            log.exception("idr-relay loop crashed")
        finally:
            try:
                loop.close()
            finally:
                with self._lock:
                    self._loop = None
                    self._status.update(running=False, listen=None)
                self._started.set()  # unblock start() even on early failure

    async def _run(self):
        self._stop_event = asyncio.Event()
        transport = None
        try:
            transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
                lambda: _IdrRelay(self._dest), local_addr=("0.0.0.0", self._port)
            )
            sa = transport.get_extra_info("sockname")
            with self._lock:
                self._status.update(running=True, listen="%s:%d" % (sa[0], sa[1]) if sa else None)
        except OSError as e:
            log.warning("idr-relay bind 0.0.0.0:%d failed: %s", self._port, e)
            with self._lock:
                self._status.update(running=False, listen=None)
        self._started.set()
        try:
            await self._stop_event.wait()
        finally:
            if transport is not None:
                transport.close()
            with self._lock:
                self._status.update(running=False, listen=None)
