"""GS-side probe measurement: spawn a FEC-off wfb_rx per probe radio_port,
parse each one's stdout for per-MCS PER/RSSI. Threaded asyncio, mirroring
DynamicLinkController. Observe-only; independent of the wfb-ng runner."""
from __future__ import annotations

import asyncio
import logging
import threading

from .parser import McsAggregator, parse_line

log = logging.getLogger("fpvdgs.probe")

WFB_RX = "/usr/bin/wfb_rx"


async def _default_spawn(cmd):
    return await asyncio.create_subprocess_exec(
        *cmd, stdout=asyncio.subprocess.PIPE, stderr=asyncio.subprocess.DEVNULL)


class ProbeController:
    def __init__(self, snapshot, *, spawn=None, ewma_alpha: float = 0.25):
        self._snap = dict(snapshot)
        self._spawn = spawn or _default_spawn   # cmd(list[str]) -> proc (await or sync in tests)
        self._alpha = ewma_alpha
        self._lock = threading.RLock()
        self._lifecycle = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None                 # asyncio.Event, created in-loop
        self._started = threading.Event()
        self._agg = McsAggregator(alpha=ewma_alpha)
        self._status = {"running": False, "streams": 0}

    # ---- thread-safe public API -----------------------------------------
    def start(self):
        with self._lifecycle:
            with self._lock:
                if self._thread and self._thread.is_alive():
                    return
                self._started.clear()
                self._thread = threading.Thread(target=self._thread_main,
                                                name="probe-controller", daemon=True)
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
                self._snap = dict(snapshot)
                self._agg = McsAggregator(alpha=self._alpha)
            # Restart unconditionally if it was running (mirrors the sibling
            # DynamicLinkController). A disabled snapshot just re-runs _run,
            # spawns nothing, reports running=False and parks — restarting on a
            # disable→enable round-trip would otherwise be impossible, since the
            # disabling set_config clears _thread.
            if running:
                self.start()

    def status(self):
        with self._lock:
            st = dict(self._status)
            mcs = {str(m): v for m, v in self._agg.snapshot().items()}
            enabled = bool(self._snap.get("enabled"))
        st["enabled"] = enabled
        st["mcs"] = mcs
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
            log.exception("probe controller loop crashed")
        finally:
            try:
                loop.close()
            finally:
                with self._lock:
                    self._loop = None
                    self._status.update(running=False, streams=0)
                self._started.set()   # unblock start() even on early failure

    def _build_cmd(self, port: int, sink: int) -> list[str]:
        # wfb_rx (rx.cpp getopt "K:fa:c:u:U:p:l:i:e:R:s:") — -l is the log_interval.
        snap = self._snap
        return [WFB_RX, "-K", str(snap["key"]), "-i", str(snap["linkId"]),
                "-p", str(port), "-c", "127.0.0.1", "-u", str(sink),
                "-l", str(snap.get("rxL", 50)), *list(snap["wlans"])]

    async def _read_stream(self, proc):
        cur_mcs = None
        while True:
            line = await proc.stdout.readline()
            if not line:
                break
            ev = parse_line(line.decode("utf-8", "replace"))
            if ev is None:
                continue
            kind, d = ev
            if kind == "RX_ANT":
                cur_mcs = d["mcs"]
                with self._lock:
                    self._agg.on_rx_ant(d["mcs"], d["rssi"], d["snr"])
            elif kind == "PKT" and cur_mcs is not None:
                with self._lock:
                    self._agg.on_pkt(cur_mcs, d["data"], d["lost"])

    async def _run(self):
        self._stop_event = asyncio.Event()
        snap = self._snap
        procs, tasks = [], []
        enabled = bool(snap.get("enabled"))
        try:
            if enabled:
                base, n = int(snap["basePort"]), int(snap.get("maxStreams", 4))
                for i in range(n):
                    cmd = self._build_cmd(base + i, 7000 + i)   # 7000+i = throwaway sink (discarded)
                    res = self._spawn(cmd)
                    proc = await res if asyncio.iscoroutine(res) else res
                    # append BEFORE creating the read task so a spawn failure on a
                    # later stream still finds this proc in `procs` for cleanup.
                    procs.append(proc)
                    tasks.append(asyncio.ensure_future(self._read_stream(proc)))
            # Observe-only and disabled-aware: a disabled controller spawns
            # nothing and reports running=False (it still parks on stop_event so
            # stop() can join the thread cleanly).
            self._set(running=enabled, streams=len(procs))
            self._started.set()
            await self._stop_event.wait()
        finally:
            # Runs on the normal stop path AND on a mid-loop spawn failure, so
            # already-spawned wfb_rx procs are never orphaned holding radio_ports.
            self._set(running=False, streams=0)
            self._started.set()   # ensure start() never hangs on a spawn failure
            for p in procs:
                try:
                    p.kill()
                except Exception:
                    pass
            for t in tasks:
                t.cancel()
            for p in procs:
                try:
                    await p.wait()
                except Exception:
                    pass
