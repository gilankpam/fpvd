"""asyncio subprocess supervisor for one wfb_rx/wfb_tx child.

Asyncio-native sibling of `fpvdgs.runner_supervisor.ProcessSupervisor`: same
crash-loop-budget / group-kill semantics, but tasks instead of a watcher
thread, and no locks (everything below runs on the engine's single event
loop, so there is no cross-thread mutation to guard).

Lifecycle
---------
`start()` spawns the child, pumps its stdout into a fresh line parser
(`RxLineParser`/`TxLineParser`, chosen by `spec.kind`) wired to the
`StatsHub`, and for `kind == "tx"` waits for the LISTEN handshake before
returning. A `tx` child that never completes the handshake within
`ready_timeout` is killed and treated as a failed start — no crash is
recorded and no restart loop is engaged (the caller decides whether to
retry). A child that dies *on its own* while starting (rx: exits before we
can observe it alive; tx: exits before handshaking) is left in place and
handed to the same crash watch that covers a later runtime crash, mirroring
`ProcessSupervisor.start()` always arming its watcher regardless of the
readiness result.

Once running, an unexpected exit triggers: backoff sleep -> respawn, with a
sliding-window crash-loop budget (`max_restarts` crashes within
`restart_window` seconds). Exceeding the budget sets `fault=True`, calls
`on_fault(self)` once, and stops supervising (no more auto-restarts).
`stop()` is an operator action: it cancels the watch task first, so a
deliberate stop is never counted as a crash.

TX RESPAWN NOTE (read by Task 11's engine wiring): every (re)spawn builds a
*brand-new* `TxLineParser` instance (`self._parser`), because a restarted
wfb_tx re-advertises its unix sockets/ports from scratch over stdout and a
reused parser could keep serving stale socket names from the previous
process incarnation. This means `tx_parser` is a different object after
every restart, starting out empty (`unix_sockets == {}`) until the new
handshake completes. The engine must not cache `tx_parser.unix_sockets`
across a restart — it should re-read `child.tx_parser.unix_sockets` (and
`.ports` / `.control_port` if used) each time it needs them, and in
particular after observing `autoRestarts` increment or `running` flip back
to True for a tx child.
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import os
import signal
import time
from collections.abc import Callable

from .aggregator import StatsHub
from .graph import ServiceSpec
from .lineproto import RxLineParser, TxLineParser

log = logging.getLogger("fpvdgs.wfb")

KILL_TIMEOUT_S = 5.0


class WfbChild:
    def __init__(
        self,
        spec: ServiceSpec,
        hub: StatsHub,
        *,
        ready_timeout: float = 10.0,
        max_restarts: int = 5,
        restart_window: float = 60.0,
        backoff: float = 0.5,
        on_fault: Callable[["WfbChild"], None] | None = None,
    ):
        self.spec = spec
        self._hub = hub
        self.ready_timeout = ready_timeout
        self.max_restarts = max_restarts
        self.restart_window = restart_window
        self.backoff = backoff
        self._on_fault = on_fault

        self._proc: asyncio.subprocess.Process | None = None
        self._parser: RxLineParser | TxLineParser | None = None
        self._pump_task: asyncio.Task | None = None
        self._watch_task: asyncio.Task | None = None

        self._restarts = 0  # operator-initiated (visibility only; no restart() API here)
        self._auto_restarts = 0  # watcher-initiated (crash) restarts
        self._last_exit: int | None = None
        self._fault = False
        self._recent: list[float] = []  # monotonic timestamps of crash auto-restarts
        self._supervise = False

    # -- public: engine reads this after start() (see TX RESPAWN NOTE) ------
    @property
    def tx_parser(self) -> TxLineParser | None:
        return self._parser

    # -- lifecycle ------------------------------------------------------------
    async def start(self) -> bool:
        self._fault = False
        self._recent = []
        ok = await self._spawn_and_wait()
        if self._proc is not None:
            # Either genuinely ready, or died on its own while starting —
            # both are handed to the crash watch (mirrors ProcessSupervisor,
            # which arms its watcher unconditionally). A deliberate kill
            # (tx handshake timeout) clears self._proc first, so it is
            # excluded here.
            self._supervise = True
            self._watch_task = asyncio.ensure_future(self._watch())
        return ok

    async def stop(self) -> None:
        self._supervise = False
        if self._watch_task is not None:
            self._watch_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._watch_task
            self._watch_task = None
        await self._kill_process()
        if self._pump_task is not None:
            pump_task, self._pump_task = self._pump_task, None
            try:
                await asyncio.wait_for(pump_task, timeout=1.0)
            except (TimeoutError, asyncio.TimeoutError):
                pump_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await pump_task

    def state(self) -> dict:
        running = bool(self._proc and self._proc.returncode is None)
        return {
            "running": running,
            "pid": self._proc.pid if running else None,
            "restarts": self._restarts,
            "autoRestarts": self._auto_restarts,
            "lastExit": self._last_exit,
            "fault": self._fault,
        }

    # -- spawn + readiness ------------------------------------------------------
    def _make_parser(self) -> RxLineParser | TxLineParser:
        if self.spec.kind == "tx":
            return TxLineParser(self.spec.name, self._hub.update_tx_stats)
        return RxLineParser(
            self.spec.name, self._hub.update_rx_stats, self._hub.process_new_session
        )

    async def _spawn_and_wait(self) -> bool:
        self._parser = self._make_parser()
        try:
            self._proc = await asyncio.create_subprocess_exec(
                *self.spec.argv,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError as e:
            self._proc = None
            self._last_exit = e.errno if e.errno is not None else -1
            log.warning("wfb child %s: spawn failed: %s", self.spec.name, e)
            return False

        if self._pump_task is not None and not self._pump_task.done():
            # A respawn overwrites _pump_task below; cancel the previous
            # incarnation first so it doesn't linger as an orphaned,
            # never-awaited task ("Task was destroyed but it is pending").
            self._pump_task.cancel()
        self._pump_task = asyncio.ensure_future(self._pump_stdout(self._proc, self._parser))

        if self.spec.kind == "tx":
            return await self._wait_tx_ready()
        return self._check_rx_ready()

    def _check_rx_ready(self) -> bool:
        # rx has no handshake: ready as soon as it is spawned. Only a
        # same-tick exit (already reaped by the time we check) is a failed
        # start; self._proc is left set so start() still arms the watch.
        if self._proc.returncode is not None:
            self._last_exit = self._proc.returncode
            return False
        return True

    async def _wait_tx_ready(self) -> bool:
        loop = asyncio.get_running_loop()
        handshake: asyncio.Future = loop.create_future()

        def _on_handshake() -> None:
            if not handshake.done():
                handshake.set_result(True)

        self._parser.on_handshake = _on_handshake

        exit_task = asyncio.ensure_future(self._proc.wait())
        try:
            done, _pending = await asyncio.wait(
                {handshake, exit_task},
                timeout=self.ready_timeout,
                return_when=asyncio.FIRST_COMPLETED,
            )
            if handshake in done:
                return True
            if exit_task in done:
                # died on its own before handshaking -> not a deliberate
                # kill; leave self._proc set so start() arms the watch.
                self._last_exit = exit_task.result()
                return False
            # timed out, still alive: this is a deliberate kill, not a
            # crash — no restart budget is spent on it.
            log.warning(
                "wfb child %s: tx handshake timed out after %.1fs",
                self.spec.name,
                self.ready_timeout,
            )
            await self._kill_process()
            return False
        finally:
            if not exit_task.done():
                exit_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await exit_task

    async def _pump_stdout(self, proc: asyncio.subprocess.Process, parser) -> None:
        try:
            while True:
                line = await proc.stdout.readline()
                if not line:
                    return
                parser.feed_line(line.decode(errors="replace"))
        except asyncio.CancelledError:
            raise
        except Exception:
            log.exception("wfb child %s: stdout pump crashed", self.spec.name)

    # -- kill -------------------------------------------------------------------
    async def _kill_process(self) -> None:
        proc = self._proc
        if proc is None:
            return
        if proc.returncode is None:
            try:
                os.killpg(proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(proc.wait(), timeout=KILL_TIMEOUT_S)
            except (TimeoutError, asyncio.TimeoutError):
                try:
                    os.killpg(proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                await proc.wait()
        self._last_exit = proc.returncode
        self._proc = None

    # -- crash watch --------------------------------------------------------
    async def _watch(self) -> None:
        """Invariant: this loop may only end in one of two terminal states —
        supervising a live process (a `return` with `self._proc` set and a
        later iteration awaiting it), or `fault=True` with `on_fault` fired
        (or an operator `stop()`, which cancels this task outright).
        `running=False, fault=False` with the watch task dead must be
        unreachable — a failed respawn is charged against the same
        crash-loop budget as a live crash instead of returning silently.
        """
        while True:
            proc = self._proc
            if proc is None:
                return
            rc = await proc.wait()
            if not self._supervise:
                return  # stop() already handled the kill; not a crash

            self._last_exit = rc

            # Keep attempting a respawn while under the crash-loop budget.
            # A failed respawn attempt (spawn OSError, or a tx handshake
            # timeout on the retry — self._proc left None) loops back here
            # and is charged against the same budget as a live crash; only
            # a successful respawn (break) or an exhausted budget (return)
            # leaves this inner loop.
            while True:
                now = time.monotonic()
                self._recent = [t for t in self._recent if now - t < self.restart_window]
                self._recent.append(now)
                if len(self._recent) > self.max_restarts:
                    self._fault = True
                    self._supervise = False
                    self._proc = None
                    if self._on_fault is not None:
                        try:
                            self._on_fault(self)
                        except Exception:
                            log.exception("wfb child %s: on_fault callback failed", self.spec.name)
                    return

                self._auto_restarts += 1
                await asyncio.sleep(self.backoff)
                if not self._supervise:
                    return

                await self._spawn_and_wait()
                if self._proc is not None:
                    break  # respawn succeeded; resume watching it above
