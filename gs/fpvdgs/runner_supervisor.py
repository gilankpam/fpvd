"""Spawn, monitor, and restart a child process.

A background watcher thread auto-restarts the child if it exits unexpectedly,
with a crash-loop fault guard. Operator-initiated restarts (start/stop/restart)
do NOT count toward the crash-loop budget and clear any prior fault.

ProcessSupervisor is generic: parameterized by an argv (swappable at runtime
via set_argv), an extra-env dict, and a readiness strategy:
  - probe  : ready as soon as ready_check() is True before the timeout; a
             timeout (or early exit) is a failed start.
  - settle : ready iff the process is still alive at the end of the timeout
             window (ready_check=None, ready_on_timeout=True); an early exit is
             a failed start. (pixelpilot: no port to probe.)
"""

import os
import signal
import subprocess
import threading
import time


def _wfb_nics() -> list[str]:
    out = subprocess.run(["wfb-nics"], capture_output=True, text=True, check=True)
    return out.stdout.split()


def resolve_wlans(cfg: dict) -> list[str]:
    """Thin shim over the card model: every existing caller (status,
    beamforming, retune) wants only LOCAL ifaces, same as before Phase 2's
    remote cards. Local import avoids a circular import (wfb.cards falls
    back to this module's `_wfb_nics` when no detector is passed)."""
    from .wfb.cards import local_ifaces, resolve_cards

    return local_ifaces(resolve_cards(cfg, nic_detector=_wfb_nics))


class ProcessSupervisor:
    def __init__(
        self,
        argv,
        env=None,
        ready_check=None,
        ready_timeout=10.0,
        ready_on_timeout=False,
        log_path=None,
        max_restarts=5,
        restart_window=60.0,
        poll_interval=0.5,
        backoff=0.5,
    ):
        self._argv_list = list(argv)
        self._extra_env = dict(env or {})
        self._ready_check = ready_check
        self.ready_timeout = ready_timeout
        self._ready_on_timeout = ready_on_timeout
        self.log_path = log_path
        self.max_restarts = max_restarts
        self.restart_window = restart_window
        self.poll_interval = poll_interval
        self.backoff = backoff

        self._proc = None
        self._log_fh = None
        self._restarts = 0  # operator-initiated restarts (visibility)
        self._auto_restarts = 0  # watcher-initiated (crash) restarts
        self._last_exit = None
        self._fault = False
        self._recent = []  # timestamps of crash auto-restarts (budget)
        self._supervise = False  # watcher resurrects only while True
        self._watcher = None
        self._stop_evt = threading.Event()
        self._lock = threading.RLock()

    # ---- runtime argv -----------------------------------------------------
    def set_argv(self, argv):
        with self._lock:
            self._argv_list = list(argv)

    def set_env(self, env):
        with self._lock:
            self._extra_env = dict(env or {})

    # ---- process plumbing -------------------------------------------------
    def _env(self):
        env = dict(os.environ)
        env.update(self._extra_env)
        return env

    def _spawn(self):
        self._log_fh = open(self.log_path, "ab") if self.log_path else None
        try:
            self._proc = subprocess.Popen(
                self._argv_list,
                env=self._env(),
                stdout=(self._log_fh or subprocess.DEVNULL),
                stderr=subprocess.STDOUT,
                start_new_session=True,
            )
        except OSError:
            self._close_log()
            self._proc = None
            raise

    def _wait_ready(self):
        deadline = time.monotonic() + self.ready_timeout
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                self._last_exit = self._proc.returncode
                return False
            if self._ready_check is not None and self._ready_check():
                return True
            time.sleep(0.2)
        return self._ready_on_timeout

    def _spawn_and_wait(self):
        try:
            self._spawn()
        except OSError as e:
            self._last_exit = e.errno if e.errno is not None else -1
            return False
        return self._wait_ready()

    def _close_log(self):
        if self._log_fh is not None:
            self._log_fh.close()
            self._log_fh = None

    def _kill_locked(self):
        if not self._proc:
            return
        if self._proc.poll() is None:
            try:
                os.killpg(self._proc.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                self._proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(self._proc.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                self._proc.wait()
        self._last_exit = self._proc.returncode
        self._proc = None
        self._close_log()

    # ---- watcher ----------------------------------------------------------
    def _ensure_watcher(self):
        if self._watcher is None or not self._watcher.is_alive():
            self._stop_evt.clear()
            self._watcher = threading.Thread(target=self._watch_loop, daemon=True)
            self._watcher.start()

    def _watch_loop(self):
        while not self._stop_evt.wait(self.poll_interval):
            with self._lock:
                if not self._supervise or self._proc is None:
                    continue
                if self._proc.poll() is None:
                    continue
                self._on_crash_locked()

    def _on_crash_locked(self):
        # the child exited unexpectedly while we were supervising it
        self._last_exit = self._proc.returncode
        self._proc = None
        self._close_log()
        now = time.monotonic()
        self._recent = [t for t in self._recent if now - t < self.restart_window]
        self._recent.append(now)
        if len(self._recent) > self.max_restarts:
            self._fault = True
            self._supervise = False
            return
        self._auto_restarts += 1
        time.sleep(self.backoff)
        self._spawn_and_wait()

    # ---- operator API -----------------------------------------------------
    def start(self):
        with self._lock:
            self._fault = False
            self._recent = []
            ok = self._spawn_and_wait()
            self._supervise = True
            self._ensure_watcher()
            return ok

    def stop(self):
        with self._lock:
            self._supervise = False
            self._kill_locked()

    def restart(self, config=None):
        # `config` is accepted for interface parity with WfbEngine.restart but
        # ignored: this supervisor's child re-reads the rendered cfg file, which
        # api._apply_gs writes from the pending config before the restart.
        with self._lock:
            self.stop()
            self._restarts += 1
            return self.start()

    def shutdown(self):
        self._stop_evt.set()
        self.stop()
        w = self._watcher
        if w is not None and w is not threading.current_thread():
            w.join(timeout=2)
        self._watcher = None

    def state(self):
        with self._lock:
            running = bool(self._proc and self._proc.poll() is None)
            return {
                "running": running,
                "pid": self._proc.pid if running else None,
                "restarts": self._restarts,
                "autoRestarts": self._auto_restarts,
                "lastExit": self._last_exit,
                "fault": self._fault,
            }
