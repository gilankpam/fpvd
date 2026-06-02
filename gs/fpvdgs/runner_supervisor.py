"""Spawn, monitor, and restart the wfb-runner child process.

A background watcher thread auto-restarts the runner if it exits unexpectedly,
with a crash-loop fault guard. Operator-initiated restarts (start/stop/restart,
driven by API applies) do NOT count toward the crash-loop budget and clear any
prior fault.
"""

import os
import signal
import socket
import subprocess
import threading
import time


def _wfb_nics() -> list[str]:
    out = subprocess.run(["wfb-nics"], capture_output=True, text=True, check=True)
    return out.stdout.split()


def resolve_wlans(cfg: dict) -> list[str]:
    wlans = cfg.get("link", {}).get("wlans", "auto")
    if wlans == "auto" or wlans is None:
        return _wfb_nics()
    return list(wlans)


def _port_open(port: int, host: str = "127.0.0.1") -> bool:
    try:
        with socket.create_connection((host, port), timeout=0.3):
            return True
    except OSError:
        return False


class RunnerSupervisor:
    def __init__(self, runner_cmd, cfg_out, profile, wlans, ready_port=8103,
                 ready_timeout=10.0, log_path=None, max_restarts=5,
                 restart_window=60.0, poll_interval=0.5, backoff=0.5):
        self.runner_cmd = runner_cmd
        self.cfg_out = cfg_out
        self.profile = profile
        self.wlans = wlans
        self.ready_port = ready_port
        self.ready_timeout = ready_timeout
        self.log_path = log_path
        self.max_restarts = max_restarts
        self.restart_window = restart_window
        self.poll_interval = poll_interval
        self.backoff = backoff

        self._proc = None
        self._log_fh = None
        self._restarts = 0          # operator-initiated restarts (visibility)
        self._auto_restarts = 0     # watcher-initiated (crash) restarts
        self._last_exit = None
        self._fault = False
        self._recent = []           # timestamps of crash auto-restarts (budget)
        self._supervise = False     # watcher resurrects only while True
        self._watcher = None
        self._stop_evt = threading.Event()
        self._lock = threading.RLock()

    # ---- process plumbing -------------------------------------------------
    def _argv(self):
        return list(self.runner_cmd) + ["--profiles", self.profile, "--wlans", *self.wlans]

    def _env(self):
        env = dict(os.environ)
        env["WIFIBROADCAST_CFG"] = self.cfg_out
        return env

    def _spawn(self):
        self._log_fh = open(self.log_path, "ab") if self.log_path else None
        self._proc = subprocess.Popen(self._argv(), env=self._env(),
                                      stdout=(self._log_fh or subprocess.DEVNULL),
                                      stderr=subprocess.STDOUT,
                                      start_new_session=True)

    def _wait_ready(self):
        deadline = time.monotonic() + self.ready_timeout
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                self._last_exit = self._proc.returncode
                return False
            if _port_open(self.ready_port):
                return True
            time.sleep(0.2)
        return False

    def _spawn_and_wait(self):
        self._spawn()
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
        # the runner exited unexpectedly while we were supervising it
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

    def restart(self):
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
