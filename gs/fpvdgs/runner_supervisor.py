"""Spawn, monitor, and restart the wfb-runner child process."""

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
    def __init__(self, runner_cmd: list[str], cfg_out: str, profile: str,
                 wlans: list[str], ready_port: int = 8103,
                 ready_timeout: float = 10.0, log_path: str | None = None,
                 max_restarts: int = 5, restart_window: float = 60.0):
        self.runner_cmd = runner_cmd
        self.cfg_out = cfg_out
        self.profile = profile
        self.wlans = wlans
        self.ready_port = ready_port
        self.ready_timeout = ready_timeout
        self.log_path = log_path
        self.max_restarts = max_restarts
        self.restart_window = restart_window

        self._proc: subprocess.Popen | None = None
        self._log_fh = None
        self._restarts = 0
        self._last_exit: int | None = None
        self._fault = False
        self._recent: list[float] = []
        self._lock = threading.RLock()

    def _argv(self) -> list[str]:
        return list(self.runner_cmd) + ["--profiles", self.profile, "--wlans", *self.wlans]

    def _env(self) -> dict:
        env = dict(os.environ)
        env["WIFIBROADCAST_CFG"] = self.cfg_out
        return env

    def _spawn(self) -> None:
        self._log_fh = open(self.log_path, "ab") if self.log_path else None
        self._proc = subprocess.Popen(self._argv(), env=self._env(),
                                      stdout=(self._log_fh or subprocess.DEVNULL),
                                      stderr=subprocess.STDOUT,
                                      start_new_session=True)

    def _wait_ready(self) -> bool:
        deadline = time.monotonic() + self.ready_timeout
        while time.monotonic() < deadline:
            if self._proc.poll() is not None:
                self._last_exit = self._proc.returncode
                return False
            if _port_open(self.ready_port):
                return True
            time.sleep(0.2)
        return False

    def start(self) -> bool:
        with self._lock:
            self._spawn()
            return self._wait_ready()

    def stop(self) -> None:
        with self._lock:
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
            if self._log_fh is not None:
                self._log_fh.close()
                self._log_fh = None

    def restart(self) -> bool:
        with self._lock:
            self.stop()
            self._restarts += 1
            now = time.monotonic()
            self._recent = [t for t in self._recent if now - t < self.restart_window]
            self._recent.append(now)
            if len(self._recent) > self.max_restarts:
                self._fault = True
                return False
            return self.start()

    def state(self) -> dict:
        with self._lock:
            running = bool(self._proc and self._proc.poll() is None)
            return {
                "running": running,
                "pid": self._proc.pid if running else None,
                "restarts": self._restarts,
                "lastExit": self._last_exit,
                "fault": self._fault,
            }
