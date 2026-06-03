# PixelPilot Managed Service + Config API — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make fpvd-GS spawn/supervise the `pixelpilot` binary as a second managed child and model its GS-local launch knobs through the existing `/config` + `/apply`, applying changes by bouncing only PixelPilot.

**Architecture:** Generalize `RunnerSupervisor` into a reusable `ProcessSupervisor` (argv/env/readiness-strategy, runtime `set_argv`); wfb and PixelPilot become two instances. A new `pixelpilot.py` renderer turns the config block into the child argv (reproducing the current systemd `ExecStart`). `_apply_gs` bounces each subsystem (wfb / pixelpilot / dynamicLink) only when its own slice changed. Deploy retires the stock PixelPilot init script so fpvd is sole owner.

**Tech Stack:** Python 3 stdlib only (`subprocess`, `threading`, `socket`, `signal`), pytest. Spec: `docs/superpowers/specs/2026-06-03-pixelpilot-managed-service-design.md`.

**Test convention:** all commands run from the `gs/` directory; `from fpvdgs...` imports resolve there. Unit tests in `gs/tests/unit/`, integration in `gs/tests/integration/`, fixtures in `gs/tests/fixtures/`.

---

### Task 1: PixelPilot argv renderer

**Files:**
- Create: `gs/fpvdgs/pixelpilot.py`
- Test: `gs/tests/unit/test_pixelpilot_render.py`

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_pixelpilot_render.py`:

```python
from fpvdgs.pixelpilot import render_pixelpilot_argv

DEFAULTS = {
    "pixelpilot": {
        "enabled": True,
        "bin": "/usr/bin/pixelpilot",
        "configPath": "/etc/pixelpilot/pixelpilot.yaml",
        "screenMode": "1920x1080@60",
        "videoScale": 1.0,
        "osdConfigPath": "/etc/pixelpilot/config_osd.json",
        "dvrFramerate": 60,
        "dvrDir": "/var/dvr",
        "dvrTemplate": "record_%Y-%m-%d_%H-%M-%S.mp4",
        "extraArgs": [],
    }
}


def test_defaults_reproduce_execstart():
    assert render_pixelpilot_argv(DEFAULTS) == [
        "/usr/bin/pixelpilot",
        "--osd", "--osd-custom-message",
        "--osd-config", "/etc/pixelpilot/config_osd.json",
        "--screen-mode", "1920x1080@60",
        "--video-scale", "1.0",
        "--dvr-framerate", "60",
        "--dvr-fmp4", "--dvr-sequenced-files",
        "--dvr-template", "/var/dvr/record_%Y-%m-%d_%H-%M-%S.mp4",
        "--config", "/etc/pixelpilot/pixelpilot.yaml",
    ]


def test_knobs_are_reflected():
    cfg = {"pixelpilot": dict(DEFAULTS["pixelpilot"],
                              screenMode="1280x720@60", videoScale=1.5,
                              dvrFramerate=30,
                              osdConfigPath="/tmp/osd.json")}
    argv = render_pixelpilot_argv(cfg)
    assert argv[argv.index("--screen-mode") + 1] == "1280x720@60"
    assert argv[argv.index("--video-scale") + 1] == "1.5"
    assert argv[argv.index("--dvr-framerate") + 1] == "30"
    assert argv[argv.index("--osd-config") + 1] == "/tmp/osd.json"


def test_extra_args_appended_verbatim():
    cfg = {"pixelpilot": dict(DEFAULTS["pixelpilot"],
                              extraArgs=["--no-vsync", "--foo", "bar"])}
    assert render_pixelpilot_argv(cfg)[-3:] == ["--no-vsync", "--foo", "bar"]


def test_missing_block_uses_builtin_defaults():
    # An empty config still renders a valid argv (defaults baked into the renderer).
    argv = render_pixelpilot_argv({})
    assert argv[0] == "/usr/bin/pixelpilot"
    assert "--config" in argv
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_pixelpilot_render.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.pixelpilot'`.

- [ ] **Step 3: Write minimal implementation**

Create `gs/fpvdgs/pixelpilot.py`:

```python
"""Render the pixelpilot child argv from the effective config.

Reproduces the stock systemd ExecStart byte-for-byte at defaults:
  pixelpilot --osd --osd-custom-message --osd-config OSD --screen-mode SM
             --video-scale VS --dvr-framerate FPS --dvr-fmp4 --dvr-sequenced-files
             --dvr-template DIR/TMPL --config CONFIG [EXTRA...]
The always-on flags are baked in here; the four operator knobs and the
structural paths come from the `pixelpilot` config block.
"""


def render_pixelpilot_argv(effective: dict) -> list[str]:
    pp = effective.get("pixelpilot", {})
    dvr_dir = pp.get("dvrDir", "/var/dvr")
    dvr_template = pp.get("dvrTemplate", "record_%Y-%m-%d_%H-%M-%S.mp4")
    return [
        pp.get("bin", "/usr/bin/pixelpilot"),
        "--osd", "--osd-custom-message",
        "--osd-config", pp.get("osdConfigPath", "/etc/pixelpilot/config_osd.json"),
        "--screen-mode", pp.get("screenMode", "1920x1080@60"),
        "--video-scale", str(pp.get("videoScale", 1.0)),
        "--dvr-framerate", str(pp.get("dvrFramerate", 60)),
        "--dvr-fmp4", "--dvr-sequenced-files",
        "--dvr-template", f"{dvr_dir}/{dvr_template}",
        "--config", pp.get("configPath", "/etc/pixelpilot/pixelpilot.yaml"),
        *pp.get("extraArgs", []),
    ]
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_pixelpilot_render.py -q`
Expected: PASS (4 passed).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/pixelpilot.py gs/tests/unit/test_pixelpilot_render.py
git commit -m "feat(gs): pixelpilot argv renderer"
```

---

### Task 2: Schema — accept & validate the `pixelpilot` block

**Files:**
- Modify: `gs/fpvdgs/schema.py:6` (`CONFIG_TOP_KEYS`) and `validate_effective`
- Test: `gs/tests/unit/test_schema.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_schema.py`:

```python
def test_config_patch_accepts_pixelpilot():
    # should not raise
    schema.validate_config_patch({"pixelpilot": {"screenMode": "1280x720@60"}})


def test_validate_effective_accepts_pixelpilot_block():
    cfg = {"link": {"channel": 132, "width": 40, "region": "US"},
           "pixelpilot": {"enabled": True, "videoScale": 1.0,
                          "dvrFramerate": 60, "screenMode": "1920x1080@60",
                          "extraArgs": []}}
    schema.validate_effective(cfg)  # no raise


def test_validate_effective_rejects_bad_pixelpilot():
    base = {"link": {"channel": 132, "width": 40, "region": "US"}}
    import pytest
    for bad in (
        {"videoScale": 0},
        {"videoScale": -1.0},
        {"videoScale": "x"},
        {"dvrFramerate": 0},
        {"dvrFramerate": 1.5},
        {"enabled": "yes"},
        {"screenMode": ""},
        {"extraArgs": "not-a-list"},
        {"extraArgs": [1, 2]},
    ):
        with pytest.raises(schema.SchemaError):
            schema.validate_effective({**base, "pixelpilot": bad})
```

(`gs/tests/unit/test_schema.py` already imports `from fpvdgs import schema`; if not, add it.)

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -q`
Expected: FAIL — `test_config_patch_accepts_pixelpilot` raises "unknown config keys: ['pixelpilot']"; the reject test fails because nothing validates the block yet.

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/schema.py`, change `CONFIG_TOP_KEYS`:

```python
CONFIG_TOP_KEYS = {"wfb", "drone", "dynamicLink", "pixelpilot"}   # link is excluded
```

Add the validator at the end of the file:

```python
def _validate_pixelpilot(pp: dict) -> None:
    if not isinstance(pp.get("enabled", True), bool):
        raise SchemaError("pixelpilot.enabled must be a bool")
    vs = pp.get("videoScale", 1.0)
    if isinstance(vs, bool) or not isinstance(vs, (int, float)) or vs <= 0:
        raise SchemaError("pixelpilot.videoScale must be a positive number")
    fps = pp.get("dvrFramerate", 60)
    if isinstance(fps, bool) or not isinstance(fps, int) or fps <= 0:
        raise SchemaError("pixelpilot.dvrFramerate must be a positive int")
    for key in ("screenMode", "bin", "configPath", "osdConfigPath",
                "dvrDir", "dvrTemplate"):
        val = pp.get(key)
        if val is not None and (not isinstance(val, str) or not val):
            raise SchemaError(f"pixelpilot.{key} must be a non-empty string")
    extra = pp.get("extraArgs", [])
    if not isinstance(extra, list) or not all(isinstance(a, str) for a in extra):
        raise SchemaError("pixelpilot.extraArgs must be a list of strings")
```

In `validate_effective`, after the `dl = cfg.get("dynamicLink")` block, add:

```python
    pp = cfg.get("pixelpilot")
    if pp is not None:
        _validate_pixelpilot(pp)
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/schema.py gs/tests/unit/test_schema.py
git commit -m "feat(gs): schema accepts and validates pixelpilot block"
```

---

### Task 3: Generalize `RunnerSupervisor` → `ProcessSupervisor`

**Files:**
- Modify: `gs/fpvdgs/runner_supervisor.py` (extract base class; wfb keeps current behavior)
- Test: `gs/tests/unit/test_process_supervisor.py` (new) + existing `gs/tests/unit/test_runner_supervisor.py` (must stay green)

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_process_supervisor.py`:

```python
import time

from fpvdgs.runner_supervisor import ProcessSupervisor


def _settle(argv, **kw):
    # readiness = "still alive at end of a short window" (no port probe)
    return ProcessSupervisor(argv, ready_check=None, ready_timeout=0.4,
                             ready_on_timeout=True, poll_interval=0.05,
                             backoff=0.05, **kw)


def test_settle_readiness_start_succeeds_for_living_process():
    sup = _settle(["sleep", "30"])
    try:
        assert sup.start() is True
        st = sup.state()
        assert st["running"] is True and st["pid"] > 0
    finally:
        sup.shutdown()


def test_settle_readiness_immediate_exit_is_failed_start():
    sup = _settle(["python3", "-c", "import sys; sys.exit(3)"], max_restarts=2)
    try:
        assert sup.start() is False
        # crash-loop guard trips after the budget
        deadline = time.time() + 6
        while time.time() < deadline and not sup.state()["fault"]:
            time.sleep(0.05)
        assert sup.state()["fault"] is True
        assert sup.state()["lastExit"] == 3
    finally:
        sup.shutdown()


def test_set_argv_is_used_on_restart():
    sup = _settle(["sleep", "30"])
    try:
        sup.start()
        sup.set_argv(["sleep", "31"])
        sup.restart()
        # the live process now runs the swapped argv
        import subprocess
        pid = sup.state()["pid"]
        out = subprocess.run(["ps", "-o", "args=", "-p", str(pid)],
                             capture_output=True, text=True).stdout
        assert "31" in out
    finally:
        sup.shutdown()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_process_supervisor.py -q`
Expected: FAIL — `ImportError: cannot import name 'ProcessSupervisor'`.

- [ ] **Step 3: Write minimal implementation**

Replace the contents of `gs/fpvdgs/runner_supervisor.py` with:

```python
"""Spawn, monitor, and restart a child process.

A background watcher thread auto-restarts the child if it exits unexpectedly,
with a crash-loop fault guard. Operator-initiated restarts (start/stop/restart)
do NOT count toward the crash-loop budget and clear any prior fault.

ProcessSupervisor is generic: parameterized by an argv (swappable at runtime
via set_argv), an extra-env dict, and a readiness strategy:
  - probe  : ready as soon as ready_check() is True before the timeout; a
             timeout (or early exit) is a failed start. (wfb: port :8103.)
  - settle : ready iff the process is still alive at the end of the timeout
             window (ready_check=None, ready_on_timeout=True); an early exit is
             a failed start. (pixelpilot: no port to probe.)

RunnerSupervisor specializes it for the wfb data plane.
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


class ProcessSupervisor:
    def __init__(self, argv, env=None, ready_check=None, ready_timeout=10.0,
                 ready_on_timeout=False, log_path=None, max_restarts=5,
                 restart_window=60.0, poll_interval=0.5, backoff=0.5):
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
        self._restarts = 0          # operator-initiated restarts (visibility)
        self._auto_restarts = 0     # watcher-initiated (crash) restarts
        self._last_exit = None
        self._fault = False
        self._recent = []           # timestamps of crash auto-restarts (budget)
        self._supervise = False     # watcher resurrects only while True
        self._watcher = None
        self._stop_evt = threading.Event()
        self._lock = threading.RLock()

    # ---- runtime argv -----------------------------------------------------
    def set_argv(self, argv):
        with self._lock:
            self._argv_list = list(argv)

    # ---- process plumbing -------------------------------------------------
    def _env(self):
        env = dict(os.environ)
        env.update(self._extra_env)
        return env

    def _spawn(self):
        self._log_fh = open(self.log_path, "ab") if self.log_path else None
        self._proc = subprocess.Popen(self._argv_list, env=self._env(),
                                      stdout=(self._log_fh or subprocess.DEVNULL),
                                      stderr=subprocess.STDOUT,
                                      start_new_session=True)

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


class RunnerSupervisor(ProcessSupervisor):
    """The wfb data plane: argv = runner_cmd --profiles P --wlans W..., env sets
    WIFIBROADCAST_CFG, readiness = the wfb-ng stats port (:8103) opening."""

    def __init__(self, runner_cmd, cfg_out, profile, wlans, ready_port=8103,
                 ready_timeout=10.0, log_path=None, max_restarts=5,
                 restart_window=60.0, poll_interval=0.5, backoff=0.5):
        argv = list(runner_cmd) + ["--profiles", profile, "--wlans", *wlans]
        super().__init__(
            argv, env={"WIFIBROADCAST_CFG": cfg_out},
            ready_check=lambda: _port_open(ready_port),
            ready_timeout=ready_timeout, ready_on_timeout=False,
            log_path=log_path, max_restarts=max_restarts,
            restart_window=restart_window, poll_interval=poll_interval,
            backoff=backoff)
```

- [ ] **Step 4: Run both suites to verify they pass**

Run: `cd gs && python -m pytest tests/unit/test_process_supervisor.py tests/unit/test_runner_supervisor.py -q`
Expected: PASS — the new ProcessSupervisor tests pass AND the existing wfb `RunnerSupervisor` tests are unchanged-green (confirms the wfb path is preserved).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/runner_supervisor.py gs/tests/unit/test_process_supervisor.py
git commit -m "refactor(gs): extract ProcessSupervisor; RunnerSupervisor subclasses it"
```

---

### Task 4: Add the `pixelpilot` block to shipped defaults

**Files:**
- Modify: `gs/etc/defaults.json`
- Test: `gs/tests/unit/test_schema.py` (defaults load + validate)

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_schema.py`:

```python
def test_shipped_defaults_include_pixelpilot_and_validate():
    import json, pathlib
    p = pathlib.Path(__file__).resolve().parents[2] / "etc" / "defaults.json"
    cfg = json.loads(p.read_text())
    assert "pixelpilot" in cfg
    assert cfg["pixelpilot"]["enabled"] is True
    schema.validate_effective(cfg)  # no raise
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_schema.py::test_shipped_defaults_include_pixelpilot_and_validate -q`
Expected: FAIL — `assert "pixelpilot" in cfg` (block not present yet).

- [ ] **Step 3: Write minimal implementation**

In `gs/etc/defaults.json`, add a top-level `pixelpilot` key (after `dynamicLink`):

```json
  "pixelpilot": {
    "enabled": true,
    "bin": "/usr/bin/pixelpilot",
    "configPath": "/etc/pixelpilot/pixelpilot.yaml",
    "screenMode": "1920x1080@60",
    "videoScale": 1.0,
    "osdConfigPath": "/etc/pixelpilot/config_osd.json",
    "dvrFramerate": 60,
    "dvrDir": "/var/dvr",
    "dvrTemplate": "record_%Y-%m-%d_%H-%M-%S.mp4",
    "extraArgs": []
  }
```

(Remember the comma after the preceding `dynamicLink` object.)

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/etc/defaults.json gs/tests/unit/test_schema.py
git commit -m "feat(gs): ship pixelpilot defaults block"
```

---

### Task 5: Status — include the `pixelpilot` block

**Files:**
- Modify: `gs/fpvdgs/status.py:32-57` (`build_status`)
- Test: `gs/tests/unit/test_status.py` (new)

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_status.py`:

```python
from fpvdgs.status import build_status


def _runner_state():
    return {"running": True, "pid": 1, "restarts": 0, "autoRestarts": 0,
            "lastExit": None, "fault": False}


def test_status_omits_pixelpilot_when_not_given():
    out = build_status("1.0", _runner_state(), {}, {"reachable": True})
    assert "pixelpilot" not in out


def test_status_includes_pixelpilot_block():
    pp = {"enabled": True, "running": True, "pid": 42, "restarts": 0,
          "autoRestarts": 0, "lastExit": None, "fault": False}
    out = build_status("1.0", _runner_state(), {}, {"reachable": True},
                       pixelpilot=pp)
    assert out["pixelpilot"]["running"] is True
    assert out["pixelpilot"]["pid"] == 42
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_status.py -q`
Expected: FAIL — `build_status() got an unexpected keyword argument 'pixelpilot'`.

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/status.py`, extend `build_status`'s signature and body:

```python
def build_status(version: str, runner_state: dict, wlans: dict,
                 drone_probe: dict, link_stats: dict | None = None,
                 uptime_ms: int | None = None,
                 dynamic_link: dict | None = None,
                 pixelpilot: dict | None = None) -> dict:
```

and just before `return out`:

```python
    if pixelpilot is not None:
        out["pixelpilot"] = pixelpilot
    return out
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_status.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/status.py gs/tests/unit/test_status.py
git commit -m "feat(gs): status carries pixelpilot block"
```

---

### Task 6: API — granular `/apply` + PixelPilot routing

**Files:**
- Modify: `gs/fpvdgs/api.py` (import renderer; `Api.__init__` gains `pixelpilot`; `_apply_gs` excludes pixelpilot from the wfb-bounce trigger; new `_route_pixelpilot`)
- Test: `gs/tests/unit/test_api.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_api.py`:

```python
# --- pixelpilot apply routing ---
class _FakePP:
    def __init__(self):
        self.calls = []
    def set_argv(self, argv):
        self.calls.append(("set_argv", argv))
    def start(self):
        self.calls.append(("start", None))
    def stop(self):
        self.calls.append(("stop", None))
    def restart(self):
        self.calls.append(("restart", None))


def _api_with_pp(tmp_path):
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient
    defaults = {"link": {"channel": 132, "width": 40, "region": "US"},
                "wfb": {"profile": "gs", "raw": {}},
                "drone": {"endpoint": "http://10.5.0.10:8080"},
                "pixelpilot": {"enabled": True, "screenMode": "1920x1080@60",
                               "videoScale": 1.0, "dvrFramerate": 60,
                               "extraArgs": []}}
    store = ConfigStore(defaults)
    runner = _FakeRunner()      # defined earlier in this file
    pp = _FakePP()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=DroneClient("http://127.0.0.1:1"), link=None,
              status_fn=lambda: {}, cfg_out=cfg_out, pixelpilot=pp)
    return api, store, pp, runner


def test_pixelpilot_change_restarts_pp_not_wfb(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"screenMode": "1280x720@60"}})
    code, body = api.handle("POST", "/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert any(c[0] == "set_argv" for c in pp.calls)
    assert ("restart", None) in pp.calls
    assert runner.restarts == 0      # PixelPilot-only change: radio untouched
    assert store.effective()["pixelpilot"]["screenMode"] == "1280x720@60"


def test_wfb_change_does_not_touch_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1
    assert pp.calls == []            # pixelpilot untouched


def test_pixelpilot_disable_then_enable(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    store.patch({"pixelpilot": {"enabled": False}})
    api.handle("POST", "/apply", {}, b"")
    assert ("stop", None) in pp.calls
    pp.calls.clear()
    store.patch({"pixelpilot": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    assert ("start", None) in pp.calls     # off->on uses start(), not restart()
    assert ("restart", None) not in pp.calls


def test_patch_config_accepts_pixelpilot(tmp_path):
    api, store, pp, runner = _api_with_pp(tmp_path)
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"pixelpilot": {"videoScale": 1.5}}).encode())
    assert code == 200
    assert store.pending()["pixelpilot"]["videoScale"] == 1.5
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_api.py -q`
Expected: FAIL — `Api.__init__() got an unexpected keyword argument 'pixelpilot'`.

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/api.py`, add the import near the existing dynlink import:

```python
from .pixelpilot import render_pixelpilot_argv
```

Extend `Api.__init__` to accept and store the supervisor (add `pixelpilot=None` as the last param):

```python
    def __init__(self, store, schema, render_mod, runner, drone, link,
                 status_fn, cfg_out, dynlink=None, pixelpilot=None):
        ...
        self.dynlink = dynlink
        self.pixelpilot = pixelpilot
```

In `_apply_gs`, replace the `non_dl_changed` computation/branch so the wfb bounce excludes BOTH `dynamicLink` and `pixelpilot`, and add the pixelpilot routing call before `commit`:

```python
        # Anything outside dynamicLink/pixelpilot (link already equal) needs the runner.
        wfb_changed = (self._without(pending, "dynamicLink", "pixelpilot")
                       != self._without(effective, "dynamicLink", "pixelpilot"))
        if wfb_changed:
            self.render_mod.write_cfg(self.cfg_out,
                                      self.render_mod.render_cfg(pending))
            if not self.runner.restart():
                self.render_mod.restore_bak(self.cfg_out)
                self.runner.restart()
                return 500, {"applied": False,
                             "error": "runner failed; rolled back to last-good cfg"}

        self._route_dynamic_link(effective.get("dynamicLink", {}),
                                 pending.get("dynamicLink", {}), pending)
        self._route_pixelpilot(effective.get("pixelpilot", {}),
                               pending.get("pixelpilot", {}), pending)
        self.store.commit()
        return 200, {"applied": True}
```

Add the new method next to `_route_dynamic_link`:

```python
    def _route_pixelpilot(self, pp_old, pp_new, pending):
        """Start/stop/restart the PixelPilot child. Never bounces the wfb
        runner. Mirrors _route_dynamic_link (set_argv ≈ set_config)."""
        if self.pixelpilot is None or pp_old == pp_new:
            return
        was, now = bool(pp_old.get("enabled", True)), bool(pp_new.get("enabled", True))
        if now:
            self.pixelpilot.set_argv(render_pixelpilot_argv(pending))
            if was:
                self.pixelpilot.restart()
            else:
                self.pixelpilot.start()
        elif was and not now:
            self.pixelpilot.stop()
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_api.py -q`
Expected: PASS — including the existing dynamicLink routing tests (still green; the renamed `wfb_changed` preserves their behavior).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/api.py gs/tests/unit/test_api.py
git commit -m "feat(gs): granular /apply with pixelpilot start/stop/restart routing"
```

---

### Task 7: Wire the PixelPilot supervisor into `App`/`build_app` + SIGTERM teardown

**Files:**
- Modify: `gs/fpvdgs/supervisor.py` (`App` holds the pixelpilot supervisor; `build_app` constructs it, passes to `Api`, adds status; `main` installs a SIGTERM handler so children are reaped on `S99fpvd stop`)
- Test: `gs/tests/unit/test_app_wiring.py` (new)

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_app_wiring.py`:

```python
from fpvdgs.supervisor import App
from fpvdgs.config import ConfigStore


class _Fake:
    def __init__(self):
        self.calls = []
    def start(self):
        self.calls.append("start")
    def stop(self):
        self.calls.append("stop")
    def shutdown(self):
        self.calls.append("shutdown")
    def serve_forever(self):
        pass


def _app(pp_enabled):
    store = ConfigStore({"pixelpilot": {"enabled": pp_enabled},
                         "dynamicLink": {"enabled": False}})
    runner, http, dynlink, pp = _Fake(), _Fake(), _Fake(), _Fake()
    return App(store, runner, http, api=None, dynlink=dynlink, pixelpilot=pp), pp, runner


def test_app_starts_pixelpilot_when_enabled():
    app, pp, runner = _app(True)
    app.start()
    assert "start" in pp.calls
    assert "start" in runner.calls


def test_app_skips_pixelpilot_when_disabled():
    app, pp, runner = _app(False)
    app.start()
    assert pp.calls == []
    assert "start" in runner.calls


def test_app_shutdown_stops_pixelpilot():
    app, pp, runner = _app(True)
    app.shutdown()
    assert "shutdown" in pp.calls
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_app_wiring.py -q`
Expected: FAIL — `App.__init__()` takes no `pixelpilot` keyword.

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/supervisor.py`:

Add imports near the top imports:

```python
import signal
from .pixelpilot import render_pixelpilot_argv
from .runner_supervisor import RunnerSupervisor, ProcessSupervisor, resolve_wlans
```

(replace the existing `from .runner_supervisor import RunnerSupervisor, resolve_wlans` line).

Replace `App` with:

```python
class App:
    def __init__(self, store, runner, http_server, api, dynlink, pixelpilot=None):
        self.store = store
        self.runner = runner
        self.http = http_server
        self.api = api
        self.dynlink = dynlink
        self.pixelpilot = pixelpilot

    def start(self):
        self.runner.start()
        if (self.pixelpilot is not None
                and self.store.effective().get("pixelpilot", {}).get("enabled", True)):
            self.pixelpilot.start()
        if self.store.effective().get("dynamicLink", {}).get("enabled"):
            self.dynlink.start()

    def serve_forever(self):
        self.http.serve_forever()

    def shutdown(self):
        self.http.shutdown()
        self.dynlink.stop()
        if self.pixelpilot is not None:
            self.pixelpilot.shutdown()
        self.runner.shutdown()
```

In `build_app`, after the `dynlink = DynamicLinkController(...)` line, construct the PixelPilot supervisor:

```python
    pixelpilot = ProcessSupervisor(
        argv=render_pixelpilot_argv(effective),
        ready_timeout=1.5, ready_on_timeout=True,   # settle: alive through the window
        log_path="/tmp/pixelpilot.log")
```

Add a PixelPilot status helper next to `_dynamic_link_status` and include it in `status_fn`'s `build_status` call:

```python
    def _pixelpilot_status():
        pp_cfg = store.effective().get("pixelpilot", {})
        if not bool(pp_cfg.get("enabled", True)):
            return {"enabled": False, "running": False}
        return {"enabled": True, **pixelpilot.state()}
```

and pass it through:

```python
        return status_mod.build_status(__version__, runner.state(), wlan_info, probe,
                                       uptime_ms=uptime_ms,
                                       dynamic_link=_dynamic_link_status(reachable),
                                       pixelpilot=_pixelpilot_status())
```

Pass the supervisor to `Api` and `App`:

```python
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone, link=link, status_fn=status_fn, cfg_out=cfg_out,
              dynlink=dynlink, pixelpilot=pixelpilot)

    http_server = make_http_server(api, host, port)
    return App(store, runner, http_server, api, dynlink, pixelpilot=pixelpilot)
```

In `main`, install a SIGTERM handler so `S99fpvd stop` (start-stop-daemon -K → SIGTERM) runs the `finally: app.shutdown()` and reaps both child processes instead of orphaning them. Add immediately before `app.start()`:

```python
    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, _on_sigterm)
```

(Raising `KeyboardInterrupt` on SIGTERM reuses the existing `try/except KeyboardInterrupt/finally: app.shutdown()` path, making process teardown correct for both the wfb runner and PixelPilot on stop/restart/rollback. The handler runs in the main thread, which is blocked in `app.serve_forever()`, so the exception propagates out cleanly.)

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_app_wiring.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/supervisor.py gs/tests/unit/test_app_wiring.py
git commit -m "feat(gs): supervise pixelpilot as a second managed child; SIGTERM teardown"
```

---

### Task 8: Integration — render → supervisor seam with a fake binary

**Files:**
- Create: `gs/tests/fixtures/fake_pixelpilot.sh`
- Test: `gs/tests/integration/test_pixelpilot_supervisor.py`

- [ ] **Step 1: Write the failing test**

Create `gs/tests/fixtures/fake_pixelpilot.sh`:

```sh
#!/bin/sh
# Test double for pixelpilot. Appends its argv to $PP_ARGV_FILE (if set), then
# either exits non-zero (when --die is present) or sleeps to stay "running".
[ -n "$PP_ARGV_FILE" ] && echo "$@" >> "$PP_ARGV_FILE"
case " $* " in
  *" --die "*) exit 7 ;;
esac
exec sleep 30
```

Make it executable:

```bash
chmod +x gs/tests/fixtures/fake_pixelpilot.sh
```

Create `gs/tests/integration/test_pixelpilot_supervisor.py`:

```python
import os
import time

from fpvdgs.runner_supervisor import ProcessSupervisor

FIXTURE = os.path.join(os.path.dirname(__file__), "..", "fixtures",
                       "fake_pixelpilot.sh")


def _sup(argv, **kw):
    return ProcessSupervisor(argv, ready_check=None, ready_timeout=0.4,
                             ready_on_timeout=True, poll_interval=0.05,
                             backoff=0.05, **kw)


def test_set_argv_changes_what_gets_spawned(tmp_path, monkeypatch):
    argv_file = tmp_path / "argv.log"
    monkeypatch.setenv("PP_ARGV_FILE", str(argv_file))
    sup = _sup([FIXTURE, "--screen-mode", "MODE_A"])
    try:
        assert sup.start() is True
        sup.set_argv([FIXTURE, "--screen-mode", "MODE_B"])
        sup.restart()
        time.sleep(0.1)
        logged = argv_file.read_text()
        assert "MODE_A" in logged and "MODE_B" in logged
    finally:
        sup.shutdown()


def test_crash_recovery_then_fault(tmp_path):
    sup = _sup([FIXTURE, "--die"], max_restarts=2)
    try:
        assert sup.start() is False           # exits 7 immediately
        deadline = time.time() + 6
        while time.time() < deadline and not sup.state()["fault"]:
            time.sleep(0.05)
        assert sup.state()["fault"] is True
        assert sup.state()["lastExit"] == 7
    finally:
        sup.shutdown()
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/integration/test_pixelpilot_supervisor.py -q`
Expected: FAIL initially if the fixture is not executable or missing — then PASS once created/chmod'd. If it already passes after Step 1 (fixture present), that's acceptable for an integration seam test; confirm both assertions exercise real spawns.

- [ ] **Step 3: (No new production code — this validates Tasks 1 + 3 together.)**

If the test fails for a real reason (e.g., argv not swapped), fix `ProcessSupervisor.set_argv`/`_spawn` from Task 3.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && python -m pytest tests/integration/test_pixelpilot_supervisor.py -q`
Expected: PASS (2 passed).

- [ ] **Step 5: Commit**

```bash
git add gs/tests/fixtures/fake_pixelpilot.sh gs/tests/integration/test_pixelpilot_supervisor.py
git commit -m "test(gs): integration — render→supervisor seam with fake pixelpilot"
```

---

### Task 9: Deploy takeover + rollback + verify

**Files:**
- Modify: `deploy/gs/deploy.sh` (retire the stock PixelPilot init script; verify `pidof pixelpilot`)
- Modify: `deploy/gs/rollback.sh` (restore the init script)

No automated test (shell deploy against a live SBC). Changes are byte-exact below.

- [ ] **Step 1: Add the PixelPilot init-script takeover to `deploy/gs/deploy.sh`**

In the heredoc passed to `remote '...'`, immediately AFTER the `S99dynamic-link-gs` retire block and BEFORE `: > /tmp/fpvd.log`, insert:

```sh
    # Retire the stock PixelPilot init script — fpvd now spawns/supervises the
    # pixelpilot binary directly (single owner of process + config). Stop it,
    # move it to the rollback dir. Idempotent: never clobber on re-deploy.
    for pp in /etc/init.d/S*pixelpilot*; do
        [ -e "$pp" ] || continue
        [ -x "$pp" ] && "$pp" stop >/dev/null 2>&1 || true
        mv "$pp" /root/fpvd-gs-rollback/
    done
```

- [ ] **Step 2: Add the verify line to `deploy/gs/deploy.sh`**

In the `[verify]` `remote '...'` block, after the `procs:` line, add:

```sh
    printf "  pp:    "; pidof pixelpilot >/dev/null 2>&1 && echo running || echo DOWN
```

- [ ] **Step 3: Add the restore to `deploy/gs/rollback.sh`**

In the `ssh ... '...'` heredoc, immediately BEFORE `echo rollback-done`, insert:

```sh
    # restore the stock PixelPilot init script retired by deploy.sh (fpvd's
    # shutdown already stopped its pixelpilot child when S99fpvd stopped).
    for pp in /root/fpvd-gs-rollback/S*pixelpilot*; do
        [ -e "$pp" ] || continue
        name="$(basename "$pp")"
        mv "$pp" "/etc/init.d/$name"
        chmod +x "/etc/init.d/$name"
        "/etc/init.d/$name" start >/dev/null 2>&1 || true
    done
```

- [ ] **Step 4: Lint the shell scripts**

Run: `bash -n deploy/gs/deploy.sh && bash -n deploy/gs/rollback.sh && echo OK`
Expected: `OK` (no syntax errors).

- [ ] **Step 5: Commit**

```bash
git add deploy/gs/deploy.sh deploy/gs/rollback.sh
git commit -m "deploy(gs): retire/restore stock pixelpilot init script on takeover"
```

---

### Task 10: Documentation

**Files:**
- Modify: `gs/README.md` (API table note + config reference + smoke step)
- Modify: `docs/api.md` (PixelPilot managed service section)

- [ ] **Step 1: Update `gs/README.md`**

In the intro paragraph, change "supervises the wfb data plane" to note it now also supervises PixelPilot. Add a `### pixelpilot` subsection under "Config reference" documenting the block:

```markdown
### `pixelpilot`

fpvd-GS spawns and supervises the `pixelpilot` binary as a managed child and
builds its argv from this block (reproducing the stock `ExecStart` at defaults).
Changes apply by restarting PixelPilot only — the radio link is untouched.

```json
"pixelpilot": {
  "enabled": true,
  "bin": "/usr/bin/pixelpilot",
  "configPath": "/etc/pixelpilot/pixelpilot.yaml",
  "screenMode": "1920x1080@60",
  "videoScale": 1.0,
  "osdConfigPath": "/etc/pixelpilot/config_osd.json",
  "dvrFramerate": 60,
  "dvrDir": "/var/dvr",
  "dvrTemplate": "record_%Y-%m-%d_%H-%M-%S.mp4",
  "extraArgs": []
}
```

| Key | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Arms PixelPilot supervision; toggle via `PATCH /config` + `POST /apply`. |
| `screenMode` | string | `1920x1080@60` | `--screen-mode` (HDMI output mode). |
| `videoScale` | number | `1.0` | `--video-scale`. |
| `osdConfigPath` | string | `/etc/pixelpilot/config_osd.json` | `--osd-config`. |
| `dvrFramerate` | int | `60` | `--dvr-framerate`. |
| `bin`/`configPath`/`dvrDir`/`dvrTemplate` | string | (see above) | Structural paths (rarely changed). |
| `extraArgs` | list[str] | `[]` | Verbatim-appended flags (escape hatch for un-modeled options). |

`GET /status.pixelpilot` shows `{enabled, running, pid, restarts, autoRestarts,
lastExit, fault}`; `{enabled:false, running:false}` when disabled.
```

In the on-device smoke list, add:

```markdown
8. PixelPilot: `pidof pixelpilot` present; `curl -s :8080/status` shows `pixelpilot.running:true`. `curl -XPATCH :8080/config -d '{"pixelpilot":{"videoScale":1.5}}'` then `curl -XPOST :8080/apply` — 200; only PixelPilot restarts (wfb_rx/wfb_tx PIDs unchanged).
```

- [ ] **Step 2: Update `docs/api.md`**

Add a "PixelPilot managed service" section mirroring the README block: that `pixelpilot.*` flows through `/config` + `/apply`, applies bounce only PixelPilot, and `/status.pixelpilot` reports supervisor state. Note the deploy retires the stock `S*pixelpilot*` init script (rollback restores it).

- [ ] **Step 3: Commit**

```bash
git add gs/README.md docs/api.md
git commit -m "docs(gs): document pixelpilot managed service + config block"
```

---

### Task 11: Full suite + final verification

- [ ] **Step 1: Run the entire GS test suite**

Run: `cd gs && python -m pytest -q`
Expected: PASS — all unit + integration tests green (new and pre-existing).

- [ ] **Step 2: Sanity-check the renderer against the stock ExecStart by eye**

Run: `cd gs && python3 -c "import json,pathlib; from fpvdgs.pixelpilot import render_pixelpilot_argv; print(' '.join(render_pixelpilot_argv(json.loads(pathlib.Path('etc/defaults.json').read_text()))))"`
Expected: prints exactly
`/usr/bin/pixelpilot --osd --osd-custom-message --osd-config /etc/pixelpilot/config_osd.json --screen-mode 1920x1080@60 --video-scale 1.0 --dvr-framerate 60 --dvr-fmp4 --dvr-sequenced-files --dvr-template /var/dvr/record_%Y-%m-%d_%H-%M-%S.mp4 --config /etc/pixelpilot/pixelpilot.yaml`
(matches `debian/pixelpilot-rk.pixelpilot.service` modulo the env-var substitutions).

- [ ] **Step 3: Final commit (if any docs/tweaks remain)**

```bash
git add -A
git commit -m "chore(gs): finalize pixelpilot managed service" --allow-empty
```

---

## Notes for the implementer

- **Order matters:** Tasks 1–2 (pure functions) have no deps; Task 3 is the refactor that the wiring (6, 7) depends on; do them in order.
- **Existing tests are a safety net:** after Task 3, `tests/unit/test_runner_supervisor.py` must stay green — that's the proof the wfb path is byte-for-byte preserved. After Task 6, the existing dynamicLink routing tests in `test_api.py` must stay green.
- **No live apply:** PixelPilot config changes apply on child restart. That is by design (see spec Non-goals).
- **Provisioned files:** `/etc/pixelpilot/{pixelpilot.yaml,config_osd.json}` and the binary are device-provisioned; fpvd points at them, never creates them.
- **`extraArgs`** is the escape hatch for any PixelPilot flag not modeled first-class (e.g., richer DVR/OSD rows) until a follow-up models them.
```
