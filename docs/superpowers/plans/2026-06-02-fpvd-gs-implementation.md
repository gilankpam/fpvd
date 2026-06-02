# fpvd-GS Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Build `fpvd` for the ground station — a Python daemon that owns the GS wfb radio config, supervises the wfb data plane (built on the `wfb_ng` library, replacing `wfb-server`), and exposes a single-front-door HTTP API (`/config`, `/apply`, opaque `/air/*` drone proxy, and a GS-local-first `/link` coordinator).

**Architecture:** Two processes. A pure-stdlib **supervisor** (`fpvdgs.supervisor`) owns config + HTTP API + process supervision and never imports `wfb_ng` (unit-testable without a radio). It spawns a **runner** child (`fpvdgs.runner`) that imports `wfb_ng` and runs the orchestration, driven by a cfg file the supervisor renders from its JSON config. Link/overlap params (channel/width/region/beamforming/linkId) are applied GS-locally-first and pushed to the drone fpvd best-effort.

**Tech Stack:** Python 3.13 (stdlib only for the supervisor: `http.server`, `json`, `subprocess`, `threading`, `urllib`); `wfb_ng` + Twisted (runner only, already on the GS); `pytest` for tests. Existing drone code is C++/CMake.

**Spec:** `docs/superpowers/specs/2026-06-02-fpvd-gs-design.md`

**Conventions:** TDD (test → fail → implement → pass → commit). Frequent commits. End every commit message body with `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`. Work on branch `feat/fpvd-gs`.

**Resolved §15 defaults baked into this plan:** package `fpvdgs`, daemon invoked as `fpvd`, init `S99fpvd`, config dir `/etc/fpvd/`; `link.inSync`/`droneReachable` come from a cached drone probe refreshed on each drone interaction.

---

## File Structure

**Relocated (Task 1):** all existing top-level C++ → `drone/` (`CMakeLists.txt`, `cmake/`, `src/`, `tests/`, `etc/`, `scripts/`, `third_party/`, `shell.nix`, `build/`).

**New GS tree:**

```
gs/
  pyproject.toml                # package metadata; pytest config
  fpvdgs/
    __init__.py                 # __version__
    config.py                   # deep_merge(); ConfigStore (defaults+overlay→effective; pending; commit; reset; persist)
    schema.py                   # LINK_KEYS; validate_config_patch(); validate_link_patch(); validate_effective(); SchemaError
    render.py                   # render_cfg(effective)->str; write_cfg(path,text) atomic + .bak
    drone_client.py             # DroneClient(endpoint): get/patch_config, apply, get_status, healthz, reachable(); DroneUnreachable
    runner_supervisor.py        # resolve_wlans(); RunnerSupervisor (spawn/monitor/restart/stop/readiness/state)
    status.py                   # parse_iw_info(text)->dict; build_status(...)
    link.py                     # LinkCoordinator(store,renderer,runner,drone): apply_link(apply_to)->dict
    api.py                      # Api(deps).handle(method,path,query,body)->(status,obj); make_http_server()
    runner.py                   # wfb_ng-backed data-plane entrypoint (imports wfb_ng.server)
    supervisor.py               # main(): parse argv, wire deps, render+spawn runner, serve HTTP
  etc/
    defaults.json               # GS baseline (seeded from live values)
  scripts/
    S99fpvd                     # init script (replaces S98wifibroadcast)
  tests/
    conftest.py                 # fixtures: free_port, fake_drone server, tmp paths
    unit/
      test_config.py
      test_schema.py
      test_render.py
      test_drone_client.py
      test_runner_supervisor.py
      test_status.py
      test_link.py
      test_api.py
      test_runner.py
    integration/
      test_supervisor_e2e.py
deploy/gs/
  deploy.sh                     # rsync pkg + init to GS, backup/disable S98wifibroadcast, restart
  rollback.sh                   # restore S98wifibroadcast + saved cfg
```

**Key interfaces (locked here; tasks must match these names/signatures):**

- `config.deep_merge(base: dict, overlay: dict) -> dict`
- `config.ConfigStore(defaults: dict, overlay: dict|None=None, overlay_path: str|None=None)`
  - `.defaults()`, `.effective()`, `.pending()`, `.patch(sparse: dict)`, `.commit()`, `.reset()`
  - classmethod `.load(defaults_path: str, overlay_path: str) -> ConfigStore`
- `schema.LINK_KEYS = {"channel","width","txpower","region","linkId","beamforming","wlans"}`
- `schema.validate_config_patch(sparse: dict) -> None` (raises `SchemaError`; rejects any `link.*`)
- `schema.validate_link_patch(sparse: dict) -> None` (raises `SchemaError`; only `link.*` allowed)
- `schema.validate_effective(cfg: dict) -> None`
- `render.render_cfg(effective: dict) -> str`; `render.write_cfg(path: str, text: str) -> None`
- `drone_client.DroneClient(endpoint: str, timeout: float=4.0)` with `.healthz()->bool`, `.patch_config(d)`, `.apply()`, `.get_status()->dict`, `.get_config()->dict`; raises `DroneUnreachable`
- `runner_supervisor.resolve_wlans(cfg: dict) -> list[str]`
- `runner_supervisor.RunnerSupervisor(runner_cmd: list[str], cfg_out: str, profile: str, wlans: list[str], ready_port: int=8103, ready_timeout: float=10.0)`
  - `.start()`, `.stop()`, `.restart()`, `.state() -> dict` (`running,pid,restarts,lastExit`)
- `status.parse_iw_info(text: str) -> dict`; `status.build_status(version, runner_state, wlans, drone_probe) -> dict`
- `link.LinkCoordinator(store, renderer_write, runner, drone).apply_link(apply_to: str) -> dict` (`renderer_write(effective: dict)` captures `cfg_out`)
- `api.Api(store, schema, render_mod, runner, drone, link, status_fn, cfg_out).handle(method, path, query, body: bytes) -> tuple[int, object]`

---

## Task 1: Relocate C++ into `drone/`

**Files:**
- Move: `CMakeLists.txt`, `cmake/`, `src/`, `tests/`, `etc/`, `scripts/`, `third_party/`, `shell.nix`, `build/` → `drone/`
- Modify: `deploy/drone/deploy.sh`
- Modify: `README.md`

- [ ] **Step 1: Move the C++ tree as a unit (preserves history)**

```bash
cd /home/gilankpam/Projects/drone/fpvd
mkdir -p drone
git mv CMakeLists.txt cmake src tests etc scripts third_party shell.nix drone/
# build/ is generated and gitignored; move it if present so paths line up locally
[ -d build ] && mv build drone/ || true
```

- [ ] **Step 2: Repoint the drone deploy script**

In `deploy/drone/deploy.sh`, `REPO` resolves to the repo root but the C++ now lives in `drone/`. Change the build/strip/binary paths:

```bash
# was: REPO="$(cd "$(dirname "$0")/../.." && pwd)"
#      BIN="$REPO/build/ssc338q/fpvd"
REPO="$(cd "$(dirname "$0")/../.." && pwd)"
CPP="$REPO/drone"
BIN="$CPP/build/ssc338q/fpvd"
```

And the build invocation (was `cd "$REPO" && cmake -S . -B build/ssc338q …`):

```bash
    ( cd "$CPP" && nix-shell --run "cmake -S . -B build/ssc338q \
        -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-ssc338q.cmake -DCMAKE_BUILD_TYPE=Release \
        && cmake --build build/ssc338q --target fpvd -j" )
```

And the strip step's `cd "$REPO"` → `cd "$CPP"`.

Also repoint the remaining `$REPO/<moved>` source paths (the moved `scripts/` and `etc/`) to `$CPP` — the `copy` lines that push radio scripts + defaults:
```bash
copy "$CPP/scripts/radio-up.sh"   /usr/libexec/fpvd/radio-up.sh
copy "$CPP/scripts/radio-tune.sh" /usr/libexec/fpvd/radio-tune.sh
copy "$CPP/etc/defaults.json"     /etc/fpvd/defaults.json
```
(Grep the script for any other `$REPO/` reference to a moved path and repoint to `$CPP/`.)

- [ ] **Step 3: Update README build paths**

In `README.md`, change host/target build commands to run from `drone/`:

```
    cmake -S drone -B drone/build -DCMAKE_BUILD_TYPE=Debug
    cmake --build drone/build -j
    ./drone/build/fpvd_tests

    cmake -S drone -B drone/build/ssc338q -DCMAKE_TOOLCHAIN_FILE=drone/cmake/toolchain-ssc338q.cmake
    cmake --build drone/build/ssc338q --target fpvd -j
```

- [ ] **Step 4: Verify the host build + tests still pass after the move**

Run:
```bash
cd /home/gilankpam/Projects/drone/fpvd
rm -rf drone/build/host
cmake -S drone -B drone/build/host -DCMAKE_BUILD_TYPE=Debug && cmake --build drone/build/host -j && ./drone/build/host/fpvd_tests
```
Expected: CMake configures (the `${CMAKE_SOURCE_DIR}`-relative includes resolve under `drone/`), build succeeds, all doctest cases pass.

- [ ] **Step 5: Commit**

```bash
git add -A
git commit -m "refactor: relocate drone C++ into drone/ for polyglot repo

Mechanical git mv of the C++ tree (CMakeLists, cmake, src, tests, etc,
scripts, third_party, shell.nix) into drone/. Repoint deploy/drone/deploy.sh
build paths and README. No source changes; \${CMAKE_SOURCE_DIR}-relative
includes are unaffected.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: GS package skeleton + pytest harness

**Files:**
- Create: `gs/pyproject.toml`
- Create: `gs/fpvdgs/__init__.py`
- Create: `gs/tests/__init__.py`, `gs/tests/unit/__init__.py`
- Test: `gs/tests/unit/test_version.py`

- [ ] **Step 1: Write the failing test**

`gs/tests/unit/test_version.py`:
```python
import fpvdgs


def test_version_is_a_string():
    assert isinstance(fpvdgs.__version__, str)
    assert fpvdgs.__version__
```

- [ ] **Step 2: Run it to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_version.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs'`.

- [ ] **Step 3: Create the package + packaging metadata**

`gs/fpvdgs/__init__.py`:
```python
"""fpvd — ground-station FPV supervisor."""

__version__ = "0.1.0"
```

`gs/pyproject.toml`:
```toml
[build-system]
requires = ["setuptools>=61"]
build-backend = "setuptools.build_meta"

[project]
name = "fpvdgs"
version = "0.1.0"
description = "fpvd ground-station wfb supervisor"
requires-python = ">=3.11"

[project.scripts]
fpvd = "fpvdgs.supervisor:main"
fpvd-runner = "fpvdgs.runner:main"

[tool.setuptools.packages.find]
include = ["fpvdgs*"]

[tool.pytest.ini_options]
pythonpath = ["."]
testpaths = ["tests"]
```

Create empty `gs/tests/__init__.py` and `gs/tests/unit/__init__.py`.

- [ ] **Step 4: Run it to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_version.py -v`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/pyproject.toml gs/fpvdgs/__init__.py gs/tests
git commit -m "feat(gs): fpvdgs package skeleton + pytest harness

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: ConfigStore

**Files:**
- Create: `gs/fpvdgs/config.py`
- Test: `gs/tests/unit/test_config.py`

- [ ] **Step 1: Write the failing tests**

`gs/tests/unit/test_config.py`:
```python
import json

from fpvdgs.config import ConfigStore, deep_merge


def test_deep_merge_recurses_and_overrides():
    base = {"link": {"channel": 132, "width": 40}, "wfb": {"profile": "gs"}}
    overlay = {"link": {"channel": 100}}
    assert deep_merge(base, overlay) == {
        "link": {"channel": 100, "width": 40},
        "wfb": {"profile": "gs"},
    }


def test_deep_merge_does_not_mutate_inputs():
    base = {"a": {"b": 1}}
    overlay = {"a": {"c": 2}}
    deep_merge(base, overlay)
    assert base == {"a": {"b": 1}}
    assert overlay == {"a": {"c": 2}}


def test_effective_is_defaults_when_no_overlay():
    s = ConfigStore({"link": {"channel": 132}})
    assert s.effective() == {"link": {"channel": 132}}


def test_patch_accumulates_into_pending_without_touching_effective():
    s = ConfigStore({"link": {"channel": 132, "width": 40}})
    s.patch({"link": {"width": 20}})
    assert s.effective() == {"link": {"channel": 132, "width": 40}}
    assert s.pending() == {"link": {"channel": 132, "width": 20}}


def test_commit_promotes_pending_to_effective():
    s = ConfigStore({"link": {"channel": 132}})
    s.patch({"link": {"channel": 100}})
    s.commit()
    assert s.effective() == {"link": {"channel": 100}}


def test_reset_drops_overlay_and_pending():
    s = ConfigStore({"link": {"channel": 132}})
    s.patch({"link": {"channel": 100}})
    s.commit()
    s.reset()
    assert s.effective() == {"link": {"channel": 132}}
    assert s.pending() == {"link": {"channel": 132}}


def test_load_and_persist_roundtrip(tmp_path):
    defaults = tmp_path / "defaults.json"
    overlay = tmp_path / "config.json"
    defaults.write_text(json.dumps({"link": {"channel": 132}}))
    s = ConfigStore.load(str(defaults), str(overlay))
    s.patch({"link": {"channel": 100}})
    s.commit()
    assert json.loads(overlay.read_text()) == {"link": {"channel": 100}}
    # reload sees the persisted overlay
    s2 = ConfigStore.load(str(defaults), str(overlay))
    assert s2.effective() == {"link": {"channel": 100}}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_config.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.config'`.

- [ ] **Step 3: Implement `config.py`**

`gs/fpvdgs/config.py`:
```python
"""Config store: defaults baked-in, sparse user overlay, pending edits."""

import copy
import json
import os
import threading


def deep_merge(base: dict, overlay: dict) -> dict:
    """Return a new dict: overlay deep-merged onto base. Inputs untouched."""
    out = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


class ConfigStore:
    def __init__(self, defaults: dict, overlay: dict | None = None,
                 overlay_path: str | None = None):
        self._defaults = copy.deepcopy(defaults)
        self._overlay = copy.deepcopy(overlay) if overlay else {}
        self._pending = copy.deepcopy(self._overlay)
        self._overlay_path = overlay_path
        self._lock = threading.RLock()

    @classmethod
    def load(cls, defaults_path: str, overlay_path: str) -> "ConfigStore":
        with open(defaults_path) as f:
            defaults = json.load(f)
        overlay = {}
        if overlay_path and os.path.exists(overlay_path):
            with open(overlay_path) as f:
                overlay = json.load(f)
        return cls(defaults, overlay, overlay_path)

    def defaults(self) -> dict:
        return copy.deepcopy(self._defaults)

    def effective(self) -> dict:
        with self._lock:
            return deep_merge(self._defaults, self._overlay)

    def pending(self) -> dict:
        with self._lock:
            return deep_merge(self._defaults, self._pending)

    def patch(self, sparse: dict) -> None:
        with self._lock:
            self._pending = deep_merge(self._pending, sparse)

    def commit(self) -> None:
        with self._lock:
            self._overlay = copy.deepcopy(self._pending)
            self._persist()

    def reset(self) -> None:
        with self._lock:
            self._overlay = {}
            self._pending = {}
            self._persist()

    def _persist(self) -> None:
        if not self._overlay_path:
            return
        tmp = self._overlay_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._overlay, f, indent=2)
        os.replace(tmp, self._overlay_path)
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_config.py -v`
Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/config.py gs/tests/unit/test_config.py
git commit -m "feat(gs): ConfigStore (defaults+overlay, pending, persist)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Schema validation

**Files:**
- Create: `gs/fpvdgs/schema.py`
- Test: `gs/tests/unit/test_schema.py`

- [ ] **Step 1: Write the failing tests**

`gs/tests/unit/test_schema.py`:
```python
import pytest

from fpvdgs.schema import (
    SchemaError,
    validate_config_patch,
    validate_link_patch,
    validate_effective,
)


def test_config_patch_rejects_link_keys():
    with pytest.raises(SchemaError) as e:
        validate_config_patch({"link": {"channel": 100}})
    assert "link" in str(e.value)


def test_config_patch_allows_non_link():
    validate_config_patch({"wfb": {"mavlink": {"peer": "connect://127.0.0.1:14550"}}})


def test_config_patch_rejects_unknown_top_level():
    with pytest.raises(SchemaError):
        validate_config_patch({"bogus": 1})


def test_link_patch_allows_only_link():
    validate_link_patch({"link": {"channel": 100, "width": 20}})
    with pytest.raises(SchemaError):
        validate_link_patch({"wfb": {"profile": "gs"}})


def test_link_patch_rejects_unknown_link_key():
    with pytest.raises(SchemaError):
        validate_link_patch({"link": {"mcs": 5}})


def test_validate_effective_checks_width_domain():
    with pytest.raises(SchemaError):
        validate_effective({"link": {"channel": 132, "width": 80, "region": "US"}})


def test_validate_effective_ok():
    validate_effective({
        "link": {"channel": 132, "width": 40, "txpower": 19, "region": "US",
                 "linkId": 7669206, "beamforming": {"enabled": False}, "wlans": "auto"},
        "wfb": {"profile": "gs", "mavlink": {"peer": "connect://127.0.0.1:14550"}, "raw": {}},
        "drone": {"endpoint": "http://10.5.0.10:8080"},
    })
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.schema'`.

- [ ] **Step 3: Implement `schema.py`**

`gs/fpvdgs/schema.py`:
```python
"""Validation rules. Link/overlap params are mutated ONLY via /link."""

LINK_KEYS = {"channel", "width", "txpower", "region", "linkId", "beamforming", "wlans"}
CONFIG_TOP_KEYS = {"wfb", "drone"}      # link is excluded on purpose
ALL_TOP_KEYS = {"link"} | CONFIG_TOP_KEYS
VALID_WIDTHS = {20, 40}                  # 10 reserved for future


class SchemaError(ValueError):
    pass


def validate_config_patch(sparse: dict) -> None:
    """A /config PATCH: any top-level key except `link`."""
    if "link" in sparse:
        raise SchemaError("link.* is read-only via /config; use /link")
    unknown = set(sparse) - CONFIG_TOP_KEYS
    if unknown:
        raise SchemaError(f"unknown config keys: {sorted(unknown)}")


def validate_link_patch(sparse: dict) -> None:
    """A /link PATCH: only `link.*`, only known link keys."""
    if set(sparse) - {"link"}:
        raise SchemaError("only link.* allowed via /link")
    link = sparse.get("link", {})
    if not isinstance(link, dict) or not link:
        raise SchemaError("link patch must be a non-empty object")
    unknown = set(link) - LINK_KEYS
    if unknown:
        raise SchemaError(f"unknown link keys: {sorted(unknown)}")


def validate_effective(cfg: dict) -> None:
    """Sanity-check the full effective config before rendering/applying."""
    link = cfg.get("link", {})
    width = link.get("width")
    if width is not None and width not in VALID_WIDTHS:
        raise SchemaError(f"link.width must be one of {sorted(VALID_WIDTHS)}")
    if not link.get("region"):
        raise SchemaError("link.region is required")
    if not link.get("channel"):
        raise SchemaError("link.channel is required")
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -v`
Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/schema.py gs/tests/unit/test_schema.py
git commit -m "feat(gs): schema validation with /link overlap guard

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: CfgRenderer

**Files:**
- Create: `gs/fpvdgs/render.py`
- Test: `gs/tests/unit/test_render.py`

- [ ] **Step 1: Write the failing tests**

`gs/tests/unit/test_render.py`:
```python
import configparser

from fpvdgs.render import render_cfg, write_cfg

EFFECTIVE = {
    "link": {"channel": 132, "width": 40, "txpower": 19, "region": "US",
             "linkId": 7669206, "beamforming": {"enabled": False}, "wlans": "auto"},
    "wfb": {"profile": "gs", "mavlink": {"peer": "connect://127.0.0.1:14550"},
            "raw": {"gs_tunnel": {"ldpc": "1"}}},
    "drone": {"endpoint": "http://10.5.0.10:8080"},
}


def _parse(text):
    cp = configparser.RawConfigParser(strict=False)
    cp.read_string(text)
    return cp


def test_render_has_generated_header():
    text = render_cfg(EFFECTIVE)
    assert text.lstrip().startswith("#")
    assert "generated by fpvd" in text.lower()


def test_render_maps_common_section():
    cp = _parse(render_cfg(EFFECTIVE))
    assert cp.get("common", "wifi_channel") == "132"
    assert cp.get("common", "wifi_region") == "'US'"
    assert cp.get("common", "wifi_txpower") == "19"


def test_render_maps_video_bandwidth_from_width():
    cp = _parse(render_cfg(EFFECTIVE))
    assert cp.get("gs_video", "bandwidth") == "40"


def test_render_maps_mavlink_peer():
    cp = _parse(render_cfg(EFFECTIVE))
    assert cp.get("gs_mavlink", "peer") == "connect://127.0.0.1:14550"


def test_render_raw_passthrough():
    cp = _parse(render_cfg(EFFECTIVE))
    assert cp.get("gs_tunnel", "ldpc") == "1"


def test_write_cfg_atomic_keeps_bak(tmp_path):
    p = tmp_path / "wifibroadcast.cfg"
    write_cfg(str(p), "first\n")
    write_cfg(str(p), "second\n")
    assert p.read_text() == "second\n"
    assert (tmp_path / "wifibroadcast.cfg.bak").read_text() == "first\n"
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_render.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.render'`.

- [ ] **Step 3: Implement `render.py`**

`gs/fpvdgs/render.py`:
```python
"""Render the effective config to /etc/wifibroadcast.cfg (a generated artifact)."""

import os

HEADER = (
    "# generated by fpvd — do not edit.\n"
    "# source of truth: /etc/fpvd/{defaults,config}.json\n"
)


def render_cfg(effective: dict) -> str:
    link = effective.get("link", {})
    wfb = effective.get("wfb", {})
    lines = [HEADER, "[common]"]
    lines.append(f"wifi_channel = {link['channel']}")
    lines.append(f"wifi_region = '{link['region']}'")
    if link.get("txpower") is not None:
        lines.append(f"wifi_txpower = {link['txpower']}")
    if link.get("linkId") is not None:
        lines.append(f"link_id = {link['linkId']}")

    # gs_video: width drives the card bandwidth (HT20/HT40)
    lines.append("")
    lines.append("[gs_video]")
    lines.append(f"bandwidth = {link['width']}")

    mav = wfb.get("mavlink", {})
    if mav.get("peer"):
        lines.append("")
        lines.append("[gs_mavlink]")
        lines.append(f"peer = {mav['peer']}")

    # raw passthrough: {section: {key: value}}
    for section, kv in (wfb.get("raw") or {}).items():
        lines.append("")
        lines.append(f"[{section}]")
        for key, value in kv.items():
            lines.append(f"{key} = {value}")

    return "\n".join(lines) + "\n"


def write_cfg(path: str, text: str) -> None:
    """Atomic write; previous content preserved as <path>.bak."""
    if os.path.exists(path):
        os.replace(path, path + ".bak")
    tmp = path + ".tmp"
    with open(tmp, "w") as f:
        f.write(text)
    os.replace(tmp, path)
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_render.py -v`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/render.py gs/tests/unit/test_render.py
git commit -m "feat(gs): CfgRenderer (config -> wifibroadcast.cfg, atomic + .bak)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: DroneClient + test fixtures

**Files:**
- Create: `gs/fpvdgs/drone_client.py`
- Create: `gs/tests/conftest.py`
- Test: `gs/tests/unit/test_drone_client.py`

- [ ] **Step 1: Write shared fixtures**

`gs/tests/conftest.py`:
```python
import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import pytest


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


@pytest.fixture
def free_port():
    return _free_port()


@pytest.fixture
def fake_drone():
    """A stub drone fpvd. .calls records (method, path, body). .fail toggles 500s."""
    state = {"calls": [], "fail": False, "config": {"link": {"channel": 132}}}

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _body(self):
            n = int(self.headers.get("content-length", 0))
            return self.rfile.read(n) if n else b""

        def _send(self, code, obj):
            data = json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            state["calls"].append(("GET", self.path, b""))
            if self.path == "/healthz":
                self._send(200, {"ok": True})
            elif self.path == "/status":
                self._send(200, {"link": {"channel": state["config"]["link"]["channel"]}})
            elif self.path == "/config":
                self._send(200, state["config"])
            else:
                self._send(404, {"error": "nf"})

        def do_PATCH(self):
            body = self._body()
            state["calls"].append(("PATCH", self.path, body))
            if state["fail"]:
                self._send(500, {"error": "boom"})
                return
            patch = json.loads(body or b"{}")
            state["config"].setdefault("link", {}).update(patch.get("link", {}))
            self._send(200, state["config"])

        def do_POST(self):
            self.state_calls = state["calls"].append(("POST", self.path, self._body()))
            self._send(500 if state["fail"] else 200, {"applied": not state["fail"]})

    port = _free_port()
    srv = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    t = threading.Thread(target=srv.serve_forever, daemon=True)
    t.start()
    state["endpoint"] = f"http://127.0.0.1:{port}"
    yield state
    srv.shutdown()
```

- [ ] **Step 2: Write the failing tests**

`gs/tests/unit/test_drone_client.py`:
```python
import pytest

from fpvdgs.drone_client import DroneClient, DroneUnreachable


def test_healthz_true_when_up(fake_drone):
    c = DroneClient(fake_drone["endpoint"])
    assert c.healthz() is True


def test_healthz_false_when_down():
    c = DroneClient("http://127.0.0.1:9", timeout=0.3)  # nothing listens on :9
    assert c.healthz() is False


def test_patch_then_apply_records_calls(fake_drone):
    c = DroneClient(fake_drone["endpoint"])
    c.patch_config({"link": {"channel": 100}})
    c.apply()
    methods = [(m, p) for (m, p, _b) in fake_drone["calls"]]
    assert ("PATCH", "/config") in methods
    assert ("POST", "/apply") in methods


def test_apply_raises_on_drone_error(fake_drone):
    fake_drone["fail"] = True
    c = DroneClient(fake_drone["endpoint"])
    with pytest.raises(DroneUnreachable):
        c.apply()


def test_unreachable_raises(monkeypatch):
    c = DroneClient("http://127.0.0.1:9", timeout=0.3)
    with pytest.raises(DroneUnreachable):
        c.get_status()
```

- [ ] **Step 3: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_drone_client.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.drone_client'`.

- [ ] **Step 4: Implement `drone_client.py`**

`gs/fpvdgs/drone_client.py`:
```python
"""HTTP client to the drone fpvd. Used by /air proxy and /link coordination."""

import json
import urllib.error
import urllib.request


class DroneUnreachable(Exception):
    pass


class DroneClient:
    def __init__(self, endpoint: str, timeout: float = 4.0):
        self.endpoint = endpoint.rstrip("/")
        self.timeout = timeout

    def _request(self, method: str, path: str, body: dict | None = None) -> tuple[int, bytes]:
        data = json.dumps(body).encode() if body is not None else None
        req = urllib.request.Request(self.endpoint + path, data=data, method=method)
        if data is not None:
            req.add_header("content-type", "application/json")
        try:
            with urllib.request.urlopen(req, timeout=self.timeout) as resp:
                return resp.status, resp.read()
        except urllib.error.HTTPError as e:
            return e.code, e.read()
        except (urllib.error.URLError, OSError) as e:
            raise DroneUnreachable(str(e))

    def _ok_json(self, method: str, path: str, body: dict | None = None) -> dict:
        code, raw = self._request(method, path, body)
        if code >= 400:
            raise DroneUnreachable(f"drone {method} {path} -> {code}")
        return json.loads(raw or b"{}")

    def healthz(self) -> bool:
        try:
            code, _ = self._request("GET", "/healthz")
            return code == 200
        except DroneUnreachable:
            return False

    def get_config(self) -> dict:
        return self._ok_json("GET", "/config")

    def get_status(self) -> dict:
        return self._ok_json("GET", "/status")

    def patch_config(self, sparse: dict) -> dict:
        return self._ok_json("PATCH", "/config", sparse)

    def apply(self) -> dict:
        return self._ok_json("POST", "/apply")
```

- [ ] **Step 5: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_drone_client.py -v`
Expected: PASS (5 tests).

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/drone_client.py gs/tests/conftest.py gs/tests/unit/test_drone_client.py
git commit -m "feat(gs): DroneClient + fake-drone test fixture

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: RunnerSupervisor

**Files:**
- Create: `gs/fpvdgs/runner_supervisor.py`
- Test: `gs/tests/unit/test_runner_supervisor.py`

- [ ] **Step 1: Write the failing tests**

`gs/tests/unit/test_runner_supervisor.py`:
```python
import socket
import time

from fpvdgs.runner_supervisor import RunnerSupervisor, resolve_wlans


def test_resolve_wlans_explicit_list():
    assert resolve_wlans({"link": {"wlans": ["wlan0", "wlan1"]}}) == ["wlan0", "wlan1"]


def test_resolve_wlans_auto_uses_wfb_nics(monkeypatch):
    import fpvdgs.runner_supervisor as rs
    monkeypatch.setattr(rs, "_wfb_nics", lambda: ["wlxAAA", "wlxBBB"])
    assert resolve_wlans({"link": {"wlans": "auto"}}) == ["wlxAAA", "wlxBBB"]


def _listener_cmd(port):
    # a fake runner that opens the readiness port and sleeps
    code = (
        "import socket,time;"
        "s=socket.socket();s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);"
        f"s.bind(('127.0.0.1',{port}));s.listen(1);"
        "time.sleep(30)"
    )
    return ["python3", "-c", code]


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_start_reaches_ready_then_stop():
    port = _free_port()
    sup = RunnerSupervisor(_listener_cmd(port), cfg_out="/tmp/ignored.cfg",
                           profile="gs", wlans=["wlan0"], ready_port=port,
                           ready_timeout=5.0)
    assert sup.start() is True
    assert sup.state()["running"] is True
    assert sup.state()["pid"] > 0
    sup.stop()
    time.sleep(0.2)
    assert sup.state()["running"] is False


def test_restart_increments_counter():
    port = _free_port()
    sup = RunnerSupervisor(_listener_cmd(port), cfg_out="/tmp/ignored.cfg",
                           profile="gs", wlans=["wlan0"], ready_port=port,
                           ready_timeout=5.0)
    sup.start()
    sup.restart()
    assert sup.state()["restarts"] >= 1
    sup.stop()
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_runner_supervisor.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.runner_supervisor'`.

- [ ] **Step 3: Implement `runner_supervisor.py`**

`gs/fpvdgs/runner_supervisor.py`:
```python
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
        log = open(self.log_path, "ab") if self.log_path else subprocess.DEVNULL
        self._proc = subprocess.Popen(self._argv(), env=self._env(),
                                      stdout=log, stderr=subprocess.STDOUT,
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
                    os.killpg(self._proc.pid, signal.SIGKILL)
            self._last_exit = self._proc.returncode
            self._proc = None

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
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_runner_supervisor.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/runner_supervisor.py gs/tests/unit/test_runner_supervisor.py
git commit -m "feat(gs): RunnerSupervisor (spawn/monitor/restart/readiness)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 8: Status probe (`iw` parse + assembly)

**Files:**
- Create: `gs/fpvdgs/status.py`
- Test: `gs/tests/unit/test_status.py`

- [ ] **Step 1: Write the failing tests**

`gs/tests/unit/test_status.py`:
```python
from fpvdgs.status import parse_iw_info, build_status

IW = """Interface wlx84fc146c36e6
\tifindex 5
\ttype monitor
\tchannel 132 (5660 MHz), width: 40 MHz, center1: 5670 MHz
\ttxpower 19.00 dBm
"""


def test_parse_iw_info():
    d = parse_iw_info(IW)
    assert d["type"] == "monitor"
    assert d["channel"] == 132
    assert d["freqMhz"] == 5660
    assert d["widthMhz"] == 40
    assert d["txpowerDbm"] == 19.0


def test_build_status_shape():
    s = build_status(
        version="0.1.0",
        runner_state={"running": True, "pid": 591, "restarts": 0, "lastExit": None},
        wlans={"wlx84fc146c36e6": parse_iw_info(IW)},
        drone_probe={"reachable": True, "inSync": True, "linkId": 7669206},
    )
    assert s["runner"]["pid"] == 591
    assert s["radio"][0]["wlan"] == "wlx84fc146c36e6"
    assert s["radio"][0]["channel"] == 132
    assert s["link"]["droneReachable"] is True
    assert s["link"]["inSync"] is True
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_status.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.status'`.

- [ ] **Step 3: Implement `status.py`**

`gs/fpvdgs/status.py`:
```python
"""Assemble GET /status: runner state + per-wlan radio state + link/drone."""

import re
import subprocess


def parse_iw_info(text: str) -> dict:
    out: dict = {}
    m = re.search(r"^\s*type (\w+)", text, re.M)
    if m:
        out["type"] = m.group(1)
    m = re.search(r"channel (\d+) \((\d+) MHz\), width: (\d+) MHz", text)
    if m:
        out["channel"] = int(m.group(1))
        out["freqMhz"] = int(m.group(2))
        out["widthMhz"] = int(m.group(3))
    m = re.search(r"txpower ([\d.]+) dBm", text)
    if m:
        out["txpowerDbm"] = float(m.group(1))
    return out


def iw_info(wlan: str) -> dict:
    try:
        out = subprocess.run(["iw", "dev", wlan, "info"],
                             capture_output=True, text=True, timeout=3)
        return parse_iw_info(out.stdout)
    except (OSError, subprocess.SubprocessError):
        return {}


def build_status(version: str, runner_state: dict, wlans: dict,
                 drone_probe: dict, link_stats: dict | None = None) -> dict:
    radio = []
    for wlan, info in wlans.items():
        radio.append({"wlan": wlan, **info})
    link = {
        "linkId": drone_probe.get("linkId"),
        "droneReachable": drone_probe.get("reachable", False),
        "inSync": drone_probe.get("inSync"),
    }
    if link_stats:
        link["stats"] = link_stats
    return {
        "fpvd": {"version": version},
        "runner": runner_state,
        "radio": radio,
        "link": link,
    }
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_status.py -v`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/status.py gs/tests/unit/test_status.py
git commit -m "feat(gs): status probe (iw parse + status assembly)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 9: LinkCoordinator

**Files:**
- Create: `gs/fpvdgs/link.py`
- Test: `gs/tests/unit/test_link.py`

The coordinator owns the GS-local-first apply: it always applies the GS side, and best-effort pushes the link subset to the drone when `apply_to == "both"` and the drone is reachable.

- [ ] **Step 1: Write the failing tests**

`gs/tests/unit/test_link.py`:
```python
from fpvdgs.config import ConfigStore
from fpvdgs.link import LinkCoordinator


class FakeRunner:
    def __init__(self):
        self.restarts = 0

    def restart(self):
        self.restarts += 1
        return True


class FakeDrone:
    def __init__(self, reachable=True):
        self._reachable = reachable
        self.patched = None
        self.applied = False

    def healthz(self):
        return self._reachable

    def patch_config(self, sparse):
        if not self._reachable:
            from fpvdgs.drone_client import DroneUnreachable
            raise DroneUnreachable("down")
        self.patched = sparse
        return {}

    def apply(self):
        self.applied = True
        return {}


def _store():
    return ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"}})


def _coord(store, runner, drone, written):
    return LinkCoordinator(store, lambda cfg: written.append(cfg), runner, drone)


def test_apply_both_reachable_pushes_drone_then_applies_gs():
    store = _store()
    store.patch({"link": {"channel": 100}})
    runner, drone, written = FakeRunner(), FakeDrone(reachable=True), []
    res = _coord(store, runner, drone, written).apply_link("both")
    # Only the shared subset (channel/width/linkId) is pushed — not region.
    assert drone.patched == {"link": {"channel": 100, "width": 40}}
    assert drone.applied is True
    assert runner.restarts == 1
    assert store.effective()["link"]["channel"] == 100
    assert res == {"gsApplied": True, "droneApplied": True,
                   "droneReachable": True, "inSync": True}


def test_apply_both_drone_down_still_applies_gs():
    store = _store()
    store.patch({"link": {"channel": 100}})
    runner, drone, written = FakeRunner(), FakeDrone(reachable=False), []
    res = _coord(store, runner, drone, written).apply_link("both")
    assert runner.restarts == 1
    assert store.effective()["link"]["channel"] == 100
    assert res["gsApplied"] is True
    assert res["droneApplied"] is False
    assert res["droneReachable"] is False


def test_apply_gs_scope_skips_drone_even_if_reachable():
    store = _store()
    store.patch({"link": {"channel": 100}})
    runner, drone, written = FakeRunner(), FakeDrone(reachable=True), []
    res = _coord(store, runner, drone, written).apply_link("gs")
    assert drone.patched is None
    assert drone.applied is False
    assert runner.restarts == 1
    assert res["droneApplied"] is False
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_link.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.link'`.

- [ ] **Step 3: Implement `link.py`**

`gs/fpvdgs/link.py`:
```python
"""GS-local-first link coordinator.

A link change ALWAYS applies on the GS (it is how a link is established).
The drone push is best-effort, only for apply_to == "both" and only when the
drone is reachable — never a precondition.
"""

from .drone_client import DroneUnreachable

# Only the truly-shared radio params go to the drone. GS-only keys
# (region, wlans, txpower, beamforming) are per-side and never pushed.
DRONE_PUSH_KEYS = ("channel", "width", "linkId")


class LinkCoordinator:
    def __init__(self, store, renderer_write, runner, drone):
        # renderer_write(effective_cfg: dict) -> None  renders + writes the cfg file
        self.store = store
        self.renderer_write = renderer_write
        self.runner = runner
        self.drone = drone

    def apply_link(self, apply_to: str = "both") -> dict:
        pending = self.store.pending()
        link = pending.get("link", {})

        drone_applied = False
        drone_reachable = False
        if apply_to == "both":
            drone_reachable = self.drone.healthz()
            if drone_reachable:
                push = {k: link[k] for k in DRONE_PUSH_KEYS if k in link}
                try:
                    self.drone.patch_config({"link": push})
                    self.drone.apply()
                    drone_applied = True
                except DroneUnreachable:
                    drone_reachable = False

        # Apply the GS side unconditionally.
        self.store.commit()
        self.renderer_write(self.store.effective())
        gs_applied = self.runner.restart()

        return {
            "gsApplied": bool(gs_applied),
            "droneApplied": drone_applied,
            "droneReachable": drone_reachable,
            "inSync": drone_applied,
        }
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_link.py -v`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/link.py gs/tests/unit/test_link.py
git commit -m "feat(gs): LinkCoordinator (GS-local-first, best-effort drone push)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 10: HTTP API router

**Files:**
- Create: `gs/fpvdgs/api.py`
- Test: `gs/tests/unit/test_api.py`

`Api.handle` is transport-free (no sockets) so it unit-tests directly. The drone proxy forwards opaque bytes; `/link` uses the coordinator; `/config` rejects overlap.

- [ ] **Step 1: Write the failing tests**

`gs/tests/unit/test_api.py`:
```python
import json

from fpvdgs import schema, render as render_mod
from fpvdgs.config import ConfigStore
from fpvdgs.api import Api


class FakeRunner:
    def restart(self):
        return True

    def state(self):
        return {"running": True, "pid": 1, "restarts": 0, "lastExit": None, "fault": False}


class FakeDrone:
    def __init__(self):
        self.calls = []

    def healthz(self):
        return True

    def patch_config(self, d):
        self.calls.append(("PATCH", d))
        return {}

    def apply(self):
        self.calls.append(("POST", "/apply", None))
        return {}

    # opaque proxy hook (Api._proxy calls drone._request)
    def _request(self, method, path, body=None):
        self.calls.append((method, path, body))
        return 200, json.dumps({"proxied": path}).encode()


def _api(written):
    import os
    import tempfile
    cfg_out = os.path.join(tempfile.mkdtemp(), "wifibroadcast.cfg")
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"},
                         "wfb": {"profile": "gs"}, "drone": {"endpoint": "http://x"}},
                        overlay_path=None)
    from fpvdgs.link import LinkCoordinator
    drone = FakeDrone()
    runner = FakeRunner()
    link = LinkCoordinator(store, lambda cfg: written.append(cfg), runner, drone)
    return Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
               drone=drone, link=link, status_fn=lambda: {"ok": True},
               cfg_out=cfg_out), store, drone


def test_healthz():
    api, _, _ = _api([])
    code, _ = api.handle("GET", "/healthz", {}, b"")
    assert code == 200


def test_get_config_returns_effective():
    api, _, _ = _api([])
    code, obj = api.handle("GET", "/config", {}, b"")
    assert code == 200
    assert obj["link"]["channel"] == 132


def test_patch_config_rejects_link():
    api, _, _ = _api([])
    code, obj = api.handle("PATCH", "/config", {}, json.dumps({"link": {"channel": 100}}).encode())
    assert code == 400
    assert "link" in obj["error"]


def test_patch_config_then_apply():
    written = []
    api, store, _ = _api(written)
    code, _ = api.handle("PATCH", "/config", {},
                         json.dumps({"wfb": {"profile": "gs2"}}).encode())
    assert code == 200
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert store.effective()["wfb"]["profile"] == "gs2"
    assert written  # cfg rendered


def test_link_apply_both():
    written = []
    api, store, drone = _api(written)
    api.handle("PATCH", "/link", {}, json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/link/apply", {}, json.dumps({"applyTo": "both"}).encode())
    assert code == 200
    assert obj["gsApplied"] is True
    assert obj["droneApplied"] is True
    assert store.effective()["link"]["channel"] == 100


def test_air_config_is_proxied_opaquely():
    api, _, drone = _api([])
    code, raw = api.handle("PATCH", "/air/config", {}, b'{"video":{"bitrate":9000}}')
    assert code == 200
    assert any(c[0] == "PATCH" and c[1] == "/config" for c in drone.calls)
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_api.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.api'`.

- [ ] **Step 3: Implement `api.py`**

`gs/fpvdgs/api.py`:
```python
"""Single front-door HTTP API: /config /apply /reset /defaults /status /healthz,
opaque /air/* drone proxy, and /link coordinator. Transport-free `handle()`."""

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import urlparse, parse_qs

from .schema import SchemaError


class Api:
    def __init__(self, store, schema, render_mod, runner, drone, link, status_fn, cfg_out):
        self.store = store
        self.schema = schema
        self.render_mod = render_mod
        self.runner = runner
        self.drone = drone
        self.link = link
        self.status_fn = status_fn
        self.cfg_out = cfg_out

    def _json(self, body: bytes) -> dict:
        return json.loads(body or b"{}")

    def handle(self, method: str, path: str, query: dict, body: bytes):
        try:
            if path.startswith("/air/"):
                return self._proxy(method, path, body)
            key = (method, path)
            if key == ("GET", "/healthz"):
                return 200, {"ok": True}
            if key == ("GET", "/defaults"):
                return 200, self.store.defaults()
            if key == ("GET", "/config"):
                pending = query.get("pending", ["false"])[0] == "true"
                return 200, (self.store.pending() if pending else self.store.effective())
            if key == ("PATCH", "/config"):
                sparse = self._json(body)
                self.schema.validate_config_patch(sparse)
                self.store.patch(sparse)
                return 200, self.store.pending()
            if key == ("POST", "/apply"):
                return self._apply_gs()
            if key == ("POST", "/reset"):
                self.store.reset()
                self.render_mod.write_cfg(self.cfg_out,
                                          self.render_mod.render_cfg(self.store.effective()))
                self.runner.restart()
                return 200, {"reset": True}
            if key == ("GET", "/status"):
                return 200, self.status_fn()
            if key == ("GET", "/link"):
                return 200, self._link_view()
            if key == ("PATCH", "/link"):
                sparse = self._json(body)
                self.schema.validate_link_patch(sparse)
                self.store.patch(sparse)
                return 200, self.store.pending().get("link", {})
            if key == ("POST", "/link/apply"):
                apply_to = self._json(body).get("applyTo", "both")
                return 200, self.link.apply_link(apply_to)
            return 404, {"error": "not found"}
        except SchemaError as e:
            return 400, {"error": str(e)}
        except Exception as e:  # surfaced, never silent
            return 500, {"error": str(e)}

    def _apply_gs(self):
        pending = self.store.pending()
        # Guard: link drift must go through /link/apply (drone coordination).
        if pending.get("link") != self.store.effective().get("link"):
            return 409, {"error": "link changed; use POST /link/apply"}
        self.schema.validate_effective(pending)
        self.store.commit()
        self.render_mod.write_cfg(self.cfg_out,
                                  self.render_mod.render_cfg(self.store.effective()))
        ok = self.runner.restart()
        return (200 if ok else 500), {"applied": bool(ok)}

    def _link_view(self):
        link = dict(self.store.effective().get("link", {}))
        reachable = self.drone.healthz()
        link["droneReachable"] = reachable
        return link

    def _proxy(self, method, path, body):
        sub = path[len("/air"):]  # "/config", "/apply", "/status"
        endpoint_method = {"GET": "GET", "PATCH": "PATCH", "POST": "POST"}.get(method)
        if endpoint_method is None:
            return 405, {"error": "method not allowed"}
        try:
            code, raw = self.drone._request(endpoint_method, sub,
                                            self._json(body) if body else None)
            return code, json.loads(raw or b"{}")
        except Exception as e:
            return 502, {"error": f"drone unreachable: {e}"}


def make_http_server(api: Api, host: str, port: int) -> ThreadingHTTPServer:
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _dispatch(self, method):
            parsed = urlparse(self.path)
            n = int(self.headers.get("content-length", 0))
            body = self.rfile.read(n) if n else b""
            code, obj = api.handle(method, parsed.path, parse_qs(parsed.query), body)
            data = obj if isinstance(obj, (bytes, bytearray)) else json.dumps(obj).encode()
            self.send_response(code)
            self.send_header("content-type", "application/json")
            self.send_header("content-length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            self._dispatch("GET")

        def do_POST(self):
            self._dispatch("POST")

        def do_PATCH(self):
            self._dispatch("PATCH")

    return ThreadingHTTPServer((host, port), Handler)
```

Note: `_proxy` calls `drone._request`, which `DroneClient` provides. Tests pass a `FakeDrone.proxy`? No — they assert `drone.calls` via `patch_config`. Adjust the FakeDrone to expose `_request`:

In `test_api.py`, replace `FakeDrone.proxy` with:
```python
    def _request(self, method, path, body=None):
        self.calls.append((method, path, body))
        return 200, json.dumps({"proxied": path}).encode()
```
(Keep `patch_config`/`apply`/`healthz` as-is.)

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_api.py -v`
Expected: PASS (6 tests).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/api.py gs/tests/unit/test_api.py
git commit -m "feat(gs): HTTP API router (/config /apply /link, opaque /air proxy)

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 11: Runner entrypoint (wfb_ng-backed)

**Files:**
- Create: `gs/fpvdgs/runner.py`
- Test: `gs/tests/unit/test_runner.py`

The runner sets `WIFIBROADCAST_CFG` (the supervisor sets it in the child env at spawn) and calls `wfb_ng.server.main()` with assembled argv. `wfb_ng` may be absent on the dev host, so the test only checks argv assembly + the env guard.

- [ ] **Step 1: Write the failing tests**

`gs/tests/unit/test_runner.py`:
```python
import pytest

from fpvdgs import runner


def test_build_argv():
    assert runner.build_argv("gs", ["wlan0", "wlan1"]) == [
        "--profiles", "gs", "--wlans", "wlan0", "wlan1"]


def test_main_requires_cfg_env(monkeypatch):
    monkeypatch.delenv("WIFIBROADCAST_CFG", raising=False)
    with pytest.raises(SystemExit):
        runner.main(["--profiles", "gs", "--wlans", "wlan0"])
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_runner.py -v`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.runner'`.

- [ ] **Step 3: Implement `runner.py`**

`gs/fpvdgs/runner.py`:
```python
"""fpvd-runner: the wfb data plane, built on the wfb_ng library.

Spawned and supervised by the fpvd supervisor. WIFIBROADCAST_CFG must already
be in the environment (the supervisor sets it before spawn) so that wfb_ng.conf
parses our rendered cfg at import time.
"""

import os
import sys


def build_argv(profile: str, wlans: list[str]) -> list[str]:
    return ["--profiles", profile, "--wlans", *wlans]


def main(argv: list[str] | None = None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    if "WIFIBROADCAST_CFG" not in os.environ:
        sys.stderr.write("fpvd-runner: WIFIBROADCAST_CFG not set\n")
        raise SystemExit(2)
    from wfb_ng import server  # noqa: E402  (cfg parsed at import; env already set)
    sys.argv = ["fpvd-runner", *argv]
    server.main()


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/unit/test_runner.py -v`
Expected: PASS (2 tests). (The `wfb_ng` import is never reached in these tests.)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/runner.py gs/tests/unit/test_runner.py
git commit -m "feat(gs): wfb-runner entrypoint on the wfb_ng library

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 12: Supervisor main + end-to-end integration

**Files:**
- Create: `gs/fpvdgs/supervisor.py`
- Test: `gs/tests/integration/__init__.py`, `gs/tests/integration/test_supervisor_e2e.py`

- [ ] **Step 1: Write the failing integration test**

`gs/tests/integration/__init__.py`: (empty)

`gs/tests/integration/test_supervisor_e2e.py`:
```python
import json
import socket
import threading
import time
import urllib.request

import pytest

from fpvdgs import supervisor


def _free_port():
    s = socket.socket()
    s.bind(("127.0.0.1", 0))
    p = s.getsockname()[1]
    s.close()
    return p


def _req(base, method, path, body=None):
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request(base + path, data=data, method=method)
    if data:
        req.add_header("content-type", "application/json")
    with urllib.request.urlopen(req, timeout=3) as r:
        return r.status, json.loads(r.read() or b"{}")


@pytest.fixture
def daemon(tmp_path, fake_drone):
    defaults = tmp_path / "defaults.json"
    overlay = tmp_path / "config.json"
    cfg_out = tmp_path / "wifibroadcast.cfg"
    ready_port = _free_port()
    api_port = _free_port()
    defaults.write_text(json.dumps({
        "link": {"channel": 132, "width": 40, "region": "US", "txpower": 19,
                 "linkId": 7669206, "wlans": ["wlan0"]},
        "wfb": {"profile": "gs"},
        "drone": {"endpoint": fake_drone["endpoint"]},
    }))
    # a fake runner that just opens the readiness port
    fake_runner = ["python3", "-c",
                   ("import socket,time;s=socket.socket();"
                    "s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1);"
                    f"s.bind(('127.0.0.1',{ready_port}));s.listen(1);time.sleep(30)")]
    app = supervisor.build_app(
        defaults_path=str(defaults), overlay_path=str(overlay),
        cfg_out=str(cfg_out), host="127.0.0.1", port=api_port,
        runner_cmd=fake_runner, ready_port=ready_port, ready_timeout=5.0)
    app.start()
    t = threading.Thread(target=app.serve_forever, daemon=True)
    t.start()
    time.sleep(0.3)
    base = f"http://127.0.0.1:{api_port}"
    yield base, cfg_out, fake_drone
    app.shutdown()


def test_healthz_and_status(daemon):
    base, _, _ = daemon
    assert _req(base, "GET", "/healthz")[0] == 200
    code, st = _req(base, "GET", "/status")
    assert code == 200
    assert st["runner"]["running"] is True


def test_cfg_rendered_on_boot(daemon):
    _, cfg_out, _ = daemon
    text = cfg_out.read_text()
    assert "wifi_channel = 132" in text


def test_link_apply_pushes_drone_and_rerenders(daemon):
    base, cfg_out, fake_drone = daemon
    _req(base, "PATCH", "/link", {"link": {"channel": 100}})
    code, obj = _req(base, "POST", "/link/apply", {"applyTo": "both"})
    assert code == 200 and obj["droneApplied"] is True
    assert "wifi_channel = 100" in cfg_out.read_text()
    assert any(m == "PATCH" and p == "/config" for (m, p, _b) in fake_drone["calls"])


def test_air_proxy_roundtrip(daemon):
    base, _, fake_drone = daemon
    code, _ = _req(base, "PATCH", "/air/config", {"video": {"bitrate": 9000}})
    assert code == 200
    assert any(p == "/config" for (_m, p, _b) in fake_drone["calls"])
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd gs && python -m pytest tests/integration/test_supervisor_e2e.py -v`
Expected: FAIL — `AttributeError: module 'fpvdgs.supervisor' has no attribute 'build_app'`.

- [ ] **Step 3: Implement `supervisor.py`**

`gs/fpvdgs/supervisor.py`:
```python
"""fpvd supervisor: owns config + HTTP API + runner supervision. Pure stdlib."""

import argparse
import sys

from . import __version__, render as render_mod, schema, status as status_mod
from .api import Api, make_http_server
from .config import ConfigStore
from .drone_client import DroneClient
from .link import LinkCoordinator
from .runner_supervisor import RunnerSupervisor, resolve_wlans


class App:
    def __init__(self, store, runner, http_server):
        self.store = store
        self.runner = runner
        self.http = http_server

    def start(self):
        self.runner.start()

    def serve_forever(self):
        self.http.serve_forever()

    def shutdown(self):
        self.http.shutdown()
        self.runner.stop()


def build_app(defaults_path, overlay_path, cfg_out, host, port,
              runner_cmd, ready_port=8103, ready_timeout=10.0, log_path=None):
    store = ConfigStore.load(defaults_path, overlay_path)
    effective = store.effective()
    schema.validate_effective(effective)

    # Render the cfg the runner will read.
    render_mod.write_cfg(cfg_out, render_mod.render_cfg(effective))

    profile = effective.get("wfb", {}).get("profile", "gs")
    wlans = resolve_wlans(effective)
    runner = RunnerSupervisor(runner_cmd, cfg_out=cfg_out, profile=profile,
                              wlans=wlans, ready_port=ready_port,
                              ready_timeout=ready_timeout, log_path=log_path)

    drone = DroneClient(effective.get("drone", {}).get("endpoint", "http://10.5.0.10:8080"))

    def renderer_write(eff):
        render_mod.write_cfg(cfg_out, render_mod.render_cfg(eff))

    link = LinkCoordinator(store, renderer_write, runner, drone)

    def status_fn():
        wlan_info = {w: status_mod.iw_info(w) for w in resolve_wlans(store.effective())}
        reachable = drone.healthz()
        eff_link = store.effective().get("link", {})
        probe = {"reachable": reachable, "linkId": eff_link.get("linkId"),
                 "inSync": None}
        return status_mod.build_status(__version__, runner.state(), wlan_info, probe)

    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone, link=link, status_fn=status_fn, cfg_out=cfg_out)

    http_server = make_http_server(api, host, port)
    return App(store, runner, http_server)


def main(argv=None):
    p = argparse.ArgumentParser(prog="fpvd")
    p.add_argument("--defaults", default="/etc/fpvd/defaults.json")
    p.add_argument("--config", default="/etc/fpvd/config.json")
    p.add_argument("--cfg-out", default="/etc/wifibroadcast.cfg")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--log", default=None)
    p.add_argument("--runner", default=None,
                   help="runner command (default: this python -m fpvdgs.runner)")
    args = p.parse_args(argv)

    runner_cmd = (args.runner.split() if args.runner
                  else [sys.executable, "-m", "fpvdgs.runner"])
    app = build_app(args.defaults, args.config, args.cfg_out, args.host, args.port,
                    runner_cmd, log_path=args.log)
    app.start()
    sys.stderr.write(f"fpvd: listening on {args.host}:{args.port}\n")
    try:
        app.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()


if __name__ == "__main__":
    main()
```

Note: `cfg_out` is passed explicitly to `Api` (constructor) and captured by the `renderer_write` closure given to `LinkCoordinator`, so both render to the same configured path with no global state.

- [ ] **Step 4: Run to verify it passes**

Run: `cd gs && python -m pytest tests/integration/test_supervisor_e2e.py -v`
Expected: PASS (4 tests).

- [ ] **Step 5: Run the whole suite**

Run: `cd gs && python -m pytest -q`
Expected: all unit + integration tests PASS.

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/supervisor.py gs/tests/integration
git commit -m "feat(gs): supervisor main + end-to-end integration tests

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 13: Defaults baseline + init script

**Files:**
- Create: `gs/etc/defaults.json`
- Create: `gs/scripts/S99fpvd`

- [ ] **Step 1: Write the GS defaults (seeded from the live GS)**

`gs/etc/defaults.json`:
```json
{
  "link": {
    "channel": 132,
    "width": 40,
    "txpower": 19,
    "region": "US",
    "linkId": 7669206,
    "beamforming": { "enabled": false },
    "wlans": "auto"
  },
  "wfb": {
    "profile": "gs",
    "mavlink": { "peer": "connect://127.0.0.1:14550" },
    "raw": {}
  },
  "drone": {
    "endpoint": "http://10.5.0.10:8080"
  }
}
```

- [ ] **Step 2: Write the init script**

`gs/scripts/S99fpvd`:
```sh
#!/bin/sh
DAEMON=fpvd
PIDFILE=/var/run/fpvd.pid
LOG=/tmp/fpvd.log
ARGS="--defaults /etc/fpvd/defaults.json --config /etc/fpvd/config.json --cfg-out /etc/wifibroadcast.cfg --port 8080 --log $LOG"

start() {
    printf 'Starting %s: ' "$DAEMON"
    modprobe tun 2>/dev/null
    start-stop-daemon -S -q -b -m -p "$PIDFILE" -x /usr/bin/fpvd -- $ARGS
    [ $? = 0 ] && echo "OK" || echo "FAIL"
}
stop() {
    printf 'Stopping %s: ' "$DAEMON"
    start-stop-daemon -K -q -p "$PIDFILE"
    [ $? = 0 ] && echo "OK" || echo "FAIL"
}
case "$1" in
    start)   start ;;
    stop)    stop ;;
    restart) stop; sleep 1; start ;;
    *) echo "Usage: $0 {start|stop|restart}"; exit 1 ;;
esac
```

- [ ] **Step 3: Verify the defaults validate + render**

Run:
```bash
cd gs && python3 -c "
import json
from fpvdgs import schema, render
cfg = json.load(open('etc/defaults.json'))
schema.validate_effective(cfg)
print(render.render_cfg(cfg))
"
```
Expected: prints a wifibroadcast.cfg with `[common] wifi_channel = 132`, `wifi_region = 'US'`, `[gs_video] bandwidth = 40`, `[gs_mavlink] peer = connect://127.0.0.1:14550`. No validation error.

- [ ] **Step 4: Commit**

```bash
chmod +x gs/scripts/S99fpvd
git add gs/etc/defaults.json gs/scripts/S99fpvd
git commit -m "feat(gs): defaults baseline + S99fpvd init script

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 14: Deploy + rollback scripts

**Files:**
- Create: `deploy/gs/deploy.sh`
- Create: `deploy/gs/rollback.sh`

- [ ] **Step 1: Write the deploy script**

`deploy/gs/deploy.sh`:
```bash
#!/usr/bin/env bash
# deploy/gs/deploy.sh — install fpvd (GS) onto an OpenIPC SBC ground station.
#
# Pure Python: no build. Copies the fpvdgs package + init script, backs up and
# disables the stock S98wifibroadcast (wfb-server), then starts fpvd.
#
# Usage: ./deploy/gs/deploy.sh [--host IP] [--user USER]
# Env overrides: GS_HOST, GS_USER.
set -euo pipefail

GS_HOST="${GS_HOST:-10.18.0.1}"
GS_USER="${GS_USER:-root}"
while [ $# -gt 0 ]; do
    case "$1" in
        --host) GS_HOST="$2"; shift 2 ;;
        --user) GS_USER="$2"; shift 2 ;;
        -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
GS="$REPO/gs"
TARGET="${GS_USER}@${GS_HOST}"
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o LogLevel=error)
remote() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }

# Install into the GS's real site-packages (where wfb_ng already lives) so that
# both `python3 -m fpvdgs.supervisor` and the spawned `python3 -m fpvdgs.runner`
# import without any sys.path/PYTHONPATH hacks.
SITE="$(remote 'python3 -c "import site; print(site.getsitepackages()[0])"')"
echo "[push] fpvdgs -> $TARGET:$SITE/fpvdgs  (+ init + defaults)"
remote "mkdir -p /etc/fpvd '$SITE/fpvdgs'"
scp -O "${SSH_OPTS[@]}" "$GS/fpvdgs"/*.py "$TARGET:$SITE/fpvdgs/"
# defaults (do not clobber an existing user overlay /etc/fpvd/config.json)
scp -O "${SSH_OPTS[@]}" "$GS/etc/defaults.json" "$TARGET:/etc/fpvd/defaults.json"
scp -O "${SSH_OPTS[@]}" "$GS/scripts/S99fpvd"  "$TARGET:/etc/init.d/S99fpvd"

echo "[install] fpvd launcher + backup/disable S98wifibroadcast"
remote '
    set -e
    # launcher: `fpvd` runs the now-importable package entrypoint
    printf "#!/bin/sh\nexec python3 -m fpvdgs.supervisor \"\$@\"\n" > /usr/bin/fpvd
    chmod +x /usr/bin/fpvd /etc/init.d/S99fpvd

    mkdir -p /root/fpvd-gs-rollback
    cp -a /etc/wifibroadcast.cfg /root/fpvd-gs-rollback/wifibroadcast.cfg.orig 2>/dev/null || true
    [ -f /etc/init.d/S98wifibroadcast ] && cp -a /etc/init.d/S98wifibroadcast /root/fpvd-gs-rollback/ || true
    [ -x /etc/init.d/S98wifibroadcast ] && /etc/init.d/S98wifibroadcast stop >/dev/null 2>&1 || true
    sleep 2
    rm -f /etc/init.d/S98wifibroadcast
    : > /tmp/fpvd.log
    /etc/init.d/S99fpvd start
'

echo "[verify]"
sleep 5
remote '
    printf "  procs: "; for p in fpvd wfb_rx wfb_tx; do
        printf "%s=%s " "$p" "$(pidof $p 2>/dev/null | cut -d" " -f1 || echo -)"; done; echo
    printf "  api:   "; curl -s http://127.0.0.1:8080/status | head -c 200; echo
    printf "  8103:  "; (echo > /dev/tcp/127.0.0.1/8103) 2>/dev/null && echo open || echo closed
'
echo "[done] fpvd (GS) deployed to $GS_HOST. Rollback: deploy/gs/rollback.sh --host $GS_HOST"
```

- [ ] **Step 2: Write the rollback script**

`deploy/gs/rollback.sh`:
```bash
#!/usr/bin/env bash
# deploy/gs/rollback.sh — restore the stock S98wifibroadcast on the GS.
set -euo pipefail
GS_HOST="${GS_HOST:-10.18.0.1}"; GS_USER="${GS_USER:-root}"
while [ $# -gt 0 ]; do case "$1" in
    --host) GS_HOST="$2"; shift 2 ;; --user) GS_USER="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;; esac; done
TARGET="${GS_USER}@${GS_HOST}"
ssh -o StrictHostKeyChecking=accept-new "$TARGET" '
    set -e
    [ -x /etc/init.d/S99fpvd ] && /etc/init.d/S99fpvd stop >/dev/null 2>&1 || true
    rm -f /etc/init.d/S99fpvd /usr/bin/fpvd
    SITE="$(python3 -c "import site; print(site.getsitepackages()[0])")"
    rm -rf "$SITE/fpvdgs"
    if [ -f /root/fpvd-gs-rollback/S98wifibroadcast ]; then
        cp -a /root/fpvd-gs-rollback/S98wifibroadcast /etc/init.d/S98wifibroadcast
        cp -a /root/fpvd-gs-rollback/wifibroadcast.cfg.orig /etc/wifibroadcast.cfg 2>/dev/null || true
        chmod +x /etc/init.d/S98wifibroadcast
        /etc/init.d/S98wifibroadcast start
    fi
    echo rollback-done
'
```

- [ ] **Step 3: Lint the scripts**

Run: `bash -n deploy/gs/deploy.sh && bash -n deploy/gs/rollback.sh && echo OK`
Expected: `OK` (no syntax errors).

- [ ] **Step 4: Commit**

```bash
chmod +x deploy/gs/deploy.sh deploy/gs/rollback.sh
git add deploy/gs/deploy.sh deploy/gs/rollback.sh
git commit -m "feat(gs): deploy + rollback scripts for the ground station

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 15: README + on-device smoke checklist

**Files:**
- Create: `gs/README.md`

- [ ] **Step 1: Write the GS README**

`gs/README.md`:
```markdown
# fpvd (ground station)

Python peer to the drone `fpvd`. Owns the GS wfb radio config and supervises
the wfb data plane (built on the `wfb_ng` library, replacing `wfb-server`).
Single front-door HTTP API on `:8080`.

See `../docs/superpowers/specs/2026-06-02-fpvd-gs-design.md`.

## Test (dev host)

    cd gs && python -m pytest -q

## Deploy (GS)

    ./deploy/gs/deploy.sh --host 10.18.0.1
    # rollback:
    ./deploy/gs/rollback.sh --host 10.18.0.1

## API

| Method | Path | Behavior |
|---|---|---|
| GET | /config[?pending=true] | effective or pending GS config |
| PATCH | /config | merge sparse JSON into pending (link.* rejected) |
| POST | /apply | commit pending → effective; render cfg; bounce runner |
| POST | /reset | drop overlay |
| GET | /defaults | baseline |
| GET | /status | daemon + runner/radio/link state |
| GET | /healthz | 200 |
| GET/PATCH | /air/config, POST /air/apply, GET /air/status | opaque proxy to drone fpvd |
| GET | /link | overlap params + droneReachable |
| PATCH/POST | /link, /link/apply | GS-local-first link change; applyTo "gs"|"both" |

## On-device smoke (run after deploy; needs the drone reachable for /air and /link "both")

1. `pidof fpvd wfb_rx wfb_tx` — all present; no `wfb-server`/`S98wifibroadcast`.
2. `curl -s :8080/status` — runner.running true; radio shows channel/width per wlan.
3. `(echo>/dev/tcp/127.0.0.1/8103)` — open; dynamic-link-gs still connected; video flowing.
4. GS-local: `curl -XPATCH :8080/config -d '{"wfb":{"mavlink":{"peer":"connect://127.0.0.1:14550"}}}'` then `curl -XPOST :8080/apply` — 200; only the runner bounced.
5. Link bootstrap (drone reachable): `curl -XPATCH :8080/link -d '{"link":{"channel":100}}'` then `curl -XPOST :8080/link/apply -d '{"applyTo":"both"}'` — `{gsApplied:true,droneApplied:true}`; link re-establishes on the new channel.
6. Link bootstrap (drone offline / different channel): same with `{"applyTo":"gs"}` — `droneApplied:false`, GS moves to the drone's channel and the link comes up.
7. `/air`: `curl :8080/air/status` round-trips the drone fpvd's status.
```

- [ ] **Step 2: Commit**

```bash
git add gs/README.md
git commit -m "docs(gs): README + on-device smoke checklist

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Final verification

- [ ] **Run the full GS suite:** `cd gs && python -m pytest -q` → all pass.
- [ ] **Confirm the drone build still works post-relocation:** `cmake -S drone -B drone/build/host -DCMAKE_BUILD_TYPE=Debug && cmake --build drone/build/host -j && ./drone/build/host/fpvd_tests` → pass.
- [ ] **On-device smoke** (per `gs/README.md`) once the drone is powered on and the GS is reachable.

---

## Notes / deferred (per spec §14)

- Coordinated/seamless switching (timed handshake + auto-revert), live `iw`
  retune without a runner bounce, beamforming activation, and 10 MHz width are
  **future work** — `/link` is their home. Not in this plan.
- gsmenu re-pointing (mapping its `get/set air|gs …` onto `/config`, `/air/*`,
  `/link`) is a separate follow-up on the GS image side, tracked in the spec's
  client-mapping section.
