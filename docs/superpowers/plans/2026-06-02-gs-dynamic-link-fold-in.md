# GS dynamic-link fold-in — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fold the ground-station side of the standalone `dynamic-link` project into `fpvd`'s GS daemon as an in-process asyncio control loop on a daemon thread, configured through fpvd's existing `PATCH /config` + `POST /apply` and applied at runtime with **no wfb restart**.

**Architecture:** Lift the dynamic-link GS *core* (policy/signals/stats/fec/bitrate/predictor/profile/wire/return_link/drone_config/tunnel_listener) verbatim into a new `fpvdgs/dynlink/` package. Wrap it in a `DynamicLinkController` that owns one daemon thread running an asyncio event loop, exposing a thread-safe `start()/stop()/set_config()/status()` surface to the (thread-based, blocking-HTTP) supervisor. `dynamicLink` config flows through the normal config store; `POST /apply` diffs subsystems and routes dynamic-link deltas to the controller without ever bouncing the wfb runner.

**Tech Stack:** Python 3.11+, stdlib only (`asyncio` is stdlib — no PyYAML, no `wfb_ng` package). pytest (no `pytest-asyncio` needed — the controller is tested through its thread-safe public API).

**Spec:** `docs/superpowers/specs/2026-06-02-gs-dynamic-link-fold-in-design.md`

**Source repo (migration source):** `/home/gilankpam/Projects/drone/dynamic-link/gs/dynamic_link/`

---

## File Structure

Created under `gs/fpvdgs/dynlink/`:

| File | Responsibility |
| --- | --- |
| `__init__.py` | package marker |
| `decision.py` | (lifted) Decision dataclass |
| `predictor.py` | (lifted) latency-budget gate |
| `dynamic_fec.py` | (lifted) `(k,n)` compute, NEscalator, EmitGate |
| `bitrate.py` | (lifted) wire-target → bitrate |
| `stats_client.py` | (lifted) async wfb-ng JSON stats client |
| `signals.py` | (lifted) signal aggregator / EWMA |
| `return_link.py` | (lifted) non-blocking UDP sender |
| `wire.py` | (lifted) DLK1 v2 encoder + HELLO/PING codecs |
| `drone_config.py` | (lifted) P4a HELLO handshake state |
| `tunnel_listener.py` | (lifted) GS UDP listener (DLHE receive) |
| `policy.py` | (lifted) dual-gate selector + trailing loop |
| `profile.py` | (lifted, de-YAML'd) radio-profile loader — JSON |
| `profiles/m8812eu2.json` | (converted) radio profile data |
| `config_build.py` | **new** — fpvd `dynamicLink` block → PolicyConfig/aggregator/profile/snapshot |
| `controller.py` | **new** — `DynamicLinkController` (thread + asyncio loop) |

Modified under `gs/fpvdgs/`:

| File | Change |
| --- | --- |
| `schema.py` | allow `dynamicLink` in config patches; validate curated keys |
| `etc/defaults.json` | add `dynamicLink` block |
| `render.py` | emit `log_interval = 100` unconditionally |
| `api.py` | `_apply_gs` diffs subsystems; routes dynamic-link delta to controller |
| `status.py` | `build_status` accepts a `dynamic_link` section |
| `supervisor.py` | construct controller; start/stop in `App`; assemble status |

Tests under `gs/tests/unit/` (ported tests prefixed `test_dl_`) and `gs/tests/integration/`.

---

## Conventions

- All commands run from the GS package root unless noted: `cd /home/gilankpam/Projects/drone/fpvd/gs`
- Run tests with `python -m pytest` (uses `pyproject.toml` `pythonpath = ["."]`).
- Commit after every task. Branch is already `gs-dynamic-link-fold-in`.

---

### Task 1: Scaffold `dynlink/` package and lift the stdlib-only core modules

These eleven modules import only stdlib + each other (verified import graph). They are copied **verbatim**; only intra-package import prefixes change from `dynamic_link.` to `fpvdgs.dynlink.`.

**Files:**
- Create: `gs/fpvdgs/dynlink/__init__.py`
- Create (copied): `decision.py`, `predictor.py`, `dynamic_fec.py`, `bitrate.py`, `stats_client.py`, `signals.py`, `return_link.py`, `wire.py`, `drone_config.py`, `tunnel_listener.py`, `policy.py`
- Test: `gs/tests/unit/test_dl_imports.py`

- [ ] **Step 1: Write the failing import smoke test**

```python
# gs/tests/unit/test_dl_imports.py
"""Smoke test: every lifted dynlink core module imports cleanly (no PyYAML,
no wfb_ng, no leftover dynamic_link.* imports)."""

import importlib

import pytest

MODULES = [
    "decision", "predictor", "dynamic_fec", "bitrate", "stats_client",
    "signals", "return_link", "wire", "drone_config", "tunnel_listener",
    "policy",
]


@pytest.mark.parametrize("name", MODULES)
def test_module_imports(name):
    mod = importlib.import_module(f"fpvdgs.dynlink.{name}")
    assert mod is not None


def test_no_yaml_dependency_in_core():
    # None of the core modules may pull PyYAML.
    import sys
    for name in MODULES:
        importlib.import_module(f"fpvdgs.dynlink.{name}")
    assert "yaml" not in sys.modules
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && python -m pytest tests/unit/test_dl_imports.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.dynlink'`

- [ ] **Step 3: Create the package and copy the core modules with import rewrite**

```bash
cd /home/gilankpam/Projects/drone/fpvd/gs
SRC=/home/gilankpam/Projects/drone/dynamic-link/gs/dynamic_link
mkdir -p fpvdgs/dynlink
: > fpvdgs/dynlink/__init__.py
for m in decision predictor dynamic_fec bitrate stats_client signals \
         return_link wire drone_config tunnel_listener policy; do
  sed 's/\bdynamic_link\./fpvdgs.dynlink./g' "$SRC/$m.py" > "fpvdgs/dynlink/$m.py"
done
```

Note: intra-package imports in these files use the relative form `from .x import ...`, which is unaffected by the package move. The `sed` only rewrites any absolute `dynamic_link.` references (e.g. in docstrings/log names). Verify none remain:

```bash
grep -rn "dynamic_link" fpvdgs/dynlink/ || echo "clean"
```
Expected: `clean`

- [ ] **Step 4: Run the test to verify it passes**

Run: `python -m pytest tests/unit/test_dl_imports.py -q`
Expected: PASS (12 passed)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink gs/tests/unit/test_dl_imports.py
git commit -m "dynlink: lift stdlib-only GS core modules into fpvdgs.dynlink"
```

---

### Task 2: De-YAML the profile loader and convert the radio profile to JSON

`profile.py` is the only core module that imported `yaml`. Switch it to JSON and convert the one packaged profile. The validator already coerces keys with `int(...)`, so JSON string keys (`"20"`, `"0"`) load correctly.

**Files:**
- Modify: `gs/fpvdgs/dynlink/profile.py`
- Create: `gs/fpvdgs/dynlink/profiles/m8812eu2.json`
- Test: `gs/tests/unit/test_dl_profile.py`

- [ ] **Step 1: Write the failing test**

```python
# gs/tests/unit/test_dl_profile.py
from pathlib import Path

import pytest

from fpvdgs.dynlink.profile import ProfileError, RadioProfile, load_profile

PROFILES = Path(__file__).resolve().parents[2] / "fpvdgs" / "dynlink" / "profiles"


def test_load_m8812eu2_from_json():
    p = load_profile("m8812eu2", [PROFILES])
    assert isinstance(p, RadioProfile)
    assert p.name == "BL-M8812EU2"
    assert p.chipset == "RTL8812EU"
    assert p.bandwidth_supported == (20, 40)
    # JSON string keys must have been coerced back to ints.
    assert p.snr_floor_dB[20][0] == 5.0
    assert p.data_rate_Mbps_LGI[40][5] == 108.0


def test_missing_profile_raises():
    with pytest.raises(ProfileError):
        load_profile("does_not_exist", [PROFILES])
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python -m pytest tests/unit/test_dl_profile.py -q`
Expected: FAIL — `load_profile` looks for `m8812eu2.yaml` (and `import yaml` may still be present).

- [ ] **Step 3: Convert the loader to JSON**

In `gs/fpvdgs/dynlink/profile.py`, replace the import and the two loader functions:

Replace:
```python
import yaml
```
with:
```python
import json
```

Replace `load_profile` / `load_profile_file`:
```python
def load_profile(name: str, search_dirs: list[Path]) -> RadioProfile:
    """Resolve `<dir>/<name>.json` in order; load and validate the first hit."""
    for d in search_dirs:
        candidate = Path(d) / f"{name}.json"
        if candidate.is_file():
            return load_profile_file(candidate)
    searched = ", ".join(str(p) for p in search_dirs)
    raise ProfileError(f"radio profile {name!r} not found in: {searched}")


def load_profile_file(path: Path) -> RadioProfile:
    with open(path, "r") as fd:
        data = json.load(fd)
    if not isinstance(data, dict):
        raise ProfileError(f"{path}: top-level must be a mapping")
    return _validate(data, source=str(path))
```

- [ ] **Step 4: Create the JSON profile**

```json
// gs/fpvdgs/dynlink/profiles/m8812eu2.json
{
  "name": "BL-M8812EU2",
  "chipset": "RTL8812EU",
  "mcs_min": 0,
  "mcs_max": 5,
  "bandwidth_supported": [20, 40],
  "bandwidth_default": 20,
  "tx_power_min_dBm": 0,
  "tx_power_max_dBm": 30,
  "data_rate_Mbps_LGI": {
    "20": {"0": 6.5, "1": 13.0, "2": 19.5, "3": 26.0, "4": 39.0, "5": 52.0, "6": 58.5, "7": 65.0},
    "40": {"0": 13.5, "1": 27.0, "2": 40.5, "3": 54.0, "4": 81.0, "5": 108.0, "6": 121.5, "7": 135.0}
  },
  "snr_floor_dB": {
    "20": {"0": 5, "1": 8, "2": 11, "3": 14, "4": 17, "5": 20, "6": 23, "7": 25},
    "40": {"0": 8, "1": 11, "2": 14, "3": 17, "4": 20, "5": 23, "6": 26, "7": 28}
  },
  "preamble_us_per_frame": 170
}
```

Strip the `//` comment line above before saving — it is a label, not valid JSON.

- [ ] **Step 5: Run the test to verify it passes**

Run: `python -m pytest tests/unit/test_dl_profile.py -q`
Expected: PASS (2 passed)

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/dynlink/profile.py gs/fpvdgs/dynlink/profiles gs/tests/unit/test_dl_profile.py
git commit -m "dynlink: load radio profiles from JSON (drop PyYAML)"
```

---

### Task 3: Port the core policy/signal/fec unit tests

These are synchronous tests of the lifted compute core. Copy them with the same import rewrite plus a profile-dir fix. No bodies change.

**Files:**
- Create (ported): `gs/tests/unit/test_dl_policy_leading.py`, `test_dl_policy_trailing.py`, `test_dl_dynamic_fec.py`, `test_dl_policy_dynamic_fec_e2e.py`, `test_dl_bitrate.py`, `test_dl_predictor.py`, `test_dl_signals.py`, `test_dl_drone_config.py`

- [ ] **Step 1: Copy and rewrite the ported tests**

```bash
cd /home/gilankpam/Projects/drone/fpvd/gs
SRCT=/home/gilankpam/Projects/drone/dynamic-link/tests
for t in policy_leading policy_trailing dynamic_fec policy_dynamic_fec_e2e \
         bitrate predictor signals drone_config; do
  sed -e 's/\bdynamic_link\./fpvdgs.dynlink./g' \
      -e 's/from dynamic_link\b/from fpvdgs.dynlink/g' \
      "$SRCT/test_$t.py" > "tests/unit/test_dl_$t.py"
done
```

- [ ] **Step 2: Fix the radio-profile directory constant in the ported tests**

Any ported test that loads a radio profile defines `REPO_ROOT`/`PACKAGED_DIR` pointing at the old `conf/radios`. Replace those with the new JSON profiles dir. In each ported file that references `conf` / `radios`, set the constant to:

```python
from pathlib import Path
PACKAGED_DIR = Path(__file__).resolve().parents[2] / "fpvdgs" / "dynlink" / "profiles"
```

Find them:
```bash
grep -ln "radios\|PACKAGED_DIR\|REPO_ROOT" tests/unit/test_dl_*.py
```
Edit each hit so the profile search dir is `PACKAGED_DIR` above (and remove any now-unused `REPO_ROOT`). `load_profile(name, [PACKAGED_DIR])` then resolves `<name>.json`.

- [ ] **Step 3: Run the ported tests**

Run: `python -m pytest tests/unit/test_dl_policy_leading.py tests/unit/test_dl_policy_trailing.py tests/unit/test_dl_dynamic_fec.py tests/unit/test_dl_policy_dynamic_fec_e2e.py tests/unit/test_dl_bitrate.py tests/unit/test_dl_predictor.py tests/unit/test_dl_signals.py tests/unit/test_dl_drone_config.py -q`
Expected: PASS (all green). If a test fails only on the profile path, re-check Step 2 for that file.

- [ ] **Step 4: Commit**

```bash
git add gs/tests/unit/test_dl_*.py
git commit -m "dynlink: port core policy/signal/fec unit tests"
```

---

### Task 4: Wire-contract golden test (pinned from the C/C++ authority)

The standalone repo's contract test shells out to `drone/build/dl-inject`. fpvd's drone is C++ with no such CLI, so instead pin the **golden bytes** captured from the authority (`dl-inject --dry-run`); the fpvd drone's `dynlink/wire.cpp` is a port of the same `dl_wire.c`, so matching these proves GS↔drone agreement. The lifted Python encoder has been verified to reproduce these exactly.

**Files:**
- Test: `gs/tests/unit/test_dl_wire_contract.py`

- [ ] **Step 1: Write the failing test with pinned goldens**

```python
# gs/tests/unit/test_dl_wire_contract.py
"""Wire-format contract: the GS DLK1 v2 encoder must produce bytes that the
drone's dynlink/wire.cpp decoder accepts. Goldens were captured from the
authoritative C encoder (dynamic-link `dl-inject --dry-run`); fpvd's C++
decoder is a port of the same dl_wire.c. Do not regenerate these from the
Python encoder — that would make the test circular."""
from fpvdgs.dynlink.decision import Decision
from fpvdgs.dynlink.wire import (
    Hello, HelloAck, encode, encode_hello, encode_hello_ack,
)

# Decisions are 31 bytes (62 hex); HELLO / HELLO-ACK are 32 bytes (64 hex).
GOLDEN_DECISION_1 = "444c4b31020000000000000100000001051412080e022ee0000000a34fec51"
GOLDEN_DECISION_NEG = "444c4b310200000000000007000000070014f602040107d000000086b0d80c"
GOLDEN_DECISION_MAX = "444c4b3102000000ffffffffffffffff07281e081003fde8000000a092ca14"
GOLDEN_HELLO = "444c484502000000cafebabe0f9a003cdeadbeef0000000000000000b193a0b1"
GOLDEN_HELLO_ACK = "444c48410200000012345678000000000000000000000000000000005286d325"


def _decision(**overrides) -> Decision:
    base = Decision(timestamp=0.0, mcs=5, bandwidth=20, tx_power_dBm=18,
                    k=8, n=14, depth=2, bitrate_kbps=12000)
    for k, v in overrides.items():
        setattr(base, k, v)
    return base


def test_decision_golden():
    assert encode(_decision(), sequence=1).hex() == GOLDEN_DECISION_1


def test_decision_signed_tx_power():
    pkt = encode(_decision(mcs=0, tx_power_dBm=-10, k=2, n=4, depth=1,
                           bitrate_kbps=2000), sequence=7)
    assert pkt.hex() == GOLDEN_DECISION_NEG
    assert pkt[18] == 0xF6   # two's-complement -10


def test_decision_max_values():
    pkt = encode(_decision(mcs=7, bandwidth=40, tx_power_dBm=30, k=8, n=16,
                           depth=3, bitrate_kbps=65000), sequence=0xFFFFFFFF)
    assert pkt.hex() == GOLDEN_DECISION_MAX


def test_decision_magic_and_version():
    pkt = encode(_decision(), sequence=1)
    assert pkt[:4] == b"DLK1"
    assert pkt[4] == 2


def test_hello_golden():
    pkt = encode_hello(Hello(generation_id=0xCAFEBABE, mtu_bytes=3994,
                             fps=60, applier_build_sha=0xDEADBEEF))
    assert pkt.hex() == GOLDEN_HELLO


def test_hello_ack_golden():
    pkt = encode_hello_ack(HelloAck(generation_id_echo=0x12345678))
    assert pkt.hex() == GOLDEN_HELLO_ACK
```

- [ ] **Step 2: Run the test**

Run: `python -m pytest tests/unit/test_dl_wire_contract.py -q`
Expected: PASS (6 passed). If a decision golden mismatches, the lifted `wire.py` diverged from upstream — re-copy it verbatim.

- [ ] **Step 3: Commit**

```bash
git add gs/tests/unit/test_dl_wire_contract.py
git commit -m "dynlink: pin DLK1 wire-contract goldens from the C authority"
```

---

### Task 5: `config_build.py` — fpvd `dynamicLink` block → policy config

Adapts the standalone `service._build_policy_config` / `_build_aggregator` to fpvd's curated-keys-plus-`tuning` model. The lifted builders consume a `raw` dict shaped like the old `gs.yaml`; this module constructs that `raw` from `tuning` and overlays the curated keys, then delegates. Also builds the controller snapshot (drone target).

**Files:**
- Create: `gs/fpvdgs/dynlink/config_build.py`
- Test: `gs/tests/unit/test_dl_config_build.py`

- [ ] **Step 1: Write the failing test**

```python
# gs/tests/unit/test_dl_config_build.py
from fpvdgs.dynlink.config_build import (
    build_aggregator, build_policy_config, make_dl_snapshot, resolve_profile,
)


def _block(**over):
    blk = {
        "enabled": True, "maxMcs": 5, "bandwidth": 20,
        "txpower": {"min": 18, "max": 28}, "radioProfile": "m8812eu2",
        "droneAddr": None, "dronePort": 9999, "tuning": {},
    }
    blk.update(over)
    return blk


def test_curated_keys_map_into_policy_config():
    cfg = build_policy_config(_block())
    assert cfg.gate.max_mcs == 5
    assert cfg.leading.bandwidth == 20
    assert cfg.leading.tx_power_min_dBm == 18
    assert cfg.leading.tx_power_max_dBm == 28


def test_tuning_passthrough_overrides_defaults():
    cfg = build_policy_config(_block(tuning={"gate": {"hysteresis_up_db": 4.0}}))
    assert cfg.gate.hysteresis_up_db == 4.0
    # curated key still wins over any tuning attempt at the same field
    cfg2 = build_policy_config(_block(maxMcs=3, tuning={"gate": {"max_mcs": 7}}))
    assert cfg2.gate.max_mcs == 3


def test_resolve_profile_uses_packaged_json():
    prof = resolve_profile(_block(radioProfile="m8812eu2"))
    assert prof.name == "BL-M8812EU2"


def test_build_aggregator_reads_tuning_smoothing():
    agg = build_aggregator(_block(tuning={"smoothing": {"ewma_alpha_rssi": 0.5}}))
    assert agg.ewma_alpha_rssi == 0.5


def test_make_dl_snapshot_defaults_drone_host_from_endpoint():
    eff = {"dynamicLink": _block(droneAddr=None),
           "drone": {"endpoint": "http://10.5.0.10:8080"}}
    snap = make_dl_snapshot(eff)
    assert snap["droneAddr"] == "10.5.0.10"
    assert snap["dronePort"] == 9999


def test_make_dl_snapshot_explicit_drone_addr_wins():
    eff = {"dynamicLink": _block(droneAddr="10.5.0.99", dronePort=12345),
           "drone": {"endpoint": "http://10.5.0.10:8080"}}
    snap = make_dl_snapshot(eff)
    assert snap["droneAddr"] == "10.5.0.99"
    assert snap["dronePort"] == 12345
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `python -m pytest tests/unit/test_dl_config_build.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.dynlink.config_build'`

- [ ] **Step 3: Create `config_build.py` header, then append the lifted builders**

First write the header (imports + `PROFILES_DIR` + wrappers). Create the file with this content:

```python
# gs/fpvdgs/dynlink/config_build.py
"""Translate fpvd's `dynamicLink` config block into the policy/aggregator
objects the lifted control core expects, and build the controller snapshot.

The lifted `_build_policy_config(raw)` / `_build_aggregator(raw)` consume a
dict shaped like the old gs.yaml. We construct that `raw` from the opaque
`tuning` passthrough, then overlay the curated top-level keys so they always
win."""
from __future__ import annotations

import copy
import logging
from pathlib import Path
from urllib.parse import urlparse

from .policy import (
    CooldownConfig, FECBounds, GateConfig, LeadingLoopConfig,
    PolicyConfig, ProfileSelectionConfig, SafeDefaults,
)
from .bitrate import BitrateConfig
from .dynamic_fec import DynamicFecConfig
from .predictor import PredictorConfig
from .profile import RadioProfile, load_profile
from .signals import SignalAggregator

log = logging.getLogger("fpvdgs.dynlink")

PROFILES_DIR = Path(__file__).resolve().parent / "profiles"


def _raw_from_block(block: dict) -> dict:
    """Build a gs.yaml-shaped `raw` dict: tuning is the base, curated keys
    are overlaid so they always win over any tuning attempt."""
    raw = copy.deepcopy(block.get("tuning") or {})
    leading = raw.setdefault("leading_loop", {})
    gate = raw.setdefault("gate", {})
    if "bandwidth" in block:
        leading["bandwidth"] = int(block["bandwidth"])
    tx = block.get("txpower") or {}
    if "min" in tx:
        leading["tx_power_min_dBm"] = float(tx["min"])
    if "max" in tx:
        leading["tx_power_max_dBm"] = float(tx["max"])
    if "maxMcs" in block:
        gate["max_mcs"] = int(block["maxMcs"])
    return raw


def build_policy_config(block: dict) -> PolicyConfig:
    return _build_policy_config(_raw_from_block(block))


def build_aggregator(block: dict) -> SignalAggregator:
    return _build_aggregator(_raw_from_block(block))


def resolve_profile(block: dict) -> RadioProfile:
    name = block.get("radioProfile", "m8812eu2")
    return load_profile(name, [PROFILES_DIR])


def make_dl_snapshot(effective: dict) -> dict:
    """Self-contained snapshot the controller consumes. Resolves the drone
    UDP target: explicit dynamicLink.droneAddr wins, else the host from
    drone.endpoint; port defaults to 9999 (the fpvd drone's listener)."""
    block = dict(effective.get("dynamicLink", {}))
    endpoint = effective.get("drone", {}).get("endpoint", "http://10.5.0.10:8080")
    host = urlparse(endpoint).hostname or "10.5.0.10"
    block["droneAddr"] = block.get("droneAddr") or host
    block["dronePort"] = int(block.get("dronePort") or 9999)
    return block
```

Then append the lifted builders to the end of the file. Lines 61–294 of the source hold exactly `_DEPRECATED_LEADING_KEYS`, `_build_policy_config`, and `_build_aggregator`:

```bash
cd /home/gilankpam/Projects/drone/fpvd/gs
SRC=/home/gilankpam/Projects/drone/dynamic-link/gs/dynamic_link/service.py
{ echo; echo; sed -n '61,294p' "$SRC"; } >> fpvdgs/dynlink/config_build.py
```

These functions reference only names already imported in the header (`LeadingLoopConfig`, `GateConfig`, `ProfileSelectionConfig`, `CooldownConfig`, `FECBounds`, `DynamicFecConfig`, `SafeDefaults`, `PredictorConfig`, `BitrateConfig`, `PolicyConfig`, `SignalAggregator`, `log`). Verify the file imports cleanly:

```bash
python -c "import fpvdgs.dynlink.config_build" && echo OK
```
Expected: `OK`. Definition order is fine — `build_policy_config` resolves `_build_policy_config` at call time, not at import.

- [ ] **Step 4: Run the test to verify it passes**

Run: `python -m pytest tests/unit/test_dl_config_build.py -q`
Expected: PASS (6 passed)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/config_build.py gs/tests/unit/test_dl_config_build.py
git commit -m "dynlink: build policy config + snapshot from fpvd dynamicLink block"
```

---

### Task 6: `DynamicLinkController` — in-process asyncio loop on a daemon thread

The thread-safe wrapper. One daemon thread owns an asyncio loop that runs the stats client → policy → return-link pipeline plus the HELLO listener. `set_config` while running rebuilds the loop (stop+start) from the new snapshot — the wfb runner is never touched. A `stats_client_factory` seam lets tests inject canned events without speaking the wfb JSON wire.

**Files:**
- Create: `gs/fpvdgs/dynlink/controller.py`
- Test: `gs/tests/unit/test_dl_controller.py`

- [ ] **Step 1: Write the failing tests**

```python
# gs/tests/unit/test_dl_controller.py
import socket
import time

from fpvdgs.dynlink.controller import DynamicLinkController
from fpvdgs.dynlink.stats_client import RxAnt, RxEvent, SessionInfo


def _snapshot(drone_port, **over):
    snap = {
        "enabled": True, "maxMcs": 5, "bandwidth": 20,
        "txpower": {"min": 18, "max": 28}, "radioProfile": "m8812eu2",
        "droneAddr": "127.0.0.1", "dronePort": drone_port, "tuning": {},
    }
    snap.update(over)
    return snap


def _rx_event():
    return RxEvent(
        timestamp=1.0, id="video",
        packets_window={"out": 100, "lost": 0, "data": 100},
        rx_ant_stats=[RxAnt(ant=0, freq=5825, mcs=2, bw=20, pkt_recv=100,
                            rssi_min=-60, rssi_avg=-55, rssi_max=-50,
                            snr_min=20, snr_avg=25, snr_max=30)],
        session=SessionInfo(fec_type="rs", fec_k=8, fec_n=12, epoch=1,
                            interleave_depth=1, contract_version=1),
    )


class _IdleStatsClient:
    """Connects (sets statsConnected) but emits nothing until stopped."""
    def __init__(self, endpoint, on_event):
        self._stop = False

    async def run(self):
        while not self._stop:
            import asyncio
            await asyncio.sleep(0.02)

    def stop(self):
        self._stop = True


class _OneShotStatsClient:
    """Emits a single RxEvent on connect, then idles."""
    def __init__(self, endpoint, on_event):
        self._on_event = on_event
        self._stop = False

    async def run(self):
        import asyncio
        self._on_event(_rx_event())
        while not self._stop:
            await asyncio.sleep(0.02)

    def stop(self):
        self._stop = True


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    return s, port


def test_start_sets_running_then_stop_joins():
    c = DynamicLinkController(_snapshot(40000),
                              stats_client_factory=_IdleStatsClient,
                              gs_listen_port=0)
    c.start()
    try:
        assert c.status()["running"] is True
    finally:
        c.stop()
    assert c.status()["running"] is False


def test_emits_decision_packet_to_drone():
    sock, port = _free_udp_port()
    sock.settimeout(2.0)
    c = DynamicLinkController(_snapshot(port),
                              stats_client_factory=_OneShotStatsClient,
                              gs_listen_port=0)
    c.start()
    try:
        data, _ = sock.recvfrom(64)
    finally:
        c.stop()
        sock.close()
    assert data[:4] == b"DLK1"
    assert len(data) == 31
    st = c.status()
    assert st["decision"]["mcs"] is not None
    assert st["emitSeq"] >= 1


def test_set_config_while_running_rebuilds_with_new_drone_port():
    sock_a, port_a = _free_udp_port()
    sock_b, port_b = _free_udp_port()
    sock_b.settimeout(2.0)
    c = DynamicLinkController(_snapshot(port_a),
                              stats_client_factory=_OneShotStatsClient,
                              gs_listen_port=0)
    c.start()
    try:
        c.set_config(_snapshot(port_b))
        data, _ = sock_b.recvfrom(64)   # now arrives on the new port
        assert data[:4] == b"DLK1"
        assert c.status()["running"] is True
    finally:
        c.stop()
        sock_a.close()
        sock_b.close()
```

- [ ] **Step 2: Run the tests to verify they fail**

Run: `python -m pytest tests/unit/test_dl_controller.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'fpvdgs.dynlink.controller'`

- [ ] **Step 3: Implement the controller**

```python
# gs/fpvdgs/dynlink/controller.py
"""In-process GS dynamic-link controller.

Owns one daemon thread running an asyncio event loop: stats client →
SignalAggregator → Policy → wire encode → ReturnLink, plus the P4a HELLO
listener. Thread-safe surface for the (thread-based) supervisor:
start/stop/set_config/status. A config change while running rebuilds the
loop from the new snapshot; the wfb runner is never touched."""
from __future__ import annotations

import asyncio
import logging
import threading
import time

from .config_build import build_aggregator, build_policy_config, resolve_profile
from .drone_config import DroneConfigState
from .policy import Policy
from .return_link import ReturnLink
from .stats_client import RxEvent, SessionEvent, StatsClient
from .tunnel_listener import TunnelListener
from .wire import Encoder as WireEncoder, encode_hello_ack

log = logging.getLogger("fpvdgs.dynlink")


class DynamicLinkController:
    def __init__(self, snapshot, *, stats_endpoint="tcp://127.0.0.1:8103",
                 gs_listen_addr="0.0.0.0", gs_listen_port=5801,
                 stats_client_factory=StatsClient):
        self._snapshot = dict(snapshot)
        self._stats_endpoint = stats_endpoint
        self._gs_listen = (gs_listen_addr, gs_listen_port)
        self._make_stats = stats_client_factory
        self._lock = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None         # asyncio.Event, created in-loop
        self._started = threading.Event()
        self._status = {"running": False, "statsConnected": False,
                        "decision": None, "lastEmitMs": None, "emitSeq": 0,
                        "reason": "", "hello": "none"}

    # ---- thread-safe public API -----------------------------------------
    def start(self):
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._started.clear()
            self._thread = threading.Thread(target=self._thread_main,
                                            name="dl-controller", daemon=True)
            self._thread.start()
        self._started.wait(timeout=5.0)

    def stop(self):
        with self._lock:
            loop, stop, thread = self._loop, self._stop_event, self._thread
        if loop is not None and stop is not None:
            loop.call_soon_threadsafe(stop.set)
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)
        with self._lock:
            self._thread = None

    def set_config(self, snapshot):
        """Apply a new snapshot. If running, rebuild the loop (stop+start)
        from the new config — the wfb runner is untouched."""
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
        drone_cfg = DroneConfigState()
        policy = Policy(build_policy_config(snap), profile, drone_config=drone_cfg)
        aggregator = build_aggregator(snap)
        return_link = ReturnLink(snap["droneAddr"], int(snap["dronePort"]))
        encoder = WireEncoder(seq=1)

        def _on_hello(h):
            drone_cfg.on_hello(h)
            ack = drone_cfg.build_ack()
            if ack is not None:
                return_link.send_hello_ack(encode_hello_ack(ack))
                self._set(hello="acked")

        listener = TunnelListener(self._gs_listen[0], self._gs_listen[1],
                                  on_pong=None, on_hello=_on_hello)
        try:
            await listener.start()
        except OSError as e:
            log.warning("dl: HELLO listener bind %s failed: %s", self._gs_listen, e)
            listener = None

        def on_event(ev):
            if isinstance(ev, SessionEvent):
                aggregator.update_session(ev.session)
                return
            if isinstance(ev, RxEvent):
                signals = aggregator.consume(ev)
                decision = policy.tick(signals)
                return_link.send(encoder.encode(decision))
                self._set(
                    decision={"mcs": decision.mcs, "k": decision.k,
                              "n": decision.n, "depth": decision.depth,
                              "txpowerDbm": decision.tx_power_dBm,
                              "bitrateKbps": decision.bitrate_kbps},
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
            if listener is not None:
                listener.stop()
            return_link.close()
            self._set(running=False, statsConnected=False)

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
```

The single `drone_cfg = DroneConfigState()` instance is shared between `Policy` (which reads handshake state to gate emit) and the `_on_hello` callback (which feeds it and builds the ACK) — so the policy sees the same sync state the listener updates.

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python -m pytest tests/unit/test_dl_controller.py -q`
Expected: PASS (3 passed). The emit tests rely on the lifted policy producing a Decision on the first tick (it does — observer mode logs decisions from tick 1 upstream).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/controller.py gs/tests/unit/test_dl_controller.py
git commit -m "dynlink: add in-process DynamicLinkController (thread + asyncio loop)"
```

---

### Task 7: Config schema + defaults for the `dynamicLink` block

**Files:**
- Modify: `gs/fpvdgs/schema.py`
- Modify: `gs/etc/defaults.json`
- Test: `gs/tests/unit/test_schema.py` (append)

- [ ] **Step 1: Write the failing tests (append to `tests/unit/test_schema.py`)**

```python
# --- dynamicLink ---
import pytest
from fpvdgs import schema
from fpvdgs.schema import SchemaError


def _eff(**dl):
    base = {"link": {"channel": 132, "width": 40, "region": "US"},
            "dynamicLink": {"enabled": False, "maxMcs": 5, "bandwidth": 20,
                            "txpower": {"min": 18, "max": 28},
                            "radioProfile": "m8812eu2", "dronePort": 9999,
                            "tuning": {}}}
    base["dynamicLink"].update(dl)
    return base


def test_config_patch_allows_dynamiclink():
    schema.validate_config_patch({"dynamicLink": {"enabled": True}})  # no raise


def test_effective_accepts_valid_dynamiclink():
    schema.validate_effective(_eff())  # no raise


def test_effective_rejects_bad_max_mcs():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(maxMcs=9))


def test_effective_rejects_bad_bandwidth():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(bandwidth=15))


def test_effective_rejects_inverted_txpower():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(txpower={"min": 30, "max": 10}))
```

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest tests/unit/test_schema.py -q`
Expected: FAIL — `dynamicLink` is an unknown config key in `validate_config_patch`.

- [ ] **Step 3: Update `schema.py`**

Add `dynamicLink` to the allowed top-level config keys and a validator:

Change:
```python
CONFIG_TOP_KEYS = {"wfb", "drone"}      # link is excluded on purpose
```
to:
```python
CONFIG_TOP_KEYS = {"wfb", "drone", "dynamicLink"}   # link is excluded on purpose
DL_BANDWIDTHS = {20, 40}
```

Append to `validate_effective`, after the existing link checks:
```python
    dl = cfg.get("dynamicLink")
    if dl is not None:
        _validate_dynamic_link(dl)


def _validate_dynamic_link(dl: dict) -> None:
    max_mcs = dl.get("maxMcs", 5)
    if not isinstance(max_mcs, int) or not 0 <= max_mcs <= 7:
        raise SchemaError("dynamicLink.maxMcs must be an int in 0..7")
    bw = dl.get("bandwidth", 20)
    if bw not in DL_BANDWIDTHS:
        raise SchemaError(f"dynamicLink.bandwidth must be one of {sorted(DL_BANDWIDTHS)}")
    tx = dl.get("txpower", {}) or {}
    lo, hi = tx.get("min", 0), tx.get("max", 30)
    if lo > hi:
        raise SchemaError("dynamicLink.txpower.min must be <= max")
    port = dl.get("dronePort", 9999)
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("dynamicLink.dronePort must be an int in 1..65535")
```

- [ ] **Step 4: Add the default block to `etc/defaults.json`**

Add a `"dynamicLink"` key alongside `"link"`, `"wfb"`, `"drone"`:
```json
  "dynamicLink": {
    "enabled": false,
    "maxMcs": 5,
    "bandwidth": 20,
    "txpower": { "min": 18, "max": 28 },
    "radioProfile": "m8812eu2",
    "droneAddr": null,
    "dronePort": 9999,
    "tuning": {}
  }
```

- [ ] **Step 5: Run the tests to verify they pass**

Run: `python -m pytest tests/unit/test_schema.py -q`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/schema.py gs/etc/defaults.json gs/tests/unit/test_schema.py
git commit -m "config: add validated dynamicLink block to GS schema + defaults"
```

---

### Task 8: Render `log_interval = 100` unconditionally

The policy needs 10 Hz stats. Emitting `log_interval = 100` always means enabling/disabling dynamic-link never requires a cfg change or runner bounce.

**Files:**
- Modify: `gs/fpvdgs/render.py`
- Test: `gs/tests/unit/test_render.py` (append)

- [ ] **Step 1: Write the failing test (append to `tests/unit/test_render.py`)**

```python
def test_render_emits_10hz_log_interval():
    from fpvdgs import render
    cfg = {"link": {"channel": 132, "width": 40, "region": "US"}, "wfb": {}}
    text = render.render_cfg(cfg)
    assert "log_interval = 100" in text
    # it belongs to [common]
    common = text.split("[common]", 1)[1].split("[", 1)[0]
    assert "log_interval = 100" in common
```

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest tests/unit/test_render.py -q`
Expected: FAIL — no `log_interval` in output.

- [ ] **Step 3: Emit it in `render_cfg`**

In `gs/fpvdgs/render.py`, in `render_cfg`, after the `wifi_region`/`wifi_txpower` lines in the `[common]` section (before the `[gs_video]` block), add:
```python
    # Dynamic-link needs 10 Hz stats (log_interval ms). Emitted unconditionally
    # so enabling/disabling dynamic-link never requires a runner bounce.
    lines.append("log_interval = 100")
```
Place this append immediately after the `wifi_txpower` block and before the `width = link["width"]` line.

- [ ] **Step 4: Run the test to verify it passes**

Run: `python -m pytest tests/unit/test_render.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/render.py gs/tests/unit/test_render.py
git commit -m "render: emit log_interval=100 unconditionally for 10Hz dynamic-link stats"
```

---

### Task 9: Route `dynamicLink` deltas through `POST /apply` without bouncing the runner

`_apply_gs` is refined to diff subsystems: render+bounce the runner only when a non-`dynamicLink` field changed, and always route the dynamic-link delta to the controller (`start`/`stop`/`set_config`). The controller is injected as `dynlink` (defaulting to `None`, so existing tests/callers are unaffected).

**Files:**
- Modify: `gs/fpvdgs/api.py`
- Test: `gs/tests/unit/test_api.py` (append)

- [ ] **Step 1: Write the failing tests (append to `tests/unit/test_api.py`)**

```python
# --- dynamicLink apply routing ---
class _FakeController:
    def __init__(self):
        self.calls = []
    def start(self):
        self.calls.append(("start", None))
    def stop(self):
        self.calls.append(("stop", None))
    def set_config(self, snap):
        self.calls.append(("set_config", snap))


class _FakeRunner:
    def __init__(self):
        self.restarts = 0
    def restart(self):
        self.restarts += 1
        return True


def _api_with_dynlink(tmp_path):
    import json
    from fpvdgs import render, schema
    from fpvdgs.api import Api
    from fpvdgs.config import ConfigStore
    from fpvdgs.drone_client import DroneClient

    defaults = {"link": {"channel": 132, "width": 40, "region": "US"},
                "wfb": {"profile": "gs", "raw": {}},
                "drone": {"endpoint": "http://10.5.0.10:8080"},
                "dynamicLink": {"enabled": False, "maxMcs": 5, "bandwidth": 20,
                                "txpower": {"min": 18, "max": 28},
                                "radioProfile": "m8812eu2", "droneAddr": None,
                                "dronePort": 9999, "tuning": {}}}
    store = ConfigStore(defaults)
    ctrl = _FakeController()
    runner = _FakeRunner()
    cfg_out = str(tmp_path / "wfb.cfg")
    api = Api(store=store, schema=schema, render_mod=render, runner=runner,
              drone=DroneClient("http://127.0.0.1:1"), link=None,
              status_fn=lambda: {}, cfg_out=cfg_out, dynlink=ctrl)
    return api, store, ctrl, runner


def test_enable_dynamiclink_starts_controller_without_bouncing_runner(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    code, body = api.handle("POST", "/apply", {}, b"")
    assert code == 200 and body["applied"] is True
    assert ("start", None) in ctrl.calls
    assert runner.restarts == 0          # dynamic-link-only change: no bounce
    assert store.effective()["dynamicLink"]["enabled"] is True


def test_disable_dynamiclink_stops_controller(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    store.patch({"dynamicLink": {"enabled": False}})
    api.handle("POST", "/apply", {}, b"")
    assert ("stop", None) in ctrl.calls
    assert runner.restarts == 0


def test_tuning_change_while_enabled_calls_set_config(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    api.handle("POST", "/apply", {}, b"")
    store.patch({"dynamicLink": {"maxMcs": 3}})
    api.handle("POST", "/apply", {}, b"")
    assert any(c[0] == "set_config" for c in ctrl.calls)
    assert runner.restarts == 0


def test_wfb_change_bounces_runner_and_leaves_controller_alone(tmp_path):
    api, store, ctrl, runner = _api_with_dynlink(tmp_path)
    store.patch({"wfb": {"raw": {"common": {"foo": 1}}}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert runner.restarts == 1          # non-dynamicLink change: bounce
    assert ctrl.calls == []              # controller untouched (stayed disabled)
```

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest tests/unit/test_api.py -q`
Expected: FAIL — `Api.__init__` has no `dynlink` parameter.

- [ ] **Step 3: Update `api.py`**

Add the `dynlink` dependency and the subsystem-diffing apply path.

In `Api.__init__`, add the parameter (default `None`) and store it:
```python
    def __init__(self, store, schema, render_mod, runner, drone, link,
                 status_fn, cfg_out, dynlink=None):
        ...
        self.dynlink = dynlink
```

Add the import at the top of the file:
```python
from .dynlink.config_build import make_dl_snapshot
```

Replace `_apply_gs` with:
```python
    @staticmethod
    def _without(cfg: dict, *keys) -> dict:
        return {k: v for k, v in cfg.items() if k not in keys}

    def _apply_gs(self):
        pending = self.store.pending()
        effective = self.store.effective()
        # Guard: link drift must go through /link/apply (drone coordination).
        if pending.get("link") != effective.get("link"):
            return 409, {"error": "link changed; use POST /link/apply"}
        self.schema.validate_effective(pending)

        # Anything outside dynamicLink (link already equal) needs the runner.
        non_dl_changed = (self._without(pending, "dynamicLink")
                          != self._without(effective, "dynamicLink"))
        if non_dl_changed:
            self.render_mod.write_cfg(self.cfg_out,
                                      self.render_mod.render_cfg(pending))
            if not self.runner.restart():
                self.render_mod.restore_bak(self.cfg_out)
                self.runner.restart()
                return 500, {"applied": False,
                             "error": "runner failed; rolled back to last-good cfg"}

        self._route_dynamic_link(effective.get("dynamicLink", {}),
                                 pending.get("dynamicLink", {}), pending)
        self.store.commit()
        return 200, {"applied": True}

    def _route_dynamic_link(self, dl_old, dl_new, pending):
        """Start/stop/reconfigure the in-process controller. Never bounces
        the wfb runner."""
        if self.dynlink is None:
            return
        was, now = bool(dl_old.get("enabled")), bool(dl_new.get("enabled"))
        if not was and now:
            self.dynlink.set_config(make_dl_snapshot(pending))
            self.dynlink.start()
        elif was and not now:
            self.dynlink.stop()
        elif was and now and dl_old != dl_new:
            self.dynlink.set_config(make_dl_snapshot(pending))
```

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python -m pytest tests/unit/test_api.py -q`
Expected: PASS (existing api tests still green; 4 new pass)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/api.py gs/tests/unit/test_api.py
git commit -m "api: route dynamicLink applies to the controller without bouncing wfb"
```

---

### Task 10: Surface dynamic-link state in `GET /status`

**Files:**
- Modify: `gs/fpvdgs/status.py`
- Test: `gs/tests/unit/test_status.py` (append)

- [ ] **Step 1: Write the failing test (append to `tests/unit/test_status.py`)**

```python
def test_build_status_includes_dynamic_link_section():
    from fpvdgs import status
    dl = {"enabled": True, "running": True, "statsConnected": True,
          "decision": {"mcs": 4, "k": 8, "n": 12, "depth": 1,
                       "txpowerDbm": 22, "bitrateKbps": 9000},
          "lastEmitMs": 1234, "emitSeq": 5, "reason": "snr_margin",
          "drone": {"reachable": True, "dynamicLinkActive": True, "hello": "acked"}}
    out = status.build_status("0.1.0", {"running": True}, {}, {"reachable": True},
                              dynamic_link=dl)
    assert out["dynamicLink"]["running"] is True
    assert out["dynamicLink"]["decision"]["mcs"] == 4
    assert out["dynamicLink"]["drone"]["hello"] == "acked"


def test_build_status_omits_dynamic_link_when_absent():
    from fpvdgs import status
    out = status.build_status("0.1.0", {"running": True}, {}, {"reachable": True})
    assert "dynamicLink" not in out
```

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest tests/unit/test_status.py -q`
Expected: FAIL — `build_status` has no `dynamic_link` parameter.

- [ ] **Step 3: Update `status.py`**

Add the optional parameter and include it when present:
```python
def build_status(version: str, runner_state: dict, wlans: dict,
                 drone_probe: dict, link_stats: dict | None = None,
                 uptime_ms: int | None = None,
                 dynamic_link: dict | None = None) -> dict:
```
At the end, before `return`:
```python
    out = {
        "fpvd": fpvd,
        "runner": runner_state,
        "radio": radio,
        "link": link,
    }
    if dynamic_link is not None:
        out["dynamicLink"] = dynamic_link
    return out
```
(Replace the existing `return {...}` with the `out` assembly above.)

- [ ] **Step 4: Run the tests to verify they pass**

Run: `python -m pytest tests/unit/test_status.py -q`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/status.py gs/tests/unit/test_status.py
git commit -m "status: add optional dynamicLink section"
```

---

### Task 11: Wire the controller into the supervisor lifecycle and status

**Files:**
- Modify: `gs/fpvdgs/supervisor.py`
- Test: `gs/tests/integration/test_supervisor_e2e.py` (append)

- [ ] **Step 1: Write the failing integration test (append to `tests/integration/test_supervisor_e2e.py`)**

```python
def test_dynamiclink_assembled_into_status_and_controller_built(tmp_path, monkeypatch):
    """build_app constructs a controller; status_fn merges its state.
    Uses a stub controller via monkeypatch so no sockets/threads are needed."""
    import json
    from fpvdgs import supervisor

    class _StubController:
        def __init__(self, *a, **k):
            self.started = False
        def start(self): self.started = True
        def stop(self): self.started = False
        def set_config(self, snap): pass
        def status(self):
            return {"running": self.started, "statsConnected": False,
                    "decision": None, "lastEmitMs": None, "emitSeq": 0,
                    "reason": "", "hello": "none"}

    monkeypatch.setattr(supervisor, "DynamicLinkController", _StubController)
    # Avoid spawning the real runner / radio probing.
    monkeypatch.setattr(supervisor, "resolve_wlans", lambda cfg: ["wlan0"])

    defaults = tmp_path / "defaults.json"
    defaults.write_text(json.dumps({
        "link": {"channel": 132, "width": 40, "region": "US"},
        "wfb": {"profile": "gs", "raw": {}},
        "drone": {"endpoint": "http://127.0.0.1:1"},
        "dynamicLink": {"enabled": False, "maxMcs": 5, "bandwidth": 20,
                        "txpower": {"min": 18, "max": 28},
                        "radioProfile": "m8812eu2", "droneAddr": None,
                        "dronePort": 9999, "tuning": {}}}))
    cfg_out = tmp_path / "wfb.cfg"

    app = supervisor.build_app(str(defaults), str(tmp_path / "config.json"),
                               str(cfg_out), "127.0.0.1", 0,
                               runner_cmd=["true"])
    code, body = app.api.handle("GET", "/status", {}, b"")
    assert code == 200
    assert "dynamicLink" in body
    assert body["dynamicLink"]["running"] is False
```

Note: this test reads `app.api` — expose the `Api` on `App` in Step 3.

- [ ] **Step 2: Run to verify failure**

Run: `python -m pytest tests/integration/test_supervisor_e2e.py -q`
Expected: FAIL — `supervisor` has no `DynamicLinkController`; `App` has no `api`.

- [ ] **Step 3: Update `supervisor.py`**

Add the import:
```python
from .dynlink.controller import DynamicLinkController
from .dynlink.config_build import make_dl_snapshot
```

Extend `App` to hold the controller and Api, and start/stop it:
```python
class App:
    def __init__(self, store, runner, http_server, api, dynlink):
        self.store = store
        self.runner = runner
        self.http = http_server
        self.api = api
        self.dynlink = dynlink

    def start(self):
        self.runner.start()
        if self.store.effective().get("dynamicLink", {}).get("enabled"):
            self.dynlink.start()

    def serve_forever(self):
        self.http.serve_forever()

    def shutdown(self):
        self.http.shutdown()
        self.dynlink.stop()
        self.runner.shutdown()
```

In `build_app`, after constructing `drone` and before `status_fn`, build the controller:
```python
    dynlink = DynamicLinkController(make_dl_snapshot(effective))
```

Extend `status_fn` to merge controller + drone state:
```python
    def _dynamic_link_status():
        eff_dl = store.effective().get("dynamicLink", {})
        st = dynlink.status()
        st["enabled"] = bool(eff_dl.get("enabled"))
        drone_active = None
        try:
            drone_active = drone.get_status().get("link", {}).get("dynamicLinkActive")
        except Exception:
            drone_active = None
        st["drone"] = {"reachable": drone.healthz(),
                       "dynamicLinkActive": drone_active,
                       "hello": st.pop("hello", "none")}
        return st

    def status_fn():
        wlan_info = {w: status_mod.iw_info(w) for w in resolve_wlans(store.effective())}
        reachable = drone.healthz()
        eff_link = store.effective().get("link", {})
        probe = {"reachable": reachable, "linkId": eff_link.get("linkId"),
                 "inSync": link.in_sync() if link else None}
        uptime_ms = int((time.monotonic() - started) * 1000)
        return status_mod.build_status(__version__, runner.state(), wlan_info, probe,
                                       uptime_ms=uptime_ms,
                                       dynamic_link=_dynamic_link_status())
```

Pass `dynlink` into the `Api` and return it on the `App`:
```python
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone, link=link, status_fn=status_fn, cfg_out=cfg_out,
              dynlink=dynlink)
    http_server = make_http_server(api, host, port)
    return App(store, runner, http_server, api, dynlink)
```

- [ ] **Step 4: Run the integration test**

Run: `python -m pytest tests/integration/test_supervisor_e2e.py -q`
Expected: PASS

- [ ] **Step 5: Run the full GS suite**

Run: `python -m pytest -q`
Expected: PASS (all green)

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/supervisor.py gs/tests/integration/test_supervisor_e2e.py
git commit -m "supervisor: construct dynamic-link controller, wire lifecycle + status"
```

---

### Task 12: Docs and final verification

**Files:**
- Modify: `gs/README.md` or `README.md` (whichever documents GS config/endpoints)
- Modify: `README.md` config reference (add `dynamicLink` block)

- [ ] **Step 1: Document the `dynamicLink` config block and behavior**

In the GS config reference, add a `dynamicLink` subsection covering: `enabled`, `maxMcs`, `bandwidth`, `txpower.min/max`, `radioProfile`, `droneAddr`/`dronePort` (default drone host : 9999), and `tuning` (opaque passthrough). State the operating model: the controller is a stats client of the runner's `:8103`; enabling/disabling/tuning is applied at runtime with no wfb restart; the drone's `dynamicLink.enabled` must be armed separately (visible in `/status.dynamicLink.drone`).

- [ ] **Step 2: Full test + lint sweep**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && python -m pytest -q`
Expected: PASS (entire suite)

Run: `grep -rn "TODO\|dynamic_link\." gs/fpvdgs/dynlink/ || echo clean`
Expected: `clean` (no leftover absolute imports or TODOs)

- [ ] **Step 3: Commit**

```bash
git add README.md gs/README.md 2>/dev/null; git commit -m "docs: document GS dynamicLink config block and runtime-apply behavior"
```

---

## Notes for the implementer

- **No `pytest-asyncio`.** The controller is tested through its thread-safe public API; the asyncio loop lives entirely inside the controller's own thread. Do not add async test fixtures.
- **The wire is sacred.** If any golden in Task 4 fails, the lifted `wire.py` was altered — re-copy it verbatim from the source. The drone decoder is the authority.
- **`set_config` semantics (v1).** A config change while running rebuilds the control loop (stop+start) from the new snapshot. This briefly pauses decision emit — well within the drone's multi-second watchdog. In-place live tunable swap is a future refinement; it is NOT required to satisfy "no wfb restart" (the runner is never touched here).
- **Stats reconnect.** `_stats_loop` reconnects on disconnect, so a real link change that bounces the runner does not kill the controller. Confirm `StatsClient.run()` returns/raises on disconnect (it should); the loop handles both.
- **Open spec items** (verify in passing): GS HELLO-listen bind `10.5.0.1:5801` against the live drone build; wfb-ng stats JSON schema parity between fpvd and dynamic-link.
