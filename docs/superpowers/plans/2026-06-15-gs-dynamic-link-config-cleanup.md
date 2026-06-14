# GS Dynamic-Link Config Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Mirror the drone's config model on the GS daemon — code-as-default-source behind a single full `config.json` with a tolerant loader — and de-opacify the `dynamicLink` controller config: flatten the opaque `tuning` passthrough into explicit validated blocks, merge/delete dead knobs, and freeze calibration/internal knobs as code constants.

**Architecture:** `fpvdgs` is a stdlib Python daemon. Config flows: `config.json` (full) deep-merged onto code defaults (`config_defaults.default_config()`) in `ConfigStore` → `dynlink/config_build.py` maps the `dynamicLink` block onto policy/aggregator dataclasses → the in-process `DynamicLinkController`. This plan keeps the GS test suite green **as a whole** at every commit (config_build/import coupling makes partial refactors go red), so tasks are ordered as coherent green milestones.

**Tech Stack:** Python ≥3.11, stdlib only (no deps), pytest. Run from `gs/`.

**Spec:** `docs/superpowers/specs/2026-06-15-gs-dynamic-link-config-cleanup-design.md`

**Test command (full suite — run after every task):**
```bash
cd gs && .venv/bin/python -m pytest tests/ -q
```

---

## File Structure

**Created:**
- `gs/fpvdgs/config_defaults.py` — `default_config()`: the single source of GS config defaults (the former `etc/defaults.json` content, in code). The `dynamicLink` subtree is assembled from the dynlink dataclasses so its defaults stay DRY with the code that consumes them.

**Modified:**
- `gs/fpvdgs/config.py` — `ConfigStore`: load `config.json` deep-merged onto `default_config()`; persist the full effective config; tolerant warn-on-unknown.
- `gs/fpvdgs/supervisor.py` — `build_app` drops `defaults_path`; `main` drops `--defaults`, adds `--dump-config`.
- `gs/fpvdgs/dynlink/policy.py` — merge `GateConfig`+`ProfileSelectionConfig` → `SelectorConfig` (drop dead fields); `learnedPrior` always-on.
- `gs/fpvdgs/dynlink/config_build.py` — delete the `tuning` reshape + deprecation machinery; read explicit blocks; freeze learned-prior/flightlog-internals/rssi-norm-curve.
- `gs/fpvdgs/dynlink/controller.py` — `video_id` frozen to `"video"`.
- `gs/fpvdgs/schema.py` — validate the flat `dynamicLink` blocks.
- `gs/fpvdgs/probe/config_build.py` — freeze probe knobs; drop the orphaned `effective['probe']` read.
- `gs/scripts/S99fpvd`, `deploy/gs/deploy.sh`, `docs/api.md` — drop `defaults.json`; seed via `--dump-config`; doc the flat schema.

**Deleted:** `gs/etc/defaults.json`, `deploy/gs/config.json`, the stale `gs/build/lib/fpvdgs/dynlink/profiles/*.json` artifact.

---

## Task 1: Model migration — code defaults + single full config.json + `--dump-config`

Move `etc/defaults.json` into code (`config_defaults.default_config()`, **old `dynamicLink` shape preserved** — `tuning: {}`), rewrite `ConfigStore` to deep-merge `config.json` onto code defaults and persist the full effective config, add `--dump-config`, drop `--defaults`. The `dynamicLink` knobs stay opaque here (still routed through `tuning`) — this task is the config-plumbing migration only, so the suite stays green.

**Files:**
- Create: `gs/fpvdgs/config_defaults.py`
- Modify: `gs/fpvdgs/config.py`, `gs/fpvdgs/supervisor.py`, `gs/scripts/S99fpvd`
- Delete: `gs/etc/defaults.json`
- Test: `gs/tests/unit/test_config.py`, `gs/tests/unit/test_app_wiring.py`, `gs/tests/integration/test_supervisor_e2e.py`

- [ ] **Step 1: Create the code-defaults module**

Create `gs/fpvdgs/config_defaults.py` (verbatim port of `etc/defaults.json`; `dynamicLink` keeps the current shape including `tuning: {}` — it is restructured in Task 2):

```python
"""Single source of GS config defaults (the former etc/defaults.json, in code).

Mirrors the drone: code holds every default; config.json is the full effective
config, merged onto these defaults. `--dump-config` materializes this tree."""
from __future__ import annotations


def default_config() -> dict:
    return {
        "link": {
            "channel": 132, "width": 20, "txPowerDbm": None, "region": "US",
            "linkId": 7669206, "beamforming": {"enabled": False}, "wlans": "auto",
        },
        "wfb": {
            "profile": "gs",
            "mavlink": {"peer": "connect://127.0.0.1:14550"},
            "raw": {},
        },
        "drone": {"endpoint": "http://10.5.0.10:8080"},
        "dynamicLink": {
            "enabled": False, "maxMcs": 5, "radioProfile": "m8812eu2",
            "droneAddr": None, "dronePort": 9999, "videoStreamId": "video",
            "tuning": {},
        },
        "idrForward": {"enabled": True, "port": 11223},
        "pixelpilot": {
            "enabled": True, "bin": "/usr/bin/pixelpilot", "env": {},
            "configPath": "/etc/pixelpilot.yaml",
            "osdConfigPath": "/etc/pixelpilot/osd.json",
            "screenMode": "1920x1080@60", "videoScale": 1.0, "codec": "h265",
            "rtpPort": 5600, "rtpJitterMs": 0,
            "dvr": {
                "framerate": 60, "dir": "/media/dvr",
                "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
                "fmp4": True, "sequencedFiles": True, "osd": False,
                "mode": "raw", "maxSizeMb": 4000, "reencCodec": "h264",
                "reencBitrate": 8000, "reencFps": 30, "reencResolution": "1080p",
            },
            "extraArgs": [],
        },
    }
```

- [ ] **Step 2: Write failing ConfigStore tests for the new model**

Replace the body of `gs/tests/unit/test_config.py` with:

```python
import json

from fpvdgs.config import ConfigStore, deep_merge
from fpvdgs.config_defaults import default_config


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


def test_effective_is_defaults_when_no_loaded():
    s = ConfigStore({"link": {"channel": 132}})
    assert s.effective() == {"link": {"channel": 132}}


def test_loaded_deep_merges_onto_defaults():
    s = ConfigStore({"link": {"channel": 132, "width": 40}},
                    {"link": {"width": 20}})
    assert s.effective() == {"link": {"channel": 132, "width": 20}}


def test_missing_key_falls_back_to_default():
    s = ConfigStore({"link": {"channel": 132}, "drone": {"endpoint": "x"}},
                    {"link": {"channel": 9}})
    assert s.effective()["drone"]["endpoint"] == "x"   # untouched key defaults


def test_patch_accumulates_into_pending_without_touching_effective():
    s = ConfigStore({"link": {"channel": 132, "width": 40}})
    s.patch({"link": {"width": 20}})
    assert s.effective() == {"link": {"channel": 132, "width": 40}}
    assert s.pending() == {"link": {"channel": 132, "width": 20}}


def test_commit_promotes_pending_and_persists_full(tmp_path):
    cfg = tmp_path / "config.json"
    s = ConfigStore({"link": {"channel": 132, "width": 40}},
                    config_path=str(cfg))
    s.patch({"link": {"channel": 100}})
    s.commit()
    # persisted file is the FULL effective config (no sparse diff)
    assert json.loads(cfg.read_text()) == {"link": {"channel": 100, "width": 40}}


def test_reset_restores_defaults(tmp_path):
    cfg = tmp_path / "config.json"
    s = ConfigStore({"link": {"channel": 132}}, {"link": {"channel": 5}},
                    config_path=str(cfg))
    s.reset()
    assert s.effective() == {"link": {"channel": 132}}
    assert json.loads(cfg.read_text()) == {"link": {"channel": 132}}


def test_load_uses_code_defaults_when_no_file(tmp_path):
    s = ConfigStore.load(str(tmp_path / "absent.json"))
    assert s.effective() == default_config()


def test_load_merges_file_onto_code_defaults(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"link": {"channel": 100}}))
    s = ConfigStore.load(str(cfg))
    assert s.effective()["link"]["channel"] == 100
    assert s.effective()["drone"] == default_config()["drone"]   # defaulted


def test_legacy_key_in_file_does_not_break_load(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"probe": {"enabled": True}}))
    s = ConfigStore.load(str(cfg))               # must not raise
    assert s.effective()["link"]["channel"] == default_config()["link"]["channel"]
```

- [ ] **Step 3: Run the tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_config.py -q`
Expected: FAIL — `ConfigStore.load()` still takes two args; `effective()` recomputes from overlay; no `config_path` kwarg.

- [ ] **Step 4: Rewrite `ConfigStore`**

Replace the body of `gs/fpvdgs/config.py` with:

```python
"""Config store: code defaults + a single full config.json, deep-merged.

Mirrors the drone model — code (config_defaults.default_config) is the single
source of defaults; config.json holds the full effective config and is merged
onto the defaults so a missing key takes its default. Persistence rewrites the
full effective config (no sparse overlay)."""

import copy
import json
import os
import threading

from .config_defaults import default_config


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
    def __init__(self, defaults: dict, loaded: dict | None = None,
                 config_path: str | None = None):
        self._defaults = copy.deepcopy(defaults)
        self._config = deep_merge(self._defaults, loaded or {})
        self._pending = copy.deepcopy(self._config)
        self._config_path = config_path
        self._lock = threading.RLock()

    @classmethod
    def load(cls, config_path: str) -> "ConfigStore":
        defaults = default_config()
        loaded = {}
        if config_path and os.path.exists(config_path):
            with open(config_path) as f:
                loaded = json.load(f)
        return cls(defaults, loaded, config_path)

    def defaults(self) -> dict:
        return copy.deepcopy(self._defaults)

    def effective(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._config)

    def pending(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._pending)

    def patch(self, sparse: dict) -> None:
        with self._lock:
            self._pending = deep_merge(self._pending, sparse)

    def commit(self) -> None:
        with self._lock:
            self._config = copy.deepcopy(self._pending)
            self._persist()

    def reset(self) -> None:
        with self._lock:
            self._config = copy.deepcopy(self._defaults)
            self._pending = copy.deepcopy(self._defaults)
            self._persist()

    def _persist(self) -> None:
        if not self._config_path:
            return
        tmp = self._config_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._config, f, indent=2)
        os.replace(tmp, self._config_path)
```

- [ ] **Step 5: Run the ConfigStore tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_config.py -q`
Expected: PASS

- [ ] **Step 6: Update `build_app` + `main` in `supervisor.py`**

In `gs/fpvdgs/supervisor.py`:

(a) Add `import json` to the top imports (after `import argparse`).

(b) Change the `build_app` signature — drop `defaults_path`:

```python
def build_app(config_path, cfg_out, host, port,
              runner_cmd, ready_port=8103, ready_timeout=10.0, log_path=None,
              probe_spawn=None):
    store = ConfigStore.load(config_path)
```
(Replace the old first line `def build_app(defaults_path, overlay_path, cfg_out, host, port,` ... and the `store = ConfigStore.load(defaults_path, overlay_path)` line.)

(c) In `main`, replace the `--defaults`/`--config` args and the `build_app(...)` call:

```python
    p.add_argument("--config", default="/etc/fpvd/config.json")
    p.add_argument("--dump-config", action="store_true",
                   help="print the full default config as JSON and exit")
    p.add_argument("--cfg-out", default="/etc/wifibroadcast.cfg")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--log", default=None)
    p.add_argument("--runner", default=None,
                   help="runner command (default: this python -m fpvdgs.runner)")
    args = p.parse_args(argv)

    if args.dump_config:
        from .config_defaults import default_config
        print(json.dumps(default_config(), indent=2))
        return

    runner_cmd = (args.runner.split() if args.runner
                  else [sys.executable, "-m", "fpvdgs.runner"])
    app = build_app(args.config, args.cfg_out, args.host, args.port,
                    runner_cmd, log_path=args.log)
```
(Delete the old `p.add_argument("--defaults", ...)` line.)

- [ ] **Step 7: Update the test/e2e `build_app` call sites and delete `defaults.json`**

(a) `gs/tests/unit/test_app_wiring.py` — `test_build_app_wires_api_collaborators` (lines ~77-84): replace the two-file setup with a single config and the new signature:

```python
    config = tmp_path / "config.json"
    config.write_text('{"link": {"region": "US", "channel": 132, "width": 20, '
                      '"wlans": ["wlan0"]}}')

    app = sup.build_app(str(config), str(tmp_path / "out.cfg"),
                        "127.0.0.1", 0, runner_cmd=["true"])
```

(b) `gs/tests/integration/test_supervisor_e2e.py` — at each `build_app(...)` call (around lines 47, 127, 180) the test writes its config into a `defaults` file and passes `defaults_path=`/`overlay_path=`. Change each to write that config into the `config.json` path and call `build_app(str(config_json), str(cfg_out), ...)` (single config arg, no `defaults_path`). The written config is now a partial overlay merged onto code defaults — keep only the keys each test asserts on.

(c) Delete the shipped defaults file:

```bash
git rm gs/etc/defaults.json
```

- [ ] **Step 8: Drop `--defaults` from the init script**

In `gs/scripts/S99fpvd` line 5, remove `--defaults /etc/fpvd/defaults.json` from `ARGS` (keep `--config /etc/fpvd/config.json` and the rest):

```sh
ARGS="--config /etc/fpvd/config.json --cfg-out /etc/wifibroadcast.cfg --port 8080 --log $LOG"
```

- [ ] **Step 9: Verify `--dump-config` works**

Run: `cd gs && .venv/bin/python -m fpvdgs.supervisor --dump-config`
Expected: prints the full default config JSON (the `default_config()` tree) and exits 0.

- [ ] **Step 10: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS (all). If a test still references `defaults_path`/`overlay_path` or the old `ConfigStore.load(a, b)`, fix it to the new single-config form.

- [ ] **Step 11: Commit**

```bash
git add gs/fpvdgs/config_defaults.py gs/fpvdgs/config.py gs/fpvdgs/supervisor.py \
        gs/scripts/S99fpvd gs/tests/unit/test_config.py gs/tests/unit/test_app_wiring.py \
        gs/tests/integration/test_supervisor_e2e.py
git rm gs/etc/defaults.json
git commit -m "gs: code-as-default-source + single full config.json + --dump-config

Mirror the drone config model on the GS: ConfigStore loads config.json
deep-merged onto config_defaults.default_config() (code = single source);
commit persists the full effective config; drop etc/defaults.json and the
--defaults arg; add --dump-config. dynamicLink knobs unchanged (still tuning)."
```

---

## Task 2: Restructure `dynamicLink` — merge `SelectorConfig`, flatten `tuning`, freeze calibration reads

Merge `GateConfig`+`ProfileSelectionConfig` (+`PolicyConfig.starvation_windows`) into one `SelectorConfig` (dropping the four dead fields), replace the opaque `tuning` passthrough with explicit `selector`/`smoothing`/`flightlog`/`rssiNorm` blocks, and freeze the learned-prior internals, flightlog internals, and rssi-norm curve by no longer reading them from config. Freeze `videoStreamId` to the `"video"` constant.

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py`, `gs/fpvdgs/dynlink/config_build.py`, `gs/fpvdgs/config_defaults.py`, `gs/fpvdgs/dynlink/controller.py`
- Test: `gs/tests/unit/test_dl_config_build.py`, `gs/tests/unit/test_dl_policy_leading.py`

- [ ] **Step 1: Merge the dataclasses in `policy.py`**

In `gs/fpvdgs/dynlink/policy.py`, replace the `GateConfig` and `ProfileSelectionConfig` dataclasses (the whole block from `@dataclass\nclass GateConfig:` through the end of `class ProfileSelectionConfig`) with a single `SelectorConfig`:

```python
@dataclass
class SelectorConfig:
    """Probe-driven promote + reactive demote + timing/cadence.

    Promote: the `current+1` probe rung must read clean (EWMA success
    >= probe_viable_threshold) and fresh (within probe_freshness_ms) for
    promote_debounce_windows consecutive ticks, and clear the
    hold_modes_down_ms / min_between_changes_ms cooldowns. Demote: a kept
    Channel-B emergency (loss/fec/starvation), or a video on-air PER breach
    (video_demote_per). starvation_windows is the consecutive-starved-window
    count before link_starved feeds the emergency demote.
    """
    # Probe-driven promote
    probe_viable_threshold: float = 0.99
    probe_freshness_ms: float = 500.0
    promote_debounce_windows: int = 3
    # Reactive demote
    video_demote_per: float = 0.05
    emergency_loss_rate: float = 0.05
    emergency_fec_pressure: float = 0.80
    # MCS bound
    max_mcs: int = 7
    # Timing/cadence (promote cooldowns; demotes bypass them)
    hold_modes_down_ms: int = 2000
    min_between_changes_ms: int = 200
    # Total-blackout failsafe: consecutive starved windows before link_starved
    # feeds the emergency demote (10 Hz → 5 windows = 0.5 s).
    starvation_windows: int = 5
```

Also fix the now-stale comment in `gs/fpvdgs/dynlink/learned_prior.py:20`: change `# rung ceiling (matches GateConfig.max_mcs default and the drone)` to reference `SelectorConfig.max_mcs`.

- [ ] **Step 2: Update `PolicyConfig` and `LeadingSelector`/`Policy` to use `SelectorConfig`**

(a) `PolicyConfig` — replace its `gate`/`selection`/`starvation_windows` fields:

```python
@dataclass
class PolicyConfig:
    selector: SelectorConfig = field(default_factory=SelectorConfig)
    learned_prior: LearnedPriorConfig = field(default_factory=LearnedPriorConfig)
    flightlog: FlightLogConfig = field(default_factory=FlightLogConfig)
```

(b) `LeadingSelector.__init__` — take one config; alias `self.gate`/`self.sel` to it so the rest of the method bodies (which reference `self.gate.*` and `self.sel.*`) need no further change:

```python
    def __init__(self, cfg: SelectorConfig):
        # One merged config; alias both names so select()'s body is unchanged.
        self.gate = cfg
        self.sel = cfg
        cap = int(cfg.max_mcs)
        if cap < 0:
            raise ValueError(f"max_mcs={cfg.max_mcs} excludes every MCS")
        self._cap_mcs = cap
        start_mcs = 1
        if start_mcs > cap:
            start_mcs = cap
        self.state = LeadingState(current_mcs=start_mcs)
        self._reasons: list[str] = []
        self._promote_clean = 0
```
(Replace the old `def __init__(self, gate, sel):` and its `self.gate = gate` / `self.sel = sel` lines; keep everything below `self._cap_mcs = cap` as-is.)

(c) In `Policy.__init__`, change the selector construction:

```python
        self.leading = LeadingSelector(cfg.selector)
```
(was `LeadingSelector(cfg.gate, cfg.selection)`).

(d) In `Policy.tick`, change the three `self.cfg.gate`/`self.cfg.starvation_windows` references to `self.cfg.selector`:
- `self._starvation_count >= self.cfg.starvation_windows` → `>= self.cfg.selector.starvation_windows`
- `(1.0 - rung["per"]) >= self.cfg.gate.probe_viable_threshold` → `self.cfg.selector.probe_viable_threshold`
- `signals.residual_loss_w < self.cfg.gate.video_demote_per` → `self.cfg.selector.video_demote_per`

- [ ] **Step 3: Rewrite `config_build.py` to read the flat blocks**

Replace the entire body of `gs/fpvdgs/dynlink/config_build.py` with:

```python
# gs/fpvdgs/dynlink/config_build.py
"""Map fpvd's `dynamicLink` config block onto the policy/aggregator objects
the controller consumes, and build the controller snapshot.

The block is explicit (no opaque `tuning` passthrough): `selector` and
`smoothing` carry the tunable knobs; `flightlog`/`rssiNorm` expose only an
`enabled` toggle (their internals are frozen code constants); learned-prior
internals are frozen entirely. camelCase JSON maps to the dataclasses'
snake_case fields."""
from __future__ import annotations

from urllib.parse import urlparse

from .flightlog import FlightLogConfig
from .learned_prior import LearnedPriorConfig
from .policy import PolicyConfig, SelectorConfig
from .signals import RssiNormConfig, SignalAggregator


def build_policy_config(block: dict) -> PolicyConfig:
    sel = block.get("selector", {}) or {}
    d = SelectorConfig()
    selector = SelectorConfig(
        probe_viable_threshold=float(sel.get("probeViableThreshold", d.probe_viable_threshold)),
        probe_freshness_ms=float(sel.get("probeFreshnessMs", d.probe_freshness_ms)),
        promote_debounce_windows=int(sel.get("promoteDebounceWindows", d.promote_debounce_windows)),
        video_demote_per=float(sel.get("videoDemotePer", d.video_demote_per)),
        emergency_loss_rate=float(sel.get("emergencyLossRate", d.emergency_loss_rate)),
        emergency_fec_pressure=float(sel.get("emergencyFecPressure", d.emergency_fec_pressure)),
        max_mcs=int(block.get("maxMcs", d.max_mcs)),
        hold_modes_down_ms=int(sel.get("holdModesDownMs", d.hold_modes_down_ms)),
        min_between_changes_ms=int(sel.get("minBetweenChangesMs", d.min_between_changes_ms)),
        starvation_windows=int(sel.get("starvationWindows", d.starvation_windows)),
    )
    fl = block.get("flightlog", {}) or {}
    # flightlog internals are frozen — read only `enabled`.
    flightlog = FlightLogConfig(enabled=bool(fl.get("enabled", True)))
    return PolicyConfig(
        selector=selector,
        learned_prior=LearnedPriorConfig(),   # frozen: always-on, internal defaults
        flightlog=flightlog,
    )


def build_aggregator(block: dict) -> SignalAggregator:
    s = block.get("smoothing", {}) or {}
    rn = block.get("rssiNorm", {}) or {}
    d = SignalAggregator()
    dn = RssiNormConfig()
    # rssiNorm curve is frozen — read only `enabled` (the rollback toggle).
    rssi_norm = RssiNormConfig(enabled=bool(rn.get("enabled", dn.enabled)))
    return SignalAggregator(
        ewma_alpha_rssi=float(s.get("ewmaAlphaRssi", d.ewma_alpha_rssi)),
        ewma_alpha_fec=float(s.get("ewmaAlphaFec", d.ewma_alpha_fec)),
        ewma_alpha_burst=float(s.get("ewmaAlphaBurst", d.ewma_alpha_burst)),
        starvation_threshold_pps=float(
            s.get("starvationThresholdPps", d.starvation_threshold_pps)),
        rssi_norm=rssi_norm,
    )


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

- [ ] **Step 4: Update `config_defaults.py` to the flat `dynamicLink` shape**

In `gs/fpvdgs/config_defaults.py`, add imports at the top and replace the `"dynamicLink": {...}` literal in `default_config()` with a call to a new helper assembled from the dataclasses:

```python
from .dynlink.policy import SelectorConfig
from .dynlink.signals import SignalAggregator
```

```python
def _dynamic_link_defaults() -> dict:
    sel = SelectorConfig()
    agg = SignalAggregator()
    return {
        "enabled": False, "maxMcs": 5, "radioProfile": "m8812eu2",
        "droneAddr": None, "dronePort": 9999,
        "selector": {
            "probeViableThreshold": sel.probe_viable_threshold,
            "probeFreshnessMs": sel.probe_freshness_ms,
            "promoteDebounceWindows": sel.promote_debounce_windows,
            "videoDemotePer": sel.video_demote_per,
            "emergencyLossRate": sel.emergency_loss_rate,
            "emergencyFecPressure": sel.emergency_fec_pressure,
            "holdModesDownMs": sel.hold_modes_down_ms,
            "minBetweenChangesMs": sel.min_between_changes_ms,
            "starvationWindows": sel.starvation_windows,
        },
        "smoothing": {
            "ewmaAlphaRssi": agg.ewma_alpha_rssi,
            "ewmaAlphaFec": agg.ewma_alpha_fec,
            "ewmaAlphaBurst": agg.ewma_alpha_burst,
            "starvationThresholdPps": agg.starvation_threshold_pps,
        },
        "flightlog": {"enabled": True},
        "rssiNorm": {"enabled": True},
    }
```
Then in `default_config()` set `"dynamicLink": _dynamic_link_defaults(),` (replacing the old literal with `videoStreamId`/`tuning`). Note `maxMcs` defaults to `5` here (the operator cap), distinct from `SelectorConfig.max_mcs`'s `7` ceiling fallback.

- [ ] **Step 5: Freeze `videoStreamId` in the controller**

In `gs/fpvdgs/dynlink/controller.py` (in `_run`), replace:

```python
        video_id = (snap.get("videoStreamId") or "video").lower()
```
with:

```python
        video_id = "video"   # frozen: the wfb video stream id
```

- [ ] **Step 6: Rewrite the `config_build` tests**

Replace the body of `gs/tests/unit/test_dl_config_build.py` with:

```python
from fpvdgs.dynlink.config_build import (
    build_aggregator, build_policy_config, make_dl_snapshot,
)


def _block(**over):
    blk = {"enabled": True, "maxMcs": 5, "radioProfile": "m8812eu2",
           "droneAddr": None, "dronePort": 9999}
    blk.update(over)
    return blk


def test_maxmcs_maps_into_selector():
    cfg = build_policy_config(_block())
    assert cfg.selector.max_mcs == 5


def test_selector_block_overrides_defaults():
    cfg = build_policy_config(_block(selector={
        "probeViableThreshold": 0.95, "promoteDebounceWindows": 2,
        "holdModesDownMs": 1000, "minBetweenChangesMs": 100,
        "starvationWindows": 9,
    }))
    s = cfg.selector
    assert s.probe_viable_threshold == 0.95
    assert s.promote_debounce_windows == 2
    assert s.hold_modes_down_ms == 1000
    assert s.min_between_changes_ms == 100
    assert s.starvation_windows == 9
    # unspecified selector knobs keep their defaults
    assert s.video_demote_per == 0.05
    assert s.emergency_loss_rate == 0.05


def test_selector_defaults_when_absent():
    cfg = build_policy_config(_block())
    s = cfg.selector
    assert s.probe_viable_threshold == 0.99
    assert s.probe_freshness_ms == 500.0
    assert s.hold_modes_down_ms == 2000
    assert s.starvation_windows == 5


def test_smoothing_block_overrides_defaults():
    agg = build_aggregator(_block(smoothing={"ewmaAlphaRssi": 0.5,
                                              "starvationThresholdPps": 75}))
    assert agg.ewma_alpha_rssi == 0.5
    assert agg.starvation_threshold_pps == 75
    assert agg.ewma_alpha_fec == 0.2   # default


def test_learned_prior_is_frozen_defaults_regardless_of_config():
    # An attempt to tune learned-prior internals via config is ignored.
    cfg = build_policy_config(_block(learnedPrior={"binWidthDb": 3.0,
                                                   "minSamplesWarmstart": 7}))
    assert cfg.learned_prior.bin_width_db == 2.0
    assert cfg.learned_prior.min_samples_warmstart == 20


def test_flightlog_reads_only_enabled():
    cfg = build_policy_config(_block(flightlog={"enabled": False,
                                                "dir": "/tmp/ignored"}))
    assert cfg.flightlog.enabled is False
    assert cfg.flightlog.dir == "/media/dvr/log/dynamic-link/"   # frozen default


def test_rssi_norm_reads_only_enabled_curve_frozen():
    agg = build_aggregator(_block(rssiNorm={"enabled": False,
                                            "tx_power_dbm_by_mcs": [1, 2, 3]}))
    assert agg.rssi_norm.enabled is False
    assert agg.rssi_norm.p_ref_dbm == 29
    assert agg.rssi_norm.tx_power_dbm_by_mcs == (29, 28, 25, 23, 19, 19, 19, 19)


def test_rssi_norm_defaults_enabled():
    agg = build_aggregator(_block())
    assert agg.rssi_norm.enabled is True


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

- [ ] **Step 7: Update the `LeadingSelector` constructor in `test_dl_policy_leading.py`**

In `gs/tests/unit/test_dl_policy_leading.py`:

(a) Change the import (lines 10-16) to drop `GateConfig`/`ProfileSelectionConfig` and add `SelectorConfig`:

```python
from fpvdgs.dynlink.policy import (
    LeadingSelector,
    Policy,
    PolicyConfig,
    SelectorConfig,
)
```

(b) Replace the `_selector(...)` helper (lines 24-59) — drop the four dead params (`max_mcs_step_up`, `hold_fallback_mode_ms`, `fast_downgrade`, `upward_confidence_loops`) and build one `SelectorConfig`:

```python
def _selector(*,
              probe_viable_threshold: float = 0.99,
              probe_freshness_ms: float = 500.0,
              promote_debounce_windows: int = 3,
              video_demote_per: float = 0.05,
              emergency_loss_rate: float = 0.05,
              emergency_fec_pressure: float = 0.80,
              max_mcs: int = 7,
              hold_modes_down_ms: int = 0,
              min_between_changes_ms: int = 0,
              starvation_windows: int = 5,
              ) -> LeadingSelector:
    return LeadingSelector(SelectorConfig(
        probe_viable_threshold=probe_viable_threshold,
        probe_freshness_ms=probe_freshness_ms,
        promote_debounce_windows=promote_debounce_windows,
        video_demote_per=video_demote_per,
        emergency_loss_rate=emergency_loss_rate,
        emergency_fec_pressure=emergency_fec_pressure,
        max_mcs=max_mcs,
        hold_modes_down_ms=hold_modes_down_ms,
        min_between_changes_ms=min_between_changes_ms,
        starvation_windows=starvation_windows,
    ))
```
(The `test_max_mcs_too_low_raises` test expects the `ValueError` to match `"max_mcs"` — the new message `f"max_mcs={cfg.max_mcs} ..."` still matches.)

- [ ] **Step 8: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS. Note: `test_dl_controller.py` passes a `"tuning": {}` key in its snapshot — the new `build_policy_config` ignores unknown block keys (it reads `selector`/`smoothing`), so that test stays green; drop the stale `"tuning": {}` opportunistically. The likely red site is `test_supervisor_e2e.py` if it asserts on `tuning`-derived values — update its `dynamicLink` config to the flat `selector`/`smoothing` shape (or drop the empty `"tuning": {}`).

- [ ] **Step 9: Commit**

```bash
git add gs/fpvdgs/dynlink/policy.py gs/fpvdgs/dynlink/config_build.py \
        gs/fpvdgs/config_defaults.py gs/fpvdgs/dynlink/controller.py \
        gs/tests/unit/test_dl_config_build.py gs/tests/unit/test_dl_policy_leading.py \
        gs/tests/unit/test_dl_controller.py gs/tests/integration/test_supervisor_e2e.py
git commit -m "gs: de-opacify dynamicLink — merge SelectorConfig, flatten tuning, freeze calibration

Merge GateConfig+ProfileSelectionConfig(+starvation_windows) -> SelectorConfig
(drop dead max_mcs_step_up/hold_fallback_mode_ms/fast_downgrade/
upward_confidence_loops). Replace the opaque tuning passthrough with explicit
selector/smoothing blocks; freeze learned-prior internals, flightlog internals,
rssi-norm curve, and videoStreamId."
```

---

## Task 3: Validate the flat `dynamicLink` blocks + warn-on-unknown loader

Add range validation for the new `selector`/`smoothing`/`flightlog`/`rssiNorm` blocks (PATCH-strict), and a tolerant warn-on-unknown pass on load (boot-tolerant).

**Files:**
- Modify: `gs/fpvdgs/schema.py`, `gs/fpvdgs/config.py`
- Test: `gs/tests/unit/test_schema.py`, `gs/tests/unit/test_config.py`

- [ ] **Step 1: Write failing schema tests**

Append to `gs/tests/unit/test_schema.py`:

```python
def _dl(**over):
    base = {"enabled": True, "maxMcs": 5, "radioProfile": "m8812eu2",
            "droneAddr": None, "dronePort": 9999}
    base.update(over)
    return {"link": {"channel": 132, "region": "US", "width": 20},
            "dynamicLink": base}


def test_validate_effective_accepts_flat_dynamic_link():
    schema.validate_effective(_dl(selector={"probeViableThreshold": 0.9},
                                  smoothing={"ewmaAlphaRssi": 0.3},
                                  flightlog={"enabled": True},
                                  rssiNorm={"enabled": True}))  # no raise


def test_selector_probability_out_of_range_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(selector={"probeViableThreshold": 1.5}))


def test_smoothing_alpha_out_of_range_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(smoothing={"ewmaAlphaRssi": 0}))


def test_patch_rejects_unknown_dynamic_link_subkey():
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"dynamicLink": {"bogusKnob": 1}})


def test_patch_accepts_known_dynamic_link_keys():
    schema.validate_config_patch({"dynamicLink": {"selector": {}, "maxMcs": 4}})
```
(Ensure `import pytest` is present at the top of the file.)

- [ ] **Step 2: Run to verify failure**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_schema.py -q`
Expected: FAIL — no range checks for the flat blocks; unknown sub-key not rejected.

- [ ] **Step 3: Implement the validation in `schema.py`**

Add the known-key set and rewrite `_validate_dynamic_link` in `gs/fpvdgs/schema.py`:

```python
DYNAMIC_LINK_KEYS = {"enabled", "maxMcs", "radioProfile", "droneAddr",
                     "dronePort", "selector", "smoothing", "flightlog", "rssiNorm"}
SELECTOR_KEYS = {"probeViableThreshold", "probeFreshnessMs",
                 "promoteDebounceWindows", "videoDemotePer", "emergencyLossRate",
                 "emergencyFecPressure", "holdModesDownMs", "minBetweenChangesMs",
                 "starvationWindows"}
SMOOTHING_KEYS = {"ewmaAlphaRssi", "ewmaAlphaFec", "ewmaAlphaBurst",
                  "starvationThresholdPps"}
```

```python
def _validate_dynamic_link(dl: dict) -> None:
    unknown = set(dl) - DYNAMIC_LINK_KEYS
    if unknown:
        raise SchemaError(f"unknown dynamicLink keys: {sorted(unknown)}")
    max_mcs = dl.get("maxMcs", 5)
    if not isinstance(max_mcs, int) or isinstance(max_mcs, bool) or not 0 <= max_mcs <= 7:
        raise SchemaError("dynamicLink.maxMcs must be an int in 0..7")
    port = dl.get("dronePort", 9999)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("dynamicLink.dronePort must be an int in 1..65535")
    profile = dl.get("radioProfile", "m8812eu2")
    if not isinstance(profile, str) or not profile:
        raise SchemaError("dynamicLink.radioProfile must be a non-empty string")
    sel = dl.get("selector", {})
    if sel:
        _validate_block_keys("dynamicLink.selector", sel, SELECTOR_KEYS)
        for k in ("probeViableThreshold", "videoDemotePer", "emergencyLossRate",
                  "emergencyFecPressure"):
            _validate_prob(f"dynamicLink.selector.{k}", sel.get(k))
        for k in ("promoteDebounceWindows", "starvationWindows"):
            _validate_pos_int(f"dynamicLink.selector.{k}", sel.get(k))
        for k in ("probeFreshnessMs", "holdModesDownMs", "minBetweenChangesMs"):
            _validate_non_neg_num(f"dynamicLink.selector.{k}", sel.get(k))
    sm = dl.get("smoothing", {})
    if sm:
        _validate_block_keys("dynamicLink.smoothing", sm, SMOOTHING_KEYS)
        for k in ("ewmaAlphaRssi", "ewmaAlphaFec", "ewmaAlphaBurst"):
            _validate_alpha(f"dynamicLink.smoothing.{k}", sm.get(k))
        _validate_non_neg_num("dynamicLink.smoothing.starvationThresholdPps",
                              sm.get("starvationThresholdPps"))
    for sub in ("flightlog", "rssiNorm"):
        blk = dl.get(sub, {})
        if blk:
            _validate_block_keys(f"dynamicLink.{sub}", blk, {"enabled"})
            if not isinstance(blk.get("enabled", True), bool):
                raise SchemaError(f"dynamicLink.{sub}.enabled must be a bool")


def _validate_block_keys(name: str, blk: dict, known: set) -> None:
    if not isinstance(blk, dict):
        raise SchemaError(f"{name} must be an object")
    unknown = set(blk) - known
    if unknown:
        raise SchemaError(f"unknown {name} keys: {sorted(unknown)}")


def _validate_prob(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))
                          or not 0.0 <= v <= 1.0):
        raise SchemaError(f"{name} must be a number in 0..1")


def _validate_alpha(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))
                          or not 0.0 < v <= 1.0):
        raise SchemaError(f"{name} must be a number in (0,1]")


def _validate_pos_int(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, int) or v <= 0):
        raise SchemaError(f"{name} must be a positive int")


def _validate_non_neg_num(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))
                          or v < 0):
        raise SchemaError(f"{name} must be a non-negative number")
```

Then extend `validate_config_patch` so a PATCH rejects unknown `dynamicLink` sub-keys (interactive-strict). After the existing `link` check, add:

```python
    dl = sparse.get("dynamicLink")
    if dl is not None:
        if not isinstance(dl, dict):
            raise SchemaError("dynamicLink must be an object")
        unknown_dl = set(dl) - DYNAMIC_LINK_KEYS
        if unknown_dl:
            raise SchemaError(f"unknown dynamicLink keys: {sorted(unknown_dl)}")
```

- [ ] **Step 4: Run schema tests to verify pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_schema.py -q`
Expected: PASS. If `test_validate_effective_ok` / `test_dynamic_link_*` (lines ~50-76) still build `dynamicLink` with a `tuning: {}` key, update them to drop `tuning` (it's now an unknown key and is rejected by `_validate_dynamic_link`).

- [ ] **Step 5: Write the failing warn-on-unknown loader test**

Append to `gs/tests/unit/test_config.py`:

```python
def test_load_warns_on_unknown_keys(tmp_path, caplog):
    import logging
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({
        "bogusTop": 1,
        "dynamicLink": {"tuning": {"gate": {}}, "selector": {}},
    }))
    with caplog.at_level(logging.WARNING):
        s = ConfigStore.load(str(cfg))      # must not raise
    msgs = " ".join(r.message for r in caplog.records)
    assert "bogusTop" in msgs
    assert "tuning" in msgs                 # stale dynamicLink key warned
    assert s.effective()["link"]["channel"] == default_config()["link"]["channel"]
```

- [ ] **Step 6: Implement the warner in `config.py`**

Add to `gs/fpvdgs/config.py` (and call it from `load` after parsing the file):

```python
import logging

from .schema import DYNAMIC_LINK_KEYS

log = logging.getLogger("fpvdgs.config")


def _warn_unknown(loaded: dict, defaults: dict) -> None:
    """Warn (never fail) on keys in the loaded config absent from the code
    defaults. Scoped to the top level + the dynamicLink subtree — the blocks
    this cleanup restructured. Other blocks (pixelpilot/wfb/link) hold open
    maps (env, raw) and are left to value-validation, not key-walking."""
    for key in set(loaded) - set(defaults):
        log.warning("ignoring unknown config key: %s", key)
    dl = loaded.get("dynamicLink")
    if isinstance(dl, dict):
        for key in set(dl) - DYNAMIC_LINK_KEYS:
            log.warning("ignoring unknown dynamicLink key: %s", key)
```

In `ConfigStore.load`, call it after the file is parsed:

```python
        if config_path and os.path.exists(config_path):
            with open(config_path) as f:
                loaded = json.load(f)
            _warn_unknown(loaded, defaults)
```

- [ ] **Step 7: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 8: Commit**

```bash
git add gs/fpvdgs/schema.py gs/fpvdgs/config.py \
        gs/tests/unit/test_schema.py gs/tests/unit/test_config.py
git commit -m "gs: validate flat dynamicLink blocks (PATCH-strict) + warn-on-unknown load (boot-tolerant)"
```

---

## Task 4: `learnedPrior` always-on cleanup

Remove the `enabled` knob from the learned prior; the controller constructs it unconditionally whenever dynamic link runs. Simplify the now-always-true guards in `Policy`.

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py`, `gs/fpvdgs/dynlink/policy.py`
- Test: `gs/tests/unit/test_dl_policy_learned.py`, `gs/tests/unit/test_dl_policy_leading.py`

- [ ] **Step 1: Remove the `enabled` field from `LearnedPriorConfig`**

In `gs/fpvdgs/dynlink/learned_prior.py`, delete the `enabled: bool = True` line from the `LearnedPriorConfig` dataclass (keep all other fields).

- [ ] **Step 2: Make `Policy` construct the prior unconditionally**

In `gs/fpvdgs/dynlink/policy.py`, `Policy.__init__`, replace:

```python
        self.learned_prior = (
            LearnedPrior(profile_name, cfg.learned_prior)
            if cfg.learned_prior.enabled else None
        )
```
with:

```python
        # Always-on: the learned prior is an unconditional part of the loop.
        self.learned_prior = LearnedPrior(profile_name, cfg.learned_prior)
```

Then simplify the four now-always-true guards in `Policy.tick` (the prior is never `None`):
- `if not self._cold_started and signals.rssi is not None:` block — replace the inner `seed = (self.learned_prior.warmstart_seed(...) if self.learned_prior is not None else None)` with `seed = self.learned_prior.warmstart_seed(signals.rssi)`.
- `if (self.learned_prior is not None and signals.rssi is not None):` (predictive) → `if signals.rssi is not None:`.
- `if self.learned_prior is not None and signals.rssi is not None:` (ingest) → `if signals.rssi is not None:`.
- In the flightlog `write({...})` `"ceiling"` field: `(self.learned_prior.ceiling(signals.rssi) if self.learned_prior and signals.rssi is not None else None)` → `(self.learned_prior.ceiling(signals.rssi) if signals.rssi is not None else None)`.
- In `Policy.close`: `if self.learned_prior is not None:` → drop the guard, call `self.learned_prior.flush()` directly.

- [ ] **Step 3: Update the tests that disabled the prior**

(a) `gs/tests/unit/test_dl_policy_learned.py` line ~159: change `LearnedPriorConfig(enabled=True, persist_dir=str(tmp_path))` to `LearnedPriorConfig(persist_dir=str(tmp_path))` (drop the removed kwarg).

(b) `gs/tests/unit/test_dl_policy_leading.py` — `test_strong_rssi_does_not_raise_mcs_without_probe_or_prior` (lines ~240-262): the prior can no longer be disabled, so isolate it with an empty `persist_dir` (cold prior → no warm-start seed → MCS stays at boot). Change the signature to take `tmp_path` and the cfg line:

```python
def test_strong_rssi_does_not_raise_mcs_without_probe_or_prior(tmp_path):
    ...
    cfg = PolicyConfig(learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path)))
    policy = Policy(cfg)
```
(Keep the rest: a strong-RSSI `Signals`, assert `decision.mcs == 1`. With no probe and a cold prior there is no seed, so the boot MCS holds.)

- [ ] **Step 4: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS. If any other test constructs `LearnedPriorConfig(enabled=...)`, drop that kwarg.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/learned_prior.py gs/fpvdgs/dynlink/policy.py \
        gs/tests/unit/test_dl_policy_learned.py gs/tests/unit/test_dl_policy_leading.py
git commit -m "gs: learnedPrior always-on — drop the enabled knob, construct unconditionally"
```

---

## Task 5: Freeze probe constants

Delete the orphaned `effective['probe']` read in `make_probe_snapshot`; `rxL`/`ewmaAlpha`/`blackoutWindows` come from the frozen `PROBE_*` module constants (`PROBE_RX_L` stays `50`).

**Files:**
- Modify: `gs/fpvdgs/probe/config_build.py`
- Test: `gs/tests/unit/test_probe_config_build.py` (new)

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_probe_config_build.py`:

```python
from fpvdgs.probe.config_build import (
    make_probe_snapshot, PROBE_PORT, PROBE_RX_L, PROBE_EWMA_ALPHA,
    PROBE_BLACKOUT_WINDOWS,
)


def _eff(**dl):
    return {"link": {"linkId": 7669206, "wlans": ["wlan0"]},
            "dynamicLink": {"enabled": True, **dl}}


def test_snapshot_uses_frozen_probe_constants():
    snap = make_probe_snapshot(_eff())
    assert snap["port"] == PROBE_PORT
    assert snap["rxL"] == PROBE_RX_L == 50
    assert snap["ewmaAlpha"] == PROBE_EWMA_ALPHA
    assert snap["blackoutWindows"] == PROBE_BLACKOUT_WINDOWS


def test_snapshot_ignores_orphaned_probe_block():
    # A stale top-level `probe` block (hand-added on an old device) is no
    # longer honored — the knobs are frozen constants now.
    eff = _eff()
    eff["probe"] = {"rxL": 800, "ewmaAlpha": 0.9, "blackoutWindows": 99}
    snap = make_probe_snapshot(eff)
    assert snap["rxL"] == 50
    assert snap["ewmaAlpha"] == PROBE_EWMA_ALPHA
    assert snap["blackoutWindows"] == PROBE_BLACKOUT_WINDOWS
```

- [ ] **Step 2: Run to verify failure**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_probe_config_build.py -q`
Expected: FAIL — `make_probe_snapshot` still reads `effective['probe']`, so `test_snapshot_ignores_orphaned_probe_block` returns `rxL == 800`.

- [ ] **Step 3: Freeze the constants in `make_probe_snapshot`**

In `gs/fpvdgs/probe/config_build.py`, replace `make_probe_snapshot` (drop the `probe = effective.get("probe", ...)` read):

```python
def make_probe_snapshot(effective: dict) -> dict:
    """Snapshot for the single probe wfb_rx: fixed port + key/linkId/wlans.
    The per-window measurement knobs (rxL, ewmaAlpha, blackoutWindows) are
    frozen calibration constants — there is no config path (rxL=50 is
    consistent with selector.probeFreshnessMs=500, so a probed rung never
    reads stale between wfb_rx stats batches)."""
    return {
        "port": PROBE_PORT,
        "rxL": PROBE_RX_L,
        "ewmaAlpha": PROBE_EWMA_ALPHA,
        "blackoutWindows": PROBE_BLACKOUT_WINDOWS,
        "key": GS_KEY,
        "linkId": effective.get("link", {}).get("linkId"),
        "wlans": resolve_wlans(effective),
    }
```

- [ ] **Step 4: Run to verify pass + full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_probe_config_build.py -q && .venv/bin/python -m pytest tests/ -q`
Expected: PASS (both).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/probe/config_build.py gs/tests/unit/test_probe_config_build.py
git commit -m "gs: freeze probe knobs (rxL=50/ewmaAlpha/blackoutWindows) — drop orphaned effective['probe'] read"
```

---

## Task 6: Deploy + docs

Stop shipping `defaults.json`; seed `config.json` from `--dump-config`; delete the stale seed file + build artifact; rewrite the GS `dynamicLink` API doc and add a tuning reference.

**Files:**
- Modify: `deploy/gs/deploy.sh`, `docs/api.md`
- Create: `docs/gs-dynamic-link-tuning.md`
- Delete: `deploy/gs/config.json`, `gs/build/lib/fpvdgs/dynlink/profiles/m8812eu2.json` (if tracked)

- [ ] **Step 1: Update `deploy/gs/deploy.sh`**

(a) Remove the `defaults.json` scp (line ~48):
```
scp -O "${SSH_OPTS[@]}" "$GS/etc/defaults.json" "$TARGET:/etc/fpvd/defaults.json"
```
Delete that line.

(b) Replace the initial-config seed block (lines ~50-57) to seed from `--dump-config` atomically, after the package is installed:

```sh
# initial config.json — generated from the code defaults (fpvd --dump-config),
# installed ONLY on first deploy; never clobbers operator edits.
if remote 'test -e /etc/fpvd/config.json'; then
    echo "[skip] /etc/fpvd/config.json exists — operator config preserved"
else
    echo "[seed] config.json <- fpvd --dump-config"
    remote 'fpvd --dump-config > /etc/fpvd/config.json.tmp && mv /etc/fpvd/config.json.tmp /etc/fpvd/config.json'
fi
```
(The `fpvd` launcher is installed earlier in the script; `--dump-config` needs no device state. The temp+mv guards against a failed dump leaving an empty file.)

- [ ] **Step 2: Delete the stale seed file and build artifact**

```bash
git rm deploy/gs/config.json
git rm --ignore-unmatch gs/build/lib/fpvdgs/dynlink/profiles/m8812eu2.json
```
(The second is under `build/` — `--ignore-unmatch` is a no-op if it isn't tracked. Confirm with `git status` that nothing under `gs/build/` remains staged that shouldn't be.)

- [ ] **Step 3: Rewrite the GS `dynamicLink` API doc**

In `docs/api.md`, replace the GS `dynamicLink` section and its "Tuning passthrough" subsection (the block around lines 516-677 covering `tuning`, `gs.yaml`, and `profiles/<name>.json`) with the flat schema: document the exposed keys (`enabled`, `maxMcs`, `radioProfile`, `droneAddr`, `dronePort`) and the `selector`/`smoothing` blocks + the `flightlog`/`rssiNorm` `enabled` toggles, exactly as in `default_config()`. State that all other knobs (learned-prior internals, probe window, rssi-norm curve, flightlog storage, `videoStreamId`) are frozen code constants documented in `docs/gs-dynamic-link-tuning.md`. Remove every `tuning`, `gs.yaml`, and `profiles/<name>.json` reference.

- [ ] **Step 4: Add the tuning reference doc**

Create `docs/gs-dynamic-link-tuning.md` documenting, for each **exposed** knob: purpose + valid range (group `selector` as operational, `smoothing` as advanced), and a **Frozen constants** section listing the learned-prior internals, probe constants (`rxL=50`, `ewmaAlpha=0.25`, `blackoutWindows=10`, `port=50`), the rssi-norm curve (`pRefDbm=29`, `txPowerDbmByMcs=(29,28,25,23,19,19,19,19)` — must mirror the drone `txpower_curve.hpp`), flightlog storage defaults, and `videoStreamId="video"`, each with its source file. The generated `config.json` (`fpvd --dump-config`) is the canonical inventory of exposed knobs; this doc adds semantics.

- [ ] **Step 5: Sanity-check no stale references remain**

Run: `cd /home/gilankpam/Projects/drone/fpvd && grep -rn "tuning\|gs.yaml\|profiles/" docs/api.md gs/fpvdgs/ ; grep -rn "defaults.json" deploy/gs/ gs/scripts/`
Expected: no live references in `docs/api.md` (only the new doc), `gs/fpvdgs/`, the deploy script, or the init script. (Spec/plan history under `docs/superpowers/` may still mention them — that's fine.)

- [ ] **Step 6: Run the full suite one final time**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS.

- [ ] **Step 7: Commit**

```bash
git add deploy/gs/deploy.sh docs/api.md docs/gs-dynamic-link-tuning.md
git rm deploy/gs/config.json
git commit -m "gs deploy/docs: seed config.json via --dump-config, drop defaults.json + stale seed; doc the flat dynamicLink schema"
```

---

## Self-Review notes (already reconciled)

- **Spec coverage:** Change 1 → Task 1 (+ warner in Task 3). Change 2 (flatten/validate) → Tasks 2-3. Change 3 (merge) → Task 2. Change 4 (freezes: learnedPrior/probe/flightlog/rssiNorm/videoStreamId) → Tasks 2, 4, 5. Change 5 (deploy/docs) → Task 6.
- **Green-at-every-commit:** each task ends with the full `pytest tests/ -q` and lists the likely red sites (`test_dl_controller.py`, `test_supervisor_e2e.py`) so partial-refactor breakage is fixed before commit.
- **Type consistency:** `SelectorConfig` (Task 2) is referenced identically in `config_build` (Task 2), `config_defaults` (Task 2), `schema` keys (Task 3), and the policy tests; `DYNAMIC_LINK_KEYS` is defined in `schema.py` (Task 3) and imported by `config.py`'s warner (Task 3); `PROBE_*` constants (Task 5) are imported by the new probe test.
- **Known deferrals:** `radioProfile` is intentionally **not** renamed (would orphan learned-prior history). The warner is scoped to top-level + `dynamicLink` (other blocks hold open maps).
