# GS Config Facade (Plan 2C of 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the GS `/config` + `/apply` the single front door PixelPilot uses: `GET /config` returns ONE unified Option-C tree merged from the GS-local config and a live drone `/config` fetch; `PATCH /config` and `POST /apply` route each leaf to the right side (GS-local / shared-link coordinator / drone proxy); `/link` and `/air` are retired from the consumer surface.

**Architecture:** This is sub-plan **2C of 3** (the facade) of the unified-config GS work. The drone stays authoritative over its own config and self-restores on reboot (non-goal to collapse it). The GS unifies only the front door via a **composed facade**: a new pure-mapping module `gs/fpvdgs/facade.py` translates between the flat GS+drone schemas and the nested unified tree; a thin `DroneConfigCache` holds a last-seen snapshot of the drone subtree so `GET /config` can render `_meta.droneStale` when the drone is unreachable; the `Api` `/config` + `/apply` handlers are rewired to the unified tree; `/link` + `/air` external routes are removed.

**IMPORTANT naming note:** the spec (`docs/superpowers/specs/2026-06-09-unified-config-design.md`) writes the adaptive-link feature as `adaptiveLink`. That rename was **reverted** — the GS and drone both use **`dynamicLink`**. So everywhere the spec says `adaptiveLink`, this plan uses **`dynamicLink`** (`dynamicLink.controller` = GS, `dynamicLink.applier` = drone, `dynamicLink.enabled` = shared/both). All other spec details (tree shape, routing, `_meta`, apply lanes, per-field policy) stand.

**Tech Stack:** Python 3, stdlib `http.server`, pytest. No mocking framework — tests construct dicts and a `FakeDrone`/`FakeStore` and assert on returned dicts.

**Build & test (from `gs/`):** `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest`. Single test: `.venv/bin/python -m pytest tests/unit/test_facade.py::<name>`. Keep the suite green at the end of every task (baseline: 322 passing).

---

## The unified tree (target `GET /config`)

```jsonc
{
  "_meta": { "droneReachable": true, "droneLastSeen": "2026-06-10T…Z", "droneStale": false },
  "link": {
    "channel": 132, "width": 20, "linkId": 7669206,        // SHARED  → BOTH
    "beamforming": { "enabled": false },                    // SHARED  → BOTH
    "gs":    { "region": "US", "rxpower": null, "wlans": "auto" },   // → GS
    "drone": { "mcs": 3, "txpower": 25, "txpowerCurve": null, "fec": {"k":8,"n":12},
               "stbc": false, "ldpc": false, "mtu": 1500, "wlanAdapter": null }  // → DRONE (stale when unreachable)
  },
  "dynamicLink": {
    "enabled": false,                                       // SHARED → BOTH (hard-gated)
    "controller": { "maxMcs": 5, "radioProfile": "m8812eu2", "dronePort": 9999, "tuning": {} },  // → GS
    "applier": { "healthTimeoutMs": 10000, "interleavingSupported": true, "minIdrIntervalMs": 500,
                 "applyStaggerMs": 50, "applySubPaceMs": 5, "osd": {…}, "roiQp": {…},
                 "failsafe": {…}, "bitrate": {…}, "fec": {…} }   // → DRONE (drone's dynamicLink minus `enabled`)
  },
  "video": {…}, "image": {…}, "telemetry": {…}, "recording": {…}, "services": {…},   // → DRONE passthrough
  "wfb": {…}, "pixelpilot": {…}, "droneLink": { "endpoint": "…" }                    // → GS passthrough
}
```

## Field → side routing (the single source of truth for the mapping)

| Unified path | Side | GS-local path | Drone path |
|---|---|---|---|
| `link.channel` / `width` / `linkId` / `beamforming` | **SHARED** | `link.<k>` | `link.<k>` (pushed by coordinator at apply) |
| `link.gs.region` / `rxpower` / `wlans` | GS | `link.<k>` | — |
| `link.drone.<k>` (mcs, txpower, txpowerCurve, fec, stbc, ldpc, mtu, wlanAdapter) | DRONE | — | `link.<k>` |
| `dynamicLink.enabled` | **SHARED/BOTH** (hard-gated) | `dynamicLink.enabled` | `dynamicLink.enabled` |
| `dynamicLink.controller.<k>` | GS | `dynamicLink.controller.<k>` | — |
| `dynamicLink.applier.<k>` | DRONE | — | `dynamicLink.<k>` |
| `video` / `image` / `telemetry` / `recording` / `services` | DRONE | — | `<section>` |
| `wfb` / `pixelpilot` / `droneLink` | GS | `<section>` | — |

So: **unified `link.drone` ≡ the drone's `link` minus the shared keys**; **unified `dynamicLink.applier` ≡ the drone's `dynamicLink` minus `enabled`**; **unified `link.gs` ≡ the GS's `link` minus the shared keys**.

---

## File map

| File | Change |
|---|---|
| `gs/fpvdgs/facade.py` | **NEW** — pure mapping: `build_config_tree(gs_eff, drone_cfg, meta)` (flat→unified) + `split_patch(unified_patch)` (unified→`(gs_sparse, drone_sparse, touches_shared_link)`) + `FacadeError` + the routing constant tables |
| `gs/fpvdgs/drone_cache.py` | **NEW** — `DroneConfigCache`: wraps `DroneClient.get_config()`, holds last-seen drone config + timestamp, returns `(drone_cfg, meta)` (live or stale) |
| `gs/fpvdgs/api.py` | rewire `GET/PATCH /config` + `POST /apply` to the unified tree; 3-lane apply; remove `/link` + `/air` routes; refactor `_apply_gs`/`apply_link` to compose + commit once |
| `gs/fpvdgs/link.py` | `apply_link` no longer self-commits / no `applyTo` — it becomes a lane the unified apply drives (commit hoisted to `/apply`) |
| `gs/fpvdgs/supervisor.py` | construct `DroneConfigCache`; pass to `Api`; drop the `/link`-era wiring that's gone |
| `gs/fpvdgs/status.py` | (optional, small) the `drone` summary digest already exists in `_dynamic_link_status`; extend only if needed |
| `gs/tests/unit/test_facade.py` | **NEW** — tree-build + split-patch tests |
| `gs/tests/unit/test_drone_cache.py` | **NEW** — live/stale cache tests |
| `gs/tests/unit/test_api.py`, `test_api_*` | rewrite `/config` + `/apply` tests to the unified tree; delete `/link` + `/air` tests |
| `docs/api.md`, `gs/README.md` | document the unified tree, `_meta`, apply lanes, removal of `/link`+`/air` |

---

## Task 1: `facade.py` — build the unified tree (read mapping)

**Files:**
- Create: `gs/fpvdgs/facade.py`
- Create: `gs/tests/unit/test_facade.py`

- [ ] **Step 1: Write the failing tests** — create `gs/tests/unit/test_facade.py`:

```python
from fpvdgs.facade import build_config_tree

GS_EFF = {
    "link": {"channel": 132, "width": 20, "linkId": 7669206,
             "beamforming": {"enabled": False},
             "region": "US", "rxpower": None, "wlans": "auto"},
    "dynamicLink": {"enabled": False,
                    "controller": {"maxMcs": 5, "radioProfile": "m8812eu2",
                                   "dronePort": 9999, "tuning": {}}},
    "wfb": {"profile": "gs"}, "pixelpilot": {"enabled": True},
    "droneLink": {"endpoint": "http://10.5.0.10:8080"},
}
DRONE_CFG = {
    "link": {"channel": 132, "width": 20, "linkId": 7669206,
             "beamforming": {"enabled": False, "remoteMac": "", "ackTimeout": 255, "intervalMs": 100},
             "mcs": 3, "txpower": 25, "txpowerCurve": None, "fec": {"k": 8, "n": 12},
             "stbc": False, "ldpc": False, "mtu": 1500, "wlanAdapter": None},
    "dynamicLink": {"enabled": False, "healthTimeoutMs": 10000, "failsafe": {"mcs": 1},
                    "bitrate": {"minBitrateKbps": 1000}, "fec": {"kMin": 2}},
    "video": {"codec": "h265", "fps": 60}, "image": {"mirror": False},
    "telemetry": {"router": "msposd"}, "recording": {"enabled": False}, "services": {},
}
META = {"droneReachable": True, "droneLastSeen": "2026-06-10T00:00:00Z", "droneStale": False}


def test_link_splits_shared_gs_drone():
    t = build_config_tree(GS_EFF, DRONE_CFG, META)["link"]
    # shared at the top (from the GS's authoritative copy)
    assert t["channel"] == 132 and t["width"] == 20 and t["linkId"] == 7669206
    assert t["beamforming"] == {"enabled": False}
    # gs sub = GS link minus shared
    assert t["gs"] == {"region": "US", "rxpower": None, "wlans": "auto"}
    # drone sub = drone link minus shared keys
    assert t["drone"]["mcs"] == 3 and t["drone"]["txpower"] == 25
    assert "channel" not in t["drone"] and "beamforming" not in t["drone"]


def test_dynamiclink_splits_controller_applier_enabled():
    dl = build_config_tree(GS_EFF, DRONE_CFG, META)["dynamicLink"]
    assert dl["enabled"] is False
    assert dl["controller"]["maxMcs"] == 5
    # applier = drone dynamicLink minus enabled
    assert "enabled" not in dl["applier"]
    assert dl["applier"]["healthTimeoutMs"] == 10000
    assert dl["applier"]["failsafe"] == {"mcs": 1}


def test_wholly_owned_sections_passthrough():
    t = build_config_tree(GS_EFF, DRONE_CFG, META)
    assert t["video"] == {"codec": "h265", "fps": 60}      # drone
    assert t["telemetry"] == {"router": "msposd"}          # drone
    assert t["wfb"] == {"profile": "gs"}                    # gs
    assert t["droneLink"] == {"endpoint": "http://10.5.0.10:8080"}  # gs
    assert t["_meta"] == META


def test_stale_drone_subtree_is_last_seen_not_blank():
    # drone_cfg is the cached last-seen; meta says stale. The tree still carries it.
    stale_meta = {"droneReachable": False, "droneLastSeen": "2026-06-10T00:00:00Z", "droneStale": True}
    t = build_config_tree(GS_EFF, DRONE_CFG, stale_meta)
    assert t["link"]["drone"]["mcs"] == 3   # last-seen, not blank
    assert t["_meta"]["droneStale"] is True


def test_never_seen_drone_yields_empty_drone_subtrees():
    t = build_config_tree(GS_EFF, None, {"droneReachable": False, "droneStale": True})
    assert t["link"]["drone"] == {}
    assert t["dynamicLink"]["applier"] == {}
    assert t["video"] == {}   # never-seen drone section is blank
    # GS side is always live
    assert t["link"]["gs"]["region"] == "US"
    assert t["dynamicLink"]["controller"]["maxMcs"] == 5
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_facade.py -q`
Expected: FAIL (`ModuleNotFoundError: fpvdgs.facade`).

- [ ] **Step 3: Create `gs/fpvdgs/facade.py`**

```python
"""Compose the GS-local config and the drone's config into ONE unified
Option-C tree (GET /config), and route unified patches back to each side.

Mapping (single source of truth):
  link.{channel,width,linkId,beamforming}  SHARED  (GS holds the live copy)
  link.gs.{region,rxpower,wlans}            GS      (GS link minus shared)
  link.drone.*                             DRONE   (drone link minus shared)
  dynamicLink.enabled                      SHARED/BOTH (hard-gated)
  dynamicLink.controller.*                 GS
  dynamicLink.applier.*                    DRONE   (drone dynamicLink minus enabled)
  video/image/telemetry/recording/services DRONE   (passthrough)
  wfb/pixelpilot/droneLink                 GS      (passthrough)
"""
from __future__ import annotations

SHARED_LINK_KEYS = ("channel", "width", "linkId", "beamforming")
GS_LINK_KEYS = ("region", "rxpower", "wlans")
DRONE_SECTIONS = ("video", "image", "telemetry", "recording", "services")
GS_SECTIONS = ("wfb", "pixelpilot", "droneLink")


class FacadeError(ValueError):
    """A unified PATCH touched an unknown or read-only path."""


def build_config_tree(gs_eff: dict, drone_cfg: dict | None, meta: dict) -> dict:
    """Merge the GS effective config and the drone config (live or last-seen,
    or None if never seen) into the unified Option-C tree. `meta` is the
    caller-built `_meta` block (reachability/staleness)."""
    gs_link = gs_eff.get("link", {})
    drone_link = (drone_cfg or {}).get("link", {})
    link = {k: gs_link[k] for k in SHARED_LINK_KEYS if k in gs_link}
    link["gs"] = {k: gs_link[k] for k in GS_LINK_KEYS if k in gs_link}
    link["drone"] = {k: v for k, v in drone_link.items() if k not in SHARED_LINK_KEYS}

    gs_dl = gs_eff.get("dynamicLink", {})
    drone_dl = (drone_cfg or {}).get("dynamicLink", {})
    dynamic_link = {
        "enabled": bool(gs_dl.get("enabled", False)),
        "controller": gs_dl.get("controller", {}),
        "applier": {k: v for k, v in drone_dl.items() if k != "enabled"},
    }

    out = {"_meta": meta, "link": link, "dynamicLink": dynamic_link}
    for s in DRONE_SECTIONS:
        out[s] = (drone_cfg or {}).get(s, {})
    for s in GS_SECTIONS:
        if s in gs_eff:
            out[s] = gs_eff[s]
    return out
```

- [ ] **Step 4: Run to verify they pass**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_facade.py -q`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/facade.py gs/tests/unit/test_facade.py
git commit -m "gs: facade.build_config_tree — merge GS + drone config into the unified Option-C tree"
```

---

## Task 2: `DroneConfigCache` — last-seen snapshot + `_meta`

**Files:**
- Create: `gs/fpvdgs/drone_cache.py`
- Create: `gs/tests/unit/test_drone_cache.py`

The cache wraps `DroneClient.get_config()`. On a successful read it refreshes the snapshot + timestamp and returns `(drone_cfg, meta{reachable:true, stale:false})`. On `DroneUnreachable` it returns `(last_seen_or_None, meta{reachable:false, stale:true})`. A `clock` callable (returns an ISO-8601 string) is injected for testability.

- [ ] **Step 1: Write the failing tests** — create `gs/tests/unit/test_drone_cache.py`:

```python
from fpvdgs.drone_cache import DroneConfigCache
from fpvdgs.drone_client import DroneUnreachable


class FakeDrone:
    def __init__(self):
        self.cfg = {"link": {"mcs": 3}}
        self.up = True
    def get_config(self):
        if not self.up:
            raise DroneUnreachable("down")
        return self.cfg


def test_live_read_refreshes_snapshot_and_meta():
    d = FakeDrone()
    clk = iter(["2026-06-10T00:00:01Z", "2026-06-10T00:00:02Z"])
    cache = DroneConfigCache(d, clock=lambda: next(clk))
    cfg, meta = cache.read()
    assert cfg == {"link": {"mcs": 3}}
    assert meta == {"droneReachable": True, "droneLastSeen": "2026-06-10T00:00:01Z",
                    "droneStale": False}


def test_unreachable_serves_last_seen_with_stale_meta():
    d = FakeDrone()
    clk = iter(["2026-06-10T00:00:01Z", "2026-06-10T00:00:09Z"])
    cache = DroneConfigCache(d, clock=lambda: next(clk))
    cache.read()                      # seeds the snapshot at ...01Z
    d.up = False
    cfg, meta = cache.read()
    assert cfg == {"link": {"mcs": 3}}   # last-seen, not None
    assert meta["droneReachable"] is False
    assert meta["droneStale"] is True
    assert meta["droneLastSeen"] == "2026-06-10T00:00:01Z"   # the last SUCCESSFUL read


def test_never_seen_drone_returns_none_cfg():
    d = FakeDrone(); d.up = False
    cache = DroneConfigCache(d, clock=lambda: "2026-06-10T00:00:09Z")
    cfg, meta = cache.read()
    assert cfg is None
    assert meta == {"droneReachable": False, "droneLastSeen": None, "droneStale": True}
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_drone_cache.py -q`
Expected: FAIL (no module).

- [ ] **Step 3: Create `gs/fpvdgs/drone_cache.py`**

```python
"""Thin last-seen cache of the drone's /config, so GET /config can render the
drone subtree (grayed via _meta.droneStale) when the drone is unreachable.
The drone stays authoritative; this is a read-only render aid only."""
from __future__ import annotations

import copy
import datetime

from .drone_client import DroneUnreachable


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class DroneConfigCache:
    def __init__(self, drone, *, clock=_utc_now_iso):
        self._drone = drone
        self._clock = clock
        self._last_cfg = None
        self._last_seen = None

    def read(self):
        """Return (drone_cfg_or_None, meta). Refreshes the snapshot on success;
        serves the last-seen snapshot with droneStale on failure."""
        try:
            cfg = self._drone.get_config()
        except DroneUnreachable:
            return (copy.deepcopy(self._last_cfg),
                    {"droneReachable": False, "droneLastSeen": self._last_seen,
                     "droneStale": True})
        self._last_cfg = copy.deepcopy(cfg)
        self._last_seen = self._clock()
        return (copy.deepcopy(cfg),
                {"droneReachable": True, "droneLastSeen": self._last_seen,
                 "droneStale": False})
```

- [ ] **Step 4: Run to verify they pass**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_drone_cache.py -q`
Expected: PASS (3 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/drone_cache.py gs/tests/unit/test_drone_cache.py
git commit -m "gs: DroneConfigCache — last-seen drone /config snapshot + _meta staleness"
```

---

## Task 3: wire `GET /config` to the unified tree

**Files:**
- Modify: `gs/fpvdgs/api.py` (the `GET /config` handler + `Api.__init__`)
- Modify: `gs/fpvdgs/supervisor.py` (construct `DroneConfigCache`, pass to `Api`)
- Test: `gs/tests/unit/test_api.py` (or a focused `test_api_config_get.py`)

After this task `GET /config` returns the unified tree; `PATCH /config` and `POST /apply` still use the OLD flat handlers (rewired in Tasks 4-5). `GET /config?pending=true` returns the unified tree built from `store.pending()` for the GS side (drone side is always its live/last-seen — the drone owns its pending).

- [ ] **Step 1: Add a `drone_cache` param to `Api.__init__`** (after `drone`): `drone_cache=None`, stored as `self.drone_cache`. In `supervisor.build_app`, construct `from .drone_cache import DroneConfigCache; drone_cache = DroneConfigCache(drone)` and pass `drone_cache=drone_cache` to `Api(...)`.

- [ ] **Step 2: Write the failing test** — add to `gs/tests/unit/test_api.py` a test that `GET /config` returns the unified tree. Use the file's existing Api test harness (a `_api(...)` helper builds an `Api` with a `FakeStore`/`FakeDrone`); construct it with a fake `drone_cache` whose `.read()` returns `(DRONE_CFG, META)`:

```python
def test_get_config_returns_unified_tree(/* use this file's Api harness */):
    # drone_cache.read() -> (drone_cfg, meta); store.effective() -> gs_eff
    # GET /config body must have _meta, link.gs, link.drone, dynamicLink.controller/applier
    code, body = api_get("/config")
    assert code == 200
    assert body["_meta"]["droneReachable"] is True
    assert body["link"]["gs"]["region"] == "US"
    assert body["link"]["drone"]["mcs"] == 3
    assert body["dynamicLink"]["controller"]["maxMcs"] == 5
    assert "applier" in body["dynamicLink"]
```
(Match the file's existing harness names — read `test_api.py` first to see how it constructs `Api` and issues requests. Provide a `FakeDroneCache` with a `.read()` returning `(drone_cfg, meta)`.)

- [ ] **Step 3: Rewire the `GET /config` handler** in `gs/fpvdgs/api.py`. Replace the body that returns `store.effective()`/`store.pending()` with:

```python
        from .facade import build_config_tree   # or top-of-file import
        gs_cfg = self.store.pending() if pending else self.store.effective()
        drone_cfg, meta = self.drone_cache.read()
        return 200, build_config_tree(gs_cfg, drone_cfg, meta)
```
(Keep the `?pending=true` parsing exactly as today; only the assembled body changes.)

- [ ] **Step 4: Run** the config-get test + full suite:

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_api.py -q`
Expected: the new test passes. **Some existing `/config` tests that asserted the OLD flat body will fail** — that is expected; migrate them to the unified tree shape here (the GS-flat `/config` body is gone). Do NOT touch `/apply`/`/link`/`/air` tests yet.

- [ ] **Step 5: Full suite**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest -q`
Expected: PASS (the only churn is `/config` GET tests migrated to the unified shape).

- [ ] **Step 6: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/api.py gs/fpvdgs/supervisor.py gs/tests/
git commit -m "gs: GET /config returns the unified Option-C tree (facade read path)"
```

---

## Task 4: `split_patch` + unified `PATCH /config` routing

**Files:**
- Modify: `gs/fpvdgs/facade.py` (add `split_patch`)
- Modify: `gs/fpvdgs/api.py` (rewire `PATCH /config`)
- Test: `gs/tests/unit/test_facade.py`, `gs/tests/unit/test_api.py`

`split_patch` turns a unified sparse PATCH into `(gs_sparse, drone_sparse, touches_shared_link)`. Shared link keys route to the GS pending only (the coordinator pushes them to the drone at apply); `dynamicLink.enabled` routes to BOTH (GS pending + drone). `PATCH /config` validates the GS portion locally (reject → 400, no mutation), proxies the drone portion (drone validates → 400 with field on reject, GS untouched), then patches the GS pending; it marks the drone lane dirty when a drone patch was sent.

- [ ] **Step 1: Write the failing `split_patch` tests** — add to `gs/tests/unit/test_facade.py`:

```python
from fpvdgs.facade import split_patch, FacadeError
import pytest

def test_split_link_gs_drone_shared():
    gs, drone, shared = split_patch({"link": {"channel": 140, "gs": {"rxpower": 20},
                                              "drone": {"mcs": 4}}})
    assert gs == {"link": {"channel": 140, "rxpower": 20}}   # shared + gs-sub flattened
    assert drone == {"link": {"mcs": 4}}                     # drone-sub flattened
    assert shared is True

def test_split_dynamiclink_controller_applier_enabled():
    gs, drone, shared = split_patch({"dynamicLink": {
        "enabled": True, "controller": {"maxMcs": 6}, "applier": {"failsafe": {"mcs": 2}}}})
    assert gs == {"dynamicLink": {"enabled": True, "controller": {"maxMcs": 6}}}
    assert drone == {"dynamicLink": {"enabled": True, "failsafe": {"mcs": 2}}}
    assert shared is False   # dynamicLink.enabled is a BOTH field but not the shared-LINK lane

def test_split_wholly_owned_sections():
    gs, drone, _ = split_patch({"video": {"bitrate": 9000}, "pixelpilot": {"enabled": False}})
    assert drone == {"video": {"bitrate": 9000}}
    assert gs == {"pixelpilot": {"enabled": False}}

def test_split_rejects_meta_and_unknown():
    with pytest.raises(FacadeError):
        split_patch({"_meta": {"droneStale": False}})
    with pytest.raises(FacadeError):
        split_patch({"bogus": {}})
```

- [ ] **Step 2: Run to verify fail** — `pytest tests/unit/test_facade.py -k split -q` → FAIL.

- [ ] **Step 3: Implement `split_patch`** in `gs/fpvdgs/facade.py`:

```python
def split_patch(patch: dict) -> tuple[dict, dict, bool]:
    """Route a unified sparse PATCH into (gs_sparse, drone_sparse, touches_shared_link).
    Shared link keys go to the GS pending only (the coordinator pushes them to the
    drone at apply). dynamicLink.enabled goes to BOTH. Raises FacadeError on a
    read-only (_meta) or unknown section."""
    gs: dict = {}
    drone: dict = {}
    for top, val in patch.items():
        if top == "_meta":
            raise FacadeError("_meta is read-only")
        elif top == "link":
            _split_link(val or {}, gs, drone)
        elif top == "dynamicLink":
            _split_dynamic_link(val or {}, gs, drone)
        elif top in DRONE_SECTIONS:
            drone[top] = val
        elif top in GS_SECTIONS:
            gs[top] = val
        else:
            raise FacadeError(f"unknown config section: {top!r}")
    touches_shared = bool(set((patch.get("link") or {})) & set(SHARED_LINK_KEYS))
    return gs, drone, touches_shared


def _split_link(link: dict, gs: dict, drone: dict) -> None:
    gs_link, drone_link = {}, {}
    for k, v in link.items():
        if k in SHARED_LINK_KEYS:
            gs_link[k] = v                 # shared → GS pending (coordinator pushes at apply)
        elif k == "gs":
            gs_link.update(v or {})        # link.gs.* → GS link.*
        elif k == "drone":
            drone_link.update(v or {})     # link.drone.* → drone link.*
        else:
            raise FacadeError(f"unknown link key: {k!r}")
    if gs_link:
        gs["link"] = gs_link
    if drone_link:
        drone["link"] = drone_link


def _split_dynamic_link(dl: dict, gs: dict, drone: dict) -> None:
    gs_dl, drone_dl = {}, {}
    for k, v in dl.items():
        if k == "enabled":
            gs_dl["enabled"] = v           # BOTH
            drone_dl["enabled"] = v
        elif k == "controller":
            gs_dl["controller"] = v        # GS
        elif k == "applier":
            drone_dl.update(v or {})       # applier.* → drone dynamicLink.*
        else:
            raise FacadeError(f"unknown dynamicLink key: {k!r}")
    if gs_dl:
        gs["dynamicLink"] = gs_dl
    if drone_dl:
        drone["dynamicLink"] = drone_dl
```

- [ ] **Step 4: Run** `pytest tests/unit/test_facade.py -q` → PASS.

- [ ] **Step 5: Rewire `PATCH /config`** in `gs/fpvdgs/api.py`. The new flow:

```python
        # parse body -> patch (unified)
        from .facade import split_patch, FacadeError
        try:
            gs_sparse, drone_sparse, _ = split_patch(patch)
        except FacadeError as e:
            return 400, {"error": "bad_config", "message": str(e)}
        # 1) validate the GS portion locally (no mutation) by merging onto pending
        if gs_sparse:
            merged = deep_merge(self.store.pending(), gs_sparse)
            try:
                self.schema.validate_effective(merged)
            except self.schema.SchemaError as e:
                return 400, {"error": "bad_config", "message": str(e)}
        # 2) proxy the drone portion (drone validates; rejects leave GS untouched)
        if drone_sparse:
            try:
                self.drone.patch_config(drone_sparse)
                self._drone_dirty = True
            except DroneRejected as e:
                return 400, {"error": "drone_rejected", "message": e.message, "details": e.body}
            except DroneUnreachable:
                return 502, {"error": "drone_unreachable"}
        # 3) patch the GS pending
        if gs_sparse:
            self.store.patch(gs_sparse)
        # return the unified pending tree
        drone_cfg, meta = self.drone_cache.read()
        return 200, build_config_tree(self.store.pending(), drone_cfg, meta)
```

Add `self._drone_dirty = False` in `Api.__init__`. Import `deep_merge` from `.config`, and `DroneRejected`/`DroneUnreachable` from `.drone_client` (some may already be imported). NOTE the old `validate_config_patch` / `validate_link_patch` calls in the PATCH handler are replaced by this split-and-validate flow.

- [ ] **Step 6: Migrate `test_api.py` PATCH tests** to the unified shape — a unified PATCH `{"pixelpilot": {...}}` still routes to GS; `{"video": {...}}` routes to the (fake) drone and sets `_drone_dirty`; `{"link": {"gs": {"rxpower": 20}}}` patches GS `link.rxpower`; `{"_meta": …}` → 400. Use the file's Api harness with a `FakeDrone` recording `patch_config` calls.

- [ ] **Step 7: Run the full suite** — `pytest -q`. Expected PASS (the `/config` PATCH tests migrated; `/apply` still old — Task 5).

- [ ] **Step 8: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/facade.py gs/fpvdgs/api.py gs/tests/
git commit -m "gs: PATCH /config routes the unified tree to GS pending + drone proxy"
```

---

## Task 5: unified `POST /apply` — 3-lane router + per-field policy

**Files:**
- Modify: `gs/fpvdgs/api.py` (`_apply_gs` → unified `_apply`)
- Modify: `gs/fpvdgs/link.py` (`apply_link` stops self-committing; commit hoists to `/apply`)
- Test: `gs/tests/unit/test_api.py`, `gs/tests/unit/test_link.py`

This is the heart of 2C. `POST /apply` fires the lanes whose leaves changed and returns a per-lane result. **The shared-link coordinator and the GS-local apply currently each call `store.commit()`** — they must be refactored so `/apply` commits the GS store **once** after all GS-side lanes succeed.

### Lanes & policy

| Lane | Trigger | Mechanism | Drone-unreachable policy |
|---|---|---|---|
| **Shared link** | `link.{channel,width,linkId,beamforming}` changed (GS pending vs effective) | coordinator: GS-first retune/bounce + best-effort drone push | **soft-degrade** → GS-only, `droneApplied:false` |
| **GS-local** | `wfb`/`pixelpilot`/`dynamicLink.controller`/`dynamicLink.enabled` changed | render+runner bounce (wfb), route pixelpilot, route dynamic-link controller | n/a (GS-local) |
| **Drone** | `self._drone_dirty` (a drone-routed PATCH happened) | `drone.apply()` | required (drone-routed) |
| **gate** | `dynamicLink.enabled` toggled (pending vs effective) | — | **hard-gate**: if drone unreachable → 409, change nothing |

- [ ] **Step 1: De-commit the coordinator.** In `gs/fpvdgs/link.py`, change `apply_link` so it does NOT call `store.commit()` and drop the `apply_to` parameter / the `applyTo gs|both` distinction (the unified apply always coordinates "both", auto-degrading). It still: validates, does the drone push (best-effort, soft-degrade on unreachable), decides live-retune vs bounce, renders, and **returns** its result dict (`gsApplied`, `droneApplied`, `droneReachable`, `inSync`, `mode`, `beamforming`, `droneError`) WITHOUT committing. Update `test_link.py`: the coordinator tests now assert the returned result + that `store.commit` was NOT called by the coordinator (the caller commits). Keep the rollback-on-failure behavior (restore last-good, bounce) — but rollback now means "do not let the caller commit"; signal failure via `gsApplied=False` and leave pending intact for the caller to discard via the render path. (Read `apply_link` fully; preserve the retune/bounce/beamforming logic verbatim, only hoist the commit.)

- [ ] **Step 2: Write the failing `/apply` lane tests** — in `gs/tests/unit/test_api.py`, drive these scenarios through the Api harness (FakeStore/FakeDrone/FakeLink/FakeRunner):
  - `pixelpilot`-only change → GS-local lane fires (pixelpilot restart), drone lane NOT fired, commit once. Result `{"gs": {...}, "drone": {"applied": false, "fired": false}}`.
  - `video`-only change (drone PATCH set `_drone_dirty`) → drone lane fires `drone.apply()`, GS lane no-op.
  - shared `link.channel` change, drone reachable → coordinator pushes, `droneApplied:true`.
  - shared `link.channel` change, drone unreachable → **soft-degrade**: `gsApplied:true, droneApplied:false`, apply still 200.
  - `dynamicLink.enabled: true` while drone unreachable → **hard-gate 409**, nothing committed.
  - `dynamicLink.enabled: true` while reachable → controller starts + drone enable pushed.

  Provide the exact assertions against the per-lane result body.

- [ ] **Step 3: Implement the unified `_apply`** in `gs/fpvdgs/api.py` (replacing `_apply_gs`). Pseudocode-to-fill (write the real code, no placeholders):

```python
    def _apply(self):
        pending = self.store.pending()
        effective = self.store.effective()
        result = {}

        # --- hard-gate: dynamicLink.enabled toggle requires the drone reachable ---
        en_old = effective.get("dynamicLink", {}).get("enabled", False)
        en_new = pending.get("dynamicLink", {}).get("enabled", False)
        if en_old != en_new and not self.drone.healthz():
            return 409, {"error": "drone_unreachable_for_arm",
                         "message": "dynamicLink.enabled requires the drone reachable"}

        # --- shared-link lane (coordinator) ---
        link_changed = pending.get("link") != effective.get("link")
        if link_changed:
            result["sharedLink"] = self.link.apply_link()   # no commit inside
            if not result["sharedLink"].get("gsApplied"):
                return 500, {"error": "link_apply_failed", **result}

        # --- GS-local lane (wfb / pixelpilot / dynamicLink.controller+enabled) ---
        wfb_changed = (self._without(pending, "dynamicLink", "pixelpilot", "link")
                       != self._without(effective, "dynamicLink", "pixelpilot", "link"))
        try:
            self.schema.validate_effective(pending)
        except self.schema.SchemaError as e:
            return 400, {"error": "bad_config", "message": str(e)}
        if wfb_changed:
            # render + runner bounce (existing logic, moved here; restore-on-failure)
            ...
        self._route_dynamic_link(effective.get("dynamicLink", {}),
                                 pending.get("dynamicLink", {}), pending)
        self._route_pixelpilot(effective.get("pixelpilot", {}),
                               pending.get("pixelpilot", {}), pending)
        result["gs"] = {"applied": True, "wfbBounced": wfb_changed}

        # --- commit the GS store ONCE (covers shared-link + GS-local) ---
        self.store.commit()

        # --- drone lane ---
        if self._drone_dirty:
            try:
                self.drone.apply()
                result["drone"] = {"fired": True, "applied": True}
            except DroneUnreachable:
                result["drone"] = {"fired": True, "applied": False, "reachable": False}
            except DroneRejected as e:
                result["drone"] = {"fired": True, "applied": False, "error": e.message}
            self._drone_dirty = False
        else:
            result["drone"] = {"fired": False, "applied": False}

        return 200, {"applied": True, **result}
```

Carefully port the existing `_apply_gs` wfb render + `runner.restart()` restore-on-failure block into the `if wfb_changed:` branch. The `_without(..., "link")` addition keeps the link lane out of the wfb diff (the coordinator owns link). Note: the coordinator's `apply_link` already rendered the cfg for link changes; ensure the wfb render path and the coordinator render path don't double-bounce — if `link_changed` and the coordinator already bounced, skip the wfb bounce for the link portion (the `_without(..., "link")` diff handles this: a link-only change yields `wfb_changed == False`).

- [ ] **Step 4: Run** the `/apply` lane tests + `test_link.py` → PASS.

- [ ] **Step 5: Full suite** — `pytest -q`. Migrate any remaining `_apply_gs`-era tests. Expected PASS.

- [ ] **Step 6: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/api.py gs/fpvdgs/link.py gs/tests/
git commit -m "gs: unified POST /apply — 3-lane router (shared-link / GS-local / drone) with per-field policy"
```

---

## Task 6: retire `/link` and `/air`; status drone-summary

**Files:**
- Modify: `gs/fpvdgs/api.py` (remove `/link`, `/link/apply`, `/air/*` routes + their helpers `_link_view`, `_proxy`)
- Modify: `gs/fpvdgs/supervisor.py` (drop any wiring that only fed `/link`/`/air`)
- Test: delete `/link` + `/air` tests; keep the coordinator unit tests (they drive `link.apply_link` directly)

- [ ] **Step 1: Remove the routes.** In `gs/fpvdgs/api.py`'s request router, delete the `GET/PATCH /link`, `POST /link/apply`, and `GET/PATCH/POST /air/*` branches and the now-unused `_link_view` / `_proxy` helpers. The `LinkCoordinator` object stays (the unified `/apply` drives it); the `DroneClient` stays (the facade + cache use it). Only the external HTTP routes go.

- [ ] **Step 2: Sweep tests.** Delete tests that exercise `/link`, `/link/apply`, `/air/*` as HTTP routes. Coordinator behavior is still covered by `test_link.py` (driving `apply_link` directly) and the `/apply` shared-link lane tests (Task 5). `grep -rn "'/link'\|\"/link\"\|/air" gs/tests` → every hit removed or migrated.

- [ ] **Step 3: Confirm the status drone-summary is sufficient.** `GET /status` already carries a `drone` sub-block (`reachable`, `dynamicLinkActive`) via `_dynamic_link_status` (supervisor.py). The spec's `/status` drone-summary field set is an open question driven by PixelPilot's menu; for THIS task, leave `/status` as-is (no `/air` resurrection needed — PP reads `/status` + the unified `/config`). If a test asserted PP used `/air/status`, point it at `/status`.

- [ ] **Step 4: Full suite** — `pytest -q`. Expected PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/api.py gs/fpvdgs/supervisor.py gs/tests/
git commit -m "gs: retire /link and /air external routes — folded into unified /config + /apply"
```

---

## Task 7: docs

**Files:**
- Modify: `docs/api.md` (GS section)
- Modify: `gs/README.md`

- [ ] **Step 1: Rewrite the GS API surface in `docs/api.md`** — the GS front door is now `GET/PATCH /config`, `POST /apply`, `GET /status`, `GET /healthz`, `POST /reset`, `GET /defaults`. Remove `/link`, `/link/apply`, `/air/*`. Document:
  - The unified tree (the `## The unified tree` block above) with `_meta`, `link.{shared, gs, drone}`, `dynamicLink.{enabled, controller, applier}`, the wholly-owned sections, and the section→side routing table.
  - The apply lanes + per-field policy table (soft-degrade shared link; hard-gate `dynamicLink.enabled`), and the per-lane `/apply` result shape.
  - `_meta.droneStale` semantics (drone subtrees are last-seen and grayed when stale).

- [ ] **Step 2: Update `gs/README.md`** — the API table (lines ~22-33) drops `/link` + `/air` rows and notes `/config` is the unified front door; add a one-line pointer to `docs/api.md` for the unified tree.

- [ ] **Step 3: Final full suite** — `cd gs && .venv/bin/python -m pytest -q` → PASS (all).

- [ ] **Step 4: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add docs/api.md gs/README.md
git commit -m "docs: unified GS /config facade — tree, apply lanes, retire /link + /air"
```

---

## Done criteria

- `cd gs && .venv/bin/python -m pytest` is green.
- `GET /config` returns the unified Option-C tree (`_meta` + `link.{shared,gs,drone}` + `dynamicLink.{enabled,controller,applier}` + drone passthrough sections + GS sections); drone subtrees are last-seen with `_meta.droneStale: true` when unreachable.
- `PATCH /config` routes each leaf (GS pending / drone proxy), validates per side, and rejects `_meta`/unknown sections with 400.
- `POST /apply` fires only the changed lanes, soft-degrades the shared link to GS-only when the drone is unreachable, hard-gates `dynamicLink.enabled` on drone reachability, and returns a per-lane result; the GS store commits exactly once per apply.
- `/link`, `/link/apply`, `/air/*` are gone; PixelPilot's surface is `/config` + `/apply` + `/status`.
- `docs/api.md` + `gs/README.md` describe the unified surface.

## Carry-forwards (NOT in 2C — track for cutover)

- **GS overlay migration / "go fresh config"** — the on-device `/etc/fpvd/config.json` is old-shape; decided "go fresh" (the shipped `deploy/gs/config.json` is already new-shape). Confirm the cutover step replaces the device overlay rather than migrating it.
- **Drone `safe`→`failsafe` overlay key** at cutover (spec open question).
- **88XXau txpower dBm** hardware validation before shipping to those airframes (deployment radio 8812eu unaffected).
- **GS RSSI/EIRP normalization** convergence (keep the `tuning.rssi_norm` mirror; sourcing from `/status.radio.txpowerCurve` is future work).
