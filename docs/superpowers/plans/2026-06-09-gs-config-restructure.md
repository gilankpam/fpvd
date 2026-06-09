# GS Config Restructure (Plan 2A of 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Reshape the ground-station fpvd's *own* config to the unified naming: rename `link.txpower`→`link.rxpower`, `dynamicLink`→`adaptiveLink` (with a `controller` sub-block + unified `enabled`, dropping the now-vestigial `bandwidth`/`txpower`/`videoStreamId` and the IDR keys), and `drone`→`droneLink`. Keep `/config /apply /link /air` working with the new GS-local shape.

**Architecture:** This is sub-plan **2A of 3** for the unified-config GS work. **Scope boundary:** the GS config *store* keeps a **flat `link`** (region/rxpower/wlans/channel/width/linkId/beamforming all at `link.*`). The Option-C `link.gs`/`link.drone` *nesting* and the drone-side merge are **2C** (the facade). The IDR-relay decouple is **2B**. This plan only renames/restructures GS-local config keys and rewires the readers.

**Tech Stack:** Python 3, stdlib `http.server`, pytest. No mocking framework — tests construct objects from dicts and assert on returned dicts.

**Build & test (from `gs/`):** `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest`. Single test: `.venv/bin/python -m pytest tests/unit/test_schema.py::<name>`. Keep the suite green at the end of every task.

**Spec:** `docs/superpowers/specs/2026-06-09-unified-config-design.md` (Type 2 station-role naming, Type 3 controller/applier + gated enable, controller field trim).

---

## Target GS-local config shape (the store/`defaults.json` after 2A)

```jsonc
{
  "link": {                          // FLAT in the store (2C nests for the API)
    "channel": 132, "width": 40, "linkId": 7669206,
    "beamforming": { "enabled": false },
    "region": "US", "rxpower": null, "wlans": "auto"   // rxpower: was txpower (GS = receiver)
  },
  "wfb": { "profile": "gs", "mavlink": { "peer": "..." }, "raw": {} },
  "droneLink": { "endpoint": "http://10.5.0.10:8080" },     // was "drone"
  "adaptiveLink": {                                          // was "dynamicLink"
    "enabled": false,
    "controller": {
      "maxMcs": 5, "radioProfile": "m8812eu2",
      "droneAddr": null, "dronePort": 9999, "tuning": {}
      // dropped: bandwidth (from link.width), txpower, videoStreamId, idrForward, idrPort
    }
  },
  "pixelpilot": { … unchanged … }
}
```

`videoStreamId` becomes the internal constant `"video"`; `bandwidth` is derived from `link.width`; the IDR keys are removed (2B makes the relay always-on).

---

## File map

| File | Change |
|---|---|
| `gs/etc/defaults.json` | reshape to the target shape above |
| `gs/fpvdgs/schema.py` | `LINK_KEYS` (txpower→rxpower); `CONFIG_TOP_KEYS` (drone→droneLink, dynamicLink→adaptiveLink); `validate_effective` (adaptiveLink/droneLink); `_validate_dynamic_link`→`_validate_adaptive_link` (drop bandwidth/txpower/videoStreamId/idr checks; validate `controller.*`) |
| `gs/fpvdgs/render.py` | `link['txpower']`→`link.get('rxpower')` |
| `gs/fpvdgs/radio.py` | `link.get('txpower')`→`link.get('rxpower')` |
| `gs/fpvdgs/link.py` | `_can_retune_live` set `txpower`→`rxpower`; comments |
| `gs/fpvdgs/dynlink/config_build.py` | `make_dl_snapshot`: read `adaptiveLink.controller` + `droneLink.endpoint`; inject `bandwidth` from `link.width`; hardcode `videoStreamId="video"`; `_raw_from_block` drops the txpower handling |
| `gs/fpvdgs/api.py` | `_apply_gs` exclusion + `_route_dynamic_link`: `dynamicLink`→`adaptiveLink` |
| `gs/fpvdgs/supervisor.py` | wiring: `effective["drone"]["endpoint"]`→`["droneLink"]`; `["dynamicLink"]["enabled"]`→`["adaptiveLink"]["enabled"]`; drone-status block |
| `gs/fpvdgs/status.py` | the `dynamicLink` status key + reads (rename to `adaptiveLink` for consistency) |
| `gs/tests/...` | update fixtures/asserts to the new shape |

> The `probe` lifecycle "rides adaptiveLink" — wherever the code keys probe start/stop or the apply-exclusion on `dynamicLink`, it moves to `adaptiveLink`.

---

## Task 1: Reshape `defaults.json` + schema

**Files:**
- Modify: `gs/etc/defaults.json`
- Modify: `gs/fpvdgs/schema.py`
- Test: `gs/tests/unit/test_schema.py`

- [ ] **Step 1: Write failing tests** — add to `gs/tests/unit/test_schema.py`:

```python
def test_config_patch_accepts_adaptivelink_and_dronelink():
    from fpvdgs import schema
    schema.validate_config_patch({"adaptiveLink": {"enabled": True}})
    schema.validate_config_patch({"droneLink": {"endpoint": "http://x:8080"}})

def test_config_patch_rejects_old_keys():
    import pytest
    from fpvdgs import schema
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"dynamicLink": {"enabled": True}})
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"drone": {"endpoint": "x"}})

def test_link_patch_accepts_rxpower_not_txpower():
    import pytest
    from fpvdgs import schema
    schema.validate_link_patch({"link": {"rxpower": 20}})
    with pytest.raises(schema.SchemaError):
        schema.validate_link_patch({"link": {"txpower": 20}})

def test_validate_adaptivelink_controller():
    import pytest
    from fpvdgs import schema
    schema.validate_effective({
        "link": {"channel": 132, "region": "US", "width": 40},
        "adaptiveLink": {"enabled": True,
                         "controller": {"maxMcs": 5, "radioProfile": "m8812eu2",
                                        "dronePort": 9999}},
    })
    with pytest.raises(schema.SchemaError):
        schema.validate_effective({
            "link": {"channel": 132, "region": "US", "width": 40},
            "adaptiveLink": {"controller": {"maxMcs": 9}},   # out of 0..7
        })
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_schema.py -k "adaptivelink or dronelink or rxpower or controller" -q`
Expected: FAIL (old keys still required; `txpower` still accepted; no `adaptiveLink` validation).

- [ ] **Step 3: Update `schema.py`**

Replace the key sets and rename the validator. New top of `gs/fpvdgs/schema.py`:

```python
LINK_KEYS = {"channel", "width", "rxpower", "region", "linkId", "beamforming", "wlans"}
CONFIG_TOP_KEYS = {"wfb", "droneLink", "adaptiveLink", "pixelpilot"}   # link excluded
ALL_TOP_KEYS = {"link"} | CONFIG_TOP_KEYS
```

In `validate_effective`, change the `dynamicLink` lookup + call:

```python
    al = cfg.get("adaptiveLink")
    if al is not None:
        _validate_adaptive_link(al)
```

Replace `_validate_dynamic_link` with `_validate_adaptive_link` (validates only the surviving controller fields):

```python
def _validate_adaptive_link(al: dict) -> None:
    if not isinstance(al.get("enabled", False), bool):
        raise SchemaError("adaptiveLink.enabled must be a bool")
    ctl = al.get("controller", {}) or {}
    max_mcs = ctl.get("maxMcs", 5)
    if not isinstance(max_mcs, int) or not 0 <= max_mcs <= 7:
        raise SchemaError("adaptiveLink.controller.maxMcs must be an int in 0..7")
    port = ctl.get("dronePort", 9999)
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("adaptiveLink.controller.dronePort must be an int in 1..65535")
    profile = ctl.get("radioProfile", "m8812eu2")
    if not (DL_PROFILES_DIR / f"{profile}.json").is_file():
        available = sorted(p.stem for p in DL_PROFILES_DIR.glob("*.json"))
        raise SchemaError(
            f"adaptiveLink.controller.radioProfile {profile!r} not found; available: {available}")
```

(Remove the now-unused `DL_BANDWIDTHS` if nothing else references it — `grep -n DL_BANDWIDTHS fpvdgs/`.)

- [ ] **Step 4: Reshape `gs/etc/defaults.json`** to the target shape (see "Target GS-local config shape" above): rename `link.txpower`→`link.rxpower`; rename top-level `drone`→`droneLink`; rename `dynamicLink`→`adaptiveLink` and restructure it to `{"enabled": false, "controller": {"maxMcs": 5, "radioProfile": "m8812eu2", "droneAddr": null, "dronePort": 9999, "tuning": {}}}` (dropping `bandwidth`, `txpower`, `videoStreamId`, `idrForward`, `idrPort`). Leave `wfb` and `pixelpilot` unchanged.

- [ ] **Step 5: Run the schema tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_schema.py -q`
Expected: PASS. (Other suites may still fail — fixed in later tasks.)

- [ ] **Step 6: Commit**

```bash
git add gs/etc/defaults.json gs/fpvdgs/schema.py gs/tests/unit/test_schema.py
git commit -m "gs: reshape config to adaptiveLink/droneLink/link.rxpower (schema + defaults)"
```

---

## Task 2: Rewire `render.py` and `radio.py` (txpower→rxpower)

**Files:**
- Modify: `gs/fpvdgs/render.py:23-26`
- Modify: `gs/fpvdgs/radio.py:35-49`
- Test: `gs/tests/unit/test_render.py`

- [ ] **Step 1: Update the failing test** — in `gs/tests/unit/test_render.py`, find the case asserting `wifi_txpower` is emitted from `link.txpower` and change the input to use `rxpower`. Add/adjust:

```python
def test_render_emits_rxpower_as_wifi_txpower():
    from fpvdgs.render import render_cfg
    out = render_cfg({"link": {"channel": 132, "width": 40, "region": "US",
                               "rxpower": 1500}, "wfb": {"profile": "gs", "raw": {}}})
    assert "wifi_txpower = 1500" in out

def test_render_omits_txpower_when_rxpower_null():
    from fpvdgs.render import render_cfg
    out = render_cfg({"link": {"channel": 132, "width": 40, "region": "US",
                               "rxpower": None}, "wfb": {"profile": "gs", "raw": {}}})
    assert "wifi_txpower" not in out
```

- [ ] **Step 2: Run to verify fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_render.py -k rxpower -q`
Expected: FAIL (render still reads `link['txpower']`).

- [ ] **Step 3: Update `render.py:23-26`**

```python
    lines.append(f"wifi_region = {_lit(link['region'])}")
    if link.get("rxpower") is not None:
        # NOTE: wfb-ng wifi_txpower is in mBm; leave unset to keep the driver default.
        lines.append(f"wifi_txpower = {_lit(link['rxpower'])}")
```

- [ ] **Step 4: Update `radio.py`** — change `txpower = link.get("txpower")` (line ~40) to `txpower = link.get("rxpower")`. Leave the local variable name `txpower` and the `iw … set txpower` commands as-is (those are the `iw` verb, not the config key). Update the surrounding comment to say "rxpower (GS card power)".

- [ ] **Step 5: Run to verify pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_render.py tests/unit/test_link.py -q`
Expected: render tests pass. (If `test_link.py` references `txpower`, fix in Task 3.)

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/render.py gs/fpvdgs/radio.py gs/tests/unit/test_render.py
git commit -m "gs: read link.rxpower (renamed from txpower) in render + radio retune"
```

---

## Task 3: Rewire `link.py` (LinkCoordinator)

**Files:**
- Modify: `gs/fpvdgs/link.py`
- Test: `gs/tests/unit/test_link.py`

- [ ] **Step 1: Update `link.py`**

In `_can_retune_live` (line ~54) change the allowed live-retune set from `{"channel", "width", "txpower", "region"}` to `{"channel", "width", "rxpower", "region"}`. Update the comment at line ~46 (`channel/width/txpower/region`) to `rxpower`. `DRONE_PUSH_KEYS = ("channel", "width", "linkId")` is unchanged (rxpower is per-side, never pushed). Update the module comment at line ~12 (`(region, wlans, txpower)`) to `rxpower`.

- [ ] **Step 2: Update tests** — in `gs/tests/unit/test_link.py`, change any test input that sets `link["txpower"]` to `link["rxpower"]` (these exercise the live-retune-vs-bounce decision). `grep -n txpower tests/unit/test_link.py`.

- [ ] **Step 3: Run**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_link.py -q`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add gs/fpvdgs/link.py gs/tests/unit/test_link.py
git commit -m "gs: link coordinator uses link.rxpower for live-retune classification"
```

---

## Task 4: Rewire `config_build.make_dl_snapshot` to the new shape

**Files:**
- Modify: `gs/fpvdgs/dynlink/config_build.py`
- Test: `gs/tests/unit/test_dl_config_build.py`

- [ ] **Step 1: Write failing test** — add to `gs/tests/unit/test_dl_config_build.py`:

```python
def test_make_dl_snapshot_reads_adaptivelink_controller_and_dronelink():
    from fpvdgs.dynlink.config_build import make_dl_snapshot
    eff = {
        "link": {"width": 40},
        "droneLink": {"endpoint": "http://10.0.0.9:8080"},
        "adaptiveLink": {"enabled": True,
                         "controller": {"maxMcs": 6, "dronePort": 9999,
                                        "radioProfile": "m8812eu2", "tuning": {}}},
    }
    snap = make_dl_snapshot(eff)
    assert snap["maxMcs"] == 6
    assert snap["droneAddr"] == "10.0.0.9"      # host parsed from droneLink.endpoint
    assert snap["dronePort"] == 9999
    assert snap["bandwidth"] == 40              # derived from link.width
    assert snap["videoStreamId"] == "video"     # internal constant
```

- [ ] **Step 2: Run to verify fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_config_build.py -k adaptivelink -q`
Expected: FAIL (reads `dynamicLink`/`drone` today).

- [ ] **Step 3: Update `make_dl_snapshot`** in `config_build.py`:

```python
def make_dl_snapshot(effective: dict) -> dict:
    """Self-contained snapshot the controller consumes. The controller block is
    adaptiveLink.controller; bandwidth is derived from link.width; the drone UDP
    target resolves from controller.droneAddr else the host of droneLink.endpoint."""
    al = effective.get("adaptiveLink", {})
    block = dict(al.get("controller", {}))
    block["enabled"] = bool(al.get("enabled", False))
    # bandwidth is the RF width — single source of truth is link.width (10/20 -> 20, 40 -> 40).
    width = int(effective.get("link", {}).get("width", 20))
    block["bandwidth"] = 40 if width == 40 else 20
    block["videoStreamId"] = "video"
    endpoint = effective.get("droneLink", {}).get("endpoint", "http://10.5.0.10:8080")
    host = urlparse(endpoint).hostname or "10.5.0.10"
    block["droneAddr"] = block.get("droneAddr") or host
    block["dronePort"] = int(block.get("dronePort") or 9999)
    return block
```

- [ ] **Step 4: Update `_raw_from_block`** — it currently reads `block["bandwidth"]`, `block["txpower"]`, `block["maxMcs"]`. Keep `bandwidth` and `maxMcs` (both present in the snapshot now), and **remove the `txpower` handling** (controller has no `txpower`):

```python
def _raw_from_block(block: dict) -> dict:
    raw = copy.deepcopy(block.get("tuning") or {})
    leading = raw.setdefault("leading_loop", {})
    gate = raw.setdefault("gate", {})
    if "bandwidth" in block:
        leading["bandwidth"] = int(block["bandwidth"])
    if "maxMcs" in block:
        gate["max_mcs"] = int(block["maxMcs"])
    return raw
```

- [ ] **Step 5: Run to verify pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_config_build.py -q`
Expected: PASS. (Update any existing case in that file that builds a `dynamicLink`/`drone`-shaped `effective` to the new keys.)

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/dynlink/config_build.py gs/tests/unit/test_dl_config_build.py
git commit -m "gs: make_dl_snapshot reads adaptiveLink.controller + droneLink; bandwidth from link.width"
```

---

## Task 5: Rewire `api.py` apply routing

**Files:**
- Modify: `gs/fpvdgs/api.py`
- Test: `gs/tests/unit/test_api.py`

- [ ] **Step 1: Update the apply exclusion + dynamic-link routing in `api.py`.** In `_apply_gs`, change the wfb-change exclusion (lines ~89-90):

```python
        wfb_changed = (self._without(pending, "adaptiveLink", "pixelpilot")
                       != self._without(effective, "adaptiveLink", "pixelpilot"))
```

and the routing call (lines ~100-101):

```python
        self._route_dynamic_link(effective.get("adaptiveLink", {}),
                                 pending.get("adaptiveLink", {}), pending)
```

In `_route_dynamic_link`, the `enabled` is now at the top of the `adaptiveLink` block (it already reads `dl_old.get("enabled")` / `dl_new.get("enabled")` — that still works since `adaptiveLink.enabled` is top-level). No other change needed there, but update the docstring/comment referring to `dynamicLink`.

- [ ] **Step 2: Update `test_api.py`** — change any test that PATCHes `{"dynamicLink": …}` or `{"drone": …}` to `{"adaptiveLink": …}` / `{"droneLink": …}`, and any `link` patch using `txpower` to `rxpower`. `grep -n 'dynamicLink\|"drone"\|txpower' tests/unit/test_api.py`.

- [ ] **Step 3: Run**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_api.py -q`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add gs/fpvdgs/api.py gs/tests/unit/test_api.py
git commit -m "gs: apply routing keys off adaptiveLink (was dynamicLink)"
```

---

## Task 6: Rewire `supervisor.py` + `status.py`

**Files:**
- Modify: `gs/fpvdgs/supervisor.py`
- Modify: `gs/fpvdgs/status.py`
- Test: `gs/tests/unit/test_app_wiring.py`, `gs/tests/unit/test_status.py`, `gs/tests/integration/test_supervisor_e2e.py`

- [ ] **Step 1: Update `supervisor.py` wiring.** `grep -n 'dynamicLink\|\["drone"\]\|"drone"' fpvdgs/supervisor.py`. Replace:
  - `effective["drone"]["endpoint"]` → `effective["droneLink"]["endpoint"]` (DroneClient construction).
  - every `effective["dynamicLink"]["enabled"]` / `.get("dynamicLink", {})` → `effective["adaptiveLink"]["enabled"]` / `.get("adaptiveLink", {})` (App.start gating of dynlink+probe; any apply/status helper).
  - `_dynamic_link_status` keeps building the `drone` sub-block; no key change inside it.

- [ ] **Step 2: Update `status.py`.** The `build_status` param/key `dynamic_link` writes `out["dynamicLink"] = dynamic_link` (status.py:59). Rename the emitted key to `adaptiveLink` for consistency with the config:

```python
    if dynamic_link is not None:
        out["adaptiveLink"] = dynamic_link
```

Update the corresponding caller in `supervisor.py` (the status_fn) — the variable that holds the controller status is passed as `dynamic_link=`; keep the kwarg name or rename, but ensure the emitted key is `adaptiveLink`.

- [ ] **Step 3: Update tests.** In `test_status.py`, change assertions on `out["dynamicLink"]` → `out["adaptiveLink"]`. In `test_app_wiring.py` and `test_supervisor_e2e.py`, change any `effective` fixture / patch bodies using `dynamicLink`/`drone`/`txpower` to the new keys. `grep -rn 'dynamicLink\|"drone"\|txpower' tests/unit/test_app_wiring.py tests/unit/test_status.py tests/integration/test_supervisor_e2e.py`.

- [ ] **Step 4: Run the FULL suite**

Run: `cd gs && .venv/bin/python -m pytest -q`
Expected: PASS (all). Hunt down any straggler reference: `grep -rn 'dynamicLink\|"drone"\|link.*txpower\|videoStreamId\|idrForward\|idrPort' fpvdgs tests | grep -v idr_relay` — every hit must be intentional (e.g. a docstring) or fixed.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/supervisor.py gs/fpvdgs/status.py gs/tests/
git commit -m "gs: wire supervisor + status to adaptiveLink/droneLink"
```

---

## Task 7: Sweep for stragglers + docs

**Files:**
- Modify: any remaining `gs/fpvdgs/*` straggler
- Modify: `docs/api.md` (GS section)

- [ ] **Step 1: Full straggler sweep**

Run: `cd /home/gilankpam/Projects/drone/fpvd && grep -rn --include='*.py' -e 'dynamicLink' -e '"drone"' -e "'drone'" -e 'videoStreamId' -e 'idrForward' -e 'idrPort' gs/fpvdgs`
Every remaining hit must be either (a) the IDR relay internals in `dynlink/controller.py` (left for 2B), or (b) a string/docstring that is intentionally descriptive. Fix any real config read that still uses an old key. Also `grep -rn "link\['txpower'\]\|link.get(\"txpower\"\|link.get('txpower'" gs/fpvdgs` — expect none.

- [ ] **Step 2: Update `docs/api.md`** GS section (the "Ground-station API (fpvd-GS)" heading and below): rename `dynamicLink`→`adaptiveLink` with the `controller` sub-block (drop `bandwidth`/`txpower`/`videoStreamId`/`idrForward`/`idrPort` from the documented fields — note `videoStreamId` is now internal and bandwidth derives from `link.width`); rename `drone.endpoint`→`droneLink.endpoint`; rename GS `link.txpower`→`link.rxpower`. Do NOT yet document the unified `link.gs`/`link.drone` nesting or the merged drone view — that is 2C. Add a one-line note that the IDR relay config moved out (2B).

- [ ] **Step 3: Final full suite**

Run: `cd gs && .venv/bin/python -m pytest -q`
Expected: PASS (all).

- [ ] **Step 4: Commit**

```bash
git add gs/fpvdgs docs/api.md
git commit -m "docs/api + gs: finish adaptiveLink/droneLink/rxpower rename sweep"
```

---

## Done criteria

- `cd gs && .venv/bin/python -m pytest` is green.
- No `dynamicLink` / top-level `drone` / `link.txpower` / `videoStreamId` / config `idrForward`/`idrPort` references remain in `gs/fpvdgs` except the IDR-relay internals (2B) and intentional docstrings.
- `adaptiveLink.controller` holds only `maxMcs`/`radioProfile`/`droneAddr`/`dronePort`/`tuning`; `bandwidth` derives from `link.width`; `videoStreamId` is the constant `"video"`.
- `/config`, `/apply`, `/link`, `/air` still function with the new GS-local shape (the unified facade is 2C).
