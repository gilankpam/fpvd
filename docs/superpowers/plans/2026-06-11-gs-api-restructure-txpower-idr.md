# GS API Restructure + txpower→dBm + IDR Decouple — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Split the GS HTTP API into `/gs/*` (GS-local) + `/air/*` (drone proxy), remove the `/link` coordinator in favor of client orchestration, unify all txpower config on dBm, and give the IDR forwarder its own config block + lifecycle.

**Architecture:** Three independent parts. **Part A** (txpower→dBm) spans both daemons and is self-contained. **Part B** (GS API restructure) deletes `LinkCoordinator`, folds `link` into `/gs/config`, inlines a GS-local link-apply into `_apply_gs`, and makes the GS beamformee self-reconcile via the armer. **Part C** (IDR decouple) extracts an always-available `IdrRelay` module gated by a new `idrForward` block. Do the parts in order A → B → C; each leaves the tree green.

**Tech Stack:** Python 3 stdlib (GS daemon, `gs/fpvdgs/`), pytest. C++ with nlohmann/json + doctest (drone daemon, `drone/`). POSIX `sh` radio scripts.

**Test commands:**
- GS: `cd gs && python -m pytest tests/unit/<file> -v` (or `python -m pytest tests -q` for all)
- Drone: build then `cd drone && ./build/fpvd_tests "<test-name-substring>"` (NOT ctest). Configure/build with the project's existing CMake build dir (`cmake --build build`).

---

## Part A — txpower unified on dBm

The canonical unit at every config/API surface becomes **dBm**. The static key
is renamed `link.txpower` → `link.txPowerDbm` on both daemons. Edges convert to
the driver unit (`iw ... txpower fixed <mBm>`, `mBm = dBm * 100`), matching the
already-validated drone dynamic path (`radio_txpower.cpp`).

### Task A1: GS render — `link.txPowerDbm` → `wifi_txpower` (mBm)

**Files:**
- Modify: `gs/fpvdgs/render.py:18-26`
- Test: `gs/tests/unit/test_render.py`

- [ ] **Step 1: Update the existing render tests to the dBm contract**

In `gs/tests/unit/test_render.py`, the fixture link uses `"txpower": 19` and
asserts `wifi_txpower == 19`. Change the key to `txPowerDbm` and assert the mBm
conversion. Find the fixture (around line 7) and the assertion (around line 42):

```python
# fixture link block — change the key name:
"link": {"channel": 132, "width": 40, "txPowerDbm": 19, "region": "US",
```

```python
def test_render_includes_txpower():
    cfg = _parse(render_mod.render_cfg(_eff()))
    # 19 dBm -> 1900 mBm (wfb-ng wifi_txpower units)
    assert cfg["common"]["wifi_txpower"] == 1900
```

Leave `test_render_omits_txpower_when_unset` as-is except ensure its fixture
omits `txPowerDbm` (no txpower key) — it already asserts `wifi_txpower` absent.

- [ ] **Step 2: Run the tests, verify they fail**

Run: `cd gs && python -m pytest tests/unit/test_render.py -v`
Expected: FAIL — `wifi_txpower` is `19` (old key still read) or KeyError.

- [ ] **Step 3: Implement the conversion in render.py**

Replace `render.py:24-26`:

```python
    if link.get("txPowerDbm") is not None:
        # wfb-ng wifi_txpower is mBm (dBm * 100); leave unset to keep the
        # driver default.
        lines.append(f"wifi_txpower = {_lit(int(link['txPowerDbm']) * 100)}")
```

- [ ] **Step 4: Run the tests, verify they pass**

Run: `cd gs && python -m pytest tests/unit/test_render.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/render.py gs/tests/unit/test_render.py
git commit -m "gs/render: link.txPowerDbm (dBm) -> wifi_txpower mBm"
```

### Task A2: GS live retune — `txPowerDbm` → mBm at the `iw` edge

**Files:**
- Modify: `gs/fpvdgs/radio.py:27-52`
- Test: `gs/tests/unit/test_radio.py`

- [ ] **Step 1: Add/adjust a test asserting dBm→mBm in the fixed txpower arg**

In `gs/tests/unit/test_radio.py`, add (or adapt the existing txpower test):

```python
from fpvdgs import radio


def test_retune_commands_txpower_dbm_to_mbm():
    cmds = radio.retune_commands(["wlan0"], {"channel": 132, "width": 40,
                                             "region": "US", "txPowerDbm": 20})
    # 20 dBm -> 2000 mBm
    assert ["iw", "dev", "wlan0", "set", "txpower", "fixed", "2000"] in cmds


def test_retune_commands_txpower_none_is_auto():
    cmds = radio.retune_commands(["wlan0"], {"channel": 132, "width": 40,
                                             "region": "US", "txPowerDbm": None})
    assert ["iw", "dev", "wlan0", "set", "txpower", "auto"] in cmds
```

- [ ] **Step 2: Run, verify fail**

Run: `cd gs && python -m pytest tests/unit/test_radio.py -k txpower -v`
Expected: FAIL — current code reads `link.get("txpower")` and emits the raw value.

- [ ] **Step 3: Implement in radio.py**

In `retune_commands` (`radio.py:27-52`), change the txpower read + emit:

```python
    txpower_dbm = link.get("txPowerDbm")
    for wlan in wlans:
        if channel is not None:
            cmds.append(iw_args(wlan, channel, width))
        # None => 'auto' (driver default), so lowering back to null reverts live;
        # a value is dBm, converted to fixed mBm (wfb-ng's wifi_txpower units).
        if txpower_dbm is None:
            cmds.append(["iw", "dev", wlan, "set", "txpower", "auto"])
        else:
            cmds.append(["iw", "dev", wlan, "set", "txpower", "fixed",
                         str(int(txpower_dbm) * 100)])
```

Update the docstring line referencing "txpower is mBm" to "txPowerDbm is dBm,
converted to mBm".

- [ ] **Step 4: Run, verify pass**

Run: `cd gs && python -m pytest tests/unit/test_radio.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/radio.py gs/tests/unit/test_radio.py
git commit -m "gs/radio: retune reads link.txPowerDbm, converts dBm->mBm"
```

### Task A3: Drone — rename `Link.txpower` → `txPowerDbm`, validate dBm range

**Files:**
- Modify: `drone/src/config/schema.hpp:56,66-67`
- Modify: `drone/src/config/validate.cpp:56-57`
- Modify: `drone/src/config/diff.cpp:47`
- Modify: `drone/src/config/lock.cpp:12`
- Test: `drone/tests/unit/test_validate.cpp`

This is an atomic C++ rename (the tree won't compile half-done), so adjust the
test first, then rename across all four files together.

- [ ] **Step 1: Update the validate test to the dBm range**

In `drone/tests/unit/test_validate.cpp`, the existing `link.txpower` case checks
`1..63`. Replace it with a dBm-range case (`-10..30`, matching
`dynamicLink.safe.txPowerDbm`):

```cpp
TEST_CASE("validate: link.txPowerDbm in [-10,30]") {
    Config c{}; c.link.txPowerDbm = 31;
    auto errs = validate(c);
    REQUIRE(errs.size() >= 1);
    bool found = false;
    for (auto& e : errs) if (e.path == "link.txPowerDbm") found = true;
    CHECK(found);

    Config c2{}; c2.link.txPowerDbm = -11;
    auto errs2 = validate(c2);
    bool found2 = false;
    for (auto& e : errs2) if (e.path == "link.txPowerDbm") found2 = true;
    CHECK(found2);

    Config c3{}; c3.link.txPowerDbm = 20;   // in range
    auto errs3 = validate(c3);
    for (auto& e : errs3) CHECK(e.path != "link.txPowerDbm");
}
```

If a `test_schema.cpp` case references `link.txpower`, rename it to
`txPowerDbm` there too (round-trip test).

- [ ] **Step 2: Build, verify the test fails to compile/pass**

Run: `cd drone && cmake --build build 2>&1 | tail -5`
Expected: compile error — `Config::link` has no member `txPowerDbm`.

- [ ] **Step 3: Rename across schema/validate/diff/lock**

`schema.hpp:56` — rename the field and pick a dBm default (use the dynamic
`safe` default of 20 dBm, not a mechanical 1):

```cpp
    int txPowerDbm{20};
```

`schema.hpp:66-67` — the `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Link, ...)`
list: replace `txpower` with `txPowerDbm`:

```cpp
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Link, channel, width, txPowerDbm,
                                                mcs, fec, stbc, ldpc, linkId,
                                                mtu, wlanAdapter, beamforming)
```

`validate.cpp:56-57` — dBm range:

```cpp
    if (c.link.txPowerDbm < -10 || c.link.txPowerDbm > 30)
        errs.push_back({"link.txPowerDbm", "must be -10..30"});
```

`diff.cpp:47`:

```cpp
    c.nicTxpower    = la.txPowerDbm != lb.txPowerDbm;
```

`lock.cpp:12` — the locked path:

```cpp
    {"link", "txPowerDbm"},
```

(Update the `lock.cpp` comment that says "link.txpower IS locked" → "link.txPowerDbm".)

- [ ] **Step 4: Build + run validate/schema/diff/lock tests**

Run: `cd drone && cmake --build build && ./build/fpvd_tests "validate"` then
`./build/fpvd_tests "schema"` and `./build/fpvd_tests "diff"` `./build/fpvd_tests "lock"`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add drone/src/config/schema.hpp drone/src/config/validate.cpp \
        drone/src/config/diff.cpp drone/src/config/lock.cpp \
        drone/tests/unit/test_validate.cpp drone/tests/unit/test_schema.cpp
git commit -m "drone/config: rename link.txpower -> link.txPowerDbm (dBm, -10..30)"
```

### Task A4: Drone radio scripts + env — dBm×100, drop the per-driver hack

**Files:**
- Modify: `drone/src/supervise/radio.cpp:32,79` (env var name + field)
- Modify: `drone/scripts/radio-tune.sh:5,21-26`
- Modify: `drone/scripts/radio-up.sh:3,47-51`
- Test: `drone/tests/integration/test_radio_tune_script.cpp:41-62`

- [ ] **Step 1: Rewrite the radio-tune script test for the unified formula**

Replace the `"radio-tune.sh: txpower scaling sign per driver"` test
(`test_radio_tune_script.cpp:41-62`) — there is no longer a per-driver branch:

```cpp
TEST_CASE("radio-tune.sh: txpower dBm -> mBm (driver-independent)") {
    auto tmp = fs::temp_directory_path() / "fpvd-rt-txpower";
    fs::remove_all(tmp);
    auto rec = setupStubs(tmp);

    fpvd::Config c{};
    c.link.txPowerDbm = 20;          // 20 dBm -> 2000 mBm

    auto r1 = fpvd::tuneRadio("scripts/radio-tune.sh", "txpower", c, "wlan0", "88XXau");
    REQUIRE(r1.ok);
    CHECK(readAllText(rec).find("iw wlan0 set txpower fixed 2000") != std::string::npos);

    fs::remove(rec);
    auto r2 = fpvd::tuneRadio("scripts/radio-tune.sh", "txpower", c, "wlan0", "8812eu");
    REQUIRE(r2.ok);
    CHECK(readAllText(rec).find("iw wlan0 set txpower fixed 2000") != std::string::npos);

    fs::remove_all(tmp);
}
```

- [ ] **Step 2: Build, run, verify fail**

Run: `cd drone && cmake --build build && ./build/fpvd_tests "txpower"`
Expected: FAIL — compile error (`c.link.txpower` gone — already renamed in A3) or
wrong value (`250`/`-500`). After fixing the test's field name to `txPowerDbm`
the value assertion still fails until the script + env are updated.

- [ ] **Step 3: Update the env wiring in radio.cpp**

`drone/src/supervise/radio.cpp` exports the txpower env in two places
(lines ~32 and ~79). Rename both the env var and the field:

```cpp
    setenv("FPVD_TXPOWER_DBM", std::to_string(c.link.txPowerDbm).c_str(), 1);
```

- [ ] **Step 4: Update radio-tune.sh**

`radio-tune.sh` — replace the `txpower)` case (lines 21-26) with a single,
driver-independent dBm→mBm conversion, and update the inputs comment (line 5):

```sh
    txpower)
        # FPVD_TXPOWER_DBM is dBm; iw wants fixed mBm (dBm * 100). Matches the
        # adaptive-link radio path (radio_txpower.cpp).
        iw "$iface" set txpower fixed $(( ${FPVD_TXPOWER_DBM:-20} * 100 ))
        ;;
```

Update the comment header `#  FPVD_TXPOWER, FPVD_MTU` → `FPVD_TXPOWER_DBM, FPVD_MTU`.

- [ ] **Step 5: Update radio-up.sh**

`radio-up.sh:47-51` — replace the per-driver branch with:

```sh
iw $WLAN_DEV set txpower fixed $(( ${FPVD_TXPOWER_DBM:-20} * 100 ))
```

Update its inputs comment (line 3) `FPVD_TXPOWER` → `FPVD_TXPOWER_DBM`.

- [ ] **Step 6: Build, run, verify pass**

Run: `cd drone && cmake --build build && ./build/fpvd_tests "txpower"`
Expected: PASS (both driver args produce `2000`).

- [ ] **Step 7: Commit**

```bash
git add drone/src/supervise/radio.cpp drone/scripts/radio-tune.sh \
        drone/scripts/radio-up.sh drone/tests/integration/test_radio_tune_script.cpp
git commit -m "drone/radio: txpower dBm->mBm at the iw edge; drop per-driver level hack"
```

### Task A5: Defaults + fixtures — migrate txpower keys to dBm

**Files:**
- Modify: `gs/etc/defaults.json` (link block)
- Modify: `drone/etc/defaults.json:5`
- Modify: `drone/tests/fixtures/defaults.json:2`

- [ ] **Step 1: GS defaults**

In `gs/etc/defaults.json`, the link block has `"txpower": null`. Rename:

```json
    "txPowerDbm": null,
```

- [ ] **Step 2: Drone defaults**

`drone/etc/defaults.json:5` — `"txpower": 1` → choose an explicit dBm default
(20, matching the new schema default and the dynamic safe value):

```json
    "txPowerDbm": 20,
```

- [ ] **Step 3: Drone test fixture**

`drone/tests/fixtures/defaults.json:2` — change `"txpower": 1` → `"txPowerDbm": 20`
in the inline link block.

- [ ] **Step 4: Run the GS + drone suites to confirm nothing else references the old key**

Run: `cd gs && python -m pytest tests -q`
Run: `cd drone && cmake --build build && ./build/fpvd_tests`
Expected: PASS. If a store/schema test fails on the old key, grep
`grep -rn '"txpower"' gs/ drone/` and fix stragglers.

- [ ] **Step 5: Commit**

```bash
git add gs/etc/defaults.json drone/etc/defaults.json drone/tests/fixtures/defaults.json
git commit -m "defaults: migrate link.txpower -> link.txPowerDbm (dBm)"
```

> **Deployment note (not a code task):** the live, untracked `config.drone.json`
> (`link.txpower: 40`) and `config.gs.json` (`link.txpower: 1323`) must be
> hand-migrated at deploy time: `40` (raw level ×50 = 2000 mBm) → `txPowerDbm: 20`;
> `1323` mBm → `txPowerDbm: 13`. Call this out in the rollout, not in git.

---

## Part B — GS API restructure (`/gs` + `/air`, remove `/link`)

### Task B1: schema — fold `link` into `/gs/config`, drop `/link` validators

**Files:**
- Modify: `gs/fpvdgs/schema.py:1-35`
- Test: `gs/tests/unit/test_schema.py`

- [ ] **Step 1: Write/adjust schema tests for the folded link**

In `gs/tests/unit/test_schema.py`, add tests asserting `link` is now a valid
`/config` patch key and unknown link keys are still rejected:

```python
import pytest
from fpvdgs import schema


def test_config_patch_accepts_link():
    schema.validate_config_patch({"link": {"channel": 100}})  # no raise


def test_config_patch_rejects_unknown_link_key():
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"link": {"bogus": 1}})


def test_config_patch_rejects_unknown_top_key():
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"nope": {}})
```

Remove or update any existing test asserting "link is read-only via /config".

- [ ] **Step 2: Run, verify fail**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -k "link or top_key" -v`
Expected: FAIL — `validate_config_patch` currently raises on `link`.

- [ ] **Step 3: Implement schema changes**

Edit `gs/fpvdgs/schema.py`. Update `LINK_KEYS` (txpower→txPowerDbm), make `link`
a normal top key, validate link sub-keys in the config patch, and delete
`validate_link_patch`:

```python
LINK_KEYS = {"channel", "width", "txPowerDbm", "region", "linkId",
             "beamforming", "wlans"}
CONFIG_TOP_KEYS = {"link", "wfb", "drone", "dynamicLink", "pixelpilot",
                   "idrForward"}
DL_BANDWIDTHS = {20, 40}
DL_PROFILES_DIR = Path(__file__).resolve().parent / "dynlink" / "profiles"
VALID_WIDTHS = {10, 20, 40}


class SchemaError(ValueError):
    pass


def validate_config_patch(sparse: dict) -> None:
    """A /gs/config PATCH: any known top-level key, including `link`."""
    unknown = set(sparse) - CONFIG_TOP_KEYS
    if unknown:
        raise SchemaError(f"unknown config keys: {sorted(unknown)}")
    link = sparse.get("link")
    if link is not None:
        if not isinstance(link, dict):
            raise SchemaError("link must be an object")
        unknown_link = set(link) - LINK_KEYS
        if unknown_link:
            raise SchemaError(f"unknown link keys: {sorted(unknown_link)}")
```

Delete `validate_link_patch` entirely and remove the now-unused `ALL_TOP_KEYS`.

- [ ] **Step 4: Run, verify pass**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/schema.py gs/tests/unit/test_schema.py
git commit -m "gs/schema: fold link into /config patch; drop /link validator; txPowerDbm"
```

### Task B2: API — `/gs` prefix, remove `/link`, GS-local link-apply

**Files:**
- Modify: `gs/fpvdgs/api.py` (routes, `_apply_gs`, remove `_link_view`)
- Test: `gs/tests/unit/test_api.py`

This task removes the `link`/`LinkCoordinator` dependency from `Api` and adds a
fresh GS-local link applier. `Api.__init__` gains `retune`, `wlans_resolver`,
and `armer_tick` callables (injected by the supervisor) and drops `link`.

- [ ] **Step 1: Rewrite the api test harness + add routing/link-apply tests**

In `gs/tests/unit/test_api.py`, drop the `LinkCoordinator` import and rebuild
`_api()` to inject the new collaborators. Replace the top of the file:

```python
import json
import os
import tempfile

from fpvdgs import schema, render as render_mod
from fpvdgs.config import ConfigStore
from fpvdgs.api import Api


class FakeRunner:
    def __init__(self): self.restarts = 0
    def restart(self): self.restarts += 1; return True
    def state(self): return {"running": True, "pid": 1, "restarts": self.restarts,
                             "lastExit": None, "fault": False}


class FakeDrone:
    def __init__(self): self.calls = []
    def healthz(self): return True
    def _request(self, method, path, body=None):
        self.calls.append((method, path, body))
        return 200, json.dumps({"proxied": path}).encode()


def _api(retune_ok=True):
    cfg_out = os.path.join(tempfile.mkdtemp(), "wifibroadcast.cfg")
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"},
                         "wfb": {"profile": "gs"}, "drone": {"endpoint": "http://x"}},
                        overlay_path=None)
    drone = FakeDrone()
    runner = FakeRunner()
    retunes = []
    def retune(link): retunes.append(link); return retune_ok
    ticks = []
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone, status_fn=lambda: {"ok": True}, cfg_out=cfg_out,
              retune=retune, wlans_resolver=lambda cfg: ["wlan0"],
              armer_tick=lambda: ticks.append(1))
    return api, store, drone, runner, retunes, ticks, cfg_out
```

Then add routing + link-apply tests (and delete the old `/link`-based tests):

```python
def test_gs_routes_answer_under_gs_prefix():
    api, *_ = _api()
    assert api.handle("GET", "/gs/config", {}, b"")[0] == 200
    assert api.handle("GET", "/gs/status", {}, b"")[0] == 200
    assert api.handle("GET", "/gs/defaults", {}, b"")[0] == 200


def test_healthz_stays_at_root():
    api, *_ = _api()
    assert api.handle("GET", "/healthz", {}, b"")[0] == 200


def test_link_endpoints_gone():
    api, *_ = _api()
    assert api.handle("GET", "/link", {}, b"")[0] == 404
    assert api.handle("POST", "/link/apply", {}, b"")[0] == 404


def test_air_still_proxies():
    api, _, drone, *_ = _api()
    code, obj = api.handle("GET", "/air/config", {}, b"")
    assert code == 200 and ("GET", "/config", None) in drone.calls


def test_link_change_retunes_live_no_bounce():
    api, store, _, runner, retunes, ticks, _ = _api()
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and obj["applied"] is True
    assert retunes and retunes[-1]["channel"] == 100   # retuned
    assert runner.restarts == 0                          # no bounce


def test_link_change_bounces_on_wlans():
    api, store, _, runner, retunes, ticks, _ = _api()
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"link": {"wlans": ["wlan1"]}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1 and retunes == []


def test_failed_retune_falls_back_to_bounce():
    api, store, _, runner, retunes, ticks, _ = _api(retune_ok=False)
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"link": {"channel": 100}}).encode())
    code, obj = api.handle("POST", "/gs/apply", {}, b"")
    assert code == 200 and runner.restarts == 1   # retune failed -> bounced


def test_apply_fires_armer_tick():
    api, *rest = _api()
    ticks = rest[4]
    api.handle("POST", "/gs/apply", {}, b"")
    assert ticks == [1]
```

- [ ] **Step 2: Run, verify fail**

Run: `cd gs && python -m pytest tests/unit/test_api.py -v`
Expected: FAIL — `Api.__init__` signature mismatch / routes unknown.

- [ ] **Step 3: Rewrite api.py**

Replace `gs/fpvdgs/api.py` `__init__`, `handle`, `_apply_gs`, and remove
`_link_view`. Keep the existing `from .schema import SchemaError` import. New
constructor + routing:

```python
class Api:
    def __init__(self, store, schema, render_mod, runner, drone,
                 status_fn, cfg_out, dynlink=None, pixelpilot=None, probe=None,
                 retune=None, wlans_resolver=None, armer_tick=None,
                 idr_relay=None):
        self.store = store
        self.schema = schema
        self.render_mod = render_mod
        self.runner = runner
        self.drone = drone
        self.status_fn = status_fn
        self.cfg_out = cfg_out
        self.dynlink = dynlink
        self.pixelpilot = pixelpilot
        self.probe = probe
        self.retune = retune
        self.wlans_resolver = wlans_resolver
        self.armer_tick = armer_tick
        self.idr_relay = idr_relay

    def handle(self, method, path, query, body):
        try:
            if path.startswith("/air/"):
                return self._proxy(method, path, body)
            key = (method, path)
            if key == ("GET", "/healthz"):
                return 200, {"ok": True}
            if key == ("GET", "/gs/defaults"):
                return 200, self.store.defaults()
            if key == ("GET", "/gs/config"):
                pending = query.get("pending", ["false"])[0] == "true"
                return 200, (self.store.pending() if pending else self.store.effective())
            if key == ("PATCH", "/gs/config"):
                sparse = self._json(body)
                self.schema.validate_config_patch(sparse)
                self.store.patch(sparse)
                return 200, self.store.pending()
            if key == ("POST", "/gs/apply"):
                return self._apply_gs()
            if key == ("POST", "/gs/reset"):
                self.store.reset()
                self.render_mod.write_cfg(self.cfg_out,
                                          self.render_mod.render_cfg(self.store.effective()))
                self.runner.restart()
                return 200, {"reset": True}
            if key == ("GET", "/gs/status"):
                return 200, self.status_fn()
            return 404, {"error": "not found"}
        except SchemaError as e:
            return 400, {"error": str(e)}
        except Exception as e:
            return 500, {"error": str(e)}
```

`_apply_gs` — integrate the GS-local link apply, then route non-link blocks:

```python
    def _apply_gs(self):
        pending = self.store.pending()
        effective = self.store.effective()
        self.schema.validate_effective(pending)

        link_changed = pending.get("link") != effective.get("link")
        wfb_changed = (self._without(pending, "dynamicLink", "pixelpilot",
                                     "idrForward", "link")
                       != self._without(effective, "dynamicLink", "pixelpilot",
                                        "idrForward", "link"))

        if link_changed or wfb_changed:
            # Render the cfg the runner reads on a (re)start, before applying.
            self.render_mod.write_cfg(self.cfg_out,
                                      self.render_mod.render_cfg(pending))
            if not self._apply_link_local(effective.get("link", {}),
                                          pending.get("link", {}),
                                          force_bounce=wfb_changed):
                self.render_mod.write_cfg(self.cfg_out,
                                          self.render_mod.render_cfg(effective))
                self.runner.restart()
                return 500, {"applied": False,
                             "error": "apply failed; rolled back to last-good cfg"}

        self._route_dynamic_link(effective.get("dynamicLink", {}),
                                 pending.get("dynamicLink", {}), pending)
        self._route_pixelpilot(effective.get("pixelpilot", {}),
                               pending.get("pixelpilot", {}), pending)
        self._route_idr_forward(effective.get("idrForward", {}),
                                pending.get("idrForward", {}), pending)
        self.store.commit()
        if self.armer_tick is not None:
            self.armer_tick()
        return 200, {"applied": True}

    @staticmethod
    def _bw_class(width):
        return 40 if width == 40 else 20

    def _can_retune_live(self, old, new):
        """Live iw retune is safe only when changes are limited to fields iw can
        apply on a running monitor card AND the radiotap BW class is unchanged.
        beamforming is reconciled by the armer, so it is excluded here."""
        if self.retune is None:
            return False
        changed = {k for k in set(old) | set(new)
                   if k != "beamforming" and old.get(k) != new.get(k)}
        if not changed <= {"channel", "width", "txPowerDbm", "region"}:
            return False
        return self._bw_class(old.get("width")) == self._bw_class(new.get("width"))

    def _apply_link_local(self, old_link, new_link, force_bounce=False):
        """Apply the GS-local link delta: live retune when possible, else bounce.
        No drone push (client orchestrates). Returns True on success."""
        non_bf_changed = any(k != "beamforming" and old_link.get(k) != new_link.get(k)
                             for k in set(old_link) | set(new_link))
        if not force_bounce and non_bf_changed and self._can_retune_live(old_link, new_link):
            if self.retune(new_link):
                return True
            # live retune failed -> fall back to a bounce
        return self.runner.restart()
```

Keep `_route_dynamic_link`, `_route_pixelpilot`, `_without`, `_json`, `_proxy`
as-is (proxy still strips `/air`). Add `_route_idr_forward` in Part C (Task C3);
for now stub it so this task compiles:

```python
    def _route_idr_forward(self, old, new, pending):
        return  # implemented in Part C
```

- [ ] **Step 4: Run, verify pass**

Run: `cd gs && python -m pytest tests/unit/test_api.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/api.py gs/tests/unit/test_api.py
git commit -m "gs/api: /gs + /air routes; remove /link; GS-local link-apply (retune/bounce)"
```

### Task B3: Beamforming armer — full reconcile (arm AND disarm)

**Files:**
- Modify: `gs/fpvdgs/beamforming_armer.py:49-69`
- Test: `gs/tests/unit/test_beamforming_armer.py`

- [ ] **Step 1: Write tests for arm + disarm reconcile**

In `gs/tests/unit/test_beamforming_armer.py` (create if absent):

```python
from fpvdgs.beamforming_armer import BeamformingArmer


class FakeBf:
    def __init__(self, supported=True, state="disabled"):
        self._supported = supported
        self._state = state
        self.calls = []
    def supported(self, iface): return self._supported
    def status(self): return {"state": self._state}
    def reconcile(self, enabled, iface, mac):
        self.calls.append((enabled, iface, mac))
        self._state = "active" if enabled else "disabled"
        return self.status()


class FakeDrone:
    def __init__(self, reachable=True, mac="aa:bb:cc:dd:ee:ff"):
        self._reachable = reachable; self._mac = mac
    def healthz(self): return self._reachable
    def get_status(self): return {"beamforming": {"localMac": self._mac}}


def _armer(bf, drone, enabled):
    cfg = {"link": {"beamforming": {"enabled": enabled}}}
    return BeamformingArmer(bf, drone, lambda c: ["wlan0"], lambda: cfg)


def test_arms_when_enabled_and_inactive():
    bf = FakeBf(state="disabled")
    _armer(bf, FakeDrone(), True)._tick()
    assert bf.calls == [(True, "wlan0", "aa:bb:cc:dd:ee:ff")]


def test_disarms_when_disabled_and_active():
    bf = FakeBf(state="active")
    _armer(bf, FakeDrone(), False)._tick()
    assert bf.calls == [(False, "wlan0", "")]


def test_noop_when_already_in_desired_state():
    bf_on = FakeBf(state="active")
    _armer(bf_on, FakeDrone(), True)._tick()
    assert bf_on.calls == []
    bf_off = FakeBf(state="disabled")
    _armer(bf_off, FakeDrone(), False)._tick()
    assert bf_off.calls == []
```

- [ ] **Step 2: Run, verify fail**

Run: `cd gs && python -m pytest tests/unit/test_beamforming_armer.py -v`
Expected: FAIL — current `_tick` only arms; the disarm test fails.

- [ ] **Step 3: Implement full reconcile in `_tick`**

Replace `beamforming_armer.py` `_tick` (lines 49-69):

```python
    def _tick(self):
        """Reconcile the beamformee to config: arm when enabled+inactive,
        disarm when disabled+active. Reads the drone MAC read-only; never
        pushes to the drone (the client owns the drone-side handshake)."""
        cfg = self._cfg()
        bf = (cfg.get("link", {}) or {}).get("beamforming", {}) or {}
        want = bool(bf.get("enabled"))
        active = self._bf.status().get("state") == "active"

        if not want:
            if active:
                wlans = self._wlans(cfg) or []
                primary = wlans[0] if wlans else None
                if primary:
                    self._bf.reconcile(False, primary, "")
            return

        if active:
            return
        wlans = self._wlans(cfg) or []
        primary = wlans[0] if wlans else None
        if not primary or not self._bf.supported(primary):
            return
        if not self._drone.healthz():
            return
        try:
            mac = (self._drone.get_status()
                   .get("beamforming", {}).get("localMac", ""))
        except (DroneUnreachable, DroneRejected):
            return
        if mac:
            self._bf.reconcile(True, primary, mac)
```

Update the module docstring: it now disarms too.

- [ ] **Step 4: Run, verify pass**

Run: `cd gs && python -m pytest tests/unit/test_beamforming_armer.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/beamforming_armer.py gs/tests/unit/test_beamforming_armer.py
git commit -m "gs/bf-armer: full reconcile (arm + disarm) from config"
```

### Task B4: schema — reject enabling BF on an incapable card (fail-fast)

**Files:**
- Modify: `gs/fpvdgs/schema.py` (`validate_effective` + `_validate_beamforming`)
- Modify: `gs/fpvdgs/supervisor.py` (inject the capability probe into validate)
- Test: `gs/tests/unit/test_schema.py`

The capability check needs to know the primary card + whether it supports
`bf_monitor_conf`. Inject a `bf_capable(cfg) -> bool` callable into
`validate_effective` via a module-level hook the supervisor sets at boot, so the
schema stays import-light and unit tests can override it.

- [ ] **Step 1: Test the fail-fast rejection**

Add to `gs/tests/unit/test_schema.py`:

```python
def test_enable_bf_on_incapable_card_rejected():
    schema.set_bf_capable(lambda cfg: False)
    try:
        with pytest.raises(schema.SchemaError):
            schema.validate_effective({"link": {"channel": 1, "region": "US",
                                                 "beamforming": {"enabled": True}}})
    finally:
        schema.set_bf_capable(lambda cfg: True)


def test_enable_bf_on_capable_card_ok():
    schema.set_bf_capable(lambda cfg: True)
    schema.validate_effective({"link": {"channel": 1, "region": "US",
                                        "beamforming": {"enabled": True}}})
```

- [ ] **Step 2: Run, verify fail**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -k bf -v`
Expected: FAIL — `set_bf_capable` undefined.

- [ ] **Step 3: Implement the capability hook**

In `gs/fpvdgs/schema.py`, add a module hook (default: capable, so unit tests of
other modules are unaffected) and enforce it in `validate_effective`:

```python
_bf_capable = None   # callable(cfg) -> bool; None => unknown => allow


def set_bf_capable(fn) -> None:
    global _bf_capable
    _bf_capable = fn
```

In `validate_effective`, after computing `bf = link.get("beamforming")` and
calling `_validate_beamforming(bf)`, add:

```python
    if bf is not None and bf.get("enabled") and _bf_capable is not None:
        if not _bf_capable(cfg):
            raise SchemaError(
                "beamforming requires a card with a bf_monitor_conf node "
                "(GS driver lacks CONFIG_BEAMFORMING_MONITOR)")
```

- [ ] **Step 4: Wire the probe in the supervisor**

In `gs/fpvdgs/supervisor.py` `build_app`, after `beamforming = BeamformingController()`
and `resolve_wlans` are available, register the probe:

```python
    def _bf_capable(cfg):
        wlans = resolve_wlans(cfg)
        primary = wlans[0] if wlans else None
        return bool(primary and beamforming.supported(primary))
    schema.set_bf_capable(_bf_capable)
```

- [ ] **Step 5: Run, verify pass**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -v`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/schema.py gs/fpvdgs/supervisor.py gs/tests/unit/test_schema.py
git commit -m "gs/schema: fail-fast reject enabling BF on a card without bf_monitor_conf"
```

### Task B5: supervisor — delete `LinkCoordinator`, wire new Api + armer

**Files:**
- Modify: `gs/fpvdgs/supervisor.py` (Api construction, App wiring)
- Delete: `gs/fpvdgs/link.py`
- Delete: `gs/tests/unit/test_link.py`
- Test: `gs/tests/unit/test_app_wiring.py` (adjust)

- [ ] **Step 1: Delete the LinkCoordinator module + its tests**

```bash
git rm gs/fpvdgs/link.py gs/tests/unit/test_link.py
```

- [ ] **Step 2: Update supervisor wiring**

In `gs/fpvdgs/supervisor.py`:
- Remove `from .link import LinkCoordinator` (line 28).
- Remove the `link = LinkCoordinator(...)` block (lines 115-119).
- In `status_fn`, replace `link.in_sync()` usage (lines 163-164) — drop `inSync`
  from the GS probe block (it was drone-coordination state; client owns that now):

```python
        probe = {"reachable": reachable, "linkId": eff_link.get("linkId")}
```

- Change the `Api(...)` construction (lines 173-175) to inject the new
  collaborators and drop `link`:

```python
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone, status_fn=status_fn, cfg_out=cfg_out,
              dynlink=dynlink, pixelpilot=pixelpilot, probe=probe_ctrl,
              retune=lambda lnk: radio.retune(resolve_wlans(store.effective()), lnk),
              wlans_resolver=resolve_wlans,
              armer_tick=lambda: armer._tick())
```

(`armer` is the `BeamformingArmer` already built at line 105.)

- [ ] **Step 3: Run the app-wiring + full GS suite**

Run: `cd gs && python -m pytest tests -q`
Expected: PASS. Fix any `test_app_wiring.py` references to `link`/`LinkCoordinator`.

- [ ] **Step 4: Commit**

```bash
git add gs/fpvdgs/supervisor.py gs/tests/unit/test_app_wiring.py
git commit -m "gs/supervisor: delete LinkCoordinator; inject retune+armer_tick into Api"
```

---

## Part C — IDR forwarder decouple

### Task C1: `idr_relay.py` module + `drone_host_from_endpoint`

**Files:**
- Create: `gs/fpvdgs/idr_relay.py`
- Test: `gs/tests/unit/test_idr_relay.py`

- [ ] **Step 1: Write the relay lifecycle + host-helper tests**

Create `gs/tests/unit/test_idr_relay.py`:

```python
import socket
import time

from fpvdgs.idr_relay import IdrRelay, drone_host_from_endpoint


def test_drone_host_from_endpoint():
    assert drone_host_from_endpoint("http://10.5.0.10:8080") == "10.5.0.10"
    assert drone_host_from_endpoint("http://host:1/x") == "host"
    assert drone_host_from_endpoint("") == "10.5.0.10"          # default
    assert drone_host_from_endpoint(None) == "10.5.0.10"


def test_relay_forwards_datagrams():
    # drone-side receiver
    dst = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst.bind(("127.0.0.1", 0))
    dst_port = dst.getsockname()[1]
    relay = IdrRelay("127.0.0.1", port=0)
    relay._dest = ("127.0.0.1", dst_port)   # forward target
    relay.start()
    try:
        listen = relay.status()["listen"]
        assert listen
        host, port = listen.rsplit(":", 1)
        src = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        src.sendto(b"IDR", ("127.0.0.1", int(port)))
        dst.settimeout(2.0)
        data, _ = dst.recvfrom(16)
        assert data == b"IDR"
    finally:
        relay.stop()
        dst.close()


def test_relay_start_stop_status():
    relay = IdrRelay("127.0.0.1", port=0)
    relay.start()
    assert relay.status()["running"] is True
    relay.stop()
    assert relay.status()["running"] is False
```

- [ ] **Step 2: Run, verify fail**

Run: `cd gs && python -m pytest tests/unit/test_idr_relay.py -v`
Expected: FAIL — module does not exist.

- [ ] **Step 3: Create idr_relay.py**

Salvage the `IdrRelay` class verbatim from `feat/unified-config`
(`git show feat/unified-config:gs/fpvdgs/idr_relay.py`), and add the host helper
(it lived in `dynlink/config_build.py` on that branch — colocate it here):

```python
from urllib.parse import urlparse

def drone_host_from_endpoint(endpoint, default="10.5.0.10"):
    if not endpoint:
        return default
    return urlparse(endpoint).hostname or default
```

The salvaged `IdrRelay` already exposes `start()`, `stop()`, `status()` (keys
`running`, `listen`) and binds `0.0.0.0:<port>` forwarding to `self._dest`.

- [ ] **Step 4: Run, verify pass**

Run: `cd gs && python -m pytest tests/unit/test_idr_relay.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/idr_relay.py gs/tests/unit/test_idr_relay.py
git commit -m "gs/idr-relay: standalone always-available IDR relay module + host helper"
```

### Task C2: schema + defaults — `idrForward` block, remove dynamicLink IDR keys

**Files:**
- Modify: `gs/fpvdgs/schema.py` (`_validate_dynamic_link`, add `_validate_idr_forward`)
- Modify: `gs/etc/defaults.json`
- Test: `gs/tests/unit/test_schema.py`

- [ ] **Step 1: Tests for the new block + removed keys**

Add to `gs/tests/unit/test_schema.py`:

```python
def test_idr_forward_validates():
    schema.validate_effective({"link": {"channel": 1, "region": "US"},
                               "idrForward": {"enabled": True, "port": 11223}})
    with pytest.raises(schema.SchemaError):
        schema.validate_effective({"link": {"channel": 1, "region": "US"},
                                   "idrForward": {"enabled": True, "port": 0}})


def test_dynamiclink_idr_keys_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"dynamicLink": {"idrPort": 11223}})
```

For `test_dynamiclink_idr_keys_rejected` to work, `_validate_dynamic_link` must
reject unknown keys. If it doesn't currently enumerate allowed keys, instead
assert the weaker contract that `idrForward`/`idrPort` are simply ignored there
and not consumed — adjust the test to check the controller no longer reads them
(covered in C4). Keep whichever the existing schema style supports; do not invent
key-allowlisting if the module doesn't already do it.

- [ ] **Step 2: Run, verify fail**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -k idr -v`
Expected: FAIL

- [ ] **Step 3: Implement schema changes**

In `gs/fpvdgs/schema.py`:
- Remove the `idr_port` validation from `_validate_dynamic_link` (lines 83-85).
- Add `idrForward` validation, called from `validate_effective`:

```python
def _validate_idr_forward(idr: dict) -> None:
    if not isinstance(idr.get("enabled", True), bool):
        raise SchemaError("idrForward.enabled must be a bool")
    port = idr.get("port", 11223)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("idrForward.port must be an int in 1..65535")
```

In `validate_effective`, after the pixelpilot block:

```python
    idr = cfg.get("idrForward")
    if idr is not None:
        _validate_idr_forward(idr)
```

(`idrForward` is already in `CONFIG_TOP_KEYS` from Task B1.)

- [ ] **Step 4: Update GS defaults**

In `gs/etc/defaults.json`: remove `"idrForward": true` and `"idrPort": 11223`
from the `dynamicLink` block, and add a top-level block:

```json
  "idrForward": {
    "enabled": true,
    "port": 11223
  },
```

- [ ] **Step 5: Run, verify pass**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -v`
Expected: PASS

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/schema.py gs/etc/defaults.json gs/tests/unit/test_schema.py
git commit -m "gs/schema: add idrForward block; drop dynamicLink idr keys"
```

### Task C3: API — `_route_idr_forward` + apply integration

**Files:**
- Modify: `gs/fpvdgs/api.py` (`_route_idr_forward`)
- Test: `gs/tests/unit/test_api.py`

- [ ] **Step 1: Test the apply router for idrForward**

Add to `gs/tests/unit/test_api.py` (extend `_api` to inject a fake relay):

```python
class FakeRelay:
    def __init__(self): self.events = []; self._running = False
    def start(self): self._running = True; self.events.append("start")
    def stop(self): self._running = False; self.events.append("stop")
    def status(self): return {"running": self._running, "listen": None}


def test_idr_forward_apply_starts_and_stops():
    cfg_out = os.path.join(tempfile.mkdtemp(), "wifibroadcast.cfg")
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"},
                         "wfb": {"profile": "gs"}, "drone": {"endpoint": "http://x"},
                         "idrForward": {"enabled": False, "port": 11223}},
                        overlay_path=None)
    relay = FakeRelay()
    api = Api(store=store, schema=schema, render_mod=render_mod, runner=FakeRunner(),
              drone=FakeDrone(), status_fn=lambda: {}, cfg_out=cfg_out,
              retune=lambda l: True, wlans_resolver=lambda c: ["wlan0"],
              armer_tick=lambda: None, idr_relay=relay)
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"idrForward": {"enabled": True}}).encode())
    api.handle("POST", "/gs/apply", {}, b"")
    assert "start" in relay.events
    api.handle("PATCH", "/gs/config", {},
               json.dumps({"idrForward": {"enabled": False}}).encode())
    api.handle("POST", "/gs/apply", {}, b"")
    assert relay.events[-1] == "stop"
```

- [ ] **Step 2: Run, verify fail**

Run: `cd gs && python -m pytest tests/unit/test_api.py -k idr -v`
Expected: FAIL — `_route_idr_forward` is the stub from B2.

- [ ] **Step 3: Implement `_route_idr_forward`**

Replace the stub in `api.py`:

```python
    def _route_idr_forward(self, old, new, pending):
        """Start/stop the always-available IDR relay on idrForward changes.
        Independent of dynamicLink. Never bounces the wfb runner."""
        if self.idr_relay is None or old == new:
            return
        was = bool(old.get("enabled", True))
        now = bool(new.get("enabled", True))
        if now and not was:
            self.idr_relay.start()
        elif was and not now:
            self.idr_relay.stop()
        elif now and was and old.get("port") != new.get("port"):
            self.idr_relay.stop()
            self.idr_relay.start()
```

(Port changes require rebuilding the relay's dest/bind. If `IdrRelay` binds a
fixed `port` at construction, the supervisor rebuilds it on port change — for the
unit test, stop/start on the fake is sufficient; see C4 for the real wiring note.)

- [ ] **Step 4: Run, verify pass**

Run: `cd gs && python -m pytest tests/unit/test_api.py -v`
Expected: PASS

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/api.py gs/tests/unit/test_api.py
git commit -m "gs/api: route idrForward changes to the relay in /gs/apply"
```

### Task C4: controller — drop embedded `_IdrRelay`; supervisor wires `IdrRelay`

**Files:**
- Modify: `gs/fpvdgs/dynlink/controller.py` (remove `_IdrRelay` + idr code)
- Modify: `gs/fpvdgs/supervisor.py` (build + boot-start + App wiring)
- Test: `gs/tests/unit/test_dl_controller.py`, `gs/tests/integration/test_supervisor_e2e.py`

- [ ] **Step 1: Adjust controller tests that asserted idrListen / relay**

In `gs/tests/unit/test_dl_controller.py`, remove assertions on `idrListen` /
`idrForward` (the relay is no longer the controller's responsibility). The
controller's status block drops `idrListen`.

- [ ] **Step 2: Run, verify fail (or red where idr asserted)**

Run: `cd gs && python -m pytest tests/unit/test_dl_controller.py -v`
Expected: FAIL on any remaining idr assertions; otherwise proceed.

- [ ] **Step 3: Remove `_IdrRelay` from controller.py**

In `gs/fpvdgs/dynlink/controller.py`:
- Delete the `_IdrRelay` class (lines ~25-44).
- Delete the IDR-relay setup block in `_run` (lines ~143-164: `idr_transport`,
  the `snap.get("idrForward")` branch, the `self._set(idrListen=...)` calls).
- In the `finally` of `_run`, remove `idr_transport.close()` and `idrListen=None`.
- Remove `idrListen` from the initial `self._status` dict (line ~64).

- [ ] **Step 4: Wire `IdrRelay` into the supervisor + App**

In `gs/fpvdgs/supervisor.py`:
- Import: `from .idr_relay import IdrRelay, drone_host_from_endpoint`.
- Build it in `build_app` (after `drone` is created):

```python
    idr_cfg = effective.get("idrForward", {})
    endpoint = effective.get("drone", {}).get("endpoint", "http://10.5.0.10:8080")
    idr_relay = IdrRelay(drone_host_from_endpoint(endpoint),
                         port=int(idr_cfg.get("port", 11223)))
```

- Pass `idr_relay=idr_relay` to `Api(...)`.
- Add `idr_relay` to the `App` constructor + boot-start/shutdown (mirror
  pixelpilot). In `App.__init__` add `idr_relay=None`; in `App.start`:

```python
        if (self.idr_relay is not None
                and self.store.effective().get("idrForward", {}).get("enabled", True)):
            self.idr_relay.start()
```

  In `App.shutdown`, add `if self.idr_relay is not None: self.idr_relay.stop()`.
- Pass `idr_relay=idr_relay` into the `App(...)` return.

> Port-change note: `_route_idr_forward` stop/start reuses the same `IdrRelay`
> instance, which keeps its construction-time port. A `port` change therefore
> needs the relay rebuilt. Keep it simple: on a `port` change, `App`/supervisor
> need not hot-rebuild — document that `idrForward.port` changes take effect on
> daemon restart (the `enabled` toggle is the hot path). If hot port-change is
> required, have `_route_idr_forward` construct a fresh `IdrRelay`; not needed for
> the default deployment.

- [ ] **Step 5: Run controller + e2e suites**

Run: `cd gs && python -m pytest tests/unit/test_dl_controller.py tests/integration/test_supervisor_e2e.py -v`
Expected: PASS. Update `test_supervisor_e2e.py` for the `/gs` routes + relay
lifecycle if it asserted old paths.

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/dynlink/controller.py gs/fpvdgs/supervisor.py \
        gs/tests/unit/test_dl_controller.py gs/tests/integration/test_supervisor_e2e.py
git commit -m "gs: move IDR relay out of dynlink controller into App-owned IdrRelay"
```

### Task C5: full GS suite + drone suite green

**Files:** none (verification)

- [ ] **Step 1: GS suite**

Run: `cd gs && python -m pytest tests -q`
Expected: PASS. Grep for stragglers: `grep -rn '"/config"\|"/apply"\|"/link"\|txpower\b\|idrPort\|LinkCoordinator' gs/fpvdgs/`
and fix any non-`/air` GS-local route or old-key reference.

- [ ] **Step 2: Drone suite**

Run: `cd drone && cmake --build build && ./build/fpvd_tests`
Expected: PASS.

- [ ] **Step 3: Commit any straggler fixes**

```bash
git add -A && git commit -m "cleanup: remove stragglers from API/txpower/idr restructure"
```

---

## Part D — Docs

### Task D1: API docs — new route map, txpower dBm, idrForward, client orchestration

**Files:**
- Modify: `gs/README.md` (and any `docs/api/*` describing `/config`, `/link`, `/air`)

- [ ] **Step 1: Update the route + config reference**

Document, with no placeholders:
- Route map: `/gs/{config,apply,reset,status,defaults}`, `/air/*` proxy,
  `/healthz` at root; `/link` and `/link/apply` removed.
- `link.txPowerDbm` (dBm, `-10..30`, null = driver default) on both daemons;
  note the `wfb_txpower`/`iw` edge converts ×100 to mBm.
- The top-level `idrForward: { enabled, port }` block; note it is independent of
  `dynamicLink.enabled`.
- **Client orchestration** for a shared link change (channel/width/linkId):
  PATCH `/air/config` + `/gs/config`, then POST `/air/apply` **then** `/gs/apply`
  (drone first on a channel move).
- **Beamforming enable sequence** (client-owned MAC handshake): read GS card MAC
  from `/gs/status` (`beamforming.localMac`) → PATCH `/air/config` with
  `link.beamforming = {enabled:true, remoteMac:<GS MAC>}` and `link.stbc=false` →
  POST `/air/apply` → PATCH `/gs/config` `link.beamforming.enabled=true` → POST
  `/gs/apply`. Disable: reverse `enabled` on both sides; restore drone `stbc=true`.

- [ ] **Step 2: Commit**

```bash
git add gs/README.md docs/
git commit -m "docs/api: /gs+/air routes, txPowerDbm, idrForward, client orchestration"
```

---

## Self-Review Notes (for the implementer)

- **Spec coverage:** Section 1 (routes/link) → B1, B2, B5, D1. Section 1
  GS-local apply → B2. Section 1 beamforming → B3, B4. Section 2 (txpower) →
  A1–A5. Section 3 (IDR) → C1–C4. Section 4 (testing) → embedded per task.
- **Type/name consistency:** the static key is `txPowerDbm` everywhere (GS:
  schema/render/radio/defaults; drone: schema/validate/diff/lock/radio.cpp/
  scripts/defaults); env var is `FPVD_TXPOWER_DBM`; the new block is `idrForward`
  with keys `enabled`/`port`; `Api` gains `retune`, `wlans_resolver`,
  `armer_tick`, `idr_relay` and loses `link`.
- **Ordering:** A → B → C → D. Within B, B2 stubs `_route_idr_forward`; C3
  implements it. B5 deletes `link.py` only after B2 removed the `Api` dependency.
