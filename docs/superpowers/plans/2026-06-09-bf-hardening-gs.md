# BF Hardening — GS Plan (#3 + #4) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make GS `/link/apply` enable beamforming reliably: auto-manage the drone's `link.stbc`, surface drone validation rejections as real errors (not "unreachable"), and tolerate the flaky air link via a longer timeout.

**Architecture:** Two focused changes. (1) `DroneClient` distinguishes a 4xx *rejection* (`DroneRejected`) from a connectivity failure (`DroneUnreachable`) and uses a 10 s timeout. (2) `LinkCoordinator` bundles `link.stbc` into the drone push opposite to `beamforming.enabled` (false→enable, true→disable) and reports a `DroneRejected` as `res["droneError"]` without flipping `droneReachable`.

**Tech Stack:** Python 3.13 stdlib, pytest.

**Spec:** `docs/superpowers/specs/2026-06-09-bf-hardening-design.md` (#3, #4)

**Branch:** `feat/bf-hardening` (already checked out).

**Run tests from `gs/`:** `cd gs && .venv/bin/python -m pytest tests/unit/<file> -v`

---

## File Structure

- **Modify** `gs/fpvdgs/drone_client.py` — add `DroneRejected`; split `_ok_json` (4xx→reject, ≥500→unreachable); default timeout 4.0→10.0.
- **Modify** `gs/fpvdgs/link.py` — push `stbc` with beamforming; catch `DroneRejected` → `res["droneError"]`.
- **Test** `gs/tests/unit/test_drone_client.py`, `gs/tests/unit/test_link.py`.

---

### Task 1: DroneClient — `DroneRejected`, 4xx/5xx split, 10s timeout

**Files:**
- Modify: `gs/fpvdgs/drone_client.py`
- Test: `gs/tests/unit/test_drone_client.py`

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_drone_client.py`:

```python
def test_default_timeout_is_10s():
    c = DroneClient("http://example.invalid")
    assert c.timeout == 10.0


def test_4xx_raises_drone_rejected_with_code_and_body(fake_drone):
    from fpvdgs.drone_client import DroneRejected
    fake_drone["reject"] = (400, {"error": "validation",
                                  "message": "requires link.stbc=false",
                                  "details": [{"path": "link.beamforming"}]})
    c = DroneClient(fake_drone["endpoint"])
    with pytest.raises(DroneRejected) as ei:
        c.patch_config({"link": {"beamforming": {"enabled": True}}})
    assert ei.value.code == 400
    assert "stbc" in ei.value.message
    assert ei.value.body["details"][0]["path"] == "link.beamforming"


def test_5xx_still_raises_unreachable(fake_drone):
    fake_drone["fail"] = True   # fixture returns 500 on apply
    c = DroneClient(fake_drone["endpoint"])
    with pytest.raises(DroneUnreachable):
        c.apply()
```

- [ ] **Step 2: Add `reject` support to the fake_drone fixture**

The new 4xx test needs the fixture to return a chosen 4xx on PATCH. In `gs/tests/unit/conftest.py`, find the `fake_drone` fixture's request handler. It currently has a `do_PATCH` (or unified handler) that returns 200/500. Add a `reject` hook so a `(code, body)` tuple forces that response on PATCH. Locate the PATCH branch (it sets `state["config"]` and returns 200) and wrap it:

```python
        def do_PATCH(self):
            if state.get("reject") is not None:
                code, body = state["reject"]
                self._send(code, body)
                return
            # ... existing PATCH handling unchanged (record call, merge config, send 200) ...
```

Read the actual fixture first (`gs/tests/unit/conftest.py`, the `fake_drone` fixture starting ~line 23) and insert the `reject` short-circuit at the top of the PATCH handler, matching its real structure (method name, `_send` signature `_send(code, obj)`, and how it records `state["calls"]`). Initialize `"reject": None` in the `state` dict literal alongside `"fail": False`.

- [ ] **Step 3: Run tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_drone_client.py -v`
Expected: FAIL — `test_default_timeout_is_10s` (timeout is 4.0), `test_4xx_raises_drone_rejected...` (`ImportError`/raises `DroneUnreachable` instead).

- [ ] **Step 4: Implement in `gs/fpvdgs/drone_client.py`**

Add the exception after `DroneUnreachable` (lines 8-9):

```python
class DroneRejected(Exception):
    """The drone returned a 4xx — a validation/permission rejection, NOT a
    connectivity failure. Carries the status code and parsed error body."""
    def __init__(self, code: int, body):
        self.code = code
        self.body = body
        self.message = (body.get("message") if isinstance(body, dict) else None) \
            or f"drone rejected ({code})"
        super().__init__(f"{code}: {self.message}")
```

Change the timeout default (line 13):

```python
    def __init__(self, endpoint: str, timeout: float = 10.0):
```

Replace `_ok_json` (lines 30-34) with the 4xx/5xx split:

```python
    def _ok_json(self, method: str, path: str, body: dict | None = None) -> dict:
        code, raw = self._request(method, path, body)
        if 400 <= code < 500:
            try:
                parsed = json.loads(raw or b"{}")
            except ValueError:
                parsed = {"raw": raw.decode("utf-8", "replace")}
            raise DroneRejected(code, parsed)
        if code >= 500:
            raise DroneUnreachable(f"drone {method} {path} -> {code}")
        return json.loads(raw or b"{}")
```

- [ ] **Step 5: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_drone_client.py -v`
Expected: PASS (all, incl. the 3 new; existing `test_apply_raises_on_drone_error` still passes because `fail`=500→`DroneUnreachable`).

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/drone_client.py gs/tests/unit/test_drone_client.py gs/tests/unit/conftest.py
git commit -m "feat(gs/drone_client): DroneRejected for 4xx, 10s timeout"
```

---

### Task 2: Coordinator — push `stbc` with BF, surface `DroneRejected`

**Files:**
- Modify: `gs/fpvdgs/link.py`
- Test: `gs/tests/unit/test_link.py`

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_link.py` (reuses the existing `FakeRunner`, `FakeBf`, `BfDrone`, `_bf_store`, `_bf_coord` helpers defined earlier in the file):

```python
def test_bf_enable_pushes_stbc_false():
    store = _bf_store()
    store.patch({"link": {"beamforming": {"enabled": True}}})
    runner, drone, bf = FakeRunner(), BfDrone(), FakeBf()
    _bf_coord(store, runner, drone, bf).apply_link("both")
    assert drone.patched["link"]["stbc"] is False
    assert drone.patched["link"]["beamforming"]["enabled"] is True


def test_bf_disable_pushes_stbc_true():
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US",
                                  "beamforming": {"enabled": True}}})
    store.patch({"link": {"beamforming": {"enabled": False}}})
    runner, drone, bf = FakeRunner(), BfDrone(), FakeBf()
    _bf_coord(store, runner, drone, bf).apply_link("both")
    assert drone.patched["link"]["stbc"] is True
    assert drone.patched["link"]["beamforming"]["enabled"] is False


def test_drone_rejection_surfaced_not_unreachable():
    from fpvdgs.drone_client import DroneRejected

    class RejectingDrone(BfDrone):
        def patch_config(self, sparse):
            raise DroneRejected(400, {"message": "requires link.stbc=false",
                                      "details": [{"path": "link.beamforming"}]})

    store = _bf_store()
    store.patch({"link": {"beamforming": {"enabled": True}}})
    runner, drone, bf = FakeRunner(), RejectingDrone(), FakeBf()
    res = _bf_coord(store, runner, drone, bf).apply_link("both")
    # A validation rejection is a real error, NOT "drone unreachable".
    assert res["droneReachable"] is True
    assert res["droneApplied"] is False
    assert res["droneError"]["code"] == 400
    assert "stbc" in res["droneError"]["message"]
    # GS still applies locally (best-effort).
    assert res["gsApplied"] is True
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_link.py -k "stbc or rejection" -v`
Expected: FAIL — no `stbc` in the push; `DroneRejected` propagates uncaught (becomes a 500/raises) so `droneError` is absent.

- [ ] **Step 3: Implement in `gs/fpvdgs/link.py`**

(a) Extend the import (line 8):

```python
from .drone_client import DroneUnreachable, DroneRejected
```

(b) In `apply_link`, add `drone_error = None` next to the other init (the line `drone_mac = ""`, ~line 101):

```python
        drone_applied = False
        drone_reachable = False
        drone_mac = ""
        drone_error = None
```

(c) In the beamforming-push block (~lines 110-113), add the `stbc` flip:

```python
                if bf_changed and self.beamforming is not None:
                    gs_mac = self.beamforming.local_mac(primary) if primary else ""
                    push["beamforming"] = {"enabled": bf_enabled,
                                           "remoteMac": gs_mac}
                    # STBC and TX beamforming are mutually exclusive on the drone
                    # (it rejects beamforming while stbc=true). Flip stbc to match:
                    # false to enable BF, true to restore on disable.
                    push["stbc"] = not bf_enabled
```

(d) Add a `DroneRejected` catch to the existing try/except (~lines 127-128), BEFORE the `DroneUnreachable` catch:

```python
                except DroneRejected as e:
                    # Validation rejection — a real error, NOT a connectivity
                    # failure. Keep drone_reachable True; surface the error.
                    drone_error = {"code": e.code, "message": e.message,
                                   "details": e.body.get("details")
                                              if isinstance(e.body, dict) else None}
                except DroneUnreachable:
                    drone_reachable = False
```

(e) In the result dict (~after line 173, where `res["beamforming"]` is set), add:

```python
        if drone_error is not None:
            res["droneError"] = drone_error
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_link.py -v`
Expected: PASS — all existing link tests AND the 3 new ones.

- [ ] **Step 5: Run the full GS suite (no regressions)**

Run: `cd gs && .venv/bin/python -m pytest tests/unit -q`
Expected: all pass (1 pre-existing skip OK).

- [ ] **Step 6: Commit**

```bash
git add gs/fpvdgs/link.py gs/tests/unit/test_link.py
git commit -m "feat(gs/link): auto-manage stbc with BF; surface DroneRejected as droneError"
```

---

## Notes for the drone push payload

After this plan, enabling BF pushes `{"link": {"beamforming": {...}, "stbc": false}}`
to the drone. `stbc` is NOT a `DRONE_PUSH_KEYS` member and is only added when
`bf_changed`, so non-BF applies are unaffected. The drone's dynamic-link lock does
not cover `stbc`/`beamforming` (`drone/src/config/lock.cpp`), so the push is not
rejected by the lock.

## Self-Review

**Spec coverage (#3, #4):**
- #4 timeout 4→10 → Task 1 ✓
- #3b `DroneRejected` (4xx vs 5xx/unreachable) → Task 1 ✓
- #3a auto-manage stbc (false enable / true disable) → Task 2 (c) ✓
- #3b surface as `res["droneError"]`, not `droneReachable=false` → Task 2 (d)(e) ✓
- GS still applies locally on rejection → asserted in Task 2 test ✓

**Type consistency:** `DroneRejected(code, body)` with `.code/.body/.message` used identically in Tasks 1 & 2. `res["droneError"] = {"code","message","details"}`.

**Placeholder scan:** none — all steps have concrete code. Step 2 of Task 1 requires reading the real `conftest.py` fixture to match its handler shape (the only non-verbatim step, by necessity — the fixture's internal structure isn't quoted here; the engineer matches `_send`/`state` usage shown).
