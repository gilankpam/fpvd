# GS Beamforming (Downlink) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a GS beamformee responder and wire `link.beamforming` through the `/link/apply` coordinator so a single apply arms the GS and pushes the cross-referenced config to the drone — making the drone's existing downlink beamforming produce real array gain.

**Architecture:** A new config-only `BeamformingController` (no sounding loop — the rtl88x2eu MAC hardware auto-echoes the compressed beamforming report within SIFS once `bf_monitor_conf` is armed). The `LinkCoordinator` resolves both MACs at apply time, hard-rejects when the GS card lacks the `bf_monitor_conf` proc node, pushes `{enabled, remoteMac: gs_mac}` to the drone, and reconciles the GS controller orthogonally to the RF retune/bounce path so toggling BF never disrupts video.

**Tech Stack:** Python 3.13 stdlib, pytest. Mirrors the existing drone-side `drone/src/supervise/beamforming.cpp`.

**Spec:** `docs/superpowers/specs/2026-06-09-gs-beamforming-downlink-design.md`

**Branch:** `feat/gs-beamforming-downlink` (already checked out, based off `main`).

**Run tests from `gs/`:** `cd gs && python -m pytest tests/unit/<file> -v`

---

## File Structure

- **Create** `gs/fpvdgs/beamforming.py` — GS beamformee controller (`BeamformingController`) + `read_mac` helper. One responsibility: arm/disarm/report the monitor-BF proc node.
- **Modify** `gs/fpvdgs/schema.py` — validate the `link.beamforming` shape in `validate_effective`.
- **Modify** `gs/fpvdgs/link.py` — `LinkCoordinator`: capability hard-reject, MAC exchange, transformed drone push, orthogonal BF reconcile, result block.
- **Modify** `gs/fpvdgs/status.py` — `build_status` gains an optional `beamforming` section.
- **Modify** `gs/fpvdgs/supervisor.py` — construct the controller and wire it into the coordinator and `/status`.
- **Create** `gs/tests/unit/test_beamforming.py`; **extend** `test_link.py`, `test_schema.py`, `test_status.py`.

---

### Task 1: GS beamformee controller

**Files:**
- Create: `gs/fpvdgs/beamforming.py`
- Test: `gs/tests/unit/test_beamforming.py`

- [ ] **Step 1: Write the failing tests**

Create `gs/tests/unit/test_beamforming.py`:

```python
from fpvdgs.beamforming import BeamformingController, read_mac


def _node(tmp_path, iface):
    """Create a fake bf_monitor_conf proc node; return (proc_base, conf_path)."""
    proc = tmp_path / "proc"
    (proc / iface).mkdir(parents=True)
    conf = proc / iface / "bf_monitor_conf"
    conf.write_text("")
    return str(proc), conf


def _sys(tmp_path, iface, mac):
    sysd = tmp_path / "sys"
    (sysd / iface).mkdir(parents=True)
    (sysd / iface / "address").write_text(mac + "\n")
    return str(sysd)


def test_supported_true_when_node_present(tmp_path):
    proc, _ = _node(tmp_path, "wlan0")
    bf = BeamformingController(proc_base=proc)
    assert bf.supported("wlan0") is True


def test_supported_false_when_node_absent(tmp_path):
    bf = BeamformingController(proc_base=str(tmp_path / "proc"))
    assert bf.supported("wlan0") is False


def test_read_mac(tmp_path):
    sysd = _sys(tmp_path, "wlan0", "84:fc:14:6c:36:e6")
    assert read_mac("wlan0", sys_base=sysd) == "84:fc:14:6c:36:e6"


def test_enable_writes_conf_and_reports_active(tmp_path):
    proc, conf = _node(tmp_path, "wlan0")
    sysd = _sys(tmp_path, "wlan0", "84:fc:14:6c:36:e6")
    bf = BeamformingController(proc_base=proc, sys_base=sysd)
    st = bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    assert conf.read_text() == "1 00:c0:ca:dd:ee:ff 0 0"
    assert st["state"] == "active"
    assert st["requested"] is True
    assert st["peerMac"] == "00:c0:ca:dd:ee:ff"
    assert st["localMac"] == "84:fc:14:6c:36:e6"


def test_disable_writes_reset_and_reports_disabled(tmp_path):
    proc, conf = _node(tmp_path, "wlan0")
    bf = BeamformingController(proc_base=proc)
    bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    st = bf.reconcile(False, "wlan0", "")
    assert conf.read_text() == "0 00:00:00:00:00:00 0 0"
    assert st["state"] == "disabled"
    assert st["requested"] is False


def test_unsupported_when_no_node(tmp_path):
    bf = BeamformingController(proc_base=str(tmp_path / "proc"))
    st = bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    assert st["state"] == "unsupported"
    assert "bf_monitor_conf" in st["reason"]


def test_idempotent_no_rewrite(tmp_path):
    proc, conf = _node(tmp_path, "wlan0")
    bf = BeamformingController(proc_base=proc)
    bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    conf.write_text("SENTINEL")          # prove a second reconcile does NOT rewrite
    bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    assert conf.read_text() == "SENTINEL"


def test_write_failure_reports_error(tmp_path):
    # Node directory exists for supported(), but the conf path is a directory so
    # open(..., "w") raises -> state=error.
    proc = tmp_path / "proc"
    (proc / "wlan0" / "bf_monitor_conf").mkdir(parents=True)
    bf = BeamformingController(proc_base=str(proc))
    st = bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    assert st["state"] == "error"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd gs && python -m pytest tests/unit/test_beamforming.py -v`
Expected: FAIL with `ModuleNotFoundError: No module named 'fpvdgs.beamforming'`

- [ ] **Step 3: Write the implementation**

Create `gs/fpvdgs/beamforming.py`:

```python
"""GS beamformee responder.

Arms the rtl88x2eu monitor-BF hardware (`CONFIG_BEAMFORMING_MONITOR`) to
auto-echo the drone's downlink sounding. Config-only: once `bf_monitor_conf`
is armed with the drone's MAC, the WLAN-MAC hardware assembles and transmits
the VHT compressed beamforming report within SIFS, so there is NO sounding
loop here (unlike the drone-side beamformer). Mirrors the shape of
`drone/src/supervise/beamforming.cpp`, beamformee half only.
"""

import os

PROC_BASE = "/proc/net/rtl88x2eu"
SYS_BASE = "/sys/class/net"


def read_mac(iface: str, sys_base: str = SYS_BASE) -> str:
    try:
        with open(f"{sys_base}/{iface}/address") as f:
            return f.read().strip()
    except OSError:
        return ""


class BeamformingController:
    def __init__(self, proc_base: str = PROC_BASE, sys_base: str = SYS_BASE):
        self._proc_base = proc_base
        self._sys_base = sys_base
        self._armed = False
        self._iface = ""
        self._peer = ""
        self._state = "disabled"   # disabled | unsupported | active | error
        self._reason = ""

    def supported(self, iface: str) -> bool:
        return os.path.exists(f"{self._proc_base}/{iface}/bf_monitor_conf")

    def local_mac(self, iface: str) -> str:
        return read_mac(iface, self._sys_base)

    def _write_conf(self, iface: str, content: str) -> bool:
        try:
            with open(f"{self._proc_base}/{iface}/bf_monitor_conf", "w") as f:
                f.write(content)
            return True
        except OSError:
            return False

    def reconcile(self, enabled: bool, iface: str, peer_mac: str) -> dict:
        if not enabled:
            if self._armed and self.supported(self._iface):
                self._write_conf(self._iface, "0 00:00:00:00:00:00 0 0")
            self._armed, self._iface, self._peer = False, iface, ""
            self._state, self._reason = "disabled", ""
            return self.status()

        if not self.supported(iface):
            self._armed, self._iface, self._peer = False, iface, peer_mac
            self._state = "unsupported"
            self._reason = f"no bf_monitor_conf node on {iface}"
            return self.status()

        if self._armed and self._iface == iface and self._peer == peer_mac:
            return self.status()   # idempotent: no rewrite

        if self._write_conf(iface, f"1 {peer_mac} 0 0"):
            self._armed, self._iface, self._peer = True, iface, peer_mac
            self._state, self._reason = "active", ""
        else:
            self._armed, self._iface, self._peer = False, iface, peer_mac
            self._state, self._reason = "error", "bf_monitor_conf write failed"
        return self.status()

    def status(self) -> dict:
        return {
            "requested": self._state != "disabled",
            "state": self._state,
            "reason": self._reason,
            "iface": self._iface,
            "localMac": self.local_mac(self._iface) if self._iface else "",
            "peerMac": self._peer,
        }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && python -m pytest tests/unit/test_beamforming.py -v`
Expected: PASS (8 passed)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/beamforming.py gs/tests/unit/test_beamforming.py
git commit -m "feat(gs/beamforming): config-only beamformee controller"
```

---

### Task 2: Schema validation for `link.beamforming`

**Files:**
- Modify: `gs/fpvdgs/schema.py`
- Test: `gs/tests/unit/test_schema.py`

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_schema.py`:

```python
def test_beamforming_enabled_bool_ok():
    from fpvdgs import schema
    cfg = {"link": {"region": "US", "channel": 132,
                    "beamforming": {"enabled": True}}}
    schema.validate_effective(cfg)   # must not raise


def test_beamforming_enabled_must_be_bool():
    from fpvdgs import schema
    cfg = {"link": {"region": "US", "channel": 132,
                    "beamforming": {"enabled": "yes"}}}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)


def test_beamforming_rejects_unknown_subkey():
    from fpvdgs import schema
    cfg = {"link": {"region": "US", "channel": 132,
                    "beamforming": {"enabled": True, "remoteMac": "x"}}}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)
```

(Note: `test_schema.py` already imports `pytest` at the top.)

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -k beamforming -v`
Expected: FAIL — `test_beamforming_enabled_must_be_bool` and `test_beamforming_rejects_unknown_subkey` do not raise (no validation yet)

- [ ] **Step 3: Write the implementation**

In `gs/fpvdgs/schema.py`, in `validate_effective`, add the beamforming check right after the existing `channel` check (after line 47 `raise SchemaError("link.channel is required")`):

```python
    bf = link.get("beamforming")
    if bf is not None:
        _validate_beamforming(bf)
```

Then add this new function (place it next to `_validate_dynamic_link`):

```python
def _validate_beamforming(bf: dict) -> None:
    if not isinstance(bf, dict):
        raise SchemaError("link.beamforming must be an object")
    unknown = set(bf) - {"enabled"}
    if unknown:
        raise SchemaError(f"unknown link.beamforming keys: {sorted(unknown)}")
    if not isinstance(bf.get("enabled", False), bool):
        raise SchemaError("link.beamforming.enabled must be a bool")
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && python -m pytest tests/unit/test_schema.py -v`
Expected: PASS (all, including the 3 new beamforming cases)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/schema.py gs/tests/unit/test_schema.py
git commit -m "feat(gs/schema): validate link.beamforming shape"
```

---

### Task 3: Coordinator — hard-reject, MAC exchange, push, orthogonal reconcile

**Files:**
- Modify: `gs/fpvdgs/link.py`
- Test: `gs/tests/unit/test_link.py`

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_link.py`:

```python
class FakeBf:
    """Stub beamformee controller for coordinator tests."""
    def __init__(self, supported=True, gs_mac="84:fc:14:6c:36:e6"):
        self._supported = supported
        self._gs_mac = gs_mac
        self.calls = []          # records (enabled, iface, peer)
        self._armed = False

    def supported(self, iface):
        return self._supported

    def local_mac(self, iface):
        return self._gs_mac

    def reconcile(self, enabled, iface, peer):
        self.calls.append((enabled, iface, peer))
        self._armed = bool(enabled)
        return {"state": "active" if enabled else "disabled",
                "iface": iface, "peerMac": peer, "localMac": self._gs_mac,
                "requested": bool(enabled), "reason": ""}

    def status(self):
        return {"state": "active" if self._armed else "disabled"}


class BfDrone(FakeDrone):
    """FakeDrone that also answers GET /status with a drone card MAC."""
    def __init__(self, reachable=True, drone_mac="00:c0:ca:dd:ee:ff"):
        super().__init__(reachable=reachable)
        self._drone_mac = drone_mac

    def get_status(self):
        return {"beamforming": {"localMac": self._drone_mac}}


def _bf_store():
    return ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"}})


def _bf_coord(store, runner, drone, bf, primary="wlan0"):
    return LinkCoordinator(store, lambda cfg: None, runner, drone,
                           beamforming=bf, wlans_resolver=lambda cfg: [primary])


def test_bf_enable_hard_rejects_when_unsupported():
    store = _bf_store()
    store.patch({"link": {"beamforming": {"enabled": True}}})
    coord = _bf_coord(store, FakeRunner(), BfDrone(), FakeBf(supported=False))
    with pytest.raises(schema.SchemaError):
        coord.apply_link("both")
    assert store.effective()["link"].get("beamforming") in (None, {})  # not committed


def test_bf_enable_pushes_transformed_mac_and_arms_gs():
    store = _bf_store()
    store.patch({"link": {"beamforming": {"enabled": True}}})
    runner, drone, bf = FakeRunner(), BfDrone(drone_mac="00:c0:ca:dd:ee:ff"), FakeBf(gs_mac="84:fc:14:6c:36:e6")
    res = _bf_coord(store, runner, drone, bf).apply_link("both")
    # Drone receives the GS MAC as its remoteMac (transformed, not echoed).
    assert drone.patched == {"link": {"beamforming": {"enabled": True,
                                                      "remoteMac": "84:fc:14:6c:36:e6"}}}
    assert drone.applied is True
    # GS armed to respond to the drone's MAC.
    assert bf.calls == [(True, "wlan0", "00:c0:ca:dd:ee:ff")]
    # BF-only change must NOT bounce the pipeline.
    assert runner.restarts == 0
    assert res["mode"] == "none"
    assert res["beamforming"]["state"] == "active"
    assert store.effective()["link"]["beamforming"]["enabled"] is True


def test_bf_only_change_does_not_bounce_or_retune():
    store = _bf_store()
    store.patch({"link": {"beamforming": {"enabled": True}}})
    runner, retune = FakeRunner(), FakeRetune(ok=True)
    coord = LinkCoordinator(store, lambda cfg: None, runner, BfDrone(),
                            retune=retune, beamforming=FakeBf(),
                            wlans_resolver=lambda cfg: ["wlan0"])
    coord.apply_link("both")
    assert runner.restarts == 0
    assert retune.calls == []          # no RF action for a BF-only change


def test_bf_enable_drone_unreachable_reports_pending_still_applies_gs():
    store = _bf_store()
    store.patch({"link": {"beamforming": {"enabled": True}}})
    runner, bf = FakeRunner(), FakeBf()
    res = _bf_coord(store, runner, BfDrone(reachable=False), bf).apply_link("both")
    assert bf.calls == []                       # can't arm without the drone MAC
    assert res["beamforming"]["state"] == "pending"
    assert res["droneApplied"] is False
    assert store.effective()["link"]["beamforming"]["enabled"] is True  # intent persists


def test_bf_disable_resets_gs():
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US",
                                  "beamforming": {"enabled": True}}})
    store.patch({"link": {"beamforming": {"enabled": False}}})
    runner, bf = FakeRunner(), FakeBf()
    res = _bf_coord(store, runner, BfDrone(), bf).apply_link("both")
    assert bf.calls == [(False, "wlan0", "")]
    assert res["beamforming"]["state"] == "disabled"
    assert runner.restarts == 0


def test_channel_plus_bf_change_retunes_live_without_bf_bounce():
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"}})
    store.patch({"link": {"channel": 100, "beamforming": {"enabled": True}}})
    runner, retune, bf = FakeRunner(), FakeRetune(ok=True), FakeBf()
    coord = LinkCoordinator(store, lambda cfg: None, runner, BfDrone(),
                            retune=retune, beamforming=bf,
                            wlans_resolver=lambda cfg: ["wlan0"])
    res = coord.apply_link("both")
    assert res["mode"] == "live"        # channel still live-retunes; BF doesn't force a bounce
    assert retune.calls[0]["channel"] == 100
    assert runner.restarts == 0
    assert bf.calls == [(True, "wlan0", "00:c0:ca:dd:ee:ff")]
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd gs && python -m pytest tests/unit/test_link.py -k bf -v`
Expected: FAIL — `LinkCoordinator.__init__` got an unexpected keyword argument `beamforming`

- [ ] **Step 3: Write the implementation**

Replace the whole of `gs/fpvdgs/link.py` with:

```python
"""GS-local-first link coordinator.

A link change ALWAYS applies on the GS (it is how a link is established).
The drone push is best-effort, only for apply_to == "both" and only when the
drone is reachable — never a precondition.
"""

from .drone_client import DroneUnreachable
from .schema import SchemaError

# Only the truly-shared radio params go to the drone. GS-only keys
# (region, wlans, txpower) are per-side and never pushed. `beamforming` is
# pushed separately (with the MAC transformed) by apply_link.
DRONE_PUSH_KEYS = ("channel", "width", "linkId")


def _bw_class(width):
    """Radiotap bandwidth class: 10 and 20 MHz are wire-identical (BW_20);
    only 40 differs (BW_40). See wfb-ng src/tx.cpp."""
    return 40 if width == 40 else 20


class LinkCoordinator:
    def __init__(self, store, renderer_write, runner, drone, validate=None,
                 retune=None, beamforming=None, wlans_resolver=None):
        # renderer_write(effective_cfg: dict) -> None  renders + writes the cfg file
        # validate(effective_cfg: dict) -> None         raises on invalid values (optional)
        # retune(link: dict) -> bool                     live iw retune (optional; None = always bounce)
        # beamforming                                    GS beamformee controller (optional)
        # wlans_resolver(cfg: dict) -> list[str]         resolves the card list; [0] is the BF peer
        self.store = store
        self.renderer_write = renderer_write
        self.runner = runner
        self.drone = drone
        self.validate = validate
        self.retune = retune
        self.beamforming = beamforming
        self.wlans_resolver = wlans_resolver
        self._last_sync = None

    def in_sync(self):
        return self._last_sync

    def _can_retune_live(self, old, new):
        """A live iw retune is safe only when the change is limited to fields
        that `iw` can apply on a running monitor card (channel/width/txpower/
        region) AND the radiotap BW class is unchanged. `beamforming` is
        reconciled separately, so it is excluded here. Anything else (wlans,
        linkId, …) or a 40 MHz crossing falls back to a full runner bounce."""
        if self.retune is None:
            return False
        changed = {k for k in set(old) | set(new)
                   if k != "beamforming" and old.get(k) != new.get(k)}
        if not changed <= {"channel", "width", "txpower", "region"}:
            return False
        return _bw_class(old.get("width")) == _bw_class(new.get("width"))

    def _primary_iface(self, cfg):
        if self.beamforming is None or self.wlans_resolver is None:
            return None
        wlans = self.wlans_resolver(cfg)
        return wlans[0] if wlans else None

    def _reconcile_beamforming(self, enabled, primary, drone_mac):
        """Arm/disarm the GS beamformee. Orthogonal to retune/bounce. Returns
        the status block for the apply result, or None when BF is not wired."""
        if self.beamforming is None or primary is None:
            return None
        if enabled and not drone_mac:
            # Can't arm without the drone's MAC (drone unreachable / no status).
            st = dict(self.beamforming.status())
            st["state"] = "pending"
            st["reason"] = "drone unreachable; peer MAC unknown"
            return st
        return self.beamforming.reconcile(enabled, primary,
                                          drone_mac if enabled else "")

    def apply_link(self, apply_to: str = "both") -> dict:
        pending_cfg = self.store.pending()
        if self.validate is not None:
            self.validate(pending_cfg)   # raises (e.g. SchemaError) on bad values
        link = pending_cfg.get("link", {})
        last_good = self.store.effective()
        old_link = last_good.get("link", {})

        primary = self._primary_iface(pending_cfg)
        bf_new = link.get("beamforming") or {}
        bf_enabled = bool(bf_new.get("enabled"))
        bf_changed = bf_new != (old_link.get("beamforming") or {})

        # Capability hard-reject: enabling BF requires the bf_monitor_conf node
        # to exist on the primary card RIGHT NOW. Aborts before any commit/push.
        if bf_enabled and self.beamforming is not None:
            if primary is None or not self.beamforming.supported(primary):
                raise SchemaError(
                    f"beamforming unavailable on {primary}: no bf_monitor_conf "
                    f"node (GS driver lacks CONFIG_BEAMFORMING_MONITOR)")

        gs_mac = self.beamforming.local_mac(primary) if (
            self.beamforming is not None and primary) else ""

        drone_applied = False
        drone_reachable = False
        drone_mac = ""
        if apply_to == "both":
            drone_reachable = self.drone.healthz()
            if drone_reachable:
                # Push only the shared keys that actually CHANGED (see the
                # dynamic-link-locked-width regression). beamforming is pushed
                # separately, with the GS MAC as the drone's remoteMac.
                push = {k: link[k] for k in DRONE_PUSH_KEYS
                        if k in link and link[k] != old_link.get(k)}
                if bf_changed and self.beamforming is not None:
                    push["beamforming"] = {"enabled": bf_enabled,
                                           "remoteMac": gs_mac}
                try:
                    if bf_enabled and self.beamforming is not None:
                        drone_mac = (self.drone.get_status()
                                     .get("beamforming", {}).get("localMac", ""))
                    if push:
                        self.drone.patch_config({"link": push})
                        self.drone.apply()
                    drone_applied = True   # empty push => already in sync
                except DroneUnreachable:
                    drone_reachable = False

        # GS-side beamforming reconcile — orthogonal to the RF path below.
        bf_result = self._reconcile_beamforming(bf_enabled, primary, drone_mac)

        # The retune/bounce decision uses only the NON-beamforming link delta,
        # so toggling BF never retunes or bounces the running video pipeline.
        non_bf_changed = any(k != "beamforming" and old_link.get(k) != link.get(k)
                             for k in set(old_link) | set(link))

        self.renderer_write(pending_cfg)
        if not non_bf_changed:
            gs_applied = True
            mode = "none"
        else:
            live = self._can_retune_live(old_link, link)
            if live:
                gs_applied = self.retune(link)
                mode = "live"
                if not gs_applied:           # live retune failed → bounce
                    gs_applied = self.runner.restart()
                    mode = "bounce"
            else:
                gs_applied = self.runner.restart()
                mode = "bounce"

        if gs_applied:
            self.store.commit()
            self._last_sync = (apply_to == "both") and drone_applied
        else:
            self.renderer_write(last_good)
            self.runner.restart()

        res = {
            "gsApplied": bool(gs_applied),
            "droneApplied": drone_applied,
            "droneReachable": drone_reachable,
            "inSync": bool(gs_applied) and drone_applied,
            "mode": mode,
        }
        if bf_result is not None:
            res["beamforming"] = bf_result
        return res
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && python -m pytest tests/unit/test_link.py -v`
Expected: PASS — all existing link tests (unchanged behavior: BF not wired ⇒ no `beamforming` key, identical `mode` results) **and** the 6 new `bf` tests.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/link.py gs/tests/unit/test_link.py
git commit -m "feat(gs/link): wire beamforming through /link/apply (hard-reject, MAC exchange, orthogonal reconcile)"
```

---

### Task 4: Status — expose the beamforming block

**Files:**
- Modify: `gs/fpvdgs/status.py`
- Test: `gs/tests/unit/test_status.py`

- [ ] **Step 1: Write the failing tests**

Append to `gs/tests/unit/test_status.py`:

```python
def test_status_omits_beamforming_when_not_given():
    out = build_status("1.0", _runner_state(), {}, {"reachable": True})
    assert "beamforming" not in out


def test_status_includes_beamforming_block():
    bf = {"requested": True, "state": "active", "reason": "",
          "iface": "wlan0", "localMac": "84:fc:14:6c:36:e6",
          "peerMac": "00:c0:ca:dd:ee:ff"}
    out = build_status("1.0", _runner_state(), {}, {"reachable": True},
                       beamforming=bf)
    assert out["beamforming"]["state"] == "active"
    assert out["beamforming"]["peerMac"] == "00:c0:ca:dd:ee:ff"
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd gs && python -m pytest tests/unit/test_status.py -k beamforming -v`
Expected: FAIL — `build_status() got an unexpected keyword argument 'beamforming'`

- [ ] **Step 3: Write the implementation**

In `gs/fpvdgs/status.py`, update the `build_status` signature to add the param:

```python
def build_status(version: str, runner_state: dict, wlans: dict,
                 drone_probe: dict, link_stats: dict | None = None,
                 uptime_ms: int | None = None,
                 dynamic_link: dict | None = None,
                 pixelpilot: dict | None = None,
                 beamforming: dict | None = None) -> dict:
```

Then, just before `return out`, add:

```python
    if beamforming is not None:
        out["beamforming"] = beamforming
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && python -m pytest tests/unit/test_status.py -v`
Expected: PASS (all, including the 2 new beamforming cases)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/status.py gs/tests/unit/test_status.py
git commit -m "feat(gs/status): expose beamforming block in /status"
```

---

### Task 5: Supervisor wiring

**Files:**
- Modify: `gs/fpvdgs/supervisor.py`
- Test: `gs/tests/unit/test_app_wiring.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_app_wiring.py`:

```python
def test_link_coordinator_has_beamforming_wired(tmp_path, monkeypatch):
    """build_app must wire a BeamformingController + wlans_resolver into the
    coordinator so /link/apply can arm the GS beamformee."""
    import fpvdgs.supervisor as sup
    from fpvdgs.beamforming import BeamformingController

    monkeypatch.setattr(sup, "resolve_wlans", lambda cfg: ["wlan0"])
    monkeypatch.setattr(sup.render_mod, "write_cfg", lambda *a, **k: None)
    monkeypatch.setattr(sup.render_mod, "render_cfg", lambda eff: "")

    defaults = tmp_path / "defaults.json"
    defaults.write_text('{"link": {"region": "US", "channel": 132, "width": 20}}')
    overlay = tmp_path / "config.json"
    overlay.write_text("{}")

    app = sup.build_app(str(defaults), str(overlay), str(tmp_path / "out.cfg"),
                        "127.0.0.1", 0, runner_cmd=["true"])
    assert isinstance(app.api.link.beamforming, BeamformingController)
    assert app.api.link.wlans_resolver is not None
```

(If `test_app_wiring.py` does not exist, create it with `import fpvdgs.supervisor` at top. Check first: `ls gs/tests/unit/test_app_wiring.py`.)

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && python -m pytest tests/unit/test_app_wiring.py -k beamforming -v`
Expected: FAIL — `app.api.link.beamforming` is `None`

- [ ] **Step 3: Write the implementation**

In `gs/fpvdgs/supervisor.py`:

(a) Add the import after the existing `from .api import ...` line (line 9 area):

```python
from .beamforming import BeamformingController
```

(b) In `build_app`, construct the controller just before the `LinkCoordinator(...)` call (after the `pixelpilot = ProcessSupervisor(...)` block, near line 70):

```python
    beamforming = BeamformingController()
```

(c) Extend the `LinkCoordinator(...)` construction (lines 79-81) to pass it:

```python
    link = LinkCoordinator(store, renderer_write, runner, drone,
                           validate=schema.validate_effective,
                           retune=lambda lnk: radio.retune(wlans, lnk),
                           beamforming=beamforming,
                           wlans_resolver=resolve_wlans)
```

(d) In `status_fn` (the `build_status(...)` call near line 111), add the beamforming arg:

```python
        return status_mod.build_status(__version__, runner.state(), wlan_info, probe,
                                       uptime_ms=uptime_ms,
                                       dynamic_link=_dynamic_link_status(reachable),
                                       pixelpilot=_pixelpilot_status(),
                                       beamforming=beamforming.status())
```

- [ ] **Step 4: Run the full GS test suite**

Run: `cd gs && python -m pytest tests/unit -q`
Expected: PASS (entire suite green, including the new wiring test)

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/supervisor.py gs/tests/unit/test_app_wiring.py
git commit -m "feat(gs/supervisor): wire BeamformingController into coordinator + status"
```

---

## Out of scope / follow-ups

- **Boot-time re-arming:** the GS arms only on `/link/apply`. After a GS reboot
  with `beamforming.enabled` already in config, BF stays disabled until the next
  apply (the operator re-applies `/link`, or a future change arms at boot once
  the drone is reachable). The drone side already reconciles at boot.
- **GS driver build** with `CONFIG_BEAMFORMING_MONITOR` (operator prereq; the
  hard-reject correctly refuses to enable BF until the node exists).
- **Hardware verification gate** (from the spec): after the driver is deployed
  and BF enabled, confirm the drone's `/status` `beamforming` goes `active` and
  `bf_monitor_rfinfo` populates a non-zero report.

---

## Self-Review

**Spec coverage:**
- GS beamformee controller (config-only, no loop) → Task 1 ✓
- Schema `{enabled}` shape, no persisted MAC → Task 2 ✓
- Coordinator: GS-only hard-reject → Task 3 ✓
- Coordinator: auto MAC-exchange + transformed drone push → Task 3 ✓
- Coordinator: BF orthogonal to retune/bounce (no pipeline disruption) → Task 3 ✓
- Coordinator: drone-unreachable best-effort (link applies, BF reported) → Task 3 ✓
- Result `beamforming` block → Task 3 ✓
- `/status` beamforming block → Tasks 4 (build_status) + 5 (wiring) ✓
- Availability check = proc-node presence (not VHT bits) → `supported()` Task 1, used by hard-reject Task 3 ✓
- TDD throughout; hardware-verification gate noted as follow-up ✓

**Type consistency:** `reconcile(enabled, iface, peer_mac)`, `supported(iface)`,
`local_mac(iface)`, `status()` used identically across Tasks 1/3/5. Result keys
(`gsApplied`, `droneApplied`, `mode`, `beamforming`) consistent. `build_status(..., beamforming=)`
matches the supervisor call.

**Placeholder scan:** none — every step contains complete code and exact commands.
