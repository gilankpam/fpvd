import pytest

from fpvdgs import schema
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
    # Only the CHANGED shared key is pushed. Unchanged width is omitted so it
    # can't trip the drone's dynamic-link lock; region is GS-only and never sent.
    assert drone.patched == {"link": {"channel": 100}}
    assert drone.applied is True
    assert runner.restarts == 1
    assert store.effective()["link"]["channel"] == 100
    # no retune wired -> bounce path
    assert res == {"gsApplied": True, "droneApplied": True, "droneReachable": True,
                   "inSync": True, "mode": "bounce"}


def test_apply_both_omits_unchanged_locked_width():
    # Regression for the dynamic-link-locked channel bug. The drone refuses any
    # PATCH that *writes* link.width while DL is enabled — even to its current
    # value (the lock counts writes by structure, not by value-change). So a
    # channel-only apply must push ONLY {channel}; bundling the unchanged width
    # gets the whole patch (incl. the allowed channel) rejected and the drone
    # never retunes -> "GS changed, drone stays".
    from fpvdgs.drone_client import DroneUnreachable

    class DlLockedDrone:
        """Mirrors the drone's checkDynamicLinkLock: a body writing link.width is
        refused with dynamic_link_locked, surfaced to us as a 4xx -> Unreachable."""
        def __init__(self):
            self.patched = None
            self.applied = False

        def healthz(self):
            return True

        def patch_config(self, sparse):
            if "width" in sparse.get("link", {}):
                raise DroneUnreachable("drone PATCH /config -> 400 dynamic_link_locked")
            self.patched = sparse
            return {}

        def apply(self):
            self.applied = True
            return {}

    store = _store()                          # channel 132, width 40
    store.patch({"link": {"channel": 100}})   # ONLY channel changes
    runner, drone, written = FakeRunner(), DlLockedDrone(), []
    res = _coord(store, runner, drone, written).apply_link("both")

    assert drone.patched == {"link": {"channel": 100}}   # unchanged width NOT bundled
    assert drone.applied is True
    assert res["droneApplied"] is True
    assert res["inSync"] is True


def test_apply_both_pushes_width_when_width_actually_changes():
    # The delta push must still send width when it genuinely changes (DL off).
    store = _store()                          # width 40
    store.patch({"link": {"width": 20}})      # width changes
    runner, drone, written = FakeRunner(), FakeDrone(reachable=True), []
    res = _coord(store, runner, drone, written).apply_link("both")
    assert drone.patched == {"link": {"width": 20}}
    assert res["droneApplied"] is True


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


def test_apply_both_healthz_ok_but_patch_raises_still_applies_gs():
    from fpvdgs.drone_client import DroneUnreachable

    class FlakyDrone:
        def __init__(self):
            self.applied = False

        def healthz(self):
            return True  # says healthy...

        def patch_config(self, sparse):
            raise DroneUnreachable("dropped mid-push")  # ...but the push fails

        def apply(self):
            self.applied = True

    store = _store()
    store.patch({"link": {"channel": 100}})
    runner, drone, written = FakeRunner(), FlakyDrone(), []
    res = _coord(store, runner, drone, written).apply_link("both")
    # GS still applies despite the drone dropping mid-push
    assert runner.restarts == 1
    assert store.effective()["link"]["channel"] == 100
    assert drone.applied is False
    assert res["droneApplied"] is False
    assert res["droneReachable"] is False
    assert res["inSync"] is False


def test_apply_link_rollback_on_runner_failure():
    class FailingRunner:
        def restart(self):
            return False

    store = _store()
    store.patch({"link": {"channel": 100}})
    written = []
    coord = LinkCoordinator(store, lambda cfg: written.append(cfg),
                            FailingRunner(), FakeDrone(reachable=False))
    res = coord.apply_link("gs")
    assert res["gsApplied"] is False
    assert store.effective()["link"]["channel"] == 132    # not committed
    assert written[-1]["link"]["channel"] == 132           # rolled back to last-good


def test_apply_link_validates_bad_width():
    store = _store()
    store.patch({"link": {"width": 80}})
    coord = LinkCoordinator(store, lambda cfg: None, FakeRunner(),
                            FakeDrone(reachable=True), validate=schema.validate_effective)
    with pytest.raises(schema.SchemaError):
        coord.apply_link("both")


# --- live iw retune (no process restart) ------------------------------------

class FakeRetune:
    def __init__(self, ok=True):
        self.ok = ok
        self.calls = []          # records each link dict passed

    def __call__(self, link):
        self.calls.append(link)
        return self.ok


def test_channel_change_retunes_live_without_restart():
    store = _store()  # width 40
    store.patch({"link": {"channel": 100}})
    runner, drone, retune = FakeRunner(), FakeDrone(reachable=False), FakeRetune(ok=True)
    coord = LinkCoordinator(store, lambda cfg: None, runner, drone, retune=retune)
    res = coord.apply_link("gs")
    assert res["mode"] == "live"
    assert res["gsApplied"] is True
    assert len(retune.calls) == 1
    assert retune.calls[0]["channel"] == 100 and retune.calls[0]["width"] == 40
    assert runner.restarts == 0          # NO process restart
    assert store.effective()["link"]["channel"] == 100


def test_10_to_20_width_retunes_live():
    store = ConfigStore({"link": {"channel": 132, "width": 10, "region": "US"}})
    store.patch({"link": {"width": 20}})
    runner, drone, retune = FakeRunner(), FakeDrone(reachable=False), FakeRetune(ok=True)
    coord = LinkCoordinator(store, lambda cfg: None, runner, drone, retune=retune)
    res = coord.apply_link("gs")
    assert res["mode"] == "live"          # 10<->20 stays radiotap BW_20, no -B change
    assert retune.calls[0]["width"] == 20
    assert runner.restarts == 0


def test_txpower_change_retunes_live():
    store = _store()  # width 40, no txpower
    store.patch({"link": {"txpower": 2200}})
    runner, drone, retune = FakeRunner(), FakeDrone(reachable=False), FakeRetune(ok=True)
    coord = LinkCoordinator(store, lambda cfg: None, runner, drone, retune=retune)
    res = coord.apply_link("gs")
    assert res["mode"] == "live"          # txpower is pure iw, no restart
    assert retune.calls[0]["txpower"] == 2200
    assert runner.restarts == 0


def test_region_change_retunes_live():
    store = _store()  # region US
    store.patch({"link": {"region": "BO"}})
    runner, drone, retune = FakeRunner(), FakeDrone(reachable=False), FakeRetune(ok=True)
    coord = LinkCoordinator(store, lambda cfg: None, runner, drone, retune=retune)
    res = coord.apply_link("gs")
    assert res["mode"] == "live"          # region is pure iw (iw reg set), no restart
    assert retune.calls[0]["region"] == "BO"
    assert runner.restarts == 0


def test_20_to_40_width_bounces():
    store = ConfigStore({"link": {"channel": 132, "width": 20, "region": "US"}})
    store.patch({"link": {"width": 40}})
    runner, drone, retune = FakeRunner(), FakeDrone(reachable=False), FakeRetune(ok=True)
    coord = LinkCoordinator(store, lambda cfg: None, runner, drone, retune=retune)
    res = coord.apply_link("gs")
    assert res["mode"] == "bounce"        # crossing the 40 MHz BW class needs a -B change
    assert retune.calls == []
    assert runner.restarts == 1


def test_structural_change_bounces():
    store = _store()
    store.patch({"link": {"wlans": ["wlanX"]}})   # interface list change -> respawn
    runner, drone, retune = FakeRunner(), FakeDrone(reachable=False), FakeRetune(ok=True)
    coord = LinkCoordinator(store, lambda cfg: None, runner, drone, retune=retune)
    res = coord.apply_link("gs")
    assert res["mode"] == "bounce"        # not channel/width/txpower/region -> bounce
    assert retune.calls == []
    assert runner.restarts == 1


def test_live_retune_failure_falls_back_to_bounce():
    store = _store()  # width 40
    store.patch({"link": {"channel": 100}})
    runner, drone, retune = FakeRunner(), FakeDrone(reachable=False), FakeRetune(ok=False)
    coord = LinkCoordinator(store, lambda cfg: None, runner, drone, retune=retune)
    res = coord.apply_link("gs")
    assert retune.calls[0]["channel"] == 100   # tried live first
    assert runner.restarts == 1                # fell back to a bounce
    assert res["mode"] == "bounce"
    assert res["gsApplied"] is True
    assert store.effective()["link"]["channel"] == 100


# --- beamforming through /link/apply ----------------------------------------

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
    runner = FakeRunner()
    drone = BfDrone(drone_mac="00:c0:ca:dd:ee:ff")
    bf = FakeBf(gs_mac="84:fc:14:6c:36:e6")
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


def test_bf_enable_gs_scope_pending_no_drone_contact():
    # apply_to="gs": the drone is never contacted, so the GS can't learn the
    # drone MAC -> BF reports pending (NOT a hard-reject), GS still applies and
    # the intent commits. supported() is True so the hard-reject doesn't fire.
    store = _bf_store()
    store.patch({"link": {"beamforming": {"enabled": True}}})
    runner, drone, bf = FakeRunner(), BfDrone(reachable=True), FakeBf()
    res = _bf_coord(store, runner, drone, bf).apply_link("gs")
    assert drone.patched is None          # drone untouched on gs-scope
    assert bf.calls == []                 # not armed (no drone MAC)
    assert res["beamforming"]["state"] == "pending"
    assert res["droneApplied"] is False
    assert res["mode"] == "none"
    assert store.effective()["link"]["beamforming"]["enabled"] is True


class StagedBfDrone(FakeDrone):
    """Realistic drone: /status.beamforming.localMac is empty UNTIL BF is enabled
    via a pushed patch_config (mirrors the real drone, which only resolves its
    card MAC when its own BF reconciles enabled)."""
    def __init__(self, reachable=True, drone_mac="00:c0:ca:dd:ee:ff"):
        super().__init__(reachable=reachable)
        self._drone_mac = drone_mac
        self._bf_enabled = False

    def patch_config(self, sparse):
        bf = sparse.get("link", {}).get("beamforming")
        if bf is not None:
            self._bf_enabled = bool(bf.get("enabled"))
        return super().patch_config(sparse)

    def get_status(self):
        return {"beamforming": {"localMac": self._drone_mac if self._bf_enabled else ""}}


def test_bf_enable_arms_gs_in_single_apply_against_staged_drone():
    # The drone reports localMac only AFTER its BF is enabled. The coordinator
    # must push the enable FIRST, then read localMac, so the GS arms in ONE apply.
    store = _bf_store()
    store.patch({"link": {"beamforming": {"enabled": True}}})
    runner, drone, bf = FakeRunner(), StagedBfDrone(), FakeBf()
    res = _bf_coord(store, runner, drone, bf).apply_link("both")
    assert bf.calls == [(True, "wlan0", "00:c0:ca:dd:ee:ff")]   # armed this apply
    assert res["beamforming"]["state"] == "active"


def test_rollback_reconciles_bf_to_last_good():
    class FailingRunner:
        def __init__(self):
            self.restarts = 0
        def restart(self):
            self.restarts += 1
            return False
    # last-good: BF off. Apply enables BF + a structural wlans change (forces a
    # bounce), and the bounce fails -> rollback. BF must reconcile back to off.
    store = ConfigStore({"link": {"channel": 132, "width": 40, "region": "US"}})
    store.patch({"link": {"beamforming": {"enabled": True}, "wlans": ["wlanX"]}})
    runner, drone, bf = FailingRunner(), BfDrone(), FakeBf()
    coord = LinkCoordinator(store, lambda cfg: None, runner, drone,
                            beamforming=bf, wlans_resolver=lambda cfg: ["wlan0"])
    res = coord.apply_link("both")
    assert res["gsApplied"] is False
    assert bf.calls[-1] == (False, "wlan0", "")    # disarmed back to last-good
    assert store.effective()["link"].get("beamforming") in (None, {})  # not committed
