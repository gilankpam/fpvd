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
