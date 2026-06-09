from fpvdgs.beamforming_armer import BeamformingArmer
from fpvdgs.drone_client import DroneUnreachable


class FakeBf:
    def __init__(self, state="disabled", supported=True):
        self._state = state
        self._supported = supported
        self.calls = []

    def status(self):
        return {"state": self._state}

    def supported(self, iface):
        return self._supported

    def reconcile(self, enabled, iface, peer):
        self.calls.append((enabled, iface, peer))
        self._state = "active" if enabled else "disabled"
        return {"state": self._state}


class FakeDrone:
    def __init__(self, reachable=True, mac="dc:84:03:e7:8f:0c", raise_status=False):
        self._reachable = reachable
        self._mac = mac
        self._raise = raise_status

    def healthz(self):
        return self._reachable

    def get_status(self):
        if self._raise:
            raise DroneUnreachable("down")
        return {"beamforming": {"localMac": self._mac}}


def _armer(bf, drone, enabled=True, wlans=None):
    cfg = {"link": {"beamforming": {"enabled": enabled}, "wlans": wlans or ["wlan0"]}}
    return BeamformingArmer(bf, drone, lambda c: c["link"]["wlans"], lambda: cfg)


def test_arms_when_enabled_not_active_drone_up():
    bf, drone = FakeBf(state="disabled"), FakeDrone()
    _armer(bf, drone)._tick()
    assert bf.calls == [(True, "wlan0", "dc:84:03:e7:8f:0c")]


def test_noop_when_already_active():
    bf, drone = FakeBf(state="active"), FakeDrone()
    _armer(bf, drone)._tick()
    assert bf.calls == []


def test_noop_when_disabled():
    bf, drone = FakeBf(state="disabled"), FakeDrone()
    _armer(bf, drone, enabled=False)._tick()
    assert bf.calls == []


def test_noop_when_drone_down():
    bf, drone = FakeBf(state="disabled"), FakeDrone(reachable=False)
    _armer(bf, drone)._tick()
    assert bf.calls == []


def test_survives_get_status_raising():
    bf, drone = FakeBf(state="disabled"), FakeDrone(raise_status=True)
    _armer(bf, drone)._tick()   # must not raise
    assert bf.calls == []


def test_noop_when_unsupported():
    bf, drone = FakeBf(state="disabled", supported=False), FakeDrone()
    _armer(bf, drone)._tick()
    assert bf.calls == []


def test_noop_when_no_drone_mac():
    bf, drone = FakeBf(state="disabled"), FakeDrone(mac="")
    _armer(bf, drone)._tick()
    assert bf.calls == []
