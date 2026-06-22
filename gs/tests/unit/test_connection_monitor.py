import time

from fpvdgs.connection_monitor import ConnectionMonitor, ConnectionMonitorConfig
from fpvdgs.dynlink.stats_client import RxEvent
from fpvdgs.events import DRONE_CONNECTED, DRONE_DISCONNECTED, EventBus


def _tunnel_rx(stream_id="tunnel rx"):
    return RxEvent(timestamp=1.0, id=stream_id, packets_window={"data": 1})


def _stats_factory(control):
    """Factory whose client emits a tunnel rx each loop while control['emit'];
    control['id'] selects the stream id so a test can emit non-tunnel records."""

    class _Stats:
        def __init__(self, endpoint, on_event):
            self._on = on_event
            self._stop = False

        async def run(self):
            import asyncio

            while not self._stop:
                if control.get("emit"):
                    self._on(_tunnel_rx(control.get("id", "tunnel rx")))
                await asyncio.sleep(0.01)

        def stop(self):
            self._stop = True

    return _Stats


class _FakeDrone:
    def __init__(self, status_ok=True, healthz_ok=True, version="d1"):
        self.status_ok = status_ok
        self.healthz_ok = healthz_ok
        self.version = version

    def get_status(self):
        if not self.status_ok:
            raise RuntimeError("unreachable")
        return {"version": self.version}

    def healthz(self):
        return self.healthz_ok


def _fast_cfg(**over):
    base = dict(
        tunnel_stale_s=0.2,
        http_poll_s=0.02,
        http_timeout_s=0.5,
        http_fail_count=2,
        eval_interval_s=0.02,
    )
    base.update(over)
    return ConnectionMonitorConfig(**base)


def _wait(pred, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


def test_connects_when_tunnel_and_http_ok():
    control = {"emit": True}
    bus = EventBus()
    got = []
    bus.subscribe(DRONE_CONNECTED, got.append)
    m = ConnectionMonitor(
        bus, _FakeDrone(version="d-9"), _fast_cfg(), stats_client_factory=_stats_factory(control)
    )
    m.start()
    try:
        assert _wait(lambda: bool(got)), "expected DRONE_CONNECTED"
        assert got[0]["drone"]["version"] == "d-9"
        assert m.status()["state"] == "connected"
    finally:
        m.stop()


def test_disconnect_on_tunnel_loss():
    control = {"emit": True}
    bus = EventBus()
    events = []
    bus.subscribe(DRONE_DISCONNECTED, events.append)
    m = ConnectionMonitor(
        bus, _FakeDrone(), _fast_cfg(), stats_client_factory=_stats_factory(control)
    )
    m.start()
    try:
        assert _wait(lambda: m.status()["state"] == "connected")
        control["emit"] = False  # tunnel goes silent
        assert _wait(lambda: bool(events)), "expected DRONE_DISCONNECTED"
        assert events[0]["reason"] == "tunnel_lost"
    finally:
        m.stop()


def test_disconnect_on_http_failure():
    control = {"emit": True}
    bus = EventBus()
    events = []
    bus.subscribe(DRONE_DISCONNECTED, events.append)
    drone = _FakeDrone()
    m = ConnectionMonitor(bus, drone, _fast_cfg(), stats_client_factory=_stats_factory(control))
    m.start()
    try:
        assert _wait(lambda: m.status()["state"] == "connected")
        drone.healthz_ok = False  # heartbeat starts failing
        assert _wait(lambda: bool(events)), "expected DRONE_DISCONNECTED"
        assert events[0]["reason"] == "http_failed"
    finally:
        m.stop()


def test_armed_without_http_never_announces_connected():
    control = {"emit": True}
    bus = EventBus()
    got = []
    bus.subscribe(DRONE_CONNECTED, got.append)
    m = ConnectionMonitor(
        bus, _FakeDrone(status_ok=False), _fast_cfg(), stats_client_factory=_stats_factory(control)
    )
    m.start()
    try:
        time.sleep(0.5)
        assert got == []  # tunnel up but HTTP never confirms
        assert m.status()["state"] == "armed"  # ARMED is observable in status()
    finally:
        m.stop()


def test_only_tunnel_stream_arms_the_monitor():
    control = {"emit": True, "id": "video rx"}  # video, not tunnel
    bus = EventBus()
    got = []
    bus.subscribe(DRONE_CONNECTED, got.append)
    m = ConnectionMonitor(
        bus, _FakeDrone(), _fast_cfg(), stats_client_factory=_stats_factory(control)
    )
    m.start()
    try:
        time.sleep(0.5)
        assert got == []  # never armed -> never connected
    finally:
        m.stop()


def test_disabled_does_not_start_a_thread():
    bus = EventBus()
    got = []
    bus.subscribe(DRONE_CONNECTED, got.append)
    m = ConnectionMonitor(
        bus,
        _FakeDrone(),
        _fast_cfg(enabled=False),
        stats_client_factory=_stats_factory({"emit": True}),
    )
    m.start()
    try:
        time.sleep(0.3)
        assert got == []
        assert m.status()["enabled"] is False
    finally:
        m.stop()


def test_enter_connected_payload_carries_radio_calibration():
    from fpvdgs.connection_monitor import ConnectionMonitor
    from fpvdgs.events import DRONE_CONNECTED, EventBus

    bus = EventBus()
    seen = []
    bus.subscribe(DRONE_CONNECTED, lambda p: seen.append(p))
    m = ConnectionMonitor(bus, drone_client=None)

    snap = {
        "version": "vX",
        "radio": {"adapterId": "bl-m8812eu2", "txPowerCurve": [29, 28, 25, 23, 19, 19, 19, 19]},
    }
    m._enter_connected(snap, now=1.0)

    assert len(seen) == 1
    radio = seen[0]["drone"]["radio"]
    assert radio["adapterId"] == "bl-m8812eu2"
    assert radio["txPowerCurve"] == [29, 28, 25, 23, 19, 19, 19, 19]


def test_enter_connected_without_radio_block_omits_it():
    from fpvdgs.connection_monitor import ConnectionMonitor
    from fpvdgs.events import DRONE_CONNECTED, EventBus

    bus = EventBus()
    seen = []
    bus.subscribe(DRONE_CONNECTED, lambda p: seen.append(p))
    m = ConnectionMonitor(bus, drone_client=None)

    m._enter_connected({"version": "vOld"}, now=1.0)
    assert "radio" not in seen[0]["drone"]
