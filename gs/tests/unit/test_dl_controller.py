import dataclasses
import socket
import time

import pytest

import fpvdgs.dynlink.controller as controller_mod
from fpvdgs.dynlink.controller import DynamicLinkController
from fpvdgs.dynlink.stats_client import RxAnt, RxEvent, SessionInfo


@pytest.fixture(autouse=True)
def _isolate_dl_disk(tmp_path, monkeypatch):
    """Keep the controller's real Policy off the shared on-disk paths.

    build_policy_config freezes learned_prior.persist_dir (/etc/fpvd/learned)
    and flightlog.dir (/media/dvr/log/dynamic-link/), and a real
    DynamicLinkController here reads/writes both. On dev/CI those paths are
    unwritable so the I/O errors are silently swallowed, but on a writable box
    (root, or the bench device) these tests would read stale state and clobber
    the production learned-prior / flight-log files. Redirect both to tmp_path —
    the real persist logic still runs, just isolated."""
    real = controller_mod.build_policy_config

    def _to_tmp(block):
        cfg = real(block)
        return dataclasses.replace(
            cfg,
            learned_prior=dataclasses.replace(
                cfg.learned_prior, persist_dir=str(tmp_path / "learned")
            ),
            flightlog=dataclasses.replace(cfg.flightlog, dir=str(tmp_path / "fl")),
        )

    monkeypatch.setattr(controller_mod, "build_policy_config", _to_tmp)


def _snapshot(drone_port, **over):
    snap = {
        "enabled": True,
        "maxMcs": 5,
        "radioProfile": "m8812eu2",
        "droneAddr": "127.0.0.1",
        "dronePort": drone_port,
    }
    snap.update(over)
    return snap


def _rx_event(stream_id="video rx", data=100, out=100, lost=0):
    return RxEvent(
        timestamp=1.0,
        id=stream_id,
        packets_window={"out": out, "lost": lost, "data": data},
        rx_ant_stats=[
            RxAnt(
                ant=0,
                freq=5825,
                mcs=2,
                bw=20,
                pkt_recv=100,
                rssi_min=-60,
                rssi_avg=-55,
                rssi_max=-50,
                snr_min=20,
                snr_avg=25,
                snr_max=30,
            )
        ],
        session=SessionInfo(
            fec_type="rs", fec_k=8, fec_n=12, epoch=1, interleave_depth=1, contract_version=1
        ),
    )


class _IdleStatsClient:
    """Connects (sets statsConnected) but emits nothing until stopped."""

    def __init__(self, endpoint, on_event):
        self._stop = False

    async def run(self):
        while not self._stop:
            import asyncio

            await asyncio.sleep(0.02)

    def stop(self):
        self._stop = True


class _OneShotStatsClient:
    """Emits a single RxEvent on connect, then idles."""

    def __init__(self, endpoint, on_event):
        self._on_event = on_event
        self._stop = False

    async def run(self):
        import asyncio

        self._on_event(_rx_event())
        while not self._stop:
            await asyncio.sleep(0.02)

    def stop(self):
        self._stop = True


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    return s, port


def test_start_sets_running_then_stop_joins():
    c = DynamicLinkController(_snapshot(40000), stats_client_factory=_IdleStatsClient)
    c.start()
    try:
        assert c.status()["running"] is True
    finally:
        c.stop()
    assert c.status()["running"] is False


def test_emits_decision_packet_to_drone():
    sock, port = _free_udp_port()
    sock.settimeout(2.0)
    c = DynamicLinkController(_snapshot(port), stats_client_factory=_OneShotStatsClient)
    c.start()
    try:
        data, _ = sock.recvfrom(64)
    finally:
        c.stop()
        sock.close()
    assert data[:4] == b"DLK1"
    assert len(data) == 15
    assert data[4] == 3  # version == 3
    st = c.status()
    assert st["decision"]["mcs"] is not None
    assert st["emitSeq"] >= 1


def test_set_config_while_running_rebuilds_with_new_drone_port():
    sock_a, port_a = _free_udp_port()
    sock_b, port_b = _free_udp_port()
    sock_b.settimeout(2.0)
    c = DynamicLinkController(_snapshot(port_a), stats_client_factory=_OneShotStatsClient)
    c.start()
    try:
        c.set_config(_snapshot(port_b))
        data, _ = sock_b.recvfrom(64)  # now arrives on the new port
        assert data[:4] == b"DLK1"
        assert c.status()["running"] is True
    finally:
        c.stop()
        sock_a.close()
        sock_b.close()


def test_concurrent_set_config_no_hang():
    import threading as _t

    sock, port = _free_udp_port()
    c = DynamicLinkController(_snapshot(port), stats_client_factory=_IdleStatsClient)
    c.start()
    errors = []

    def hammer():
        try:
            for _ in range(5):
                c.set_config(_snapshot(port))
        except Exception as e:  # noqa: BLE001
            errors.append(e)

    threads = [_t.Thread(target=hammer) for _ in range(4)]
    for t in threads:
        t.start()
    for t in threads:
        t.join(timeout=20)
    alive = [t for t in threads if t.is_alive()]
    try:
        assert not alive, "lifecycle hammer threads hung"
        assert not errors, f"errors during concurrent set_config: {errors}"
        assert c.status()["running"] is True
    finally:
        c.stop()
        sock.close()


def test_non_video_streams_are_ignored():
    # :8103 interleaves video/mavlink/tunnel rx records. Only the video stream
    # may drive the policy — a low-rate non-video record must NOT be consumed
    # (it would trip link_starved and pin MCS at the floor).
    sock, port = _free_udp_port()
    sock.settimeout(2.0)

    class _MixedStats:
        def __init__(self, endpoint, on_event):
            self._on = on_event
            self._stop = False

        async def run(self):
            import asyncio

            self._on(_rx_event(stream_id="tunnel rx", data=0, out=0))  # ignore
            self._on(_rx_event(stream_id="video rx"))  # one decision
            while not self._stop:
                await asyncio.sleep(0.02)

        def stop(self):
            self._stop = True

    c = DynamicLinkController(_snapshot(port), stats_client_factory=_MixedStats)
    c.start()
    try:
        data, _ = sock.recvfrom(64)
        assert data[:4] == b"DLK1"
        time.sleep(0.2)
        assert c.status()["emitSeq"] == 1  # tunnel ignored; only video emitted
    finally:
        c.stop()
        sock.close()


class _RepeatStatsClient:
    """Emits a video RxEvent every ~20 ms until stopped."""

    def __init__(self, endpoint, on_event):
        self._on_event = on_event
        self._stop = False

    async def run(self):
        import asyncio

        while not self._stop:
            self._on_event(_rx_event())
            await asyncio.sleep(0.02)

    def stop(self):
        self._stop = True


def test_controller_forwards_probe_snapshot_to_policy():
    # The controller must pull the probe snapshot each policy tick (so the
    # selector can promote). With the sync-gate gone, Policy.tick() runs from
    # the first stats event — no HELLO handshake required.
    seen = {}

    def fake_probe_status():
        seen["called"] = seen.get("called", 0) + 1
        return {"running": True, "streams": 1, "mcs": {}}

    drone_sock, drone_port = _free_udp_port()
    c = DynamicLinkController(
        _snapshot(drone_port),
        stats_client_factory=_RepeatStatsClient,
        probe_status=fake_probe_status,
    )
    c.start()
    try:
        deadline = time.monotonic() + 1.5
        while seen.get("called", 0) < 1 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert seen.get("called", 0) >= 1  # tick loop pulled the probe snapshot
    finally:
        c.stop()
        drone_sock.close()


def test_connect_event_resets_selector_and_begins_flight():
    from fpvdgs.events import DRONE_CONNECTED, EventBus

    bus = EventBus()
    drone_sock, drone_port = _free_udp_port()
    c = DynamicLinkController(
        _snapshot(drone_port), stats_client_factory=_RepeatStatsClient, bus=bus
    )
    c.start()
    try:
        # wait for the policy to exist, then simulate a climbed-up session
        deadline = time.monotonic() + 1.5
        while c._policy is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert c._policy is not None
        # GIL-atomic write; a loop tick may run between this and the publish,
        # but the post-reset assertion (==1) holds regardless of what 5 became.
        c._policy.leading.state.current_mcs = 5

        bus.publish(DRONE_CONNECTED, {"state": "connected", "drone": {}})

        deadline = time.monotonic() + 1.5
        while c._policy.leading.state.current_mcs != 1 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert c._policy.leading.state.current_mcs == 1  # reset to boot on reconnect
    finally:
        c.stop()
        drone_sock.close()


def test_disconnect_event_flushes_prior():
    from fpvdgs.events import DRONE_DISCONNECTED, EventBus

    bus = EventBus()
    drone_sock, drone_port = _free_udp_port()
    c = DynamicLinkController(
        _snapshot(drone_port), stats_client_factory=_RepeatStatsClient, bus=bus
    )
    c.start()
    try:
        deadline = time.monotonic() + 1.5
        while c._policy is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert c._policy is not None
        flushed = []
        c._policy.learned_prior.flush = lambda: flushed.append(True)  # spy

        bus.publish(DRONE_DISCONNECTED, {"state": "disconnected", "reason": "tunnel_lost"})

        deadline = time.monotonic() + 1.5
        while not flushed and time.monotonic() < deadline:
            time.sleep(0.02)
        assert flushed, "disconnect must flush the learned prior"
    finally:
        c.stop()
        drone_sock.close()


def test_publish_after_stop_is_safe_noop():
    # The _loop-None guard in _marshal must make a connection event a harmless
    # no-op once the controller has stopped (e.g. dynamicLink disabled).
    from fpvdgs.events import DRONE_CONNECTED, DRONE_DISCONNECTED, EventBus

    bus = EventBus()
    drone_sock, drone_port = _free_udp_port()
    c = DynamicLinkController(
        _snapshot(drone_port), stats_client_factory=_RepeatStatsClient, bus=bus
    )
    c.start()
    c.stop()
    # _loop is None now -> these must not raise
    bus.publish(DRONE_CONNECTED, {"state": "connected", "drone": {}})
    bus.publish(DRONE_DISCONNECTED, {"state": "disconnected", "reason": "tunnel_lost"})
    drone_sock.close()  # reaching here without an exception is the assertion
