import socket
import time

from fpvdgs.dynlink.controller import DynamicLinkController
from fpvdgs.dynlink.stats_client import RxAnt, RxEvent, SessionInfo


def _snapshot(drone_port, **over):
    snap = {
        "enabled": True, "maxMcs": 5, "bandwidth": 20,
        "txpower": {"min": 18, "max": 28}, "radioProfile": "m8812eu2",
        "droneAddr": "127.0.0.1", "dronePort": drone_port, "tuning": {},
        # IDR relay off by default so unrelated tests don't contend for :11223;
        # the dedicated relay test enables it on an ephemeral port.
        "idrForward": False,
    }
    snap.update(over)
    return snap


def _rx_event(stream_id="video rx", data=100, out=100, lost=0):
    return RxEvent(
        timestamp=1.0, id=stream_id,
        packets_window={"out": out, "lost": lost, "data": data},
        rx_ant_stats=[RxAnt(ant=0, freq=5825, mcs=2, bw=20, pkt_recv=100,
                            rssi_min=-60, rssi_avg=-55, rssi_max=-50,
                            snr_min=20, snr_avg=25, snr_max=30)],
        session=SessionInfo(fec_type="rs", fec_k=8, fec_n=12, epoch=1,
                            interleave_depth=1, contract_version=1),
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
    c = DynamicLinkController(_snapshot(40000),
                              stats_client_factory=_IdleStatsClient,
                              gs_listen_port=0)
    c.start()
    try:
        assert c.status()["running"] is True
    finally:
        c.stop()
    assert c.status()["running"] is False


def test_emits_decision_packet_to_drone():
    sock, port = _free_udp_port()
    sock.settimeout(2.0)
    c = DynamicLinkController(_snapshot(port),
                              stats_client_factory=_OneShotStatsClient,
                              gs_listen_port=0)
    c.start()
    try:
        data, _ = sock.recvfrom(64)
    finally:
        c.stop()
        sock.close()
    assert data[:4] == b"DLK1"
    assert len(data) == 31
    st = c.status()
    assert st["decision"]["mcs"] is not None
    assert st["emitSeq"] >= 1


def test_set_config_while_running_rebuilds_with_new_drone_port():
    sock_a, port_a = _free_udp_port()
    sock_b, port_b = _free_udp_port()
    sock_b.settimeout(2.0)
    c = DynamicLinkController(_snapshot(port_a),
                              stats_client_factory=_OneShotStatsClient,
                              gs_listen_port=0)
    c.start()
    try:
        c.set_config(_snapshot(port_b))
        data, _ = sock_b.recvfrom(64)   # now arrives on the new port
        assert data[:4] == b"DLK1"
        assert c.status()["running"] is True
    finally:
        c.stop()
        sock_a.close()
        sock_b.close()


def test_concurrent_set_config_no_hang():
    import threading as _t

    sock, port = _free_udp_port()
    c = DynamicLinkController(_snapshot(port),
                              stats_client_factory=_IdleStatsClient,
                              gs_listen_port=0)
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
            self._on(_rx_event(stream_id="video rx"))                  # one decision
            while not self._stop:
                await asyncio.sleep(0.02)

        def stop(self):
            self._stop = True

    c = DynamicLinkController(_snapshot(port), stats_client_factory=_MixedStats,
                              gs_listen_port=0)
    c.start()
    try:
        data, _ = sock.recvfrom(64)
        assert data[:4] == b"DLK1"
        time.sleep(0.2)
        assert c.status()["emitSeq"] == 1   # tunnel ignored; only video emitted
    finally:
        c.stop()
        sock.close()


def test_idr_relay_binds_inaddr_any_so_it_can_forward_off_loopback():
    # Regression: the relay reuses its listen socket to forward each token to
    # the drone. If it binds 127.0.0.1, that forward sendto() fails with EINVAL
    # for any non-loopback drone (the error is swallowed, so every IDR request
    # is dropped silently). The listen socket MUST bind INADDR_ANY (0.0.0.0).
    #
    # We can't also assert delivery to a fake same-host drone: a 0.0.0.0:idrPort
    # bind overlaps any 127.0.0.x:idrPort a fake drone would use (EADDRINUSE).
    # Forwarding loopback->loopback works with either bind anyway, so it would
    # never catch this bug — assert the bound listen address directly instead.
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    idr_port = probe.getsockname()[1]
    probe.close()

    c = DynamicLinkController(
        _snapshot(40010, droneAddr="10.255.255.1", idrForward=True, idrPort=idr_port),
        stats_client_factory=_IdleStatsClient, gs_listen_port=0)
    c.start()
    try:
        assert c.status()["idrListen"] == "0.0.0.0:%d" % idr_port
    finally:
        c.stop()
