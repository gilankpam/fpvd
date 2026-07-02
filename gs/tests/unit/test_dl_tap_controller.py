"""Controller tap integration: micro-driven ticks, fast demote, fallback."""

import dataclasses
import socket
import struct
import time

import pytest

import fpvdgs.dynlink.controller as controller_mod
from fpvdgs.dynlink.controller import DynamicLinkController


@pytest.fixture(autouse=True)
def _isolate_dl_disk(tmp_path, monkeypatch):
    """Same isolation as test_dl_controller.py: keep the real Policy off the
    shared learned-prior / flightlog paths."""
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


class _IdleStatsClient:
    def __init__(self, endpoint, on_event):
        self._stopped = False

    def stop(self):
        self._stopped = True

    async def run(self):
        import asyncio

        while not self._stopped:
            await asyncio.sleep(0.01)


def _drone_sock():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    s.settimeout(3.0)
    return s, s.getsockname()[1]


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


def _micro_bytes(seq, ts_ms, data=9, fec=0, lost=0, out=10, snr=30, mcs=1):
    hdr = struct.pack("<BBHQIIIIIB", 1, 1, seq, ts_ms, out, data, fec, lost, out, 1)
    bucket = struct.pack(
        "<HBBQIbbbbbbhhh",
        5805,
        mcs,
        20,
        0x100,
        10,
        -70,
        -65,
        -60,
        snr - 3,
        snr,
        snr + 3,
        -1,
        -1,
        -1,
    )
    return hdr + bucket


def _loss_bytes(seq, ts_ms, lost):
    return struct.pack("<BBHQIII", 2, 1, seq, ts_ms, lost, 0, 0)


def _controller(drone_port, tap_port):
    snap = {
        "enabled": True,
        "maxMcs": 5,
        "droneAddr": "127.0.0.1",
        "dronePort": drone_port,
        "tap": {"enabled": True, "port": tap_port, "staleMs": 500, "captureRaw": False},
    }
    return DynamicLinkController(snap, stats_client_factory=_IdleStatsClient)


def _recv_decisions(sock, n, timeout=3.0):
    out = []
    deadline = time.monotonic() + timeout
    while len(out) < n and time.monotonic() < deadline:
        try:
            out.append(sock.recv(64))
        except socket.timeout:
            break
    return out


def test_micro_records_drive_ticks_every_tenth():
    drone, drone_port = _drone_sock()
    tap_port = _free_udp_port()
    ctl = _controller(drone_port, tap_port)
    ctl.start()
    try:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        for i in range(20):
            tx.sendto(_micro_bytes(i, 1000 + i * 10), ("127.0.0.1", tap_port))
            time.sleep(0.005)
        decisions = _recv_decisions(drone, 2)
        assert len(decisions) == 2  # 20 micros -> 2 ticks, no per-micro spam
        assert ctl.status()["tapActive"] is True
    finally:
        ctl.stop()
        drone.close()


def test_loss_record_fires_fast_demote_immediately():
    drone, drone_port = _drone_sock()
    tap_port = _free_udp_port()
    ctl = _controller(drone_port, tap_port)
    ctl.start()
    try:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # establish a healthy rolling window (9 micros: no tick yet)
        for i in range(9):
            tx.sendto(_micro_bytes(i, 1000 + i * 10), ("127.0.0.1", tap_port))
            time.sleep(0.002)
        # immediate loss: 30 lost >> 5% of ~90 out
        tx.sendto(_loss_bytes(100, 1095, 30), ("127.0.0.1", tap_port))
        decisions = _recv_decisions(drone, 1)
        assert len(decisions) == 1  # demote emitted without waiting for a tick
        st = ctl.status()
        assert "_fast" in st["reason"]
    finally:
        ctl.stop()
        drone.close()


def test_both_feeds_live_no_double_tick():
    """With the tap alive, :8103 RxEvents must not tick (session-only)."""
    import asyncio as _asyncio

    from fpvdgs.dynlink.stats_client import RxAnt, RxEvent

    class _ChattyStats:
        """Emits an RxEvent every 10 ms — would tick constantly in fallback."""

        def __init__(self, endpoint, on_event):
            self._on_event = on_event
            self._stopped = False

        def stop(self):
            self._stopped = True

        async def run(self):
            ev = RxEvent(
                timestamp=1.0,
                id="video rx",
                packets_window={"out": 100, "lost": 0, "data": 100},
                rx_ant_stats=[
                    RxAnt(
                        ant=0,
                        freq=5805,
                        mcs=1,
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
            )
            # let the test's first micro land first, so the tap is already
            # alive when events start flowing (deterministic, not a race)
            await _asyncio.sleep(0.05)
            while not self._stopped:
                self._on_event(ev)
                await _asyncio.sleep(0.01)

    drone, drone_port = _drone_sock()
    tap_port = _free_udp_port()
    snap = {
        "enabled": True,
        "maxMcs": 5,
        "droneAddr": "127.0.0.1",
        "dronePort": drone_port,
        "tap": {"enabled": True, "port": tap_port, "staleMs": 500, "captureRaw": False},
    }
    ctl = DynamicLinkController(snap, stats_client_factory=_ChattyStats)
    ctl.start()
    try:
        tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # make the tap alive FIRST, then let both feeds run ~100 ms
        tx.sendto(_micro_bytes(0, 1000), ("127.0.0.1", tap_port))
        time.sleep(0.02)
        for i in range(1, 10):
            tx.sendto(_micro_bytes(i, 1000 + i * 10), ("127.0.0.1", tap_port))
            time.sleep(0.005)
        # exactly ONE tick (10th micro) despite ~10 concurrent RxEvents. This
        # must observe only the short "both feeds alive" window, well short
        # of staleMs (500 ms): once the tap goes stale, the ever-chatty
        # :8103 fallback legitimately fires its own tick, which is a
        # different (later) scenario than what this test checks. _drone_sock
        # sets a long per-recv socket timeout (3.0 s) for the other tests'
        # generous windows, but that means a blocking recv() here would
        # happily wait right past our short deadline and pick up that later,
        # unrelated fallback tick too (observed landing a 2nd decision at the
        # 500 ms staleness boundary regardless of the deadline argument).
        # Shorten the per-call timeout so each recv() actually bounds itself
        # to this test's short window.
        drone.settimeout(0.05)
        decisions = _recv_decisions(drone, 2, timeout=0.15)
        assert len(decisions) == 1
        assert ctl.status()["tapActive"] is True
    finally:
        ctl.stop()
        drone.close()


def test_no_tap_falls_back_to_stats_events():
    from fpvdgs.dynlink.stats_client import RxAnt, RxEvent

    emitted = {}

    class _OneShotStats:
        def __init__(self, endpoint, on_event):
            self._on_event = on_event
            self._stopped = False

        def stop(self):
            self._stopped = True

        async def run(self):
            import asyncio

            ev = RxEvent(
                timestamp=1.0,
                id="video rx",
                packets_window={"out": 100, "lost": 0, "data": 100},
                rx_ant_stats=[
                    RxAnt(
                        ant=0,
                        freq=5805,
                        mcs=1,
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
            )
            self._on_event(ev)
            emitted["done"] = True
            while not self._stopped:
                await asyncio.sleep(0.01)

    drone, drone_port = _drone_sock()
    tap_port = _free_udp_port()
    snap = {
        "enabled": True,
        "maxMcs": 5,
        "droneAddr": "127.0.0.1",
        "dronePort": drone_port,
        "tap": {"enabled": True, "port": tap_port, "staleMs": 500, "captureRaw": False},
    }
    ctl = DynamicLinkController(snap, stats_client_factory=_OneShotStats)
    ctl.start()
    try:
        decisions = _recv_decisions(drone, 1)
        assert len(decisions) == 1  # :8103 event ticked because the tap is silent
        assert ctl.status()["tapActive"] is False
    finally:
        ctl.stop()
        drone.close()


def test_tap_capture_roundtrip(tmp_path):
    from fpvdgs.dynlink.tap_client import TapCapture, TapReplayClient
    from fpvdgs.dynlink.tap_wire import decode

    micro = decode(
        bytes.fromhex(
            "010102017b4f0772900100008e0000008200000003000000010000008d00000002"
            "ad160514000100000000000058000000b9bec2161a1d0e0012001500"
            "ad160514010100000000000036000000b2b6ba0f1316ffffffffffff"
        )
    )
    loss = decode(bytes.fromhex("02010301834f0772900100000400000000ce010005ce0100"))

    path = str(tmp_path / "cap.jsonl")
    cap = TapCapture(path)
    cap.write("micro", micro)
    cap.write("loss", loss)
    cap.close()

    micros, losses = [], []
    TapReplayClient(path, micros.append, losses.append).run()
    assert len(micros) == 1 and len(losses) == 1
    assert micros[0] == micro
    assert losses[0] == loss
