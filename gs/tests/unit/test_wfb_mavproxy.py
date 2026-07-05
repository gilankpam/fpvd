"""Tests for the mavlink framing parser, RADIO_STATUS packing, and
MavlinkService (gs/fpvdgs/wfb/mavproxy.py).

pack_radio_status golden (test c): pymavlink is not installed in this repo's
gs/.venv (verified: `import pymavlink` -> ModuleNotFoundError), so the
17-byte literal below was hand-verified with TWO independent CRC
implementations instead of round-tripping through the module under test:
the nibble-based x25crc from the brief, and a from-scratch table-driven
CRC-16/MCRF4XX (poly 0x8408, init 0xFFFF, no final xor — the well-documented
algorithm MAVLink's checksum.py uses). Both produced crc=0x2278 for the
same header+payload+crc_extra input, giving little-endian tail bytes
b"\\x78\\x22" below.
"""

import asyncio
import os
import socket
import struct

import pytest

from fpvdgs.wfb.mavproxy import (
    MavlinkConfig,
    MavlinkService,
    mavlink_parser_gen,
    pack_radio_status,
    x25crc,
)


def run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


# ---- (a)/(b)/GC: pure framing parser ---------------------------------------


def test_parser_splits_two_frames_with_garbage_prefix_and_resync():
    frame1 = pack_radio_status(0, 3, 68, 100, 100, 100, 10, 0, 5, 2)
    frame2 = pack_radio_status(1, 3, 68, 90, 90, 100, 12, 0, 1, 0)
    garbage = b"\x00\x01\x02"

    gen = mavlink_parser_gen()
    next(gen)
    frames = gen.send(garbage + frame1 + frame2)

    assert frames == [frame1, frame2]


def test_parser_v2_frame_with_signature_flag_length_math():
    # mavlink 2 header (after the 0xFD stx byte): plen, incompat_flags,
    # compat_flags, seq, sys_id, comp_id, msg_id_low(u16), msg_id_high(u8).
    plen = 4
    header = struct.pack("<BBBBBBHB", plen, 0x01, 0, 7, 1, 1, 0, 0)
    payload = b"\xaa" * plen
    crc = b"\x00\x00"
    sig = b"\x00" * 13  # signature present because incompat_flags & 0x01
    frame = b"\xfd" + header + payload + crc + sig
    # mlen = plen + 25 (signed) = 4 + 25 = 29, matching the frame we built.
    assert len(frame) == 29

    gen = mavlink_parser_gen()
    next(gen)
    frames = gen.send(frame)

    assert frames == [frame]


def test_parser_unsupported_flag_bits_logged_but_still_split(caplog):
    plen = 2
    # incompat_flags = 0x03 -> unsupported bit set alongside the signature bit
    header = struct.pack("<BBBBBBHB", plen, 0x03, 0, 1, 1, 1, 0, 0)
    frame = b"\xfd" + header + (b"\x11" * plen) + b"\x00\x00" + (b"\x00" * 13)

    gen = mavlink_parser_gen()
    next(gen)
    with caplog.at_level("WARNING"):
        frames = gen.send(frame)

    assert frames == [frame]
    assert any("Unsupported mavlink flags" in r.message for r in caplog.records)


def test_parser_gc_resets_skip_after_large_garbage_run():
    frame = pack_radio_status(0, 3, 68, 1, 1, 100, 1, 0, 0, 0)
    garbage = b"\x00" * 5000  # > 4096 GC threshold, all invalid stx bytes

    gen = mavlink_parser_gen()
    next(gen)
    frames = gen.send(garbage + frame)
    assert frames == [frame]

    # parser keeps working correctly across the GC boundary on a second call
    frame2 = pack_radio_status(1, 3, 68, 2, 2, 100, 2, 0, 0, 0)
    frames2 = gen.send(frame2)
    assert frames2 == [frame2]


# ---- (c): pack_radio_status / x25crc goldens -------------------------------


def test_x25crc_matches_independent_reference():
    def crc16_mcrf4xx(data, crc=0xFFFF):
        for b in data:
            crc ^= b
            for _ in range(8):
                crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
        return crc & 0xFFFF

    data = b"hello radio status crc check"
    assert x25crc(data) == crc16_mcrf4xx(data)


def test_pack_radio_status_golden_bytes():
    got = pack_radio_status(0, 3, 68, 100, 100, 100, 10, 0, 5, 2)
    expected = bytes(
        [
            0xFE,
            0x09,
            0x00,
            0x03,
            0x44,
            0x6D,
            0x05,
            0x00,
            0x02,
            0x00,
            0x64,
            0x64,
            0x64,
            0x0A,
            0x00,
            0x78,
            0x22,
        ]
    )
    assert got == expected
    assert len(got) == 17


PYMAVLINK_AVAILABLE = True
try:
    import pymavlink.mavutil  # noqa: F401
except ImportError:
    PYMAVLINK_AVAILABLE = False


@pytest.mark.skipif(not PYMAVLINK_AVAILABLE, reason="pymavlink not installed")
def test_pack_radio_status_matches_pymavlink_encoding():
    # Optional cross-check only exercised if pymavlink happens to be present
    # (it is not, in this repo's gs/.venv) — never a runtime dependency.
    from pymavlink.dialects.v10 import ardupilotmega as mavlink1

    mav = mavlink1.MAVLink(None, srcSystem=3, srcComponent=68)
    msg = mav.radio_status_encode(100, 100, 100, 10, 0, 5, 2)
    expected = bytearray(msg.pack(mav))
    expected[2] = 0  # normalize seq (pymavlink's global send-seq counter)
    got = bytearray(pack_radio_status(0, 3, 68, 100, 100, 100, 10, 0, 5, 2))
    assert bytes(got) == bytes(expected)


# ---- (d): end-to-end asyncio service ---------------------------------------


class _FakeHub:
    def __init__(self):
        self.rssi_cb = None

    def add_rssi_cb(self, cb):
        self.rssi_cb = cb


async def _recv_n_datagrams(sock, count, timeout=2.0):
    out = []
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while len(out) < count:
        try:
            out.append(sock.recvfrom(65536))
        except BlockingIOError:
            if loop.time() > deadline:
                raise TimeoutError(f"only received {len(out)}/{count} datagrams")
            await asyncio.sleep(0.005)
    return out


async def _recv_one(sock, timeout=2.0):
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while True:
        try:
            return sock.recv(65536)
        except BlockingIOError:
            if loop.time() > deadline:
                raise TimeoutError("no datagram arrived")
            await asyncio.sleep(0.005)


def test_end_to_end_downlink_split_and_uplink_via_agg_queue():
    async def main():
        peer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer_sock.bind(("127.0.0.1", 0))
        peer_sock.setblocking(False)
        peer_port = peer_sock.getsockname()[1]

        rx_unix_path = "fpvd-test-mav-rx-" + os.urandom(4).hex()
        tx_unix_path = "fpvd-test-mav-tx-" + os.urandom(4).hex()

        cfg = MavlinkConfig(
            peer=f"connect://127.0.0.1:{peer_port}",
            agg_max_size=1445,
            agg_timeout=0.05,
        )
        service = MavlinkService(cfg)
        hub = _FakeHub()
        loop = asyncio.get_running_loop()
        await service.start(loop, rx_unix_path, hub)

        fake_rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        fake_tx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            # downlink: fake wfb_rx sends a 2-frame batch -> 2 separate UDP
            # datagrams must arrive at the peer.
            frame1 = pack_radio_status(0, 3, 68, 100, 100, 100, 10, 0, 5, 2)
            frame2 = pack_radio_status(1, 3, 68, 90, 90, 100, 12, 0, 1, 0)
            fake_rx.connect("\0" + rx_unix_path)
            fake_rx.send(frame1 + frame2)

            datagrams = await _recv_n_datagrams(peer_sock, 2)
            assert [d for d, _addr in datagrams] == [frame1, frame2]
            service_addr = datagrams[0][1]

            # uplink: peer datagram -> AggQueue -> current tx unix socket,
            # once set_tx_socket points the service at our fake wfb_tx.
            fake_tx.setblocking(False)
            fake_tx.bind("\0" + tx_unix_path)
            service.set_tx_socket(tx_unix_path)

            uplink_payload = b"\x01\x02\x03uplink-from-mavlink-router"
            peer_sock.sendto(uplink_payload, service_addr)

            got = await _recv_one(fake_tx)
            assert got == uplink_payload
        finally:
            fake_rx.close()
            fake_tx.close()
            await service.stop()
            peer_sock.close()

    run(main())


def test_rssi_cb_injects_radio_status_to_peer():
    async def main():
        peer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer_sock.bind(("127.0.0.1", 0))
        peer_sock.setblocking(False)
        peer_port = peer_sock.getsockname()[1]

        rx_unix_path = "fpvd-test-mav-rssi-rx-" + os.urandom(4).hex()
        cfg = MavlinkConfig(peer=f"connect://127.0.0.1:{peer_port}", sys_id=3, comp_id=68)
        service = MavlinkService(cfg)
        hub = _FakeHub()
        loop = asyncio.get_running_loop()
        await service.start(loop, rx_unix_path, hub)

        try:
            assert hub.rssi_cb is not None
            hub.rssi_cb(-40, -80, 5, 2, 1)  # (rssi, noise, rx_errors, rx_fec, flags)

            got = await _recv_one(peer_sock)
            expected = pack_radio_status(
                0, 3, 68, (-40) % 256, (-40) % 256, 100, (-80) % 256, 1, 5, 2
            )
            assert got == expected
        finally:
            await service.stop()
            peer_sock.close()

    run(main())


def test_uplink_dropped_when_no_tx_socket_set():
    async def main():
        peer_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        peer_sock.bind(("127.0.0.1", 0))
        peer_sock.setblocking(False)
        peer_port = peer_sock.getsockname()[1]

        rx_unix_path = "fpvd-test-mav-notx-" + os.urandom(4).hex()
        cfg = MavlinkConfig(peer=f"connect://127.0.0.1:{peer_port}", agg_timeout=0.02)
        service = MavlinkService(cfg)
        hub = _FakeHub()
        loop = asyncio.get_running_loop()
        await service.start(loop, rx_unix_path, hub)

        try:
            # Drive the uplink path directly (no tx socket has been set via
            # set_tx_socket yet) -> AggQueue's flush->send() must be a
            # silent no-op, never raise.
            service._on_uplink(b"nobody-home", ("127.0.0.1", 1))
            await asyncio.sleep(0.05)
        finally:
            await service.stop()
            peer_sock.close()

    run(main())


def test_invalid_peer_address_raises():
    async def main():
        cfg = MavlinkConfig(peer="bogus://nope")
        service = MavlinkService(cfg)
        with pytest.raises(ValueError):
            await service.start(asyncio.get_running_loop(), "unused-path", _FakeHub())

    run(main())
