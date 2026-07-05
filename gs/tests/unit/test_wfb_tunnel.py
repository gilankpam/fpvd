"""Tests for the tuntap tunnel (gs/fpvdgs/wfb/tunnel.py): the frame batch
codec, the keepalive state machine, and the asyncio TunnelService plumbing.

Real-tun integration (opening /dev/net/tun + `ip` CLI configuration) is not
tested here — it needs root and a real network namespace. Instead,
`_FakeTun` stands in for `TunTap` via the `tun_factory` injection point.

`_FakeTun` uses a `socket.socketpair()` rather than a plain `os.pipe()`: a
real tun fd is a single, full-duplex fd (`os.read` drains uplink packets,
`os.write` injects downlink packets, both on the same fd number) — a pipe
only ever flows one direction per fd, so it cannot stand in for both
directions at once. A socketpair is bidirectional and both ends behave
like ordinary fds under `os.read`/`os.write`, which is what the service
code under test actually calls.
"""

import asyncio
import errno
import os
import socket
import struct

import pytest

from fpvdgs.wfb.tunnel import (
    TunnelConfig,
    TunnelService,
    pack_tun,
    unpack_batch,
)


def run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


# ---- (a): pure frame batch codec -------------------------------------------


def test_pack_tun_round_trip_single_packet():
    data = b"hello-ip-packet"
    assert unpack_batch(pack_tun(data)) == [data]


def test_unpack_batch_multi_packet():
    p1, p2, p3 = b"a", b"bb", b"ccc"
    batch = pack_tun(p1) + pack_tun(p2) + pack_tun(p3)
    assert unpack_batch(batch) == [p1, p2, p3]


def test_unpack_batch_empty_message_is_keepalive_noop():
    assert unpack_batch(b"") == []


def test_unpack_batch_corrupted_header_logs_and_stops(caplog):
    p1 = b"a"
    batch = pack_tun(p1) + b"\x00"  # 1 trailing byte: not enough for a length header
    with caplog.at_level("WARNING"):
        result = unpack_batch(batch)
    assert result == [p1]
    assert any("Corrupted tunneled packet header" in r.message for r in caplog.records)


def test_unpack_batch_truncated_body_logs_and_stops():
    p1 = b"a"
    # claims a 10-byte body but only 5 bytes follow
    batch = pack_tun(p1) + struct.pack("!H", 10) + b"short"
    result = unpack_batch(batch)
    assert result == [p1]


def test_unpack_batch_truncated_tail_after_valid_packets_still_returns_prior():
    p1, p2 = b"first", b"second"
    good = pack_tun(p1) + pack_tun(p2)
    truncated_tail = good + struct.pack("!H", 99) + b"nope"
    assert unpack_batch(truncated_tail) == [p1, p2]


# ---- (b): keepalive state machine, driven directly (no real sleeps) -------


class _RecordingSock:
    def __init__(self):
        self.sent: list[bytes] = []

    def send(self, data):
        self.sent.append(data)


def _service_with_fake_tx_socks():
    service = TunnelService(TunnelConfig())
    a, b = _RecordingSock(), _RecordingSock()
    service._tx_socks = {"a": a, "b": b}
    service._tx_sock_name = "a"
    service._tx_sock = a
    return service, a, b


def test_keepalive_fresh_rx_suppresses_broadcast_but_not_directed():
    service, a, b = _service_with_fake_tx_socks()
    service._pkt_in_sem = 1  # RX seen within the last interval -> no broadcast
    service._pkt_out_sem = 0  # idle TX -> directed keepalive to current only

    service._keepalive_tick()

    assert a.sent == [b""]  # current tx socket got the directed keepalive
    assert b.sent == []  # never touched: broadcast route did not fire
    assert service._pkt_in_sem == 0
    assert service._pkt_out_sem == 0


def test_keepalive_idle_two_intervals_broadcasts_to_all():
    service, a, b = _service_with_fake_tx_socks()
    service._pkt_in_sem = 0  # no RX for 2 whole intervals
    service._pkt_out_sem = 0  # also idle TX, but broadcast route takes priority

    service._keepalive_tick()

    assert a.sent == [b""]
    assert b.sent == [b""]


def test_keepalive_tx_traffic_suppresses_directed_keepalive():
    service, a, b = _service_with_fake_tx_socks()
    service._pkt_in_sem = 1  # not idle enough to broadcast
    service._pkt_out_sem = 1  # fresh TX -> no directed keepalive either

    service._keepalive_tick()

    assert a.sent == []
    assert b.sent == []
    assert service._pkt_in_sem == 0
    assert service._pkt_out_sem == 0


def test_keepalive_full_lifecycle_matches_state_machine():
    service, a, b = _service_with_fake_tx_socks()
    # A batch just arrived from the peer and we just sent uplink data.
    service._pkt_in_sem = 2
    service._pkt_out_sem = 1

    service._keepalive_tick()  # fresh both ways -> nothing sent
    assert a.sent == [] and b.sent == []
    assert (service._pkt_in_sem, service._pkt_out_sem) == (1, 0)

    service._keepalive_tick()  # pkt_out_sem now 0 -> directed keepalive
    assert a.sent == [b""] and b.sent == []
    assert (service._pkt_in_sem, service._pkt_out_sem) == (0, 0)

    service._keepalive_tick()  # pkt_in_sem now 0 too -> broadcast to all
    assert a.sent == [b"", b""] and b.sent == [b""]
    assert (service._pkt_in_sem, service._pkt_out_sem) == (0, 0)


# ---- (c)/(d): asyncio TunnelService plumbing via a fake tun fd -------------


class _FakeTun:
    """Test double for `TunTap` — see module docstring for why a
    socketpair, not a plain `os.pipe()`, backs the fake fd."""

    def __init__(self, name, addr_cidr, mtu):
        self.name = name
        self.addr_cidr = addr_cidr
        self.mtu = mtu - 2
        # SOCK_DGRAM so each os.read/os.write is one packet, matching a real
        # tun fd's one-read-one-packet semantics (SOCK_STREAM would coalesce
        # writes and lose the packet boundary the codec relies on).
        self.sock, self.peer = socket.socketpair(socket.AF_UNIX, socket.SOCK_DGRAM)
        self.sock.setblocking(False)
        self.peer.setblocking(False)
        self.fd = self.sock.fileno()
        self.closed = False

    def close(self):
        self.closed = True
        self.sock.close()


async def _recv_one(sock, timeout=2.0):
    loop = asyncio.get_running_loop()
    deadline = loop.time() + timeout
    while True:
        try:
            return sock.recv(65536)
        except BlockingIOError:
            if loop.time() > deadline:
                raise TimeoutError("no data arrived") from None
            await asyncio.sleep(0.005)


def test_tun_open_retries_on_transient_ebusy_then_succeeds():
    # A non-persistent tun netdev is torn down asynchronously by the kernel
    # after the old fd closes, so a fast engine restart can reach TUNSETIFF for
    # the same ifname while the old netdev still exists -> EBUSY. start() must
    # ride out that transient rather than fail engine setup (and crash-loop).
    async def main():
        cfg = TunnelConfig(keepalive_s=10.0)
        service = TunnelService(cfg)
        fake_tun = _FakeTun(cfg.ifname, cfg.ifaddr, cfg.mtu)
        calls = []

        def factory(name, addr_cidr, mtu):
            calls.append((name, addr_cidr, mtu))
            if len(calls) < 3:
                raise OSError(errno.EBUSY, "Device or resource busy")
            return fake_tun

        rx_unix_path = "fpvd-test-tunnel-rx-" + os.urandom(4).hex()
        loop = asyncio.get_running_loop()
        await service.start(loop, rx_unix_path, tun_factory=factory)
        try:
            assert len(calls) == 3  # two EBUSY retries, third open succeeds
            assert service._tun is fake_tun
        finally:
            await service.stop()

    run(main())


def test_tun_open_does_not_retry_non_ebusy_errors():
    # A non-EBUSY failure is a real error (missing /dev/net/tun, bad perms) —
    # fail fast, do not spin on the retry budget.
    async def main():
        cfg = TunnelConfig(keepalive_s=10.0)
        service = TunnelService(cfg)
        calls = []

        def factory(name, addr_cidr, mtu):
            calls.append(1)
            raise OSError(errno.EPERM, "Operation not permitted")

        rx_unix_path = "fpvd-test-tunnel-rx-" + os.urandom(4).hex()
        loop = asyncio.get_running_loop()
        with pytest.raises(OSError) as ei:
            await service.start(loop, rx_unix_path, tun_factory=factory)
        assert ei.value.errno == errno.EPERM
        assert len(calls) == 1  # raised on the first attempt, no retry

    run(main())


def test_tun_read_batches_through_agg_and_forwards_to_current_tx_socket():
    async def main():
        cfg = TunnelConfig(agg_timeout=0.01, keepalive_s=10.0)
        service = TunnelService(cfg)
        fake_tun = _FakeTun(cfg.ifname, cfg.ifaddr, cfg.mtu)

        def factory(name, addr_cidr, mtu):
            return fake_tun

        rx_unix_path = "fpvd-test-tunnel-rx-" + os.urandom(4).hex()
        tx_unix_path = "fpvd-test-tunnel-tx-" + os.urandom(4).hex()

        loop = asyncio.get_running_loop()
        await service.start(loop, rx_unix_path, tun_factory=factory)

        fake_tx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        fake_tx.setblocking(False)
        fake_tx.bind("\0" + tx_unix_path)
        try:
            service.set_tx_socket(tx_unix_path)

            payload = b"uplink-ip-packet-from-the-os"
            fake_tun.peer.send(payload)  # simulate the kernel handing a packet to the tun fd

            got = await _recv_one(fake_tx)
            assert got == pack_tun(payload)
        finally:
            fake_tx.close()
            await service.stop()

    run(main())


def test_rx_unix_batch_written_unpacked_to_fake_tun_fd():
    async def main():
        cfg = TunnelConfig(agg_timeout=0.01, keepalive_s=10.0)
        service = TunnelService(cfg)
        fake_tun = _FakeTun(cfg.ifname, cfg.ifaddr, cfg.mtu)

        def factory(name, addr_cidr, mtu):
            return fake_tun

        rx_unix_path = "fpvd-test-tunnel-rx2-" + os.urandom(4).hex()
        loop = asyncio.get_running_loop()
        await service.start(loop, rx_unix_path, tun_factory=factory)

        fake_rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            pkt1, pkt2 = b"downlink-packet-one", b"downlink-packet-two"
            batch = pack_tun(pkt1) + pack_tun(pkt2)
            fake_rx.connect("\0" + rx_unix_path)
            fake_rx.send(batch)

            got1 = await _recv_one(fake_tun.peer)
            got2 = await _recv_one(fake_tun.peer)
            assert [got1, got2] == [pkt1, pkt2]
        finally:
            fake_rx.close()
            await service.stop()

    run(main())


def test_rx_empty_batch_is_ignored_as_keepalive():
    async def main():
        cfg = TunnelConfig(agg_timeout=0.01, keepalive_s=10.0)
        service = TunnelService(cfg)
        fake_tun = _FakeTun(cfg.ifname, cfg.ifaddr, cfg.mtu)

        def factory(name, addr_cidr, mtu):
            return fake_tun

        rx_unix_path = "fpvd-test-tunnel-rx3-" + os.urandom(4).hex()
        loop = asyncio.get_running_loop()
        await service.start(loop, rx_unix_path, tun_factory=factory)

        fake_rx = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        try:
            assert service._pkt_in_sem == 0
            fake_rx.connect("\0" + rx_unix_path)
            fake_rx.send(b"")  # keepalive

            # give the datagram a moment to be delivered, then confirm
            # pkt_in_sem was bumped (peer is alive) with nothing written to tun
            for _ in range(50):
                if service._pkt_in_sem == 2:
                    break
                await asyncio.sleep(0.005)
            assert service._pkt_in_sem == 2

            try:
                fake_tun.peer.recv(65536)
            except BlockingIOError:
                pass
            else:
                raise AssertionError("keepalive payload must not be written to the tun fd")
        finally:
            fake_rx.close()
            await service.stop()

    run(main())


def test_set_tx_socket_reconnects_after_same_name_wfb_tx_respawn():
    """The realistic wfb_tx respawn case: a respawned child reuses the SAME
    argv, so it re-advertises the IDENTICAL abstract socket name as its dead
    predecessor. `set_tx_socket` must not reuse the cached (now-orphaned)
    connected socket -- a connected AF_UNIX DGRAM socket whose peer died
    keeps failing (ECONNREFUSED) and never rebinds on its own. It must
    close the stale cached socket and reconnect to the NEW listener bound
    at that name, mirroring MavlinkService.set_tx_socket's unconditional
    close+reconnect."""
    service = TunnelService(TunnelConfig())
    name = "fpvd-test-tunnel-tx-respawn-" + os.urandom(4).hex()

    listener1 = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    listener1.settimeout(2.0)
    listener1.bind("\0" + name)
    try:
        service.set_tx_socket(name)
        service._send_to_current_tx(b"first")
        assert listener1.recv(65536) == b"first"
    finally:
        listener1.close()

    # listener1 is gone: the old connected socket's peer is now dead.
    # Bind a BRAND NEW listener at the SAME abstract name -- simulating the
    # respawned wfb_tx's fresh handshake advertising an identical name.
    listener2 = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    listener2.settimeout(2.0)
    listener2.bind("\0" + name)
    try:
        service.set_tx_socket(name)  # same name, re-fired by the engine
        service._send_to_current_tx(b"second")
        got = listener2.recv(65536)
        assert got == b"second"
    finally:
        listener2.close()
        for sock in service._tx_socks.values():
            sock.close()


def test_set_all_tx_sockets_updates_broadcast_set_and_reuses_current():
    async def main():
        cfg = TunnelConfig(agg_timeout=0.01, keepalive_s=10.0)
        service = TunnelService(cfg)
        fake_tun = _FakeTun(cfg.ifname, cfg.ifaddr, cfg.mtu)

        def factory(name, addr_cidr, mtu):
            return fake_tun

        rx_unix_path = "fpvd-test-tunnel-rx4-" + os.urandom(4).hex()
        tx_a = "fpvd-test-tunnel-tx-a-" + os.urandom(4).hex()
        tx_b = "fpvd-test-tunnel-tx-b-" + os.urandom(4).hex()

        loop = asyncio.get_running_loop()
        await service.start(loop, rx_unix_path, tun_factory=factory)

        fake_a = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        fake_a.setblocking(False)
        fake_a.bind("\0" + tx_a)
        fake_b = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        fake_b.setblocking(False)
        fake_b.bind("\0" + tx_b)
        try:
            service.set_all_tx_sockets([tx_a, tx_b])
            service.set_tx_socket(tx_a)
            assert set(service._tx_socks) == {tx_a, tx_b}
            assert service._tx_sock_name == tx_a

            # dropping tx_a from the broadcast set while it's the current
            # selection clears the current selection too (it's no longer
            # a socket the service owns).
            service.set_all_tx_sockets([tx_b])
            assert set(service._tx_socks) == {tx_b}
            assert service._tx_sock_name is None
        finally:
            fake_a.close()
            fake_b.close()
            await service.stop()

    run(main())


def test_set_tx_socket_then_set_all_tx_sockets_respawn_sequence_no_resource_leak():
    """Regression test for the respawn sequence where set_tx_socket(name) is
    immediately followed by set_all_tx_sockets([name]) on the same tick — the
    real engine respawn path.

    The socket created by set_tx_socket must be explicitly closed by
    set_all_tx_sockets when it reconnects (not left to GC). This test verifies
    the fix by checking that: (a) the respawn sequence succeeds, (b) both
    current-socket and broadcast paths work after reconnect, and (c) the
    underlying implementation properly closes old sockets before reconnecting."""
    service = TunnelService(TunnelConfig())
    name = "fpvd-test-tunnel-respawn-" + os.urandom(4).hex()

    # Listener 1: first bind at this name (engine's initial set_tx_socket)
    listener1 = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    listener1.settimeout(2.0)
    listener1.bind("\0" + name)
    try:
        # Simulate the first activation: set_tx_socket creates and caches a socket
        service.set_tx_socket(name)
        assert service._tx_sock_name == name
        assert name in service._tx_socks
        first_sock_id = id(service._tx_socks[name])

        # Close listener1 to simulate wfb_tx shutdown
        listener1.close()

        # Now listener2 represents the respawned wfb_tx at the SAME abstract name
        listener2 = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        listener2.settimeout(2.0)
        listener2.bind("\0" + name)
        try:
            # Respawn sequence: set_tx_socket then set_all_tx_sockets on same tick
            # The respawn path in the real engine calls set_tx_socket then
            # immediately calls set_all_tx_sockets for the same name.
            service.set_tx_socket(name)
            second_sock_id = id(service._tx_socks[name])
            assert second_sock_id != first_sock_id, "set_tx_socket should create a fresh socket"

            # This is the critical operation: set_all_tx_sockets must close the
            # socket created above and reconnect. The fix ensures we call
            # explicit .close() on the popped socket instead of leaving it to GC.
            service.set_all_tx_sockets([name])
            third_sock_id = id(service._tx_socks[name])
            assert (
                third_sock_id != second_sock_id
            ), "set_all_tx_sockets should create a fresh socket"

            # Verify the new socket works on both paths:
            # (1) current socket path (used for uplink)
            service._send_to_current_tx(b"current-path-msg")
            assert listener2.recv(65536) == b"current-path-msg"

            # (2) broadcast path (used for keepalive)
            service._broadcast(b"broadcast-msg")
            assert listener2.recv(65536) == b"broadcast-msg"
        finally:
            listener2.close()
    finally:
        # Cleanup
        for sock in service._tx_socks.values():
            sock.close()
        service._tx_socks = {}
