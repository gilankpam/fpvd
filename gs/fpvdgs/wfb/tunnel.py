"""TUN/TAP tunnel bridging the drone's dynamic-link return channel (and any
other IP traffic riding the wfb tunnel) to the operator-side wfb_rx/wfb_tx
unix datagram sockets.

Port of wfb_ng's `tuntap.TUNTAPTransport` + `TUNTAPProtocol` (Twisted) to a
plain asyncio `TunTap` fd wrapper + `TunnelService`. Differences from
upstream, per the native-orchestration design:

- No pyroute2: `TunTap` configures the interface via the `ip` CLI
  (`ip link set ... up mtu ...` + `ip addr add ... dev ...`) instead of an
  `IPRoute()` netlink session.
- The batch codec (`pack_tun`/`unpack_batch`) is split into pure functions
  so the wire framing is unit-testable without opening a real tun fd.
- The keepalive `LoopingCall` is a plain `loop.call_later` chain, with the
  per-tick logic in `_keepalive_tick` so tests can drive the state machine
  directly with no real sleeps.

Keepalive semantics (unchanged from upstream `send_keepalive`): two
counters, `pkt_in_sem`/`pkt_out_sem`. Receiving a batch from the peer sets
`pkt_in_sem = 2`; reading data off the tun device (uplink) sets
`pkt_out_sem = 1`. Every `keepalive_s` tick: if there has been no RX from
the peer for two whole intervals (`pkt_in_sem == 0`), send an empty
keepalive to ALL tx sockets (this is what lets each end use independent
directed antennas and/or different frequency channels per card); else if
there has been no TX for one interval (`pkt_out_sem == 0`), send an empty
keepalive to the CURRENT tx socket only. Both counters then decrement,
floored at 0. An incoming empty payload is itself a keepalive and is
otherwise ignored.
"""

from __future__ import annotations

import asyncio
import fcntl
import logging
import os
import socket
import struct
import subprocess
from dataclasses import dataclass

from .agg_queue import AggQueue

log = logging.getLogger("fpvdgs.wfb")

TUNSETIFF = 0x400454CA
IFF_TUN = 0x0001
IFF_NO_PI = 0x1000


# -- pure wire framing --------------------------------------------------------


def pack_tun(data: bytes) -> bytes:
    """Length-prefix a single packet for the radio batch. Port of
    `TUNTAPProtocol.dataReceived`'s `struct.pack('!H', len(data)) + data`."""
    return struct.pack("!H", len(data)) + data


def unpack_batch(msg: bytes) -> list[bytes]:
    """Split a radio batch back into individual packets. Port of
    `TUNTAPProtocol.write`. A corrupted header (fewer than 2 bytes left) or
    a truncated body logs a warning and stops processing the rest of the
    batch (packets already parsed before the corruption are still
    returned). An empty batch is a keepalive and yields no packets."""
    packets: list[bytes] = []
    i = 0
    n = len(msg)
    while i < n:
        if n - i < 2:
            log.warning("Corrupted tunneled packet header: %r", msg[i:])
            break

        (pkt_size,) = struct.unpack("!H", msg[i : i + 2])
        i += 2

        if n - i < pkt_size:
            log.warning("Truncated tunneled packet body: %r", msg[i:])
            break

        packets.append(msg[i : i + pkt_size])
        i += pkt_size
    return packets


# -- tun device fd wrapper -----------------------------------------------------


class TunTap:
    """Opens `/dev/net/tun` in TUN|NO_PI mode and configures it via the `ip`
    CLI (no pyroute2 dependency). `mtu` is the caller-supplied radio-batch
    mtu; the tun interface itself is brought up at `mtu - 2` bytes (two
    bytes reserved for the length header each packet carries in the radio
    batch)."""

    def __init__(self, name: str, addr_cidr: str, mtu: int, dev: str = "/dev/net/tun"):
        self.name = name
        self.mtu = mtu - 2
        self.fd = os.open(dev, os.O_RDWR | os.O_NONBLOCK)
        try:
            fcntl.ioctl(
                self.fd,
                TUNSETIFF,
                struct.pack("16sH", name.encode("ascii"), IFF_TUN | IFF_NO_PI),
            )
            subprocess.run(["ip", "link", "set", name, "up", "mtu", str(self.mtu)], check=True)
            subprocess.run(["ip", "addr", "add", addr_cidr, "dev", name], check=True)
        except Exception:
            os.close(self.fd)
            raise

    def close(self) -> None:
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


# -- config ---------------------------------------------------------------


@dataclass
class TunnelConfig:
    ifname: str = "gs-wfb"
    ifaddr: str = "10.5.0.1/24"
    mtu: int = 1445
    agg_timeout: float = 0.005
    keepalive_s: float = 0.05  # 0.5 * log_interval(100ms) / 1000


class _RxUnixProtocol(asyncio.DatagramProtocol):
    """AF_UNIX SOCK_DGRAM endpoint wfb_rx delivers downlink batches to."""

    def __init__(self, on_batch):
        self._on_batch = on_batch

    def datagram_received(self, data, addr):
        self._on_batch(data)

    def error_received(self, exc):
        log.warning("tunnel: rx unix socket error: %s", exc)


class TunnelService:
    """Bridges the tun device to the wfb_rx/wfb_tx unix datagram sockets.

    Uplink (tun -> radio): a `loop.add_reader` on the tun fd reads raw IP
    packets, length-prefixes them (`pack_tun`), and feeds an `AggQueue`
    that batches them onto the CURRENTLY-selected tx socket
    (`set_tx_socket`).

    Downlink (radio -> tun): batches arriving on the rx unix datagram
    socket are split (`unpack_batch`) and written to the tun fd unpacked.

    See the module docstring for the keepalive state machine.
    """

    def __init__(self, cfg: TunnelConfig):
        self.cfg = cfg
        self._loop: asyncio.AbstractEventLoop | None = None
        self._tun = None
        self._agg: AggQueue | None = None
        self._rx_transport = None

        self._tx_socks: dict[str, socket.socket] = {}
        self._tx_sock_name: str | None = None
        self._tx_sock: socket.socket | None = None

        self._pkt_in_sem = 0
        self._pkt_out_sem = 0
        self._keepalive_handle: asyncio.TimerHandle | None = None

        self.dropped_tun_writes = 0

    async def start(self, loop, rx_unix_path: str, tun_factory=TunTap) -> None:
        self._loop = loop
        self._tun = tun_factory(self.cfg.ifname, self.cfg.ifaddr, self.cfg.mtu)
        self._agg = AggQueue(self.cfg.mtu, self.cfg.agg_timeout, self._send_to_current_tx)

        loop.add_reader(self._tun.fd, self._on_tun_readable)

        rx_sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        rx_sock.setblocking(False)
        rx_sock.bind("\0" + rx_unix_path)
        self._rx_transport, _ = await loop.create_datagram_endpoint(
            lambda: _RxUnixProtocol(self._on_rx_batch),
            sock=rx_sock,
        )

        self._schedule_keepalive()

    async def stop(self) -> None:
        # Stop anything that can call agg.put()/write to the tun fd before
        # tearing those down (AggQueue must not see .put() after .close()).
        if self._keepalive_handle is not None:
            self._keepalive_handle.cancel()
            self._keepalive_handle = None
        if self._rx_transport is not None:
            self._rx_transport.close()
            self._rx_transport = None
        if self._tun is not None and self._loop is not None:
            self._loop.remove_reader(self._tun.fd)
        if self._agg is not None:
            self._agg.flush()
            self._agg.close()
            self._agg = None
        for sock in self._tx_socks.values():
            sock.close()
        self._tx_socks = {}
        self._tx_sock = None
        self._tx_sock_name = None
        if self._tun is not None:
            self._tun.close()
            self._tun = None

    # -- tx antenna selection -------------------------------------------------

    def set_all_tx_sockets(self, names) -> None:
        """(Re)establish the full set of connected wfb_tx unix sockets used
        for the keepalive broadcast route (send-to-all when idle).

        ALWAYS closes any previously-cached socket for a given name and
        connects a fresh one, even if the name was already cached — a
        respawned wfb_tx reuses the SAME argv, so it re-advertises the
        IDENTICAL abstract socket name as its dead predecessor. A
        connected AF_UNIX DGRAM socket whose peer died keeps failing
        (ECONNREFUSED) and never rebinds to the new same-name listener on
        its own, so reuse-by-name would leave the tunnel blackholed after
        a same-name respawn. Mirrors `MavlinkService.set_tx_socket`'s
        unconditional close+reconnect."""
        names = list(names)
        old = dict(self._tx_socks)
        new: dict[str, socket.socket] = {}
        for name in names:
            existing = old.pop(name, None)
            if existing is not None:
                existing.close()
            sock = self._connect_tx_socket(name)
            if sock is not None:
                new[name] = sock

        for sock in old.values():
            sock.close()

        self._tx_socks = new
        if self._tx_sock_name is not None:
            self._tx_sock = new.get(self._tx_sock_name)
            if self._tx_sock is None:
                self._tx_sock_name = None

    def set_tx_socket(self, name: str | None) -> None:
        """Select the CURRENT tx socket (the ant-sel callback's active
        card) used for uplink data and directed keepalives.

        ALWAYS closes any cached socket for `name` and connects a fresh
        one — see `set_all_tx_sockets` for why a same-name respawn must
        not reuse a cached socket."""
        if name is None:
            self._tx_sock_name = None
            self._tx_sock = None
            return

        old = self._tx_socks.pop(name, None)
        if old is not None:
            old.close()

        sock = self._connect_tx_socket(name)
        if sock is None:
            return
        self._tx_socks[name] = sock

        self._tx_sock_name = name
        self._tx_sock = sock

    @staticmethod
    def _connect_tx_socket(name: str) -> socket.socket | None:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.setblocking(False)
        try:
            sock.connect("\0" + name)
        except OSError as e:
            log.warning("tunnel: connect tx socket %r failed: %s", name, e)
            sock.close()
            return None
        return sock

    # -- keepalive --------------------------------------------------------

    def _schedule_keepalive(self) -> None:
        self._keepalive_handle = self._loop.call_later(self.cfg.keepalive_s, self._keepalive_loop)

    def _keepalive_loop(self) -> None:
        self._keepalive_tick()
        self._schedule_keepalive()

    def _keepalive_tick(self) -> None:
        """One `send_keepalive` iteration. Exposed directly (rather than
        only reachable via the real timer) so tests can drive the state
        machine without real sleeps."""
        if self._pkt_in_sem == 0:
            self._broadcast(b"")
        elif self._pkt_out_sem == 0:
            self._send_to_current_tx(b"")

        if self._pkt_in_sem > 0:
            self._pkt_in_sem -= 1
        if self._pkt_out_sem > 0:
            self._pkt_out_sem -= 1

    def _broadcast(self, data: bytes) -> None:
        for name, sock in self._tx_socks.items():
            try:
                sock.send(data)
            except OSError as e:
                log.warning("tunnel: broadcast send to %r failed: %s", name, e)

    def _send_to_current_tx(self, data: bytes) -> None:
        if self._tx_sock is None:
            return
        try:
            self._tx_sock.send(data)
        except OSError as e:
            log.warning("tunnel: tx socket send failed: %s", e)

    # -- uplink: tun -> radio -----------------------------------------------

    def _on_tun_readable(self) -> None:
        try:
            data = os.read(self._tun.fd, self._tun.mtu)
        except BlockingIOError:
            return
        except OSError as e:
            log.warning("tunnel: tun read error: %s", e)
            return
        if not data:
            return

        self._pkt_out_sem = 1
        if self._agg is not None:
            self._agg.put(pack_tun(data))

    # -- downlink: radio -> tun -----------------------------------------------

    def _on_rx_batch(self, data: bytes) -> None:
        self._pkt_in_sem = 2

        # Ignore incoming empty keepalive payloads.
        if not data:
            return

        for pkt in unpack_batch(data):
            self._write_to_tun(pkt)

    def _write_to_tun(self, pkt: bytes) -> None:
        if self._tun is None:
            return
        try:
            os.write(self._tun.fd, pkt)
        except BlockingIOError:
            self.dropped_tun_writes += 1
            log.warning("tunnel: tun write dropped (fd busy)")
