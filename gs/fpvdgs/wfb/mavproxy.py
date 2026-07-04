"""Mavlink framing, RADIO_STATUS injection, and the mavlink proxy service.

Port of wfb_ng's `mavlink_protocol.mavlink_parser_gen` (framing) and
`proxy.MavlinkUDPProxyProtocol` local-UDP-peer mode (no serial, no OSD
mirror, no mavlink TCP relay, no ARM hooks — none of those are in the GS's
rendered config). `MavlinkService` bridges wfb_rx/wfb_tx unix datagram
sockets to the local mavlink UDP peer (e.g. mavlink-router):

- downlink: wfb_rx batches arriving on an abstract-namespace AF_UNIX
  SOCK_DGRAM socket are split into individual mavlink frames (mirrors
  MavlinkUDPProxyProtocol.write's batch-splitting, done "due to issues
  with mavlink-router") and sent to the UDP peer one frame at a time.
- uplink: UDP peer datagrams are batched through an AggQueue and written
  to whichever wfb_tx unix socket is currently selected as the TX antenna
  (set_tx_socket, called by the engine's ant-sel callback).
- RSSI injection: StatsHub's per-window rssi_cb hand-packs a MAVLink v1
  RADIO_STATUS frame and writes it straight to the UDP peer (same write
  path as downlink, so it goes through the same one-frame split).

Everything here runs on a single asyncio loop (the wfb engine's), so no
cross-thread locking is needed inside this module.
"""

from __future__ import annotations

import asyncio
import logging
import re
import socket
import struct
from dataclasses import dataclass

from .agg_queue import AggQueue

log = logging.getLogger("fpvdgs.wfb")

_PEER_RE = re.compile(r"^(?P<scheme>connect|listen)://(?P<addr>[^:/]+):(?P<port>[0-9]+)$")

# GC threshold for the parser's leading-garbage counter — port of wfb-ng's
# `if skip > 4096: buffer = buffer[skip:]`.
_GC_THRESHOLD = 4096


def x25crc(data: bytes, crc: int = 0xFFFF) -> int:
    """X.25 / CRC-16-MCRF4XX, as used by MAVLink's checksum."""
    for b in data:
        tmp = (b ^ (crc & 0xFF)) & 0xFF
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


RADIO_STATUS_ID = 109
RADIO_STATUS_CRC_EXTRA = 185


def pack_radio_status(
    seq, sys_id, comp_id, rssi, remrssi, txbuf, noise, remnoise, rxerrors, fixed
) -> bytes:
    """Hand-pack a MAVLink v1 RADIO_STATUS (msg id 109) frame."""
    payload = struct.pack(
        "<HHBBBBB",
        rxerrors & 0xFFFF,
        fixed & 0xFFFF,
        rssi & 0xFF,
        remrssi & 0xFF,
        txbuf & 0xFF,
        noise & 0xFF,
        remnoise & 0xFF,
    )
    hdr = struct.pack("<BBBBBB", 0xFE, len(payload), seq & 0xFF, sys_id, comp_id, RADIO_STATUS_ID)
    crc = x25crc(hdr[1:] + payload + bytes([RADIO_STATUS_CRC_EXTRA]))
    return hdr + payload + struct.pack("<H", crc)


def mavlink_parser_gen():
    """Generator-based mavlink framer. Port of
    `wfb_ng.mavlink_protocol.mavlink_parser_gen` (parse_l2 dropped — the GS
    only ever needs raw frame bytes, never L2-decoded header fields).

    Usage: `gen = mavlink_parser_gen(); next(gen)` primes it, then each
    `gen.send(data)` feeds a chunk of bytes and returns the list of
    complete mavlink frames found in THAT call (not accumulated across
    calls — matching upstream, which resets `mlist` every iteration).
    """
    buffer = bytearray()
    mlist = []
    skip = 0
    bad = 0

    while True:
        # GC: drop consumed/garbage prefix once it grows past the threshold.
        if skip > _GC_THRESHOLD:
            buffer = buffer[skip:]
            skip = 0

        data = yield mlist
        mlist = []

        if not data:
            continue

        buffer.extend(data)

        while len(buffer) - skip >= 8:
            version = buffer[skip]

            if version == 0xFE:  # mavlink 1
                mlen = 8 + buffer[skip + 1]
            elif version == 0xFD:  # mavlink 2
                mlen, flags = struct.unpack("BB", buffer[skip + 1 : skip + 3])
                if flags & ~0x01:
                    log.warning("Unsupported mavlink flags: 0x%x", flags)
                mlen += 25 if flags & 0x01 else 12
            else:
                skip += 1
                bad += 1
                continue

            if bad:
                log.warning("skip %d bad bytes before sync", bad)
                bad = 0

            if len(buffer) - skip < mlen:
                break

            mlist.append(bytes(buffer[skip : skip + mlen]))
            skip += mlen


def _split_frames(data: bytes) -> list[bytes]:
    """One-shot frame split of a single batch — a fresh parser per call,
    mirroring `MavlinkUDPProxyProtocol.write`'s `with closing(mavlink_fsm)`.
    Each unix-socket batch is expected to hold whole frames only (the
    sender-side aggregator never splits a frame across batches)."""
    gen = mavlink_parser_gen()
    next(gen)
    try:
        return gen.send(data)
    finally:
        gen.close()


@dataclass
class MavlinkConfig:
    peer: str
    inject_rssi: bool = True
    sys_id: int = 3
    comp_id: int = 68
    agg_max_size: int = 1445
    agg_timeout: float = 0.1


class _PeerProtocol(asyncio.DatagramProtocol):
    """UDP endpoint to the local mavlink peer (e.g. mavlink-router)."""

    def __init__(self, on_datagram):
        self._on_datagram = on_datagram

    def datagram_received(self, data, addr):
        self._on_datagram(data, addr)

    def error_received(self, exc):
        log.warning("mavlink: udp peer error: %s", exc)


class _RxUnixProtocol(asyncio.DatagramProtocol):
    """AF_UNIX SOCK_DGRAM endpoint wfb_rx sends downlink batches to."""

    def __init__(self, on_batch):
        self._on_batch = on_batch

    def datagram_received(self, data, addr):
        self._on_batch(data)

    def error_received(self, exc):
        log.warning("mavlink: rx unix socket error: %s", exc)


class MavlinkService:
    """Bridges wfb_rx/wfb_tx unix sockets to the local mavlink UDP peer,
    with optional RADIO_STATUS RSSI injection. See module docstring for the
    data-flow description."""

    def __init__(self, cfg: MavlinkConfig):
        self.cfg = cfg
        self._loop = None
        self._udp_transport = None
        self._rx_transport = None
        self._reply_addr = None
        self._fixed_addr = False
        self._tx_sock: socket.socket | None = None
        self._uplink_agg: AggQueue | None = None
        self._seq = 0

    async def start(self, loop, rx_unix_path: str, hub) -> None:
        """Bind the UDP peer endpoint and the RX unix-datagram endpoint,
        and (if `inject_rssi`) subscribe to the hub's RSSI callback."""
        self._loop = loop

        m = _PEER_RE.match(self.cfg.peer)
        if not m:
            raise ValueError(f"unsupported mavlink peer address: {self.cfg.peer}")
        host, port, scheme = m.group("addr"), int(m.group("port")), m.group("scheme")

        if scheme == "connect":
            self._reply_addr = (host, port)
            self._fixed_addr = True
            self._udp_transport, _ = await loop.create_datagram_endpoint(
                lambda: _PeerProtocol(self._on_uplink),
                remote_addr=(host, port),
            )
        else:  # listen
            self._reply_addr = None
            self._fixed_addr = False
            self._udp_transport, _ = await loop.create_datagram_endpoint(
                lambda: _PeerProtocol(self._on_uplink),
                local_addr=(host, port),
            )

        self._uplink_agg = AggQueue(
            self.cfg.agg_max_size, self.cfg.agg_timeout, self._send_to_tx_socket
        )

        rx_sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        rx_sock.setblocking(False)
        rx_sock.bind("\0" + rx_unix_path)
        self._rx_transport, _ = await loop.create_datagram_endpoint(
            lambda: _RxUnixProtocol(self._on_downlink),
            sock=rx_sock,
        )

        if self.cfg.inject_rssi:
            hub.add_rssi_cb(self.rssi_cb)

    def set_tx_socket(self, abstract_name: str | None) -> None:
        """(Re)target the connected unix socket used for uplink writes —
        called by the engine's ant-sel callback whenever the TX-selected
        card (and therefore the wfb_tx unix socket) changes."""
        old = self._tx_sock
        self._tx_sock = None
        if old is not None:
            old.close()

        if abstract_name is None:
            return

        sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
        sock.setblocking(False)
        try:
            sock.connect("\0" + abstract_name)
        except OSError as e:
            log.warning("mavlink: connect tx socket %r failed: %s", abstract_name, e)
            sock.close()
            return
        self._tx_sock = sock

    def rssi_cb(self, rssi, noise, rx_errors, rx_fec, flags) -> None:
        """StatsHub rssi_cb: pack + inject a RADIO_STATUS frame straight to
        the UDP peer. Field mapping mirrors wfb-ng's `send_rssi` exactly
        (flags is carried in `remnoise` — txbuf is fixed at 100 since PX4
        throttles bandwidth on that field)."""
        if not self.cfg.inject_rssi:
            return
        seq = self._seq
        self._seq = (self._seq + 1) & 0xFF
        frame = pack_radio_status(
            seq,
            self.cfg.sys_id,
            self.cfg.comp_id,
            rssi % 256,
            rssi % 256,
            100,
            noise % 256,
            flags,
            rx_errors,
            rx_fec,
        )
        self._write_to_peer(frame)

    async def stop(self) -> None:
        # Stop anything that can call agg.put() before tearing the queue
        # down (AggQueue must not see .put() after .close()).
        if self._rx_transport is not None:
            self._rx_transport.close()
            self._rx_transport = None
        if self._udp_transport is not None:
            self._udp_transport.close()
            self._udp_transport = None
        if self._uplink_agg is not None:
            self._uplink_agg.flush()
            self._uplink_agg.close()
            self._uplink_agg = None
        if self._tx_sock is not None:
            self._tx_sock.close()
            self._tx_sock = None

    # -- internals ----------------------------------------------------------
    def _on_downlink(self, data: bytes) -> None:
        self._write_to_peer(data)

    def _write_to_peer(self, data: bytes) -> None:
        if self._udp_transport is None or self._reply_addr is None:
            return
        for frame in _split_frames(data):
            try:
                if self._fixed_addr:
                    self._udp_transport.sendto(frame)
                else:
                    self._udp_transport.sendto(frame, self._reply_addr)
            except OSError as e:
                log.warning("mavlink: udp send failed: %s", e)

    def _on_uplink(self, data: bytes, addr) -> None:
        if not self._fixed_addr:
            self._reply_addr = addr
        if self._uplink_agg is not None:
            self._uplink_agg.put(data)

    def _send_to_tx_socket(self, data: bytes) -> None:
        if self._tx_sock is None:
            return
        try:
            self._tx_sock.send(data)
        except OSError as e:
            log.warning("mavlink: tx socket send failed: %s", e)
