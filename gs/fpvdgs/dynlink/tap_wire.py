"""Decoder for the wfb_rx dynlink tap (tap wire v1).

Little-endian packed binary over localhost UDP, emitted by the forked
wfb_rx `-D <port>` (src/dynlink_tap.hpp). Two record types: MICRO (one
10 ms stats window; also the liveness heartbeat) and LOSS (immediate,
coalesced loss notification). Golden-hex tested byte-identical against
the fork's encoder (2026-07-02 spec)."""

from __future__ import annotations

import struct
from dataclasses import dataclass, field

from .stats_client import RxAnt

TAP_TYPE_MICRO = 0x01
TAP_TYPE_LOSS = 0x02
TAP_VERSION = 1

_HDR = struct.Struct("<BBHQIIIIIB")  # type, ver, seq, ts_ms, all/data/fec/lost/out, n
_BUCKET = struct.Struct("<HBBQIbbbbbbhhh")
_LOSS = struct.Struct("<BBHQIII")


class TapDecodeError(ValueError):
    """Malformed tap datagram (wrong length / truncated)."""


@dataclass
class TapMicro:
    """One 10 ms window from the tap. timestamp_ms is the wfb_rx clock —
    diagnostics only; consumers stamp Signals from the GS clock."""

    seq: int
    timestamp_ms: int
    pkt_all: int
    pkt_data: int
    pkt_fec_rec: int
    pkt_lost: int
    pkt_out: int
    rx_ant_stats: list[RxAnt] = field(default_factory=list)


@dataclass
class TapLoss:
    seq: int
    timestamp_ms: int
    lost_count: int
    last_seq: int
    new_seq: int


def decode(data: bytes) -> TapMicro | TapLoss | None:
    """One datagram -> record. Returns None for an unknown type/version
    (caller counts + ignores: forward-compat and clean fallback after a
    partial deploy). Raises TapDecodeError on a malformed datagram."""
    if len(data) < 2:
        raise TapDecodeError(f"short datagram ({len(data)} bytes)")
    rtype, version = data[0], data[1]
    if version != TAP_VERSION or rtype not in (TAP_TYPE_MICRO, TAP_TYPE_LOSS):
        return None
    if rtype == TAP_TYPE_LOSS:
        if len(data) != _LOSS.size:
            raise TapDecodeError(f"LOSS: {len(data)} bytes, want {_LOSS.size}")
        _, _, seq, ts, lost, last, new = _LOSS.unpack(data)
        return TapLoss(seq=seq, timestamp_ms=ts, lost_count=lost, last_seq=last, new_seq=new)
    if len(data) < _HDR.size:
        raise TapDecodeError(f"MICRO: {len(data)} bytes, want >= {_HDR.size}")
    _, _, seq, ts, p_all, p_data, p_fec, p_lost, p_out, n = _HDR.unpack_from(data, 0)
    if len(data) != _HDR.size + n * _BUCKET.size:
        raise TapDecodeError(f"MICRO: {len(data)} bytes for {n} buckets")
    ants: list[RxAnt] = []
    off = _HDR.size
    for _ in range(n):
        (freq, mcs, bw, ant, recv, rmin, ravg, rmax, smin, savg, smax, emin, eavg, emax) = (
            _BUCKET.unpack_from(data, off)
        )
        off += _BUCKET.size
        ants.append(
            RxAnt(
                ant=ant,
                freq=freq,
                mcs=mcs,
                bw=bw,
                pkt_recv=recv,
                rssi_min=rmin,
                rssi_avg=ravg,
                rssi_max=rmax,
                snr_min=smin,
                snr_avg=savg,
                snr_max=smax,
                evm_min=emin,
                evm_avg=eavg,
                evm_max=emax,
            )
        )
    return TapMicro(
        seq=seq,
        timestamp_ms=ts,
        pkt_all=p_all,
        pkt_data=p_data,
        pkt_fec_rec=p_fec,
        pkt_lost=p_lost,
        pkt_out=p_out,
        rx_ant_stats=ants,
    )
