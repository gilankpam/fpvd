"""Phase 3b — v3 decision-packet serialiser (mcs-only).

Byte-for-byte mirror of the drone's `drone/src/dl_wire.c` v3 decoder. The
authority is the drone C implementation; this module must match it exactly.
The test at `tests/unit/test_dl_wire_contract.py` cross-checks goldens.

Wire layout (big-endian, 15 bytes on-wire = 11 payload + 4 CRC32):

    off  size  field
     0    4    magic    = 0x444C4B31 ('DLK1')
     4    1    version  = 3
     5    1    flags
     6    4    sequence
    10    1    mcs
    11    4    crc32(bytes[0..10])    # big-endian u32
"""
from __future__ import annotations

import struct
from dataclasses import dataclass

from .decision import Decision

MAGIC        = 0x444C4B31    # 'DLK1'
VERSION      = 3
PAYLOAD_SIZE = 11
ON_WIRE_SIZE = 15


def _crc32(data: bytes) -> int:
    """Reflected CRC-32 (IEEE 802.3 / zlib-compatible) — same as
    `dl_wire_crc32` in the C implementation."""
    import binascii
    return binascii.crc32(data) & 0xFFFFFFFF


@dataclass
class Encoder:
    """Stateful encoder with a monotonic sequence counter.

    Start at `seq` (default 1; 0 is reserved as the "never seen" sentinel
    on the drone side's dedup logic, though any value works).
    """
    seq: int = 1

    def encode(
        self,
        decision: Decision,
        *,
        timestamp_ms: int | None = None,
        sequence: int | None = None,
    ) -> bytes:
        """Serialise one Decision to the 15-byte v3 on-wire form.

        Only `mcs` is carried on the wire in v3; all other Decision fields
        are ignored. `timestamp_ms` is accepted for call-site compatibility
        but ignored. `sequence` overrides the internal counter if provided;
        otherwise the counter is used and post-incremented.
        """
        if sequence is None:
            sequence = self.seq
            self.seq = (self.seq + 1) & 0xFFFFFFFF
        return _encode_raw(version=VERSION, flags=0,
                           sequence=sequence, mcs=int(decision.mcs))


def _encode_raw(
    *,
    version: int,
    flags: int,
    sequence: int,
    mcs: int,
) -> bytes:
    payload = struct.pack(">IBBIB", MAGIC, version, flags, sequence, mcs)
    crc = _crc32(payload)
    return payload + struct.pack(">I", crc)


def encode(decision: Decision, sequence: int) -> bytes:
    """Stateless convenience — encode with an explicit sequence."""
    return Encoder(seq=sequence).encode(decision, sequence=sequence)
