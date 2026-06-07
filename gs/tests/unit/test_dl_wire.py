"""Unit tests for the v3 Decision wire encoder."""
import binascii
import struct


def test_v3_decision_encodes_15_bytes_mcs_only():
    from fpvdgs.dynlink.wire import Encoder, MAGIC, VERSION
    from fpvdgs.dynlink.decision import Decision
    enc = Encoder(seq=7)
    d = Decision(timestamp=0.0, mcs=5)
    raw = enc.encode(d, sequence=0xAABBCCDD)
    assert len(raw) == 15
    magic, ver, flags, seq, mcs = struct.unpack(">IBBIB", raw[:11])
    assert magic == MAGIC and ver == 3 and flags == 0 and seq == 0xAABBCCDD and mcs == 5
    assert binascii.crc32(raw[:11]) & 0xFFFFFFFF == struct.unpack(">I", raw[11:15])[0]


def test_v3_version_constant():
    from fpvdgs.dynlink.wire import VERSION
    assert VERSION == 3


def test_v3_on_wire_size_constant():
    from fpvdgs.dynlink.wire import ON_WIRE_SIZE
    assert ON_WIRE_SIZE == 15


def test_v3_payload_size_constant():
    from fpvdgs.dynlink.wire import PAYLOAD_SIZE
    assert PAYLOAD_SIZE == 11


def test_v3_sequence_counter_advances():
    from fpvdgs.dynlink.wire import Encoder
    from fpvdgs.dynlink.decision import Decision
    enc = Encoder(seq=10)
    d = Decision(timestamp=0.0, mcs=3)
    raw1 = enc.encode(d)
    raw2 = enc.encode(d)
    seq1 = struct.unpack(">IBBIB", raw1[:11])[3]
    seq2 = struct.unpack(">IBBIB", raw2[:11])[3]
    assert seq1 == 10
    assert seq2 == 11


def test_v3_other_decision_fields_ignored():
    """encode produces the same bytes regardless of non-mcs fields (only
    mcs and the encoder's sequence affect the wire in v3)."""
    from fpvdgs.dynlink.wire import Encoder
    from fpvdgs.dynlink.decision import Decision
    enc1 = Encoder(seq=1)
    enc2 = Encoder(seq=1)
    d1 = Decision(timestamp=0.0, mcs=5, reason="a")
    d2 = Decision(timestamp=99.0, mcs=5, reason="b",
                  signals_snapshot={"rssi": -60})
    assert enc1.encode(d1, sequence=42) == enc2.encode(d2, sequence=42)
