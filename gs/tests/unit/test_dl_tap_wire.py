"""Golden-hex tests for the tap wire v1 decoder — vectors normative, shared
byte-for-byte with the fork's src/dynlink_tap_test.cpp."""

import pytest

MICRO_HEX = (
    "010102017b4f0772900100008e0000008200000003000000010000008d00000002"
    "ad160514000100000000000058000000b9bec2161a1d0e0012001500"
    "ad160514010100000000000036000000b2b6ba0f1316ffffffffffff"
)
LOSS_HEX = "02010301834f0772900100000400000000ce010005ce0100"


def test_micro_golden_decode():
    from fpvdgs.dynlink.tap_wire import TapMicro, decode

    rec = decode(bytes.fromhex(MICRO_HEX))
    assert isinstance(rec, TapMicro)
    assert rec.seq == 258 and rec.timestamp_ms == 1719900000123
    assert (rec.pkt_all, rec.pkt_data, rec.pkt_fec_rec, rec.pkt_lost, rec.pkt_out) == (
        142,
        130,
        3,
        1,
        141,
    )
    assert len(rec.rx_ant_stats) == 2
    a, b = rec.rx_ant_stats
    assert (a.freq, a.mcs, a.bw, a.ant, a.pkt_recv) == (5805, 5, 20, 0x100, 88)
    assert (a.rssi_min, a.rssi_avg, a.rssi_max) == (-71, -66, -62)
    assert (a.snr_min, a.snr_avg, a.snr_max) == (22, 26, 29)
    assert (a.evm_min, a.evm_avg, a.evm_max) == (14, 18, 21)
    assert (b.ant, b.pkt_recv) == (0x101, 54)
    assert (b.evm_min, b.evm_avg, b.evm_max) == (-1, -1, -1)


def test_micro_heartbeat_zero_buckets():
    from fpvdgs.dynlink.tap_wire import TapMicro, decode

    hdr = bytes.fromhex("0101") + (0).to_bytes(2, "little") + (0).to_bytes(8, "little")
    hdr += b"\x00" * 20 + b"\x00"  # 5 zero counters + n_buckets=0
    rec = decode(hdr)
    assert isinstance(rec, TapMicro) and rec.rx_ant_stats == []


def test_loss_golden_decode():
    from fpvdgs.dynlink.tap_wire import TapLoss, decode

    rec = decode(bytes.fromhex(LOSS_HEX))
    assert isinstance(rec, TapLoss)
    assert rec.seq == 259 and rec.timestamp_ms == 1719900000131
    assert (rec.lost_count, rec.last_seq, rec.new_seq) == (4, 118272, 118277)


def test_unknown_version_returns_none():
    from fpvdgs.dynlink.tap_wire import decode

    data = bytearray(bytes.fromhex(LOSS_HEX))
    data[1] = 2  # future version
    assert decode(bytes(data)) is None


def test_unknown_type_returns_none():
    from fpvdgs.dynlink.tap_wire import decode

    data = bytearray(bytes.fromhex(LOSS_HEX))
    data[0] = 0x7F
    assert decode(bytes(data)) is None


def test_truncated_raises():
    from fpvdgs.dynlink.tap_wire import TapDecodeError, decode

    with pytest.raises(TapDecodeError):
        decode(bytes.fromhex(MICRO_HEX)[:-1])
    with pytest.raises(TapDecodeError):
        decode(bytes.fromhex(LOSS_HEX)[:10])
    with pytest.raises(TapDecodeError):
        decode(b"\x01")
