"""Wire-format contract: the GS DLK1 v3 encoder must produce bytes that the
drone's dynlink/wire.cpp decoder accepts. Goldens were computed from the
authoritative v3 wire layout (15 bytes: 11 payload + 4 CRC32); the struct
layout is `struct.pack(">IBBIB", MAGIC, 3, flags, seq, mcs)` + crc32 BE u32.
Do not regenerate these from the Python encoder — that would make the test
circular. Recompute by hand from the layout spec."""

from fpvdgs.dynlink.decision import Decision
from fpvdgs.dynlink.wire import encode

# Decisions are 15 bytes (30 hex) in v3.
# Goldens computed from: struct.pack(">IBBIB", 0x444C4B31, 3, 0, seq, mcs) + crc32 BE.
GOLDEN_DECISION_1 = "444c4b3103000000000105c4a92dc5"  # seq=1,  mcs=5
GOLDEN_DECISION_MCS0 = "444c4b3103000000000700e2997ecc"  # seq=7,  mcs=0
GOLDEN_DECISION_MAX = "444c4b310300ffffffff070a61754a"  # seq=0xFFFFFFFF, mcs=7


# Fields that existed on the fat v2 Decision but were dropped in v3 — the
# encoder never read them, so callers passing them are simply ignored.
_DROPPED_FIELDS = {"bandwidth", "tx_power_dBm", "k", "n", "depth", "bitrate_kbps"}


def _decision(**overrides) -> Decision:
    base = Decision(timestamp=0.0, mcs=5)
    for k, v in overrides.items():
        if k in _DROPPED_FIELDS:
            continue
        setattr(base, k, v)
    return base


def test_decision_golden():
    assert encode(_decision(), sequence=1).hex() == GOLDEN_DECISION_1


def test_decision_mcs_zero():
    """mcs=0 at seq=7 matches the neg golden (only mcs matters in v3)."""
    pkt = encode(
        _decision(mcs=0, tx_power_dBm=-10, k=2, n=4, depth=1, bitrate_kbps=2000), sequence=7
    )
    assert pkt.hex() == GOLDEN_DECISION_MCS0


def test_decision_max_values():
    """mcs=7 at seq=0xFFFFFFFF matches the max golden."""
    pkt = encode(
        _decision(mcs=7, bandwidth=40, tx_power_dBm=30, k=8, n=16, depth=3, bitrate_kbps=65000),
        sequence=0xFFFFFFFF,
    )
    assert pkt.hex() == GOLDEN_DECISION_MAX


def test_decision_magic_and_version():
    pkt = encode(_decision(), sequence=1)
    assert pkt[:4] == b"DLK1"
    assert pkt[4] == 3
