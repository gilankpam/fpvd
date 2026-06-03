"""Wire-format contract: the GS DLK1 v2 encoder must produce bytes that the
drone's dynlink/wire.cpp decoder accepts. Goldens were captured from the
authoritative C encoder (dynamic-link `dl-inject --dry-run`); fpvd's C++
decoder is a port of the same dl_wire.c. Do not regenerate these from the
Python encoder — that would make the test circular."""
from fpvdgs.dynlink.decision import Decision
from fpvdgs.dynlink.wire import (
    Hello, HelloAck, encode, encode_hello, encode_hello_ack,
)

# Decisions are 31 bytes (62 hex); HELLO / HELLO-ACK are 32 bytes (64 hex).
GOLDEN_DECISION_1 = "444c4b31020000000000000100000001051412080e022ee0000000a34fec51"
GOLDEN_DECISION_NEG = "444c4b310200000000000007000000070014f602040107d000000086b0d80c"
GOLDEN_DECISION_MAX = "444c4b3102000000ffffffffffffffff07281e081003fde8000000a092ca14"
GOLDEN_HELLO = "444c484502000000cafebabe0f9a003cdeadbeef0000000000000000b193a0b1"
GOLDEN_HELLO_ACK = "444c48410200000012345678000000000000000000000000000000005286d325"


def _decision(**overrides) -> Decision:
    base = Decision(timestamp=0.0, mcs=5, bandwidth=20, tx_power_dBm=18,
                    k=8, n=14, depth=2, bitrate_kbps=12000)
    for k, v in overrides.items():
        setattr(base, k, v)
    return base


def test_decision_golden():
    assert encode(_decision(), sequence=1).hex() == GOLDEN_DECISION_1


def test_decision_signed_tx_power():
    pkt = encode(_decision(mcs=0, tx_power_dBm=-10, k=2, n=4, depth=1,
                           bitrate_kbps=2000), sequence=7)
    assert pkt.hex() == GOLDEN_DECISION_NEG
    assert pkt[18] == 0xF6   # two's-complement -10


def test_decision_max_values():
    pkt = encode(_decision(mcs=7, bandwidth=40, tx_power_dBm=30, k=8, n=16,
                           depth=3, bitrate_kbps=65000), sequence=0xFFFFFFFF)
    assert pkt.hex() == GOLDEN_DECISION_MAX


def test_decision_magic_and_version():
    pkt = encode(_decision(), sequence=1)
    assert pkt[:4] == b"DLK1"
    assert pkt[4] == 2


def test_hello_golden():
    pkt = encode_hello(Hello(generation_id=0xCAFEBABE, mtu_bytes=3994,
                             fps=60, applier_build_sha=0xDEADBEEF))
    assert pkt.hex() == GOLDEN_HELLO


def test_hello_ack_golden():
    pkt = encode_hello_ack(HelloAck(generation_id_echo=0x12345678))
    assert pkt.hex() == GOLDEN_HELLO_ACK
