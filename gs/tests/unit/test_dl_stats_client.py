"""Tests for fpvdgs.dynlink.stats_client — contract version handling."""
import pytest

from fpvdgs.dynlink.stats_client import (
    ContractVersionError,
    SessionEvent,
    parse_record,
)


def _session_raw(contract_version, fec_type="swfec", fec_k=50, fec_n=30):
    return {
        "type": "new_session", "timestamp": 1.0, "id": "video rx",
        "fec_type": fec_type, "fec_k": fec_k, "fec_n": fec_n,
        "epoch": 1, "interleave_depth": 1,
        "contract_version": contract_version,
    }


def test_contract_v3_swfec_session_accepted():
    ev = parse_record(_session_raw(3))
    assert isinstance(ev, SessionEvent)
    assert ev.session.fec_type == "swfec"
    assert ev.session.fec_k == 50      # overhead_pct in swfec sessions
    assert ev.session.fec_n == 30      # deadline_ms in swfec sessions
    assert ev.session.contract_version == 3


def test_contract_v3_rx_record_accepted():
    raw = {
        "type": "rx", "timestamp": 1.0, "id": "video rx",
        "packets": {"out": [100, 100], "lost": [0, 0]},
        "rx_ant_stats": [],
        "session": _session_raw(3),
    }
    ev = parse_record(raw)
    assert ev.session.contract_version == 3


def test_unknown_contract_version_still_rejected():
    with pytest.raises(ContractVersionError):
        parse_record(_session_raw(4))


def _rx_with_ant(ant):
    return {"type": "rx", "timestamp": 1.0, "id": "video rx",
            "packets": {}, "rx_ant_stats": [ant]}


def test_rx_ant_parses_evm():
    a = parse_record(_rx_with_ant({
        "ant": 0, "freq": 5660, "mcs": 4, "bw": 20, "pkt_recv": 100,
        "rssi_min": -60, "rssi_avg": -60, "rssi_max": -60,
        "snr_min": 23, "snr_avg": 25, "snr_max": 27,
        "evm_min": 81, "evm_avg": 89, "evm_max": 93})).rx_ant_stats[0]
    assert (a.evm_min, a.evm_avg, a.evm_max) == (81, 89, 93)


def test_rx_ant_evm_defaults_when_absent():
    # older wfb_rx without EVM in the tuple -> -1 sentinel, no crash
    a = parse_record(_rx_with_ant({
        "ant": 0, "freq": 5660, "mcs": 4, "bw": 20, "pkt_recv": 100,
        "rssi_min": -60, "rssi_avg": -60, "rssi_max": -60,
        "snr_min": 23, "snr_avg": 25, "snr_max": 27})).rx_ant_stats[0]
    assert (a.evm_min, a.evm_avg, a.evm_max) == (-1, -1, -1)
