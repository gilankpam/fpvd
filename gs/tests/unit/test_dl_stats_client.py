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
