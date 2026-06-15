"""Reactive loss-demote hysteresis: require N consecutive breaching windows
(residual_loss_w >= video_demote_per) before a loss demote, mirroring the
starvation hysteresis. A single transient window must not demote."""
from __future__ import annotations

import json

from fpvdgs.dynlink.policy import Policy, PolicyConfig, SelectorConfig
from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.signals import Signals


def _cfg(tmp_path, **sel):
    return PolicyConfig(
        selector=SelectorConfig(**sel),
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path)),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def _sig(loss, rssi=-50.0, ts=1.0):
    return Signals(rssi=rssi, residual_loss_w=loss, fec_work=0.0,
                   link_starved_w=False, timestamp=ts)


def _records(tmp_path):
    files = sorted((tmp_path / "fl").glob("*.jsonl"))
    assert files, "expected a flight-log file"
    with open(files[-1]) as f:
        return [json.loads(line) for line in f if line.strip()]


def test_single_loss_window_does_not_demote(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    dec = p.tick(_sig(0.06, ts=1.0))
    assert dec.mcs == 5             # one breaching window -> held
    p.close()


def test_two_consecutive_loss_windows_demote(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.06, ts=1.0))
    dec = p.tick(_sig(0.06, ts=1.1))
    assert dec.mcs == 4             # sustained -> demote
    p.close()


def test_clean_window_resets_loss_count(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.06, ts=1.0))      # count 1
    p.tick(_sig(0.0, ts=1.1))       # clean -> reset
    dec = p.tick(_sig(0.06, ts=1.2))  # count 1 again
    assert dec.mcs == 5             # not sustained -> held
    p.close()


def test_sustained_loss_demotes_each_window_after_latch(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.06, ts=1.0))         # count 1, no demote
    d2 = p.tick(_sig(0.06, ts=1.1))    # count 2 -> demote to 4
    d3 = p.tick(_sig(0.06, ts=1.2))    # still breaching -> demote to 3
    assert d2.mcs == 4 and d3.mcs == 3
    p.close()


def test_loss_windows_one_is_single_window_behavior(tmp_path):
    """loss_windows=1 reduces to the legacy single-window demote (no regression
    for an operator who opts out of hysteresis)."""
    p = Policy(_cfg(tmp_path, loss_windows=1, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    dec = p.tick(_sig(0.06, ts=1.0))   # one breaching window -> immediate demote
    assert dec.mcs == 4
    p.close()


def test_loss_gated_true_when_suppressed(tmp_path):
    p = Policy(_cfg(tmp_path, loss_windows=2, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.06, ts=1.0))         # breach but not sustained
    p.close()
    last = _records(tmp_path)[-1]
    assert last["loss_gated"] is True
    assert "video_per_demote" not in last["reason"]


def test_loss_gated_false_when_clean(tmp_path):
    p = Policy(_cfg(tmp_path, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    p.tick(_sig(0.0, ts=1.0))
    p.close()
    assert _records(tmp_path)[-1]["loss_gated"] is False
