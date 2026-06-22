"""Reactive loss demote is single-window: a window breaching video_demote_per
(residual_loss_w >= threshold) demotes immediately. There is no consecutive-
window hysteresis (the lossWindows knob was removed — SNR-jump reacts fast)."""
from __future__ import annotations

import pytest

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


def test_single_breaching_window_demotes_immediately(tmp_path):
    p = Policy(_cfg(tmp_path, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    dec = p.tick(_sig(0.06, ts=1.0))   # one breaching window -> immediate demote
    assert dec.mcs == 4
    p.close()


def test_clean_window_does_not_demote(tmp_path):
    p = Policy(_cfg(tmp_path, video_demote_per=0.05), "m8812eu2")
    p.leading.state.current_mcs = 5
    dec = p.tick(_sig(0.0, ts=1.0))    # no loss -> held
    assert dec.mcs == 5
    p.close()


def test_selector_has_no_loss_windows_knob():
    with pytest.raises(TypeError):
        SelectorConfig(loss_windows=2)
    assert not hasattr(SelectorConfig(), "loss_windows")
