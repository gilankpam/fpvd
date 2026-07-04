"""Micro-window folding: 10 tap micro-windows must equal one :8103 window."""

import pytest

from fpvdgs.dynlink.signals import MICROS_PER_WINDOW, SignalAggregator, micro_alpha
from fpvdgs.dynlink.stats_client import RxAnt, RxEvent
from fpvdgs.dynlink.tap_wire import TapLoss, TapMicro


def _ant(snr=25, rssi=-65, mcs=5):
    return RxAnt(
        ant=0x100,
        freq=5805,
        mcs=mcs,
        bw=20,
        pkt_recv=10,
        rssi_min=rssi - 5,
        rssi_avg=rssi,
        rssi_max=rssi + 5,
        snr_min=snr - 3,
        snr_avg=snr,
        snr_max=snr + 3,
    )


def _micro(ts_ms, data=9, fec=0, lost=0, out=10, ants=None):
    return TapMicro(
        seq=0,
        timestamp_ms=ts_ms,
        pkt_all=out,
        pkt_data=data,
        pkt_fec_rec=fec,
        pkt_lost=lost,
        pkt_out=out,
        rx_ant_stats=ants if ants is not None else [_ant()],
    )


def _event(ts, data=90, fec=0, lost=0, out=100, ants=None):
    return RxEvent(
        timestamp=ts,
        id="video rx",
        packets_window={"data": data, "fec_rec": fec, "lost": lost, "out": out},
        rx_ant_stats=ants if ants is not None else [_ant()],
    )


def test_micro_alpha_preserves_time_constant():
    a = micro_alpha(0.2)
    assert (1.0 - a) ** MICROS_PER_WINDOW == pytest.approx(0.8)


def test_ten_micros_equal_one_window():
    old = SignalAggregator()
    new = SignalAggregator()
    old.consume(_event(1.0, data=90, fec=3, lost=5, out=95))
    # same totals split across 10 micros: data 9x10=90, fec 3, lost 5,
    # out 5 in the first slot + 10x9 = 95
    for i in range(MICROS_PER_WINDOW):
        new.consume_micro(
            _micro(
                1000 + i * 10,
                data=9,
                fec=3 if i == 0 else 0,
                lost=5 if i == 0 else 0,
                out=5 if i == 0 else 10,
            ),
            now_s=1.0 + i * 0.01,
        )
    s_old, s_new = old.signals, new.signals
    assert s_new.residual_loss_w == pytest.approx(s_old.residual_loss_w)
    assert s_new.fec_work_rate_w == pytest.approx(s_old.fec_work_rate_w)
    assert s_new.packet_rate_w == pytest.approx(s_old.packet_rate_w)
    assert s_new.snr_w == s_old.snr_w
    assert s_new.mcs_w == s_old.mcs_w


def test_ewma_trajectory_matches_after_full_window():
    old = SignalAggregator()
    new = SignalAggregator()
    old.consume(_event(1.0, ants=[_ant(snr=25)]))
    old.consume(_event(1.1, ants=[_ant(snr=35)]))
    for i in range(MICROS_PER_WINDOW):
        new.consume_micro(_micro(1000 + i * 10, ants=[_ant(snr=25)]), now_s=1.0 + i * 0.01)
    for i in range(MICROS_PER_WINDOW):
        new.consume_micro(_micro(1100 + i * 10, ants=[_ant(snr=35)]), now_s=1.1 + i * 0.01)
    assert new.signals.snr == pytest.approx(old.signals.snr)
    assert new.signals.rssi_raw == pytest.approx(old.signals.rssi_raw)


def test_loss_record_raises_residual_immediately_no_double_count():
    agg = SignalAggregator()
    for i in range(MICROS_PER_WINDOW):
        agg.consume_micro(_micro(1000 + i * 10, lost=0, out=10), now_s=1.0 + i * 0.01)
    assert agg.signals.residual_loss_w == 0.0
    agg.consume_loss(
        TapLoss(seq=1, timestamp_ms=1101, lost_count=10, last_seq=0, new_seq=0), now_s=1.101
    )
    assert agg.signals.residual_loss_w == pytest.approx(10 / 110)
    # the next MICRO contains those same losses -> pending must clear
    agg.consume_micro(_micro(1110, lost=10, out=10), now_s=1.11)
    assert agg.signals.residual_loss_w == pytest.approx(10 / 110)


def test_micro_timestamp_uses_gs_clock():
    agg = SignalAggregator()
    agg.consume_micro(_micro(999999), now_s=42.5)
    assert agg.signals.timestamp == 42.5


def test_signals_has_tap_active_default_false():
    agg = SignalAggregator()
    assert agg.signals.tap_active is False


def test_reset_micro_window_clears_stale_state():
    agg = SignalAggregator()
    for i in range(MICROS_PER_WINDOW):
        agg.consume_micro(_micro(1000 + i * 10, lost=5, out=5), now_s=1.0 + i * 0.01)
    agg.consume_loss(
        TapLoss(seq=1, timestamp_ms=1101, lost_count=10, last_seq=0, new_seq=0), now_s=1.101
    )
    assert agg.signals.residual_loss_w > 0.0
    agg.reset_micro_window()
    agg.consume_micro(_micro(2000, lost=0, out=10), now_s=2.0)
    assert agg.signals.residual_loss_w == 0.0
