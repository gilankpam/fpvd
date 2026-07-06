"""Tests for the per-window signal aggregator + EWMA smoother (§3)."""

from __future__ import annotations

from fpvdgs.dynlink.signals import SignalAggregator
from fpvdgs.dynlink.signals import SignalAggregator as _Agg
from fpvdgs.dynlink.stats_client import RxAnt, RxEvent, SessionInfo
from fpvdgs.dynlink.stats_client import RxAnt as _RxAnt
from fpvdgs.dynlink.stats_client import RxEvent as _RxEvent


def _rx(
    ts: float,
    *,
    out: int = 0,
    lost: int = 0,
    fec_rec: int = 0,
    data: int = 0,
    bursts_rec: int = 0,
    holdoff: int = 0,
    late_deadline: int = 0,
    mcs: int = 0,
    ants: list[tuple[int, int, int, int]] | None = None,
) -> RxEvent:
    """Build a minimal RxEvent for tests. ants = [(rssi_min, rssi_avg, snr_min, snr_avg), ...]"""
    if ants is None:
        ants = [(-60, -58, 20, 22)]
    ant_stats = [
        RxAnt(
            ant=i,
            freq=5765,
            mcs=mcs,
            bw=20,
            pkt_recv=100,
            rssi_min=a[0],
            rssi_avg=a[1],
            rssi_max=a[1] + 2,
            snr_min=a[2],
            snr_avg=a[3],
            snr_max=a[3] + 2,
        )
        for i, a in enumerate(ants)
    ]
    session = SessionInfo(
        fec_type="VDM_RS",
        fec_k=8,
        fec_n=12,
        epoch=1,
        interleave_depth=1,
        contract_version=2,
    )
    return RxEvent(
        timestamp=ts,
        id="rx1",
        packets_window={
            "out": out,
            "lost": lost,
            "fec_rec": fec_rec,
            "data": data,
            "bursts_rec": bursts_rec,
            "holdoff": holdoff,
            "late_deadline": late_deadline,
        },
        rx_ant_stats=ant_stats,
        session=session,
    )


def test_residual_loss_from_lost_over_tx_primaries():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, out=99, lost=1, data=150))
    # 1 lost out of 100 tx primaries = 0.01
    assert s.residual_loss_w == 0.01


def test_residual_loss_zero_on_empty_window():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, out=0, lost=0))
    assert s.residual_loss_w == 0.0
    assert s.fec_work_rate_w == 0.0


def test_fec_work_rate():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, out=90, lost=0, fec_rec=5))
    assert s.fec_work_rate_w == 5 / 90


def test_rssi_min_is_min_across_antennas():
    agg = SignalAggregator()
    # three antennas — weakest should be picked
    s = agg.consume(_rx(0.1, ants=[(-55, -55, 20, 20), (-72, -70, 15, 17), (-60, -58, 18, 20)]))
    assert s.rssi_min_w == -72.0
    # rssi_avg_w is diversity-combined (simple average of rssi_avg)
    expected_avg = (-55 + -70 + -58) / 3
    assert abs(s.rssi_avg_w - expected_avg) < 1e-9


def test_ewma_alpha_rssi_matches_config():
    agg = SignalAggregator(ewma_alpha_rssi=0.2)
    s = agg.consume(_rx(0.1, ants=[(-60, -60, 20, 20)]))
    # First window bootstraps prev=None → EWMA returns raw value
    assert s.rssi_raw == -60.0
    # Next window at -80: 0.2*-80 + 0.8*-60 = -64
    s = agg.consume(_rx(0.2, ants=[(-80, -80, 10, 10)]))
    assert abs(s.rssi_raw - (-64.0)) < 1e-9


def test_residual_loss_is_not_smoothed():
    """§3: residual_loss must fire on raw per-window value."""
    agg = SignalAggregator()
    # Zero loss → spike → zero loss. residual_loss_w must track raw.
    agg.consume(_rx(0.1, out=100, lost=0))
    s = agg.consume(_rx(0.2, out=90, lost=10))
    assert s.residual_loss_w == 10 / 100
    s = agg.consume(_rx(0.3, out=100, lost=0))
    assert s.residual_loss_w == 0.0


def test_packet_rate_from_data_over_window():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, data=140))
    # data fragments per second at 100 ms window
    assert abs(s.packet_rate_w - 1400.0) < 1e-9


def test_burst_holdoff_late_smoothed_with_alpha_burst():
    agg = SignalAggregator(ewma_alpha_burst=0.1)
    s = agg.consume(_rx(0.1, bursts_rec=0))
    assert s.burst_rate == 0.0
    s = agg.consume(_rx(0.2, bursts_rec=10))
    # 0.1 * 100 + 0.9 * 0 = 10.0 (100 events/s)
    assert abs(s.burst_rate - 10.0) < 1e-9


def test_rssi_max_is_max_of_avgs_across_antennas():
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, ants=[(-55, -55, 20, 20), (-72, -70, 15, 17), (-60, -58, 18, 20)]))
    assert s.rssi_max_w == -55.0  # best antenna's avg


def test_ewma_smoothes_rssi_max_not_min():
    """Smoothed s.rssi_raw must track best-antenna avg (max-of-avgs), not
    the worst-antenna min — that was the survivor-bias bug."""
    agg = SignalAggregator(ewma_alpha_rssi=1.0)  # no smoothing
    s = agg.consume(_rx(0.1, ants=[(-55, -50, 25, 30), (-72, -70, 15, 17)]))
    assert s.rssi_raw == -50.0  # max(rssi_avg) — the best antenna


def test_link_starved_w_when_packet_rate_below_threshold():
    agg = SignalAggregator(starvation_threshold_pps=50.0)
    # Bootstrap a session.
    agg.consume(_rx(0.1, data=2000))
    # Now drop traffic. data=4 in 100 ms → packet_rate_w = 40 < 50 → starved.
    s = agg.consume(_rx(0.2, data=4))
    assert s.link_starved_w is True


def test_link_starved_false_when_no_session():
    """Pre-link windows must not flag starvation — only meaningful once
    we know the drone is supposed to be TXing."""
    agg = SignalAggregator(starvation_threshold_pps=50.0)
    # Build an RxEvent with no session.
    ev = _rx(0.1, data=0)
    ev.session = None
    s = agg.consume(ev)
    assert s.link_starved_w is False


def test_link_starved_false_when_packet_rate_high():
    agg = SignalAggregator(starvation_threshold_pps=50.0)
    agg.consume(_rx(0.1, data=2000))  # session bootstrapped
    s = agg.consume(_rx(0.2, data=200))  # 2000 pps - well above
    assert s.link_starved_w is False


def test_signals_has_no_unimplemented_snr_fields():
    from fpvdgs.dynlink.signals import Signals

    s = Signals()
    assert not hasattr(s, "snr_slope")
    assert not hasattr(s, "snr_max_w")


def test_rssi_normalized_by_received_mcs():
    """rssi_raw keeps the measured value (observability only now)."""
    agg = SignalAggregator(ewma_alpha_rssi=1.0)  # no smoothing → see one window
    s = agg.consume(_rx(0.1, mcs=5, ants=[(-70, -70, 10, 10)]))
    assert s.rssi_raw == -70.0  # measured, un-normalized
    assert s.rssi_max_w == -70.0


def test_rssi_ewma_removes_power_step_across_mcs_climb():
    """Fixed distance, promote MCS0→MCS5: drone power drops 29→19 so the
    measured RSSI drops ~10 dB. rssi_raw shows the step down (observability)."""
    agg = SignalAggregator(ewma_alpha_rssi=0.2)
    # Window 1: MCS0 @ raw -60
    s = agg.consume(_rx(0.1, mcs=0, ants=[(-60, -60, 20, 20)]))
    assert s.rssi_raw == -60.0
    # Window 2: MCS5 @ raw -70 (power dropped 10)
    s = agg.consume(_rx(0.2, mcs=5, ants=[(-70, -70, 12, 12)]))
    assert s.rssi_raw < -60.0  # raw EWMA steps down toward -70


def test_rssi_norm_uses_best_antenna_mcs():
    """rssi_max_w is the best antenna's rssi_avg. mcs_w is now picked from
    the best-SNR antenna (not best-RSSI anymore)."""
    agg = SignalAggregator(ewma_alpha_rssi=1.0)
    ev = _rx(0.1)
    ev.rx_ant_stats = [
        RxAnt(
            ant=0,
            freq=5765,
            mcs=5,
            bw=20,
            pkt_recv=100,
            rssi_min=-57,
            rssi_avg=-55,
            rssi_max=-53,
            snr_min=20,
            snr_avg=22,
            snr_max=24,
        ),
        RxAnt(
            ant=1,
            freq=5765,
            mcs=0,
            bw=20,
            pkt_recv=100,
            rssi_min=-72,
            rssi_avg=-70,
            rssi_max=-68,
            snr_min=10,
            snr_avg=12,
            snr_max=14,
        ),
    ]
    s = agg.consume(ev)
    assert s.rssi_max_w == -55.0  # best antenna's rssi_avg
    assert s.mcs_w == 5  # best-SNR antenna's MCS (ant 0: SNR 22)


# ── SNR + EVM aggregation (instrumentation for the flight log) ───────────────


def _evm_rxev(ants):
    return _RxEvent(
        timestamp=1.0,
        id="rx1",
        packets_window={"out": 100, "lost": 0},
        rx_ant_stats=ants,
        session=None,
    )


def _evm_ant(ant, rssi, snr, evm_min, evm_avg):
    return _RxAnt(
        ant=ant,
        freq=5660,
        mcs=4,
        bw=20,
        pkt_recv=100,
        rssi_min=rssi,
        rssi_avg=rssi,
        rssi_max=rssi,
        snr_min=snr,
        snr_avg=snr,
        snr_max=snr,
        evm_min=evm_min,
        evm_avg=evm_avg,
        evm_max=evm_avg,
    )


def test_evm_best_and_worst_dongle_stbc():
    # STBC MCS4: both paths of each dongle carry the real stream EVM.
    s = _Agg().consume(
        _evm_rxev(
            [
                _evm_ant(0, -60, 25, 81, 89),
                _evm_ant(1, -60, 25, 81, 89),  # dongle 0
                _evm_ant(256, -58, 26, 75, 80),
                _evm_ant(257, -58, 26, 75, 80),  # dongle 1
            ]
        )
    )
    assert s.evm_w == 89.0  # best dongle
    assert s.evm_lo_w == 80.0  # worst dongle
    assert s.evm_min_w == 75.0  # worst sample across dongles


def test_evm_ignores_sentinel_slots_nss1():
    # Nss=1 MCS0: path-B (odd ant) = -1 sentinels; only path-A is real.
    s = _Agg().consume(
        _evm_rxev(
            [
                _evm_ant(0, -60, 25, 87, 89),
                _evm_ant(1, -60, 25, -1, -1),  # dongle 0
                _evm_ant(256, -58, 26, 81, 82),
                _evm_ant(257, -58, 26, -1, -1),  # dongle 1
            ]
        )
    )
    assert s.evm_w == 89.0
    assert s.evm_lo_w == 82.0
    assert s.evm_min_w == 81.0


def test_evm_none_when_all_unmeasured():
    s = _Agg().consume(_evm_rxev([_evm_ant(0, -60, 25, -1, -1), _evm_ant(1, -60, 25, -1, -1)]))
    assert s.evm_w is None and s.evm_lo_w is None and s.evm_min_w is None


def test_snr_w_is_operating_antenna_snr():
    # best-RSSI antenna (ant 256, -55) is the operating point; log its SNR.
    s = _Agg().consume(_evm_rxev([_evm_ant(0, -60, 25, -1, -1), _evm_ant(256, -55, 30, -1, -1)]))
    assert s.snr_w == 30.0


def test_snr_is_raw_ewma_no_normalization():
    agg = SignalAggregator()  # no rssi_norm anymore
    # feed two windows at different received MCS; snr must NOT be shifted by any curve
    agg.consume(_rx(0.1, mcs=0, ants=[(-60, -60, 20, 20)]))
    first = agg.signals.snr
    agg.consume(_rx(0.2, mcs=4, ants=[(-60, -60, 20, 20)]))
    # identical raw SNR at a different MCS must stay identical (no per-MCS offset)
    assert agg.signals.snr == first
    assert first is not None and abs(first - 20.0) < 1e-6  # alpha seeds to first sample


def test_snr_none_before_any_antenna_data():
    from fpvdgs.dynlink.stats_client import RxEvent

    s = _Agg().consume(
        RxEvent(timestamp=1.0, id="rx", packets_window={}, rx_ant_stats=[], session=None)
    )
    assert s.snr is None


def test_reset_smoothed_clears_ewmas():
    agg = SignalAggregator()
    agg.signals.rssi = -50.0
    agg.signals.rssi_raw = -48.0
    agg.signals.snr = 30.0
    agg.reset_smoothed()
    assert agg.signals.rssi is None
    assert agg.signals.rssi_raw is None
    assert agg.signals.snr is None


def test_operating_antenna_picked_by_snr():
    # ant0: high RSSI, low SNR ; ant1: low RSSI, high SNR.
    agg = SignalAggregator()
    s = agg.consume(_rx(0.1, mcs=5, ants=[(-40, -40, 18, 18), (-55, -55, 30, 30)]))
    assert s.snr_w == 30.0  # best-SNR antenna, not best-RSSI (-40 -> 18)
    assert s.mcs_w == 5
    assert s.rssi_max_w == -40.0  # rssi_max_w stays the diversity max (observability)


def test_signals_probe_field_defaults_none():
    from fpvdgs.dynlink.signals import Signals

    s = Signals()
    assert s.probe is None
