"""Per-window signal aggregator and EWMA smoother (§3 "Derived metrics").

Each RxEvent carries one 100 ms window of counters. We compute the six
derived signals per window, then EWMA-smooth most of them. residual_loss
is intentionally *not* smoothed — one lost block in a window is already
a visible FPV glitch (§3).
"""

from __future__ import annotations

from dataclasses import dataclass, field

from .stats_client import RxEvent, SessionInfo

WINDOW_S = 0.1  # design cadence: log_interval = 100 ms (§3)


@dataclass(frozen=True)
class RssiNormConfig:
    """EIRP-normalization of the video-link RSSI by the drone's per-MCS TX
    power. The controller binds `tx_power_dbm_by_mcs` (and the derived
    `p_ref_dbm`) from the drone's /status curve at the connect event — the
    drone is the single source of truth. `enabled` is internal bind state, not
    a config knob: it is False (identity / raw RSSI) until a curve is bound,
    True while a valid drone curve is in effect."""

    enabled: bool = False
    p_ref_dbm: int = 0
    tx_power_dbm_by_mcs: tuple[int, ...] = ()


def normalize_rssi(rssi_raw: float | None, mcs: int | None, cfg: RssiNormConfig) -> float | None:
    """EIRP-normalize one RSSI reading: rssi_raw + (P_ref − curve[mcs]).
    Clamps mcs into the curve's index range. None-safe (returns rssi_raw
    when disabled, or when rssi_raw / mcs is None)."""
    if not cfg.enabled or not cfg.tx_power_dbm_by_mcs or rssi_raw is None or mcs is None:
        return rssi_raw
    n = len(cfg.tx_power_dbm_by_mcs)
    m = max(0, min(n - 1, int(mcs)))
    return rssi_raw + (cfg.p_ref_dbm - cfg.tx_power_dbm_by_mcs[m])


@dataclass
class Signals:
    """One tick's view of the controller inputs.

    Raw `_w` fields are per-100 ms-window; EWMA-smoothed inputs have no
    suffix. All optional fields are None until the first window arrives.
    """

    # Raw per-window
    rssi_min_w: float | None = None  # min across antennas of rssi_min
    rssi_avg_w: float | None = None  # diversity-combined estimate
    rssi_max_w: float | None = None  # max(rssi_avg) — best-antenna operating point
    mcs_w: int | None = None  # received MCS of the best antenna this window
    snr_w: float | None = None  # operating (best-SNR) antenna SNR (per-antenna diversity)
    # |EVM| in dB magnitude (lock_quality, uncapped, higher=better) is per
    # spatial STREAM, not per antenna: combined per dongle then across dongles.
    # None when no real EVM this window.
    evm_w: float | None = None  # best dongle (operating modulation quality)
    evm_lo_w: float | None = None  # worst dongle (diversity floor)
    evm_min_w: float | None = None  # worst per-window sample across dongles
    residual_loss_w: float = 0.0  # used raw — no smoothing (§3)
    fec_work_rate_w: float = 0.0
    packet_rate_w: float = 0.0  # fragments / sec
    burst_rate_w: float = 0.0  # events / sec
    holdoff_rate_w: float = 0.0
    late_rate_w: float = 0.0
    # Starvation flag: link is up (session known) but data fragments
    # have dropped to near zero — distinct from "healthy idle" because
    # a session implies the drone is still TXing.
    link_starved_w: bool = False

    # EWMA-smoothed controller inputs
    rssi: float | None = (
        None  # DEPRECATED control axis — always None now; rssi_raw is the logged observability
    )
    rssi_raw: float | None = None  # EWMA of the un-normalized RSSI (observability)
    snr: float | None = None  # EWMA of EIRP-normalized SNR (cross-rung control axis)
    fec_work: float = 0.0
    burst_rate: float = 0.0
    holdoff_rate: float = 0.0
    late_rate: float = 0.0

    # Last session seen (for knowing current k, n, depth).
    session: SessionInfo | None = None

    timestamp: float = 0.0
    windows_seen: int = 0
    ant_count: int = 0


def _ewma(prev: float | None, new: float, alpha: float) -> float:
    if prev is None:
        return new
    return alpha * new + (1.0 - alpha) * prev


@dataclass
class SignalAggregator:
    """Folds each RxEvent into the running Signals snapshot.

    Ownership: one aggregator per running service. Call `consume(ev)`
    on every RxEvent; read `.signals` at controller tick time.
    """

    ewma_alpha_rssi: float = 0.2
    ewma_alpha_fec: float = 0.2
    ewma_alpha_burst: float = 0.1
    # link_starved_w threshold: data fragments/sec below this counts as
    # starved. Compared per-window against packet_rate_w. Default 50 pps
    # is well below an FPV video stream's nominal ~700-1500 pps but well
    # above background noise from a stalled stream.
    starvation_threshold_pps: float = 50.0

    rssi_norm: RssiNormConfig = field(default_factory=RssiNormConfig)

    signals: Signals = field(default_factory=Signals)

    def update_session(self, session: SessionInfo) -> None:
        self.signals.session = session

    def reset_smoothed_rssi(self) -> None:
        """Drop the smoothed RSSI/SNR EWMAs so they restart clean. Called
        when the TX-power curve is (re)bound from the drone: samples taken
        under a different (or identity) normalization must not bleed into
        the freshly-normalized series."""
        self.signals.rssi = None
        self.signals.rssi_raw = None
        self.signals.snr = None

    def consume(self, ev: RxEvent) -> Signals:
        s = self.signals
        s.timestamp = ev.timestamp
        s.windows_seen += 1
        if ev.session is not None:
            s.session = ev.session

        # --- Packet-derived signals (§3) --------------------------------
        p = ev.packets_window
        out = p.get("out", 0)
        lost = p.get("lost", 0)
        fec_rec = p.get("fec_rec", 0)
        data = p.get("data", 0)
        bursts_rec = p.get("bursts_rec", 0)
        holdoff = p.get("holdoff", 0)
        late = p.get("late_deadline", 0)

        tx_primaries = out + lost  # TX-emitted primaries this window
        if tx_primaries > 0:
            s.residual_loss_w = lost / tx_primaries
            s.fec_work_rate_w = fec_rec / tx_primaries
        else:
            s.residual_loss_w = 0.0
            s.fec_work_rate_w = 0.0

        s.packet_rate_w = data / WINDOW_S
        s.burst_rate_w = bursts_rec / WINDOW_S
        s.holdoff_rate_w = holdoff / WINDOW_S
        s.late_rate_w = late / WINDOW_S

        # --- Antenna-derived signals (§3) ------------------------------
        if ev.rx_ant_stats:
            rssi_mins = [a.rssi_min for a in ev.rx_ant_stats]
            rssi_avgs = [a.rssi_avg for a in ev.rx_ant_stats]
            s.rssi_min_w = float(min(rssi_mins))
            s.rssi_avg_w = float(sum(rssi_avgs) / len(rssi_avgs))
            s.rssi_max_w = float(max(rssi_avgs))  # max RSSI across antennas (observability)
            # Operating antenna = best SNR (the sole control axis). mcs_w/snr_w
            # follow it; rssi_max_w stays the diversity max for the log.
            best_ant = max(ev.rx_ant_stats, key=lambda a: a.snr_avg)
            s.mcs_w = int(best_ant.mcs)
            s.ant_count = len(ev.rx_ant_stats)
            s.snr_w = float(best_ant.snr_avg)
            # EVM is per-STREAM, not per-antenna. Group by dongle (ant>>8);
            # a real slot has evm_avg > 0 (absent/2nd-stream slots report -1;
            # 0 = unmeasurable). The > 0 filter holds for the dB magnitude too:
            # a real lock reads positive dB, sentinels are -1/0. Per dongle:
            # stream EVM = max(evm_avg over real slots), worst sample =
            # min(evm_min). Across dongles: best = max, floor = min; worst
            # sample = min over all real slots. (Averaging raw per-"ant" EVM
            # would corrupt it — sentinels and STBC duplicates skew it.)
            d_avg: dict[int, float] = {}
            d_min: dict[int, float] = {}
            for a in ev.rx_ant_stats:
                if a.evm_avg > 0:
                    d = a.ant >> 8
                    d_avg[d] = max(d_avg.get(d, float(a.evm_avg)), float(a.evm_avg))
                    d_min[d] = min(d_min.get(d, float(a.evm_min)), float(a.evm_min))
            if d_avg:
                s.evm_w = max(d_avg.values())
                s.evm_lo_w = min(d_avg.values())
                s.evm_min_w = min(d_min.values())
        # If no antenna lines this window, keep prior values — don't
        # reset; the RSSI operating point doesn't vanish just because
        # no fragments arrived.

        # --- Starvation flag (post-blackout detection) -----------------
        # Only meaningful once we've seen a session — otherwise we'd
        # flag every pre-link tick. Bypasses the survivor-bias trap of
        # rssi because it watches packet_rate, not signal quality.
        s.link_starved_w = s.session is not None and s.packet_rate_w < self.starvation_threshold_pps

        # --- EWMA smoothing (§3) ---------------------------------------
        # The leading loop runs on SNR (best-SNR antenna, normalized below);
        # rssi_raw is observability only, smoothed from the best-antenna
        # aggregation max(rssi_avg) — what the diversity receiver decodes
        # against (and what the OSD shows), not min(rssi_min) which tracks the
        # weakest antenna and misses best-antenna degradation.
        if s.rssi_max_w is not None:
            # RSSI is observability-only now: smooth the raw value for the log,
            # but it no longer feeds control (SNR is the sole control axis).
            s.rssi_raw = _ewma(s.rssi_raw, s.rssi_max_w, self.ewma_alpha_rssi)
            # SNR scales 1:1 with our per-MCS TX power (noise unchanged), so
            # EIRP-normalize it by the received MCS BEFORE smoothing — the curve
            # mirror coupling lives here, not in RSSI.
            if s.snr_w is not None:
                snr_norm_w = normalize_rssi(s.snr_w, s.mcs_w, self.rssi_norm)
                s.snr = _ewma(s.snr, snr_norm_w, self.ewma_alpha_rssi)

        s.fec_work = _ewma(s.fec_work, s.fec_work_rate_w, self.ewma_alpha_fec)
        s.burst_rate = _ewma(s.burst_rate, s.burst_rate_w, self.ewma_alpha_burst)
        s.holdoff_rate = _ewma(s.holdoff_rate, s.holdoff_rate_w, self.ewma_alpha_burst)
        s.late_rate = _ewma(s.late_rate, s.late_rate_w, self.ewma_alpha_burst)

        return s
