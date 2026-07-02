"""Policy engine — SNR-driven MCS selector (§4).

Runs at 10 Hz, one tick per RxEvent. Pure function of
(Signals snapshot, internal hysteresis state). Emits a `{mcs}`-only
Decision on every tick.

Phase 3b: the drone computes its own bitrate / FEC / depth / tx_power
locally, so the GS no longer composes any of that. The selector decides
MCS via three promote routes (snap-back, knee-gated, explore) plus a
reactive demote with failure classification.
"""

from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field

from .decision import Decision
from .flightlog import FlightLog, FlightLogConfig
from .learned_prior import UNBOUND_KEY, LearnedPrior, LearnedPriorConfig, lsq_slope
from .signals import Signals

log = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Config dataclasses for the probe-driven selector + smoothing/learned-prior/flightlog.
# ------------------------------------------------------------------


@dataclass
class SelectorConfig:
    """SNR-driven promote (three routes) + reactive demote + timing/cadence.

    Promote routes (in priority order):
      1. snap-back: a recently-confirmed rung whose operating SNR has nearly
         recovered — re-entered at the fast rate limit, no dwell/knee gate.
      2. knee-gated: clean dwell + confident knee headroom (SNR above knee+margin).
      3. explore: cold knee on the target rung — promotes once as tuition.
    Demote: a caller-hysteresis-gated loss breach or a Channel-B emergency
    (fec/starvation). Every loss-demote is classified (fade/flap/burst).
    starvation_windows is the consecutive-starved-window count before
    link_starved feeds the emergency demote.
    """

    # Reactive demote
    video_demote_per: float = 0.05
    emergency_fec_pressure: float = 0.80
    # MCS bound
    max_mcs: int = 7
    # Timing/cadence (promote cooldowns; demotes bypass them)
    hold_modes_down_ms: int = 2000
    min_between_changes_ms: int = 200
    # Total-blackout failsafe: consecutive starved windows before link_starved
    # feeds the emergency demote (10 Hz → 5 windows = 0.5 s).
    starvation_windows: int = 5
    # Shared demote cooldown: minimum windows between consecutive demotes
    # (any path). After a demote, the next N windows are frozen so the
    # lower rung's true loss reading returns over the GS->drone->GS
    # application lag before deciding to step down again. Enforces <= 1
    # demote per N windows across reactive/predict/snr/emergency.
    demote_cooldown_windows: int = 3
    # Proactive SNR demote: consecutive ticks the current rung must be
    # confidently unviable at the live SNR before demoting (debounce; snr is already EWMA'd).
    snr_demote_debounce: int = 2
    # SNR-knee hysteresis (dB). snr_promote_margin_db gates the knee-gated
    # promote route (route 2 headroom test). snr_demote_margin_db gates the
    # proactive SNR demote. The asymmetric margins keep a stable dead-band.
    snr_promote_margin_db: float = 1.0
    snr_demote_margin_db: float = 1.5
    # Flap-damper: per-rung escalating promote back-off on observed
    # promote->loss->demote flapping. See 2026-07-02 spec.
    flap_base_backoff_ms: int = 2000
    flap_backoff_mult: float = 2.0
    flap_backoff_cap_ms: int = 30000
    flap_reset_clean_dwell_ticks: int = 30
    # --- Probe-less promote (2026-07-02 spec) ---
    # Clean dwell at the current rung before a knee/explore promote.
    promote_dwell_ticks: int = 30
    # Slope gate for knee/explore promotes (dB/tick; small negative tolerance).
    promote_slope_min: float = -0.05
    # Failure-signature classifier: promote probation window; a loss-demote
    # from a rung entered by promote within this window = genuine flap.
    trial_window_ms: int = 10000
    # Fade signature: raw snr_w this far below the (lagging) snr_ewma.
    collapse_delta_db: float = 4.0
    # Damper recovery: early suppression lift when snr_ewma exceeds the SNR
    # recorded at the rung's last flap by this margin; one flap level forgiven
    # per flap_decay_ms without a new flap on the rung.
    flap_snr_release_db: float = 3.0
    flap_decay_ms: int = 60000
    # Snap-back: a rung confirmed within confirm_ttl_ms is re-entered at the
    # fast rate limit once snr_ewma recovers to within
    # snapback_recover_margin_db of its confirmed operating SNR.
    snapback_recover_margin_db: float = 3.0
    confirm_ttl_ms: int = 60000


@dataclass
class PolicyConfig:
    selector: SelectorConfig = field(default_factory=SelectorConfig)
    learned_prior: LearnedPriorConfig = field(default_factory=LearnedPriorConfig)
    flightlog: FlightLogConfig = field(default_factory=FlightLogConfig)
    link_width: int = 20  # channel width (10/20/40); logged per tick for analysis


# ------------------------------------------------------------------
# Leading selector — dual-gate ProfileSelector (alink_gs port).
# ------------------------------------------------------------------


@dataclass
class LeadingState:
    current_mcs: int  # currently selected MCS
    # Initialise the timing anchors well in the past so the first
    # decision after boot doesn't get gated by min_between_changes_ms
    # / hold_modes_down_ms. In production ts_ms is a wall-clock value
    # in the trillions; this just guarantees the same "long-elapsed"
    # condition holds in tests where ts_ms starts near zero.
    last_change_time_ms: float = -1.0e9
    last_mcs_change_time_ms: float = -1.0e9


class LeadingSelector:
    """MCS selector — probe-less promote (three routes) + reactive demote.

    The selector decides MCS only; FEC ladder, bitrate, and depth are
    computed downstream by the drone.

    Promote routes (priority order):
      1. snap-back: recently-confirmed rung with recovered SNR, slope >= 0.
      2. knee-gated: clean dwell + confident knee headroom (SNR above knee).
      3. explore: cold knee on the target rung (one-shot tuition).

    Demote (reactive, one-step, cooldown-gated): a caller-hysteresis-gated
    loss breach (`loss_demote`) or a Channel-B emergency (fec_pressure or
    link_starved) steps exactly one rung down, gated by `can_demote`.
    Every loss-demote is classified (fade/flap/burst) for knee teaching.
    """

    def __init__(self, cfg: SelectorConfig):
        self.cfg = cfg
        cap = int(cfg.max_mcs)
        if cap < 0:
            raise ValueError(f"max_mcs={cfg.max_mcs} excludes every MCS")
        self._cap_mcs = cap
        start_mcs = 1
        if start_mcs > cap:
            start_mcs = cap
        self.state = LeadingState(current_mcs=start_mcs)
        self._reasons: list[str] = []
        # Clean consecutive ticks at the current rung (promote dwell, damper
        # clear, confirmation). Resets on any rung change or dirty tick.
        self._clean_dwell = 0
        # Flap-damper (per rung).
        self._flap_level: dict[int, int] = {}
        self._suppress_until_ms: dict[int, float] = {}
        self._snr_at_last_flap: dict[int, float] = {}
        self._last_flap_ms: dict[int, float] = {}
        self._promote_suppressed = False
        # Trial: rung entered by promote, on probation for trial_window_ms.
        self._trial_rung: int | None = None
        self._trial_until_ms = 0.0
        # Snap-back memory: rung -> (confirmed-until ts, SNR it ran clean at).
        self._confirmed_until_ms: dict[int, float] = {}
        self._confirmed_snr: dict[int, float] = {}
        self._snapback_tgt: int | None = None
        # Classified loss-demote this tick: (rung, "fade"|"flap", snr_sample).
        # Policy.tick feeds fade/flap samples to LearnedPrior.teach_failure.
        self.last_fail: tuple[int, str, float] | None = None

    # ---- helpers ----

    def _emergency_active(self, fec_pressure: float, link_starved: bool) -> bool:
        return fec_pressure >= self.cfg.emergency_fec_pressure or link_starved

    def _flap_backoff_ms(self, level: int) -> int:
        return min(
            int(self.cfg.flap_base_backoff_ms * (self.cfg.flap_backoff_mult ** min(level - 1, 40))),
            self.cfg.flap_backoff_cap_ms,
        )

    def _effective_flap_level(self, rung: int, ts_ms: float) -> int:
        """Stored level minus time decay (one level per flap_decay_ms quiet)."""
        lvl = self._flap_level.get(rung, 0)
        last = self._last_flap_ms.get(rung)
        if lvl and last is not None and self.cfg.flap_decay_ms > 0:
            lvl = max(0, lvl - int((ts_ms - last) / self.cfg.flap_decay_ms))
        return lvl

    def _charge_flap(self, rung: int, snr_ewma: float | None, ts_ms: float) -> None:
        lvl = self._effective_flap_level(rung, ts_ms) + 1
        self._flap_level[rung] = lvl
        self._last_flap_ms[rung] = ts_ms
        self._suppress_until_ms[rung] = ts_ms + self._flap_backoff_ms(lvl)
        if snr_ewma is not None:
            self._snr_at_last_flap[rung] = snr_ewma

    def _suppressed(self, rung: int, snr_ewma: float | None, ts_ms: float) -> bool:
        """Damper window live for `rung`? SNR release: conditions provably
        better than at the last flap lift the window early (level kept)."""
        if ts_ms >= self._suppress_until_ms.get(rung, 0):
            return False
        laf = self._snr_at_last_flap.get(rung)
        if (
            laf is not None
            and snr_ewma is not None
            and snr_ewma >= laf + self.cfg.flap_snr_release_db
        ):
            return False
        return True

    def _classify(self, rung: int, snr_ewma, snr_w, ts_ms: float) -> str:
        """Failure signature of a loss-demote from `rung` (spec table).
        fade: raw window SNR collapsed away from the lagging EWMA (obstruction).
        flap: SNR steady and the rung was on promote-probation (margin shortfall).
        burst: SNR steady at a settled rung (interference) — teaches nothing."""
        if (
            snr_ewma is not None
            and snr_w is not None
            and snr_w <= snr_ewma - self.cfg.collapse_delta_db
        ):
            return "fade"
        if rung == self._trial_rung and ts_ms < self._trial_until_ms:
            return "flap"
        return "burst"

    def _snapback_target(self, snr_ewma: float, ts_ms: float) -> int | None:
        """Highest recently-confirmed rung whose operating SNR is nearly back."""
        best = None
        for r, until in self._confirmed_until_ms.items():
            if until < ts_ms or r > self._cap_mcs:
                continue
            if snr_ewma >= self._confirmed_snr[r] - self.cfg.snapback_recover_margin_db:
                if best is None or r > best:
                    best = r
        return best

    # ---- main entry ----

    def select(
        self,
        *,
        snr_ewma,
        snr_w,
        slope,
        loss_rate: float,
        loss_demote: bool = False,
        target_confident: bool = False,
        target_blocked: bool = False,
        fec_pressure: float,
        link_starved: bool,
        can_demote: bool = True,
        ts_ms: float,
    ) -> tuple[int, bool]:
        """Probe-less selector: three-route promote + reactive demote.

        Returns (mcs, changed).

        Demote is reactive and one-step, gated by can_demote (Policy's shared
        cooldown). Every loss-demote is classified (fade/flap/burst): fade
        reports a raw-SNR knee sample, flap charges the damper and reports an
        EWMA knee sample, burst does neither (last_fail carries the sample to
        Policy). Promote routes, in order: snap-back (recently-confirmed rung,
        SNR recovered, slope >= 0 — bypasses dwell/knee/hold, never the
        damper), knee-gated climb (clean dwell + headroom over a confident
        knee), explore (cold knee — once-per-rung tuition; its first failure
        plants the knee and self-converts the route to knee-gated)."""
        st = self.state
        prev = st.current_mcs
        reasons: list[str] = []
        self.last_fail = None
        self._snapback_tgt = None

        def commit(new_mcs: int, why: str) -> None:
            new_mcs = max(0, min(new_mcs, self._cap_mcs))
            if new_mcs != st.current_mcs:
                st.current_mcs = new_mcs
                st.last_change_time_ms = ts_ms
                st.last_mcs_change_time_ms = ts_ms
                self._clean_dwell = 0
                reasons.append(why)

        # --- Demote (reactive, one step, cooldown-gated) ---
        emergency = loss_demote or self._emergency_active(fec_pressure, link_starved)
        if emergency:
            if can_demote:
                if loss_demote:
                    klass = self._classify(prev, snr_ewma, snr_w, ts_ms)
                    commit(prev - 1, f"video_per_demote loss={loss_rate:.3f} class={klass}")
                    if st.current_mcs != prev:
                        if klass == "fade" and snr_w is not None:
                            self.last_fail = (prev, "fade", float(snr_w))
                        elif klass == "flap" and snr_ewma is not None:
                            self.last_fail = (prev, "flap", float(snr_ewma))
                            self._charge_flap(prev, snr_ewma, ts_ms)
                else:
                    commit(
                        prev - 1,
                        f"emergency fec={fec_pressure:.3f} starved={link_starved}",
                    )
            if st.current_mcs != prev:
                self._trial_rung = None  # a demote ends any live trial
            self._clean_dwell = 0  # dirty tick even when the cooldown holds
            self._reasons = reasons
            return (st.current_mcs, st.current_mcs != prev)

        # --- Clean-tick bookkeeping: dwell, damper clear, confirmation ---
        self._clean_dwell += 1
        cur = st.current_mcs
        if self._clean_dwell >= self.cfg.flap_reset_clean_dwell_ticks:
            self._flap_level.pop(cur, None)
            self._suppress_until_ms.pop(cur, None)
            self._snr_at_last_flap.pop(cur, None)
            self._last_flap_ms.pop(cur, None)
        if self._clean_dwell >= self.cfg.promote_dwell_ticks and snr_ewma is not None:
            self._confirmed_until_ms[cur] = ts_ms + self.cfg.confirm_ttl_ms
            self._confirmed_snr[cur] = float(snr_ewma)
        if self._trial_rung is not None and ts_ms >= self._trial_until_ms:
            self._trial_rung = None  # probation survived

        # --- Promote: snap-back -> knee-gated -> explore ---
        target = cur + 1
        if snr_ewma is None or target > self._cap_mcs:
            self._promote_suppressed = False
            self._reasons = reasons
            return st.current_mcs, False

        self._promote_suppressed = self._suppressed(target, snr_ewma, ts_ms)
        within_rate = (ts_ms - st.last_change_time_ms) < self.cfg.min_between_changes_ms
        within_hold = (ts_ms - st.last_change_time_ms) < self.cfg.hold_modes_down_ms
        slope = 0.0 if slope is None else slope

        if not within_rate and not self._promote_suppressed:
            sb = self._snapback_target(float(snr_ewma), ts_ms)
            self._snapback_tgt = sb
            if sb is not None and sb > cur and slope >= 0.0:
                # Route 1: return to recently-proven altitude at the fast rate
                # limit — no dwell, no knee gate, no hold (fades never charged
                # the damper, so frees snap back freely).
                commit(target, f"snapback_promote tgt={sb}")
            elif (
                not within_hold
                and self._clean_dwell >= self.cfg.promote_dwell_ticks
                and slope >= self.cfg.promote_slope_min
            ):
                if target_confident and not target_blocked:
                    commit(target, "knee_promote")  # Route 2: earned headroom
                elif not target_confident:
                    commit(target, "explore_promote")  # Route 3: cold = tuition

        if st.current_mcs > prev:
            self._trial_rung = st.current_mcs
            self._trial_until_ms = ts_ms + self.cfg.trial_window_ms

        self._reasons = reasons
        return st.current_mcs, (st.current_mcs != prev)

    @property
    def reasons(self) -> list[str]:
        return list(self._reasons)


# ------------------------------------------------------------------
# Top-level policy: runs the selector, emits a {mcs}-only Decision.
# ------------------------------------------------------------------


class Policy:
    """Runs the SNR-driven selector and emits the `{mcs}`-only Decision.

    The selector promotes without a probe: three routes (snap-back,
    knee-gated, explore) replace the probe-debounce gate. Promotes are
    gated by SNR dwell and the per-rung learned knee; the live probe is
    no longer consulted for promotes.
    """

    def __init__(
        self,
        cfg: PolicyConfig,
        profile_name: str = UNBOUND_KEY,
    ) -> None:
        self.cfg = cfg
        self.profile_name = profile_name
        self.leading = LeadingSelector(cfg.selector)
        # Per-window link_starved_w can flicker on brief packet-rate
        # dips inside an otherwise-healthy bursty stream. Require N
        # consecutive starved windows before treating the link as
        # actually starved for the selector's emergency channel —
        # loss/FEC pressure remain direct triggers (those are real
        # glitches). At 10 Hz, starvation_windows=5 = 0.5 s of below-
        # threshold packet rate before declaring blackout.
        self._starvation_count: int = 0
        # Knee-prior learning gate: only ingest once the operating rung has
        # been unchanged for settle_ticks (loss from the last change drained).
        self._ticks_at_mcs = 0
        self._last_ingest_mcs: int | None = None
        # Learned per-card prior (always-on), keyed by the drone-reported
        # adapter id (radio.adapterId); GS-local, the live probe stays authoritative.
        self.learned_prior = LearnedPrior(profile_name, cfg.learned_prior)
        self._snr_window: deque[float] = deque(
            maxlen=cfg.learned_prior.predictive_slope_window_ticks
        )
        self._predict_demote_count = 0
        self._snr_demote_count = 0
        # Shared demote cooldown counter: ticks since the last demote. Init to
        # the cooldown so the first demote after boot is never blocked. Reset to
        # 0 on any committed demote (any path); incremented each tick.
        self._windows_since_demote = cfg.selector.demote_cooldown_windows
        self.flightlog = FlightLog(cfg.flightlog)

    def tick(self, signals: Signals) -> Decision:
        ts_ms = signals.timestamp * 1000.0 if signals.timestamp else 0.0

        # Shared demote cooldown: advance the counter, snapshot the rung we
        # started this tick at, and decide whether any demote may fire.
        self._windows_since_demote += 1
        cooldown_ok = self._windows_since_demote >= self.cfg.selector.demote_cooldown_windows
        start_mcs = self.leading.state.current_mcs

        # Starvation hysteresis: per-tick link_starved_w flickers on
        # brief packet-rate dips in bursty video. Require N consecutive
        # starved windows before the selector treats it as emergency.
        if signals.link_starved_w:
            self._starvation_count += 1
        else:
            self._starvation_count = 0
        sustained_starved = self._starvation_count >= self.cfg.selector.starvation_windows

        # Reactive loss demote: single-window. residual_loss_w is raw, but the
        # reactive demote is an SNR-jump that lands on the right rung in one move
        # (nothing to cascade), so react on the first breaching window.
        sustained_loss = signals.residual_loss_w >= self.cfg.selector.video_demote_per

        # Predictive demote (down-only, confidence-gated, debounced). If the SNR
        # prior says the CURRENT rung is unviable at the PROJECTED SNR, pre-demote
        # one rung ahead of the reactive path. A cold/unlearned rung is explorable
        # so a clean ladder never collapses to MCS0 on a no-loss fade.
        predict_reason = ""
        predict_gated = False
        if signals.snr is None:
            slope = None
        else:
            self._snr_window.append(signals.snr)
            slope = lsq_slope(self._snr_window)
        if signals.snr is not None:
            cur = self.leading.state.current_mcs
            projected_drop = -slope * self.cfg.learned_prior.predictive_horizon_ticks
            cur_unviable = self.learned_prior.snr_predictive_rung_unviable(
                signals.snr,
                slope,
                cur,
                margin=self.cfg.learned_prior.predictive_demote_margin_db,
            )
            if cur_unviable:
                if projected_drop >= self.cfg.learned_prior.predictive_min_drop_db:
                    self._predict_demote_count += 1
                    if (
                        self._predict_demote_count
                        >= self.cfg.learned_prior.predictive_debounce_windows
                        and cooldown_ok
                    ):
                        tgt = max(cur - 1, 0)
                        if tgt < cur:
                            self.leading.state.current_mcs = tgt
                            self.leading._clean_dwell = 0
                            self.leading._trial_rung = None
                            predict_reason = f"predict_demote mcs{cur}->{tgt}"
                else:
                    predict_gated = True
                    self._predict_demote_count = 0
            else:
                self._predict_demote_count = 0

        # Proactive SNR demote (down-only, debounced, cooldown-gated).
        snr_demote_reason = ""
        cur_snr = self.leading.state.current_mcs
        if self.learned_prior.snr_rung_unviable(
            cur_snr, signals.snr, margin=self.cfg.selector.snr_demote_margin_db
        ):
            self._snr_demote_count += 1
            if (
                self._snr_demote_count >= self.cfg.selector.snr_demote_debounce
                and cooldown_ok
                and self.leading.state.current_mcs == start_mcs
            ):
                tgt = max(cur_snr - 1, 0)
                if tgt < cur_snr:
                    self.leading.state.current_mcs = tgt
                    self.leading._clean_dwell = 0
                    self.leading._trial_rung = None
                    snr_demote_reason = f"snr_demote mcs{cur_snr}->{tgt}"
        else:
            self._snr_demote_count = 0

        # Promote gates for the knee-driven selector.
        # target_blocked (route 2's headroom test): the failure knee for the
        # target rung is confident AND the live SNR has less than
        # snr_promote_margin_db dB of headroom ABOVE the knee. The negative
        # margin passed to snr_rung_unviable (which tests `snr < knee - margin`)
        # flips the direction: `snr < knee - (-m)` = `snr < knee + m`, so
        # promote requires SNR ≥ knee + m. The demote path uses a POSITIVE margin
        # (`snr < knee - m`) — only demote when clearly below the knee. The two
        # asymmetric margins form a stable dead-band.
        # target_confident splits route 2 (knee-gated) from route 3 (explore).
        promote_target = self.leading.state.current_mcs + 1
        promote_blocked = self.learned_prior.snr_rung_unviable(
            promote_target, signals.snr, margin=-self.cfg.selector.snr_promote_margin_db
        )
        target_confident = self.learned_prior.snr_rung_confident(promote_target)

        new_mcs, _changed = self.leading.select(
            snr_ewma=signals.snr,
            snr_w=signals.snr_w,
            slope=slope,
            loss_rate=signals.residual_loss_w,
            loss_demote=sustained_loss,
            target_confident=target_confident,
            target_blocked=promote_blocked,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            can_demote=cooldown_ok and self.leading.state.current_mcs == start_mcs,
            ts_ms=ts_ms,
        )

        if new_mcs < start_mcs:
            self._windows_since_demote = 0

        # Event-driven knee teaching: a classified fade/flap loss-demote is a
        # dirty sample for the demoted-FROM rung (correct attribution — the old
        # settle-gated dirty ingest attributed it to the post-demote rung and
        # discarded it; 2026-07-02 spec).
        fail = self.leading.last_fail
        if fail is not None:
            self.learned_prior.teach_failure(fail[0], fail[2])

        # Clean-side knee relaxation: settle-gated, clean samples only.
        if new_mcs != self._last_ingest_mcs:
            self._ticks_at_mcs = 0
        else:
            self._ticks_at_mcs += 1
        self._last_ingest_mcs = new_mcs
        prior_settled = self._ticks_at_mcs >= self.cfg.learned_prior.settle_ticks
        operating_clean = signals.residual_loss_w < self.cfg.learned_prior.viable_loss
        prior_learn = signals.snr is not None and prior_settled and operating_clean
        if prior_learn:
            self.learned_prior.ingest(
                snr=signals.snr, operating_mcs=new_mcs, operating_clean=True, settled=True
            )

        reason = "; ".join(
            r for r in ([predict_reason, snr_demote_reason] + self.leading.reasons) if r
        )
        self.flightlog.write(
            {
                "ts": signals.timestamp,
                "rssi": signals.rssi,
                "rssi_raw": signals.rssi_raw,
                "snr": signals.snr_w,
                "snr_ewma": signals.snr,
                "promote_blocked": promote_blocked,
                "promote_suppressed": self.leading._promote_suppressed,
                "flap_level": self.leading._flap_level.get(self.leading.state.current_mcs + 1, 0),
                "snr_knees": self.learned_prior.snr_knees_snapshot(),
                "evm": signals.evm_w,
                "evm_lo": signals.evm_lo_w,
                "evm_min": signals.evm_min_w,
                "mcs": new_mcs,
                "width": self.cfg.link_width,
                "reason": reason,
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "link_starved": sustained_starved,
                "fail_class": fail[1] if fail else None,
                "trial": self.leading._trial_rung,
                "snapback_tgt": self.leading._snapback_tgt,
                "clean_dwell": self.leading._clean_dwell,
                "prior_learn": prior_learn,
                "slope": slope,
                "predict_gated": predict_gated,
            }
        )
        return Decision(
            timestamp=signals.timestamp,
            mcs=new_mcs,
            reason=reason,
            signals_snapshot={
                "rssi": signals.rssi,
                "rssi_raw": signals.rssi_raw,
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "link_starved": sustained_starved,
                "mcs": new_mcs,
            },
        )

    def bind_learned_prior(self, adapter_id: str, width: int) -> None:
        """(Re)key the learned prior to the drone adapter id AND channel width.
        Called at the connect edge (and on a ground width change) before
        warm-start, so the session learns/persists under the correct per-card,
        per-width file (<adapter>__bw<width>.json). 10 MHz sees ~3 dB better SNR
        than 20, so the knees must not be shared. No-op when the key is unchanged."""
        key = f"{adapter_id}__bw{int(width)}"
        if self.learned_prior.key == key:
            return
        self.profile_name = adapter_id
        self.learned_prior = LearnedPrior(key, self.cfg.learned_prior)

    def reset_for_new_session(self) -> None:
        """Reset volatile selector + hysteresis state to boot (incl. the SNR
        slope window, so the first predictive-demote slope is computed fresh).
        A confirmed drone reconnect is a new session; re-climb from the boot MCS
        instead of resuming a stale climbed-up rung. The persistent learned_prior
        knees are kept (cross-session knowledge). This is selector state only —
        the connect handler also calls self.flightlog.begin_flight() to roll the
        flight."""
        self.leading = LeadingSelector(self.cfg.selector)
        self._starvation_count = 0
        self._ticks_at_mcs = 0
        self._last_ingest_mcs = None
        self._predict_demote_count = 0
        self._snr_demote_count = 0
        self._windows_since_demote = self.cfg.selector.demote_cooldown_windows
        self._snr_window.clear()

    def close(self) -> None:
        """Flush the learned prior + close the flight log. Called by the
        controller when the dynamicLink loop tears down."""
        self.learned_prior.flush()
        self.flightlog.close()
