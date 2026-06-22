"""Policy engine — probe-driven MCS selector (§4).

Runs at 10 Hz, one tick per RxEvent. Pure function of
(Signals snapshot, internal hysteresis state). Emits a `{mcs}`-only
Decision on every tick.

Phase 3b: the drone computes its own bitrate / FEC / depth / tx_power
locally, so the GS no longer composes any of that. The selector (Phase 2)
is the only decision: probe-promote + reactive demote, with a learned-prior
warm-start seed and starvation hysteresis feeding the emergency demote.
"""
from __future__ import annotations

import logging
from collections import deque
from dataclasses import dataclass, field

from .decision import Decision
from .flightlog import FlightLog, FlightLogConfig
from .learned_prior import LearnedPrior, LearnedPriorConfig, lsq_slope
from .signals import Signals

log = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Config dataclasses for the probe-driven selector + smoothing/learned-prior/flightlog.
# ------------------------------------------------------------------

@dataclass
class SelectorConfig:
    """Probe-driven promote + reactive demote + timing/cadence.

    Promote: the `current+1` probe rung must read clean (EWMA success
    >= probe_viable_threshold) and fresh (within probe_freshness_ms) for
    promote_debounce_windows consecutive ticks, and clear the
    hold_modes_down_ms / min_between_changes_ms cooldowns. Demote: a
    caller-hysteresis-gated loss breach (`residual_loss_w >= video_demote_per`)
    or a Channel-B emergency (fec/starvation). starvation_windows is the
    consecutive-starved-window count before link_starved feeds the emergency demote.
    """
    # Probe-driven promote
    probe_viable_threshold: float = 0.99
    probe_freshness_ms: float = 500.0
    promote_debounce_windows: int = 3
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
    # Proactive SNR demote: consecutive ticks snr_ceiling must stay below the
    # current rung before demoting to it (debounce; snr is already EWMA'd).
    snr_demote_debounce: int = 2
    # SNR-knee hysteresis (dB). The promote veto blocks a climb only when the
    # live SNR is more than snr_promote_margin_db BELOW the target rung's learned
    # knee; the proactive demote fires only when it is more than
    # snr_demote_margin_db below the current rung's knee. demote > promote opens a
    # stable dead-band: without it the zero-margin `snr < knee` veto pins MCS at
    # the rung whose knee sits a hair above the live SNR (it can only relax by
    # operating there, which the veto blocks) — the MCS-stuck-at-4 field bug.
    snr_promote_margin_db: float = 1.0
    snr_demote_margin_db: float = 1.5


@dataclass
class PolicyConfig:
    selector: SelectorConfig = field(default_factory=SelectorConfig)
    learned_prior: LearnedPriorConfig = field(default_factory=LearnedPriorConfig)
    flightlog: FlightLogConfig = field(default_factory=FlightLogConfig)


# ------------------------------------------------------------------
# Leading selector — dual-gate ProfileSelector (alink_gs port).
# ------------------------------------------------------------------

@dataclass
class LeadingState:
    current_mcs: int                  # currently selected MCS
    # Initialise the timing anchors well in the past so the first
    # decision after boot doesn't get gated by min_between_changes_ms
    # / hold_modes_down_ms. In production ts_ms is a wall-clock value
    # in the trillions; this just guarantees the same "long-elapsed"
    # condition holds in tests where ts_ms starts near zero.
    last_change_time_ms: float = -1.0e9
    last_mcs_change_time_ms: float = -1.0e9


class LeadingSelector:
    """MCS / TX power selector — probe-driven promote + reactive demote.

    The selector decides MCS only; FEC ladder, bitrate, and depth are
    computed downstream by the drone.

    Promote (slow/deliberate): the `current+1` probe rung must read
    clean (EWMA success `1 - per >= probe_viable_threshold`) and fresh
    (sample age `<= probe_freshness_ms`) for `promote_debounce_windows`
    consecutive ticks. The climb naturally stops at the ceiling — a
    cliffed `current+1` rung (per≈1.0, or absent) never debounces.

    Demote (fast/reactive): a caller-hysteresis-gated loss breach
    (`loss_demote`, i.e. `residual_loss_w >= video_demote_per`) or a
    Channel-B emergency (fec_pressure or link_starved) forces an immediate
    one-step downgrade, bypassing the promote rate limit and hold timers.
    """

    def __init__(self, cfg: SelectorConfig):
        self.cfg = cfg
        cap = int(cfg.max_mcs)
        if cap < 0:
            raise ValueError(f"max_mcs={cfg.max_mcs} excludes every MCS")
        self._cap_mcs = cap
        # Boot at the safe-default MCS (1) just like the prior loop.
        start_mcs = 1
        if start_mcs > cap:
            start_mcs = cap
        self.state = LeadingState(current_mcs=start_mcs)
        self._reasons: list[str] = []
        # Consecutive ticks the current+1 probe rung has read clean+fresh.
        # Resets on any blip, stale read, demote, or applied promote.
        self._promote_clean = 0

    # ---- helpers ----

    def _emergency_active(self, fec_pressure: float, link_starved: bool) -> bool:
        return (
            fec_pressure >= self.cfg.emergency_fec_pressure
            or link_starved
        )

    # ---- main entry ----

    def select(
        self,
        *,
        probe: dict | None,
        loss_rate: float,
        loss_demote: bool = False,
        loss_demote_target: int | None = None,
        promote_blocked: bool = False,
        fec_pressure: float,
        link_starved: bool,
        ts_ms: float,
    ) -> tuple[int, bool]:
        """Probe-driven promote + reactive demote.

        Returns (mcs, changed).

        Demote is reactive and bypasses the promote rate limit: a
        Channel-B emergency (loss/fec/starvation) or a video on-air PER
        breach forces an immediate one-step downgrade. Promote requires
        the `current+1` probe rung to read clean+fresh for
        `promote_debounce_windows` consecutive ticks AND the rate limit
        (`min_between_changes_ms` / `hold_modes_down_ms`) to be clear.
        The debounce counter accumulates across ticks even while the
        rate limit blocks a commit, so the climb fires as soon as both
        gates open.
        """
        st = self.state
        prev = st.current_mcs
        reasons: list[str] = []

        def commit(new_mcs: int, why: str) -> None:
            new_mcs = max(0, min(new_mcs, self._cap_mcs))
            if new_mcs != st.current_mcs:
                st.current_mcs = new_mcs
                st.last_change_time_ms = ts_ms
                st.last_mcs_change_time_ms = ts_ms
                self._promote_clean = 0
                reasons.append(why)

        # --- Demote (reactive, bypasses the promote rate limit) ---
        # Loss (caller-hysteresis-gated) is the common case and is attributed
        # first; FEC pressure / sustained starvation are the other emergencies.
        if loss_demote:
            # Jump straight to the rung the live SNR supports (one move, no
            # overshoot). target None (cold SNR knee) -> today's one-step demote.
            if loss_demote_target is not None:
                tgt = min(prev, int(loss_demote_target))
                commit(tgt, f"video_per_demote loss={loss_rate:.3f} -> mcs{tgt}")
            else:
                commit(prev - 1, f"video_per_demote loss={loss_rate:.3f}")
            self._reasons = reasons
            return (st.current_mcs, st.current_mcs != prev)
        if self._emergency_active(fec_pressure, link_starved):
            commit(
                prev - 1,
                f"emergency fec={fec_pressure:.3f} starved={link_starved}",
            )
            self._reasons = reasons
            return (st.current_mcs, st.current_mcs != prev)

        # --- Rate limit (promotes only; emergencies above bypass it) ---
        within_hold = (ts_ms - st.last_change_time_ms) < self.cfg.hold_modes_down_ms
        within_rate = (
            (ts_ms - st.last_change_time_ms) < self.cfg.min_between_changes_ms
        )

        # --- Promote: clean+fresh current+1 for promote_debounce_windows ---
        # The debounce counter accumulates even while the rate limit
        # blocks a commit, so the climb fires as soon as both gates open.
        target = st.current_mcs + 1
        rung = (
            (probe or {}).get("mcs", {}).get(str(target))
            if target <= self._cap_mcs else None
        )
        fresh = (
            rung is not None
            and rung.get("ageMs") is not None
            and rung["ageMs"] <= self.cfg.probe_freshness_ms
        )
        clean = (
            fresh
            and rung.get("per") is not None
            and (1.0 - rung["per"]) >= self.cfg.probe_viable_threshold
        )
        if clean:
            self._promote_clean += 1
            # SNR caps the probe's optimism: never promote to a rung the live SNR
            # CONFIDENTLY says is unviable (the 4<->3 oscillation driver — the probe
            # measures rung+1 at the current rung's TX power and reads it clean).
            # promote_blocked is precomputed in policy.tick (it needs the learned
            # prior). A rung the SNR prior hasn't learned is NOT blocked — unknown
            # != bad, so the probe may explore the frontier; otherwise the top
            # rung, whose knee can only be learned BY operating there, is forever
            # unreachable (the maxMcs-never-reached deadlock).
            if (self._promote_clean >= self.cfg.promote_debounce_windows
                    and not within_hold and not within_rate and not promote_blocked):
                commit(target, f"probe_promote mcs{target} per={rung['per']:.4f}")
        else:
            self._promote_clean = 0

        self._reasons = reasons
        return st.current_mcs, (st.current_mcs != prev)

    @property
    def reasons(self) -> list[str]:
        return list(self._reasons)


# ------------------------------------------------------------------
# Top-level policy: runs the selector, emits a {mcs}-only Decision.
# ------------------------------------------------------------------

class Policy:
    """Runs the probe-driven dual-gate selector and emits the
    `{mcs}`-only Decision."""

    def __init__(
        self,
        cfg: PolicyConfig,
        profile_name: str = "m8812eu2",
        *,
        probe_status=None,
    ) -> None:
        self.cfg = cfg
        self.profile_name = profile_name
        # Probe snapshot provider (zero-arg callable returning the
        # ProbeController.status() dict, or None). The selector promotes
        # MCS only when the probed current+1 rung reads clean+fresh. When
        # left None (e.g. tests / no probe) the selector can never
        # promote — it only reacts to emergencies.
        self._probe_status = probe_status
        # Warm-start one-shot: seed the operating MCS from the learned
        # per-card prior on the first tick where RSSI is present, so the
        # first real decision isn't stuck at the boot MCS while the probe
        # warms up. Flipped True after the single seed; the probe-driven
        # select() owns MCS thereafter. (No raw-RSSI fallback — that
        # cold-start table was removed; a cold prior just lets the probe
        # climb from boot.)
        self._cold_started = False
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
        # Learned per-card prior (always-on), keyed by the operator-set
        # dynamicLink.radioProfile; GS-local, the live probe stays authoritative.
        self.learned_prior = LearnedPrior(profile_name, cfg.learned_prior)
        self._rssi_window: deque[float] = deque(
            maxlen=cfg.learned_prior.predictive_slope_window_ticks)
        self._predict_demote_count = 0
        self._snr_demote_count = 0
        self.flightlog = FlightLog(cfg.flightlog)

    def tick(self, signals: Signals) -> Decision:
        ts_ms = signals.timestamp * 1000.0 if signals.timestamp else 0.0

        # Starvation hysteresis: per-tick link_starved_w flickers on
        # brief packet-rate dips in bursty video. Require N consecutive
        # starved windows before the selector treats it as emergency.
        if signals.link_starved_w:
            self._starvation_count += 1
        else:
            self._starvation_count = 0
        sustained_starved = (
            self._starvation_count >= self.cfg.selector.starvation_windows
        )

        # Reactive loss demote: single-window. residual_loss_w is raw, but the
        # reactive demote is an SNR-jump that lands on the right rung in one move
        # (nothing to cascade), so react on the first breaching window.
        sustained_loss = (
            signals.residual_loss_w >= self.cfg.selector.video_demote_per
        )

        # Warm-start seed (one-shot). Uses the learned per-card curve ONLY —
        # there is no RSSI hand-table fallback. Under per-MCS dynamic TX power
        # RSSI is not a reliable absolute MCS predictor, so when the prior is
        # cold the probe climbs safely from the boot MCS. Only raises the boot
        # MCS, runs before select().
        if not self._cold_started and signals.rssi is not None:
            seed = self.learned_prior.warmstart_seed(signals.rssi)
            if seed is not None and seed > self.leading.state.current_mcs:
                self.leading.state.current_mcs = min(seed, self.leading._cap_mcs)
            self._cold_started = True

        # Predictive demote (down-only, confidence-gated, debounced). If the
        # curve says the ceiling at the projected RSSI is below where we run,
        # pre-demote ahead of the reactive path. The probe still owns promotes;
        # the reactive Channel-B demote in select() remains the backstop.
        predict_reason = ""
        predict_gated = False
        if signals.rssi is None:
            slope = None
        else:
            self._rssi_window.append(signals.rssi)
            slope = lsq_slope(self._rssi_window)
        pc = None
        if signals.rssi is not None:
            pc = self.learned_prior.predictive_ceiling(signals.rssi, slope)
            cur = self.leading.state.current_mcs
            projected_drop = -slope * self.cfg.learned_prior.predictive_horizon_ticks
            if pc is not None and pc < cur:
                if projected_drop >= self.cfg.learned_prior.predictive_min_drop_db:
                    self._predict_demote_count += 1
                    if (self._predict_demote_count
                            >= self.cfg.learned_prior.predictive_debounce_windows):
                        self.leading.state.current_mcs = max(pc, 0)
                        self.leading._promote_clean = 0
                        predict_reason = f"predict_demote mcs{cur}->{pc}"
                else:
                    # pc says demote but RSSI isn't falling fast enough to matter
                    # (flat/rising = a static prior-vs-probe disagreement, or a
                    # fade too shallow to clear predictive_min_drop_db). Not a
                    # real fade — suppress (the flapping fix) and log it.
                    predict_gated = True
                    self._predict_demote_count = 0
            else:
                self._predict_demote_count = 0

        # Selector (Phase 2) is the only decision now: probe-promote +
        # reactive demote. The drone computes its own bitrate / FEC /
        # depth / tx_power locally, so we emit {mcs} only.
        loss_demote_target = self.learned_prior.snr_ceiling(signals.snr)

        # Proactive SNR demote (down-only, debounced): if the SNR prior
        # CONFIDENTLY says the CURRENT rung is unviable at the live SNR, jump
        # straight down to the highest rung it still supports, ahead of the loss
        # — catching the interference the RSSI slope-gate can't see. Gated on the
        # current rung being confidently unviable (NOT merely "below the highest
        # confident-viable rung"), so a not-yet-learned rung the probe just
        # climbed onto is not yanked back before its knee can warm.
        snr_demote_reason = ""
        cur_snr = self.leading.state.current_mcs
        if (loss_demote_target is not None
                and self.learned_prior.snr_rung_unviable(
                    cur_snr, signals.snr,
                    margin=self.cfg.selector.snr_demote_margin_db)):
            self._snr_demote_count += 1
            if self._snr_demote_count >= self.cfg.selector.snr_demote_debounce:
                self.leading.state.current_mcs = loss_demote_target
                self.leading._promote_clean = 0
                snr_demote_reason = f"snr_demote mcs{cur_snr}->{loss_demote_target}"
        else:
            self._snr_demote_count = 0

        # Promote veto: block the probe's next-rung climb only if the SNR prior
        # CONFIDENTLY says that target rung is unviable at the live SNR. A cold
        # (unlearned) target is explorable — see select()'s frontier note.
        promote_target = self.leading.state.current_mcs + 1
        promote_blocked = self.learned_prior.snr_rung_unviable(
            promote_target, signals.snr,
            margin=self.cfg.selector.snr_promote_margin_db)

        probe_snap = self._probe_status() if self._probe_status else None
        new_mcs, _changed = self.leading.select(
            probe=probe_snap,
            loss_rate=signals.residual_loss_w,
            loss_demote=sustained_loss,
            loss_demote_target=loss_demote_target,
            promote_blocked=promote_blocked,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            ts_ms=ts_ms,
        )

        # Learning gate: feed the knee prior ONLY operating-rung outcomes, and
        # only once the rung has been settled for settle_ticks (rejects fast-fade
        # transients where loss is a transition artifact, not rung unviability).
        if new_mcs != self._last_ingest_mcs:
            self._ticks_at_mcs = 0
        else:
            self._ticks_at_mcs += 1
        self._last_ingest_mcs = new_mcs
        prior_settled = self._ticks_at_mcs >= self.cfg.learned_prior.settle_ticks
        prior_learn = (signals.rssi is not None or signals.snr is not None) and prior_settled
        self.learned_prior.ingest(
            rssi=signals.rssi,
            snr=signals.snr,
            operating_mcs=new_mcs,
            operating_clean=signals.residual_loss_w < self.cfg.learned_prior.viable_loss,
            settled=prior_settled,
        )

        reason = "; ".join(
            r for r in ([predict_reason, snr_demote_reason] + self.leading.reasons) if r
        )
        # Compact per-rung probe view (per + ageMs only) so the record stays
        # small at 10 Hz against the flight-log size cap.
        probe_log = (None if probe_snap is None else {
            m: {"per": v.get("per"), "ageMs": v.get("ageMs")}
            for m, v in (probe_snap.get("mcs") or {}).items()
        })
        self.flightlog.write({
            "ts": signals.timestamp,
            "rssi": signals.rssi,
            "rssi_raw": signals.rssi_raw,
            "snr": signals.snr_w,
            "snr_norm": signals.snr,
            "snr_ceiling": loss_demote_target,
            "promote_blocked": promote_blocked,
            "snr_knees": self.learned_prior.snr_knees_snapshot(),
            "evm": signals.evm_w,
            "evm_lo": signals.evm_lo_w,
            "evm_min": signals.evm_min_w,
            "mcs": new_mcs,
            "reason": reason,
            "residual_loss_w": signals.residual_loss_w,
            "fec_work": signals.fec_work,
            "link_starved": sustained_starved,
            "ceiling": (self.learned_prior.ceiling(signals.rssi)
                        if signals.rssi is not None else None),
            "probe": probe_log,
            "pc": pc,
            "knees": self.learned_prior.knees_snapshot(),
            "prior_learn": prior_learn,
            "slope": slope,
            "predict_gated": predict_gated,
            "promote_clean": self.leading._promote_clean,
        })
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

    def reset_for_new_session(self) -> None:
        """Reset volatile selector + hysteresis state to boot (incl. the RSSI
        slope window, so the first predictive-demote slope is computed fresh).
        A confirmed drone reconnect is a new session, so re-run the learned-prior
        warm-start and re-climb from the boot MCS instead of resuming a stale
        climbed-up rung. The persistent learned_prior knees are kept
        (cross-session knowledge). This is selector state only — the connect
        handler also calls self.flightlog.begin_flight() to roll the flight."""
        self.leading = LeadingSelector(self.cfg.selector)
        self._cold_started = False
        self._starvation_count = 0
        self._ticks_at_mcs = 0
        self._last_ingest_mcs = None
        self._predict_demote_count = 0
        self._snr_demote_count = 0
        self._rssi_window.clear()

    def close(self) -> None:
        """Flush the learned prior + close the flight log. Called by the
        controller when the dynamicLink loop tears down."""
        self.learned_prior.flush()
        self.flightlog.close()
