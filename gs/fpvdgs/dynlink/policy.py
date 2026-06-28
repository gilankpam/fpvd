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
from .learned_prior import UNBOUND_KEY, LearnedPrior, LearnedPriorConfig, lsq_slope
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
    # Shared demote cooldown: minimum windows between consecutive demotes
    # (any path). After a demote, the next N windows are frozen so the
    # lower rung's true loss reading returns over the GS->drone->GS
    # application lag before deciding to step down again. Enforces <= 1
    # demote per N windows across reactive/predict/snr/emergency.
    demote_cooldown_windows: int = 3
    # Proactive SNR demote: consecutive ticks snr_ceiling must stay below the
    # current rung before demoting to it (debounce; snr is already EWMA'd).
    snr_demote_debounce: int = 2
    # SNR-knee hysteresis (dB). promote_blocked (snr_promote_margin_db) is now
    # advisory only: the live probe is authoritative for promotes, so a
    # clean+fresh+debounced rung promotes regardless. The proactive *demote* still
    # fires when the live SNR is more than snr_demote_margin_db below the current
    # rung's knee. The asymmetric demote margin (demote > promote) keeps a stable
    # dead-band on the demote path while the probe governs the promote path.
    snr_promote_margin_db: float = 1.0
    snr_demote_margin_db: float = 1.5


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
        return fec_pressure >= self.cfg.emergency_fec_pressure or link_starved

    # ---- main entry ----

    def select(
        self,
        *,
        probe: dict | None,
        loss_rate: float,
        loss_demote: bool = False,
        # Advisory only: the probe is authoritative for promotes; the caller
        # (policy.tick) computes + passes promote_blocked for flight-log
        # visibility, but select() no longer gates on it.
        promote_blocked: bool = False,
        fec_pressure: float,
        link_starved: bool,
        can_demote: bool = True,
        ts_ms: float,
    ) -> tuple[int, bool]:
        """Probe-driven promote + reactive demote.

        Returns (mcs, changed).

        Demote is reactive and one-step, gated by can_demote (owned by
        Policy's cooldown counter). Loss and Channel-B emergencies
        (fec/starvation) both step exactly one rung down when can_demote
        is True; when blocked, HOLD (no promote). Promote requires the
        `current+1` probe rung to read clean+fresh for
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

        # --- Demote (reactive, one step, cooldown-gated) ---
        # Loss and Channel-B emergencies (fec/starvation) both step exactly one
        # rung down. The cooldown (can_demote, owned by Policy) paces the descent
        # and rejects stale loss readings during the apply lag. When a demote is
        # wanted but the cooldown blocks it, HOLD (no promote) — never climb while
        # loss/emergency is live.
        emergency = loss_demote or self._emergency_active(fec_pressure, link_starved)
        if emergency:
            if can_demote:
                if loss_demote:
                    commit(prev - 1, f"video_per_demote loss={loss_rate:.3f}")
                else:
                    commit(
                        prev - 1,
                        f"emergency fec={fec_pressure:.3f} starved={link_starved}",
                    )
            self._reasons = reasons
            return (st.current_mcs, st.current_mcs != prev)

        # --- Rate limit (promotes only; emergencies above bypass it) ---
        within_hold = (ts_ms - st.last_change_time_ms) < self.cfg.hold_modes_down_ms
        within_rate = (ts_ms - st.last_change_time_ms) < self.cfg.min_between_changes_ms

        # --- Promote: clean+fresh current+1 for promote_debounce_windows ---
        # The debounce counter accumulates even while the rate limit
        # blocks a commit, so the climb fires as soon as both gates open.
        target = st.current_mcs + 1
        rung = (probe or {}).get("mcs", {}).get(str(target)) if target <= self._cap_mcs else None
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
            # Fix A: the live probe is authoritative for promotes. A clean+fresh+
            # debounced rung promotes regardless of promote_blocked — the SNR knee
            # is advisory only (logged for analysis, still drives proactive demote).
            # promote_blocked is precomputed in policy.tick (it needs the learned
            # prior) and passed through for flight-log visibility.
            if (
                self._promote_clean >= self.cfg.promote_debounce_windows
                and not within_hold
                and not within_rate
            ):
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
        profile_name: str = UNBOUND_KEY,
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
        # Learned per-card prior (always-on), keyed by the drone-reported
        # adapter id (radio.adapterId); GS-local, the live probe stays authoritative.
        self.learned_prior = LearnedPrior(profile_name, cfg.learned_prior)
        self._rssi_window: deque[float] = deque(
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
                    if (
                        self._predict_demote_count
                        >= self.cfg.learned_prior.predictive_debounce_windows
                        and cooldown_ok
                    ):
                        tgt = max(cur - 1, 0)
                        if tgt < cur:
                            self.leading.state.current_mcs = tgt
                            self.leading._promote_clean = 0
                            predict_reason = f"predict_demote mcs{cur}->{tgt}"
                else:
                    # pc says demote but RSSI isn't falling fast enough to matter
                    # (flat/rising = a static prior-vs-probe disagreement, or a
                    # fade too shallow to clear predictive_min_drop_db). Not a
                    # real fade — suppress (the flapping fix) and log it.
                    predict_gated = True
                    self._predict_demote_count = 0
            else:
                self._predict_demote_count = 0

        # snr_ceiling kept for the flight log only; no longer a demote target.
        snr_ceiling = self.learned_prior.snr_ceiling(signals.snr)

        # Proactive SNR demote (down-only, debounced): if the SNR prior
        # CONFIDENTLY says the CURRENT rung is unviable at the live SNR, jump
        # straight down to the highest rung it still supports, ahead of the loss
        # — catching the interference the RSSI slope-gate can't see. Gated on the
        # current rung being confidently unviable (NOT merely "below the highest
        # confident-viable rung"), so a not-yet-learned rung the probe just
        # climbed onto is not yanked back before its knee can warm.
        snr_demote_reason = ""
        cur_snr = self.leading.state.current_mcs
        if snr_ceiling is not None and self.learned_prior.snr_rung_unviable(
            cur_snr, signals.snr, margin=self.cfg.selector.snr_demote_margin_db
        ):
            self._snr_demote_count += 1
            if self._snr_demote_count >= self.cfg.selector.snr_demote_debounce:
                self.leading.state.current_mcs = snr_ceiling
                self.leading._promote_clean = 0
                snr_demote_reason = f"snr_demote mcs{cur_snr}->{snr_ceiling}"
        else:
            self._snr_demote_count = 0

        # Promote veto: block the probe's next-rung climb only if the SNR prior
        # CONFIDENTLY says that target rung is unviable at the live SNR. A cold
        # (unlearned) target is explorable — see select()'s frontier note.
        promote_target = self.leading.state.current_mcs + 1
        promote_blocked = self.learned_prior.snr_rung_unviable(
            promote_target, signals.snr, margin=self.cfg.selector.snr_promote_margin_db
        )

        probe_snap = self._probe_status() if self._probe_status else None
        new_mcs, _changed = self.leading.select(
            probe=probe_snap,
            loss_rate=signals.residual_loss_w,
            loss_demote=sustained_loss,
            promote_blocked=promote_blocked,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            can_demote=cooldown_ok and self.leading.state.current_mcs == start_mcs,
            ts_ms=ts_ms,
        )

        if new_mcs < start_mcs:
            self._windows_since_demote = 0

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
        probe_log = (
            None
            if probe_snap is None
            else {
                m: {"per": v.get("per"), "ageMs": v.get("ageMs")}
                for m, v in (probe_snap.get("mcs") or {}).items()
            }
        )
        self.flightlog.write(
            {
                "ts": signals.timestamp,
                "rssi": signals.rssi,
                "rssi_raw": signals.rssi_raw,
                "snr": signals.snr_w,
                "snr_norm": signals.snr,
                "snr_ceiling": snr_ceiling,
                "promote_blocked": promote_blocked,
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
                "ceiling": (
                    self.learned_prior.ceiling(signals.rssi) if signals.rssi is not None else None
                ),
                "probe": probe_log,
                "pc": pc,
                "knees": self.learned_prior.knees_snapshot(),
                "prior_learn": prior_learn,
                "slope": slope,
                "predict_gated": predict_gated,
                "promote_clean": self.leading._promote_clean,
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
