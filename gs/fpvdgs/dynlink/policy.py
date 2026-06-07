"""Policy engine — leading + trailing loops (§4).

Runs at 10 Hz, one tick per RxEvent. Pure function of
(Signals snapshot, internal hysteresis state). Emits a Decision on
every tick; `knobs_changed` records which knobs actually moved.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

from .bitrate import BitrateConfig, compute_bitrate_kbps, compute_wire_target_kbps
from .decision import Decision
from .drone_config import DroneConfigState
from .dynamic_fec import (
    DynamicFecConfig,
    EmitGate,
    NEscalator,
    clamp_n_for_bitrate_floor,
    compute_k,
    compute_n,
)
from .predictor import (
    BudgetExhausted,
    PredictorConfig,
    Proposal,
    fit_or_degrade,
)
from .profile import MCSRow, RadioProfile
from .signals import Signals

log = logging.getLogger(__name__)


# ------------------------------------------------------------------
# Config dataclasses — mirror the §6 gs.yaml layout.
# ------------------------------------------------------------------

@dataclass
class LeadingLoopConfig:
    """Static / hardware-side knobs that aren't part of the gate.

    Carries the TX-power range and bandwidth the probe-driven selector
    needs from the radio side, plus deprecated keys kept for back-compat
    YAML parsing (parsed and ignored by the current selector).
    """
    bandwidth: int = 20
    # MCS-coupled TX power: power = max - (mcs / max_mcs) * (max - min).
    # Atomic per-tick. Inputs to the selector's _compute_tx_power().
    tx_power_min_dBm: float = 5.0
    tx_power_max_dBm: float = 23.0

    # Deprecated — kept so old gs.yaml files still parse. The new
    # selector ignores these. Operator should migrate to `gate:` and
    # `profile_selection:` sections; service.py emits a WARN if any
    # of these are explicitly set.
    mcs_max: int = 7
    snr_margin_db: float = 3.0
    snr_up_guard_db: float = 2.0
    snr_up_hold_ms: float = 2000.0
    snr_down_hold_ms: float = 500.0
    loss_margin_weight: float = 20.0
    fec_margin_weight: float = 20.0
    forced_drop_inhibit_ms: float = 5000.0
    rssi_up_guard_db: float = 3.0
    rssi_up_hold_ms: float = 2000.0
    rssi_down_hold_ms: float = 500.0
    rssi_target_dBm: float = -60.0
    rssi_deadband_db: float = 3.0
    tx_power_cooldown_ms: float = 1000.0
    tx_power_freeze_after_mcs_ms: float = 2000.0
    tx_power_step_max_db: float = 3.0
    tx_power_gain_up_db: float = 1.0
    tx_power_gain_down_db: float = 1.0


@dataclass
class GateConfig:
    """Probe-driven promote + emergency (Channel-B) demote.

    Promote: the `current+1` probe rung must read clean (EWMA success
    >= probe_viable_threshold) and fresh (within probe_freshness_ms) for
    promote_debounce_windows consecutive ticks. Demote: the kept Channel-B
    emergency (loss/fec/starvation) plus a video on-air PER breach
    (video_demote_per on (lost+fec_rec)/(out+lost)).
    """
    # Probe-driven promote
    probe_viable_threshold: float = 0.99   # min EWMA success (1 - per) to climb
    probe_freshness_ms: float = 500.0      # max age of the probed rung's sample
    promote_debounce_windows: int = 3      # consecutive clean ticks before a climb
    # Reactive demote
    video_demote_per: float = 0.05         # (lost+fec_rec)/(out+lost) demote breach
    emergency_loss_rate: float = 0.05
    emergency_fec_pressure: float = 0.80
    # MCS bounds
    max_mcs: int = 7
    max_mcs_step_up: int = 1


@dataclass
class ProfileSelectionConfig:
    """Timing/cadence knobs for the dual-gate selector."""
    hold_fallback_mode_ms: int = 1000
    hold_modes_down_ms: int = 2000
    min_between_changes_ms: int = 200
    fast_downgrade: bool = True
    upward_confidence_loops: int = 4


@dataclass
class CooldownConfig:
    min_change_interval_ms_fec: float = 200.0
    min_change_interval_ms_depth: float = 200.0
    min_change_interval_ms_radio: float = 500.0
    min_change_interval_ms_cross: float = 50.0


@dataclass(frozen=True)
class FECBounds:
    """Defensive ceilings for FEC. `(k, n)` is computed at runtime
    by `dynamic_fec`; only `depth_max` remains here.
    """
    depth_max: int = 3


@dataclass
class SafeDefaults:
    k: int = 8
    n: int = 12
    depth: int = 1
    mcs: int = 1


@dataclass
class PolicyConfig:
    leading: LeadingLoopConfig = field(default_factory=LeadingLoopConfig)
    gate: GateConfig = field(default_factory=GateConfig)
    selection: ProfileSelectionConfig = field(
        default_factory=ProfileSelectionConfig
    )
    cooldown: CooldownConfig = field(default_factory=CooldownConfig)
    fec: FECBounds = field(default_factory=FECBounds)
    safe: SafeDefaults = field(default_factory=SafeDefaults)
    bitrate: BitrateConfig = field(default_factory=BitrateConfig)
    dynamic_fec: DynamicFecConfig = field(default_factory=DynamicFecConfig)
    predictor: PredictorConfig = field(default_factory=PredictorConfig)
    max_latency_ms: float = 50.0
    # Trailing-loop depth=1 → 2 bootstrap requires the last N windows
    # all show residual_loss (§4.2 sustained-loss trigger). Default 3.
    sustained_loss_windows: int = 3
    # §4.2 step-down: how many consecutive zero-loss windows before
    # depth steps down one notch. At 10 Hz, 10 windows = 1 s of clean
    # link before reclaiming a depth step. Walks down one step per
    # threshold-met period (counter resets after each step), so a full
    # depth=3 → 1 recovery takes ~2 s of sustained clean link.
    clean_windows_for_depth_stepdown: int = 10
    # Total-blackout failsafe: this many consecutive starved windows
    # (packet_rate_w < starvation_threshold while session active) trips
    # forced_mcs_drop and pins TX power to max. Intentionally short —
    # at 10 Hz, 5 windows = 0.5 s — because starvation is unambiguous
    # and the alternative is letting the link sit silent.
    starvation_windows: int = 5


# ------------------------------------------------------------------
# Leading selector — dual-gate ProfileSelector (alink_gs port).
# ------------------------------------------------------------------

@dataclass
class LeadingState:
    current_mcs: int                  # currently selected MCS
    tx_power_dBm: float               # last-applied power (inverse-coupled)
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
    computed downstream by the trailing loop and bitrate helper.

    Promote (slow/deliberate): the `current+1` probe rung must read
    clean (EWMA success `1 - per >= probe_viable_threshold`) and fresh
    (sample age `<= probe_freshness_ms`) for `promote_debounce_windows`
    consecutive ticks. The climb naturally stops at the ceiling — a
    cliffed `current+1` rung (per≈1.0, or absent) never debounces.

    Demote (fast/reactive): the Channel-B emergency triggers (loss_rate,
    fec_pressure, or link_starved) and a video on-air PER breach
    (`loss_rate >= video_demote_per`) force an immediate one-step
    downgrade, bypassing the promote rate limit and hold timers.

    TX power follows MCS via inverse coupling: low MCS → high power,
    high MCS → low power. Atomic per tick.
    """

    def __init__(
        self,
        leading: LeadingLoopConfig,
        gate: GateConfig,
        sel: ProfileSelectionConfig,
        profile: RadioProfile,
    ):
        self.leading = leading
        self.gate = gate
        self.sel = sel
        self.profile = profile
        # Build the row table. Profile's mcs_max is the hardware ceiling;
        # gate.max_mcs is the operator's runtime cap. Clamp to the lower.
        cap = min(int(gate.max_mcs), profile.mcs_max)
        if cap < profile.mcs_min:
            raise ValueError(
                f"gate.max_mcs={gate.max_mcs} excludes every MCS in "
                f"profile {profile.name!r} (mcs_min={profile.mcs_min}, "
                f"mcs_max={profile.mcs_max})"
            )
        # rows: descending by MCS (highest first). `_row()` and
        # `current_row` look up the table by MCS; the probe-driven
        # selector no longer walks it by SNR margin.
        rows = profile.snr_mcs_map(
            leading.bandwidth,
            snr_margin_db=0.0,           # static margin lives in gate
        )
        self.rows: list[MCSRow] = [
            r for r in rows if profile.mcs_min <= r.mcs <= cap
        ]
        if not self.rows:
            raise ValueError("LeadingSelector: empty MCS row table")
        self._cap_mcs = cap
        # Boot at the safe-default MCS (1) just like the prior loop.
        start_mcs = 1
        if start_mcs > cap:
            start_mcs = cap
        self.state = LeadingState(
            current_mcs=start_mcs,
            tx_power_dBm=leading.tx_power_max_dBm,  # survival: start at max
        )
        self._reasons: list[str] = []
        # Consecutive ticks the current+1 probe rung has read clean+fresh.
        # Resets on any blip, stale read, demote, or applied promote.
        self._promote_clean = 0

    # ---- helpers ----

    def _row(self, mcs: int) -> MCSRow:
        for r in self.rows:
            if r.mcs == mcs:
                return r
        return self.rows[-1]   # mcs below mcs_min → lowest row

    @property
    def current_row(self) -> MCSRow:
        return self._row(self.state.current_mcs)

    def _emergency_active(
        self, loss_rate: float, fec_pressure: float, link_starved: bool
    ) -> bool:
        return (
            loss_rate >= self.gate.emergency_loss_rate
            or fec_pressure >= self.gate.emergency_fec_pressure
            or link_starved
        )

    def _compute_tx_power(self, mcs: int) -> float:
        """Inverse MCS↔power coupling. Atomic per tick."""
        cap = max(1, int(self._cap_mcs))
        t = max(0.0, min(1.0, mcs / cap))
        return (
            self.leading.tx_power_max_dBm
            - t * (self.leading.tx_power_max_dBm
                   - self.leading.tx_power_min_dBm)
        )

    # ---- main entry ----

    def select(
        self,
        *,
        probe: dict | None,
        loss_rate: float,
        fec_pressure: float,
        link_starved: bool,
        ts_ms: float,
    ) -> tuple[int, float, bool]:
        """Probe-driven promote + reactive demote.

        Returns (mcs, tx_power_dBm, changed).

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
                st.tx_power_dBm = self._compute_tx_power(new_mcs)
                st.last_change_time_ms = ts_ms
                st.last_mcs_change_time_ms = ts_ms
                self._promote_clean = 0
                reasons.append(why)

        # --- Demote: emergency (Channel B) or video-PER breach (reactive) ---
        if self._emergency_active(loss_rate, fec_pressure, link_starved):
            commit(
                prev - 1,
                f"emergency loss={loss_rate:.3f} fec={fec_pressure:.3f} "
                f"starved={link_starved}",
            )
            self._reasons = reasons
            return (st.current_mcs, st.tx_power_dBm,
                    st.current_mcs != prev)
        if loss_rate >= self.gate.video_demote_per:
            commit(prev - 1, f"video_per_demote loss={loss_rate:.3f}")
            self._reasons = reasons
            return (st.current_mcs, st.tx_power_dBm,
                    st.current_mcs != prev)

        # --- Rate limit (promotes only; emergencies above bypass it) ---
        within_hold = (ts_ms - st.last_change_time_ms) < self.sel.hold_modes_down_ms
        within_rate = (
            (ts_ms - st.last_change_time_ms) < self.sel.min_between_changes_ms
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
            and rung["ageMs"] <= self.gate.probe_freshness_ms
        )
        clean = (
            fresh
            and rung.get("per") is not None
            and (1.0 - rung["per"]) >= self.gate.probe_viable_threshold
        )
        if clean:
            self._promote_clean += 1
            if (self._promote_clean >= self.gate.promote_debounce_windows
                    and not within_hold and not within_rate):
                commit(target, f"probe_promote mcs{target} per={rung['per']:.4f}")
        else:
            self._promote_clean = 0

        # Same-MCS path: keep power consistent with current MCS.
        if st.current_mcs == prev:
            st.tx_power_dBm = self._compute_tx_power(st.current_mcs)

        self._reasons = reasons
        return st.current_mcs, st.tx_power_dBm, (st.current_mcs != prev)

    @property
    def reasons(self) -> list[str]:
        return list(self._reasons)


# ------------------------------------------------------------------
# Trailing loop — depth.
# ------------------------------------------------------------------
#
# `(k, n)` is computed each tick by `dynamic_fec.compute_k` /
# `compute_n` from `(bitrate, mtu, fps)` plus an `NEscalator` that
# ramps redundancy on sustained `residual_loss`. `EmitGate` bundles
# solo `(k, n)` rewrites onto MCS-change ticks to keep the wire
# cadence cheap. The trailing loop here does two things only:
#
# Manage `depth` (interleaver) bootstrap + step-down. depth is
# independent of `(k, n)` and its reconfig cost is small.
#
# See `docs/knob-cadence-bench.md` for the empirical justification.


def _ipi_ms_for_encoder(encoder_kbps: float, mtu_bytes: int) -> float:
    """Inter-packet interval (ms) implied by a given encoder rate
    and packet size. Returns a large number for non-positive rates."""
    if encoder_kbps <= 0.0:
        return 1000.0
    mtu_bits = mtu_bytes * 8
    return mtu_bits / encoder_kbps


@dataclass
class TrailingState:
    last_depth_change_ts: float = 0.0
    # Consecutive zero-loss windows since last loss; drives depth
    # step-down. Resets to 0 on any loss tick or after a step-down.
    consecutive_clean_windows: int = 0
    # Last N windows' loss state, used by `sustained_loss()` for the
    # depth=1 → depth=2 bootstrap trigger.
    recent_loss_windows: list[bool] = field(default_factory=list)


class TrailingLoop:
    """Depth bootstrap + step-down (§4.2)."""

    def __init__(self, cfg: PolicyConfig):
        self.cfg = cfg
        self.state = TrailingState()
        self._reasons: list[str] = []

    def tick(
        self,
        signals: Signals,
        current_depth: int,
        ts_ms: float,
        *,
        interleaving_supported: bool = True,
    ) -> int:
        """Decide depth for this tick.

        Returns the next depth value. `(k, n)` is supplied
        deterministically by the radio profile row that the leading
        loop selected — the trailing loop never moves it.
        """
        self._reasons = []

        if not interleaving_supported:
            return 1

        st = self.state
        had_loss = signals.residual_loss_w > 0.0

        # Track consecutive clean windows for depth step-down.
        if had_loss:
            st.consecutive_clean_windows = 0
        else:
            st.consecutive_clean_windows += 1

        # Track recent loss windows for the depth=1 → 2 bootstrap.
        st.recent_loss_windows.append(had_loss)
        if len(st.recent_loss_windows) > self.cfg.sustained_loss_windows:
            st.recent_loss_windows = st.recent_loss_windows[
                -self.cfg.sustained_loss_windows:
            ]

        new_depth = current_depth

        # Depth path (independent of FEC `(k, n)`). Two triggers:
        #
        #   bootstrap  (depth=1 → 2): wfb-ng's burst/holdoff counters are
        #     interleaver-internal and structurally zero while depth==1
        #     (rx.cpp:357 short-circuits the interleaved code path), so
        #     they can never trigger the first depth raise. Use a
        #     non-interleaver proxy: sustained loss across the window plus
        #     busy FEC. Once depth>1 the interleaver is engaged and the
        #     real signals become live.
        #
        #   refine     (depth ≥ 2 → higher): the design doc §4.2 trigger
        #     using burst_rate + holdoff_rate. Valid because the interleaver
        #     is on and these counters now reflect reality.
        cooled_depth = (ts_ms - st.last_depth_change_ts
                        >= self.cfg.cooldown.min_change_interval_ms_depth)
        depth_raised = False
        if cooled_depth and current_depth < self.cfg.fec.depth_max:
            bootstrap = (
                current_depth == 1
                and self.sustained_loss()
                and signals.fec_work > 0.10
            )
            refine = (
                signals.burst_rate > 1.0 and signals.holdoff_rate > 0.0
            )
            if bootstrap or refine:
                new_depth = min(current_depth + 1, self.cfg.fec.depth_max)
                st.last_depth_change_ts = ts_ms
                depth_raised = True
                if bootstrap:
                    self._reasons.append(
                        f"sustained_loss fec_work={signals.fec_work:.3f} "
                        f"-> depth={new_depth} (bootstrap)"
                    )
                else:
                    self._reasons.append(
                        f"burst={signals.burst_rate:.1f} "
                        f"holdoff={signals.holdoff_rate:.1f} "
                        f"-> depth={new_depth}"
                    )
        # Step-down (§4.2): after sustained clean link, reclaim a depth
        # step. Walks down one notch per threshold-met period; counter
        # resets so the next step-down requires another clean window.
        # Don't step down on the same tick we raised.
        if (not depth_raised
                and current_depth > 1
                and cooled_depth
                and st.consecutive_clean_windows
                    >= self.cfg.clean_windows_for_depth_stepdown):
            new_depth = max(current_depth - 1, 1)
            st.last_depth_change_ts = ts_ms
            st.consecutive_clean_windows = 0
            self._reasons.append(
                f"clean*{self.cfg.clean_windows_for_depth_stepdown} "
                f"-> depth={new_depth} (stepdown)"
            )

        return new_depth

    @property
    def reasons(self) -> list[str]:
        return list(self._reasons)

    def sustained_loss(self) -> bool:
        """True when the last N windows all had loss (§4.2)."""
        if len(self.state.recent_loss_windows) < self.cfg.sustained_loss_windows:
            return False
        return all(self.state.recent_loss_windows)


# ------------------------------------------------------------------
# Top-level policy: composes leading + trailing + predictor.
# ------------------------------------------------------------------

# Coarse RSSI -> initial MCS, ONLY for cold-start before probe data exists.
# Intentionally conservative; the probe takes over and refines from here.
# (Phase 4 replaces this with the learned per-card prior.)
# Floors must stay in descending order: coarse_mcs_for_rssi returns the first match.
_COLD_START_RSSI_DBM = [(-55, 5), (-65, 3), (-75, 1), (-200, 0)]


def coarse_mcs_for_rssi(rssi):
    if rssi is None:
        return 0
    for floor, mcs in _COLD_START_RSSI_DBM:
        if rssi >= floor:
            return mcs
    return 0


@dataclass
class PolicyState:
    mcs: int
    bandwidth: int
    tx_power_dBm: int
    k: int
    n: int
    depth: int
    bitrate_kbps: int


class Policy:
    """Composes the dual-gate selector + trailing loop + latency-budget
    predictor."""

    def __init__(
        self,
        cfg: PolicyConfig,
        profile: RadioProfile,
        *,
        drone_config: DroneConfigState | None = None,
        probe_status=None,
    ) -> None:
        self.cfg = cfg
        self.profile = profile
        # P4a: when set, Policy.tick emits safe-defaults until the drone
        # sends its first HELLO (DroneConfigState transitions to SYNCED).
        # Left None for back-compat with tests that don't need the gate.
        self.drone_config = drone_config
        # Probe snapshot provider (zero-arg callable returning the
        # ProbeController.status() dict, or None). The selector promotes
        # MCS only when the probed current+1 rung reads clean+fresh. When
        # left None (e.g. tests / no probe) the selector can never
        # promote — it only reacts to emergencies.
        self._probe_status = probe_status
        # Cold-start one-shot: seed the operating MCS from the single
        # link-RSSI via a coarse table on the first post-sync tick where
        # RSSI is present, so the first real decision isn't stuck at the
        # safe floor while the probe warms up. Flipped True after the
        # single seed; the probe-driven select() owns MCS thereafter.
        self._cold_started = False
        self.leading = LeadingSelector(
            cfg.leading, cfg.gate, cfg.selection, profile
        )
        self.trailing = TrailingLoop(cfg)
        # Per-window link_starved_w can flicker on brief packet-rate
        # dips inside an otherwise-healthy bursty stream. Require N
        # consecutive starved windows before treating the link as
        # actually starved for the selector's emergency channel —
        # loss/FEC pressure remain direct triggers (those are real
        # glitches). At 10 Hz, starvation_windows=5 = 0.5 s of below-
        # threshold packet rate before declaring blackout.
        self._starvation_count: int = 0
        # Boot at the leading selector's chosen row. `(k, n)` come
        # from `cfg.safe` — dynamic-FEC starts emitting computed values
        # once `tick()` has seen its first signal snapshot.
        row = self.leading.current_row
        # is_synced() guard needed at construction: drone_config may be
        # non-None but pre-HELLO. Policy.tick() has an early-return guard
        # for that case so it can use a simpler check.
        mtu_for_init = (
            self.drone_config.mtu_bytes
            if self.drone_config is not None and self.drone_config.is_synced()
            else 1400
        )
        _init_wire_target = compute_wire_target_kbps(
            profile, cfg.leading.bandwidth, row.mcs, mtu_for_init,
            cfg.bitrate.utilization_factor,
        )
        self.state = PolicyState(
            mcs=row.mcs,
            bandwidth=cfg.leading.bandwidth,
            tx_power_dBm=int(self.leading.state.tx_power_dBm),
            k=cfg.safe.k,
            n=cfg.safe.n,
            depth=cfg.safe.depth,
            bitrate_kbps=compute_bitrate_kbps(
                wire_target_kbps=_init_wire_target,
                k=cfg.safe.k, n=cfg.safe.n,
                min_bitrate_kbps=cfg.bitrate.min_bitrate_kbps,
                max_bitrate_kbps=cfg.bitrate.max_bitrate_kbps,
            ),
        )
        # Dynamic-FEC state. `_n_escalator` tracks residual-loss
        # hysteresis; `_emit_gate` debounces solo (k, n) changes and
        # bundles them onto MCS-change ticks; `_tick_counter` indexes
        # those debounce windows.
        self._n_escalator = NEscalator(cfg.dynamic_fec)
        self._emit_gate = EmitGate()
        self._tick_counter = 0

    def _safe_decision(self, *, timestamp: float, reason: str) -> Decision:
        """Conservative-defaults Decision used while gated (e.g. before
        the drone's first HELLO). Knobs come from `cfg.safe`; radio
        bounds come from `cfg.leading`. No knobs_changed and no signal
        snapshot — this is a placeholder heartbeat, not a real
        decision."""
        safe = self.cfg.safe
        return Decision(
            timestamp=timestamp,
            mcs=safe.mcs,
            bandwidth=self.cfg.leading.bandwidth,
            tx_power_dBm=int(round(self.cfg.leading.tx_power_min_dBm)),
            k=safe.k,
            n=safe.n,
            depth=safe.depth,
            bitrate_kbps=int(self.cfg.bitrate.min_bitrate_kbps),
            reason=reason,
        )

    def tick(self, signals: Signals) -> Decision:
        # P4a: until the drone has reported its config (mtu, fps,
        # generation_id) via DLHE, emit a safe-defaults decision
        # regardless of incoming signals. Keeps the wire heartbeat
        # alive without applying speculative parameters.
        if self.drone_config is not None and not self.drone_config.is_synced():
            return self._safe_decision(
                timestamp=signals.timestamp,
                reason="awaiting_drone_config",
            )

        ts_ms = signals.timestamp * 1000.0 if signals.timestamp else 0.0
        prev = PolicyState(**self.state.__dict__)

        # Starvation hysteresis: per-tick link_starved_w flickers on
        # brief packet-rate dips in bursty video. Require N consecutive
        # starved windows before the selector treats it as emergency.
        if signals.link_starved_w:
            self._starvation_count += 1
        else:
            self._starvation_count = 0
        sustained_starved = (
            self._starvation_count >= self.cfg.starvation_windows
        )

        # Cold-start seed (one-shot): before any probe data exists the
        # selector would sit at the safe floor while the probe warms up.
        # Seed the operating MCS once from the single link-RSSI via a
        # coarse table. Conservative, only raises (never lowers) the MCS,
        # and runs before select() so the first real decision reflects it.
        if not self._cold_started and signals.rssi is not None:
            seed = coarse_mcs_for_rssi(signals.rssi)
            if seed > self.leading.state.current_mcs:
                self.leading.state.current_mcs = min(seed, self.leading._cap_mcs)
                self.leading.state.tx_power_dBm = self.leading._compute_tx_power(
                    self.leading.state.current_mcs)
            self._cold_started = True

        # Dual-gate selector picks MCS + computes inverse-coupled TX
        # power. Channel B (emergency) is owned by the selector; we
        # don't need to compute forced_drop here anymore.
        new_mcs, tx_power, mcs_changed = self.leading.select(
            probe=self._probe_status() if self._probe_status else None,
            loss_rate=signals.residual_loss_w,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            ts_ms=ts_ms,
        )
        row = self.leading.current_row

        # mtu/fps come from drone HELLO when available; safe fallbacks otherwise.
        mtu = self.drone_config.mtu_bytes if self.drone_config else 1400
        fps = self.drone_config.fps if self.drone_config else 60

        # wire_target_kbps is the anchor: function of (MCS, bw, mtu, util)
        # only — no FEC feedback. Encoder bitrate later shrinks against
        # this as (k, n) grow under loss.
        wire_target_kbps = compute_wire_target_kbps(
            self.profile, self.state.bandwidth, row.mcs,
            mtu, self.cfg.bitrate.utilization_factor,
        )

        # k sized for the worst-case wire rate (full utilization).
        candidate_k = compute_k(
            wire_target_kbps=wire_target_kbps,
            mtu_bytes=mtu, fps=fps,
            cfg=self.cfg.dynamic_fec,
        )

        # n: base + escalation, then clamp to keep bitrate >= floor.
        escalation = self._n_escalator.update(
            loss=float(signals.residual_loss_w)
        )
        n_unclamped = compute_n(
            k=candidate_k, n_escalation=escalation, cfg=self.cfg.dynamic_fec,
        )
        candidate_n = clamp_n_for_bitrate_floor(
            n_candidate=n_unclamped,
            k=candidate_k,
            wire_target_kbps=wire_target_kbps,
            min_bitrate_kbps=self.cfg.bitrate.min_bitrate_kbps,
        )

        # EmitGate decides what actually rides this tick.
        if self._emit_gate.should_emit(
            candidate_k, candidate_n, mcs_changed,
            current_tick=self._tick_counter,
        ):
            new_k, new_n = candidate_k, candidate_n
            self._emit_gate.commit(new_k, new_n, self._tick_counter)
        else:
            new_k = self._emit_gate.last_k or self.cfg.safe.k
            new_n = self._emit_gate.last_n or self.cfg.safe.n

        # Bitrate derived from the EMITTED (k, n) — they ride together.
        new_bitrate_kbps = compute_bitrate_kbps(
            wire_target_kbps=wire_target_kbps,
            k=new_k, n=new_n,
            min_bitrate_kbps=self.cfg.bitrate.min_bitrate_kbps,
            max_bitrate_kbps=self.cfg.bitrate.max_bitrate_kbps,
        )

        self._tick_counter += 1

        mtu_for_predictor = (
            self.drone_config.mtu_bytes
            if self.drone_config and self.drone_config.is_synced()
            else 1400
        )
        ipi_ms = _ipi_ms_for_encoder(float(new_bitrate_kbps), mtu_for_predictor)
        # Per-tick predictor cfg with the live ipi_ms.
        predictor_cfg = PredictorConfig(
            per_packet_airtime_us=self.cfg.predictor.per_packet_airtime_us,
            inter_packet_interval_ms=ipi_ms,
            fec_decode_ms=self.cfg.predictor.fec_decode_ms,
            block_duration_ms=self.cfg.predictor.block_duration_ms,
        )

        new_depth = self.trailing.tick(
            signals, self.state.depth, ts_ms,
            interleaving_supported=(
                self.drone_config.interleaving_supported
                if self.drone_config is not None
                else True
            ),
        )

        # Latency-budget gate — defensive last-resort. Runs against
        # the dynamically computed `(k, n)` from `dynamic_fec` plus
        # whatever depth the trailing loop just picked; fires when
        # the combined block-decode + interleaver cost overshoots
        # the cap. On budget exhaustion we hold the previous state
        # rather than silently rewriting `(k, n)` — the bench showed
        # reactive `(k, n)` rewrites are costly.
        proposal = Proposal(k=new_k, n=new_n, depth=new_depth)
        reason_budget = ""
        try:
            adjusted = fit_or_degrade(
                proposal, self.cfg.max_latency_ms, predictor_cfg,
            )
            if adjusted != proposal:
                reason_budget = (
                    f"budget_degrade {proposal}->{adjusted}"
                )
            new_k, new_n, new_depth = adjusted.k, adjusted.n, adjusted.depth
        except BudgetExhausted:
            reason_budget = "budget_exhausted"
            new_k, new_n, new_depth = (
                self.state.k, self.state.n, self.state.depth
            )

        # Recompute bitrate from the final (k, n) after the budget gate
        # may have reverted them. If BudgetExhausted did not fire this is
        # a no-op; if it did, the Decision carries a bitrate consistent
        # with the held (k, n) rather than the candidate that was rejected.
        # ipi_ms above used the candidate bitrate — one-tick stale on
        # budget-exhausted paths, a pre-existing circular dependency.
        new_bitrate_kbps = compute_bitrate_kbps(
            wire_target_kbps=wire_target_kbps,
            k=new_k, n=new_n,
            min_bitrate_kbps=self.cfg.bitrate.min_bitrate_kbps,
            max_bitrate_kbps=self.cfg.bitrate.max_bitrate_kbps,
        )

        # Commit new state.
        self.state.mcs = row.mcs
        self.state.tx_power_dBm = int(round(tx_power))
        self.state.k = new_k
        self.state.n = new_n
        self.state.depth = new_depth
        self.state.bitrate_kbps = new_bitrate_kbps

        # Assemble Decision.
        knobs_changed: list[str] = []
        if self.state.mcs != prev.mcs:
            knobs_changed.append("mcs")
        if self.state.bitrate_kbps != prev.bitrate_kbps:
            knobs_changed.append("bitrate")
        if self.state.tx_power_dBm != prev.tx_power_dBm:
            knobs_changed.append("tx_power")
        if (self.state.k, self.state.n) != (prev.k, prev.n):
            knobs_changed.append("fec")
        if self.state.depth != prev.depth:
            knobs_changed.append("depth")

        reasons = self.leading.reasons + self.trailing.reasons
        if reason_budget:
            reasons.append(reason_budget)

        return Decision(
            timestamp=signals.timestamp,
            mcs=self.state.mcs,
            bandwidth=self.state.bandwidth,
            tx_power_dBm=self.state.tx_power_dBm,
            k=self.state.k,
            n=self.state.n,
            depth=self.state.depth,
            bitrate_kbps=self.state.bitrate_kbps,
            reason="; ".join(reasons),
            knobs_changed=knobs_changed,
            signals_snapshot={
                "rssi": signals.rssi,
                "rssi_min_w": signals.rssi_min_w,
                "rssi_max_w": signals.rssi_max_w,
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "burst_rate": signals.burst_rate,
                "holdoff_rate": signals.holdoff_rate,
                "packet_rate_w": signals.packet_rate_w,
                "link_starved_w": signals.link_starved_w,
            },
        )
