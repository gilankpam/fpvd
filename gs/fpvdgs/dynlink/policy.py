"""Policy engine — probe-driven MCS selector (§4).

Runs at 10 Hz, one tick per RxEvent. Pure function of
(Signals snapshot, internal hysteresis state). Emits a `{mcs}`-only
Decision on every tick.

Phase 3b: the drone computes its own bitrate / FEC / depth / tx_power
locally, so the GS no longer composes any of that. The selector (Phase 2)
is the only decision: probe-promote + reactive demote, with an RSSI
cold-start seed and starvation hysteresis feeding the emergency demote.
"""
from __future__ import annotations

import logging
from dataclasses import dataclass, field

from .decision import Decision
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
class SafeDefaults:
    mcs: int = 1


@dataclass
class PolicyConfig:
    leading: LeadingLoopConfig = field(default_factory=LeadingLoopConfig)
    gate: GateConfig = field(default_factory=GateConfig)
    selection: ProfileSelectionConfig = field(
        default_factory=ProfileSelectionConfig
    )
    safe: SafeDefaults = field(default_factory=SafeDefaults)
    # Total-blackout failsafe: this many consecutive starved windows
    # (packet_rate_w < starvation_threshold while session active) feeds
    # the selector's link_starved emergency demote. Intentionally short —
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
    computed downstream by the drone.

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
# Top-level policy: runs the selector, emits a {mcs}-only Decision.
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


class Policy:
    """Runs the probe-driven dual-gate selector and emits the
    `{mcs}`-only Decision."""

    def __init__(
        self,
        cfg: PolicyConfig,
        profile: RadioProfile,
        *,
        probe_status=None,
    ) -> None:
        self.cfg = cfg
        self.profile = profile
        # Probe snapshot provider (zero-arg callable returning the
        # ProbeController.status() dict, or None). The selector promotes
        # MCS only when the probed current+1 rung reads clean+fresh. When
        # left None (e.g. tests / no probe) the selector can never
        # promote — it only reacts to emergencies.
        self._probe_status = probe_status
        # Cold-start one-shot: seed the operating MCS from the single
        # link-RSSI via a coarse table on the first tick where RSSI is
        # present, so the first real decision isn't stuck at the safe
        # floor while the probe warms up. Flipped True after the single
        # seed; the probe-driven select() owns MCS thereafter.
        self._cold_started = False
        self.leading = LeadingSelector(
            cfg.leading, cfg.gate, cfg.selection, profile
        )
        # Per-window link_starved_w can flicker on brief packet-rate
        # dips inside an otherwise-healthy bursty stream. Require N
        # consecutive starved windows before treating the link as
        # actually starved for the selector's emergency channel —
        # loss/FEC pressure remain direct triggers (those are real
        # glitches). At 10 Hz, starvation_windows=5 = 0.5 s of below-
        # threshold packet rate before declaring blackout.
        self._starvation_count: int = 0
        # Boot at the leading selector's chosen row.
        row = self.leading.current_row
        self.state = PolicyState(mcs=row.mcs)

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

        # Selector (Phase 2) is the only decision now: probe-promote +
        # reactive demote. The drone computes its own bitrate / FEC /
        # depth / tx_power locally, so we emit {mcs} only.
        new_mcs, _tx, _changed = self.leading.select(
            probe=self._probe_status() if self._probe_status else None,
            loss_rate=signals.residual_loss_w,
            fec_pressure=signals.fec_work,
            link_starved=sustained_starved,
            ts_ms=ts_ms,
        )
        self.state.mcs = new_mcs

        return Decision(
            timestamp=signals.timestamp,
            mcs=new_mcs,
            reason="; ".join(self.leading.reasons) if self.leading.reasons else "",
            signals_snapshot={
                "rssi": signals.rssi,
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "link_starved": sustained_starved,
                "mcs": new_mcs,
            },
        )
