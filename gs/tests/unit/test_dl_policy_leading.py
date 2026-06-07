"""Tests for the probe-driven LeadingSelector: probe-promote (climb one
MCS step when the current+1 probe rung reads clean+fresh for
promote_debounce_windows consecutive ticks) + reactive demote (Channel-B
emergency loss/fec/starvation, or a video-PER breach) + inverse
MCS<->TX-power coupling."""
from __future__ import annotations

from pathlib import Path

import pytest

from fpvdgs.dynlink.policy import (
    GateConfig,
    LeadingLoopConfig,
    LeadingSelector,
    Policy,
    PolicyConfig,
    ProfileSelectionConfig,
    SafeDefaults,
)
from fpvdgs.dynlink.profile import load_profile

PACKAGED_DIR = Path(__file__).resolve().parents[2] / "fpvdgs" / "dynlink" / "profiles"

# Defaults below are deliberately friendly to testing — timing knobs are
# small (<= the 1000 ms tick spacing the promote tests use) so the climb
# isn't gated by the rate limiter. Where a test wants a specific timing
# path it overrides explicitly.


def _selector(*,
              # leading
              tx_power_min_dBm: float = 18.0,
              tx_power_max_dBm: float = 28.0,
              bandwidth: int = 20,
              # gate overrides (probe-driven)
              probe_viable_threshold: float = 0.99,
              probe_freshness_ms: float = 500.0,
              promote_debounce_windows: int = 3,
              video_demote_per: float = 0.05,
              emergency_loss_rate: float = 0.05,
              emergency_fec_pressure: float = 0.80,
              max_mcs: int = 7,
              max_mcs_step_up: int = 1,
              # selection overrides — kept small so promotes aren't
              # rate-limited across 1000 ms-spaced ticks.
              hold_fallback_mode_ms: int = 0,
              hold_modes_down_ms: int = 0,
              min_between_changes_ms: int = 0,
              fast_downgrade: bool = True,
              upward_confidence_loops: int = 1,
              ) -> LeadingSelector:
    profile = load_profile("m8812eu2", [PACKAGED_DIR])
    leading = LeadingLoopConfig(
        bandwidth=bandwidth,
        tx_power_min_dBm=tx_power_min_dBm,
        tx_power_max_dBm=tx_power_max_dBm,
    )
    gate = GateConfig(
        probe_viable_threshold=probe_viable_threshold,
        probe_freshness_ms=probe_freshness_ms,
        promote_debounce_windows=promote_debounce_windows,
        video_demote_per=video_demote_per,
        emergency_loss_rate=emergency_loss_rate,
        emergency_fec_pressure=emergency_fec_pressure,
        max_mcs=max_mcs,
        max_mcs_step_up=max_mcs_step_up,
    )
    sel = ProfileSelectionConfig(
        hold_fallback_mode_ms=hold_fallback_mode_ms,
        hold_modes_down_ms=hold_modes_down_ms,
        min_between_changes_ms=min_between_changes_ms,
        fast_downgrade=fast_downgrade,
        upward_confidence_loops=upward_confidence_loops,
    )
    return LeadingSelector(leading, gate, sel, profile)


def _probe(viable_mcs, *, per=0.0, age_ms=0.0):
    """Probe snapshot where every rung up to viable_mcs reads clean+fresh,
    and rungs above it are cliffed (per=1.0)."""
    mcs = {}
    for m in range(0, 8):
        p = per if m <= viable_mcs else 1.0
        mcs[str(m)] = {"per": p, "rssi": -60, "snr": 20, "windows": 50,
                       "ageMs": age_ms}
    return {"running": True, "streams": 1, "mcs": mcs}


def _select(s, *, probe=None, loss=0.0, fec=0.0, link_starved=False, ts_ms=0.0):
    return s.select(probe=probe if probe is not None else _probe(7),
                    loss_rate=loss, fec_pressure=fec,
                    link_starved=link_starved, ts_ms=ts_ms)


def _drive_to_mcs_probe(s, target, max_ticks=400):
    ts = 0.0
    for _ in range(max_ticks):
        ts += 1000.0
        s.select(probe=_probe(7), loss_rate=0.0, fec_pressure=0.0,
                 link_starved=False, ts_ms=ts)
        if s.state.current_mcs >= target:
            break
    return s.state.current_mcs


# ── Initial state ───────────────────────────────────────────────────────────


def test_starts_at_safe_default_mcs1():
    s = _selector()
    assert s.state.current_mcs == 1


def test_initial_tx_power_at_max():
    s = _selector(tx_power_min_dBm=18.0, tx_power_max_dBm=28.0)
    assert s.state.tx_power_dBm == 28.0


def test_max_mcs_too_low_raises():
    with pytest.raises(ValueError, match="max_mcs"):
        _selector(max_mcs=-1)


# ── Probe-driven promote ────────────────────────────────────────────────────


def test_promotes_one_step_when_next_rung_clean_after_debounce():
    s = _selector(max_mcs=5, promote_debounce_windows=3,
                  probe_viable_threshold=0.99, probe_freshness_ms=500)
    start = s.state.current_mcs
    ts = 0.0
    last = start
    # need debounce windows of clean current+1, with rate limit satisfied
    for _ in range(8):
        ts += 1000.0
        mcs, _, _ = _select(s, probe=_probe(5), ts_ms=ts)
        last = mcs
    assert last == start + 1 or last > start   # climbed at least one rung


def test_does_not_promote_on_single_clean_blip():
    s = _selector(max_mcs=5, promote_debounce_windows=3)
    start = s.state.current_mcs
    mcs, _, changed = _select(s, probe=_probe(5), ts_ms=1000.0)  # 1 window only
    assert mcs == start and not changed


def test_stops_climbing_at_ceiling():
    s = _selector(max_mcs=3, promote_debounce_windows=1)
    ts = 0.0
    for _ in range(20):
        ts += 1000.0
        mcs, _, _ = _select(s, probe=_probe(3), ts_ms=ts)
    assert s.state.current_mcs == 3   # cliffed above 3, won't exceed


def test_no_promote_when_probe_stale():
    s = _selector(max_mcs=5, promote_debounce_windows=1, probe_freshness_ms=500)
    start = s.state.current_mcs
    ts = 0.0
    for _ in range(5):
        ts += 1000.0
        mcs, _, _ = _select(s, probe=_probe(5, age_ms=999.0), ts_ms=ts)  # stale
    assert s.state.current_mcs == start


def test_no_promote_when_next_rung_cliffed():
    """current+1 reads per=1.0 (cliffed) — debounce never accumulates."""
    s = _selector(max_mcs=7, promote_debounce_windows=1)
    start = s.state.current_mcs
    ts = 0.0
    for _ in range(10):
        ts += 1000.0
        _select(s, probe=_probe(start), ts_ms=ts)  # only up to start is clean
    assert s.state.current_mcs == start


def test_no_promote_without_probe():
    """No probe snapshot (None) → can't promote, but doesn't error."""
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    start = s.state.current_mcs
    ts = 0.0
    for _ in range(5):
        ts += 1000.0
        mcs, _, changed = s.select(
            probe=None, loss_rate=0.0, fec_pressure=0.0,
            link_starved=False, ts_ms=ts,
        )
    assert s.state.current_mcs == start


# ── Reactive demote: Channel-B emergency ────────────────────────────────────


def test_emergency_loss_still_demotes_one_step():
    s = _selector(emergency_loss_rate=0.05, max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, _, changed = _select(s, loss=0.06, ts_ms=99999.0)
    assert changed and mcs == pre - 1


def test_emergency_fec_pressure_demotes_one_step():
    s = _selector(emergency_fec_pressure=0.80, max_mcs=5,
                  promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, _, changed = _select(s, fec=0.85, ts_ms=99999.0)
    assert changed and mcs == pre - 1


def test_emergency_link_starved_demotes_one_step():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, _, changed = _select(s, link_starved=True, ts_ms=99999.0)
    assert changed and mcs == pre - 1


def test_emergency_below_threshold_no_demote():
    """loss=0.04 (below 0.05 default, also below video_demote_per=0.05) →
    no emergency, MCS holds."""
    s = _selector(emergency_loss_rate=0.05, video_demote_per=0.05, max_mcs=5,
                  promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, _, changed = _select(s, loss=0.04, ts_ms=99999.0)
    assert not changed
    assert mcs == pre


def test_emergency_at_mcs0_cannot_force_below():
    """Already at MCS 0 — emergency has nowhere to go, no change."""
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    ts = 0.0
    while s.state.current_mcs > 0:
        ts += 1000.0
        _select(s, link_starved=True, ts_ms=ts)
    ts += 1000.0
    mcs, _, changed = _select(s, link_starved=True, ts_ms=ts)
    assert not changed
    assert mcs == 0


# ── Reactive demote: video-PER breach ───────────────────────────────────────


def test_video_per_breach_demotes():
    # Separate the thresholds so ONLY the video-PER branch can fire
    # (emergency_loss_rate high enough that _emergency_active stays False).
    s = _selector(video_demote_per=0.03, emergency_loss_rate=0.50,
                  max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    # loss between video_demote_per (0.03) and emergency_loss_rate (0.50):
    # emergency stays inactive, video-PER breach fires.
    mcs, _, changed = _select(s, loss=0.04, ts_ms=99999.0)
    assert changed and mcs == pre - 1


# ── Inverse MCS<->TX power coupling ─────────────────────────────────────────


def test_tx_power_at_mcs0_is_max():
    s = _selector(tx_power_min_dBm=18.0, tx_power_max_dBm=28.0, max_mcs=5,
                  promote_debounce_windows=1)
    ts = 0.0
    while s.state.current_mcs > 0:
        ts += 1000.0
        _select(s, link_starved=True, ts_ms=ts)
    assert s.state.current_mcs == 0
    assert int(round(s.state.tx_power_dBm)) == 28


def test_tx_power_at_max_mcs_is_min():
    s = _selector(tx_power_min_dBm=18.0, tx_power_max_dBm=28.0, max_mcs=5,
                  promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    assert s.state.current_mcs == 5
    assert int(round(s.state.tx_power_dBm)) == 18


def test_tx_power_jumps_atomically_on_emergency_drop():
    s = _selector(tx_power_min_dBm=18.0, tx_power_max_dBm=28.0, max_mcs=5,
                  promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    assert int(round(s.state.tx_power_dBm)) == 18
    # Single emergency tick → MCS 5 → 4 → power follows in same tick.
    _, _, changed = _select(s, loss=0.10, ts_ms=99999.0)
    assert changed
    assert s.state.current_mcs == 4
    # MCS4 with cap=5, range [18,28]: power = 28 - (4/5)*10 = 20.
    assert int(round(s.state.tx_power_dBm)) == 20


# ── Policy-level: drone_config safe-defaults gate (P4a Task 10) ────────────

def test_policy_emits_safe_defaults_until_drone_synced():
    """Until the drone has sent a HELLO, the policy must emit a
    safe-defaults decision regardless of the incoming signals."""
    from fpvdgs.dynlink.drone_config import DroneConfigState
    from fpvdgs.dynlink.signals import Signals
    from fpvdgs.dynlink.stats_client import SessionInfo
    from fpvdgs.dynlink.wire import Hello

    profile = load_profile("m8812eu2", [PACKAGED_DIR])
    cfg = PolicyConfig(
        leading=LeadingLoopConfig(tx_power_min_dBm=5.0, tx_power_max_dBm=30.0),
        safe=SafeDefaults(k=8, n=12, depth=1, mcs=1),
    )
    drone_cfg = DroneConfigState()
    policy = Policy(cfg, profile, drone_config=drone_cfg)

    # Healthy signals that would otherwise drive normal MCS selection.
    signals = Signals(
        rssi=-55.0, rssi_min_w=-55.0, rssi_max_w=-55.0,
        residual_loss_w=0.0, fec_work=0.0,
        timestamp=0.0,
        link_starved_w=False,
        session=SessionInfo(
            fec_type="VDM_RS", fec_k=8, fec_n=12, epoch=1,
            interleave_depth=1, contract_version=2,
        ),
    )

    decision_before = policy.tick(signals)
    assert decision_before.mcs == cfg.safe.mcs
    assert decision_before.k == cfg.safe.k
    assert decision_before.n == cfg.safe.n
    assert decision_before.depth == cfg.safe.depth
    assert decision_before.reason == "awaiting_drone_config"

    # Synthesize a HELLO; policy should now emit "real" decisions.
    drone_cfg.on_hello(Hello(generation_id=1, mtu_bytes=1400, fps=60))
    assert drone_cfg.is_synced()
    decision_after = policy.tick(signals)
    # The real path runs; reason is no longer the gate sentinel.
    assert decision_after.reason != "awaiting_drone_config"
