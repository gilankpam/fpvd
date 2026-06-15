"""Tests for the probe-driven LeadingSelector: probe-promote (climb one
MCS step when the current+1 probe rung reads clean+fresh for
promote_debounce_windows consecutive ticks) + reactive demote (Channel-B
emergency loss/fec/starvation, or a video-PER breach)."""
from __future__ import annotations

import pytest

from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.policy import (
    LeadingSelector,
    Policy,
    PolicyConfig,
    SelectorConfig,
)

# Defaults below are deliberately friendly to testing — timing knobs are
# small (<= the 1000 ms tick spacing the promote tests use) so the climb
# isn't gated by the rate limiter. Where a test wants a specific timing
# path it overrides explicitly.


def _selector(*,
              probe_viable_threshold: float = 0.99,
              probe_freshness_ms: float = 500.0,
              promote_debounce_windows: int = 3,
              video_demote_per: float = 0.05,
              emergency_fec_pressure: float = 0.80,
              max_mcs: int = 7,
              hold_modes_down_ms: int = 0,
              min_between_changes_ms: int = 0,
              starvation_windows: int = 5,
              ) -> LeadingSelector:
    return LeadingSelector(SelectorConfig(
        probe_viable_threshold=probe_viable_threshold,
        probe_freshness_ms=probe_freshness_ms,
        promote_debounce_windows=promote_debounce_windows,
        video_demote_per=video_demote_per,
        emergency_fec_pressure=emergency_fec_pressure,
        max_mcs=max_mcs,
        hold_modes_down_ms=hold_modes_down_ms,
        min_between_changes_ms=min_between_changes_ms,
        starvation_windows=starvation_windows,
    ))


def _probe(viable_mcs, *, per=0.0, age_ms=0.0):
    """Probe snapshot where every rung up to viable_mcs reads clean+fresh,
    and rungs above it are cliffed (per=1.0)."""
    mcs = {}
    for m in range(0, 8):
        p = per if m <= viable_mcs else 1.0
        mcs[str(m)] = {"per": p, "rssi": -60, "snr": 20, "windows": 50,
                       "ageMs": age_ms}
    return {"running": True, "streams": 1, "mcs": mcs}


def _select(s, *, probe=None, loss=0.0, loss_demote=False, fec=0.0,
            link_starved=False, ts_ms=0.0):
    return s.select(probe=probe if probe is not None else _probe(7),
                    loss_rate=loss, loss_demote=loss_demote, fec_pressure=fec,
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
        mcs, _ = _select(s, probe=_probe(5), ts_ms=ts)
        last = mcs
    assert last == start + 1 or last > start   # climbed at least one rung


def test_does_not_promote_on_single_clean_blip():
    s = _selector(max_mcs=5, promote_debounce_windows=3)
    start = s.state.current_mcs
    mcs, changed = _select(s, probe=_probe(5), ts_ms=1000.0)  # 1 window only
    assert mcs == start and not changed


def test_stops_climbing_at_ceiling():
    s = _selector(max_mcs=3, promote_debounce_windows=1)
    ts = 0.0
    for _ in range(20):
        ts += 1000.0
        mcs, _ = _select(s, probe=_probe(3), ts_ms=ts)
    assert s.state.current_mcs == 3   # cliffed above 3, won't exceed


def test_no_promote_when_probe_stale():
    s = _selector(max_mcs=5, promote_debounce_windows=1, probe_freshness_ms=500)
    start = s.state.current_mcs
    ts = 0.0
    for _ in range(5):
        ts += 1000.0
        mcs, _ = _select(s, probe=_probe(5, age_ms=999.0), ts_ms=ts)  # stale
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
        mcs, changed = s.select(
            probe=None, loss_rate=0.0, fec_pressure=0.0,
            link_starved=False, ts_ms=ts,
        )
    assert s.state.current_mcs == start


# ── Reactive demote: Channel-B emergency ────────────────────────────────────


def test_loss_demote_one_step():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, changed = _select(s, loss=0.06, loss_demote=True, ts_ms=99999.0)
    assert changed and mcs == pre - 1


def test_emergency_fec_pressure_demotes_one_step():
    s = _selector(emergency_fec_pressure=0.80, max_mcs=5,
                  promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, changed = _select(s, fec=0.85, ts_ms=99999.0)
    assert changed and mcs == pre - 1


def test_emergency_link_starved_demotes_one_step():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, changed = _select(s, link_starved=True, ts_ms=99999.0)
    assert changed and mcs == pre - 1


def test_loss_not_demoted_when_loss_demote_false():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    pre = s.state.current_mcs
    mcs, changed = _select(s, loss=0.06, loss_demote=False, ts_ms=99999.0)
    assert not changed and mcs == pre


def test_emergency_at_mcs0_cannot_force_below():
    """Already at MCS 0 — emergency has nowhere to go, no change."""
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    ts = 0.0
    while s.state.current_mcs > 0:
        ts += 1000.0
        _select(s, link_starved=True, ts_ms=ts)
    ts += 1000.0
    mcs, changed = _select(s, link_starved=True, ts_ms=ts)
    assert not changed
    assert mcs == 0


# ── Reactive demote: video-PER breach ───────────────────────────────────────


def test_loss_demote_reason_is_video_per_not_emergency():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    _select(s, loss=0.07, loss_demote=True, ts_ms=99999.0)
    assert any("video_per_demote" in r for r in s._reasons)
    assert not any("emergency" in r for r in s._reasons)


def test_strong_rssi_does_not_raise_mcs_without_probe_or_prior(tmp_path):
    """The RSSI cold-start seed was removed: a strong RSSI alone must NOT
    raise the operating MCS. With no probe data (select() cannot promote on
    its own) and a cold prior (empty persist_dir → no warm-start seed), the
    first tick stays at the boot MCS (1); in production the probe climbs from
    there."""
    from fpvdgs.dynlink.signals import Signals

    cfg = PolicyConfig(learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path)))
    # No probe_status → selector can never promote; cold prior (empty dir) →
    # no warm-start seed. Any MCS > boot would have to come from the removed
    # RSSI cold-start.
    policy = Policy(cfg, "test-cold-prior")  # cold: persist_dir is an empty tmp_path, so no warm-start seed

    strong_signals = Signals(
        rssi=-50.0, rssi_min_w=-50.0, rssi_max_w=-50.0,
        residual_loss_w=0.0, fec_work=0.0,
        timestamp=1.0, link_starved_w=False,
    )
    decision = policy.tick(strong_signals)
    assert decision.mcs == 1, (
        f"strong RSSI must not raise MCS without probe/prior data, "
        f"got {decision.mcs}"
    )
