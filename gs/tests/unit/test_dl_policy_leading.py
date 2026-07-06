"""LeadingSelector — probe-less promote routes, failure classifier, damper."""

from dataclasses import replace

import pytest

from fpvdgs.dynlink.policy import LeadingSelector, SelectorConfig

T0 = 1_000_000.0  # ms
TICK = 100.0


def mk(**over):
    return LeadingSelector(replace(SelectorConfig(), **over))


def step(
    s,
    ts,
    *,
    snr=30.0,
    snr_w=None,
    slope=0.0,
    loss=0.0,
    loss_demote=False,
    confident=False,
    blocked=False,
    fec=0.0,
    starved=False,
    can_demote=True,
):
    return s.select(
        snr_ewma=snr,
        snr_w=snr if snr_w is None else snr_w,
        slope=slope,
        loss_rate=loss,
        loss_demote=loss_demote,
        target_confident=confident,
        target_blocked=blocked,
        fec_pressure=fec,
        link_starved=starved,
        can_demote=can_demote,
        ts_ms=ts,
    )


def climb_one(s, ts, *, snr=30.0, confident=False, blocked=False):
    """Dwell clean until the selector takes one explore/knee promote; returns ts."""
    start = s.state.current_mcs
    for _ in range(200):
        ts += TICK
        step(s, ts, snr=snr, confident=confident, blocked=blocked)
        if s.state.current_mcs != start:
            return ts
    raise AssertionError("selector never promoted")


# ---- promote routes ------------------------------------------------------


def test_explore_promote_cold_target_after_dwell():
    s = mk()
    ts = T0
    ts = climb_one(s, ts)  # 1 -> 2, cold target => explore route
    assert s.state.current_mcs == 2
    assert any("explore_promote" in r for r in s.reasons)


def test_knee_promote_requires_headroom():
    s = mk()
    ts = T0
    # confident target, blocked (snr below knee+margin) => never promotes
    for _ in range(100):
        ts += TICK
        step(s, ts, confident=True, blocked=True)
    assert s.state.current_mcs == 1
    # headroom appears => knee_promote
    ts = climb_one(s, ts, confident=True, blocked=False)
    assert any("knee_promote" in r for r in s.reasons)


def test_no_promote_while_snr_falling():
    s = mk()
    ts = T0
    for _ in range(100):
        ts += TICK
        step(s, ts, slope=-0.2)  # falling hard: below promote_slope_min
    assert s.state.current_mcs == 1


def test_no_promote_without_snr():
    s = mk()
    ts = T0
    for _ in range(100):
        ts += TICK
        s.select(
            snr_ewma=None,
            snr_w=None,
            slope=0.0,
            loss_rate=0.0,
            loss_demote=False,
            target_confident=False,
            target_blocked=False,
            fec_pressure=0.0,
            link_starved=False,
            can_demote=True,
            ts_ms=ts,
        )
    assert s.state.current_mcs == 1


# ---- failure classifier --------------------------------------------------


def test_trial_flap_charges_damper_and_reports_fail():
    s = mk()
    ts = climb_one(s, T0)  # now at 2, on trial
    ts += TICK
    step(s, ts, loss=0.2, loss_demote=True)  # steady SNR, on trial => flap
    assert s.state.current_mcs == 1
    assert s.last_fail == (2, "flap", 30.0)
    assert s._flap_level.get(2) == 1
    assert any("class=flap" in r for r in s.reasons)


def test_trial_fade_charges_damper_and_teaches_raw_snr():
    """2026-07-06 spec A1 (flight 000003): a fade loss on a rung entered by
    promote charges the damper — periodic fades otherwise re-arm snap-back
    damper-free. Knee teaching keeps the fade class (raw snr_w sample)."""
    s = mk()
    ts = climb_one(s, T0)  # at 2, on trial
    ts += TICK
    step(s, ts, snr=25.0, snr_w=12.0, loss=0.3, loss_demote=True)  # collapse >= 4 dB
    assert s.last_fail == (2, "fade", 12.0)
    assert s._flap_level.get(2) == 1  # NEW: probation loss charges regardless of class
    assert any("class=fade" in r for r in s.reasons)


def test_settled_fade_teaches_but_never_charges():
    """A fade at a SETTLED rung still charges nothing: holding altitude
    through an isolated fade and snapping back stays free (spec A1 keeps
    the original design intent where it is valid)."""
    s = mk(max_mcs=2)  # cap so 150 clean ticks don't keep climbing
    ts = climb_one(s, T0)  # at 2
    for _ in range(150):  # outlive the 10 s trial window
        ts += TICK
        step(s, ts)
    ts += TICK
    step(s, ts, snr=25.0, snr_w=12.0, loss=0.3, loss_demote=True)
    assert s.last_fail == (2, "fade", 12.0)  # raw snr_w, NOT the lagging EWMA
    assert s._flap_level.get(2) is None
    assert any("class=fade" in r for r in s.reasons)


def test_settled_burst_charges_nothing_teaches_nothing():
    s = mk(max_mcs=2)  # cap so 150 clean ticks don't keep climbing
    ts = climb_one(s, T0)  # at 2
    for _ in range(150):  # outlive the 10 s trial window
        ts += TICK
        step(s, ts)
    ts += TICK
    step(s, ts, loss=0.2, loss_demote=True)  # steady SNR, settled => burst
    assert s.state.current_mcs == 1
    assert s.last_fail is None
    assert s._flap_level.get(2) is None
    assert any("class=burst" in r for r in s.reasons)


def test_cascade_steps_never_charge_damper():
    s = mk(max_mcs=4)  # cap so 150 clean ticks don't keep climbing above 4
    ts = T0
    for rung in (2, 3, 4):
        ts = climb_one(s, ts)
    for _ in range(150):  # settle at 4 (out of trial)
        ts += TICK
        step(s, ts)
    # steady-SNR burst cascades 4->3->2->1 (cooldown handled by caller: can_demote=True)
    for _ in range(3):
        ts += TICK
        step(s, ts, loss=0.3, loss_demote=True)
    assert s.state.current_mcs == 1
    assert s._flap_level == {}


# ---- damper recovery -----------------------------------------------------


def _flap_at(s, ts, rung_from):
    """Drive one steady-SNR trial flap from rung_from (must be current+1)."""
    ts = climb_one(s, ts)
    assert s.state.current_mcs == rung_from
    ts += TICK
    step(s, ts, loss=0.2, loss_demote=True)
    return ts


def test_backoff_escalates_and_suppresses():
    s = mk()
    ts = _flap_at(s, T0, 2)  # level 1 => 2 s suppression on rung 2
    ts += TICK
    step(s, ts)
    assert s._promote_suppressed is True


def test_snr_release_lifts_suppression_early():
    s = mk()
    ts = _flap_at(s, T0, 2)  # flapped at snr_ewma=30
    ts += TICK
    step(s, ts, snr=33.5)  # >= 30 + flap_snr_release_db (3.0)
    assert s._promote_suppressed is False


def test_time_decay_forgives_levels():
    s = mk(flap_decay_ms=1000)  # fast decay for the test
    ts = _flap_at(s, T0, 2)
    assert s._effective_flap_level(2, ts) == 1
    assert s._effective_flap_level(2, ts + 1000.0) == 0


def test_clean_dwell_on_rung_clears_damper():
    s = mk()
    ts = _flap_at(s, T0, 2)
    # wait out suppression (level 1 = 2 s), re-promote, dwell clean 3 s on rung 2
    ts += 2100.0
    ts = climb_one(s, ts)
    assert s.state.current_mcs == 2
    for _ in range(35):
        ts += TICK
        step(s, ts)
    assert s._flap_level.get(2) is None


# ---- snap-back -----------------------------------------------------------


def _confirm_ladder_to_5(s, ts):
    for _ in range(4):  # 1->2->3->4->5
        ts = climb_one(s, ts)
    for _ in range(35):  # confirm rung 5 (>= promote_dwell_ticks clean)
        ts += TICK
        step(s, ts)
    assert s.state.current_mcs == 5
    return ts


def test_snapback_recovers_fast_after_fade():
    s = mk(max_mcs=5)
    ts = _confirm_ladder_to_5(s, T0)
    # fade: SNR collapses, cascade to 1
    for _ in range(4):
        ts += TICK
        step(s, ts, snr=25.0, snr_w=10.0, loss=0.3, loss_demote=True)
    assert s.state.current_mcs == 1
    # SNR recovers near the confirmed operating point => snap-back, no dwell
    t_rec = ts
    while s.state.current_mcs < 5:
        ts += TICK
        step(s, ts, snr=29.0)
        assert ts - t_rec < 3000.0, "snap-back too slow"
    assert any("snapback_promote" in r for r in s.reasons)


def test_snapback_respects_damper():
    # trial_window_ms=3000 so 35 clean ticks (3500ms) outlive the trial window
    s = mk(max_mcs=3, trial_window_ms=3000)
    ts = T0
    # climb to 3 and confirm
    for _ in range(2):
        ts = climb_one(s, ts)
    for _ in range(35):
        ts += TICK
        step(s, ts)
    # steady-SNR burst at 3 (settled) -> at 2, no charge
    ts += TICK
    step(s, ts, loss=0.2, loss_demote=True)  # burst (settled) -> at 2, no charge
    ts = climb_one(s, ts)  # re-promote to 3 (on trial)
    ts += TICK
    step(s, ts, loss=0.2, loss_demote=True)  # trial flap -> charge rung 3
    assert s._flap_level.get(3) == 1
    # snap-back target is 3 (confirmed) but rung 3 is suppressed => no promote yet
    ts += TICK
    step(s, ts)
    assert s.state.current_mcs == 2
    assert s._promote_suppressed is True


def test_snapback_needs_recovered_snr_and_rising_slope():
    s = mk(max_mcs=5)
    ts = _confirm_ladder_to_5(s, T0)  # confirmed_snr[5] = 30
    for _ in range(4):
        ts += TICK
        step(s, ts, snr=20.0, snr_w=10.0, loss=0.3, loss_demote=True)
    assert s.state.current_mcs == 1
    ts += TICK
    step(s, ts, snr=20.0)  # 20 < 30 - 3 => no snap-back
    assert s.state.current_mcs == 1
    ts += 300.0
    step(s, ts, snr=29.0, slope=-0.1)  # recovered but falling => no snap-back
    assert s.state.current_mcs == 1


def test_confirmation_expires():
    s = mk(max_mcs=5, confirm_ttl_ms=1000)
    ts = _confirm_ladder_to_5(s, T0)
    for _ in range(4):
        ts += TICK
        step(s, ts, snr=25.0, snr_w=10.0, loss=0.3, loss_demote=True)
    ts += 1500.0  # confirmation TTL lapsed
    for _ in range(5):
        ts += TICK
        step(s, ts, snr=29.0)
    # no snap-back: must ladder-climb (dwell) instead — still at 1 after 0.5 s
    assert s.state.current_mcs == 1


def test_confirmed_snr_is_high_water_during_degradation():
    """2026-07-06 spec A2 (flight 000003): during degradation the snap-back
    bar must NOT slide down with the channel. Confirm rung 3 at 30 dB, dwell
    clean at 24 dB (old code re-confirmed at 24), lose the rung: recovery to
    24 dB must not snap back (bar is 30-3=27); 28 dB must."""
    # trial_window_ms=3000: the 35-tick dwell (3.5 s) outlives the trial, so
    # the loss below classifies as settled burst (no damper charge) — the
    # snap-back assertions must not be damper-blocked.
    s = mk(max_mcs=3, trial_window_ms=3000)
    ts = T0
    for _ in range(2):
        ts = climb_one(s, ts)  # 1->2->3
    for _ in range(35):  # confirm rung 3 at snr 30 (and outlive the trial)
        ts += TICK
        step(s, ts)
    for _ in range(35):  # degraded but clean — old code overwrote the bar to 24
        ts += TICK
        step(s, ts, snr=24.0)
    assert s._confirmed_snr[3] == 30.0  # high-water held
    ts += TICK
    step(s, ts, snr=24.0, loss=0.2, loss_demote=True)  # settled burst -> at 2, no charge
    ts += 300.0
    step(s, ts, snr=24.0)  # 24 < 30-3 => no snap-back
    assert s.state.current_mcs == 2
    ts += 300.0
    step(s, ts, snr=28.0)  # 28 >= 27 => snap-back
    assert s.state.current_mcs == 3
    assert any("snapback_promote" in r for r in s.reasons)


def test_confirmed_snr_rebases_after_ttl_lapse():
    """After the confirmation TTL lapses the bar re-bases to current
    conditions (fresh confirmation), it does not stay at the old high-water."""
    s = mk(max_mcs=3, confirm_ttl_ms=1000)
    ts = T0
    for _ in range(2):
        ts = climb_one(s, ts)
    for _ in range(35):  # confirm rung 3 at 30
        ts += TICK
        step(s, ts)
    assert s._confirmed_snr[3] == 30.0
    ts += 1500.0  # TTL (1 s) lapses with no clean dwell extending it
    for _ in range(35):  # fresh confirmation at 24
        ts += TICK
        step(s, ts, snr=24.0)
    assert s._confirmed_snr[3] == 24.0  # re-based, not max(30, 24)


def test_snapback_blocked_by_confident_knee():
    """2026-07-06 spec A3 (flight 000036): snap-back must not re-promote into
    a rung the learned knee says is unviable at the live SNR."""
    # trial_window_ms=3000: the 35-tick dwell (3.5 s) outlives the trial, so
    # the loss classifies as settled burst (no damper charge) and the second
    # snap-back attempt isn't damper-blocked.
    s = mk(max_mcs=3, trial_window_ms=3000)
    ts = T0
    for _ in range(2):
        ts = climb_one(s, ts)
    for _ in range(35):  # confirm rung 3 at snr 30 (and outlive the trial)
        ts += TICK
        step(s, ts)
    ts += TICK
    step(s, ts, loss=0.2, loss_demote=True)  # settled burst -> at 2
    ts += 300.0
    step(s, ts, blocked=True)  # snr recovered BUT knee says rung 3 unviable
    assert s.state.current_mcs == 2
    ts += 300.0
    step(s, ts, blocked=False)  # knee headroom back => snap-back
    assert s.state.current_mcs == 3


# ---- initial state ----------------------------------------------------------


def test_starts_at_safe_default_mcs1():
    s = mk()
    assert s.state.current_mcs == 1


def test_max_mcs_too_low_raises():
    with pytest.raises(ValueError, match="max_mcs"):
        mk(max_mcs=-1)


# ---- emergency / reactive demote --------------------------------------------


def test_emergency_fec_pressure_demotes_one_step():
    s = mk()
    ts = climb_one(s, T0)  # 1 -> 2
    pre = s.state.current_mcs
    ts += TICK
    mcs, changed = step(s, ts, fec=0.85)
    assert changed and mcs == pre - 1
    assert any("emergency" in r for r in s.reasons)


def test_emergency_link_starved_demotes_one_step():
    s = mk()
    ts = climb_one(s, T0)  # 1 -> 2
    pre = s.state.current_mcs
    ts += TICK
    mcs, changed = step(s, ts, starved=True)
    assert changed and mcs == pre - 1
    assert any("emergency" in r for r in s.reasons)


def test_emergency_at_mcs0_cannot_force_below():
    """Already at MCS 0 — emergency has nowhere to go, no change."""
    s = mk()
    ts = T0
    while s.state.current_mcs > 0:
        ts += TICK
        step(s, ts, starved=True)
    ts += TICK
    mcs, changed = step(s, ts, starved=True)
    assert not changed
    assert mcs == 0


def test_loss_demote_blocked_when_cooldown_not_elapsed():
    s = mk()
    ts = climb_one(s, T0)  # 1 -> 2
    pre = s.state.current_mcs
    ts += TICK
    mcs, changed = step(s, ts, loss=0.06, loss_demote=True, can_demote=False)
    assert mcs == pre and changed is False


def test_emergency_blocked_when_cooldown_not_elapsed():
    s = mk()
    ts = climb_one(s, T0)  # 1 -> 2
    pre = s.state.current_mcs
    ts += TICK
    mcs, changed = step(s, ts, fec=0.9, can_demote=False)
    assert mcs == pre and changed is False


def test_emergency_demote_does_not_strike():
    """Emergency demotes (fec/starved) must not charge the flap-damper
    and must not set last_fail — they are infrastructure events, not
    promote-failure evidence."""
    s = mk()
    ts = climb_one(s, T0)  # 1 -> 2
    pre = s.state.current_mcs
    ts += TICK
    mcs, changed = step(s, ts, fec=0.95)
    assert changed and mcs == pre - 1
    assert s._flap_level == {}  # emergency never charges the damper
    assert s.last_fail is None  # emergency never teaches a failure


def test_external_demote_blocks_same_and_next_tick_promote():
    """2026-07-06 spec A4 (flight 000036): a Policy-level (predict/snr) demote
    must update the rate-limit clock so select() cannot snap back within
    min_between_changes_ms of it."""
    s = mk(max_mcs=3)
    ts = T0
    for _ in range(2):
        ts = climb_one(s, ts)
    for _ in range(35):  # confirm rung 3 at snr 30
        ts += TICK
        step(s, ts)
    s.external_demote(2, ts)
    assert s.state.current_mcs == 2
    assert s.state.last_change_time_ms == ts
    step(s, ts, snr=30.0)  # same tick: inside min_between_changes (200 ms)
    assert s.state.current_mcs == 2
    step(s, ts + 100.0, snr=30.0)  # still inside
    assert s.state.current_mcs == 2
    step(s, ts + 300.0, snr=30.0)  # rate limit elapsed => snap-back allowed
    assert s.state.current_mcs == 3


def test_backoff_ladder_caps_at_10s():
    """2026-07-06 spec A5: balanced pacing — a rung is never locked out
    longer than 10 s at a time (was 30 s)."""
    s = mk()
    assert s._flap_backoff_ms(1) == 2000
    assert s._flap_backoff_ms(2) == 4000
    assert s._flap_backoff_ms(3) == 8000
    assert s._flap_backoff_ms(4) == 10000
    assert s._flap_backoff_ms(10) == 10000
