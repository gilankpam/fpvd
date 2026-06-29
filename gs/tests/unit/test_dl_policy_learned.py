"""Phase 4 integration: warm-start + predictive-demote + regression."""

from __future__ import annotations

from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.policy import Policy, PolicyConfig
from fpvdgs.dynlink.signals import Signals


def _profile():
    return "m8812eu2"  # Policy takes the adapter id string (learned-prior key)


def _cfg(tmp_path, **lp):
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path), **lp),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def _sig(rssi, ts=1.0):
    return Signals(rssi=rssi, residual_loss_w=0.0, fec_work=0.0, link_starved_w=False, timestamp=ts)


def test_warm_start_seeds_from_persisted_curve(tmp_path):
    prof = _profile()
    p1 = Policy(_cfg(tmp_path, min_samples=3), prof)
    for _ in range(12):  # dirty SNR samples plant the rung-5 knee
        p1.learned_prior.ingest(snr=36.0, operating_mcs=5, operating_clean=False, settled=True)
    p1.close()
    p2 = Policy(_cfg(tmp_path, min_samples=3), prof)
    dec = p2.tick(_sig_snr(36.0, ts=1.0))
    assert dec.mcs == 5  # warm-started from the persisted SNR knee
    p2.close()


def test_unknown_curve_no_seed_stays_at_boot(tmp_path):
    p = Policy(_cfg(tmp_path, min_samples=100), _profile())
    dec = p.tick(_sig_snr(36.0, ts=1.0))
    assert dec.mcs == 1  # cold prior -> boot MCS
    p.close()


def test_predictive_demote_on_confident_fade(tmp_path):
    prof = _profile()
    p = Policy(
        _cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3, predictive_debounce_windows=1),
        prof,
    )
    # Confident SNR knees for rungs 1, 2, 5.
    p.learned_prior._snr_model._knee[1] = 20.0
    p.learned_prior._snr_model._count[1] = 12.0
    p.learned_prior._snr_model._knee[2] = 26.0
    p.learned_prior._snr_model._count[2] = 12.0
    p.learned_prior._snr_model._knee[5] = 34.0
    p.learned_prior._snr_model._count[5] = 12.0
    p.leading.state.current_mcs = 5
    p.tick(_sig_snr(34.0, ts=1.0))  # slope 0 -> no demote yet
    dec = p.tick(_sig_snr(30.0, ts=1.1))  # falling -> predict-demote steps ONE rung 5->4
    assert dec.mcs == 4
    p.close()


def _cfg_fl(tmp_path):
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    from fpvdgs.dynlink.learned_prior import LearnedPriorConfig

    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path / "lp")),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def test_decision_and_flightlog_carry_rssi_raw(tmp_path):
    """Both the decision snapshot and the flight-log record carry the
    measured rssi_raw alongside the normalized rssi."""
    import json

    p = Policy(_cfg(tmp_path), _profile())
    sig = Signals(
        rssi=-55.0,
        rssi_raw=-65.0,
        residual_loss_w=0.0,
        fec_work=0.0,
        link_starved_w=False,
        timestamp=1.0,
    )
    dec = p.tick(sig)
    # Decision snapshot
    assert dec.signals_snapshot["rssi"] == -55.0
    assert dec.signals_snapshot["rssi_raw"] == -65.0
    p.close()  # flushes the flight log

    # Flight-log record
    files = sorted((tmp_path / "fl").glob("*.jsonl"))
    assert files, "expected a flight-log file"
    with open(files[-1]) as f:
        last = [json.loads(line) for line in f if line.strip()][-1]
    assert last["rssi"] == -55.0
    assert last["rssi_raw"] == -65.0


def test_predictive_demote_blocked_when_snr_flat(tmp_path):
    """A confident current-rung knee but FLAT SNR (projected_drop 0 < min_drop)
    must NOT demote — the slope-direction gate (the flapping fix)."""
    prof = _profile()
    p = Policy(
        _cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3, predictive_debounce_windows=2),
        prof,
    )
    # High snr_demote_debounce isolates the predict path: with slope=0 the
    # projected SNR equals the current SNR, so snr_demote would fire via the
    # same condition after its debounce — prevent that so only the predict
    # slope-gate is exercised.
    p.cfg.selector.snr_demote_debounce = 100
    p.learned_prior._snr_model._knee[5] = 38.0  # cur rung unviable at live snr...
    p.learned_prior._snr_model._count[5] = 12.0
    p.leading.state.current_mcs = 5
    dec = None
    for ts in (1.0, 1.1, 1.2, 1.3, 1.4):
        dec = p.tick(_sig_snr(34.0, ts=ts))  # ...but flat -> no real fade
    assert dec.mcs == 5
    p.close()


def test_predictive_demote_blocked_when_fade_too_shallow(tmp_path):
    prof = _profile()
    p = Policy(
        _cfg(
            tmp_path,
            min_samples=3,
            predictive_horizon_ticks=3,
            predictive_debounce_windows=2,
            predictive_min_drop_db=1.0,
        ),
        prof,
    )
    # High snr_demote_debounce isolates the predict path: snr_rung_unviable uses
    # the same knee+margin as predict, so snr_demote would also fire — prevent
    # that so only the predict projected_drop gate is exercised.
    p.cfg.selector.snr_demote_debounce = 100
    p.learned_prior._snr_model._knee[5] = 38.0
    p.learned_prior._snr_model._count[5] = 12.0
    p.leading.state.current_mcs = 5
    dec = None
    for snr, ts in [(34.0, 1.0), (33.8, 1.1), (33.6, 1.2), (33.4, 1.3), (33.2, 1.4)]:
        dec = p.tick(_sig_snr(snr, ts=ts))  # 0.2 dB/tick -> 0.6 dB over horizon < 1.0
    assert dec.mcs == 5
    p.close()


def test_predictive_demote_does_not_collapse_on_cold_ladder(tmp_path):
    """Flight log 000007 regression. A clean flight learns a knee ONLY for the
    rung-0 floor; cold upper rungs must stay explorable. predict_demote must NOT
    walk a clean link down to MCS0 on a no-loss SNR fade."""
    prof = _profile()
    p = Policy(
        _cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3, predictive_debounce_windows=1),
        prof,
    )
    p.learned_prior._snr_model._knee[0] = 6.0  # only the rung-0 floor learned
    p.learned_prior._snr_model._count[0] = 12.0
    p.leading.state.current_mcs = 5
    dec = None
    for i in range(12):  # steady SNR fade, NO loss
        dec = p.tick(_sig_snr(34.0 - i, ts=1.0 + 0.1 * i))
    assert dec.mcs == 5
    p.close()


def test_reset_for_new_session_resets_selector_keeps_prior(tmp_path):
    p = Policy(_cfg(tmp_path), _profile())
    prior_before = p.learned_prior
    # Simulate a session that climbed + accumulated hysteresis state.
    p.leading.state.current_mcs = 5
    p._cold_started = True
    p._starvation_count = 4
    p._snr_demote_count = 2
    p._predict_demote_count = 5
    p._windows_since_demote = 0
    p._ticks_at_mcs = 7
    p.leading._promote_clean = 3

    p.reset_for_new_session()

    assert p.leading.state.current_mcs == 1  # back to the boot MCS
    assert p._cold_started is False  # warm-start will re-run
    assert p._starvation_count == 0
    assert p._snr_demote_count == 0
    assert p._predict_demote_count == 0
    assert p._ticks_at_mcs == 0
    assert p.leading._promote_clean == 0
    assert p.learned_prior is prior_before  # persistent knees preserved
    assert p._windows_since_demote == p.cfg.selector.demote_cooldown_windows


# ── SNR-knee promote/demote hysteresis (the MCS-stuck-at-4 field bug) ─────────
# Repro of flight log 000011: probe rung clean+fresh, RSSI ceiling allows the
# climb, but the live normalized SNR settled 0.066 dB below the target rung's
# learned SNR knee. The zero-margin veto (`snr < knee`) pinned MCS at 4 forever
# — and since the knee only relaxes by OPERATING at the rung, the veto blocked
# its own cure. Two-margin hysteresis (promote margin < demote margin) unsticks
# it without introducing a promote<->proactive-demote flap.


def _probe_snapshot(viable_mcs, *, per=0.0, age_ms=0.0):
    mcs = {}
    for m in range(0, 8):
        pv = per if m <= viable_mcs else 1.0
        mcs[str(m)] = {"per": pv, "snr": 20, "rssi": -60, "windows": 50, "ageMs": age_ms}
    return {"running": True, "streams": 1, "mcs": mcs}


def _settle_snr_knee(policy, rung, snr, clean=True, n=12):
    for _ in range(n):
        policy.learned_prior.ingest(
            snr=snr, operating_mcs=rung, operating_clean=clean, settled=True
        )


def _sig_snr(snr, ts):
    return Signals(
        rssi=None, snr=snr, residual_loss_w=0.0, fec_work=0.0, link_starved_w=False, timestamp=ts
    )


def test_snr_knee_hysteresis_unsticks_stuck_promote(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3), prof, probe_status=lambda: _probe_snapshot(5))
    _settle_snr_knee(p, 4, 34.0)  # knee[4] ~34
    _settle_snr_knee(p, 5, 36.0)  # knee[5] ~36 (the wall)
    p.leading.state.current_mcs = 4
    dec = None
    ts = 1.0
    for _ in range(10):  # > promote_debounce_windows
        dec = p.tick(_sig_snr(35.6, ts))  # 0.4 dB below knee[5]: strict locks
        ts += 1.0
    assert dec.mcs == 5  # margin clears the knife-edge veto
    p.close()


def test_snr_knee_hysteresis_holds_no_flap(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3), prof, probe_status=lambda: _probe_snapshot(5))
    _settle_snr_knee(p, 4, 34.0)
    _settle_snr_knee(p, 5, 36.0)
    p.leading.state.current_mcs = 5  # already climbed onto the rung
    dec = None
    ts = 1.0
    for _ in range(20):  # well past snr_demote_debounce
        dec = p.tick(_sig_snr(35.6, ts))  # dead band: promote-ok AND no demote
        ts += 1.0
    assert dec.mcs == 5  # no proactive-demote flap-down
    p.close()


def test_snr_knee_proactive_demote_still_fires_clearly_below(tmp_path):
    # The demote margin must not disable the proactive SNR-demote: a clear fade
    # (well past knee - demote_margin) still steps down ahead of loss.
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3), prof, probe_status=lambda: _probe_snapshot(5))
    # Direct-set: plant confident SNR knees for rungs 3, 4, 5
    p.learned_prior._snr_model._knee[3] = 30.0
    p.learned_prior._snr_model._count[3] = 12.0
    p.learned_prior._snr_model._knee[4] = 34.0
    p.learned_prior._snr_model._count[4] = 12.0
    p.learned_prior._snr_model._knee[5] = 36.0
    p.learned_prior._snr_model._count[5] = 12.0
    p.leading.state.current_mcs = 5
    dec = None
    ts = 1.0
    for _ in range(6):  # past snr_demote_debounce
        dec = p.tick(_sig_snr(33.0, ts))  # 3 dB below knee[5] -> genuinely unviable
        ts += 1.0
    assert dec.mcs < 5  # proactive SNR-demote fired
    p.close()


def test_bind_learned_prior_rekeys(tmp_path):
    from fpvdgs.dynlink.policy import Policy

    cfg = _cfg(tmp_path)  # existing helper in this file; persist_dir under tmp_path
    p = Policy(cfg)
    assert p.learned_prior.key == "unbound"
    first = p.learned_prior
    p.bind_learned_prior("bl-m8812eu2", 20)
    assert p.learned_prior.key == "bl-m8812eu2__bw20"
    assert p.learned_prior is not first
    # idempotent: same key does not rebuild
    same = p.learned_prior
    p.bind_learned_prior("bl-m8812eu2", 20)
    assert p.learned_prior is same


def test_predictive_demote_paced_by_cooldown(tmp_path):
    prof = _profile()
    p = Policy(
        _cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3, predictive_debounce_windows=1),
        prof,
    )
    for rung in (1, 2, 3, 4, 5):  # every stepped-through rung confidently unviable
        p.learned_prior._snr_model._knee[rung] = 40.0
        p.learned_prior._snr_model._count[rung] = 12.0
    p.leading.state.current_mcs = 5
    p.cfg.selector.demote_cooldown_windows = 3
    p.tick(_sig_snr(36.0, ts=1.0))  # slope baseline
    seq = []
    ts = 1.1
    for _ in range(9):  # sustained deep fade well below the knees
        seq.append(p.tick(_sig_snr(36.0 - (ts * 5), ts=ts)).mcs)
        ts += 0.1
    p.close()
    assert all(0 <= a - b <= 1 for a, b in zip(seq, seq[1:]))  # single-rung steps only
    assert seq[0] == 4  # one step on the first confident fade tick
    assert seq[1] == 4 and seq[2] == 4  # frozen during cooldown (windows 1+2 of 3)
    assert seq[3] == 3  # steps again after cooldown clears


def test_snr_demote_steps_one_rung_and_is_paced(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3), prof, probe_status=lambda: _probe_snapshot(5))
    p.learned_prior._snr_model._knee[5] = 36.0
    p.learned_prior._snr_model._count[5] = 12.0
    p.learned_prior._snr_model._knee[4] = 34.0
    p.learned_prior._snr_model._count[4] = 12.0
    p.leading.state.current_mcs = 5
    p.cfg.selector.demote_cooldown_windows = 3
    seq = []
    ts = 1.0
    for _ in range(9):
        seq.append(p.tick(_sig_snr(33.0, ts)).mcs)  # 3 dB below knee[5] -> unviable
        ts += 1.0
    p.close()
    # snr_demote_debounce=2 -> first step at the 2nd unviable tick, then paced by cooldown.
    assert seq[1] == 4  # one rung, not a jump to snr_ceiling
    assert 4 in seq and min(seq) >= 2  # gradual, never slammed to 0


def test_bind_learned_prior_keys_by_adapter_and_width(tmp_path):
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
    from fpvdgs.dynlink.policy import Policy, PolicyConfig

    cfg = PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path)),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )
    p = Policy(cfg, "unbound")

    p.bind_learned_prior("ABC123", 10)
    assert p.learned_prior.key == "ABC123__bw10"
    p.learned_prior.ingest(operating_mcs=4, operating_clean=True, settled=True)
    p.learned_prior.flush()
    assert (tmp_path / "ABC123__bw10.json").exists()

    # Same adapter, different width -> distinct key + file (no cross-contamination).
    p.bind_learned_prior("ABC123", 20)
    assert p.learned_prior.key == "ABC123__bw20"

    # Re-binding the identical adapter+width is a no-op (keeps the live model).
    same = p.learned_prior
    p.bind_learned_prior("ABC123", 20)
    assert p.learned_prior is same
    p.close()


def test_no_two_demote_paths_stack_in_same_tick(tmp_path):
    """Both snr_demote and reactive loss are active in the SAME tick; the
    current_mcs == start_mcs guard inside snr_demote and select() prevents
    them from stacking — exactly one rung is stepped, not two.

    Setup: snr_demote reaches its debounce count on tick 2 (fires, 5→4).
    Simultaneously, residual_loss_w (0.10) exceeds video_demote_per (0.05),
    so reactive loss also wants to fire in the same tick.  Without the guard,
    select() would step 4→3 for a net drop of 2; with the guard,
    self.leading.state.current_mcs is already 4 != start_mcs (5), so
    can_demote=False and loss is suppressed.  The decision reason names
    snr_demote (not loss) as the sole cause.
    """
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3), prof)

    # Plant a confident SNR knee for rung 5: snr_rung_unviable(5, 33.0, margin=1.5)
    # is True because 33 dB < knee(36) - margin(1.5) = 34.5 dB.
    p.learned_prior._snr_model._knee[5] = 36.0
    p.learned_prior._snr_model._count[5] = 12.0
    p.leading.state.current_mcs = 5

    # Tick 1: SNR unviable → snr_demote_count reaches 1 (< debounce 2); no loss.
    p.tick(
        Signals(
            rssi=None,
            snr=33.0,
            residual_loss_w=0.0,
            fec_work=0.0,
            link_starved_w=False,
            timestamp=1.0,
        )
    )
    assert p.leading.state.current_mcs == 5  # no demote yet

    # Tick 2: both demote paths active simultaneously.
    #   snr_demote: _snr_demote_count reaches 2 == snr_demote_debounce → fires (5→4),
    #               setting current_mcs=4 BEFORE select() is called.
    #   reactive loss: residual_loss_w=0.10 >= video_demote_per=0.05 → loss_demote=True.
    #   Guard: select() receives can_demote = cooldown_ok AND (current_mcs==start_mcs)
    #          = True AND (4==5) = False → reactive loss is blocked.
    # Without the guard the net drop would be 2 (5→4→3); with it, exactly 1 (5→4).
    dec = p.tick(
        Signals(
            rssi=None,
            snr=33.0,
            residual_loss_w=0.10,
            fec_work=0.0,
            link_starved_w=False,
            timestamp=2.0,
        )
    )
    assert dec.mcs == 4, f"expected one step (5→4), got {dec.mcs}"
    assert "snr_demote" in dec.reason  # confirms snr_demote fired; no loss reason appended
    p.close()


def test_snr_ceiling_still_logged_though_not_a_demote_target(tmp_path):
    import json

    from fpvdgs.dynlink.flightlog import FlightLogConfig
    from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
    from fpvdgs.dynlink.policy import PolicyConfig

    cfg = PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path / "lp")),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )
    p = Policy(cfg, _profile())
    p.learned_prior._snr_model._knee[0] = 20.0
    p.learned_prior._snr_model._count[0] = 12.0
    p.tick(_sig_snr(33.0, 1.0))
    p.close()
    files = sorted((tmp_path / "fl").glob("*.jsonl"))
    with open(files[-1]) as f:
        last = [json.loads(line) for line in f if line.strip()][-1]
    assert "snr_ceiling" in last and last["snr_ceiling"] == 0
