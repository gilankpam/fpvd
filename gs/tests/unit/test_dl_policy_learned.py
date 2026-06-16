"""Phase 4 integration: warm-start + predictive-demote + regression."""
from __future__ import annotations

from fpvdgs.dynlink.policy import Policy, PolicyConfig
from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.signals import Signals


def _profile():
    return "m8812eu2"   # Policy takes the radioProfile string (learned-prior key)


def _cfg(tmp_path, **lp):
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path), **lp),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def _sig(rssi, ts=1.0):
    return Signals(rssi=rssi, residual_loss_w=0.0, fec_work=0.0,
                   link_starved_w=False, timestamp=ts)


def _settle_knee(policy, rung, rssi, clean, n=12):
    """Prime the knee model directly via the facade (settled=True), bypassing
    the policy's tick-driven settle gate — used only for test setup."""
    for _ in range(n):
        policy.learned_prior.ingest(rssi=rssi, operating_mcs=rung,
                                    operating_clean=clean, settled=True)


def test_warm_start_seeds_from_persisted_curve(tmp_path):
    prof = _profile()
    p1 = Policy(_cfg(tmp_path, min_samples=3), prof)
    _settle_knee(p1, 5, -50.0, True)
    p1.close()
    p2 = Policy(_cfg(tmp_path, min_samples=3), prof)
    dec = p2.tick(_sig(-50.0, ts=1.0))
    assert dec.mcs == 5                      # warm-started from the persisted knee


def test_unknown_curve_no_seed_stays_at_boot(tmp_path):
    p = Policy(_cfg(tmp_path, min_samples=100), _profile())
    dec = p.tick(_sig(-50.0, ts=1.0))
    assert dec.mcs == 1                      # cold prior -> boot MCS


def test_predictive_demote_on_confident_fade(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=1), prof)
    _settle_knee(p, 1, -80.0, True)          # rung1 viable down to -80
    _settle_knee(p, 2, -62.0, True)          # rung2 viable down to -62
    _settle_knee(p, 5, -50.0, True)          # rung5 viable only >= -50
    p.leading.state.current_mcs = 5
    p.tick(_sig(-50.0, ts=1.0))              # slope 0 -> no demote yet
    dec = p.tick(_sig(-56.0, ts=1.1))        # slope -6, projected -56-18=-74 -> ceiling 1
    assert dec.mcs == 1
    p.close()


def _cfg_fl(tmp_path, flight_gap_s=15.0):
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path / "lp")),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl"), flight_gap_s=flight_gap_s),
    )


def _sig_starved(starved, ts=1.0, rssi=-55.0):
    return Signals(rssi=rssi, residual_loss_w=0.0, fec_work=0.0,
                   link_starved_w=starved, timestamp=ts)


def test_flight_rolls_on_link_gap_recovery(tmp_path, monkeypatch):
    from fpvdgs.dynlink import policy as policy_mod
    clock = {"t": 1000.0}
    monkeypatch.setattr(policy_mod.time, "monotonic", lambda: clock["t"])
    p = Policy(_cfg_fl(tmp_path, flight_gap_s=15.0), _profile())
    p.tick(_sig_starved(False, ts=1.0)); clock["t"] += 0.1     # baseline (no roll on 1st)
    p.tick(_sig_starved(False, ts=1.1))
    clock["t"] += 20.0                                          # link gone 20 s
    p.tick(_sig_starved(True, ts=2.0))                         # starved: baseline frozen
    p.tick(_sig_starved(False, ts=3.0))                        # healthy: 20 > 15 -> ROLL
    p.close()
    assert len(list((tmp_path / "fl").glob("*.jsonl"))) == 2


def test_brief_gap_does_not_roll(tmp_path, monkeypatch):
    from fpvdgs.dynlink import policy as policy_mod
    clock = {"t": 1000.0}
    monkeypatch.setattr(policy_mod.time, "monotonic", lambda: clock["t"])
    p = Policy(_cfg_fl(tmp_path, flight_gap_s=15.0), _profile())
    p.tick(_sig_starved(False, ts=1.0)); clock["t"] += 5.0     # only 5 s gap
    p.tick(_sig_starved(True, ts=2.0))
    p.tick(_sig_starved(False, ts=3.0))                        # 5 < 15 -> no roll
    p.close()
    assert len(list((tmp_path / "fl").glob("*.jsonl"))) == 1


def test_first_healthy_tick_does_not_roll(tmp_path, monkeypatch):
    from fpvdgs.dynlink import policy as policy_mod
    clock = {"t": 9999.0}     # large: would exceed any gap if baseline weren't None
    monkeypatch.setattr(policy_mod.time, "monotonic", lambda: clock["t"])
    p = Policy(_cfg_fl(tmp_path, flight_gap_s=15.0), _profile())
    p.tick(_sig_starved(False, ts=1.0))                        # 1st healthy: None baseline -> no roll
    p.close()
    assert len(list((tmp_path / "fl").glob("*.jsonl"))) == 1


def test_decision_and_flightlog_carry_rssi_raw(tmp_path):
    """Both the decision snapshot and the flight-log record carry the
    measured rssi_raw alongside the normalized rssi."""
    import json

    p = Policy(_cfg(tmp_path), _profile())
    sig = Signals(
        rssi=-55.0, rssi_raw=-65.0, residual_loss_w=0.0,
        fec_work=0.0, link_starved_w=False, timestamp=1.0,
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


def test_predictive_demote_blocked_when_rssi_flat(tmp_path):
    """Static prior-vs-probe disagreement at flat RSSI must NOT demote — the
    slope-direction gate suppresses it (the 000010/000012 flapping fix)."""
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=2), prof)
    _settle_knee(p, 2, -50.0, True)          # learned ceiling at -50 = 2
    p.leading.state.current_mcs = 5          # probe pushed above the learned ceiling
    dec = None
    for ts in (1.0, 1.1, 1.2, 1.3, 1.4):
        dec = p.tick(_sig(-50.0, ts=ts))
    assert dec.mcs == 5                       # flat RSSI -> never predict-demoted
    p.close()


def test_predictive_demote_blocked_when_fade_too_shallow(tmp_path):
    """A real but shallow downtrend (projected drop < predictive_min_drop_db)
    must NOT demote."""
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=2, predictive_min_drop_db=1.0), prof)
    _settle_knee(p, 2, -52.0, True)          # ceiling 2 across the band
    p.leading.state.current_mcs = 5
    dec = None
    for rssi, ts in [(-50.0, 1.0), (-50.2, 1.1), (-50.4, 1.2),
                     (-50.6, 1.3), (-50.8, 1.4)]:
        dec = p.tick(_sig(rssi, ts=ts))
    assert dec.mcs == 5                       # 0.2 dB/tick -> 0.6 dB over horizon < 1.0
    p.close()


def test_predictive_demote_does_not_misfire_on_detrended_rssi(tmp_path):
    """Raw RSSI (steps down on a power change) WOULD demote; EIRP-normalized
    RSSI (flat) does NOT. Exercises predictive_ceiling's projection directly."""
    from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig
    lp = LearnedPrior("test-misfire", LearnedPriorConfig(
        persist_dir=str(tmp_path), min_samples=3, predictive_horizon_ticks=3))

    def settle(rung, rssi, n=12):
        for _ in range(n):
            lp.ingest(rssi=rssi, operating_mcs=rung, operating_clean=True, settled=True)

    settle(1, -80.0); settle(2, -78.0); settle(5, -55.0)
    # Raw: rssi -62, slope -6/tick -> projected -80 -> ceiling 1 < 5 (would demote).
    assert lp.predictive_ceiling(-62.0, -6.0) == 1
    # Normalized: rssi -50, slope 0 -> projected -50 -> ceiling 5 (no demote).
    assert lp.predictive_ceiling(-50.0, 0.0) == 5
