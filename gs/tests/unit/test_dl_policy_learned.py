"""Phase 4 integration: warm-start + predictive-demote + regression."""
from __future__ import annotations

from pathlib import Path

from fpvdgs.dynlink.policy import Policy, PolicyConfig
from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.profile import load_profile
from fpvdgs.dynlink.signals import Signals

PROFILES = Path(__file__).resolve().parents[2] / "fpvdgs" / "dynlink" / "profiles"


def _profile():
    return load_profile("m8812eu2", [PROFILES])


def _cfg(tmp_path, **lp):
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path), **lp),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def _sig(rssi, ts=1.0):
    return Signals(rssi=rssi, residual_loss_w=0.0, fec_work=0.0,
                   link_starved_w=False, timestamp=ts)


def test_warm_start_seeds_from_persisted_curve(tmp_path):
    # Flight 1: build a confident curve at -50 -> ceiling 5, persist.
    prof = _profile()
    p1 = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3), prof)
    for _ in range(5):
        p1.learned_prior.ingest(rssi=-50.0, probed_rung=5, probe_clean=True,
                                operating_mcs=5, operating_clean=True)
    p1.close()   # flushes
    # Flight 2: a fresh Policy warm-starts to the learned ceiling on tick 1,
    # instead of climbing from MCS 1.
    p2 = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3), prof)
    dec = p2.tick(_sig(-50.0))
    assert dec.mcs == 5
    p2.close()


def test_unknown_curve_no_seed_stays_at_boot(tmp_path):
    # Empty store → warm-start unknown → NO RSSI cold-start seed (dropped).
    # With no probe data the MCS stays at the boot default (1); in production
    # the probe climbs from there.
    p = Policy(_cfg(tmp_path, min_samples_warmstart=100), _profile())
    dec = p.tick(_sig(-50.0))
    assert dec.mcs == 1
    p.close()


def test_predictive_demote_on_confident_fade(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
                    min_samples_predictive=3, predictive_horizon_ticks=2,
                    predictive_debounce_windows=2), prof)
    # learn: strong RSSI -> ceiling 5, the bin a fast fade lands in -> ceiling 2
    for _ in range(5):
        for rung in range(6):
            p.learned_prior.ingest(rssi=-50.0, probed_rung=rung, probe_clean=True,
                                   operating_mcs=rung, operating_clean=True)
        for rung in range(3):
            p.learned_prior.ingest(rssi=-56.0, probed_rung=rung, probe_clean=True,
                                   operating_mcs=rung, operating_clean=True)
        p.learned_prior.ingest(rssi=-56.0, probed_rung=3, probe_clean=False,
                               operating_mcs=2, operating_clean=True)
    # warm-start to 5 at -50, then a -3 dB/tick fade; after debounce the
    # operating MCS pre-demotes toward the projected ceiling (2), down-only.
    p.tick(_sig(-50.0, ts=1.0))
    p.tick(_sig(-53.0, ts=1.1))
    dec = p.tick(_sig(-56.0, ts=1.2))
    assert dec.mcs <= 2
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
