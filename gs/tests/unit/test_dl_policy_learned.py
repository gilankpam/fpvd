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


def test_unknown_curve_falls_back_to_cold_start(tmp_path):
    # Empty store → warm-start unknown → today's coarse_mcs_for_rssi seed.
    p = Policy(_cfg(tmp_path, min_samples_warmstart=100), _profile())
    dec = p.tick(_sig(-50.0))   # coarse table: rssi>=-55 → mcs 5
    assert dec.mcs == 5
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
