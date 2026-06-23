import json

from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig


def _prior(tmp_path, **kw):
    cfg = LearnedPriorConfig(persist_dir=str(tmp_path), **kw)
    return LearnedPrior("m8812eu2", cfg)


def _settle(p, rung, rssi, clean, n=12):
    for _ in range(n):
        p.ingest(rssi=rssi, operating_mcs=rung, operating_clean=clean, settled=True)


def test_empty_store_returns_unknown(tmp_path):
    p = _prior(tmp_path)
    assert p.ceiling(-50.0) is None
    assert p.warmstart_seed(-50.0) is None
    assert p.predictive_ceiling(-50.0, -1.0) is None


def test_ingest_only_learns_when_settled(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    for _ in range(10):
        p.ingest(rssi=-60.0, operating_mcs=4, operating_clean=True, settled=False)
    assert p.ceiling(-50.0) is None  # nothing learned while unsettled
    _settle(p, 4, -60.0, False, n=5)  # dirty to plant the knee
    assert p.ceiling(-50.0) == 4


def test_ingest_skips_none_rssi(tmp_path):
    p = _prior(tmp_path, min_samples=1)
    p.ingest(rssi=None, operating_mcs=4, operating_clean=True, settled=True)
    assert p.ceiling(-50.0) is None


def test_ceiling_and_warmstart_seed_from_knees(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    # Direct-set: plant confident knees without depending on learning dynamics
    p._model._knee[1] = -80.0
    p._model._count[1] = 12.0
    p._model._knee[4] = -60.0
    p._model._count[4] = 12.0
    assert p.ceiling(-55.0) == 4
    assert p.warmstart_seed(-70.0) == 1


def test_predictive_ceiling_projects_with_slope(tmp_path):
    p = _prior(tmp_path, min_samples=3, predictive_horizon_ticks=3)
    # Direct-set: plant confident knees without depending on learning dynamics
    p._model._knee[4] = -60.0
    p._model._count[4] = 12.0
    p._model._knee[1] = -80.0
    p._model._count[1] = 12.0
    # at -58 now, fading -2/tick -> projected -58 + (-2*3) = -64 -> below rung4 knee
    assert p.predictive_ceiling(-58.0, -2.0) == 1


def test_predictive_ceiling_none_rssi(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 4, -60.0, True)
    assert p.predictive_ceiling(None, -2.0) is None


def test_persistence_round_trip_v2(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 4, -60.0, False)  # dirty to plant the knee
    p.flush()
    p2 = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path), min_samples=3))
    assert p2.ceiling(-50.0) == 4


def test_v1_file_ignored_and_retrains(tmp_path):
    (tmp_path / "m8812eu2.json").write_text(
        json.dumps({"schema": 1, "bins": [2.0, -90, -30], "cells": []})
    )
    p = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path), min_samples=3))
    assert p.ceiling(-50.0) is None  # v1 ignored
    _settle(p, 4, -60.0, False)  # dirty to plant the knee
    assert p.ceiling(-50.0) == 4  # retrains on v2


def test_corrupt_file_is_ignored(tmp_path):
    (tmp_path / "m8812eu2.json").write_text("{not json")
    p = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path)))
    assert p.ceiling(-50.0) is None


def test_to_status_reports_knees(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 4, -60.0, True)
    st = p.to_status()
    assert st["key"] == "m8812eu2"
    assert isinstance(st["knees"], list) and len(st["knees"]) == 8


def test_lsq_slope_flat_is_zero():
    from fpvdgs.dynlink.learned_prior import lsq_slope

    assert lsq_slope([-50.0, -50.0, -50.0]) == 0.0


def test_lsq_slope_linear_ramp_is_exact():
    from fpvdgs.dynlink.learned_prior import lsq_slope

    # -0.5 dBm per tick ramp
    assert abs(lsq_slope([-50.0, -50.5, -51.0, -51.5, -52.0]) - (-0.5)) < 1e-9


def test_lsq_slope_rejects_lone_spike():
    from fpvdgs.dynlink.learned_prior import lsq_slope

    # a -0.5/tick ramp with one +10 dB spike stays a clear downtrend;
    # a single-tick delta at the spike would read ~+9.5
    ramp = [-0.5 * i for i in range(10)]
    ramp[5] += 10.0
    assert -0.6 < lsq_slope(ramp) < -0.35


def test_lsq_slope_under_two_samples_is_zero():
    from fpvdgs.dynlink.learned_prior import lsq_slope

    assert lsq_slope([]) == 0.0
    assert lsq_slope([-50.0]) == 0.0


def test_key_sanitized_in_filename(tmp_path):
    p = LearnedPrior("bl-m8812eu2/weird", LearnedPriorConfig(persist_dir=str(tmp_path)))
    p.flush()
    files = list(tmp_path.iterdir())
    assert len(files) == 1 and "/" not in files[0].name


def _settle_snr(p, rung, snr, clean, n=12):
    for _ in range(n):
        p.ingest(rssi=None, snr=snr, operating_mcs=rung, operating_clean=clean, settled=True)


def test_snr_ceiling_learns_independently_of_rssi(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    # Direct-set: plant confident SNR knees without depending on learning dynamics
    p._snr_model._knee[1] = 10.0
    p._snr_model._count[1] = 12.0
    p._snr_model._knee[4] = 30.0
    p._snr_model._count[4] = 12.0
    assert p.snr_ceiling(35.0) == 4
    assert p.snr_ceiling(12.0) == 1
    assert p.snr_ceiling(5.0) is None
    assert p.ceiling(-50.0) is None  # rssi model untouched (no rssi ingested)


def test_snr_ceiling_none_when_cold_or_none(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    assert p.snr_ceiling(30.0) is None  # cold
    # clean settle plants no knee (new invariant); this asserts None-input handling
    _settle_snr(p, 4, 30.0, True)
    assert p.snr_ceiling(None) is None  # None input


def test_combined_persistence_round_trip(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 4, -60.0, False)  # dirty to plant the rssi knee
    _settle_snr(p, 4, 30.0, False)  # dirty to plant the snr knee
    p.flush()
    p2 = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path), min_samples=3))
    assert p2.ceiling(-50.0) == 4
    assert p2.snr_ceiling(35.0) == 4


def test_v2_flat_file_loads_rssi_keeps_snr_cold(tmp_path):
    import json

    # a deployed v2 doc is the flat rssi-model dict (no "rssi"/"snr" wrapper)
    p1 = _prior(tmp_path, min_samples=3)
    _settle(p1, 4, -60.0, False)  # dirty to plant the rssi knee
    flat = p1._model.to_dict()
    flat["key"] = "m8812eu2"
    (tmp_path / "m8812eu2.json").write_text(json.dumps(flat))
    p2 = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path), min_samples=3))
    assert p2.ceiling(-50.0) == 4  # rssi knee survived the upgrade
    assert p2.snr_ceiling(35.0) is None  # snr starts cold


def test_load_tolerates_null_model_subkeys(tmp_path):
    import json

    # a file with rssi/snr present but null must NOT boot-brick (AttributeError)
    (tmp_path / "m8812eu2.json").write_text(
        json.dumps({"rssi": None, "snr": None, "key": "m8812eu2"})
    )
    p = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path)))
    assert p.ceiling(-50.0) is None and p.snr_ceiling(30.0) is None


# ── snr_rung_unviable: per-rung known-bad, NOT highest-confident-viable ───────
# Distinguishes "this rung is confidently unviable" (block) from "this rung is
# unknown" (explore). The frontier-cap fix for the never-promotes-to-top deadlock.


def test_snr_rung_unviable_cold_rung_is_explorable(tmp_path):
    # Only rung4 has been learned; rung5 (the frontier) is UNKNOWN, not unviable.
    p = _prior(tmp_path, min_samples=3)
    _settle_snr(p, 4, 27.0, True)
    assert p.snr_rung_unviable(5, 39.0) is False  # cold -> explorable (deadlock fix)
    assert p.snr_rung_unviable(6, 39.0) is False


def test_snr_rung_unviable_confident_rung(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    # Direct-set: plant confident SNR knee at 27 for rung4
    p._snr_model._knee[4] = 27.0
    p._snr_model._count[4] = 12.0
    assert p.snr_rung_unviable(4, 24.0) is True  # below the knee -> known-bad
    assert p.snr_rung_unviable(4, 30.0) is False  # clears the knee -> viable


def test_snr_rung_unviable_none_inputs(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    # clean settle plants no knee (new invariant); this asserts None-input handling
    _settle_snr(p, 4, 27.0, True)
    assert p.snr_rung_unviable(4, None) is False  # no live snr -> can't judge
    assert p.snr_rung_unviable(None, 39.0) is False


def test_snr_rung_unviable_margin_threads_through(tmp_path):
    # The promote/demote hysteresis margin must reach the knee model: a value a
    # hair below the knee is NOT unviable once a margin is applied.
    p = _prior(tmp_path, min_samples=3)
    # Direct-set: plant confident SNR knee at 27 for rung4
    p._snr_model._knee[4] = 27.0
    p._snr_model._count[4] = 12.0
    assert p.snr_rung_unviable(4, 26.6) is True  # strict (default margin 0)
    assert p.snr_rung_unviable(4, 26.6, margin=1.0) is False
    assert p.snr_rung_unviable(4, 25.0, margin=1.0) is True  # clearly below


def test_unbound_sentinel_prior_does_not_flush(tmp_path):
    # The pre-bind "unbound" sentinel prior must never write to disk — avoids
    # the stray unbound.json on cold start (before the drone adapter binds).
    import os

    from fpvdgs.dynlink.learned_prior import UNBOUND_KEY

    cfg = LearnedPriorConfig(persist_dir=str(tmp_path), min_samples=1)
    p = LearnedPrior(UNBOUND_KEY, cfg)
    _settle(p, 4, -60.0, True)
    p.flush()
    assert not os.path.exists(os.path.join(str(tmp_path), f"{UNBOUND_KEY}.json"))


def test_unbound_sentinel_prior_ignores_existing_file(tmp_path):
    # A stale unbound.json must NOT be loaded by the ephemeral sentinel prior.
    import os

    from fpvdgs.dynlink.learned_prior import UNBOUND_KEY

    doc = {
        "key": UNBOUND_KEY,
        "rssi": {"schema": 2, "knees": [-40.0] * 8, "counts": [99.0] * 8},
        "snr": {"schema": 2, "knees": [None] * 8, "counts": [0.0] * 8},
    }
    with open(os.path.join(str(tmp_path), f"{UNBOUND_KEY}.json"), "w") as f:
        json.dump(doc, f)
    cfg = LearnedPriorConfig(persist_dir=str(tmp_path), min_samples=1)
    p = LearnedPrior(UNBOUND_KEY, cfg)
    assert p.ceiling(-30.0) is None  # did not load the stale confident knees


def test_real_key_prior_still_flushes(tmp_path):
    # Regression: a real adapter-id key persists as before.
    import os

    cfg = LearnedPriorConfig(persist_dir=str(tmp_path), min_samples=1)
    p = LearnedPrior("bl-m8812eu2", cfg)
    _settle(p, 4, -60.0, True)
    p.flush()
    assert os.path.exists(os.path.join(str(tmp_path), "bl-m8812eu2.json"))
