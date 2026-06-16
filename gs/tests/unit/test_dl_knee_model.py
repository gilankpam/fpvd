from fpvdgs.dynlink.learned_prior import KneeModel, LearnedPriorConfig, MAX_MCS


def _model(**kw):
    return KneeModel(LearnedPriorConfig(**kw))


def test_first_sample_seeds_knee_at_rssi():
    m = _model()
    m.observe(rung=4, rssi=-60.0, clean=True)
    assert m._knee[4] == -60.0
    assert m._count[4] == 1.0


def test_clean_below_knee_pulls_down_slowly():
    m = _model(alpha_relax=0.1, recency_decay=1.0)
    m.observe(4, -60.0, clean=True)        # seed -60
    m.observe(4, -70.0, clean=True)        # works even at -70 -> knee toward -70
    # -60 + 0.1*(-70 - -60) = -61.0
    assert m._knee[4] == -61.0


def test_dirty_above_knee_pulls_up_fast():
    m = _model(alpha_tighten=0.5, recency_decay=1.0)
    m.observe(4, -70.0, clean=True)        # seed -70
    m.observe(4, -60.0, clean=False)       # fails at -60 -> knee toward -60
    # -70 + 0.5*(-60 - -70) = -65.0
    assert m._knee[4] == -65.0


def test_tighten_faster_than_relax():
    up = _model(alpha_tighten=0.25, alpha_relax=0.05, recency_decay=1.0)
    up.observe(4, -70.0, True); up.observe(4, -60.0, False)   # dirty pull up
    down = _model(alpha_tighten=0.25, alpha_relax=0.05, recency_decay=1.0)
    down.observe(4, -60.0, True); down.observe(4, -70.0, True)  # clean pull down
    moved_up = abs(up._knee[4] - (-70.0))
    moved_down = abs(down._knee[4] - (-60.0))
    assert moved_up > moved_down           # pessimistic asymmetry


def test_consistent_sample_does_not_move_knee():
    m = _model(recency_decay=1.0)
    m.observe(4, -60.0, clean=True)        # seed -60
    m.observe(4, -50.0, clean=True)        # clean ABOVE knee -> consistent, no move
    assert m._knee[4] == -60.0
    m.observe(4, -70.0, clean=False)       # dirty BELOW knee -> consistent, no move
    assert m._knee[4] == -60.0


def test_observe_ignores_out_of_range_rung():
    m = _model()
    m.observe(rung=99, rssi=-50.0, clean=True)
    assert all(k is None for k in m._knee)


def _confident(m, rung, knee, *, n=10):
    """Force a confident knee directly (bypass learning dynamics)."""
    m._knee[rung] = knee
    m._count[rung] = float(n)


def test_ceiling_none_when_cold():
    m = _model()
    assert m.ceiling(-50.0) is None


def test_ceiling_highest_confident_rung_at_or_below_rssi():
    m = _model(min_samples=8)
    _confident(m, 1, -80.0)
    _confident(m, 4, -60.0)
    assert m.ceiling(-55.0) == 4     # -60 and -80 both <= -55
    assert m.ceiling(-70.0) == 1     # only -80 <= -70
    assert m.ceiling(-90.0) is None  # nothing low enough


def test_ceiling_ignores_unconfident_knee():
    m = _model(min_samples=8)
    _confident(m, 4, -60.0, n=10)
    m._knee[5] = -55.0
    m._count[5] = 3.0                # below min_samples
    assert m.ceiling(-50.0) == 4     # rung 5 not confident -> ignored


def test_ceiling_enforces_rung_monotonicity_on_inversion():
    # Physically-impossible inversion: rung 4 viable at LOWER rssi than rung 2.
    m = _model(min_samples=8)
    _confident(m, 2, -60.0)
    _confident(m, 4, -70.0)          # inverted
    # cumulative-max raises rung 4's effective knee to -60 (pessimistic).
    assert m.ceiling(-65.0) is None  # neither effective knee (-60) <= -65
    assert m.ceiling(-58.0) == 4


def test_recency_decay_ages_out_unreinforced_knee():
    # A knee that stops being reinforced loses confidence as OTHER rungs are
    # observed, and eventually drops below min_samples (no longer in ceiling).
    m = _model(min_samples=8.0, recency_decay=0.9, alpha_relax=0.0,
               alpha_tighten=0.0)
    for _ in range(20):                      # rung 4 becomes confident
        m.observe(4, -60.0, clean=True)
    assert m.ceiling(-50.0) == 4
    for _ in range(60):                      # hammer rung 1; rung 4 decays
        m.observe(1, -80.0, clean=True)
    assert m._count[4] < 8.0                 # rung 4 confidence aged out
    assert m.ceiling(-50.0) == 1             # rung 4 no longer a confident ceiling


def test_recency_decay_one_keeps_confidence_forever():
    m = _model(min_samples=8.0, recency_decay=1.0)
    for _ in range(10):
        m.observe(4, -60.0, clean=True)
    for _ in range(1000):
        m.observe(1, -80.0, clean=True)
    assert m._count[4] == 10.0               # no decay
    assert m.ceiling(-50.0) == 4


def test_to_dict_round_trips_through_load_dict():
    m = _model()
    _confident(m, 4, -60.0, n=12)
    doc = m.to_dict()
    assert doc["schema"] == 2
    m2 = _model()
    assert m2.load_dict(doc) is True
    assert m2.ceiling(-50.0) == 4


def test_load_dict_rejects_v1_schema():
    m = _model()
    assert m.load_dict({"schema": 1, "bins": [2.0, -90, -30], "cells": []}) is False
    assert m.ceiling(-50.0) is None          # stays empty -> retrains


def test_load_dict_rejects_malformed():
    m = _model()
    assert m.load_dict({"schema": 2, "knees": [1, 2], "counts": []}) is False


def test_knees_snapshot_rounds():
    m = _model()
    m._knee[3] = -64.273
    snap = m.knees_snapshot()
    assert snap[3] == -64.3 and snap[0] is None
