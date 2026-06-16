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
