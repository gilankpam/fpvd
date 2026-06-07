from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig


def _prior(tmp_path, **kw):
    cfg = LearnedPriorConfig(persist_dir=str(tmp_path), **kw)
    return LearnedPrior("m8812eu2", cfg)


def test_rssi_bin_maps_and_rejects_out_of_range(tmp_path):
    p = _prior(tmp_path, bin_width_db=2.0, rssi_min=-90.0, rssi_max=-30.0)
    # -90 is the first bin; -30 is the last edge.
    assert p.rssi_bin(-90.0) == 0
    assert p.rssi_bin(-89.0) == 0
    assert p.rssi_bin(-88.0) == 1
    assert p.rssi_bin(-50.0) == 20
    # out of range / missing → None (not ingested, query unknown)
    assert p.rssi_bin(-91.0) is None
    assert p.rssi_bin(-29.0) is None
    assert p.rssi_bin(None) is None


def test_empty_store_returns_unknown(tmp_path):
    p = _prior(tmp_path)
    assert p.ceiling(-50.0) is None
    assert p.warmstart_seed(-50.0) is None


def test_ingest_raises_clean_and_counts(tmp_path):
    p = _prior(tmp_path, ewma_alpha=0.5, rssi_min=-90.0, rssi_max=-30.0)
    b = p.rssi_bin(-50.0)
    # 3 clean observations of rung 5 at this bin.
    for _ in range(3):
        p.ingest(rssi=-50.0, probed_rung=5, probe_clean=True,
                 operating_mcs=4, operating_clean=True)
    cell5 = p._cells[b][5]
    assert cell5[1] == 3                 # n
    assert cell5[0] is not None and cell5[0] > 0.8   # clean_ewma rose toward 1
    # operating rung 4 also got clean labels.
    assert p._cells[b][4][1] == 3
    assert p._cells[b][4][0] > 0.8


def test_ingest_cliff_lowers_clean(tmp_path):
    p = _prior(tmp_path, ewma_alpha=0.5)
    b = p.rssi_bin(-50.0)
    for _ in range(5):
        p.ingest(rssi=-50.0, probed_rung=6, probe_clean=True,
                 operating_mcs=5, operating_clean=True)
    high = p._cells[b][6][0]
    # now rung 6 cliffs repeatedly
    for _ in range(5):
        p.ingest(rssi=-50.0, probed_rung=6, probe_clean=False,
                 operating_mcs=5, operating_clean=True)
    assert p._cells[b][6][0] < high      # clean_ewma fell


def test_ingest_skips_out_of_range_rssi(tmp_path):
    p = _prior(tmp_path)
    p.ingest(rssi=-200.0, probed_rung=3, probe_clean=True,
             operating_mcs=2, operating_clean=True)
    # nothing recorded
    assert all(cell[1] == 0 for row in p._cells for cell in row)


def test_bin_ceiling_picks_highest_confident_clean_rung(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, viable_threshold=0.99,
               min_samples_warmstart=3)
    b = p.rssi_bin(-50.0)
    # rungs 0..4 clean, rung 5 cliffed — all with enough samples.
    for _ in range(3):
        for rung in (0, 1, 2, 3, 4):
            p.ingest(rssi=-50.0, probed_rung=rung, probe_clean=True,
                     operating_mcs=rung, operating_clean=True)
        p.ingest(rssi=-50.0, probed_rung=5, probe_clean=False,
                 operating_mcs=4, operating_clean=True)
    assert p.bin_ceiling(b) == 4


def test_bin_ceiling_unknown_until_min_samples(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, viable_threshold=0.99,
               min_samples_warmstart=5)
    b = p.rssi_bin(-50.0)
    for _ in range(2):  # only 2 < 5 samples
        p.ingest(rssi=-50.0, probed_rung=3, probe_clean=True,
                 operating_mcs=3, operating_clean=True)
    assert p.bin_ceiling(b) is None


def _fill_bin(p, rssi, ceiling, samples=5):
    """Make bin(rssi) report `ceiling`: rungs 0..ceiling clean, ceiling+1 cliff."""
    for _ in range(samples):
        for rung in range(ceiling + 1):
            p.ingest(rssi=rssi, probed_rung=rung, probe_clean=True,
                     operating_mcs=rung, operating_clean=True)
        if ceiling + 1 <= 7:
            p.ingest(rssi=rssi, probed_rung=ceiling + 1, probe_clean=False,
                     operating_mcs=ceiling, operating_clean=True)


def test_ceiling_uses_confident_bin_directly(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3)
    _fill_bin(p, -50.0, ceiling=5)
    assert p.ceiling(-50.0) == 5


def test_ceiling_ladder_extrapolates_unflown_bin_monotonically(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3)
    _fill_bin(p, -70.0, ceiling=2)   # weak RSSI bin
    _fill_bin(p, -50.0, ceiling=5)   # strong RSSI bin
    # -60 was never flown; the isotonic ladder must give a value between
    # the two anchors and never below the weaker / above the stronger.
    mid = p.ceiling(-60.0)
    assert mid is not None and 2 <= mid <= 5


def test_ceiling_isotonic_denoises_inversion(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3)
    _fill_bin(p, -70.0, ceiling=5)   # noisy: weak RSSI shows a high ceiling
    _fill_bin(p, -50.0, ceiling=3)   # strong RSSI shows a lower one
    # Monotonicity (more RSSI ⇒ >= ceiling) must hold after the isotonic fit.
    assert p.ceiling(-50.0) >= p.ceiling(-70.0)


def test_ceiling_unknown_with_no_confident_bins(tmp_path):
    p = _prior(tmp_path, min_samples_warmstart=100)
    _fill_bin(p, -50.0, ceiling=5, samples=3)   # below threshold
    assert p.ceiling(-50.0) is None


def test_warmstart_seed_applies_margin_and_clamp(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
               warmstart_margin=1)
    _fill_bin(p, -50.0, ceiling=5)
    assert p.warmstart_seed(-50.0) == 4          # 5 - margin(1)
    assert p.warmstart_seed(-91.0) is None        # out of range


def test_warmstart_seed_none_when_unconfident(tmp_path):
    p = _prior(tmp_path, min_samples_warmstart=100)
    _fill_bin(p, -50.0, ceiling=5, samples=3)
    assert p.warmstart_seed(-50.0) is None


def test_predictive_ceiling_projects_and_gates_on_strict_confidence(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
               min_samples_predictive=3, predictive_horizon_ticks=2,
               bin_width_db=2.0)
    _fill_bin(p, -50.0, ceiling=5)   # where we are now
    _fill_bin(p, -56.0, ceiling=2)   # where a -3 dB/tick fade lands in 2 ticks
    # slope -3 dB/tick, horizon 2 -> projected ≈ -50 + (-3*2) = -56 -> ceiling 2
    assert p.predictive_ceiling(-50.0, -3.0) == 2


def test_predictive_ceiling_needs_strict_min_samples(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
               min_samples_predictive=100, predictive_horizon_ticks=2)
    _fill_bin(p, -56.0, ceiling=2, samples=5)   # confident for warmstart, not predictive
    assert p.predictive_ceiling(-50.0, -3.0) is None
