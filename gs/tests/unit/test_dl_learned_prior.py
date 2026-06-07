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
