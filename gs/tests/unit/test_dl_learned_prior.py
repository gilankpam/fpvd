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
