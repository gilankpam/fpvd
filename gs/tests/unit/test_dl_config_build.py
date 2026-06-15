from fpvdgs.dynlink.config_build import (
    build_aggregator, build_policy_config, make_dl_snapshot,
)


def _block(**over):
    blk = {"enabled": True, "maxMcs": 5, "radioProfile": "m8812eu2",
           "dronePort": 9999}
    blk.update(over)
    return blk


def test_maxmcs_maps_into_selector():
    cfg = build_policy_config(_block())
    assert cfg.selector.max_mcs == 5


def test_selector_block_overrides_defaults():
    cfg = build_policy_config(_block(selector={
        "probeViableThreshold": 0.95, "promoteDebounceWindows": 2,
        "holdModesDownMs": 1000, "minBetweenChangesMs": 100,
        "starvationWindows": 9,
    }))
    s = cfg.selector
    assert s.probe_viable_threshold == 0.95
    assert s.promote_debounce_windows == 2
    assert s.hold_modes_down_ms == 1000
    assert s.min_between_changes_ms == 100
    assert s.starvation_windows == 9
    # unspecified selector knobs keep their defaults
    assert s.video_demote_per == 0.05
    assert s.emergency_loss_rate == 0.05


def test_selector_defaults_when_absent():
    cfg = build_policy_config(_block())
    s = cfg.selector
    assert s.probe_viable_threshold == 0.99
    assert s.probe_freshness_ms == 500.0
    assert s.hold_modes_down_ms == 2000
    assert s.starvation_windows == 5


def test_smoothing_block_overrides_defaults():
    agg = build_aggregator(_block(smoothing={"ewmaAlphaRssi": 0.5,
                                              "starvationThresholdPps": 75}))
    assert agg.ewma_alpha_rssi == 0.5
    assert agg.starvation_threshold_pps == 75
    assert agg.ewma_alpha_fec == 0.2   # default


def test_learned_prior_is_frozen_defaults_regardless_of_config():
    # An attempt to tune learned-prior internals via config is ignored.
    cfg = build_policy_config(_block(learnedPrior={"binWidthDb": 3.0,
                                                   "minSamplesWarmstart": 7}))
    assert cfg.learned_prior.bin_width_db == 2.0
    assert cfg.learned_prior.min_samples_warmstart == 20


def test_flightlog_reads_only_enabled():
    cfg = build_policy_config(_block(flightlog={"enabled": False,
                                                "dir": "/tmp/ignored"}))
    assert cfg.flightlog.enabled is False
    assert cfg.flightlog.dir == "/media/dvr/log/dynamic-link/"   # frozen default


def test_rssi_norm_reads_only_enabled_curve_frozen():
    agg = build_aggregator(_block(rssiNorm={"enabled": False,
                                            "tx_power_dbm_by_mcs": [1, 2, 3]}))
    assert agg.rssi_norm.enabled is False
    assert agg.rssi_norm.p_ref_dbm == 29
    assert agg.rssi_norm.tx_power_dbm_by_mcs == (29, 28, 25, 23, 19, 19, 19, 19)


def test_rssi_norm_defaults_enabled():
    agg = build_aggregator(_block())
    assert agg.rssi_norm.enabled is True


def test_make_dl_snapshot_uses_drone_host():
    eff = {"dynamicLink": _block(), "drone": {"host": "10.5.0.99"}}
    snap = make_dl_snapshot(eff)
    assert snap["droneAddr"] == "10.5.0.99"
    assert snap["dronePort"] == 9999


def test_make_dl_snapshot_default_host_when_drone_absent():
    snap = make_dl_snapshot({"dynamicLink": _block()})
    assert snap["droneAddr"] == "10.5.0.10"


def test_make_dl_snapshot_falls_back_to_default_port():
    eff = {"dynamicLink": _block(dronePort=None), "drone": {"host": "10.0.0.1"}}
    snap = make_dl_snapshot(eff)
    assert snap["droneAddr"] == "10.0.0.1"
    assert snap["dronePort"] == 9999
