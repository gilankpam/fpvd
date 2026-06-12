from fpvdgs.dynlink.config_build import (
    build_aggregator, build_policy_config, make_dl_snapshot,
)


def _block(**over):
    blk = {
        "enabled": True, "maxMcs": 5,
        "radioProfile": "m8812eu2",
        "droneAddr": None, "dronePort": 9999, "tuning": {},
    }
    blk.update(over)
    return blk


def test_curated_keys_map_into_policy_config():
    cfg = build_policy_config(_block())
    assert cfg.gate.max_mcs == 5


def test_tuning_passthrough_overrides_defaults():
    cfg = build_policy_config(_block(tuning={"gate": {"probe_viable_threshold": 0.95}}))
    assert cfg.gate.probe_viable_threshold == 0.95
    # curated key still wins over any tuning attempt at the same field
    cfg2 = build_policy_config(_block(maxMcs=3, tuning={"gate": {"max_mcs": 7}}))
    assert cfg2.gate.max_mcs == 3


def test_build_aggregator_reads_tuning_smoothing():
    agg = build_aggregator(_block(tuning={"smoothing": {"ewma_alpha_rssi": 0.5}}))
    assert agg.ewma_alpha_rssi == 0.5


def test_make_dl_snapshot_defaults_drone_host_from_endpoint():
    eff = {"dynamicLink": _block(droneAddr=None),
           "drone": {"endpoint": "http://10.5.0.10:8080"}}
    snap = make_dl_snapshot(eff)
    assert snap["droneAddr"] == "10.5.0.10"
    assert snap["dronePort"] == 9999


def test_make_dl_snapshot_explicit_drone_addr_wins():
    eff = {"dynamicLink": _block(droneAddr="10.5.0.99", dronePort=12345),
           "drone": {"endpoint": "http://10.5.0.10:8080"}}
    snap = make_dl_snapshot(eff)
    assert snap["droneAddr"] == "10.5.0.99"
    assert snap["dronePort"] == 12345


def test_gate_parses_probe_knobs():
    cfg = build_policy_config(_block(tuning={"gate": {
        "probe_viable_threshold": 0.97,
        "probe_freshness_ms": 400,
        "promote_debounce_windows": 2,
        "video_demote_per": 0.04,
    }}))
    g = cfg.gate
    assert g.probe_viable_threshold == 0.97
    assert g.probe_freshness_ms == 400
    assert g.promote_debounce_windows == 2
    assert g.video_demote_per == 0.04
    # emergency + bounds kept
    assert g.emergency_loss_rate == 0.05 and g.max_mcs == 5  # _block() sets maxMcs=5


def test_retired_bitrate_fec_knobs_parse_and_warn(caplog):
    import logging
    with caplog.at_level(logging.WARNING):
        cfg = build_policy_config(_block(tuning={
            "policy": {"bitrate": {"utilization_factor": 0.7, "min_bitrate_kbps": 1000}},
            "fec": {"base_redundancy_ratio": 0.4, "max_n_escalation": 6},
            "video": {"per_packet_airtime_us": 80, "max_latency_ms": 100},
        }))
    assert cfg is not None   # loads despite the retired knobs
    assert any("3a" in r.message or "drone" in r.message.lower()
               for r in caplog.records)


def test_learned_prior_config_parsed():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {"learned_prior": {
        "bin_width_db": 3.0, "min_samples_warmstart": 7,
        "flightlog": {"max_files": 2, "enabled": False},
    }}})
    assert cfg.learned_prior.bin_width_db == 3.0
    assert cfg.learned_prior.min_samples_warmstart == 7
    assert cfg.flightlog.max_files == 2
    assert cfg.flightlog.enabled is False


def test_learned_prior_defaults_when_absent():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {}})
    assert cfg.learned_prior.enabled is True
    assert cfg.learned_prior.bin_width_db == 2.0
    assert cfg.flightlog.enabled is True


def test_flightlog_dir_default_is_dvr_and_gap_default():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {}})
    assert cfg.flightlog.dir == "/media/dvr/log/dynamic-link/"
    assert cfg.flightlog.flight_gap_s == 15.0


def test_flightlog_flight_gap_s_parsed():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {"learned_prior": {"flightlog": {
        "flight_gap_s": 8.0, "dir": "/tmp/fl"}}}})
    assert cfg.flightlog.flight_gap_s == 8.0
    assert cfg.flightlog.dir == "/tmp/fl"


def test_rssi_norm_defaults_enabled_full_curve():
    agg = build_aggregator({})
    assert agg.rssi_norm.enabled is True
    assert agg.rssi_norm.p_ref_dbm == 29
    assert agg.rssi_norm.tx_power_dbm_by_mcs == (29, 28, 25, 23, 19, 19, 19, 19)


def test_rssi_norm_parsed_from_tuning_block():
    block = {"tuning": {"rssi_norm": {
        "enabled": False, "p_ref_dbm": 30,
        "tx_power_dbm_by_mcs": [30, 29, 26, 24, 20, 20, 20, 20],
    }}}
    agg = build_aggregator(block)
    assert agg.rssi_norm.enabled is False
    assert agg.rssi_norm.p_ref_dbm == 30
    assert agg.rssi_norm.tx_power_dbm_by_mcs == (30, 29, 26, 24, 20, 20, 20, 20)


def test_rssi_norm_partial_override_keeps_defaults():
    """The rollback path: flip `enabled` off without restating the curve.
    Unspecified fields fall back to the drone-mirror defaults."""
    agg = build_aggregator({"tuning": {"rssi_norm": {"enabled": False}}})
    assert agg.rssi_norm.enabled is False
    assert agg.rssi_norm.p_ref_dbm == 29
    assert agg.rssi_norm.tx_power_dbm_by_mcs == (29, 28, 25, 23, 19, 19, 19, 19)
