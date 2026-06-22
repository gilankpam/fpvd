from fpvdgs.dynlink.config_build import (
    build_aggregator,
    build_policy_config,
    make_dl_snapshot,
)


def _block(**over):
    blk = {"enabled": True, "maxMcs": 5, "dronePort": 9999}
    blk.update(over)
    return blk


def test_maxmcs_maps_into_selector():
    cfg = build_policy_config(_block())
    assert cfg.selector.max_mcs == 5


def test_selector_block_overrides_defaults():
    cfg = build_policy_config(
        _block(
            selector={
                "probeViableThreshold": 0.95,
                "promoteDebounceWindows": 2,
                "holdModesDownMs": 1000,
                "minBetweenChangesMs": 100,
                "starvationWindows": 9,
            }
        )
    )
    s = cfg.selector
    assert s.probe_viable_threshold == 0.95
    assert s.promote_debounce_windows == 2
    assert s.hold_modes_down_ms == 1000
    assert s.min_between_changes_ms == 100
    assert s.starvation_windows == 9
    # unspecified selector knobs keep their defaults
    assert s.video_demote_per == 0.05


def test_selector_defaults_when_absent():
    cfg = build_policy_config(_block())
    s = cfg.selector
    assert s.probe_viable_threshold == 0.99
    assert s.probe_freshness_ms == 500.0
    assert s.hold_modes_down_ms == 2000
    assert s.starvation_windows == 5
    # SNR-knee hysteresis dead-band defaults (demote > promote)
    assert s.snr_promote_margin_db == 1.0
    assert s.snr_demote_margin_db == 1.5


def test_snr_margin_knobs_map_into_selector():
    cfg = build_policy_config(
        _block(
            selector={
                "snrPromoteMarginDb": 0.5,
                "snrDemoteMarginDb": 2.0,
            }
        )
    )
    s = cfg.selector
    assert s.snr_promote_margin_db == 0.5
    assert s.snr_demote_margin_db == 2.0


def test_smoothing_block_overrides_defaults():
    agg = build_aggregator(_block(smoothing={"ewmaAlphaRssi": 0.5, "starvationThresholdPps": 75}))
    assert agg.ewma_alpha_rssi == 0.5
    assert agg.starvation_threshold_pps == 75
    assert agg.ewma_alpha_fec == 0.2  # default


def test_learned_prior_knobs_tunable():
    cfg = build_policy_config(
        _block(
            learnedPrior={
                "settleTicks": 8,
                "alphaTighten": 0.4,
                "alphaRelax": 0.02,
                "minSamples": 12,
                "recencyDecay": 0.999,
            }
        )
    )
    lp = cfg.learned_prior
    assert lp.settle_ticks == 8
    assert lp.alpha_tighten == 0.4
    assert lp.alpha_relax == 0.02
    assert lp.min_samples == 12
    assert lp.recency_decay == 0.999


def test_learned_prior_defaults_when_absent():
    lp = build_policy_config(_block()).learned_prior
    assert lp.settle_ticks == 5
    assert lp.alpha_tighten == 0.25
    assert lp.alpha_relax == 0.05


def test_flightlog_reads_only_enabled():
    cfg = build_policy_config(_block(flightlog={"enabled": False, "dir": "/tmp/ignored"}))
    assert cfg.flightlog.enabled is False
    assert cfg.flightlog.dir == "/media/dvr/log/dynamic-link/"  # frozen default


def test_rssi_norm_defaults_to_identity():
    # The operator config knob for RSSI-norm is retired; the aggregator starts
    # in identity and the controller binds the drone curve at the connect event.
    agg = build_aggregator(_block())
    assert agg.rssi_norm.enabled is False
    assert agg.rssi_norm.tx_power_dbm_by_mcs == ()


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


def test_learned_prior_knob_survives_loader_and_reaches_policy(tmp_path):
    import json

    from fpvdgs.config import ConfigStore
    from fpvdgs.dynlink.config_build import build_policy_config

    cfgfile = tmp_path / "config.json"
    cfgfile.write_text(json.dumps({"dynamicLink": {"learnedPrior": {"settleTicks": 9}}}))
    store = ConfigStore.load(str(cfgfile))
    dl = store.effective()["dynamicLink"]
    assert dl["learnedPrior"]["settleTicks"] == 9  # survived the strip
    assert build_policy_config(dl).learned_prior.settle_ticks == 9  # reached policy


def test_make_dl_snapshot_carries_link_width():
    eff = {"dynamicLink": _block(), "drone": {"host": "10.5.0.10"}, "link": {"width": 10}}
    assert make_dl_snapshot(eff)["linkWidth"] == 10


def test_make_dl_snapshot_link_width_defaults_to_20_when_absent():
    assert make_dl_snapshot({"dynamicLink": _block()})["linkWidth"] == 20


def test_radio_profile_is_not_a_known_dynamic_link_key():
    from fpvdgs.schema import DYNAMIC_LINK_KEYS

    assert "radioProfile" not in DYNAMIC_LINK_KEYS
