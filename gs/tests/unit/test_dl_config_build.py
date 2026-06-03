from fpvdgs.dynlink.config_build import (
    build_aggregator, build_policy_config, make_dl_snapshot, resolve_profile,
)


def _block(**over):
    blk = {
        "enabled": True, "maxMcs": 5, "bandwidth": 20,
        "txpower": {"min": 18, "max": 28}, "radioProfile": "m8812eu2",
        "droneAddr": None, "dronePort": 9999, "tuning": {},
    }
    blk.update(over)
    return blk


def test_curated_keys_map_into_policy_config():
    cfg = build_policy_config(_block())
    assert cfg.gate.max_mcs == 5
    assert cfg.leading.bandwidth == 20
    assert cfg.leading.tx_power_min_dBm == 18
    assert cfg.leading.tx_power_max_dBm == 28


def test_tuning_passthrough_overrides_defaults():
    cfg = build_policy_config(_block(tuning={"gate": {"hysteresis_up_db": 4.0}}))
    assert cfg.gate.hysteresis_up_db == 4.0
    # curated key still wins over any tuning attempt at the same field
    cfg2 = build_policy_config(_block(maxMcs=3, tuning={"gate": {"max_mcs": 7}}))
    assert cfg2.gate.max_mcs == 3


def test_resolve_profile_uses_packaged_json():
    prof = resolve_profile(_block(radioProfile="m8812eu2"))
    assert prof.name == "BL-M8812EU2"


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
