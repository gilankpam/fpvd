import pytest

from fpvdgs import schema
from fpvdgs.schema import (
    SchemaError,
    validate_config_patch,
    validate_effective,
)


def test_config_patch_accepts_link():
    schema.validate_config_patch({"link": {"channel": 100}})  # no raise


def test_config_patch_rejects_unknown_link_key():
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"link": {"bogus": 1}})


def test_config_patch_allows_non_link():
    validate_config_patch({"wfb": {"mavlink": {"peer": "connect://127.0.0.1:14550"}}})


def test_config_patch_rejects_unknown_top_level():
    with pytest.raises(SchemaError):
        validate_config_patch({"bogus": 1})


def test_validate_effective_checks_width_domain():
    with pytest.raises(SchemaError):
        validate_effective({"link": {"channel": 132, "width": 80, "region": "US"}})


def test_validate_effective_accepts_10mhz():
    # 10 MHz is supported (underclocked); must not raise.
    validate_effective({"link": {"channel": 132, "width": 10, "region": "US"}})


def test_validate_effective_ok():
    validate_effective({
        "link": {"channel": 132, "width": 40, "txPowerDbm": 19, "region": "US",
                 "linkId": 7669206, "beamforming": {"enabled": False}, "wlans": "auto"},
        "wfb": {"profile": "gs", "mavlink": {"peer": "connect://127.0.0.1:14550"}, "raw": {}},
        "drone": {"endpoint": "http://10.5.0.10:8080"},
    })


def _eff(**dl):
    base = {"link": {"channel": 132, "width": 40, "region": "US"},
            "dynamicLink": {"enabled": False, "maxMcs": 5,
                            "radioProfile": "m8812eu2", "dronePort": 9999}}
    base["dynamicLink"].update(dl)
    return base


def test_config_patch_allows_dynamiclink():
    schema.validate_config_patch({"dynamicLink": {"enabled": True}})  # no raise


def test_effective_accepts_valid_dynamiclink():
    schema.validate_effective(_eff())  # no raise


def test_effective_rejects_bad_max_mcs():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(maxMcs=9))


def test_effective_rejects_empty_radio_profile():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(radioProfile=""))


def test_effective_accepts_known_radio_profile():
    schema.validate_effective(_eff(radioProfile="m8812eu2"))  # no raise


def test_idr_forward_validates():
    schema.validate_effective({"link": {"channel": 1, "region": "US"},
                               "idrForward": {"enabled": True, "port": 11223}})
    with pytest.raises(schema.SchemaError):
        schema.validate_effective({"link": {"channel": 1, "region": "US"},
                                   "idrForward": {"enabled": True, "port": 0}})


def test_config_patch_accepts_pixelpilot():
    # should not raise
    schema.validate_config_patch({"pixelpilot": {"screenMode": "1280x720@60"}})


def test_validate_effective_accepts_pixelpilot_block():
    cfg = {"link": {"channel": 132, "width": 40, "region": "US"},
           "pixelpilot": {"enabled": True, "videoScale": 1.0,
                          "screenMode": "1920x1080@60",
                          "rtpPort": 5600, "rtpJitterMs": 1,
                          "codec": "h265",
                          "env": {},
                          "dvr": {"framerate": 60},
                          "extraArgs": []}}
    schema.validate_effective(cfg)  # no raise


def test_validate_effective_rejects_bad_pixelpilot():
    base = {"link": {"channel": 132, "width": 40, "region": "US"}}
    for bad in (
        {"videoScale": 0},
        {"videoScale": -1.0},
        {"videoScale": "x"},
        {"enabled": "yes"},
        {"screenMode": ""},
        {"extraArgs": "not-a-list"},
        {"extraArgs": [1, 2]},
        {"rtpPort": 0},
        {"rtpPort": 70000},
        {"rtpJitterMs": -1},
        {"codec": ""},
        {"dvr": {"reencBitrate": 0}},
        {"dvr": {"mode": ""}},
        {"dvr": {"osd": "yes"}},
        {"env": {"A": 1}},
        {"env": "x"},
    ):
        with pytest.raises(schema.SchemaError):
            schema.validate_effective({**base, "pixelpilot": bad})


def test_shipped_defaults_include_pixelpilot_and_validate():
    from fpvdgs.config_defaults import default_config
    cfg = default_config()
    assert "pixelpilot" in cfg
    assert cfg["pixelpilot"]["enabled"] is True
    schema.validate_effective(cfg)  # no raise


def test_beamforming_enabled_bool_ok():
    from fpvdgs import schema
    # Shape-only check: pin the capability hook to "unknown" so a probe left
    # registered by a prior build_app test doesn't shell out / reject here.
    schema.set_bf_capable(None)
    cfg = {"link": {"region": "US", "channel": 132,
                    "beamforming": {"enabled": True}}}
    schema.validate_effective(cfg)   # must not raise


def test_beamforming_enabled_must_be_bool():
    from fpvdgs import schema
    cfg = {"link": {"region": "US", "channel": 132,
                    "beamforming": {"enabled": "yes"}}}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)


def test_beamforming_rejects_unknown_subkey():
    from fpvdgs import schema
    cfg = {"link": {"region": "US", "channel": 132,
                    "beamforming": {"enabled": True, "remoteMac": "x"}}}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)


def test_validate_effective_accepts_drone_block():
    schema.validate_effective({"link": {"channel": 132, "region": "US"},
                               "drone": {"host": "10.5.0.10", "apiPort": 8080}})


def test_drone_host_empty_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective({"link": {"channel": 132, "region": "US"},
                                   "drone": {"host": ""}})


def test_drone_apiport_out_of_range_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective({"link": {"channel": 132, "region": "US"},
                                   "drone": {"apiPort": 0}})


def test_patch_rejects_unknown_drone_key():
    # the old drone.endpoint key is now unknown -> rejected on PATCH
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"drone": {"endpoint": "http://x:8080"}})


def test_patch_accepts_known_drone_keys():
    schema.validate_config_patch({"drone": {"host": "10.5.0.10", "apiPort": 8080}})


def test_enable_bf_on_incapable_card_rejected():
    schema.set_bf_capable(lambda cfg: False)
    try:
        with pytest.raises(schema.SchemaError):
            schema.validate_effective({"link": {"channel": 1, "region": "US",
                                                 "beamforming": {"enabled": True}}})
    finally:
        schema.set_bf_capable(lambda cfg: True)


def test_enable_bf_on_capable_card_ok():
    schema.set_bf_capable(lambda cfg: True)
    try:
        schema.validate_effective({"link": {"channel": 1, "region": "US",
                                            "beamforming": {"enabled": True}}})
    finally:
        schema.set_bf_capable(None)


def _dl(**over):
    base = {"enabled": True, "maxMcs": 5, "radioProfile": "m8812eu2",
            "dronePort": 9999}
    base.update(over)
    return {"link": {"channel": 132, "region": "US", "width": 20},
            "dynamicLink": base}


def test_validate_effective_accepts_flat_dynamic_link():
    schema.validate_effective(_dl(selector={"probeViableThreshold": 0.9},
                                  smoothing={"ewmaAlphaRssi": 0.3},
                                  flightlog={"enabled": True},
                                  rssiNorm={"enabled": True}))  # no raise


def test_selector_probability_out_of_range_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(selector={"probeViableThreshold": 1.5}))


def test_smoothing_alpha_out_of_range_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(smoothing={"ewmaAlphaRssi": 0}))


def test_patch_rejects_unknown_dynamic_link_subkey():
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"dynamicLink": {"bogusKnob": 1}})


def test_patch_accepts_known_dynamic_link_keys():
    schema.validate_config_patch({"dynamicLink": {"selector": {}, "maxMcs": 4}})


def test_smoothing_alpha_above_one_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(smoothing={"ewmaAlphaRssi": 1.5}))


def test_selector_non_dict_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(selector=0))


def test_learned_prior_block_accepted_in_patch():
    validate_config_patch({"dynamicLink": {"learnedPrior": {"settleTicks": 8,
                                                            "alphaTighten": 0.4}}})  # no raise


def test_learned_prior_unknown_key_rejected():
    with pytest.raises(SchemaError, match="learnedPrior"):
        validate_effective(_dl(learnedPrior={"bogus": 1}))


def test_learned_prior_value_ranges_validated():
    # all valid defaults — must not raise
    validate_effective(_dl(learnedPrior={
        "settleTicks": 5, "viableLoss": 0.05, "alphaTighten": 0.25,
        "alphaRelax": 0.05, "minSamples": 8, "recencyDecay": 0.9995,
    }))
    with pytest.raises(SchemaError):
        validate_effective(_dl(learnedPrior={"settleTicks": 0}))      # pos int required
    with pytest.raises(SchemaError):
        validate_effective(_dl(learnedPrior={"alphaTighten": 1.5}))   # (0,1]
    with pytest.raises(SchemaError):
        validate_effective(_dl(learnedPrior={"viableLoss": 2.0}))     # 0..1


def test_connection_monitor_accepts_shipped_defaults():
    from fpvdgs.config_defaults import default_config
    validate_effective(default_config())          # includes connectionMonitor; must pass


def test_connection_monitor_invariant_rejects_stale_le_poll():
    from fpvdgs.config_defaults import default_config
    cfg = default_config()
    cfg["connectionMonitor"]["tunnelStaleS"] = 1.0
    cfg["connectionMonitor"]["httpPollS"] = 1.5    # stale must be > poll
    with pytest.raises(SchemaError):
        validate_effective(cfg)


def test_connection_monitor_rejects_bad_fail_count():
    from fpvdgs.config_defaults import default_config
    cfg = default_config()
    cfg["connectionMonitor"]["httpFailCount"] = 0  # must be a positive int
    with pytest.raises(SchemaError):
        validate_effective(cfg)


def test_config_patch_accepts_connection_monitor():
    validate_config_patch({"connectionMonitor": {"enabled": False}})   # no raise
