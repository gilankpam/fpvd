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
                            "radioProfile": "m8812eu2", "dronePort": 9999,
                            "tuning": {}}}
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
