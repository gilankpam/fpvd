import pytest

from fpvdgs import schema
from fpvdgs.schema import (
    SchemaError,
    validate_config_patch,
    validate_link_patch,
    validate_effective,
)


def test_config_patch_rejects_link_keys():
    with pytest.raises(SchemaError) as e:
        validate_config_patch({"link": {"channel": 100}})
    assert "link" in str(e.value)


def test_config_patch_allows_non_link():
    validate_config_patch({"wfb": {"mavlink": {"peer": "connect://127.0.0.1:14550"}}})


def test_config_patch_rejects_unknown_top_level():
    with pytest.raises(SchemaError):
        validate_config_patch({"bogus": 1})


def test_link_patch_allows_only_link():
    validate_link_patch({"link": {"channel": 100, "width": 20}})
    with pytest.raises(SchemaError):
        validate_link_patch({"wfb": {"profile": "gs"}})


def test_link_patch_rejects_unknown_link_key():
    with pytest.raises(SchemaError):
        validate_link_patch({"link": {"mcs": 5}})


def test_validate_effective_checks_width_domain():
    with pytest.raises(SchemaError):
        validate_effective({"link": {"channel": 132, "width": 80, "region": "US"}})


def test_validate_effective_accepts_10mhz():
    # 10 MHz is supported (underclocked); must not raise.
    validate_effective({"link": {"channel": 132, "width": 10, "region": "US"}})


def test_validate_effective_ok():
    validate_effective({
        "link": {"channel": 132, "width": 40, "txpower": 19, "region": "US",
                 "linkId": 7669206, "beamforming": {"enabled": False}, "wlans": "auto"},
        "wfb": {"profile": "gs", "mavlink": {"peer": "connect://127.0.0.1:14550"}, "raw": {}},
        "drone": {"endpoint": "http://10.5.0.10:8080"},
    })


def _eff(**dl):
    base = {"link": {"channel": 132, "width": 40, "region": "US"},
            "dynamicLink": {"enabled": False, "maxMcs": 5, "bandwidth": 20,
                            "txpower": {"min": 18, "max": 28},
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


def test_effective_rejects_bad_bandwidth():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(bandwidth=15))


def test_effective_rejects_inverted_txpower():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(txpower={"min": 30, "max": 10}))


def test_effective_rejects_unknown_radio_profile():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(radioProfile="nonexistent"))


def test_effective_accepts_known_radio_profile():
    schema.validate_effective(_eff(radioProfile="m8812eu2"))  # no raise


def test_effective_rejects_bad_idr_port():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(idrPort=70000))


def test_config_patch_accepts_pixelpilot():
    # should not raise
    schema.validate_config_patch({"pixelpilot": {"screenMode": "1280x720@60"}})


def test_validate_effective_accepts_pixelpilot_block():
    cfg = {"link": {"channel": 132, "width": 40, "region": "US"},
           "pixelpilot": {"enabled": True, "videoScale": 1.0,
                          "dvrFramerate": 60, "screenMode": "1920x1080@60",
                          "extraArgs": []}}
    schema.validate_effective(cfg)  # no raise


def test_validate_effective_rejects_bad_pixelpilot():
    base = {"link": {"channel": 132, "width": 40, "region": "US"}}
    for bad in (
        {"videoScale": 0},
        {"videoScale": -1.0},
        {"videoScale": "x"},
        {"dvrFramerate": 0},
        {"dvrFramerate": 1.5},
        {"enabled": "yes"},
        {"screenMode": ""},
        {"extraArgs": "not-a-list"},
        {"extraArgs": [1, 2]},
    ):
        with pytest.raises(schema.SchemaError):
            schema.validate_effective({**base, "pixelpilot": bad})


def test_shipped_defaults_include_pixelpilot_and_validate():
    import json, pathlib
    p = pathlib.Path(__file__).resolve().parents[2] / "etc" / "defaults.json"
    cfg = json.loads(p.read_text())
    assert "pixelpilot" in cfg
    assert cfg["pixelpilot"]["enabled"] is True
    schema.validate_effective(cfg)  # no raise
