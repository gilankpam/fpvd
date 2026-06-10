import pytest

from fpvdgs import schema
from fpvdgs.schema import (
    SchemaError,
    validate_effective,
)


def test_validate_effective_checks_width_domain():
    with pytest.raises(SchemaError):
        validate_effective({"link": {"channel": 132, "width": 80, "region": "US"}})


def test_validate_effective_accepts_10mhz():
    # 10 MHz is supported (underclocked); must not raise.
    validate_effective({"link": {"channel": 132, "width": 10, "region": "US"}})


def test_validate_effective_ok():
    validate_effective({
        "link": {"channel": 132, "width": 40, "rxpower": 19, "region": "US",
                 "linkId": 7669206, "beamforming": {"enabled": False}, "wlans": "auto"},
        "wfb": {"profile": "gs", "mavlink": {"peer": "connect://127.0.0.1:14550"}, "raw": {}},
        "droneLink": {"endpoint": "http://10.5.0.10:8080"},
    })


def _eff(**ctl):
    base = {"link": {"channel": 132, "width": 40, "region": "US"},
            "dynamicLink": {"enabled": False,
                             "controller": {"maxMcs": 5,
                                            "radioProfile": "m8812eu2",
                                            "dronePort": 9999,
                                            "tuning": {}}}}
    base["dynamicLink"]["controller"].update(ctl)
    return base


def test_effective_accepts_valid_dynamiclink():
    schema.validate_effective(_eff())  # no raise


def test_effective_rejects_bad_max_mcs():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(maxMcs=9))


def test_effective_rejects_unknown_radio_profile():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(radioProfile="nonexistent"))


def test_effective_accepts_known_radio_profile():
    schema.validate_effective(_eff(radioProfile="m8812eu2"))  # no raise


def test_effective_rejects_bad_drone_port():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(dronePort=70000))


def test_validate_dynamiclink_controller():
    import pytest
    from fpvdgs import schema
    schema.validate_effective({
        "link": {"channel": 132, "region": "US", "width": 40},
        "dynamicLink": {"enabled": True,
                         "controller": {"maxMcs": 5, "radioProfile": "m8812eu2",
                                        "dronePort": 9999}},
    })
    with pytest.raises(schema.SchemaError):
        schema.validate_effective({
            "link": {"channel": 132, "region": "US", "width": 40},
            "dynamicLink": {"controller": {"maxMcs": 9}},   # out of 0..7
        })


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
    import json, pathlib
    p = pathlib.Path(__file__).resolve().parents[2] / "etc" / "defaults.json"
    cfg = json.loads(p.read_text())
    assert "pixelpilot" in cfg
    assert cfg["pixelpilot"]["enabled"] is True
    schema.validate_effective(cfg)  # no raise


def test_effective_accepts_rxpower_in_dbm_range():
    schema.validate_effective({"link": {"channel": 132, "region": "US", "width": 20,
                                        "rxpower": 20}})
    schema.validate_effective({"link": {"channel": 132, "region": "US", "width": 20,
                                        "rxpower": None}})   # null ok (driver default)


def test_effective_rejects_rxpower_out_of_dbm_range():
    with pytest.raises(SchemaError):
        schema.validate_effective({"link": {"channel": 132, "region": "US", "width": 20,
                                            "rxpower": 2007}})   # raw/legacy value, now invalid
    with pytest.raises(SchemaError):
        schema.validate_effective({"link": {"channel": 132, "region": "US", "width": 20,
                                            "rxpower": 31}})


def test_beamforming_enabled_bool_ok():
    from fpvdgs import schema
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
