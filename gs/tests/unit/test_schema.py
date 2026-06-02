import pytest

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
