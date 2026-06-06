import pytest
from fpvdgs import schema

def test_probe_in_top_keys():
    schema.validate_config_patch({"probe": {"enabled": True}})  # no raise

def test_probe_defaults_valid():
    schema.validate_effective({
        "link": {"width": 20, "region": "US", "channel": 161},
        "probe": {"enabled": False, "basePort": 50, "maxStreams": 4, "rxL": 50},
    })

@pytest.mark.parametrize("bad", [
    {"enabled": "yes"},
    {"enabled": True, "basePort": 0},
    {"enabled": True, "basePort": 70000},
    {"enabled": True, "maxStreams": 0},
    {"enabled": True, "rxL": -1},
])
def test_probe_rejects(bad):
    cfg = {"link": {"width": 20, "region": "US", "channel": 161}, "probe": bad}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)
