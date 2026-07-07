import json
import logging

from fpvdgs.config import ConfigStore, deep_merge
from fpvdgs.config_defaults import default_config


def test_deep_merge_recurses_and_overrides():
    base = {"link": {"channel": 132, "width": 40}, "wfb": {"profile": "gs"}}
    overlay = {"link": {"channel": 100}}
    assert deep_merge(base, overlay) == {
        "link": {"channel": 100, "width": 40},
        "wfb": {"profile": "gs"},
    }


def test_deep_merge_does_not_mutate_inputs():
    base = {"a": {"b": 1}}
    overlay = {"a": {"c": 2}}
    deep_merge(base, overlay)
    assert base == {"a": {"b": 1}}
    assert overlay == {"a": {"c": 2}}


def test_effective_is_defaults_when_no_loaded():
    s = ConfigStore({"link": {"channel": 132}})
    assert s.effective() == {"link": {"channel": 132}}


def test_loaded_deep_merges_onto_defaults():
    s = ConfigStore({"link": {"channel": 132, "width": 40}}, {"link": {"width": 20}})
    assert s.effective() == {"link": {"channel": 132, "width": 20}}


def test_missing_key_falls_back_to_default():
    s = ConfigStore(
        {"link": {"channel": 132}, "drone": {"endpoint": "x"}}, {"link": {"channel": 9}}
    )
    assert s.effective()["drone"]["endpoint"] == "x"  # untouched key defaults


def test_patch_accumulates_into_pending_without_touching_effective():
    s = ConfigStore({"link": {"channel": 132, "width": 40}})
    s.patch({"link": {"width": 20}})
    assert s.effective() == {"link": {"channel": 132, "width": 40}}
    assert s.pending() == {"link": {"channel": 132, "width": 20}}


def test_commit_promotes_pending_and_persists_full(tmp_path):
    cfg = tmp_path / "config.json"
    s = ConfigStore({"link": {"channel": 132, "width": 40}}, config_path=str(cfg))
    s.patch({"link": {"channel": 100}})
    s.commit()
    # persisted file is the FULL effective config (no sparse diff)
    assert json.loads(cfg.read_text()) == {"link": {"channel": 100, "width": 40}}


def test_reset_restores_defaults(tmp_path):
    cfg = tmp_path / "config.json"
    s = ConfigStore({"link": {"channel": 132}}, {"link": {"channel": 5}}, config_path=str(cfg))
    s.reset()
    assert s.effective() == {"link": {"channel": 132}}
    assert json.loads(cfg.read_text()) == {"link": {"channel": 132}}


def test_load_uses_code_defaults_when_no_file(tmp_path):
    s = ConfigStore.load(str(tmp_path / "absent.json"))
    assert s.effective() == default_config()


def test_load_merges_file_onto_code_defaults(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"link": {"channel": 100}}))
    s = ConfigStore.load(str(cfg))
    assert s.effective()["link"]["channel"] == 100
    assert s.effective()["drone"] == default_config()["drone"]  # defaulted


def test_legacy_key_in_file_does_not_break_load(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"probe": {"enabled": True}}))
    s = ConfigStore.load(str(cfg))  # must not raise
    assert s.effective()["link"]["channel"] == default_config()["link"]["channel"]


def test_load_patch_commit_reload_roundtrip(tmp_path):
    path = str(tmp_path / "config.json")
    s = ConfigStore.load(path)  # no file → starts from code defaults
    s.patch({"link": {"channel": 100}})
    s.commit()
    s2 = ConfigStore.load(path)  # reload from the persisted full config
    assert s2.effective()["link"]["channel"] == 100
    assert s2.effective()["drone"] == default_config()["drone"]  # unchanged keys survive


def test_load_warns_on_unknown_keys(tmp_path, caplog):
    cfg = tmp_path / "config.json"
    cfg.write_text(
        json.dumps(
            {
                "bogusTop": 1,
                "dynamicLink": {"tuning": {"gate": {}}, "selector": {}},
            }
        )
    )
    with caplog.at_level(logging.WARNING):
        s = ConfigStore.load(str(cfg))  # must not raise
    msgs = " ".join(r.message for r in caplog.records)
    assert "bogusTop" in msgs
    assert "tuning" in msgs  # stale dynamicLink key warned
    assert s.effective()["link"]["channel"] == default_config()["link"]["channel"]
    eff = s.effective()
    assert "bogusTop" not in eff
    assert "tuning" not in eff["dynamicLink"]


def test_stale_dynamic_link_keys_do_not_brick_boot(tmp_path):
    from fpvdgs import schema

    cfg = tmp_path / "config.json"
    cfg.write_text(
        json.dumps(
            {
                "dynamicLink": {
                    "enabled": True,
                    "maxMcs": 5,
                    "tuning": {},
                    "bandwidth": 20,
                    "txpower": {"min": 18, "max": 28},
                    "idrForward": True,
                    "idrPort": 11223,
                }
            }
        )
    )
    s = ConfigStore.load(str(cfg))  # warns + strips
    schema.validate_effective(s.effective())  # must NOT raise (boot path)
    assert "tuning" not in s.effective()["dynamicLink"]
    assert "bandwidth" not in s.effective()["dynamicLink"]


def test_stale_nested_selector_key_does_not_brick_boot(tmp_path):
    """A removed selector knob (e.g. emergencyLossRate) left in a stale
    config.json must be stripped, not bricked — validate_effective is strict on
    dynamicLink.selector keys. This is the live-GS migration after the key was
    dropped; without nested pruning the boot path crash-loops."""
    from fpvdgs import schema

    cfg = tmp_path / "config.json"
    cfg.write_text(
        json.dumps(
            {
                "dynamicLink": {
                    "enabled": True,
                    "maxMcs": 5,
                    "dronePort": 9999,
                    "selector": {
                        "videoDemotePer": 0.05,
                        "emergencyLossRate": 0.05,
                        "starvationWindows": 5,
                    },
                    "smoothing": {"ewmaAlphaRssi": 0.2, "bogusSmoothing": 1},
                }
            }
        )
    )
    s = ConfigStore.load(str(cfg))  # warns + strips
    schema.validate_effective(s.effective())  # must NOT raise (boot path)
    sel = s.effective()["dynamicLink"]["selector"]
    assert "emergencyLossRate" not in sel and sel["videoDemotePer"] == 0.05
    assert "bogusSmoothing" not in s.effective()["dynamicLink"]["smoothing"]


def test_tap_defaults_present():
    from fpvdgs.config_defaults import default_config

    tap = default_config()["dynamicLink"]["tap"]
    assert tap == {"enabled": True, "port": 8110, "staleMs": 500, "captureRaw": False}


def test_stale_tap_key_stripped_on_load():
    from fpvdgs.config import _warn_unknown
    from fpvdgs.config_defaults import default_config

    loaded = {"dynamicLink": {"tap": {"enabled": True, "removedKnob": 3}}}
    pruned = _warn_unknown(loaded, default_config())
    assert "removedKnob" not in pruned["dynamicLink"]["tap"]


def test_dynamic_link_probe_defaults_off():
    from fpvdgs.config_defaults import default_config

    dl = default_config()["dynamicLink"]
    assert dl["probe"] == {"enabled": False}


def test_probe_block_validates():
    import pytest

    from fpvdgs import schema
    from fpvdgs.config_defaults import default_config

    # Test: valid probe block passes
    cfg = default_config()
    cfg["dynamicLink"]["probe"] = {"enabled": True}
    schema.validate_effective(cfg)  # must not raise

    # Test: unknown probe key raises
    cfg = default_config()
    cfg["dynamicLink"]["probe"] = {"enabled": True, "bogus": 1}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)

    # Test: enabled non-bool raises
    cfg = default_config()
    cfg["dynamicLink"]["probe"] = {"enabled": "yes"}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)


def test_wfb_tx_selector_defaults_present():
    cfg = default_config()["wfb"]
    assert cfg["txSelector"] == {
        "rssiDeltaDb": 3,
        "counterRelDelta": 0.1,
        "counterAbsDelta": 3,
    }


def test_stale_tx_selector_key_stripped_on_load():
    """A removed/renamed txSelector knob left in a stale config.json must be
    stripped, not bricked — validate_effective is strict on wfb.txSelector
    keys. Mirrors the dynamicLink.tap strip test."""
    from fpvdgs.config import _warn_unknown

    loaded = {"wfb": {"txSelector": {"rssiDeltaDb": 5, "removedKnob": 1}}}
    pruned = _warn_unknown(loaded, default_config())
    assert "removedKnob" not in pruned["wfb"]["txSelector"]
    assert pruned["wfb"]["txSelector"]["rssiDeltaDb"] == 5


def test_stale_tx_selector_key_does_not_brick_boot(tmp_path):
    from fpvdgs import schema

    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"wfb": {"txSelector": {"rssiDeltaDb": 2, "bogusKnob": 99}}}))
    s = ConfigStore.load(str(cfg))  # warns + strips
    schema.validate_effective(s.effective())  # must NOT raise (boot path)
    txsel = s.effective()["wfb"]["txSelector"]
    assert "bogusKnob" not in txsel
    assert txsel["rssiDeltaDb"] == 2


def test_learned_prior_known_key_survives_loader_bogus_key_stripped(tmp_path, caplog):
    """A valid learnedPrior knob in config.json must survive the tolerant loader;
    an unknown learnedPrior key must be stripped (not crash the boot path).
    Mirrors the selector/smoothing strip tests."""
    from fpvdgs import schema

    cfg = tmp_path / "config.json"
    cfg.write_text(
        json.dumps({"dynamicLink": {"learnedPrior": {"settleTicks": 9, "bogusKnob": 99}}})
    )
    with caplog.at_level(logging.WARNING):
        s = ConfigStore.load(str(cfg))  # must not raise
    schema.validate_effective(s.effective())  # must NOT raise (boot path)
    lp = s.effective()["dynamicLink"]["learnedPrior"]
    assert lp["settleTicks"] == 9  # known knob survives
    assert "bogusKnob" not in lp  # unknown key stripped
    assert any("bogusKnob" in r.message for r in caplog.records)
