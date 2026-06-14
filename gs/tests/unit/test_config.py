import json

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
    s = ConfigStore({"link": {"channel": 132, "width": 40}},
                    {"link": {"width": 20}})
    assert s.effective() == {"link": {"channel": 132, "width": 20}}


def test_missing_key_falls_back_to_default():
    s = ConfigStore({"link": {"channel": 132}, "drone": {"endpoint": "x"}},
                    {"link": {"channel": 9}})
    assert s.effective()["drone"]["endpoint"] == "x"   # untouched key defaults


def test_patch_accumulates_into_pending_without_touching_effective():
    s = ConfigStore({"link": {"channel": 132, "width": 40}})
    s.patch({"link": {"width": 20}})
    assert s.effective() == {"link": {"channel": 132, "width": 40}}
    assert s.pending() == {"link": {"channel": 132, "width": 20}}


def test_commit_promotes_pending_and_persists_full(tmp_path):
    cfg = tmp_path / "config.json"
    s = ConfigStore({"link": {"channel": 132, "width": 40}},
                    config_path=str(cfg))
    s.patch({"link": {"channel": 100}})
    s.commit()
    # persisted file is the FULL effective config (no sparse diff)
    assert json.loads(cfg.read_text()) == {"link": {"channel": 100, "width": 40}}


def test_reset_restores_defaults(tmp_path):
    cfg = tmp_path / "config.json"
    s = ConfigStore({"link": {"channel": 132}}, {"link": {"channel": 5}},
                    config_path=str(cfg))
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
    assert s.effective()["drone"] == default_config()["drone"]   # defaulted


def test_legacy_key_in_file_does_not_break_load(tmp_path):
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({"probe": {"enabled": True}}))
    s = ConfigStore.load(str(cfg))               # must not raise
    assert s.effective()["link"]["channel"] == default_config()["link"]["channel"]


def test_load_patch_commit_reload_roundtrip(tmp_path):
    path = str(tmp_path / "config.json")
    s = ConfigStore.load(path)          # no file → starts from code defaults
    s.patch({"link": {"channel": 100}})
    s.commit()
    s2 = ConfigStore.load(path)         # reload from the persisted full config
    assert s2.effective()["link"]["channel"] == 100
    assert s2.effective()["drone"] == default_config()["drone"]  # unchanged keys survive


def test_load_warns_on_unknown_keys(tmp_path, caplog):
    import logging
    cfg = tmp_path / "config.json"
    cfg.write_text(json.dumps({
        "bogusTop": 1,
        "dynamicLink": {"tuning": {"gate": {}}, "selector": {}},
    }))
    with caplog.at_level(logging.WARNING):
        s = ConfigStore.load(str(cfg))      # must not raise
    msgs = " ".join(r.message for r in caplog.records)
    assert "bogusTop" in msgs
    assert "tuning" in msgs                 # stale dynamicLink key warned
    assert s.effective()["link"]["channel"] == default_config()["link"]["channel"]
