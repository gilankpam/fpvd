import json

from fpvdgs.config import ConfigStore, deep_merge


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


def test_effective_is_defaults_when_no_overlay():
    s = ConfigStore({"link": {"channel": 132}})
    assert s.effective() == {"link": {"channel": 132}}


def test_patch_accumulates_into_pending_without_touching_effective():
    s = ConfigStore({"link": {"channel": 132, "width": 40}})
    s.patch({"link": {"width": 20}})
    assert s.effective() == {"link": {"channel": 132, "width": 40}}
    assert s.pending() == {"link": {"channel": 132, "width": 20}}


def test_commit_promotes_pending_to_effective():
    s = ConfigStore({"link": {"channel": 132}})
    s.patch({"link": {"channel": 100}})
    s.commit()
    assert s.effective() == {"link": {"channel": 100}}


def test_reset_drops_overlay_and_pending():
    s = ConfigStore({"link": {"channel": 132}})
    s.patch({"link": {"channel": 100}})
    s.commit()
    s.reset()
    assert s.effective() == {"link": {"channel": 132}}
    assert s.pending() == {"link": {"channel": 132}}


def test_load_and_persist_roundtrip(tmp_path):
    defaults = tmp_path / "defaults.json"
    overlay = tmp_path / "config.json"
    defaults.write_text(json.dumps({"link": {"channel": 132}}))
    s = ConfigStore.load(str(defaults), str(overlay))
    s.patch({"link": {"channel": 100}})
    s.commit()
    assert json.loads(overlay.read_text()) == {"link": {"channel": 100}}
    s2 = ConfigStore.load(str(defaults), str(overlay))
    assert s2.effective() == {"link": {"channel": 100}}


def test_legacy_probe_key_in_overlay_does_not_break_load(tmp_path):
    (tmp_path / "defaults.json").write_text(json.dumps({"link": {"channel": 1}}))
    (tmp_path / "config.json").write_text(json.dumps({"probe": {"enabled": True}}))
    store = ConfigStore.load(str(tmp_path / "defaults.json"),
                             str(tmp_path / "config.json"))
    eff = store.effective()           # must not raise
    assert eff["link"]["channel"] == 1
