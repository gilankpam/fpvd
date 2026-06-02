"""Smoke test: every lifted dynlink core module imports cleanly (no PyYAML,
no wfb_ng, no leftover dynamic_link.* imports)."""

import importlib

import pytest

MODULES = [
    "decision", "predictor", "dynamic_fec", "bitrate", "stats_client",
    "signals", "return_link", "wire", "drone_config", "tunnel_listener",
    "policy",
]


@pytest.mark.parametrize("name", MODULES)
def test_module_imports(name):
    mod = importlib.import_module(f"fpvdgs.dynlink.{name}")
    assert mod is not None


def test_no_yaml_dependency_in_core():
    # None of the core modules may pull PyYAML.
    import sys
    for name in MODULES:
        importlib.import_module(f"fpvdgs.dynlink.{name}")
    assert "yaml" not in sys.modules
