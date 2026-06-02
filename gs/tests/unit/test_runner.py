import pytest

from fpvdgs import runner


def test_build_argv():
    assert runner.build_argv("gs", ["wlan0", "wlan1"]) == [
        "--profiles", "gs", "--wlans", "wlan0", "wlan1"]


def test_main_requires_cfg_env(monkeypatch):
    monkeypatch.delenv("WIFIBROADCAST_CFG", raising=False)
    with pytest.raises(SystemExit):
        runner.main(["--profiles", "gs", "--wlans", "wlan0"])
