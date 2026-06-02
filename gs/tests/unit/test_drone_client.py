import pytest

from fpvdgs.drone_client import DroneClient, DroneUnreachable


def test_healthz_true_when_up(fake_drone):
    c = DroneClient(fake_drone["endpoint"])
    assert c.healthz() is True


def test_healthz_false_when_down():
    c = DroneClient("http://127.0.0.1:9", timeout=0.3)  # nothing listens on :9
    assert c.healthz() is False


def test_patch_then_apply_records_calls(fake_drone):
    c = DroneClient(fake_drone["endpoint"])
    c.patch_config({"link": {"channel": 100}})
    c.apply()
    methods = [(m, p) for (m, p, _b) in fake_drone["calls"]]
    assert ("PATCH", "/config") in methods
    assert ("POST", "/apply") in methods


def test_apply_raises_on_drone_error(fake_drone):
    fake_drone["fail"] = True
    c = DroneClient(fake_drone["endpoint"])
    with pytest.raises(DroneUnreachable):
        c.apply()


def test_unreachable_raises(monkeypatch):
    c = DroneClient("http://127.0.0.1:9", timeout=0.3)
    with pytest.raises(DroneUnreachable):
        c.get_status()
