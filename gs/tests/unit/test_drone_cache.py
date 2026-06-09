from fpvdgs.drone_cache import DroneConfigCache
from fpvdgs.drone_client import DroneUnreachable


class FakeDrone:
    def __init__(self):
        self.cfg = {"link": {"mcs": 3}}
        self.up = True
    def get_config(self):
        if not self.up:
            raise DroneUnreachable("down")
        return self.cfg


def test_live_read_refreshes_snapshot_and_meta():
    d = FakeDrone()
    clk = iter(["2026-06-10T00:00:01Z", "2026-06-10T00:00:02Z"])
    cache = DroneConfigCache(d, clock=lambda: next(clk))
    cfg, meta = cache.read()
    assert cfg == {"link": {"mcs": 3}}
    assert meta == {"droneReachable": True, "droneLastSeen": "2026-06-10T00:00:01Z",
                    "droneStale": False}


def test_unreachable_serves_last_seen_with_stale_meta():
    d = FakeDrone()
    clk = iter(["2026-06-10T00:00:01Z", "2026-06-10T00:00:09Z"])
    cache = DroneConfigCache(d, clock=lambda: next(clk))
    cache.read()                      # seeds the snapshot at ...01Z
    d.up = False
    cfg, meta = cache.read()
    assert cfg == {"link": {"mcs": 3}}   # last-seen, not None
    assert meta["droneReachable"] is False
    assert meta["droneStale"] is True
    assert meta["droneLastSeen"] == "2026-06-10T00:00:01Z"   # the last SUCCESSFUL read


def test_never_seen_drone_returns_none_cfg():
    d = FakeDrone(); d.up = False
    cache = DroneConfigCache(d, clock=lambda: "2026-06-10T00:00:09Z")
    cfg, meta = cache.read()
    assert cfg is None
    assert meta == {"droneReachable": False, "droneLastSeen": None, "droneStale": True}


def test_snapshot_is_isolated_from_caller_mutation():
    # the cache must deep-copy so a caller mutating the returned cfg
    # does not corrupt the stored snapshot
    d = FakeDrone()
    cache = DroneConfigCache(d, clock=lambda: "2026-06-10T00:00:01Z")
    cfg, _ = cache.read()
    cfg["link"]["mcs"] = 999
    d.up = False
    cfg2, _ = cache.read()
    assert cfg2["link"]["mcs"] == 3   # snapshot untouched by the earlier mutation
