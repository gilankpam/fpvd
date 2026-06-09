import pytest

from fpvdgs.facade import build_config_tree, split_patch, FacadeError

GS_EFF = {
    "link": {"channel": 132, "width": 20, "linkId": 7669206,
             "beamforming": {"enabled": False},
             "region": "US", "rxpower": None, "wlans": "auto"},
    "dynamicLink": {"enabled": False,
                    "controller": {"maxMcs": 5, "radioProfile": "m8812eu2",
                                   "dronePort": 9999, "tuning": {}}},
    "wfb": {"profile": "gs"}, "pixelpilot": {"enabled": True},
    "droneLink": {"endpoint": "http://10.5.0.10:8080"},
}
DRONE_CFG = {
    "link": {"channel": 132, "width": 20, "linkId": 7669206,
             "beamforming": {"enabled": False, "remoteMac": "", "ackTimeout": 255, "intervalMs": 100},
             "mcs": 3, "txpower": 25, "txpowerCurve": None, "fec": {"k": 8, "n": 12},
             "stbc": False, "ldpc": False, "mtu": 1500, "wlanAdapter": None},
    "dynamicLink": {"enabled": False, "healthTimeoutMs": 10000, "failsafe": {"mcs": 1},
                    "bitrate": {"minBitrateKbps": 1000}, "fec": {"kMin": 2}},
    "video": {"codec": "h265", "fps": 60}, "image": {"mirror": False},
    "telemetry": {"router": "msposd"}, "recording": {"enabled": False}, "services": {},
}
META = {"droneReachable": True, "droneLastSeen": "2026-06-10T00:00:00Z", "droneStale": False}


def test_link_splits_shared_gs_drone():
    t = build_config_tree(GS_EFF, DRONE_CFG, META)["link"]
    assert t["channel"] == 132 and t["width"] == 20 and t["linkId"] == 7669206
    assert t["beamforming"] == {"enabled": False}
    assert t["gs"] == {"region": "US", "rxpower": None, "wlans": "auto"}
    assert t["drone"]["mcs"] == 3 and t["drone"]["txpower"] == 25
    assert "channel" not in t["drone"] and "beamforming" not in t["drone"]


def test_dynamiclink_splits_controller_applier_enabled():
    dl = build_config_tree(GS_EFF, DRONE_CFG, META)["dynamicLink"]
    assert dl["enabled"] is False
    assert dl["controller"]["maxMcs"] == 5
    assert "enabled" not in dl["applier"]
    assert dl["applier"]["healthTimeoutMs"] == 10000
    assert dl["applier"]["failsafe"] == {"mcs": 1}


def test_wholly_owned_sections_passthrough():
    t = build_config_tree(GS_EFF, DRONE_CFG, META)
    assert t["video"] == {"codec": "h265", "fps": 60}
    assert t["telemetry"] == {"router": "msposd"}
    assert t["wfb"] == {"profile": "gs"}
    assert t["droneLink"] == {"endpoint": "http://10.5.0.10:8080"}
    assert t["_meta"] == META


def test_stale_drone_subtree_is_last_seen_not_blank():
    stale_meta = {"droneReachable": False, "droneLastSeen": "2026-06-10T00:00:00Z", "droneStale": True}
    t = build_config_tree(GS_EFF, DRONE_CFG, stale_meta)
    assert t["link"]["drone"]["mcs"] == 3
    assert t["_meta"]["droneStale"] is True


def test_never_seen_drone_yields_empty_drone_subtrees():
    t = build_config_tree(GS_EFF, None, {"droneReachable": False, "droneStale": True})
    assert t["link"]["drone"] == {}
    assert t["dynamicLink"]["applier"] == {}
    assert t["video"] == {}
    assert t["link"]["gs"]["region"] == "US"
    assert t["dynamicLink"]["controller"]["maxMcs"] == 5


# --- split_patch (unified PATCH routing) ---
def test_split_link_gs_drone_shared():
    gs, drone, shared = split_patch({"link": {"channel": 140, "gs": {"rxpower": 20},
                                              "drone": {"mcs": 4}}})
    assert gs == {"link": {"channel": 140, "rxpower": 20}}
    assert drone == {"link": {"mcs": 4}}
    assert shared is True


def test_split_dynamiclink_controller_applier_enabled():
    gs, drone, shared = split_patch({"dynamicLink": {
        "enabled": True, "controller": {"maxMcs": 6}, "applier": {"failsafe": {"mcs": 2}}}})
    assert gs == {"dynamicLink": {"enabled": True, "controller": {"maxMcs": 6}}}
    assert drone == {"dynamicLink": {"enabled": True, "failsafe": {"mcs": 2}}}
    assert shared is False


def test_split_wholly_owned_sections():
    gs, drone, _ = split_patch({"video": {"bitrate": 9000}, "pixelpilot": {"enabled": False}})
    assert drone == {"video": {"bitrate": 9000}}
    assert gs == {"pixelpilot": {"enabled": False}}


def test_split_rejects_meta_and_unknown():
    with pytest.raises(FacadeError):
        split_patch({"_meta": {"droneStale": False}})
    with pytest.raises(FacadeError):
        split_patch({"bogus": {}})


def test_split_link_only_shared_no_drone_push():
    # a shared-link-only patch goes to GS pending only; drone push deferred to apply
    gs, drone, shared = split_patch({"link": {"channel": 140}})
    assert gs == {"link": {"channel": 140}}
    assert drone == {}
    assert shared is True
