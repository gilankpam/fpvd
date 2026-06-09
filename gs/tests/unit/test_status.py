from fpvdgs.status import parse_iw_info, build_status

IW = """Interface wlx84fc146c36e6
\tifindex 5
\ttype monitor
\tchannel 132 (5660 MHz), width: 40 MHz, center1: 5670 MHz
\ttxpower 19.00 dBm
"""


def test_parse_iw_info():
    d = parse_iw_info(IW)
    assert d["type"] == "monitor"
    assert d["channel"] == 132
    assert d["freqMhz"] == 5660
    assert d["widthMhz"] == 40
    assert d["txpowerDbm"] == 19.0


def test_build_status_shape():
    s = build_status(
        version="0.1.0",
        runner_state={"running": True, "pid": 591, "restarts": 0, "lastExit": None},
        wlans={"wlx84fc146c36e6": parse_iw_info(IW)},
        drone_probe={"reachable": True, "inSync": True, "linkId": 7669206},
    )
    assert s["runner"]["pid"] == 591
    assert s["radio"][0]["wlan"] == "wlx84fc146c36e6"
    assert s["radio"][0]["channel"] == 132
    assert s["link"]["droneReachable"] is True
    assert s["link"]["inSync"] is True


def test_build_status_includes_dynamic_link_section():
    from fpvdgs import status
    dl = {"enabled": True, "running": True, "statsConnected": True,
          "decision": {"mcs": 4, "k": 8, "n": 12, "depth": 1,
                       "txpowerDbm": 22, "bitrateKbps": 9000},
          "lastEmitMs": 1234, "emitSeq": 5, "reason": "snr_margin",
          "drone": {"reachable": True, "dynamicLinkActive": True}}
    out = status.build_status("0.1.0", {"running": True}, {}, {"reachable": True},
                              dynamic_link=dl)
    assert out["dynamicLink"]["running"] is True
    assert out["dynamicLink"]["decision"]["mcs"] == 4
    assert out["dynamicLink"]["drone"]["dynamicLinkActive"] is True


def test_build_status_omits_dynamic_link_when_absent():
    from fpvdgs import status
    out = status.build_status("0.1.0", {"running": True}, {}, {"reachable": True})
    assert "dynamicLink" not in out


def _runner_state():
    return {"running": True, "pid": 1, "restarts": 0, "autoRestarts": 0,
            "lastExit": None, "fault": False}


def test_status_omits_pixelpilot_when_not_given():
    out = build_status("1.0", _runner_state(), {}, {"reachable": True})
    assert "pixelpilot" not in out


def test_status_includes_pixelpilot_block():
    pp = {"enabled": True, "running": True, "pid": 42, "restarts": 0,
          "autoRestarts": 0, "lastExit": None, "fault": False}
    out = build_status("1.0", _runner_state(), {}, {"reachable": True},
                       pixelpilot=pp)
    assert out["pixelpilot"]["running"] is True
    assert out["pixelpilot"]["pid"] == 42


def test_build_status_includes_probe_when_present():
    drone_probe = {"reachable": True, "linkId": 7669206, "inSync": True}
    j = build_status("vX", {"running": True}, {}, drone_probe,
                     probe={"enabled": True, "running": True,
                            "streams": 2, "mcs": {"3": {"per": 0.0}}})
    assert j["probe"]["enabled"] is True
    assert j["probe"]["mcs"]["3"]["per"] == 0.0


def test_build_status_omits_probe_when_none():
    j = build_status("vX", {"running": True}, {},
                     {"reachable": False, "linkId": 1, "inSync": None})
    assert "probe" not in j


def test_adapter_id_mismatch_warns_once(caplog):
    import logging
    from fpvdgs.supervisor import adapter_matches_profile
    # bl-m8812eu2 matches radioProfile m8812eu2; m8731 does not.
    assert adapter_matches_profile("bl-m8812eu2", "m8812eu2") is True
    assert adapter_matches_profile("bl-m8731bu4", "m8812eu2") is False
    assert adapter_matches_profile(None, "m8812eu2") is True   # unknown → no warn


def test_status_omits_beamforming_when_not_given():
    out = build_status("1.0", _runner_state(), {}, {"reachable": True})
    assert "beamforming" not in out


def test_status_includes_beamforming_block():
    bf = {"requested": True, "state": "active", "reason": "",
          "iface": "wlan0", "localMac": "84:fc:14:6c:36:e6",
          "peerMac": "00:c0:ca:dd:ee:ff"}
    out = build_status("1.0", _runner_state(), {}, {"reachable": True},
                       beamforming=bf)
    assert out["beamforming"]["state"] == "active"
    assert out["beamforming"]["peerMac"] == "00:c0:ca:dd:ee:ff"
