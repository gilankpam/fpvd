from fpvdgs import radio


def test_iw_args_channel_ht20():
    assert radio.iw_args("wlan0", 132, 20) == [
        "iw", "dev", "wlan0", "set", "channel", "132", "HT20"]


def test_iw_args_10mhz():
    assert radio.iw_args("wlan0", 132, 10) == [
        "iw", "dev", "wlan0", "set", "channel", "132", "10MHz"]


def test_iw_args_ht40():
    assert radio.iw_args("wlan1", 132, 40) == [
        "iw", "dev", "wlan1", "set", "channel", "132", "HT40+"]


def test_iw_args_freq_when_above_2000():
    assert radio.iw_args("wlan0", 5660, 20) == [
        "iw", "dev", "wlan0", "set", "freq", "5660", "HT20"]


def test_retune_commands_region_channel_txpower_in_order():
    cmds = radio.retune_commands(
        ["wlan0"], {"region": "US", "channel": 132, "width": 20, "txpower": 2200})
    assert cmds == [
        ["iw", "reg", "set", "US"],
        ["iw", "dev", "wlan0", "set", "channel", "132", "HT20"],
        ["iw", "dev", "wlan0", "set", "txpower", "fixed", "2200"],
    ]


def test_retune_commands_txpower_none_sets_auto():
    cmds = radio.retune_commands(
        ["wlan0"], {"region": "US", "channel": 132, "width": 10, "txpower": None})
    assert ["iw", "reg", "set", "US"] in cmds
    assert ["iw", "dev", "wlan0", "set", "channel", "132", "10MHz"] in cmds
    assert ["iw", "dev", "wlan0", "set", "txpower", "auto"] in cmds


def test_retune_commands_multi_wlan_sets_region_once():
    cmds = radio.retune_commands(["a", "b"], {"region": "US", "channel": 132, "width": 20})
    assert cmds.count(["iw", "reg", "set", "US"]) == 1
    assert ["iw", "dev", "a", "set", "channel", "132", "HT20"] in cmds
    assert ["iw", "dev", "b", "set", "channel", "132", "HT20"] in cmds
