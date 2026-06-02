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
