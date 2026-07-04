import pytest

from fpvdgs.wfb.cards import Card, has_remote, local_ifaces, parse_cards, remote_cards


def test_string_shorthand_is_local():
    cards = parse_cards({"cards": ["wlan1", {"iface": "wlan2", "txPowerDbm": 20}]})
    assert [c.iface for c in cards] == ["wlan1", "wlan2"]
    assert all(c.is_local for c in cards)
    assert cards[1].txpower_dbm == 20


def test_remote_card_and_helpers():
    cards = parse_cards(
        {
            "cards": [
                "wlan1",
                {
                    "host": "192.168.1.10",
                    "iface": "wlan0",
                    "sshUser": "admin",
                    "sshPort": 2222,
                    "sshKey": "/etc/fpvd/id",
                    "txPowerDbm": "off",
                },
            ]
        }
    )
    r = remote_cards(cards)
    assert has_remote(cards) and len(r) == 1
    assert (r[0].host, r[0].ssh_user, r[0].ssh_port, r[0].ssh_key) == (
        "192.168.1.10",
        "admin",
        2222,
        "/etc/fpvd/id",
    )
    assert r[0].is_rx_only
    assert local_ifaces(cards) == ["wlan1"]


def test_legacy_wlans_list_migrates():
    assert [c.iface for c in parse_cards({"wlans": ["wlan1"]})] == ["wlan1"]


def test_auto_passthrough_and_auto_plus_remote_rejected():
    assert parse_cards({"cards": "auto"}) == "auto"
    assert parse_cards({"wlans": "auto"}) == "auto"
    with pytest.raises(ValueError):
        parse_cards(
            {"cards": "auto", "wlans": [{"host": "x", "iface": "y"}]}
        )  # never valid shapes anyway


def test_cards_key_wins_over_wlans_when_concrete():
    cards = parse_cards({"cards": ["wlanA"], "wlans": ["wlanB"]})
    assert [c.iface for c in cards] == ["wlanA"]


def test_default_style_effective_link_falls_back_to_legacy_wlans():
    # A merged effective config always carries the default `cards: "auto"`
    # placeholder alongside a legacy `wlans` overlay override — must resolve
    # to the wlans list, not "auto".
    cards = parse_cards({"cards": "auto", "wlans": ["wlan0"]})
    assert [c.iface for c in cards] == ["wlan0"]


def test_card_object_requires_iface():
    with pytest.raises(ValueError):
        parse_cards({"cards": [{"host": "x"}]})


def test_is_local_and_is_rx_only_defaults():
    c = Card(host=None, iface="wlan0")
    assert c.is_local
    assert not c.is_rx_only


def test_init_script_parses_from_remote_card():
    cards = parse_cards(
        {
            "cards": [
                {
                    "host": "192.168.1.10",
                    "iface": "wlan0",
                    "initScript": "iw phy phy0 interface add wlan0 type monitor || true",
                }
            ]
        }
    )
    assert cards[0].init_script == "iw phy phy0 interface add wlan0 type monitor || true"


def test_init_script_defaults_to_none_without_key():
    cards = parse_cards({"cards": [{"host": "192.168.1.10", "iface": "wlan0"}]})
    assert cards[0].init_script is None


def test_init_script_defaults_to_none_for_string_shorthand():
    cards = parse_cards({"cards": ["wlan1"]})
    assert cards[0].init_script is None
