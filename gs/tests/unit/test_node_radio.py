import subprocess

from fpvdgs.node_radio import card_radio, nodes_status, query_cards
from fpvdgs.wfb.cards import Card

IW_OUTPUT = """Interface wlan0
\ttype monitor
\tchannel 132 (5660 MHz), width: 20 MHz
\ttxpower 20.00 dBm
"""

PARSED_RADIO = {
    "type": "monitor",
    "channel": 132,
    "freqMhz": 5660,
    "widthMhz": 20,
    "txpowerDbm": 20.0,
}


class FakeCompleted:
    def __init__(self, returncode=0, stdout=""):
        self.returncode = returncode
        self.stdout = stdout


def _iw_iface(argv) -> str:
    """Recover the queried iface from either argv shape (local or remote)."""
    if argv[0] == "iw":
        return argv[2]
    return argv[-1].split()[-2]  # "...@host", "iw dev <iface> info"


def test_card_radio_local_success():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        return FakeCompleted(0, IW_OUTPUT)

    entry = card_radio(Card(host=None, iface="wlan0"), runner=runner)
    assert entry == {
        "host": None,
        "iface": "wlan0",
        "local": True,
        "reachable": True,
        "radio": PARSED_RADIO,
    }
    assert calls == [["iw", "dev", "wlan0", "info"]]


def test_card_radio_remote_success():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        return FakeCompleted(0, IW_OUTPUT)

    card = Card(
        host="10.0.0.5",
        iface="wlan1",
        ssh_user="admin",
        ssh_port=2222,
        ssh_key="/etc/fpvd/id",
    )
    entry = card_radio(card, runner=runner)
    assert entry["host"] == "10.0.0.5"
    assert entry["iface"] == "wlan1"
    assert entry["local"] is False
    assert entry["reachable"] is True
    assert entry["radio"] == PARSED_RADIO

    argv = calls[0]
    assert argv[0] == "ssh"
    assert argv[argv.index("-p") + 1] == "2222"
    assert argv[argv.index("-i") + 1] == "/etc/fpvd/id"
    assert "BatchMode=yes" in argv
    assert any("ConnectTimeout" in a for a in argv)
    assert argv[-2:] == ["admin@10.0.0.5", "iw dev wlan1 info"]


def test_card_radio_remote_omits_key_when_unset():
    calls = []

    def runner(argv, **kwargs):
        calls.append(argv)
        return FakeCompleted(0, IW_OUTPUT)

    card_radio(Card(host="10.0.0.5", iface="wlan1"), runner=runner)
    assert "-i" not in calls[0]


def test_card_radio_timeout_never_raises():
    def runner(argv, **kwargs):
        raise subprocess.TimeoutExpired(cmd=argv, timeout=6)

    entry = card_radio(Card(host=None, iface="wlan0"), runner=runner)
    assert entry["reachable"] is False
    assert "radio" not in entry


def test_card_radio_nonzero_rc_never_raises():
    def runner(argv, **kwargs):
        return FakeCompleted(1, "")

    entry = card_radio(Card(host=None, iface="wlan0"), runner=runner)
    assert entry["reachable"] is False
    assert "radio" not in entry


def test_card_radio_oserror_never_raises():
    def runner(argv, **kwargs):
        raise OSError("ssh binary missing")

    entry = card_radio(Card(host="10.0.0.5", iface="wlan1"), runner=runner)
    assert entry["reachable"] is False
    assert "radio" not in entry


def test_query_cards_preserves_order_local_and_remote_mix():
    cards = [
        Card(host=None, iface="wlan0"),
        Card(host="10.0.0.5", iface="wlan1"),
        Card(host=None, iface="wlan2"),
    ]

    def runner(argv, **kwargs):
        iface = _iw_iface(argv)
        return FakeCompleted(0, IW_OUTPUT.replace("wlan0", iface))

    entries = query_cards(cards, runner=runner)
    assert [e["iface"] for e in entries] == ["wlan0", "wlan1", "wlan2"]
    assert [e["local"] for e in entries] == [True, False, True]
    assert all(e["reachable"] for e in entries)


def test_query_cards_empty_list():
    assert query_cards([]) == []


def test_nodes_status_concrete_cards_local_and_remote():
    def runner(argv, **kwargs):
        iface = _iw_iface(argv)
        return FakeCompleted(0, IW_OUTPUT.replace("wlan0", iface))

    effective = {
        "link": {
            "cards": [
                "wlan0",
                {"host": "10.0.0.5", "iface": "wlan1", "sshUser": "admin"},
            ]
        }
    }
    result = nodes_status(effective, runner=runner)
    assert [n["iface"] for n in result["nodes"]] == ["wlan0", "wlan1"]
    assert result["nodes"][0]["local"] is True
    assert result["nodes"][1]["local"] is False


def test_nodes_status_auto_with_detector_expands_local_cards():
    def runner(argv, **kwargs):
        return FakeCompleted(0, IW_OUTPUT)

    effective = {"link": {"cards": "auto"}}
    result = nodes_status(effective, nic_detector=lambda: ["wlan0"], runner=runner)
    assert result == {
        "nodes": [
            {
                "host": None,
                "iface": "wlan0",
                "local": True,
                "reachable": True,
                "radio": PARSED_RADIO,
            }
        ]
    }


def test_nodes_status_auto_without_detector_yields_empty():
    effective = {"link": {"cards": "auto"}}
    # No runner override either: must short-circuit before any subprocess call.
    assert nodes_status(effective) == {"nodes": []}
