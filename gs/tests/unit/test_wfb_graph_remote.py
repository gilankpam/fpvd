"""Golden argv tests for the remote-wiring graph builder
(`build_graph_remote`, gs/fpvdgs/wfb/graph.py).

Fixture: the same 2-node cluster as `test_wfb_cluster.py` — two local
cards (wlan1, wlan2) plus one remote RX-only card on 192.168.1.10 (wlan0).
`plan_cluster` on that fixture yields server_port
{video:10000, mavlink:10001, tunnel:10002}, local injector_base
{video:11000, mavlink:11002, tunnel:11004}, and
peers["mavlink"] == ["127.0.0.1:11002,11003", "192.168.1.10:11001"] (see
test_wfb_cluster.py for the full derivation) — this file transcribes those
numbers into the expected `wfb_rx`/`wfb_tx` argv per the design brief.

link_id() is pinned at 7669206 (link_domain="default", same as the Phase 1
golden tests); rand_suffix is pinned to "aaaaaaaa".
"""

import copy

from fpvdgs.config_defaults import default_config
from fpvdgs.wfb.cards import Card
from fpvdgs.wfb.cluster import plan_cluster
from fpvdgs.wfb.graph import GsGraph, build_graph, build_graph_remote

LOCAL = [Card(host=None, iface="wlan1"), Card(host=None, iface="wlan2")]
REMOTE = [Card(host="192.168.1.10", iface="wlan0", txpower_dbm="off")]
LID = "7669206"
SFX = "aaaaaaaa"


def _sfx():
    return SFX


def _build(effective=None, cards=None, server_address="10.5.0.1"):
    cards = cards if cards is not None else (LOCAL + REMOTE)
    plan = plan_cluster(cards)
    return build_graph_remote(
        effective or default_config(), cards, plan, server_address, rand_suffix=_sfx
    )


# ---- GS-side aggregator/distributor legs -------------------------------


def test_video_rx_aggregator_argv():
    g = _build()
    assert g.video_rx.argv == [
        "/usr/bin/wfb_rx",
        "-a",
        "10000",
        "-p",
        "0",
        "-c",
        "127.0.0.1",
        "-u",
        "5600",
        "-K",
        "/etc/gs.key",
        "-R",
        "2097152",
        "-s",
        "2097152",
        "-l",
        "100",
        "-i",
        LID,
        "-D",
        "8110",
    ]
    assert g.video_rx.kind == "rx"
    assert g.video_rx.parser == "rx"
    assert g.video_rx.unix_path is None


def test_video_rx_aggregator_plaintext_variant_drops_dash_k():
    cfg = copy.deepcopy(default_config())
    cfg["link"]["videoEncryption"] = False
    g = _build(cfg)
    assert "-K" not in g.video_rx.argv
    assert "/etc/gs.key" not in g.video_rx.argv
    # everything else about the aggregator leg is unaffected
    assert g.video_rx.argv[:5] == ["/usr/bin/wfb_rx", "-a", "10000", "-p", "0"]
    assert g.video_rx.argv[-2:] == ["-D", "8110"]


def test_mavlink_rx_aggregator_argv():
    g = _build()
    assert g.mavlink_rx.argv == [
        "/usr/bin/wfb_rx",
        "-a",
        "10001",
        "-p",
        "16",
        "-U",
        "mavlink-rx-aaaaaaaa",
        "-K",
        "/etc/gs.key",
        "-R",
        "2097152",
        "-s",
        "2097152",
        "-l",
        "100",
        "-i",
        LID,
    ]
    assert g.mavlink_rx.kind == "rx"
    assert g.mavlink_rx.parser == "rx"
    assert g.mavlink_rx.unix_path == "mavlink-rx-aaaaaaaa"


def test_tunnel_rx_aggregator_argv():
    g = _build()
    assert g.tunnel_rx.argv == [
        "/usr/bin/wfb_rx",
        "-a",
        "10002",
        "-p",
        "32",
        "-U",
        "tunnel-rx-aaaaaaaa",
        "-K",
        "/etc/gs.key",
        "-R",
        "2097152",
        "-s",
        "2097152",
        "-l",
        "100",
        "-i",
        LID,
    ]
    assert g.tunnel_rx.kind == "rx"
    assert g.tunnel_rx.parser == "rx"
    assert g.tunnel_rx.unix_path == "tunnel-rx-aaaaaaaa"


def test_mavlink_tx_distributor_argv_and_peers():
    g = _build()
    assert g.mavlink_tx.argv == [
        "/usr/bin/wfb_tx",
        "-d",
        "-f",
        "data",
        "-p",
        "144",
        "-U",
        "mavlink-tx-aaaaaaaa",
        "-K",
        "/etc/gs.key",
        "-B",
        "20",
        "-G",
        "long",
        "-S",
        "1",
        "-L",
        "1",
        "-M",
        "1",
        "-k",
        "1",
        "-n",
        "2",
        "-T",
        "0",
        "-F",
        "0",
        "-i",
        LID,
        "-R",
        "2097152",
        "-s",
        "2097152",
        "-l",
        "100",
        "-C",
        "0",
        "127.0.0.1:11002,11003",
        "192.168.1.10:11001",
    ]
    assert g.mavlink_tx.kind == "tx"
    assert g.mavlink_tx.parser == "tx"
    assert g.mavlink_tx.unix_path == "mavlink-tx-aaaaaaaa"


def test_tunnel_tx_distributor_argv_and_peers():
    g = _build()
    assert g.tunnel_tx.argv == [
        "/usr/bin/wfb_tx",
        "-d",
        "-f",
        "data",
        "-p",
        "160",
        "-U",
        "tunnel-tx-aaaaaaaa",
        "-K",
        "/etc/gs.key",
        "-B",
        "20",
        "-G",
        "long",
        "-S",
        "1",
        "-L",
        "1",
        "-M",
        "1",
        "-k",
        "1",
        "-n",
        "2",
        "-T",
        "0",
        "-F",
        "0",
        "-i",
        LID,
        "-R",
        "2097152",
        "-s",
        "2097152",
        "-l",
        "100",
        "-C",
        "0",
        "127.0.0.1:11004,11005",
        "192.168.1.10:11002",
    ]
    assert g.tunnel_tx.kind == "tx"
    assert g.tunnel_tx.parser == "tx"
    assert g.tunnel_tx.unix_path == "tunnel-tx-aaaaaaaa"


# ---- local forwarders/injectors ----------------------------------------


def test_local_forwarders_present_one_per_service_and_no_stats_parser():
    g = _build()
    assert [f.name for f in g.local_forwarders] == ["video fwd", "mavlink fwd", "tunnel fwd"]
    for f in g.local_forwarders:
        assert f.kind == "rx"
        assert f.parser is None  # forwarders emit no IPC_MSG stats (see graph.py docstring)
        assert f.unix_path is None

    video_fwd = g.local_forwarders[0]
    assert video_fwd.argv == [
        "/usr/bin/wfb_rx",
        "-f",
        "-c",
        "127.0.0.1",
        "-u",
        "10000",
        "-p",
        "0",
        "-i",
        LID,
        "-R",
        "2097152",
        "wlan1",
        "wlan2",
    ]

    mav_fwd = g.local_forwarders[1]
    assert mav_fwd.argv == [
        "/usr/bin/wfb_rx",
        "-f",
        "-c",
        "127.0.0.1",
        "-u",
        "10001",
        "-p",
        "16",
        "-i",
        LID,
        "-R",
        "2097152",
        "wlan1",
        "wlan2",
    ]

    tun_fwd = g.local_forwarders[2]
    assert tun_fwd.argv == [
        "/usr/bin/wfb_rx",
        "-f",
        "-c",
        "127.0.0.1",
        "-u",
        "10002",
        "-p",
        "32",
        "-i",
        LID,
        "-R",
        "2097152",
        "wlan1",
        "wlan2",
    ]


def test_local_injectors_present_only_for_tx_services_and_no_stats_parser():
    g = _build()
    # video has no tx stream id -> no injector for it; only mavlink+tunnel.
    assert [inj.name for inj in g.local_injectors] == ["mavlink inj", "tunnel inj"]
    for inj in g.local_injectors:
        assert inj.kind == "tx"
        assert inj.parser is None  # injectors emit no IPC_MSG stats (see graph.py docstring)
        assert inj.unix_path is None

    mav_inj = g.local_injectors[0]
    assert mav_inj.argv == [
        "/usr/bin/wfb_tx",
        "-I",
        "11002",
        "-R",
        "2097152",
        "wlan1",
        "wlan2",
    ]

    tun_inj = g.local_injectors[1]
    assert tun_inj.argv == [
        "/usr/bin/wfb_tx",
        "-I",
        "11004",
        "-R",
        "2097152",
        "wlan1",
        "wlan2",
    ]


def test_no_local_cards_yields_no_forwarders_or_injectors():
    g = _build(cards=REMOTE)
    assert g.local_forwarders == []
    assert g.local_injectors == []


# ---- node scripts --------------------------------------------------------


def test_node_scripts_has_exactly_the_remote_node():
    g = _build()
    assert set(g.node_scripts) == {"192.168.1.10"}
    script = g.node_scripts["192.168.1.10"]
    assert isinstance(script, str)
    assert "wfb_rx -f -c 10.5.0.1" in script
    assert "wfb_tx -I 11001" in script  # mavlink injector base for the remote node


def test_all_local_cards_yields_no_node_scripts():
    g = _build(cards=LOCAL)
    assert g.node_scripts == {}


# ---- all-local guard: build_graph output is untouched -------------------


def test_build_graph_all_local_is_byte_identical_to_phase1_golden():
    wlans = ["wlan0", "wlan1"]
    g = build_graph(default_config(), wlans, rand_suffix=_sfx)
    assert g.video_rx.argv == [
        "/usr/bin/wfb_rx",
        "-p",
        "0",
        "-c",
        "127.0.0.1",
        "-u",
        "5600",
        "-K",
        "/etc/gs.key",
        "-R",
        "2097152",
        "-s",
        "2097152",
        "-l",
        "100",
        "-i",
        LID,
        "-D",
        "8110",
        "wlan0",
        "wlan1",
    ]
    assert g.local_forwarders == []
    assert g.local_injectors == []
    assert g.node_scripts == {}
    assert isinstance(g, GsGraph)
