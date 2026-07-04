from fpvdgs.wfb.cards import Card
from fpvdgs.wfb.cluster import cluster_wlan_id, plan_cluster

LOCAL = [Card(host=None, iface="wlan1"), Card(host=None, iface="wlan2")]
REMOTE = [Card(host="192.168.1.10", iface="wlan0", txpower_dbm="off")]


def test_plan_ports_and_peers():
    p = plan_cluster(LOCAL + REMOTE)
    assert p.server_port == {"video": 10000, "mavlink": 10001, "tunnel": 10002}
    # local node: video 11000+11001, mavlink 11002+11003, tunnel 11004+11005
    assert p.injector_base[("127.0.0.1", "mavlink")] == 11002
    # remote node has its OWN counter: video 11000, mavlink 11001, tunnel 11002
    assert p.injector_base[("192.168.1.10", "tunnel")] == 11002
    # peers in sorted node order; local sorts first ("127..." < "192...")
    assert p.peers["mavlink"] == ["127.0.0.1:11002,11003", "192.168.1.10:11001"]


def test_rx_only_ids():
    p = plan_cluster(LOCAL + REMOTE)
    assert p.rx_only_wlan_ids == frozenset({cluster_wlan_id("192.168.1.10", 0)})


def test_cluster_wlan_id_encoding():
    assert cluster_wlan_id("127.0.0.1", 1) == ((0x7F000001) << 24) | 1
