"""Golden argv tests for the service-graph builder (gs/fpvdgs/wfb/graph.py).

These argv strings are flight-critical: they must be byte-for-byte what
wfb-ng's `services.py` renders for the "gs" profile (a later task adds an
automated parity test against wfb-ng itself; these golden tests are the
first line of defense in the meantime).

`rand_suffix` is pinned to a fixed 8-char hex-shaped string in every test
(prod uses `os.urandom(4).hex()`, also 8 chars) so the unix-socket names in
the expected argv are stable and shaped like the real thing.
"""

import copy

from fpvdgs.config_defaults import default_config
from fpvdgs.wfb.graph import GsGraph, ServiceSpec, build_graph, link_id
from fpvdgs.wfb.mavproxy import MavlinkConfig
from fpvdgs.wfb.tunnel import TunnelConfig

WLANS = ["wlan0", "wlan1"]
SFX = "aaaaaaaa"  # 8 hex-shaped chars, matching os.urandom(4).hex()'s length


def _sfx():
    return SFX


def _build(effective=None, wlans=None):
    return build_graph(effective or default_config(), wlans or WLANS, rand_suffix=_sfx)


# ---- link_id ----------------------------------------------------------


def test_link_id_default_domain_is_pinned():
    # int.from_bytes(hashlib.sha1(b"default").digest()[:3], "big"), computed
    # once and pinned here; also matches config_defaults.default_config()'s
    # link.linkId (7669206), which is the wfb-ng "default" link_domain value.
    assert link_id("default") == 7669206
    assert link_id() == 7669206  # default arg is "default"


# ---- default-config golden argv ----------------------------------------


def test_video_rx_argv_default():
    g = _build()
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
        "7669206",
        "-D",
        "8110",
        "wlan0",
        "wlan1",
    ]
    assert g.video_rx.kind == "rx"
    assert g.video_rx.parser == "rx"
    assert g.video_rx.unix_path is None


def test_mavlink_rx_argv_default():
    g = _build()
    assert g.mavlink_rx.argv == [
        "/usr/bin/wfb_rx",
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
        "7669206",
        "wlan0",
        "wlan1",
    ]
    assert g.mavlink_rx.kind == "rx"
    assert g.mavlink_rx.parser == "rx"
    assert g.mavlink_rx.unix_path == "mavlink-rx-aaaaaaaa"
    assert g.mav_rx_sock == "mavlink-rx-aaaaaaaa"


def test_mavlink_tx_argv_default():
    g = _build()
    assert g.mavlink_tx.argv == [
        "/usr/bin/wfb_tx",
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
        "7669206",
        "-R",
        "2097152",
        "-s",
        "2097152",
        "-l",
        "100",
        "-C",
        "0",
        "wlan0",
        "wlan1",
    ]
    assert g.mavlink_tx.kind == "tx"
    assert g.mavlink_tx.parser == "tx"
    assert g.mavlink_tx.unix_path == "mavlink-tx-aaaaaaaa"


def test_tunnel_rx_argv_default():
    g = _build()
    assert g.tunnel_rx.argv == [
        "/usr/bin/wfb_rx",
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
        "7669206",
        "wlan0",
        "wlan1",
    ]
    assert g.tunnel_rx.kind == "rx"
    assert g.tunnel_rx.parser == "rx"
    assert g.tunnel_rx.unix_path == "tunnel-rx-aaaaaaaa"
    assert g.tun_rx_sock == "tunnel-rx-aaaaaaaa"


def test_tunnel_tx_argv_default():
    g = _build()
    assert g.tunnel_tx.argv == [
        "/usr/bin/wfb_tx",
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
        "7669206",
        "-R",
        "2097152",
        "-s",
        "2097152",
        "-l",
        "100",
        "-C",
        "0",
        "wlan0",
        "wlan1",
    ]
    assert g.tunnel_tx.kind == "tx"
    assert g.tunnel_tx.parser == "tx"
    assert g.tunnel_tx.unix_path == "tunnel-tx-aaaaaaaa"


# ---- ancillary graph fields ---------------------------------------------


def test_mav_peer_parsed_from_config():
    g = _build()
    assert isinstance(g.mav_peer, MavlinkConfig)
    assert g.mav_peer.peer == "connect://127.0.0.1:14550"


def test_tun_cfg_fixed_shape():
    g = _build()
    assert isinstance(g.tun_cfg, TunnelConfig)
    assert g.tun_cfg.ifname == "gs-wfb"
    assert g.tun_cfg.ifaddr == "10.5.0.1/24"
    assert g.tun_cfg.mtu == 1445
    assert g.tun_cfg.agg_timeout == 0.005


def test_service_spec_is_the_documented_dataclass():
    spec = ServiceSpec(name="x", kind="rx", argv=["a"], parser="rx", unix_path=None)
    assert spec.name == "x"
    assert spec.kind == "rx"
    assert spec.argv == ["a"]
    assert spec.parser == "rx"
    assert spec.unix_path is None


def test_gs_graph_field_names():
    g = _build()
    for field in (
        "video_rx",
        "mavlink_rx",
        "mavlink_tx",
        "tunnel_rx",
        "tunnel_tx",
        "mav_rx_sock",
        "mav_peer",
        "tun_rx_sock",
        "tun_cfg",
    ):
        assert hasattr(g, field)


# ---- variations -----------------------------------------------------------


def test_tap_disabled_drops_dash_d_on_video_rx():
    cfg = copy.deepcopy(default_config())
    cfg["dynamicLink"]["tap"]["enabled"] = False
    g = _build(cfg)
    assert "-D" not in g.video_rx.argv


def test_video_encryption_false_drops_dash_k_only_on_video_rx():
    cfg = copy.deepcopy(default_config())
    cfg["link"]["videoEncryption"] = False
    g = _build(cfg)

    assert "-K" not in g.video_rx.argv
    assert "/etc/gs.key" not in g.video_rx.argv

    # mavlink/tunnel legs stay keyed regardless of link.videoEncryption.
    assert "-K" in g.mavlink_rx.argv
    assert "-K" in g.mavlink_tx.argv
    assert "-K" in g.tunnel_rx.argv
    assert "-K" in g.tunnel_tx.argv


def test_width_10_narrows_uplink_bandwidth_only():
    cfg = copy.deepcopy(default_config())
    cfg["link"]["width"] = 10
    g = _build(cfg)

    # Uplink tx legs: -B = min(max(width, 10), 20) = 10.
    assert g.mavlink_tx.argv[g.mavlink_tx.argv.index("-B") + 1] == "10"
    assert g.tunnel_tx.argv[g.tunnel_tx.argv.index("-B") + 1] == "10"

    # wfb_rx never takes -B (video/mavlink/tunnel rx all lack the flag).
    assert "-B" not in g.video_rx.argv
    assert "-B" not in g.mavlink_rx.argv
    assert "-B" not in g.tunnel_rx.argv


def test_width_5_uplink_bandwidth_clamps_to_10():
    # 5 MHz has no radiotap bandwidth token; the fork rejects -B 5. It uses
    # 20 MHz modulation, so the uplink clamps to the 10 MHz token (BW_20 on-wire).
    cfg = copy.deepcopy(default_config())
    cfg["link"]["width"] = 5
    g = _build(cfg)

    assert g.mavlink_tx.argv[g.mavlink_tx.argv.index("-B") + 1] == "10"
    assert g.tunnel_tx.argv[g.tunnel_tx.argv.index("-B") + 1] == "10"

    # wfb_rx never takes -B (video/mavlink/tunnel rx all lack the flag).
    assert "-B" not in g.video_rx.argv
    assert "-B" not in g.mavlink_rx.argv
    assert "-B" not in g.tunnel_rx.argv


def test_width_40_clamps_uplink_bandwidth_to_20():
    cfg = copy.deepcopy(default_config())
    cfg["link"]["width"] = 40
    g = _build(cfg)
    assert g.mavlink_tx.argv[g.mavlink_tx.argv.index("-B") + 1] == "20"
    assert g.tunnel_tx.argv[g.tunnel_tx.argv.index("-B") + 1] == "20"


def test_mavlink_peer_listen_scheme_parsed():
    cfg = copy.deepcopy(default_config())
    cfg["wfb"]["mavlink"]["peer"] = "listen://0.0.0.0:14550"
    g = _build(cfg)
    assert g.mav_peer.peer == "listen://0.0.0.0:14550"


def test_mavlink_peer_missing_raises_clean_valueerror():
    import pytest

    cfg = copy.deepcopy(default_config())
    cfg["wfb"]["mavlink"]["peer"] = None
    with pytest.raises(ValueError, match="wfb.mavlink.peer required"):
        _build(cfg)


def test_wfb_raw_nonempty_warns_and_is_ignored(caplog):
    cfg = copy.deepcopy(default_config())
    cfg["wfb"]["raw"] = {"some_stray_key": "value"}
    with caplog.at_level("WARNING", logger="fpvdgs.wfb"):
        g = _build(cfg)
    assert any("wfb.raw ignored by native engine" in r.message for r in caplog.records)
    assert any("some_stray_key" in r.message for r in caplog.records)
    # And it genuinely has no effect on the rendered argv.
    assert "some_stray_key" not in g.video_rx.argv


def test_rand_suffix_drives_all_unix_socket_names():
    g = build_graph(default_config(), WLANS, rand_suffix=lambda: "feedfeed")
    assert g.mavlink_rx.unix_path == "mavlink-rx-feedfeed"
    assert g.mavlink_tx.unix_path == "mavlink-tx-feedfeed"
    assert g.tunnel_rx.unix_path == "tunnel-rx-feedfeed"
    assert g.tunnel_tx.unix_path == "tunnel-tx-feedfeed"


def test_wlans_appended_in_order_for_multiple_cards():
    g = build_graph(default_config(), ["wlan2", "wlan5"], rand_suffix=_sfx)
    assert g.video_rx.argv[-2:] == ["wlan2", "wlan5"]
    assert g.mavlink_tx.argv[-2:] == ["wlan2", "wlan5"]


def test_gs_graph_is_the_documented_type():
    assert isinstance(_build(), GsGraph)


# ---- probe_rx service spec ------------------------------------------------


def _probe_cfg(video_encryption=True):
    cfg = default_config()
    cfg["dynamicLink"]["enabled"] = True
    cfg["dynamicLink"]["probe"] = {"enabled": True}
    cfg["link"]["videoEncryption"] = video_encryption
    return cfg


def test_probe_rx_absent_by_default():
    assert _build().probe_rx is None


def test_probe_rx_absent_when_dl_disabled():
    cfg = _probe_cfg()
    cfg["dynamicLink"]["enabled"] = False
    assert _build(cfg).probe_rx is None


def test_probe_rx_local_argv():
    g = _build(_probe_cfg())
    spec = g.probe_rx
    assert spec is not None and spec.name == "probe_rx" and spec.kind == "rx"
    assert spec.parser == "probe" and spec.unix_path is None
    argv = spec.argv
    assert argv[0].endswith("wfb_rx")
    assert argv[argv.index("-p") + 1] == "50"
    assert argv[argv.index("-u") + 1] == "7000"
    assert argv[argv.index("-K") + 1] == "/etc/gs.key"
    assert argv[-2:] == ["wlan0", "wlan1"]


def test_probe_rx_keyed_even_when_video_plaintext():
    g = _build(_probe_cfg(video_encryption=False))
    assert "-K" not in g.video_rx.argv  # plaintext video (existing behavior)
    assert "-K" in g.probe_rx.argv  # probe stays keyed
