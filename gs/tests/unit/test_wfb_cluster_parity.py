"""Cluster-wiring parity golden test: `fpvdgs.wfb.cluster`/`fpvdgs.wfb.graph`'s
remote-card plumbing vs REAL wfb-ng `parse_cluster_services` +
`gen_cluster_scripts`.

This is the KEYSTONE regression net for Phase 2 (remote-over-SSH cards): it
renders the same fpvd effective config as `test_wfb_argv_parity.py`, feeds it
to the actual `wfb_ng` package (the fpvd fork, `swfec` branch), then
*directly* seeds `wfb_ng.conf.settings.cluster.nodes` with a two-node
topology (a local node with two cards, a remote rx-only node with one) and
compares:

1. Port/peer allocation: our `plan_cluster`'s per-service server (aggregator)
   ports, per-(node, service) injector bases, and per-service peer strings
   against wfb-ng's own computed `udp_port_auto` / `udp_peers_auto` /
   `tx_port_base` / `rx_fwd`.
2. The remote node's bootstrap script: our `render_node_script`'s
   `wfb_rx -f` / `wfb_tx -I` / `iw` / `ip link` lines against wfb-ng's
   `gen_cluster_scripts` output for that node, compared as LINE SETS (our
   scaffold is POSIX-sh, wfb-ng's is jinja-rendered bash -- deliberately
   different outside the payload lines).
3. The local node's forwarder/injector argvs (spawned directly by fpvd,
   never shipped through SSH) against the same payload lines extracted from
   wfb-ng's "127.0.0.1" node script (wfb-ng scripts every node uniformly,
   including localhost).
4. The GS-side aggregator (`wfb_rx -a`) / distributor (`wfb_tx -d`) argvs
   `build_graph_remote` renders, byte-compared against the cluster variants
   of `services.py`'s format strings (extending `test_wfb_argv_parity.py`'s
   transcription with the `-a %d` / `-d` + peers cluster splice).
5. The `cluster_wlan_id` rx-only-wlan-id formula, checked directly against
   the literal expression in `wfb_ng/server.py`'s `init()` (~line 164:
   `(struct.unpack("!L", socket.inet_aton(node))[0] << 24) | idx`) -- that
   code lives in `server.py`, not `cluster.py`, and (like `services.py`'s
   `init_*` functions) is a Twisted-reactor entry point, not synchronously
   callable, so this checks the formula in isolation rather than calling it.

Run (needs the wfb-ng checkout; deps `twisted`/`msgpack`/`pyserial`/
`pyroute2`/`jinja2` are test-time only, never shipped by fpvdgs):

    WFB_NG_SRC=~/Projects/poc/wfb-ng .venv/bin/python -m pytest tests/unit/test_wfb_cluster_parity.py -v

Without `WFB_NG_SRC` set, this whole file SKIPS (verified as part of the
full `pytest tests/ -q` run staying green).

What this proves vs what it assumes
------------------------------------
Proves: for a mixed local+remote two-node cluster on the "gs" profile, our
port/peer allocation is arithmetically identical to wfb-ng's real allocator
(same shared per-service server-port counter, same per-node injector-port
counter advanced in the same service order); the remote node's payload
command lines (`wfb_rx -f`, `wfb_tx -I`, `iw`, `ip link`) match wfb-ng's
rendered script line-for-line; the local node's forwarder/injector argvs
match what wfb-ng would run on "127.0.0.1" (modulo the absolute-binary-path
vs bare-binary-name difference, since ours are spawned directly without a
shell `$PATH` lookup); the GS-side aggregator/distributor argvs are
byte-identical to wfb-ng's cluster-mode `services.py` rendering; and the
`cluster_wlan_id` formula matches `server.py`'s literal rx-only-id
expression.

Assumes / does not prove -- and documents three INTENTIONAL fpvd
divergences from upstream, none of which this fixture (one local + one
remote node) can distinguish from a bug, so they're recorded here instead:

  - **No SSH-to-self.** wfb-ng's cluster mode is fully uniform: every node
    in `cluster.nodes`, including "127.0.0.1", gets a generated script and
    (in `--cluster ssh` mode) a real SSH session opened to it
    (`server.py::init`'s `SSHClientProtocol` loop has no localhost
    special-case). fpvd's `build_graph_remote` instead special-cases
    `LOCAL_NODE = "127.0.0.1"`: it never emits a script or opens a session
    for it, spawning `local_forwarders`/`local_injectors` as ordinary child
    processes. This is a deliberate optimization (skips a pointless SSH
    round-trip to the GS's own loopback) validated here only by comparing
    argv *content* against wfb-ng's would-be "127.0.0.1" script, not by
    exercising any session-management code path.
  - **Single server address for all remote nodes.** wfb-ng resolves
    `server_address` per node (`search_attr('server_address', cluster.nodes[node], cluster.__dict__)`),
    so different remote nodes can be told to reach the GS at different
    addresses (multi-homed GS). fpvd's `engine.py` derives ONE
    `server_address` (from the first remote card's route, or
    `link.serverAddress` override) and passes it to every remote node's
    `render_node_script` call. With exactly one remote node in this
    fixture the two schemes are indistinguishable; a future multi-remote-
    node, multi-homed-GS setup would need this revisited.
  - **No qdisc/fwmark shaping on cluster injectors.** wfb-ng's injector
    line conditionally appends `-Q -P <fwmark>` when the service's
    `use_qdisc` is set. `render_node_script`'s injector line never emits
    this. Not exercised here because fpvd's default "gs" profile config
    never sets `use_qdisc` for any service (so wfb-ng's own reference line
    also omits it) and fpvd exposes no config knob for it -- a real gap
    only if that knob is ever added.

Also, as in `test_wfb_argv_parity.py`: `wfb_ng/conf/local.cfg`'s
`path.bin_dir`/`path.conf_dir` overrides are neutralized post-import so the
reference matches a real deployed install (`master.cfg`'s `/usr/bin`/`/etc`).
"""

from __future__ import annotations

import os
import socket
import struct
import sys

import pytest

from fpvdgs.config import deep_merge
from fpvdgs.config_defaults import default_config
from fpvdgs.render import render_cfg, write_cfg
from fpvdgs.wfb.cards import Card
from fpvdgs.wfb.cluster import DEFAULT_STREAMS, SERVICE_ORDER, cluster_wlan_id, plan_cluster
from fpvdgs.wfb.graph import build_graph_remote

WFB_NG_SRC = os.environ.get("WFB_NG_SRC")
pytestmark = pytest.mark.skipif(
    not WFB_NG_SRC, reason="set WFB_NG_SRC to the wfb-ng checkout to run parity"
)

SFX = "aaaaaaaa"  # 8 hex-shaped chars, matching os.urandom(4).hex()'s length

LOCAL_NODE = "127.0.0.1"
REMOTE_NODE = "192.168.1.10"
GS_SERVER_ADDRESS = "10.18.0.1"


def _load_wfb_ng_cluster(cfg_path: str):
    """Like `test_wfb_argv_parity._load_wfb_ng`, but also imports
    `wfb_ng.cluster` (pulls in `jinja2`, a separate dev-only dep from the
    `services`-only import path)."""
    os.environ["WIFIBROADCAST_CFG"] = cfg_path
    if WFB_NG_SRC not in sys.path:
        sys.path.insert(0, WFB_NG_SRC)
    for name in list(sys.modules):
        if name == "wfb_ng" or name.startswith("wfb_ng."):
            del sys.modules[name]

    import wfb_ng.cluster as cluster_mod
    import wfb_ng.services as services
    from wfb_ng.conf import settings

    settings.path.bin_dir = "/usr/bin"
    settings.path.conf_dir = "/etc"
    return services, cluster_mod, settings


def _render(tmp_path, overlay: dict):
    eff = deep_merge(default_config(), overlay)
    cfg_path = tmp_path / "wifibroadcast.cfg"
    write_cfg(str(cfg_path), render_cfg(eff))
    return eff, str(cfg_path)


def _seed_cluster_settings(settings) -> None:
    settings.cluster.nodes = {
        LOCAL_NODE: {"wlans": ["wlan1", "wlan2"], "server_address": LOCAL_NODE},
        REMOTE_NODE: {"wlans": ["wlan0"], "wifi_txpower": "off"},
    }
    settings.cluster.base_port_server = 10000
    settings.cluster.base_port_node = 11000
    settings.cluster.ssh_user = "root"
    settings.cluster.ssh_port = 22
    settings.cluster.ssh_key = None
    settings.cluster.custom_init_script = None
    settings.cluster.server_address = GS_SERVER_ADDRESS


def _normalize_script_lines(text: str) -> set[str]:
    """Strip blank/comment lines, drop the trailing backgrounding scaffold
    (bash `&` vs our POSIX `& PIDS="$PIDS $!"`), collapse whitespace, and
    keep only the payload lines (`wfb_rx -f`, `wfb_tx -I`, `iw `, `ip
    link`) -- the rest of each script (trap/cleanup boilerplate, the
    `nmcli`/`sleep`/`fi` card-init guard, the ssh-mode watchdog line) is
    scaffold that intentionally differs between wfb-ng's bash and fpvd's
    POSIX sh."""
    lines: set[str] = set()
    for raw in text.splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if " &" in line:
            line = line.split(" &", 1)[0].strip()
        line = " ".join(line.split())
        if line.startswith(("wfb_rx -f", "wfb_tx -I", "iw ", "ip link")):
            lines.add(line)
    return lines


def _strip_bin_prefix(argv: list[str]) -> list[str]:
    """Our local forwarders/injectors use the absolute `WFB_BIN_DIR` path
    (no shell involved, so no `$PATH` to rely on); wfb-ng's script invokes
    the bare binary name via `$PATH`. Compare the payload, not the path."""
    return [os.path.basename(argv[0]), *argv[1:]]


# ---- GS-side cluster aggregator/distributor argv construction ----------
# Cluster variants of `test_wfb_argv_parity.py`'s transcription of
# wfb_ng/services.py's init_udp_direct_rx / init_mavlink / init_tunnel
# format strings: `-a %d` spliced in right after the binary for an
# aggregator (rx), `-d` for a distributor (tx), with the trailing
# positional replaced (aggregator: dropped; distributor: the peer list).
# Note: %(cluster)s's non-cluster branch and the %(mirror)s/%(force_vht)s/
# %(qdisc)s empty-string cases are intentionally omitted/hardcoded True-ish
# since fpvd's rendered cfg never sets mirror/force_vht/use_qdisc.


def _rx_argv_cluster(settings, key_arg_fn, cfg, agg_port, stream, conn_str, link_id, *, tap=False):
    tap_port = int(getattr(cfg, "dynlink_tap_port", 0) or 0)
    cmd = (
        "%(cmd)s -a %(agg_port)d -p %(stream)d %(conn_str)s %(key_arg)s -R %(rcv_buf_size)d "
        "-s %(snd_buf_size)d -l %(log_interval)d -i %(link_id)d%(tap)s"
        % dict(
            cmd=os.path.join(settings.path.bin_dir, "wfb_rx"),
            agg_port=agg_port,
            stream=stream,
            conn_str=conn_str,
            key_arg=key_arg_fn(cfg),
            rcv_buf_size=settings.common.tx_rcv_buf_size,
            snd_buf_size=settings.common.rx_snd_buf_size,
            log_interval=settings.common.log_interval,
            link_id=link_id,
            tap=" -D %d" % tap_port if (tap and tap_port) else "",
        )
    ).split()
    return cmd


def _tx_argv_cluster(settings, key_arg_fn, cfg, stream, unix_path, peers, link_id):
    cmd = (
        "%(cmd)s -d -f %(frame_type)s -p %(stream)d -U %(unix_socket)s %(key_arg)s "
        "-B %(bw)d -G %(gi)s -S %(stbc)d -L %(ldpc)d -M %(mcs)d"
        "%(mirror)s%(force_vht)s%(qdisc)s "
        "-k %(fec_k)d -n %(fec_n)d -T %(fec_timeout)d -F %(fec_delay)d -i %(link_id)d "
        "-R %(rcv_buf_size)d -s %(snd_buf_size)d -l %(log_interval)d -C %(control_port)d"
        % dict(
            cmd=os.path.join(settings.path.bin_dir, "wfb_tx"),
            frame_type=cfg.frame_type,
            stream=stream,
            unix_socket=unix_path,
            control_port=cfg.control_port,
            key_arg=key_arg_fn(cfg),
            bw=cfg.bandwidth,
            force_vht=" -V" if cfg.force_vht else "",
            qdisc=" -Q -P %d" % (cfg.fwmark,) if cfg.use_qdisc else "",
            gi="short" if cfg.short_gi else "long",
            stbc=cfg.stbc,
            ldpc=cfg.ldpc,
            mcs=cfg.mcs_index,
            mirror=" -m" if cfg.mirror else "",
            fec_k=cfg.fec_k,
            fec_n=cfg.fec_n,
            fec_timeout=cfg.fec_timeout,
            fec_delay=cfg.fec_delay,
            link_id=link_id,
            log_interval=settings.common.log_interval,
            rcv_buf_size=settings.common.tx_rcv_buf_size,
            snd_buf_size=settings.common.rx_snd_buf_size,
        )
    ).split() + list(peers)
    return cmd


def test_cluster_wlan_id_matches_server_formula():
    """Pure-formula check against `wfb_ng/server.py::init()`'s literal
    rx-only-wlan-id expression (~line 164-171), which lives outside
    `cluster.py` and is not synchronously callable (it's inline in a
    Twisted `@defer.inlineCallbacks` entry point)."""
    for host, idx in [("192.168.1.10", 0), ("127.0.0.1", 5), ("10.0.0.1", 3)]:
        node_ipv4_addr = struct.unpack("!L", socket.inet_aton(host))[0]
        assert cluster_wlan_id(host, idx) == ((node_ipv4_addr << 24) | idx)


def test_cluster_wiring_matches_real_wfb_ng(tmp_path):
    eff, cfg_path = _render(tmp_path, {"link": {"channel": 132, "width": 20}})

    try:
        services, cluster_mod, settings = _load_wfb_ng_cluster(cfg_path)
    except ImportError as e:
        pytest.xfail(f"wfb_ng import failed (missing dev-only dep, e.g. jinja2?): {e}")

    _seed_cluster_settings(settings)

    services_list, cluster_nodes = cluster_mod.parse_cluster_services(["gs"])
    node_scripts_ref = cluster_mod.gen_cluster_scripts(cluster_nodes, ssh_mode=True)

    assert [p for p, _ in services_list] == ["gs"]
    by_name = {name: cfg for name, _stype, cfg in services_list[0][1]}
    lid_ref = services.hash_link_domain(settings.gs.link_domain)

    # Sanity: our hardcoded stream-id table matches the real per-service cfg.
    assert DEFAULT_STREAMS["video"] == {
        "rx": by_name["video"].stream_rx,
        "tx": by_name["video"].stream_tx,
    }
    assert DEFAULT_STREAMS["mavlink"] == {
        "rx": by_name["mavlink"].stream_rx,
        "tx": by_name["mavlink"].stream_tx,
    }
    assert DEFAULT_STREAMS["tunnel"] == {
        "rx": by_name["tunnel"].stream_rx,
        "tx": by_name["tunnel"].stream_tx,
    }

    # -- our side: build the matching Card/plan/graph -----------------------
    cards = [
        Card(host=None, iface="wlan1"),
        Card(host=None, iface="wlan2"),
        Card(host=REMOTE_NODE, iface="wlan0", txpower_dbm="off"),
    ]
    plan = plan_cluster(cards, base_port_server=10000, base_port_node=11000)

    assert set(plan.nodes) == {LOCAL_NODE, REMOTE_NODE}
    assert [c.iface for c in plan.nodes[LOCAL_NODE]] == ["wlan1", "wlan2"]
    assert [c.iface for c in plan.nodes[REMOTE_NODE]] == ["wlan0"]

    # -- 1. ports / peers / injector bases -----------------------------------
    for service in SERVICE_ORDER:
        ref_cfg = by_name[service]
        assert plan.server_port[service] == ref_cfg.udp_port_auto
        assert plan.peers[service] == ref_cfg.udp_peers_auto

        for node in plan.nodes:
            ref_attrs = cluster_nodes[node][f"gs_{service}"]
            assert plan.injector_base[(node, service)] == ref_attrs["tx_port_base"]

            expected_addr, expected_port = ref_attrs["rx_fwd"]
            assert expected_port == ref_cfg.udp_port_auto
            our_addr = LOCAL_NODE if node == LOCAL_NODE else GS_SERVER_ADDRESS
            assert our_addr == expected_addr
            assert plan.server_port[service] == expected_port

    # -- rx-only wlan id ------------------------------------------------------
    remote_ipv4 = struct.unpack("!L", socket.inet_aton(REMOTE_NODE))[0]
    assert plan.rx_only_wlan_ids == frozenset({(remote_ipv4 << 24) | 0})

    # -- build the graph (GS-side argvs + local fwd/inj + remote node script)
    g = build_graph_remote(eff, cards, plan, GS_SERVER_ADDRESS, rand_suffix=lambda: SFX)

    # -- 2. remote node script: payload line sets ----------------------------
    ref_remote_lines = _normalize_script_lines(node_scripts_ref[REMOTE_NODE])
    our_remote_lines = _normalize_script_lines(g.node_scripts[REMOTE_NODE])
    assert ref_remote_lines, "reference remote-node script produced no payload lines"
    assert our_remote_lines == ref_remote_lines

    # -- 3. local node forwarder/injector argvs vs wfb-ng's "127.0.0.1" script
    ref_local_lines = _normalize_script_lines(node_scripts_ref[LOCAL_NODE])
    ref_local_cmds = {
        tuple(line.split())
        for line in ref_local_lines
        if line.startswith(("wfb_rx -f", "wfb_tx -I"))
    }
    our_local_cmds = {
        tuple(_strip_bin_prefix(spec.argv)) for spec in g.local_forwarders + g.local_injectors
    }
    assert ref_local_cmds, "reference local-node script produced no wfb_rx/wfb_tx lines"
    assert our_local_cmds == ref_local_cmds

    # -- 4. GS-side aggregator/distributor argvs -----------------------------
    video_cfg, mav_cfg, tun_cfg = by_name["video"], by_name["mavlink"], by_name["tunnel"]

    ref_video = _rx_argv_cluster(
        settings,
        services.key_arg,
        video_cfg,
        video_cfg.udp_port_auto,
        video_cfg.stream_rx,
        "-c 127.0.0.1 -u 5600",
        lid_ref,
        tap=True,
    )
    ref_mav_rx = _rx_argv_cluster(
        settings,
        services.key_arg,
        mav_cfg,
        mav_cfg.udp_port_auto,
        mav_cfg.stream_rx,
        f"-U mavlink-rx-{SFX}",
        lid_ref,
    )
    ref_mav_tx = _tx_argv_cluster(
        settings,
        services.key_arg,
        mav_cfg,
        mav_cfg.stream_tx,
        f"mavlink-tx-{SFX}",
        mav_cfg.udp_peers_auto,
        lid_ref,
    )
    ref_tun_rx = _rx_argv_cluster(
        settings,
        services.key_arg,
        tun_cfg,
        tun_cfg.udp_port_auto,
        tun_cfg.stream_rx,
        f"-U tunnel-rx-{SFX}",
        lid_ref,
    )
    ref_tun_tx = _tx_argv_cluster(
        settings,
        services.key_arg,
        tun_cfg,
        tun_cfg.stream_tx,
        f"tunnel-tx-{SFX}",
        tun_cfg.udp_peers_auto,
        lid_ref,
    )

    assert g.video_rx.argv == ref_video
    assert g.mavlink_rx.argv == ref_mav_rx
    assert g.mavlink_tx.argv == ref_mav_tx
    assert g.tunnel_rx.argv == ref_tun_rx
    assert g.tunnel_tx.argv == ref_tun_tx
