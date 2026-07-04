"""Cluster port allocation + wlan-id encoding for `link.cards` (local +
remote-over-SSH). Pure compute: no I/O, no sockets (`socket.inet_aton` is
used only as a dotted-quad parser, never to open a connection).

Faithfully ports the allocation semantics of wfb-ng's
`parse_cluster_services` (wfb_ng/cluster.py): a single server-port counter
shared across services (one GS aggregator UDP port per service), and a
per-node injector-port counter (one `wfb_tx -I` base port per (node,
service), advanced by that node's card count per service).
"""

from __future__ import annotations

import itertools
import socket
import struct
from dataclasses import dataclass, field

from .. import radio
from .cards import Card

SERVICE_ORDER = ("video", "mavlink", "tunnel")

DEFAULT_STREAMS = {
    "video": {"rx": 0, "tx": None},
    "mavlink": {"rx": 16, "tx": 144},
    "tunnel": {"rx": 32, "tx": 160},
}


def node_key(card: Card) -> str:
    """The cluster node a card belongs to: its host, or the implicit
    "127.0.0.1" local node (wfb-ng's localhost-node pattern)."""
    return card.host or "127.0.0.1"


def node_ipv4_int(host: str) -> int:
    return struct.unpack("!L", socket.inet_aton(host))[0]


def cluster_wlan_id(host: str, wlan_idx: int) -> int:
    return (node_ipv4_int(host) << 24) | wlan_idx


@dataclass
class ClusterPlan:
    nodes: dict[str, list[Card]] = field(default_factory=dict)
    server_port: dict[str, int] = field(default_factory=dict)
    injector_base: dict[tuple[str, str], int] = field(default_factory=dict)
    peers: dict[str, list[str]] = field(default_factory=dict)
    rx_only_wlan_ids: frozenset[int] = frozenset()


def plan_cluster(
    cards: list[Card],
    base_port_server: int = 10000,
    base_port_node: int = 11000,
) -> ClusterPlan:
    nodes: dict[str, list[Card]] = {}
    for card in cards:
        nodes.setdefault(node_key(card), []).append(card)

    server_port_alloc = itertools.count(base_port_server)
    server_port = {service: next(server_port_alloc) for service in SERVICE_ORDER}

    node_allocators: dict[str, itertools.count] = {}

    def get_allocator(node: str) -> itertools.count:
        alloc = node_allocators.get(node)
        if alloc is None:
            alloc = itertools.count(base_port_node)
            node_allocators[node] = alloc
        return alloc

    injector_base: dict[tuple[str, str], int] = {}
    peers: dict[str, list[str]] = {service: [] for service in SERVICE_ORDER}

    for service in SERVICE_ORDER:
        for node in sorted(nodes):
            node_cards = nodes[node]
            alloc = get_allocator(node)
            ports = [next(alloc) for _ in node_cards]
            injector_base[(node, service)] = min(ports)
            peers[service].append(f"{node}:{','.join(str(p) for p in ports)}")

    rx_only_wlan_ids = frozenset(
        cluster_wlan_id(node_key(card), idx)
        for node, node_cards in nodes.items()
        for idx, card in enumerate(node_cards)
        if card.is_rx_only
    )

    return ClusterPlan(
        nodes=nodes,
        server_port=server_port,
        injector_base=injector_base,
        peers=peers,
        rx_only_wlan_ids=rx_only_wlan_ids,
    )


def render_node_script(
    node: str,
    cards: list[Card],
    plan: ClusterPlan,
    *,
    link: dict,
    link_id: int,
    server_address: str,
    ssh_mode: bool = True,
    streams: dict | None = None,
) -> str:
    """Render the POSIX-sh bootstrap script pushed to `node` over SSH (or run
    locally): tunes each of `cards`' wlans, then spawns the wfb_rx/wfb_tx
    children for every service this node participates in, sourced from
    `plan` (port allocation) and `link` (channel/width/region).

    A port of wfb-ng's `gen_cluster_scripts`/`script_template`, sh-adapted
    (no bash on OpenWrt/BusyBox nodes or the GS bench box): `set -em`
    (no `-b`), and a portable fail-fast poll loop in place of `wait -n`
    (dash lacks it).
    """
    if streams is None:
        streams = DEFAULT_STREAMS

    width = link.get("width", 20)
    region = link.get("region")
    channel = link.get("channel")
    wlans = [card.iface for card in cards]
    wlans_str = " ".join(wlans)

    lines: list[str] = [
        "#!/bin/sh",
        "set -em",
        "",
        "export LC_ALL=C",
        "",
        'PIDS=""',
        "",
        "_cleanup()",
        "{",
        "  plist=$(jobs -p)",
        '  if [ -n "$plist" ]',
        "  then",
        "      kill -TERM $plist || true",
        "  fi",
        "  exit 1",
        "}",
        "",
        "trap _cleanup EXIT",
        "",
    ]

    lines.append(f"iw reg set {region}")
    lines.append("")

    for card in cards:
        wlan = card.iface
        lines.append(f"# init {wlan}")
        lines.append(
            f"if which nmcli > /dev/null && " f"! nmcli device show {wlan} | grep -q '(unmanaged)'"
        )
        lines.append("then")
        lines.append(f"  nmcli device set {wlan} managed no")
        lines.append("  sleep 1")
        lines.append("fi")
        lines.append("")
        lines.append(f"ip link set {wlan} down")
        lines.append(f"iw dev {wlan} set monitor otherbss")
        lines.append(f"ip link set {wlan} up")
        if channel is not None:
            lines.append(" ".join(radio.iw_args(wlan, channel, width)))
        if card.txpower_dbm not in (None, "off"):
            mbm = round(float(card.txpower_dbm) * 100)
            lines.append(f"iw dev {wlan} set txpower fixed {mbm}")
        lines.append("")

    for service in SERVICE_ORDER:
        svc_streams = streams.get(service, {})
        rx_id = svc_streams.get("rx")
        tx_id = svc_streams.get("tx")
        if rx_id is None and tx_id is None:
            continue
        lines.append(f"# {service}")
        if rx_id is not None:
            port = plan.server_port[service]
            lines.append(
                f"wfb_rx -f -c {server_address} -u {port} -p {rx_id} "
                f'-i {link_id} -R 2097152 {wlans_str} & PIDS="$PIDS $!"'
            )
        if tx_id is not None:
            injector = plan.injector_base[(node, service)]
            lines.append(f'wfb_tx -I {injector} -R 2097152 {wlans_str} & PIDS="$PIDS $!"')
        lines.append("")

    if ssh_mode:
        lines.append("# Will fail in case of connection loss")
        lines.append('(sleep 1; exec cat > /dev/null) & PIDS="$PIDS $!"')
        lines.append("")

    lines.append('echo "WFB-ng init done"')
    lines.append("")
    lines.append("while :; do")
    lines.append("  for p in $PIDS; do")
    lines.append("    kill -0 $p 2>/dev/null || exit 1")
    lines.append("  done")
    lines.append("  sleep 1")
    lines.append("done")
    lines.append("")

    return "\n".join(lines)
