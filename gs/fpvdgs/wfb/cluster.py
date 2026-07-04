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

from .cards import Card

SERVICE_ORDER = ("video", "mavlink", "tunnel")


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
