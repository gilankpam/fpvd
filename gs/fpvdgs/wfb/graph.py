"""Pure config -> argv service-graph builder for the GS wfb data plane.

Byte-for-byte port of what wfb-ng's `services.py` renders for the "gs"
profile, scoped to fpvd's rendered overrides (`log_interval=100`, key
`/etc/gs.key`, the dynlink tap flag, and the optional plaintext video
stream). Pure string building — no process spawning, no I/O — so it is
callable and testable synchronously, with no event loop. A later task adds
an automated parity test against wfb-ng itself; the golden tests here
(`gs/tests/unit/test_wfb_graph.py`) are the first line of defense in the
meantime, since these argv strings are flight-critical.

Four legs run per flight, all keyed by the same `link_id()` nonce and
sharing the wlan card list:

- video (rx only): wfb_rx forwards decoded video straight to pixelpilot
  over loopback UDP (`-c 127.0.0.1 -u 5600`), so it has no unix socket.
  Its dynlink tap (`-D <port>`) and, independently, its `-K` (video
  encryption) are the only per-leg-optional flags in this module.
- mavlink (rx + tx): bridges wfb_rx/wfb_tx abstract-namespace unix
  datagram sockets to the local mavlink UDP peer (`mavproxy.py`).
- tunnel (rx + tx): bridges wfb_rx/wfb_tx unix sockets to the tun device
  carrying the drone's dynamic-link return channel (`tunnel.py`).

`wfb_rx` never takes a `-B` (bandwidth) flag — only `wfb_tx` needs it, to
pick its rate-table row. The two uplink tx legs (mavlink, tunnel) render
`-B min(max(link.width, 10), 20)`; the video rx leg renders no `-B` at all.
A 5 MHz channel has no valid radiotap bandwidth token, so it clamps up to
the 10 MHz token — both 10 and 20 map to `BW_20` on-wire in the wfb-ng
fork, so this is identical to width-10 behavior.

`build_graph_remote` adds the cluster wiring used whenever any `link.cards`
entry is remote (over SSH): every card becomes a `wfb_rx -f` forwarder
feeding a per-service `wfb_rx -a` aggregator on the GS, and `wfb_tx -d`
distributors fan encrypted frames out to per-card `wfb_tx -I` injectors.
The GS-side aggregator/distributor legs are the same argv shapes as above
with a cluster flag (`-a <port>` / `-d`) spliced in right after the binary
and the trailing `wlans` positional replaced (aggregator: dropped
entirely; distributor: replaced by `plan.peers[service]`).

Stats-interface note (verified against the wfb-ng fork's `rx.cpp`/`tx.cpp`,
2026-07-04): only the AGGREGATOR (`wfb_rx -a`, `RXMode::AGGREGATOR`) and
DISTRIBUTOR (`wfb_tx -d`, backed by `RemoteTransmitter`) roles ever call a
non-empty `dump_stats()` and emit `IPC_MSG` lines (`RX_ANT`/`PKT`/`TX_ANT`).
`Forwarder::dump_stats` (the `-f` role) is a literal no-op override, and
`packet_injector` (the `-I` role's loop) never calls `IPC_MSG` at all — only
`WFB_ERR` diagnostics to stderr. So local forwarders/injectors carry
`parser=None`: they have no stats interface to parse, and must not be
wired into StatsHub (there is nothing to double-count, but also nothing to
feed it).
"""

from __future__ import annotations

import hashlib
import logging
from collections.abc import Callable
from dataclasses import dataclass, field

from .cards import Card, local_ifaces
from .cluster import ClusterPlan, render_node_script, streams_for
from .mavproxy import MavlinkConfig
from .tunnel import TunnelConfig

log = logging.getLogger("fpvdgs.wfb")

WFB_BIN_DIR = "/usr/bin"
GS_KEY = "/etc/gs.key"
LOG_INTERVAL = 100
BUF_R = 2097152
BUF_S = 2097152
VIDEO_UDP_PORT = 5600  # wfb-ng's gs_video default; independent of pixelpilot.rtpPort
PROBE_RADIO_PORT = 50  # MUST match the drone's kProbeRadioPort
PROBE_SINK_PORT = 7000  # throwaway -u destination; probe payload is never consumed

# base profile: short_gi=False, stbc=1, ldpc=1, mcs_index=1
GI = "long"
STBC = 1
LDPC = 1
MCS_INDEX = 1

# radio_base FEC: k=1, n=2, no fixed timeout/frame-size override
FEC_K = 1
FEC_N = 2
FEC_T = 0
FEC_F = 0


def link_id(link_domain: str = "default") -> int:
    """Port of wfb-ng's link ID derivation: the first 3 bytes of
    sha1(link_domain), big-endian. Passed to every leg's `-i` flag so all
    five wfb_rx/wfb_tx processes agree on the same radio nonce."""
    return int.from_bytes(hashlib.sha1(link_domain.encode()).digest()[:3], "big")


@dataclass
class ServiceSpec:
    name: str
    kind: str  # "rx" | "tx"
    argv: list[str]
    parser: (
        str | None
    )  # "rx" | "tx" | None (no IPC_MSG stats interface — see graph.py module docstring)
    unix_path: str | None


@dataclass
class GsGraph:
    video_rx: ServiceSpec
    mavlink_rx: ServiceSpec
    mavlink_tx: ServiceSpec
    tunnel_rx: ServiceSpec
    tunnel_tx: ServiceSpec
    mav_rx_sock: str
    mav_peer: MavlinkConfig
    tun_rx_sock: str
    tun_cfg: TunnelConfig
    # Remote wiring only (build_graph_remote); empty/default for the
    # all-local build_graph path.
    local_forwarders: list[ServiceSpec] = field(default_factory=list)
    local_injectors: list[ServiceSpec] = field(default_factory=list)
    node_scripts: dict[str, str] = field(default_factory=dict)
    probe_rx: ServiceSpec | None = None


def _rx_common_tail(lid: int) -> list[str]:
    return ["-R", str(BUF_R), "-s", str(BUF_S), "-l", str(LOG_INTERVAL), "-i", str(lid)]


def _rx_argv(
    *,
    head: list[str],
    key: list[str],
    lid: int,
    tail: list[str],
    extra: list[str] = (),
    cluster_flag: list[str] = (),
) -> list[str]:
    """Shared `wfb_rx` argv builder: binary, optional cluster flag (`-a
    <port>`, inserted right after the binary), the leg-specific `head`
    (radio_port + destination), `-K` key, the common `-R/-s/-l/-i` tail,
    any leg-specific `extra` flags (the video leg's `-D` tap), then the
    trailing positional `tail` (wlans for a local leg, empty for a cluster
    aggregator — it takes its input over UDP from forwarders instead)."""
    return (
        [f"{WFB_BIN_DIR}/wfb_rx"]
        + list(cluster_flag)
        + list(head)
        + list(key)
        + _rx_common_tail(lid)
        + list(extra)
        + list(tail)
    )


def _tx_middle(*, port: int, unix_path: str, key: list[str], bandwidth: int, lid: int) -> list[str]:
    return (
        ["-f", "data", "-p", str(port), "-U", unix_path]
        + list(key)
        + [
            "-B",
            str(bandwidth),
            "-G",
            GI,
            "-S",
            str(STBC),
            "-L",
            str(LDPC),
            "-M",
            str(MCS_INDEX),
            "-k",
            str(FEC_K),
            "-n",
            str(FEC_N),
            "-T",
            str(FEC_T),
            "-F",
            str(FEC_F),
            "-i",
            str(lid),
            "-R",
            str(BUF_R),
            "-s",
            str(BUF_S),
            "-l",
            str(LOG_INTERVAL),
            "-C",
            "0",
        ]
    )


def _tx_argv(
    *,
    port: int,
    unix_path: str,
    key: list[str],
    bandwidth: int,
    lid: int,
    tail: list[str],
    cluster_flag: list[str] = (),
) -> list[str]:
    """Shared `wfb_tx` argv builder: binary, optional cluster flag (`-d`,
    inserted right after the binary), then the radio-flag block shared by
    every tx leg (local or distributor), then the trailing positional
    `tail` (wlans for a local leg, `plan.peers[service]` for a cluster
    distributor)."""
    return (
        [f"{WFB_BIN_DIR}/wfb_tx"]
        + list(cluster_flag)
        + _tx_middle(port=port, unix_path=unix_path, key=key, bandwidth=bandwidth, lid=lid)
        + list(tail)
    )


def build_graph(effective: dict, wlans: list[str], *, rand_suffix: Callable[[], str]) -> GsGraph:
    link = effective.get("link", {}) or {}
    wfb = effective.get("wfb", {}) or {}
    dl = effective.get("dynamicLink", {}) or {}
    tap = dl.get("tap", {}) or {}

    raw = wfb.get("raw") or {}
    if raw:
        log.warning("wfb.raw ignored by native engine: %s", sorted(raw))

    lid = link_id()
    wlans = list(wlans)
    width = link.get("width", 20)
    uplink_bw = min(max(width, 10), 20)
    key_flag = ["-K", GS_KEY]
    video_key_flag = key_flag if link.get("videoEncryption", True) else []

    tap_flag = ["-D", str(tap.get("port", 8110))] if tap.get("enabled", True) else []

    # -- video (rx only) --------------------------------------------------
    video_argv = _rx_argv(
        head=["-p", "0", "-c", "127.0.0.1", "-u", str(VIDEO_UDP_PORT)],
        key=video_key_flag,
        lid=lid,
        tail=wlans,
        extra=tap_flag,
    )

    video_rx = ServiceSpec(name="video_rx", kind="rx", argv=video_argv, parser="rx", unix_path=None)

    # -- mavlink (rx + tx) --------------------------------------------------
    mav_rx_sock = f"mavlink-rx-{rand_suffix()}"
    mavlink_rx = ServiceSpec(
        name="mavlink_rx",
        kind="rx",
        argv=_rx_argv(head=["-p", "16", "-U", mav_rx_sock], key=key_flag, lid=lid, tail=wlans),
        parser="rx",
        unix_path=mav_rx_sock,
    )

    mav_tx_sock = f"mavlink-tx-{rand_suffix()}"
    mavlink_tx = ServiceSpec(
        name="mavlink_tx",
        kind="tx",
        argv=_tx_argv(
            port=144,
            unix_path=mav_tx_sock,
            key=key_flag,
            bandwidth=uplink_bw,
            lid=lid,
            tail=wlans,
        ),
        parser="tx",
        unix_path=mav_tx_sock,
    )

    # -- tunnel (rx + tx) --------------------------------------------------
    tun_rx_sock = f"tunnel-rx-{rand_suffix()}"
    tunnel_rx = ServiceSpec(
        name="tunnel_rx",
        kind="rx",
        argv=_rx_argv(head=["-p", "32", "-U", tun_rx_sock], key=key_flag, lid=lid, tail=wlans),
        parser="rx",
        unix_path=tun_rx_sock,
    )

    tun_tx_sock = f"tunnel-tx-{rand_suffix()}"
    tunnel_tx = ServiceSpec(
        name="tunnel_tx",
        kind="tx",
        argv=_tx_argv(
            port=160,
            unix_path=tun_tx_sock,
            key=key_flag,
            bandwidth=uplink_bw,
            lid=lid,
            tail=wlans,
        ),
        parser="tx",
        unix_path=tun_tx_sock,
    )

    mav_peer_url = (wfb.get("mavlink", {}) or {}).get("peer")
    if not mav_peer_url:
        raise ValueError("wfb.mavlink.peer required")
    mav_peer = MavlinkConfig(peer=mav_peer_url)
    tun_cfg = TunnelConfig(ifname="gs-wfb", ifaddr="10.5.0.1/24", mtu=1445, agg_timeout=0.005)

    # -- probe (rx only, observe-only; 2026-07-06 spec Part B) --------------
    probe_rx = None
    if bool(dl.get("enabled", False)) and bool((dl.get("probe") or {}).get("enabled", False)):
        probe_rx = ServiceSpec(
            name="probe_rx",
            kind="rx",
            argv=_rx_argv(
                head=["-p", str(PROBE_RADIO_PORT), "-c", "127.0.0.1", "-u", str(PROBE_SINK_PORT)],
                key=key_flag,  # ALWAYS keyed, independent of videoEncryption
                lid=lid,
                tail=wlans,
            ),
            parser="probe",  # in-process ProbeFeed sink, never the StatsHub
            unix_path=None,
        )

    return GsGraph(
        video_rx=video_rx,
        mavlink_rx=mavlink_rx,
        mavlink_tx=mavlink_tx,
        tunnel_rx=tunnel_rx,
        tunnel_tx=tunnel_tx,
        mav_rx_sock=mav_rx_sock,
        mav_peer=mav_peer,
        tun_rx_sock=tun_rx_sock,
        tun_cfg=tun_cfg,
        probe_rx=probe_rx,
    )


LOCAL_NODE = "127.0.0.1"


def build_graph_remote(
    effective: dict,
    cards: list[Card],
    plan: ClusterPlan,
    server_address: str,
    *,
    rand_suffix: Callable[[], str],
) -> GsGraph:
    """Remote wiring: at least one `link.cards` entry is remote-over-SSH.
    Every card (local or remote) becomes a `wfb_rx -f` forwarder feeding a
    per-service `wfb_rx -a` aggregator on the GS; `wfb_tx -d` distributors
    on the GS fan out to per-card `wfb_tx -I` injectors. See the module
    docstring for the stats-interface (`parser`) rationale.

    `cards`/`plan` come from `cards.resolve_cards` + `cluster.plan_cluster`
    (the caller's job — this function is pure argv rendering, like
    `build_graph`). `server_address` is the GS's address as seen by REMOTE
    nodes (baked into their pushed scripts); LOCAL forwarders/injectors
    always target `127.0.0.1` regardless, per wfb-ng's localhost-node
    override.
    """
    link = effective.get("link", {}) or {}
    wfb = effective.get("wfb", {}) or {}
    dl = effective.get("dynamicLink", {}) or {}
    tap = dl.get("tap", {}) or {}

    raw = wfb.get("raw") or {}
    if raw:
        log.warning("wfb.raw ignored by native engine: %s", sorted(raw))

    lid = link_id()
    width = link.get("width", 20)
    uplink_bw = min(max(width, 10), 20)
    key_flag = ["-K", GS_KEY]
    video_key_flag = key_flag if link.get("videoEncryption", True) else []
    tap_flag = ["-D", str(tap.get("port", 8110))] if tap.get("enabled", True) else []
    probe_enabled = bool(dl.get("enabled", False)) and bool(
        (dl.get("probe") or {}).get("enabled", False)
    )

    video_port = plan.server_port["video"]
    mav_port = plan.server_port["mavlink"]
    tun_port = plan.server_port["tunnel"]

    # -- video (rx only, aggregator) ---------------------------------------
    video_argv = _rx_argv(
        head=["-p", "0", "-c", "127.0.0.1", "-u", str(VIDEO_UDP_PORT)],
        key=video_key_flag,
        lid=lid,
        tail=[],
        extra=tap_flag,
        cluster_flag=["-a", str(video_port)],
    )
    video_rx = ServiceSpec(name="video_rx", kind="rx", argv=video_argv, parser="rx", unix_path=None)

    # -- mavlink (rx aggregator + tx distributor) --------------------------
    mav_rx_sock = f"mavlink-rx-{rand_suffix()}"
    mavlink_rx = ServiceSpec(
        name="mavlink_rx",
        kind="rx",
        argv=_rx_argv(
            head=["-p", "16", "-U", mav_rx_sock],
            key=key_flag,
            lid=lid,
            tail=[],
            cluster_flag=["-a", str(mav_port)],
        ),
        parser="rx",
        unix_path=mav_rx_sock,
    )

    mav_tx_sock = f"mavlink-tx-{rand_suffix()}"
    mavlink_tx = ServiceSpec(
        name="mavlink_tx",
        kind="tx",
        argv=_tx_argv(
            port=144,
            unix_path=mav_tx_sock,
            key=key_flag,
            bandwidth=uplink_bw,
            lid=lid,
            tail=plan.peers["mavlink"],
            cluster_flag=["-d"],
        ),
        parser="tx",
        unix_path=mav_tx_sock,
    )

    # -- tunnel (rx aggregator + tx distributor) ---------------------------
    tun_rx_sock = f"tunnel-rx-{rand_suffix()}"
    tunnel_rx = ServiceSpec(
        name="tunnel_rx",
        kind="rx",
        argv=_rx_argv(
            head=["-p", "32", "-U", tun_rx_sock],
            key=key_flag,
            lid=lid,
            tail=[],
            cluster_flag=["-a", str(tun_port)],
        ),
        parser="rx",
        unix_path=tun_rx_sock,
    )

    tun_tx_sock = f"tunnel-tx-{rand_suffix()}"
    tunnel_tx = ServiceSpec(
        name="tunnel_tx",
        kind="tx",
        argv=_tx_argv(
            port=160,
            unix_path=tun_tx_sock,
            key=key_flag,
            bandwidth=uplink_bw,
            lid=lid,
            tail=plan.peers["tunnel"],
            cluster_flag=["-d"],
        ),
        parser="tx",
        unix_path=tun_tx_sock,
    )

    mav_peer_url = (wfb.get("mavlink", {}) or {}).get("peer")
    if not mav_peer_url:
        raise ValueError("wfb.mavlink.peer required")
    mav_peer = MavlinkConfig(peer=mav_peer_url)
    tun_cfg = TunnelConfig(ifname="gs-wfb", ifaddr="10.5.0.1/24", mtu=1445, agg_timeout=0.005)

    # -- probe (rx only, aggregator; 2026-07-06 spec Part B) ----------------
    # Consistency requirement: probe_enabled=True but plan built without probe
    # would KeyError on plan.server_port["probe"] — that pairing is the
    # engine's responsibility (Task 5 passes the same knob to both plan_cluster
    # and the graph builder).
    probe_rx = None
    if probe_enabled:
        probe_rx = ServiceSpec(
            name="probe_rx",
            kind="rx",
            argv=_rx_argv(
                head=["-p", str(PROBE_RADIO_PORT), "-c", "127.0.0.1", "-u", str(PROBE_SINK_PORT)],
                key=key_flag,
                lid=lid,
                tail=[],
                cluster_flag=["-a", str(plan.server_port["probe"])],
            ),
            parser="probe",
            unix_path=None,
        )

    # -- local forwarders/injectors (only if the local node has cards) ----
    local_forwarders: list[ServiceSpec] = []
    local_injectors: list[ServiceSpec] = []
    local_cards = plan.nodes.get(LOCAL_NODE, [])
    if local_cards:
        ifaces = local_ifaces(cards)
        svc_streams = streams_for(probe_enabled)
        for service, streams in svc_streams.items():
            rx_id = streams["rx"]
            if rx_id is not None:
                fwd_argv = [
                    f"{WFB_BIN_DIR}/wfb_rx",
                    "-f",
                    "-c",
                    "127.0.0.1",
                    "-u",
                    str(plan.server_port[service]),
                    "-p",
                    str(rx_id),
                    "-i",
                    str(lid),
                    "-R",
                    str(BUF_R),
                ] + ifaces
                local_forwarders.append(
                    ServiceSpec(
                        name=f"{service} fwd",
                        kind="rx",
                        argv=fwd_argv,
                        parser=None,
                        unix_path=None,
                    )
                )
            tx_id = streams["tx"]
            if tx_id is not None:
                injector_base = plan.injector_base[(LOCAL_NODE, service)]
                inj_argv = [
                    f"{WFB_BIN_DIR}/wfb_tx",
                    "-I",
                    str(injector_base),
                    "-R",
                    str(BUF_R),
                ] + ifaces
                local_injectors.append(
                    ServiceSpec(
                        name=f"{service} inj",
                        kind="tx",
                        argv=inj_argv,
                        parser=None,
                        unix_path=None,
                    )
                )

    # -- remote node scripts (one per remote node) -------------------------
    node_scripts: dict[str, str] = {}
    for node in sorted(plan.nodes):
        if node == LOCAL_NODE:
            continue
        node_scripts[node] = render_node_script(
            node,
            plan.nodes[node],
            plan,
            link=link,
            link_id=lid,
            server_address=server_address,
            streams=streams_for(probe_enabled),
        )

    return GsGraph(
        video_rx=video_rx,
        mavlink_rx=mavlink_rx,
        mavlink_tx=mavlink_tx,
        tunnel_rx=tunnel_rx,
        tunnel_tx=tunnel_tx,
        mav_rx_sock=mav_rx_sock,
        mav_peer=mav_peer,
        tun_rx_sock=tun_rx_sock,
        tun_cfg=tun_cfg,
        local_forwarders=local_forwarders,
        local_injectors=local_injectors,
        node_scripts=node_scripts,
        probe_rx=probe_rx,
    )
