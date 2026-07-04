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
`-B min(link.width, 20)`; the video rx leg renders no `-B` at all.
"""

from __future__ import annotations

import hashlib
import logging
from collections.abc import Callable
from dataclasses import dataclass

from .mavproxy import MavlinkConfig
from .tunnel import TunnelConfig

log = logging.getLogger("fpvdgs.wfb")

WFB_BIN_DIR = "/usr/bin"
GS_KEY = "/etc/gs.key"
LOG_INTERVAL = 100
BUF_R = 2097152
BUF_S = 2097152
VIDEO_UDP_PORT = 5600  # wfb-ng's gs_video default; independent of pixelpilot.rtpPort

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
    parser: str  # "rx" | "tx"
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


def _rx_common_tail(lid: int) -> list[str]:
    return ["-R", str(BUF_R), "-s", str(BUF_S), "-l", str(LOG_INTERVAL), "-i", str(lid)]


def _tx_argv(
    *, port: int, unix_path: str, key: list[str], bandwidth: int, lid: int, wlans: list[str]
) -> list[str]:
    return (
        [f"{WFB_BIN_DIR}/wfb_tx", "-f", "data", "-p", str(port), "-U", unix_path]
        + key
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
        + list(wlans)
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
    uplink_bw = min(width, 20)
    key_flag = ["-K", GS_KEY]
    video_key_flag = key_flag if link.get("videoEncryption", True) else []

    # -- video (rx only) --------------------------------------------------
    video_argv = [
        f"{WFB_BIN_DIR}/wfb_rx",
        "-p",
        "0",
        "-c",
        "127.0.0.1",
        "-u",
        str(VIDEO_UDP_PORT),
    ]
    video_argv += video_key_flag
    video_argv += _rx_common_tail(lid)
    if tap.get("enabled", True):
        video_argv += ["-D", str(tap.get("port", 8110))]
    video_argv += wlans

    video_rx = ServiceSpec(name="video_rx", kind="rx", argv=video_argv, parser="rx", unix_path=None)

    # -- mavlink (rx + tx) --------------------------------------------------
    mav_rx_sock = f"mavlink-rx-{rand_suffix()}"
    mavlink_rx = ServiceSpec(
        name="mavlink_rx",
        kind="rx",
        argv=(
            [f"{WFB_BIN_DIR}/wfb_rx", "-p", "16", "-U", mav_rx_sock]
            + key_flag
            + _rx_common_tail(lid)
            + wlans
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
            wlans=wlans,
        ),
        parser="tx",
        unix_path=mav_tx_sock,
    )

    # -- tunnel (rx + tx) --------------------------------------------------
    tun_rx_sock = f"tunnel-rx-{rand_suffix()}"
    tunnel_rx = ServiceSpec(
        name="tunnel_rx",
        kind="rx",
        argv=(
            [f"{WFB_BIN_DIR}/wfb_rx", "-p", "32", "-U", tun_rx_sock]
            + key_flag
            + _rx_common_tail(lid)
            + wlans
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
            wlans=wlans,
        ),
        parser="tx",
        unix_path=tun_tx_sock,
    )

    mav_peer = MavlinkConfig(peer=(wfb.get("mavlink", {}) or {}).get("peer"))
    tun_cfg = TunnelConfig(ifname="gs-wfb", ifaddr="10.5.0.1/24", mtu=1445, agg_timeout=0.005)

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
    )
