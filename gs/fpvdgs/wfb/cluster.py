"""Cluster port allocation + wlan-id encoding for `link.cards` (local +
remote-over-SSH). Pure compute: no I/O, no sockets (`socket.inet_aton` is
used only as a dotted-quad parser, never to open a connection) — with one
exception: `derive_server_address` briefly opens a UDP socket purely for
the kernel routing-table lookup (no packet is ever sent, see below).

Faithfully ports the allocation semantics of wfb-ng's
`parse_cluster_services` (wfb_ng/cluster.py): a single server-port counter
shared across services (one GS aggregator UDP port per service), and a
per-node injector-port counter (one `wfb_tx -I` base port per (node,
service), advanced by that node's card count per service).

This module also owns the persistent-SSH node session (`NodeSession`) that
keeps a remote node's bootstrap script (`render_node_script`, above) alive
for the life of the flight: the session is the *only* thing standing
between "SSH connection drops" and "node's wfb_rx/wfb_tx children get
cleaned up" — the script's own `trap _cleanup EXIT` + the `cat > /dev/null`
watchdog line (`ssh_mode=True` in `render_node_script`) fire the moment the
session's stdin/subprocess goes away, so `NodeSession` never needs to SSH
back in to run a kill command itself.
"""

from __future__ import annotations

import asyncio
import contextlib
import itertools
import logging
import socket
import struct
from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any

from .. import radio
from .cards import Card

log = logging.getLogger("fpvdgs.wfb")

KILL_TIMEOUT_S = 5.0

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


def derive_server_address(node_host: str, override: str | None = None) -> str:
    """The address a remote node's `wfb_rx -c` should target to reach this
    GS: `override` wins verbatim (operator escape hatch for multi-homed/NAT
    setups); otherwise derive it via the UDP-connect routing trick —
    `connect()` on a `SOCK_DGRAM` socket never sends a packet, it only asks
    the kernel to resolve the route to `node_host` and binds the socket to
    the local (GS-side) source address that route would use, which is
    exactly the address the node needs to send its video back to.
    """
    if override:
        return override
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect((node_host, 22))
        return s.getsockname()[0]
    finally:
        s.close()


def ssh_argv(card: Card) -> list[str]:
    """The `ssh` argv used to open a node session for `card`'s host. Runs
    `exec sh -s` — NOT `bash -s` — because the remote nodes are OpenWrt /
    BusyBox territory with no bash; `sh -s` reads the piped script from
    stdin under whatever POSIX shell is present. `BatchMode=yes` fails fast
    on a prompt (password/host-key) instead of hanging a session forever;
    `ServerAlive*` detects a dead link even when the node itself stays
    silent; `StrictHostKeyChecking=accept-new` auto-trusts an unseen host
    key without downgrading known-host protection on a key change.
    """
    argv = [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ServerAliveInterval=5",
        "-o",
        "ServerAliveCountMax=3",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-p",
        str(card.ssh_port),
    ]
    if card.ssh_key:
        argv += ["-i", card.ssh_key]
    argv += [f"{card.ssh_user}@{card.host}", "exec sh -s"]
    return argv


class NodeSession:
    """Keeps one remote node's bootstrap script alive over a persistent SSH
    session, for the life of the flight.

    Asyncio-native sibling of `WfbChild` (`children.py`) — same
    spawn/crash-watch/backoff shape — but much simpler: no stdout stats
    parsing (stdout/stderr are logged lines only, never fed to a parser),
    no process group (a plain ssh child has nothing else to reap), and
    **degrade, never fail**: a node that can't be reached (or keeps
    dying) just backs off and keeps retrying forever, reporting
    `on_state(node, False)` while it's down — it must never take the rest
    of the array down with it (validated by the engine wiring, Task 6).

    `node` identifies the session for logging and the `on_state` callback;
    in production it is typically the node's host string, with
    `argv_builder` bound to the concrete `Card` (e.g.
    `functools.partial(ssh_argv, card)`) so `node` and the SSH connection
    details can vary independently. The default `argv_builder=ssh_argv`
    expects `node` itself to be a `Card`.

    The script is written to the child's stdin and stdin is then left
    **open** for the life of the session: closing it (or the process
    dying) is what the node-side script notices (its `cat > /dev/null`
    watchdog line exits) and unwinds via its own `trap _cleanup EXIT` —
    `NodeSession` never SSHes back in to send a kill command.
    """

    def __init__(
        self,
        node: Any,
        script: str,
        *,
        argv_builder: Callable[[Any], list[str]] = ssh_argv,
        backoff: float = 2.0,
        max_backoff: float = 30.0,
        stable_reset_s: float | None = None,
        on_state: Callable[[Any, bool], None] | None = None,
    ):
        self.node = node
        self.script = script
        self._argv_builder = argv_builder
        self.backoff = backoff
        self.max_backoff = max_backoff
        # A spawn that stayed up at least this long is "durable": the next
        # exit resets the backoff to base instead of continuing to grow it.
        # Defaults to max_backoff (one full worst-case interval of uptime
        # is enough to call the session stable).
        self._stable_reset_s = max_backoff if stable_reset_s is None else stable_reset_s
        self._on_state = on_state

        self.alive = False
        self._restarts = 0
        self._proc: asyncio.subprocess.Process | None = None
        self._pump_task: asyncio.Task | None = None
        self._watch_task: asyncio.Task | None = None
        self._supervise = False
        self._spawned_at: float | None = None

    # -- lifecycle ----------------------------------------------------------
    async def start(self) -> bool:
        self._supervise = True
        ok = await self._spawn()
        self._watch_task = asyncio.ensure_future(self._watch())
        return ok

    async def stop(self) -> None:
        self._supervise = False
        if self._watch_task is not None:
            self._watch_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._watch_task
            self._watch_task = None
        await self._kill()

    def state(self) -> dict:
        return {"alive": self.alive, "restarts": self._restarts}

    # -- spawn ----------------------------------------------------------------
    async def _spawn(self) -> bool:
        argv = self._argv_builder(self.node)
        try:
            proc = await asyncio.create_subprocess_exec(
                *argv,
                stdin=asyncio.subprocess.PIPE,
                stdout=asyncio.subprocess.PIPE,
                stderr=asyncio.subprocess.STDOUT,
            )
        except OSError as e:
            log.warning("node[%s]: spawn failed: %s", self.node, e)
            self._proc = None
            self.alive = False
            self._spawned_at = None
            return False

        self._proc = proc
        if self._pump_task is not None and not self._pump_task.done():
            self._pump_task.cancel()
        self._pump_task = asyncio.ensure_future(self._pump_stdout(proc))

        if proc.stdin is not None:
            try:
                proc.stdin.write(self.script.encode())
                await proc.stdin.drain()
            except (BrokenPipeError, ConnectionResetError) as e:
                log.warning("node[%s]: failed writing script to stdin: %s", self.node, e)
                # Left in place: the watch loop's proc.wait() will observe
                # the (presumably imminent) exit and drive the retry.

        self.alive = True
        self._spawned_at = asyncio.get_running_loop().time()
        return True

    async def _pump_stdout(self, proc: asyncio.subprocess.Process) -> None:
        try:
            while True:
                line = await proc.stdout.readline()
                if not line:
                    return
                log.info("node[%s]: %s", self.node, line.decode(errors="replace").rstrip())
        except asyncio.CancelledError:
            raise
        except Exception:
            log.exception("node[%s]: stdout pump crashed", self.node)

    # -- kill -------------------------------------------------------------------
    async def _kill(self) -> None:
        proc, self._proc = self._proc, None
        self.alive = False
        if proc is not None and proc.returncode is None:
            if proc.stdin is not None and not proc.stdin.is_closing():
                with contextlib.suppress(Exception):
                    proc.stdin.close()
            try:
                proc.terminate()
            except ProcessLookupError:
                pass
            try:
                await asyncio.wait_for(proc.wait(), timeout=KILL_TIMEOUT_S)
            except (TimeoutError, asyncio.TimeoutError):
                with contextlib.suppress(ProcessLookupError):
                    proc.kill()
                await proc.wait()
        if self._pump_task is not None:
            pump_task, self._pump_task = self._pump_task, None
            try:
                await asyncio.wait_for(pump_task, timeout=1.0)
            except (TimeoutError, asyncio.TimeoutError):
                pump_task.cancel()
                with contextlib.suppress(asyncio.CancelledError):
                    await pump_task

    # -- crash watch --------------------------------------------------------
    def _notify(self, up: bool) -> None:
        if self._on_state is None:
            return
        try:
            self._on_state(self.node, up)
        except Exception:
            log.exception("node[%s]: on_state callback failed", self.node)

    async def _watch(self) -> None:
        """Sequence per the interface contract: an unexpected exit (or a
        failed initial `start()`) reports `on_state(node, False)`, then
        backs off (exponential, capped at `max_backoff`) and keeps retrying
        the spawn *forever* (structural spawn failures, e.g. `ssh` missing,
        loop right back into the same backoff — never fatal), then reports
        `on_state(node, True)` once a respawn lands. `stop()` cancels this
        task outright, so a deliberate stop is never treated as a crash.

        A chronically-flapping node settles at the slowest retry rate and
        stays there — but only while it keeps failing fast. If a spawn
        stays up at least `_stable_reset_s` before exiting, that was a
        durable session (e.g. one unrelated blip after hours stable), so
        the backoff resets to base (`self.backoff`) before computing the
        next retry delay: a stable node reconnects fast, only a genuine
        crash-loop gets throttled toward `max_backoff`.
        """
        delay = self.backoff
        while True:
            proc = self._proc
            if proc is not None:
                spawned_at = self._spawned_at
                await proc.wait()
                if not self._supervise:
                    return
                self.alive = False
                if (
                    spawned_at is not None
                    and asyncio.get_running_loop().time() - spawned_at >= self._stable_reset_s
                ):
                    delay = self.backoff
            self._notify(False)

            while True:
                await asyncio.sleep(delay)
                delay = min(delay * 2, self.max_backoff)
                if not self._supervise:
                    return
                self._restarts += 1
                if await self._spawn():
                    break
            self._notify(True)
