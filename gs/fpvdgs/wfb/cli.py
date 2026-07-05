"""fpvd-stats: live debug CLI for the :8103 JSON stats feed.

Works against both the old wfb-ng `StatisticsJSONProtocol` server and
the native `fpvdgs.wfb.statsd.StatsServer` -- same newline-JSON wire
schema, decoded by the one shared parser
(`fpvdgs.dynlink.stats_client.parse_record`).

Deliberately NOT asyncio: a dead-simple blocking socket line reader.
This is a debug tool, not a supervised child -- no reconnect, no
event loop; on disconnect it prints a message and returns.
"""

from __future__ import annotations

import argparse
import json
import socket
import struct
import sys
from typing import Iterable, Iterator
from urllib.parse import urlparse

from fpvdgs.dynlink.stats_client import (
    ContractVersionError,
    Event,
    RxEvent,
    SessionEvent,
    SettingsEvent,
    TxEvent,
    parse_record,
)

CLEAR = "\x1b[H\x1b[2J"


# Small local duplicate of stats_client's endpoint parser: that one is
# a private (`_`-prefixed) helper of `StatsClient`, and this CLI has no
# other reason to depend on stats_client internals beyond `parse_record`.
def _parse_endpoint(url: str) -> tuple[str, int]:
    """tcp://host:port -> (host, port). Bare host:port is also accepted."""
    if "://" not in url:
        url = "tcp://" + url
    parsed = urlparse(url)
    if parsed.scheme != "tcp":
        raise ValueError(f"unsupported scheme {parsed.scheme!r} in {url!r}")
    if not parsed.hostname or not parsed.port:
        raise ValueError(f"endpoint must be tcp://host:port, got {url!r}")
    return parsed.hostname, parsed.port


def _connect(endpoint: str) -> socket.socket:
    host, port = _parse_endpoint(endpoint)
    return socket.create_connection((host, port))


def _lines(sock: socket.socket) -> Iterator[str]:
    """Blocking line iterator over a connected socket. Ends (StopIteration)
    on EOF -- the tool's only "disconnect" signal."""
    f = sock.makefile("r")
    for raw_line in f:
        line = raw_line.strip()
        if line:
            yield line


def _iter_windows(lines: Iterable[str]) -> Iterator[list[Event]]:
    """Group parsed events into per-window batches.

    A `SettingsEvent` only arrives once (on connect) but is a static
    header, so it is carried forward and prefixed onto every window.
    Everything else accumulates into the current batch until a second
    `RxEvent` for an id already in the batch arrives -- that marks the
    start of the next window, so the batch-so-far is yielded first.
    """
    settings_ev: SettingsEvent | None = None
    batch: list[Event] = []
    seen_ids: set[str] = set()
    for line in lines:
        try:
            raw = json.loads(line)
        except json.JSONDecodeError:
            continue
        try:
            ev = parse_record(raw)
        except ContractVersionError:
            sys.stderr.write(
                "fpvd-stats: unsupported feed contract_version — showing raw lines may still work with --json\n"
            )
            continue
        except (KeyError, ValueError, TypeError):
            sys.stderr.write(
                "fpvd-stats: skipping malformed record: KeyError/ValueError/TypeError\n"
            )
            continue
        if ev is None:
            continue
        if isinstance(ev, SettingsEvent):
            settings_ev = ev
            continue
        if isinstance(ev, RxEvent):
            if ev.id in seen_ids:
                yield ([settings_ev] if settings_ev else []) + batch
                batch = []
                seen_ids = set()
            seen_ids.add(ev.id)
        batch.append(ev)
    if batch:
        yield ([settings_ev] if settings_ev else []) + batch


def _fmt_triplet(lo: int, avg: int, hi: int) -> str:
    return f"{lo}/{avg}/{hi}"


def _card_label(wlan: int) -> str:
    """Render a wlan id as a card label. A cluster-encoded id (Phase 2 remote
    cards, `wfb.cluster.cluster_wlan_id`: `(node_ipv4 << 24) | wlan_idx`)
    carries the node's ipv4 in its high bits -- decode and show `node <ip>
    card <n>`. A plain local wlan id (ipv4 part == 0) keeps the Phase 1
    `card <n>` format unchanged."""
    node_part = wlan >> 24
    if node_part:
        node_ip = socket.inet_ntoa(struct.pack("!L", node_part))
        card_idx = wlan & 0xFFFFFF
        return f"node {node_ip} card {card_idx}"
    return f"card {wlan}"


def render_frame(events: list) -> str:
    """Pure renderer: a batch of events for one window -> a terminal frame.

    No I/O, no ANSI clear (the caller decides when to clear) -- kept
    separate so it is trivially unit-testable.
    """
    out: list[str] = []
    for ev in events:
        if isinstance(ev, SettingsEvent):
            out.append(
                f"[settings] profile={ev.profile} cluster={ev.is_cluster} "
                f"wlans={','.join(ev.wlans)}"
            )
        elif isinstance(ev, RxEvent):
            out.append(f"-- {ev.id} --")
            for ant in ev.rx_ant_stats:
                wlan = ant.ant >> 8
                ant_idx = ant.ant & 0xFF
                marker = " *" if ev.tx_wlan is not None and wlan == ev.tx_wlan else ""
                out.append(
                    f"{_card_label(wlan)}{marker} ant {ant_idx} | pkt {ant.pkt_recv} | "
                    f"rssi {_fmt_triplet(ant.rssi_min, ant.rssi_avg, ant.rssi_max)} | "
                    f"snr {_fmt_triplet(ant.snr_min, ant.snr_avg, ant.snr_max)} | "
                    f"evm {_fmt_triplet(ant.evm_min, ant.evm_avg, ant.evm_max)}"
                )
            fec_rec = ev.packets_window.get("fec_rec", 0)
            lost = ev.packets_window.get("lost", 0)
            bad = ev.packets_window.get("bad", 0)
            out.append(f"  counters: fec_rec={fec_rec} lost={lost} bad={bad}")
            if ev.session is not None:
                s = ev.session
                out.append(f"  session: {s.fec_type} k={s.fec_k}/n={s.fec_n} epoch={s.epoch}")
        elif isinstance(ev, SessionEvent):
            s = ev.session
            out.append(
                f"-- {ev.id} session: {s.fec_type} k={s.fec_k}/n={s.fec_n} epoch={s.epoch} --"
            )
        elif isinstance(ev, TxEvent):
            sent = ev.packets_window.get("sent", 0)
            out.append(f"-- {ev.id} (tx) -- sent={sent}")
    return "\n".join(out) + "\n"


def _run_json(lines: Iterable[str], *, once: bool) -> None:
    for line in lines:
        print(line)
        if once:
            try:
                raw = json.loads(line)
            except json.JSONDecodeError:
                continue
            if raw.get("type") == "rx":
                return
    print("fpvd-stats: disconnected", file=sys.stderr)


def _run_rendered(lines: Iterable[str], *, once: bool) -> None:
    for window in _iter_windows(lines):
        if not once:
            sys.stdout.write(CLEAR)
        sys.stdout.write(render_frame(window))
        sys.stdout.flush()
        if once:
            return
    print("fpvd-stats: disconnected", file=sys.stderr)


def main(argv: list[str] | None = None) -> None:
    argv = sys.argv[1:] if argv is None else argv
    p = argparse.ArgumentParser(prog="fpvd-stats", description="Live :8103 wfb stats viewer")
    p.add_argument("--endpoint", default="tcp://127.0.0.1:8103")
    p.add_argument("--json", action="store_true", help="print raw JSON lines, no rendering")
    p.add_argument(
        "--once",
        action="store_true",
        help="render one window (with --json: print through the first rx line) then exit",
    )
    args = p.parse_args(argv)

    try:
        sock = _connect(args.endpoint)
    except OSError as e:
        sys.stderr.write(f"fpvd-stats: connect {args.endpoint} failed: {e}\n")
        raise SystemExit(1)

    with sock:
        lines = _lines(sock)
        try:
            if args.json:
                _run_json(lines, once=args.once)
            else:
                _run_rendered(lines, once=args.once)
        except OSError as e:
            sys.stderr.write(f"fpvd-stats: disconnected: {e}\n")
            raise SystemExit(1)


if __name__ == "__main__":
    main()
