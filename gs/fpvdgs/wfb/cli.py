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


# Marker for a row (stream/antenna/counter) that exists but received nothing
# in the current window -- see StickyRenderer.
NO_DATA = "(no data)"


def _settings_line(ev: SettingsEvent) -> str:
    return f"[settings] profile={ev.profile} cluster={ev.is_cluster} wlans={','.join(ev.wlans)}"


def _ant_line(ant, tx_wlan: int | None) -> str:
    wlan = ant.ant >> 8
    ant_idx = ant.ant & 0xFF
    marker = " *" if tx_wlan is not None and wlan == tx_wlan else ""
    return (
        f"{_card_label(wlan)}{marker} ant {ant_idx} | pkt {ant.pkt_recv} | "
        f"rssi {_fmt_triplet(ant.rssi_min, ant.rssi_avg, ant.rssi_max)} | "
        f"snr {_fmt_triplet(ant.snr_min, ant.snr_avg, ant.snr_max)} | "
        f"evm {_fmt_triplet(ant.evm_min, ant.evm_avg, ant.evm_max)}"
    )


def _ant_placeholder_line(ant_id: int, tx_wlan: int | None) -> str:
    """A card+antenna slot that received nothing this window (sticky row)."""
    wlan = ant_id >> 8
    ant_idx = ant_id & 0xFF
    marker = " *" if tx_wlan is not None and wlan == tx_wlan else ""
    return f"{_card_label(wlan)}{marker} ant {ant_idx} | {NO_DATA}"


def _counters_line(ev: RxEvent) -> str:
    fec_rec = ev.packets_window.get("fec_rec", 0)
    lost = ev.packets_window.get("lost", 0)
    bad = ev.packets_window.get("bad", 0)
    return f"  counters: fec_rec={fec_rec} lost={lost} bad={bad}"


def _session_line(s) -> str:
    return f"  session: {s.fec_type} k={s.fec_k}/n={s.fec_n} epoch={s.epoch}"


def render_frame(events: list) -> str:
    """Pure renderer: a batch of events for one window -> a terminal frame.

    No I/O, no ANSI clear (the caller decides when to clear) -- kept
    separate so it is trivially unit-testable. Renders exactly the events it
    is given; the sticky "keep empty rows in place" behavior lives in
    StickyRenderer, which wraps this primitive's building blocks with
    cross-window memory.

    Streams and antennas are rendered in a fixed order (settings first, then
    streams by `id`, then each stream's antennas by card+ant index) so the
    frame stays put between windows: the raw feed carries streams in
    window-arrival order and antennas in the producer's dict order, both of
    which jitter, which would otherwise make the display shuffle every tick.
    """
    out: list[str] = []
    events = sorted(events, key=lambda ev: (0, "") if isinstance(ev, SettingsEvent) else (1, ev.id))
    for ev in events:
        if isinstance(ev, SettingsEvent):
            out.append(_settings_line(ev))
        elif isinstance(ev, RxEvent):
            out.append(f"-- {ev.id} --")
            for ant in sorted(ev.rx_ant_stats, key=lambda a: a.ant):
                out.append(_ant_line(ant, ev.tx_wlan))
            out.append(_counters_line(ev))
            if ev.session is not None:
                out.append(_session_line(ev.session))
        elif isinstance(ev, SessionEvent):
            s = ev.session
            out.append(
                f"-- {ev.id} session: {s.fec_type} k={s.fec_k}/n={s.fec_n} epoch={s.epoch} --"
            )
        elif isinstance(ev, TxEvent):
            sent = ev.packets_window.get("sent", 0)
            out.append(f"-- {ev.id} (tx) -- sent={sent}")
    return "\n".join(out) + "\n"


class StickyRenderer:
    """Stateful renderer that keeps every stream and card+antenna row in a
    fixed slot once seen, so a row never collapses when it misses a window.

    The raw feed only carries a stream (or an antenna within it) when it
    received at least one packet in that 100 ms window, so on a marginal link
    a row drops out for a window and the rows below shift up -- then it
    reappears next window. This renderer remembers the union of stream ids and
    `(card, ant)` keys seen and renders a `(no data)` placeholder for any that
    are absent this window, so the layout stays put.

    The remembered set is cleared when a stream starts a *new session* (its
    session epoch changes -- wfb_rx restart / key-epoch roll), so stale rows
    from a previous link don't linger forever.
    """

    def __init__(self) -> None:
        # id -> "rx" | "tx" (first-seen; rendered sorted by id regardless)
        self._streams: dict[str, str] = {}
        # id -> set of ant ids (wlan<<8 | ant_idx) ever seen for that stream
        self._ants: dict[str, set[int]] = {}
        # id -> last-seen session epoch, to detect a new session
        self._epoch: dict[str, int] = {}

    def render(self, events: list) -> str:
        self._maybe_reset(events)
        settings, by_id = self._register(events)
        out: list[str] = []
        if settings is not None:
            out.append(_settings_line(settings))
        for sid in sorted(self._streams):
            ev = by_id.get(sid)
            if self._streams[sid] == "tx":
                self._emit_tx(out, sid, ev)
            else:
                self._emit_rx(out, sid, ev)
        return "\n".join(out) + "\n"

    # -- state ---------------------------------------------------------------
    def _session_epochs(self, events: list):
        for ev in events:
            if isinstance(ev, RxEvent) and ev.session is not None:
                yield ev.id, ev.session.epoch
            elif isinstance(ev, SessionEvent):
                yield ev.id, ev.session.epoch

    def _maybe_reset(self, events: list) -> None:
        for sid, epoch in self._session_epochs(events):
            if sid in self._epoch and self._epoch[sid] != epoch:
                self._streams.clear()
                self._ants.clear()
                self._epoch.clear()
                return

    def _register(self, events: list):
        """Fold this window into the known-row memory; return (settings,
        {id: event}) for rendering."""
        settings: SettingsEvent | None = None
        by_id: dict[str, object] = {}
        for ev in events:
            if isinstance(ev, SettingsEvent):
                settings = ev
            elif isinstance(ev, RxEvent):
                self._streams.setdefault(ev.id, "rx")
                ants = self._ants.setdefault(ev.id, set())
                ants.update(a.ant for a in ev.rx_ant_stats)
                if ev.session is not None:
                    self._epoch[ev.id] = ev.session.epoch
                by_id[ev.id] = ev
            elif isinstance(ev, TxEvent):
                self._streams.setdefault(ev.id, "tx")
                self._ants.setdefault(ev.id, set())
                by_id[ev.id] = ev
            elif isinstance(ev, SessionEvent):
                self._streams.setdefault(ev.id, "rx")
                self._ants.setdefault(ev.id, set())
                self._epoch[ev.id] = ev.session.epoch
                by_id.setdefault(ev.id, ev)  # only if no RxEvent this window
        return settings, by_id

    # -- rendering -----------------------------------------------------------
    def _emit_rx(self, out: list[str], sid: str, ev) -> None:
        out.append(f"-- {sid} --")
        present = {a.ant: a for a in ev.rx_ant_stats} if isinstance(ev, RxEvent) else {}
        tx_wlan = ev.tx_wlan if isinstance(ev, RxEvent) else None
        for ant_id in sorted(self._ants[sid]):
            ant = present.get(ant_id)
            out.append(
                _ant_line(ant, tx_wlan)
                if ant is not None
                else _ant_placeholder_line(ant_id, tx_wlan)
            )
        if isinstance(ev, RxEvent):
            out.append(_counters_line(ev))
            if ev.session is not None:
                out.append(_session_line(ev.session))
        else:
            out.append(f"  counters: {NO_DATA}")
            if isinstance(ev, SessionEvent):
                out.append(_session_line(ev.session))

    def _emit_tx(self, out: list[str], sid: str, ev) -> None:
        if isinstance(ev, TxEvent):
            out.append(f"-- {sid} (tx) -- sent={ev.packets_window.get('sent', 0)}")
        else:
            out.append(f"-- {sid} (tx) -- {NO_DATA}")


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
    sticky = StickyRenderer()
    for window in _iter_windows(lines):
        if not once:
            sys.stdout.write(CLEAR)
        sys.stdout.write(sticky.render(window))
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
