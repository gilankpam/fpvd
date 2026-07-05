"""Parsers for the wfb_rx / wfb_tx stdout line protocol (IPC_MSG lines).

Port of wfb_ng/protocols.py RXAntennaProtocol + TXAntennaProtocol (Twisted
LineReceiver) to plain feed_line() callables. Wire format reference:
wfb-ng fork src/rx.cpp dump_stats(), src/tx.cpp.
"""

from __future__ import annotations

import logging

log = logging.getLogger("fpvdgs.wfb")

FEC_TYPES = {1: "VDM_RS", 2: "swfec"}

# RX PKT counter names, in wire order (fork emits exactly these 11;
# tolerate more, reject fewer — mirrors RXAntennaProtocol).
RX_PKT_KEYS = (
    "all",
    "all_bytes",
    "dec_err",
    "session",
    "data",
    "uniq",
    "fec_rec",
    "lost",
    "bad",
    "out",
    "out_bytes",
)
TX_PKT_KEYS = (
    "fec_timeouts",
    "incoming",
    "incoming_bytes",
    "injected",
    "injected_bytes",
    "dropped",
    "truncated",
)


class _Bad(Exception):
    pass


def _pairs(counters, cumulative):
    """(window, cumulative) pairs keyed like wfb-ng's stats dicts."""
    if cumulative is None:
        cumulative = counters
    else:
        cumulative = tuple(a + b for a, b in zip(counters, cumulative))
    return cumulative


class RxLineParser:
    def __init__(self, child_id: str, on_window, on_session):
        self.child_id = child_id
        self._on_window = on_window
        self._on_session = on_session
        self._ant: dict = {}
        self._cum: tuple | None = None
        self._session: dict | None = None

    def feed_line(self, line: str) -> None:
        cols = line.strip().split("\t")
        try:
            if len(cols) < 2:
                raise _Bad()
            cmd = cols[1]
            if cmd == "RX_ANT":
                if len(cols) != 5:
                    raise _Bad()
                key = tuple(int(v) for v in cols[2].split(":"))
                if len(key) != 3:
                    raise _Bad()
                vals = tuple(int(v) for v in cols[4].split(":"))
                if len(vals) == 7:  # pre-EVM wfb_rx
                    vals = vals + (-1, -1, -1)
                if len(vals) != 10:
                    raise _Bad()
                self._ant[(key, int(cols[3], 16))] = vals
            elif cmd == "PKT":
                if len(cols) != 3:
                    raise _Bad()
                counters = tuple(int(v) for v in cols[2].split(":"))
                if len(counters) < len(RX_PKT_KEYS):
                    raise _Bad()
                counters = counters[: len(RX_PKT_KEYS)]
                self._cum = _pairs(counters, self._cum)
                packets = dict(zip(RX_PKT_KEYS, zip(counters, self._cum)))
                self._on_window(self.child_id, packets, dict(self._ant), self._session)
                self._ant.clear()
            elif cmd == "SESSION":
                if len(cols) != 3:
                    raise _Bad()
                parts = [int(v) for v in cols[2].split(":")]
                if len(parts) < 4:
                    raise _Bad()
                epoch, fec_type, fec_k, fec_n = parts[:4]
                session = {
                    "fec_type": FEC_TYPES.get(fec_type, "Unknown"),
                    "fec_k": fec_k,
                    "fec_n": fec_n,
                    "epoch": epoch,
                    "contract_version": parts[4] if len(parts) > 4 else 1,
                }
                # SESSION arrives on-change AND once per window; dedup.
                if session != self._session:
                    self._session = session
                    self._on_session(self.child_id, session)
            else:
                raise _Bad()
        except (_Bad, ValueError):
            log.warning("bad telemetry [%s]: %s", self.child_id, line.strip())


class TxLineParser:
    def __init__(self, child_id: str, on_window):
        self.child_id = child_id
        self._on_window = on_window
        self.ports: dict[int, int] = {}
        self.unix_sockets: dict[int, str] = {}
        self.control_port: int | None = None
        self.handshake_done = False
        self.on_handshake = None
        self._ant: dict = {}
        self._cum: tuple | None = None

    def _finish_handshake(self):
        self.handshake_done = True
        if self.on_handshake is not None:
            self.on_handshake()

    def feed_line(self, line: str) -> None:
        cols = line.strip().split("\t")
        if len(cols) < 2:
            return
        cmd = cols[1]
        try:
            if cmd == "LISTEN_UDP" and len(cols) == 3:
                port, wlan_id = cols[2].split(":", 1)
                self.ports[int(wlan_id, 16)] = int(port)
            elif cmd == "LISTEN_UDP_END":
                self._finish_handshake()
            elif cmd == "LISTEN_UNIX" and len(cols) == 3:
                sock, wlan_id = cols[2].rsplit(":", 1)
                self.unix_sockets[int(wlan_id, 16)] = sock
            elif cmd == "LISTEN_UNIX_END":
                self._finish_handshake()
            elif cmd == "LISTEN_UDP_CONTROL" and len(cols) == 3:
                self.control_port = int(cols[2])
            elif cmd == "TX_ANT":
                if len(cols) != 4:
                    raise _Bad()
                self._ant[int(cols[2], 16)] = tuple(int(v) for v in cols[3].split(":"))
            elif cmd == "PKT":
                if len(cols) != 3:
                    raise _Bad()
                counters = tuple(int(v) for v in cols[2].split(":"))
                if len(counters) != len(TX_PKT_KEYS):
                    raise _Bad()
                self._cum = _pairs(counters, self._cum)
                packets = dict(zip(TX_PKT_KEYS, zip(counters, self._cum)))
                self._on_window(self.child_id, packets, dict(self._ant))
                self._ant.clear()
        except (_Bad, ValueError):
            log.warning("bad telemetry [%s]: %s", self.child_id, line.strip())
