"""Event types + wire decoder for wfb-ng's JSON stats API (§3).

The server is `StatisticsJSONProtocol` in wfb_ng/protocols.py:72 (or, on
the GS, `fpvdgs.wfb.statsd.StatsServer`, a compatible re-implementation).
Newline-delimited JSON, one record per line. First record on connect is
a 'settings' dump; subsequent records are 'rx', 'tx', or 'new_session' at
`log_interval` cadence (we require 100 ms → 10 Hz).

There is no TCP client class here anymore — native mode always receives
events via the engine's in-process, StatsClient-compatible
`client_factory()` (see `fpvdgs/wfb/engine.py`). `ReplayClient` below
replays a captured JSONL file through the same `on_event` convention for
offline use.
"""

from __future__ import annotations

import asyncio
import json
import logging
from dataclasses import dataclass, field
from typing import Any, Callable
from urllib.parse import urlparse

log = logging.getLogger(__name__)

# wfb-ng's contract_version field — bumped by upstream on
# session-record schema changes. Vanilla wfb-ng emits 1; the
# feat/interleaving_uep branch emits 2. We accept both because we
# decode the same minimal subset (fec_*, epoch) regardless.
# v3: wfb-ng swfec fork — fec_type may be 'swfec', in which case the
# session's fec_k/fec_n slots carry overhead_pct/deadline_ms.
CONTRACT_VERSIONS_SUPPORTED = frozenset({1, 2, 3})


class ContractVersionError(RuntimeError):
    """Raised when the wfb-ng feed advertises a contract_version we don't speak."""


@dataclass
class SettingsEvent:
    profile: str
    is_cluster: bool
    wlans: list[str]
    settings: dict
    timestamp: float = 0.0


@dataclass
class RxAnt:
    ant: int
    freq: int
    mcs: int
    bw: int
    pkt_recv: int
    rssi_min: int
    rssi_avg: int
    rssi_max: int
    snr_min: int
    snr_avg: int
    snr_max: int
    # |EVM| in dB magnitude (radiotap lock_quality, uncapped, higher = better).
    # Our wfb-ng now emits the dB magnitude on both ath9k and Realtek; older
    # builds emitted EVM% (0..100). Per spatial STREAM, not per antenna:
    # -1 = absent/unmeasured (e.g. the 2nd-stream sentinel slot on a
    # single-stream link). Default -1 for older wfb_rx feeds that don't
    # append the EVM fields.
    evm_min: int = -1
    evm_avg: int = -1
    evm_max: int = -1


@dataclass
class SessionInfo:
    """Session parameters from the wfb_rx SESSION record.

    For fec_type 'swfec' (contract v3), fec_k/fec_n carry
    overhead_pct/deadline_ms — the sliding-window codec has no block
    geometry. interleave_depth is a legacy v2 field; v3 feeds omit it
    (defaults to 1).
    """

    fec_type: str
    fec_k: int
    fec_n: int
    epoch: int
    interleave_depth: int
    contract_version: int


@dataclass
class RxEvent:
    """One 'rx' record — a 100 ms stats window from wfb_rx."""

    timestamp: float
    id: str
    # `packets` is {key: [window_count, cumulative_count]}; we keep window only.
    packets_window: dict[str, int] = field(default_factory=dict)
    rx_ant_stats: list[RxAnt] = field(default_factory=list)
    session: SessionInfo | None = None
    tx_wlan: int | None = None


@dataclass
class TxEvent:
    timestamp: float
    id: str
    packets_window: dict[str, int] = field(default_factory=dict)


@dataclass
class SessionEvent:
    """Emitted on-change and once per window (wfb_rx `SESSION`)."""

    timestamp: float
    id: str
    session: SessionInfo


Event = SettingsEvent | RxEvent | TxEvent | SessionEvent


def _parse_packets(d: dict) -> dict[str, int]:
    """Packets field: {key: [window, cumulative]} → {key: window}."""
    out: dict[str, int] = {}
    for k, v in d.items():
        if isinstance(v, (list, tuple)) and len(v) >= 1:
            out[k] = int(v[0])
        else:  # defensive — unknown shape
            out[k] = int(v)
    return out


def _parse_session(d: dict) -> SessionInfo:
    return SessionInfo(
        fec_type=str(d.get("fec_type", "")),
        fec_k=int(d["fec_k"]),
        fec_n=int(d["fec_n"]),
        epoch=int(d["epoch"]),
        # Vanilla wfb-ng omits this key. Default to 1 (no interleaver).
        interleave_depth=int(d.get("interleave_depth", 1)),
        contract_version=int(d["contract_version"]),
    )


def _parse_rx_ant(d: dict) -> RxAnt:
    return RxAnt(
        ant=int(d["ant"]),
        freq=int(d["freq"]),
        mcs=int(d["mcs"]),
        bw=int(d["bw"]),
        pkt_recv=int(d["pkt_recv"]),
        rssi_min=int(d["rssi_min"]),
        rssi_avg=int(d["rssi_avg"]),
        rssi_max=int(d["rssi_max"]),
        snr_min=int(d["snr_min"]),
        snr_avg=int(d["snr_avg"]),
        snr_max=int(d["snr_max"]),
        evm_min=int(d.get("evm_min", -1)),
        evm_avg=int(d.get("evm_avg", -1)),
        evm_max=int(d.get("evm_max", -1)),
    )


def parse_record(raw: dict) -> Event | None:
    """Turn one parsed JSON record into an Event. Returns None on unknown type."""
    rtype = raw.get("type")
    ts = float(raw.get("timestamp", 0.0))
    if rtype == "settings":
        return SettingsEvent(
            profile=str(raw.get("profile", "")),
            is_cluster=bool(raw.get("is_cluster", False)),
            wlans=list(raw.get("wlans", [])),
            settings=dict(raw.get("settings", {})),
            timestamp=ts,
        )
    if rtype == "rx":
        session = None
        if "session" in raw and raw["session"]:
            session = _parse_session(raw["session"])
            if session.contract_version not in CONTRACT_VERSIONS_SUPPORTED:
                raise ContractVersionError(
                    f"contract_version={session.contract_version}, "
                    f"supported {sorted(CONTRACT_VERSIONS_SUPPORTED)}"
                )
        return RxEvent(
            timestamp=ts,
            id=str(raw.get("id", "")),
            packets_window=_parse_packets(raw.get("packets", {})),
            rx_ant_stats=[_parse_rx_ant(a) for a in raw.get("rx_ant_stats", [])],
            session=session,
            tx_wlan=raw.get("tx_wlan"),
        )
    if rtype == "tx":
        return TxEvent(
            timestamp=ts,
            id=str(raw.get("id", "")),
            packets_window=_parse_packets(raw.get("packets", {})),
        )
    if rtype == "new_session":
        session = _parse_session(raw)
        if session.contract_version not in CONTRACT_VERSIONS_SUPPORTED:
            raise ContractVersionError(
                f"contract_version={session.contract_version}, "
                f"supported {sorted(CONTRACT_VERSIONS_SUPPORTED)}"
            )
        return SessionEvent(
            timestamp=ts,
            id=str(raw.get("id", "")),
            session=session,
        )
    log.debug("ignoring unknown record type: %r", rtype)
    return None


def _parse_endpoint(url: str) -> tuple[str, int]:
    """tcp://host:port → (host, port). Bare host:port is also accepted."""
    if "://" not in url:
        url = "tcp://" + url
    parsed = urlparse(url)
    if parsed.scheme != "tcp":
        raise ValueError(f"unsupported scheme {parsed.scheme!r} in {url!r}")
    if not parsed.hostname or not parsed.port:
        raise ValueError(f"endpoint must be tcp://host:port, got {url!r}")
    return parsed.hostname, parsed.port


class ReplayClient:
    """Read events from a captured JSONL file (same schema as the live feed).

    Used by `--replay` for offline validation. Yields events as fast as
    the consumer can process them (no pacing).
    """

    def __init__(
        self,
        path: str,
        on_event: Callable[[Event], Any],
    ) -> None:
        self._path = path
        self._on_event = on_event
        self._stop = asyncio.Event()

    def stop(self) -> None:
        self._stop.set()

    async def run(self) -> None:
        with open(self._path, "r") as fd:
            for line in fd:
                if self._stop.is_set():
                    break
                line = line.strip()
                if not line:
                    continue
                try:
                    raw = json.loads(line)
                except json.JSONDecodeError as e:
                    log.warning("replay: skipping malformed line: %s", e)
                    continue
                ev = parse_record(raw)
                if ev is None:
                    continue
                res = self._on_event(ev)
                if asyncio.iscoroutine(res):
                    await res
                await asyncio.sleep(0)
