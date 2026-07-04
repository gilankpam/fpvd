""":8103-compatible newline-JSON TCP stats server.

This replaces wfb-ng's `StatisticsJSONProtocol` (wfb_ng/protocols.py:72)
for GS-local consumers: the deploy health check (`ss -tln | grep 8103`),
the future `fpvd-stats` CLI, and (via a real TCP round trip, not just
in-process) the existing `StatsClient` contract test.

Design decision -- raw listener, not Event subscription
---------------------------------------------------------
`StatsHub.subscribe()` delivers already-parsed `RxEvent`/`TxEvent`
objects, but those only carry *window* values (see
`dynlink/stats_client.py`): the `packets` field collapses each
`[window, cumulative]` pair down to `window`, and there is no way to
recover the raw `((freq, mcs, bw), ant_id) -> 10-tuple` ant-stats dict
`send_stats()` flattens on the wire. Reconstructing the wire format from
an Event would therefore be lossy (cumulative counters permanently
lost) as well as redundant work (re-deriving keys the producer already
has).

Instead, `StatsServer` registers a *raw* listener via the new
`StatsHub.raw_listeners` hook (`aggregator.py`): `update_rx_stats`,
`update_tx_stats`, and `process_new_session` invoke every listener
synchronously, on the engine (producer) thread, with
`("rx"|"tx"|"new_session", child_id, raw_payload)` -- the exact
pre-flattening shapes those methods already receive. `StatsServer`
flattens them into JSON records using the same field order as
`StatisticsJSONProtocol.send_stats`.

This only works because `StatsServer` is constructed and driven on the
*same* asyncio loop the engine runs the hub's producer calls on (per
the task brief) -- unlike `subscribe()`, there is no
`call_soon_threadsafe` marshalling for raw listeners, so a
cross-thread consumer would corrupt asyncio state. If a future
consumer needs the raw feed from a different loop/thread, it should
get its own marshalling (mirroring `_Subscription`), not reuse this
hook directly.
"""

from __future__ import annotations

import asyncio
import json
import logging
import time
from typing import Callable

log = logging.getLogger("fpvdgs.wfb.statsd")

DEFAULT_MAX_WRITE_BUFFER = 256 * 1024

_RX_ANT_KEYS = ("ant", "freq", "mcs", "bw")
_RX_ANT_VALUE_KEYS = (
    "pkt_recv",
    "rssi_min",
    "rssi_avg",
    "rssi_max",
    "snr_min",
    "snr_avg",
    "snr_max",
    "evm_min",
    "evm_avg",
    "evm_max",
)
_TX_ANT_KEYS = ("ant",)
_TX_ANT_VALUE_KEYS = ("pkt_sent", "pkt_drop", "lat_min", "lat_avg", "lat_max")


def _flatten_rx_ant_stats(ant: dict) -> list[dict]:
    """`((freq, mcs, bw), ant_id) -> 10-tuple` -> wire-shaped dict list.

    Mirrors `StatisticsJSONProtocol.send_stats`'s
    `dict(zip(ka + va, (ant_id,) + k + v))` exactly.
    """
    keys = _RX_ANT_KEYS + _RX_ANT_VALUE_KEYS
    return [
        dict(zip(keys, (ant_id,) + key + tuple(values))) for (key, ant_id), values in ant.items()
    ]


def _flatten_tx_ant_stats(ant_latency: dict) -> list[dict]:
    keys = _TX_ANT_KEYS + _TX_ANT_VALUE_KEYS
    return [dict(zip(keys, (ant_id,) + tuple(values))) for ant_id, values in ant_latency.items()]


class StatsServer:
    """`:8103`-compatible newline-JSON TCP server, fed by a `StatsHub`."""

    def __init__(
        self,
        hub,
        settings_fn: Callable[[], dict],
        host: str = "127.0.0.1",
        port: int = 8103,
        *,
        max_write_buffer: int = DEFAULT_MAX_WRITE_BUFFER,
    ) -> None:
        self._hub = hub
        self._settings_fn = settings_fn
        self._host = host
        self._port = port
        self._max_write_buffer = max_write_buffer
        self._server: asyncio.base_events.Server | None = None
        self._writers: set[asyncio.StreamWriter] = set()

    @property
    def port(self) -> int:
        """Bound port -- resolves the ephemeral `0` after `start()`."""
        if self._server is not None and self._server.sockets:
            return self._server.sockets[0].getsockname()[1]
        return self._port

    async def start(self, loop: asyncio.AbstractEventLoop) -> None:
        self._hub.add_raw_listener(self._on_raw)
        self._server = await asyncio.start_server(self._handle_client, self._host, self._port)

    async def stop(self) -> None:
        self._hub.remove_raw_listener(self._on_raw)
        if self._server is not None:
            self._server.close()
            await self._server.wait_closed()
            self._server = None
        for writer in list(self._writers):
            writer.close()
        self._writers.clear()

    # -- client connection handling -----------------------------------------
    async def _handle_client(
        self, reader: asyncio.StreamReader, writer: asyncio.StreamWriter
    ) -> None:
        self._writers.add(writer)
        self._send(writer, self._settings_record())
        try:
            # Client writes are ignored (wfb-ng's `lineReceived: pass`); we
            # only read to notice disconnects (EOF / reset).
            while True:
                chunk = await reader.read(4096)
                if not chunk:
                    break
        except (ConnectionResetError, BrokenPipeError, OSError):
            pass
        finally:
            self._writers.discard(writer)
            writer.close()

    def _settings_record(self) -> dict:
        return {
            "type": "settings",
            "profile": "gs",
            "is_cluster": False,
            "wlans": [],
            "settings": self._settings_fn(),
        }

    # -- hub raw listener (engine loop thread) -------------------------------
    def _on_raw(self, kind: str, child_id: str, payload: tuple) -> None:
        if kind == "rx":
            packets, ant, session, tx_wlan = payload
            record = {
                "type": "rx",
                "timestamp": time.time(),
                "id": child_id,
                "tx_wlan": tx_wlan,
                "packets": packets,
                "rx_ant_stats": _flatten_rx_ant_stats(ant),
                "session": dict(session) if session else None,
            }
        elif kind == "tx":
            packets, ant_latency = payload
            record = {
                "type": "tx",
                "timestamp": time.time(),
                "id": child_id,
                "packets": packets,
                "tx_ant_stats": _flatten_tx_ant_stats(ant_latency),
                "rf_temperature": {},
            }
        elif kind == "new_session":
            (session,) = payload
            record = {
                "type": "new_session",
                "timestamp": time.time(),
                "id": child_id,
                **session,
            }
        else:
            log.debug("statsd: ignoring unknown raw event kind %r", kind)
            return
        self._broadcast(record)

    def _broadcast(self, record: dict) -> None:
        if not self._writers:
            return
        line = (json.dumps(record) + "\n").encode("utf-8")
        for writer in list(self._writers):
            self._send_line(writer, line)

    def _send(self, writer: asyncio.StreamWriter, record: dict) -> None:
        self._send_line(writer, (json.dumps(record) + "\n").encode("utf-8"))

    def _send_line(self, writer: asyncio.StreamWriter, line: bytes) -> None:
        transport = writer.transport
        if transport is not None and transport.get_write_buffer_size() > self._max_write_buffer:
            log.warning("statsd: slow client exceeded write buffer cap; dropping")
            self._writers.discard(writer)
            writer.close()
            return
        try:
            writer.write(line)
        except (ConnectionResetError, BrokenPipeError, OSError):
            self._writers.discard(writer)
