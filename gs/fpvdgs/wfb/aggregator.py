"""In-process stats hub: per-child windows -> aggregate -> pub/sub.

Replaces wfb-ng's AntStatsAndSelector stats half + the :8103 TCP hop for
in-process consumers. Thread model: producers call update_* from the
engine loop thread; subscribers receive on THEIR own asyncio loop via
call_soon_threadsafe (dynlink + connmon each run one loop-in-a-thread).
"""

from __future__ import annotations

import asyncio
import collections
import logging
import threading
import time

from ..dynlink.stats_client import RxAnt, RxEvent, SessionEvent, SessionInfo, TxEvent

log = logging.getLogger("fpvdgs.wfb")


class WFBFlags:
    LINK_LOST = 1
    LINK_JAMMED = 2


def _to_session_info(ses: dict) -> SessionInfo:
    return SessionInfo(
        fec_type=ses["fec_type"],
        fec_k=ses["fec_k"],
        fec_n=ses["fec_n"],
        epoch=ses["epoch"],
        interleave_depth=1,
        contract_version=ses["contract_version"],
    )


def _to_rx_event(ts, child_id, packets, ant, session, tx_wlan) -> RxEvent:
    return RxEvent(
        timestamp=ts,
        id=child_id,
        packets_window={k: v[0] for k, v in packets.items()},
        rx_ant_stats=[
            RxAnt(
                ant=ant_id,
                freq=key[0],
                mcs=key[1],
                bw=key[2],
                pkt_recv=v[0],
                rssi_min=v[1],
                rssi_avg=v[2],
                rssi_max=v[3],
                snr_min=v[4],
                snr_avg=v[5],
                snr_max=v[6],
                evm_min=v[7],
                evm_avg=v[8],
                evm_max=v[9],
            )
            for (key, ant_id), v in ant.items()
        ],
        session=_to_session_info(session) if session else None,
        tx_wlan=tx_wlan,
    )


class _Subscription:
    def __init__(self, hub, loop, on_event):
        self._hub, self._loop, self._on_event = hub, loop, on_event
        self.queue = collections.deque(maxlen=256)  # drop-oldest backstop

    def _push(self, ev):
        # runs on the producer (engine) thread
        self.queue.append(ev)
        try:
            self._loop.call_soon_threadsafe(self._drain)
        except RuntimeError:
            self._hub.unsubscribe(self)  # consumer loop closed

    def _drain(self):
        while self.queue:
            self._on_event(self.queue.popleft())

    def close(self):
        self._hub.unsubscribe(self)


class StatsHub:
    """Aggregate RX/TX stats and select the TX antenna (in-process port of
    wfb-ng's AntStatsAndSelector).

    `update_*` are called from the engine loop thread as parser callbacks
    and fan out to subscribers immediately (one event per window), mirroring
    wfb-ng's `send_stats` behavior. `aggregate_window` is a *separate* pass,
    driven by the engine at `log_interval` cadence, that merges the windows
    accumulated since the last call, runs TX-antenna selection, and computes
    the mavlink RSSI 5-tuple.
    """

    def __init__(self, tx_selector, time_fn=time.time):
        self._txsel = tx_selector
        self._time = time_fn
        self._lock = threading.Lock()
        self._subs: list[_Subscription] = []
        self._rssi_cbs, self._ant_sel_cbs = [], []
        self._cur: dict[str, tuple] = {}  # child_id -> (ant, packets)
        self._sessions: dict[str, dict] = {}  # child_id -> last session
        # raw window mirrors for statsd (Task 9): child_id -> last raw payloads
        self.last_rx_raw: dict[str, tuple] = {}
        self.last_tx_raw: dict[str, tuple] = {}

    # -- producer side (engine loop thread) --------------------------------
    def update_rx_stats(self, child_id, packets, ant, session):
        with self._lock:
            self._cur[child_id] = (ant, packets)
            self.last_rx_raw[child_id] = (packets, ant, session)
            if session:
                self._sessions[child_id] = session
            tx_wlan = self._txsel.current
            subs = list(self._subs)
        ev = _to_rx_event(self._time(), child_id, packets, ant, session, tx_wlan)
        for sub in subs:
            sub._push(ev)

    def process_new_session(self, child_id, session):
        with self._lock:
            self._sessions[child_id] = session
            subs = list(self._subs)
        ev = SessionEvent(timestamp=self._time(), id=child_id, session=_to_session_info(session))
        for sub in subs:
            sub._push(ev)

    def update_tx_stats(self, child_id, packets, ant_latency):
        with self._lock:
            self.last_tx_raw[child_id] = (packets, ant_latency)
            subs = list(self._subs)
        ev = TxEvent(
            timestamp=self._time(),
            id=child_id,
            packets_window={k: v[0] for k, v in packets.items()},
        )
        for sub in subs:
            sub._push(ev)

    @staticmethod
    def _stats_agg_by_freq_and_rxid(ant_stats_by_rx: dict) -> dict[int, tuple]:
        """Pkt-weighted merge by ant_id across children — port of wfb-ng's
        AntStatsAndSelector._stats_agg_by_freq_and_rxid. Integer division
        (`//`) is intentional, matching upstream exactly."""
        agg: dict[int, tuple] = {}
        for ant_stats in ant_stats_by_rx.values():
            for (_key, ant_id), v in ant_stats.items():
                pkt_s, rssi_min, rssi_avg, rssi_max, snr_min, snr_avg, snr_max = v[:7]
                if ant_id not in agg:
                    agg[ant_id] = (
                        pkt_s,
                        rssi_min,
                        rssi_avg * pkt_s,
                        rssi_max,
                        snr_min,
                        snr_avg * pkt_s,
                        snr_max,
                    )
                else:
                    tmp = agg[ant_id]
                    agg[ant_id] = (
                        pkt_s + tmp[0],
                        min(rssi_min, tmp[1]),
                        rssi_avg * pkt_s + tmp[2],
                        max(rssi_max, tmp[3]),
                        min(snr_min, tmp[4]),
                        snr_avg * pkt_s + tmp[5],
                        max(snr_max, tmp[6]),
                    )
        return {
            ant_id: (
                pkt_s,
                rssi_min,
                rssi_avg // pkt_s,
                rssi_max,
                snr_min,
                snr_avg // pkt_s,
                snr_max,
            )
            for ant_id, (
                pkt_s,
                rssi_min,
                rssi_avg,
                rssi_max,
                snr_min,
                snr_avg,
                snr_max,
            ) in agg.items()
        }

    def aggregate_window(self) -> dict:
        with self._lock:
            cur, self._cur = self._cur, {}
            ant_sel_cbs = list(self._ant_sel_cbs)
            rssi_cbs = list(self._rssi_cbs)

        ant_by_rx = {cid: ant for cid, (ant, _pkts) in cur.items()}
        pkts_by_rx = {cid: pkts for cid, (_ant, pkts) in cur.items()}

        stats_agg = self._stats_agg_by_freq_and_rxid(ant_by_rx)
        # (rssi, noise) tuples — mavlink_err_rate=True fixed, window values ([0]).
        card_rssi_l = [
            (rssi_avg, rssi_avg - snr_avg)
            for _pkt_s, _rssi_min, rssi_avg, _rssi_max, _snr_min, snr_avg, _snr_max in (
                stats_agg.values()
            )
        ]

        tx_sel = None
        if stats_agg:
            tx_sel = self._txsel.select(stats_agg)
            if tx_sel is not None:
                for cb in ant_sel_cbs:
                    cb(tx_sel)

        flags = 0
        bad_packets = sum(p["dec_err"][0] + p["bad"][0] for p in pkts_by_rx.values())
        if not card_rssi_l:
            flags |= WFBFlags.LINK_LOST
            mav_rssi, mav_noise = -128, -128
        else:
            if bad_packets > 0:
                flags |= WFBFlags.LINK_JAMMED
            mav_rssi, mav_noise = max(card_rssi_l)

        rx_errors = min(
            sum(p["dec_err"][0] + p["bad"][0] + p["lost"][0] for p in pkts_by_rx.values()),
            65535,
        )
        rx_fec = min(sum(p["fec_rec"][0] for p in pkts_by_rx.values()), 65535)

        rssi = (mav_rssi, mav_noise, rx_errors, rx_fec, flags)
        for cb in rssi_cbs:
            cb(*rssi)

        return {"stats_agg": stats_agg, "rssi": rssi, "tx_sel": tx_sel}

    # -- consumer side (any thread) ----------------------------------------
    def subscribe(self, loop, on_event) -> _Subscription:
        sub = _Subscription(self, loop, on_event)
        with self._lock:
            self._subs.append(sub)
        return sub

    def unsubscribe(self, sub) -> None:
        with self._lock:
            try:
                self._subs.remove(sub)
            except ValueError:
                pass  # already removed — double-close is a harmless no-op

    def add_rssi_cb(self, cb) -> None:
        with self._lock:
            self._rssi_cbs.append(cb)

    def add_ant_sel_cb(self, cb) -> None:
        with self._lock:
            self._ant_sel_cbs.append(cb)
            current = self._txsel.current
        cb(current)  # wfb-ng contract: fire once at registration with the current value

    def client_factory(self):
        hub = self

        class NativeStatsSource:
            """StatsClient-compatible in-process source (endpoint ignored)."""

            def __init__(self, endpoint, on_event, **kw):
                self._on_event = on_event
                self._stop: asyncio.Event | None = None
                self._sub: _Subscription | None = None

            async def run(self):
                self._stop = asyncio.Event()
                loop = asyncio.get_running_loop()
                self._sub = hub.subscribe(loop, self._on_event)
                # replay current sessions so late subscribers see FEC params
                with hub._lock:
                    sessions = list(hub._sessions.items())
                for child_id, ses in sessions:
                    self._on_event(
                        SessionEvent(
                            timestamp=hub._time(), id=child_id, session=_to_session_info(ses)
                        )
                    )
                try:
                    await self._stop.wait()
                finally:
                    self._sub.close()

            def stop(self):
                if self._stop is not None:
                    self._stop.set()

        return NativeStatsSource
