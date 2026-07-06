"""In-process probe measurement for the native engine (2026-07-06 spec Part B).

The probe_rx child's stdout lines land here via WfbChild's pump (engine
loop thread); the dynlink controller snapshots per-MCS (PER, freshness)
each tick (its own thread) — hence the lock. Reuses parser.py's
parse_line + McsAggregator verbatim as the measurement layer.

Freshness is stamped ONLY by windows that carried packets (data+lost > 0):
a blackout keeps the aggregator's per=1.0 pin but decays to stale (neutral
gates) PROBE_FRESH_S after the last real packet — an indefinite blackout
veto would wedge promotes at one rung when the drone probe is off but the
GS probe is on (spec deviation 1; Part A's probation damper contains the
residual glitch).
"""

from __future__ import annotations

import threading
import time

from .parser import McsAggregator, parse_line

# Frozen calibration constants — no config path (config-cleanup convention).
PROBE_FRESH_S = 0.5  # consistent with the probe rx stats cadence (-l 100)
PROBE_EWMA_ALPHA = 0.25
PROBE_BLACKOUT_WINDOWS = 10


class ProbeFeed:
    def __init__(self, time_fn=time.monotonic):
        self._time = time_fn
        self._agg = McsAggregator(alpha=PROBE_EWMA_ALPHA, blackout_windows=PROBE_BLACKOUT_WINDOWS)
        self._last_mcs: int | None = None
        self._updated: dict[int, float] = {}
        self._lock = threading.Lock()

    def feed_line(self, line: str) -> None:
        rec = parse_line(line)
        if rec is None:
            return
        kind, d = rec
        with self._lock:
            if kind == "RX_ANT":
                self._last_mcs = d["mcs"]
                self._agg.on_rx_ant(d["mcs"], d["rssi"], d["snr"])
            elif kind == "PKT" and self._last_mcs is not None:
                self._agg.on_pkt(self._last_mcs, d["data"], d["lost"])
                if d["data"] + d["lost"] > 0:
                    self._updated[self._last_mcs] = self._time()

    def snapshot_fresh(self, now: float | None = None) -> dict[int, dict]:
        """{mcs: {per, snr, fresh}} for every rung with a PER estimate."""
        if now is None:
            now = self._time()
        with self._lock:
            snap = self._agg.snapshot()
            return {
                mcs: {
                    "per": s["per"],
                    "snr": s["snr"],
                    "fresh": (now - self._updated.get(mcs, float("-inf"))) <= PROBE_FRESH_S,
                }
                for mcs, s in snap.items()
                if s["per"] is not None
            }
