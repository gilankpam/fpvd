"""TX-antenna (card) selection — faithful port of wfb-ng's
AntStatsAndSelector.select_tx_antenna (wfb_ng/protocols.py).

Rank cards by RX packet count; among cards within counter deltas of the
best, pick the highest average RSSI; switch only on a >= rssi_delta_db
improvement (hysteresis). wlan id = ant_id >> 8.
"""

from __future__ import annotations

import logging
from dataclasses import dataclass
from itertools import groupby

log = logging.getLogger("fpvdgs.wfb")


@dataclass
class TxSelectorConfig:
    rssi_delta_db: int = 3
    counter_rel_delta: float = 0.1
    counter_abs_delta: int = 3


class TxSelector:
    def __init__(self, cfg: TxSelectorConfig, rx_only_wlan_ids=frozenset()):
        self.cfg = cfg
        self.rx_only_wlan_ids = frozenset(rx_only_wlan_ids)
        self.current: int | None = None

    def select(self, stats_agg: dict) -> int | None:
        """One selection pass over an aggregated window. Returns the new
        wlan id if a switch happened, else None."""
        wlan_rssi_and_pkts: dict[int, tuple[int, int]] = {}
        max_pkts = 0

        rows = sorted(
            (ant_id >> 8, v[0], v[2])  # (wlan, pkt_s, rssi_avg)
            for ant_id, v in stats_agg.items()
        )
        for wlan_id, grp in groupby(rows, lambda r: r[0]):
            if wlan_id in self.rx_only_wlan_ids:
                continue
            grp = list(grp)
            rssi = max(r for _, _, r in grp)
            pkts = max(p for _, p, _ in grp)
            max_pkts = max(pkts, max_pkts)
            wlan_rssi_and_pkts[wlan_id] = (rssi, pkts)

        if not wlan_rssi_and_pkts:
            return None

        thr = max_pkts - max(self.cfg.counter_abs_delta, max_pkts * self.cfg.counter_rel_delta)
        near_max = {w for w, (_, p) in wlan_rssi_and_pkts.items() if p >= thr}
        if not near_max:
            return None

        new_rssi, new_wlan = max(
            (rssi, w) for w, (rssi, _) in wlan_rssi_and_pkts.items() if w in near_max
        )
        cur_rssi = wlan_rssi_and_pkts.get(self.current, (-1000, 0))[0]

        if new_wlan == self.current:
            return None
        if self.current in near_max and new_rssi - cur_rssi < self.cfg.rssi_delta_db:
            return None

        log.info(
            "tx switch wlan %s -> %s, RSSI %d -> %d dB",
            self.current,
            new_wlan,
            cur_rssi,
            new_rssi,
        )
        self.current = new_wlan
        return new_wlan
