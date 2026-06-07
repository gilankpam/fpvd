"""GS-local learned link-RSSI -> viable-ceiling-MCS prior (Phase 4, spec §3-§7).

Binned viability table per (RSSI bin, MCS rung): an EWMA clean-rate + a
decaying sample count. A derived isotonic floor ladder extrapolates into
unflown RSSI. The prior is an accelerant, never the authority — the live
probe still gates promotes; this only warm-starts the cold MCS and
predictively demotes ahead of a fade. Keyed (and persisted) per radioProfile.
"""
from __future__ import annotations

import logging
import math
from dataclasses import dataclass

log = logging.getLogger("fpvdgs.dynlink")

MAX_MCS = 7   # rung ceiling (matches GateConfig.max_mcs default and the drone)


@dataclass
class LearnedPriorConfig:
    enabled: bool = True
    bin_width_db: float = 2.0
    rssi_min: float = -90.0
    rssi_max: float = -30.0
    ewma_alpha: float = 0.1
    viable_threshold: float = 0.99
    min_samples_warmstart: int = 20
    min_samples_predictive: int = 40
    warmstart_margin: int = 0
    predictive_horizon_ticks: int = 3
    predictive_debounce_windows: int = 3
    flush_interval_observations: int = 50
    persist_dir: str = "/etc/fpvd/learned"


class LearnedPrior:
    SCHEMA_VERSION = 1

    def __init__(self, key: str, cfg: LearnedPriorConfig) -> None:
        self.key = key
        self.cfg = cfg
        self._nbins = max(
            1, int(math.ceil((cfg.rssi_max - cfg.rssi_min) / cfg.bin_width_db))
        )
        # cells[b][rung] = [clean_ewma, n]; clean_ewma None until first sample.
        self._cells: list[list[list]] = [
            [[None, 0.0] for _ in range(MAX_MCS + 1)] for _ in range(self._nbins)
        ]
        self._since_flush = 0

    def rssi_bin(self, rssi) -> int | None:
        if rssi is None:
            return None
        if rssi < self.cfg.rssi_min or rssi >= self.cfg.rssi_max:
            return None
        return int((rssi - self.cfg.rssi_min) // self.cfg.bin_width_db)

    def _update(self, b: int, rung: int, clean: bool) -> None:
        if rung < 0 or rung > MAX_MCS:
            return
        cell = self._cells[b][rung]
        v = 1.0 if clean else 0.0
        cell[0] = v if cell[0] is None else (
            self.cfg.ewma_alpha * v + (1.0 - self.cfg.ewma_alpha) * cell[0]
        )
        cell[1] += 1.0

    def ingest(self, *, rssi, probed_rung, probe_clean,
               operating_mcs, operating_clean) -> None:
        b = self.rssi_bin(rssi)
        if b is None:
            return
        if probed_rung is not None:
            self._update(b, int(probed_rung), bool(probe_clean))
        if operating_mcs is not None:
            self._update(b, int(operating_mcs), bool(operating_clean))
        self._since_flush += 1
        if self._since_flush >= self.cfg.flush_interval_observations:
            self.flush()
            self._since_flush = 0

    def flush(self) -> None:
        pass  # persistence lands in Task 6

    def bin_ceiling(self, b: int) -> int | None:
        if b < 0 or b >= self._nbins:
            return None
        best = None
        for rung in range(MAX_MCS + 1):
            ewma, n = self._cells[b][rung]
            if (ewma is not None
                    and ewma >= self.cfg.viable_threshold
                    and n >= self.cfg.min_samples_warmstart):
                best = rung
        return best

    def _ladder(self) -> list[tuple[int, int]]:
        """Isotonic (monotone-increasing-in-RSSI) ladder over confident bins.
        Returns [(bin_index, ceiling), ...] ascending by bin; ceilings are
        made non-decreasing in RSSI (pool-adjacent-violators, simple form)."""
        pts = [(b, self.bin_ceiling(b)) for b in range(self._nbins)]
        pts = [(b, c) for b, c in pts if c is not None]
        if not pts:
            return []
        # Enforce non-decreasing ceiling as bin index (RSSI) rises: walk
        # ascending, clamp each ceiling up to the running max-so-far.
        out: list[tuple[int, int]] = []
        run = -1
        for b, c in pts:
            run = max(run, c)
            out.append((b, run))
        return out

    def ceiling(self, rssi) -> int | None:
        b = self.rssi_bin(rssi)
        if b is None:
            return None
        ladder = self._ladder()
        if not ladder:
            return None
        # Look up this bin in the ladder (which enforces monotonicity across
        # all confident bins). If the bin is a confident anchor it is already
        # in the ladder; for unflown bins extrapolate from the nearest lower
        # anchor, or from the lowest anchor when below all confident bins.
        best = None
        for lb, c in ladder:
            if lb <= b:
                best = c
            else:
                break
        return best if best is not None else ladder[0][1]

    def _confident_ceiling(self, rssi, min_samples) -> int | None:
        """ceiling(rssi) but gated on `min_samples` rather than the
        warmstart default. Resolves through the isotonic ladder at the
        stricter threshold (no direct-bin short-circuit)."""
        b = self.rssi_bin(rssi)
        if b is None:
            return None

        def bin_ceiling_at(bi: int) -> int | None:
            best = None
            for rung in range(MAX_MCS + 1):
                ewma, n = self._cells[bi][rung]
                if (ewma is not None and ewma >= self.cfg.viable_threshold
                        and n >= min_samples):
                    best = rung
            return best

        # Always resolve through the isotonic ladder (same fix as ceiling():
        # NO short-circuit on the direct bin — the value must be
        # monotonicity-corrected at the stricter threshold).
        pts = [(bi, bin_ceiling_at(bi)) for bi in range(self._nbins)]
        pts = [(bi, c) for bi, c in pts if c is not None]
        if not pts:
            return None
        run = -1
        ladder = []
        for bi, c in pts:
            run = max(run, c)
            ladder.append((bi, run))
        best = None
        for lb, c in ladder:
            if lb <= b:
                best = c
            else:
                break
        return best if best is not None else ladder[0][1]

    def warmstart_seed(self, rssi) -> int | None:
        c = self._confident_ceiling(rssi, self.cfg.min_samples_warmstart)
        if c is None:
            return None
        return max(0, min(MAX_MCS, c - self.cfg.warmstart_margin))

    def predictive_ceiling(self, rssi, slope_dbm_per_tick) -> int | None:
        if rssi is None:
            return None
        projected = rssi + slope_dbm_per_tick * self.cfg.predictive_horizon_ticks
        return self._confident_ceiling(projected, self.cfg.min_samples_predictive)
