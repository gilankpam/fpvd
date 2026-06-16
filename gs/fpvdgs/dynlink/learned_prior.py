"""GS-local learned link-RSSI -> viable-ceiling-MCS prior (Phase 4, spec §3-§7).

Binned viability table per (RSSI bin, MCS rung): an EWMA clean-rate + a
decaying sample count. A derived isotonic floor ladder extrapolates into
unflown RSSI. The prior is an accelerant, never the authority — the live
probe still gates promotes; this only warm-starts the cold MCS and
predictively demotes ahead of a fade. Keyed (and persisted) per radioProfile.
"""
from __future__ import annotations

import json
import logging
import math
import os
import re
from dataclasses import dataclass

log = logging.getLogger("fpvdgs.dynlink")

MAX_MCS = 7   # rung ceiling (matches SelectorConfig.max_mcs default and the drone)


def lsq_slope(samples) -> float:
    """Least-squares gradient (dBm per tick) over an evenly-spaced sample
    sequence (x = 0, 1, ..., n-1). Unlike a single-tick delta, a lone spike
    barely moves the fit. Fewer than 2 samples → 0.0 (no trend yet)."""
    n = len(samples)
    if n < 2:
        return 0.0
    mean_x = (n - 1) / 2.0
    mean_y = sum(samples) / n
    num = sum((i - mean_x) * (y - mean_y) for i, y in enumerate(samples))
    den = sum((i - mean_x) ** 2 for i in range(n))
    return num / den if den else 0.0


@dataclass
class LearnedPriorConfig:
    # Learning (knee model)
    settle_ticks: int = 5           # rung must be unchanged this many ticks to learn
    viable_loss: float = 0.05       # residual_loss_w below this = "clean"
    alpha_tighten: float = 0.25     # dirty -> raise knee (fast, pessimistic)
    alpha_relax: float = 0.05       # clean -> lower knee (slow)
    min_samples: float = 8.0        # confidence gate (decayed count)
    recency_decay: float = 0.9995   # per-settled-observation count decay
    # Predictive machinery (unchanged from the prior design)
    predictive_horizon_ticks: int = 3
    predictive_slope_window_ticks: int = 10
    predictive_min_drop_db: float = 1.0
    predictive_debounce_windows: int = 3
    flush_interval_observations: int = 50
    persist_dir: str = "/etc/fpvd/learned"


class KneeModel:
    """Per-rung RSSI knee. knee[K] = RSSI below which rung K is unviable in
    steady state; count[K] = decayed confidence. Monotone-in-rung on read
    (cumulative max). Caller feeds only settled samples."""

    SCHEMA_VERSION = 2

    def __init__(self, cfg: LearnedPriorConfig) -> None:
        self.cfg = cfg
        self._knee: list[float | None] = [None] * (MAX_MCS + 1)
        self._count: list[float] = [0.0] * (MAX_MCS + 1)

    def observe(self, rung: int, rssi: float, clean: bool) -> None:
        if rung < 0 or rung > MAX_MCS:
            return
        d = self.cfg.recency_decay
        if d < 1.0:
            self._count = [c * d for c in self._count]
        k = self._knee[rung]
        if k is None:
            self._knee[rung] = rssi
        elif clean and rssi < k:
            self._knee[rung] = k + self.cfg.alpha_relax * (rssi - k)
        elif (not clean) and rssi > k:
            self._knee[rung] = k + self.cfg.alpha_tighten * (rssi - k)
        self._count[rung] += 1.0

    def _eff_knees(self) -> list[float | None]:
        """Confident knees made non-decreasing in rung (cumulative max)."""
        eff: list[float | None] = [None] * (MAX_MCS + 1)
        run: float | None = None
        for K in range(MAX_MCS + 1):
            if (self._knee[K] is not None
                    and self._count[K] >= self.cfg.min_samples):
                run = self._knee[K] if run is None else max(run, self._knee[K])
                eff[K] = run
        return eff

    def ceiling(self, rssi: float) -> int | None:
        eff = self._eff_knees()
        best = None
        for K in range(MAX_MCS + 1):
            if eff[K] is not None and eff[K] <= rssi:
                best = K
        return best

    def knees_snapshot(self) -> list:
        return [None if k is None else round(k, 1) for k in self._knee]

    def to_dict(self) -> dict:
        return {"schema": self.SCHEMA_VERSION,
                "knees": list(self._knee), "counts": list(self._count)}

    def load_dict(self, doc: dict) -> bool:
        if doc.get("schema") != self.SCHEMA_VERSION:
            return False
        knees = doc.get("knees")
        counts = doc.get("counts")
        if (isinstance(knees, list) and len(knees) == MAX_MCS + 1
                and isinstance(counts, list) and len(counts) == MAX_MCS + 1):
            self._knee = [None if k is None else float(k) for k in knees]
            self._count = [float(c) for c in counts]
            return True
        return False
