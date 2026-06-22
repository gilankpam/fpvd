"""GS-local learned link-RSSI → viable-ceiling-MCS prior (knee model; see
docs/superpowers/specs/2026-06-16-learned-prior-knee-model-design.md).

Per-rung RSSI knee: knee[K] = RSSI below which rung K is unviable in steady
state; recency-weighted; learns only from settled operating-rung samples.
The prior is an accelerant, never the authority — the live probe still gates
promotes; this only warm-starts the cold MCS and predictively demotes ahead
of a fade. Keyed (and persisted) per drone adapter id (radio.adapterId).
"""

from __future__ import annotations

import json
import logging
import os
import re
from dataclasses import dataclass

log = logging.getLogger("fpvdgs.dynlink")

MAX_MCS = 7  # rung ceiling (matches SelectorConfig.max_mcs default and the drone)


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
    settle_ticks: int = 5  # rung must be unchanged this many ticks to learn
    viable_loss: float = 0.05  # residual_loss_w below this = "clean"
    alpha_tighten: float = 0.25  # dirty -> raise knee (fast, pessimistic)
    alpha_relax: float = 0.05  # clean -> lower knee (slow)
    min_samples: float = 8.0  # confidence gate (decayed count)
    recency_decay: float = 0.9995  # per-settled-observation count decay
    # Predictive machinery (unchanged from the prior design)
    predictive_horizon_ticks: int = 3
    predictive_slope_window_ticks: int = 10
    predictive_min_drop_db: float = 1.0
    predictive_debounce_windows: int = 3
    flush_interval_observations: int = 50
    persist_dir: str = "/etc/fpvd/learned"


class KneeModel:
    """Per-rung viability knee, signal-agnostic (instantiated for RSSI and SNR).
    knee[K] = the signal value below which rung K is unviable in steady state;
    count[K] = decayed confidence. Monotone-in-rung on read (cumulative max).
    Caller feeds only settled samples."""

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
            if self._knee[K] is not None and self._count[K] >= self.cfg.min_samples:
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

    def rung_unviable(self, rung: int, value: float, margin: float = 0.0) -> bool:
        """True iff rung K is CONFIDENTLY unviable at `value` — its own knee is
        confident and `value` falls more than `margin` below it. A cold/unlearned
        rung returns False: unknown is not unviable, so the caller may still
        explore it. Distinct from `ceiling`, which answers "highest
        confidently-VIABLE rung"; a rung above that ceiling may simply be
        unmeasured, not known-bad.

        `margin` (dB) is hysteresis: the value must be CLEARLY below the knee to
        count as unviable. A zero-margin `value < knee` is a knife-edge — when
        the live signal settles a hair below a confident knee the rung is forever
        "unviable", and since the knee only relaxes by OPERATING there, the lock
        is self-perpetuating (the MCS-stuck field bug). Callers pass a smaller
        margin for the promote veto than for the proactive demote so the two
        gates form a stable dead-band instead of a single oscillating edge."""
        if rung < 0 or rung > MAX_MCS:
            return False
        k = self._eff_knees()[rung]
        return k is not None and value < k - margin

    def knees_snapshot(self) -> list:
        return [None if k is None else round(k, 1) for k in self._knee]

    def to_dict(self) -> dict:
        return {
            "schema": self.SCHEMA_VERSION,
            "knees": list(self._knee),
            "counts": list(self._count),
        }

    def load_dict(self, doc: dict) -> bool:
        if not isinstance(doc, dict):
            return False  # tolerant boundary: never boot-brick on a malformed file
        if doc.get("schema") != self.SCHEMA_VERSION:
            return False
        knees = doc.get("knees")
        counts = doc.get("counts")
        if (
            isinstance(knees, list)
            and len(knees) == MAX_MCS + 1
            and isinstance(counts, list)
            and len(counts) == MAX_MCS + 1
        ):
            self._knee = [None if k is None else float(k) for k in knees]
            self._count = [float(c) for c in counts]
            return True
        return False


class LearnedPrior:
    """Facade over KneeModel, keyed + persisted per drone adapter id (radio.adapterId). Keeps the
    interface policy.py depends on; the live probe stays authoritative for
    promotes — this only warm-starts and feeds the down-only predictive demote."""

    def __init__(self, key: str, cfg: LearnedPriorConfig) -> None:
        self.key = key
        self.cfg = cfg
        self._model = KneeModel(cfg)
        self._snr_model = KneeModel(cfg)
        self._since_flush = 0
        self._load()

    def ingest(self, *, rssi, snr=None, operating_mcs, operating_clean, settled) -> None:
        if operating_mcs is None or not settled:
            return
        m = int(operating_mcs)
        clean = bool(operating_clean)
        learned = False
        if rssi is not None:
            self._model.observe(m, float(rssi), clean)
            learned = True
        if snr is not None:
            self._snr_model.observe(m, float(snr), clean)
            learned = True
        if learned:
            self._since_flush += 1
            if self._since_flush >= self.cfg.flush_interval_observations:
                self.flush()
                self._since_flush = 0

    def ceiling(self, rssi) -> int | None:
        return None if rssi is None else self._model.ceiling(float(rssi))

    def predictive_ceiling(self, rssi, slope_dbm_per_tick) -> int | None:
        if rssi is None:
            return None
        projected = rssi + slope_dbm_per_tick * self.cfg.predictive_horizon_ticks
        return self._model.ceiling(projected)

    def warmstart_seed(self, rssi) -> int | None:
        return self.ceiling(rssi)

    def knees_snapshot(self) -> list:
        return self._model.knees_snapshot()

    def snr_ceiling(self, snr) -> int | None:
        return None if snr is None else self._snr_model.ceiling(float(snr))

    def snr_rung_unviable(self, target, snr, margin: float = 0.0) -> bool:
        """True iff the SNR prior CONFIDENTLY says rung `target` is unviable at
        `snr`, with `margin` dB of hysteresis. None/cold -> False (explorable).
        Gates the promote veto (on the target rung, small margin) and the
        proactive demote (on the current rung, larger margin); the asymmetric
        margins give a stable dead-band — see KneeModel.rung_unviable."""
        if target is None or snr is None:
            return False
        return self._snr_model.rung_unviable(int(target), float(snr), margin)

    def snr_knees_snapshot(self) -> list:
        return self._snr_model.knees_snapshot()

    def to_status(self) -> dict:
        return {"key": self.key, "knees": self._model.knees_snapshot()}

    def _path(self) -> str:
        safe = re.sub(r"[^A-Za-z0-9._-]", "_", self.key)
        return os.path.join(self.cfg.persist_dir, f"{safe}.json")

    def _load(self) -> None:
        try:
            with open(self._path()) as f:
                doc = json.load(f)
        except FileNotFoundError:
            return
        except (ValueError, OSError) as e:
            log.warning("learned_prior: ignoring unreadable %s: %s", self._path(), e)
            return
        # Back-compat: a v2 deploy persisted the flat rssi-model dict (no
        # "rssi"/"snr" wrapper). doc.get("rssi", doc) loads that as the rssi
        # model and leaves snr cold; a v3 combined doc loads both.
        if not self._model.load_dict(doc.get("rssi", doc)):
            log.info("learned_prior: %s rssi ignored (schema/shape) — retraining", self._path())
        snr_doc = doc.get("snr")
        if snr_doc is not None and not self._snr_model.load_dict(snr_doc):
            log.info("learned_prior: %s snr ignored (schema/shape) — retraining", self._path())

    def flush(self) -> None:
        doc = {"key": self.key, "rssi": self._model.to_dict(), "snr": self._snr_model.to_dict()}
        try:
            os.makedirs(self.cfg.persist_dir, exist_ok=True)
            tmp = self._path() + ".tmp"
            with open(tmp, "w") as f:
                json.dump(doc, f)
            os.replace(tmp, self._path())
        except OSError as e:
            log.warning("learned_prior: flush to %s failed: %s", self._path(), e)
