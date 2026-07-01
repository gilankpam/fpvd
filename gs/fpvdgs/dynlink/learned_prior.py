"""GS-local learned link-SNR → viable-ceiling-MCS prior (knee model; see
docs/superpowers/specs/2026-06-16-learned-prior-knee-model-design.md).

Per-rung SNR knee: knee[K] = the SNR at which rung K was seen to FAIL;
learned only from dirty (failed) settled samples — a clean sample never
plants or raises it. A rung that never fails stays None (explorable).
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

# Pre-bind sentinel key: until the drone's adapter id binds the real key at the
# connect event, the prior runs in-memory only (no load, no flush) so a cold
# start never leaves a stray <UNBOUND_KEY>.json on disk.
UNBOUND_KEY = "unbound"


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
    predictive_demote_margin_db: float = 1.5  # hysteresis on the cur-rung SNR knee
    flush_interval_observations: int = 50
    persist_dir: str = "/etc/fpvd/learned"


class KneeModel:
    """Per-rung viability knee, signal-agnostic (instantiated for RSSI and SNR).
    knee[K] = the signal value at which rung K was seen to FAIL; learned only
    from dirty (failed) settled samples — a clean sample never plants or raises
    it. A rung that never fails stays None (= explorable). count[K] = decayed
    confidence. Monotone-in-rung on read (cumulative max).
    Caller feeds only settled samples."""

    SCHEMA_VERSION = 3

    def __init__(self, cfg: LearnedPriorConfig) -> None:
        self.cfg = cfg
        self._knee: list[float | None] = [None] * (MAX_MCS + 1)
        self._count: list[float] = [0.0] * (MAX_MCS + 1)

    def observe(self, rung: int, value: float, clean: bool) -> None:
        if rung < 0 or rung > MAX_MCS:
            return
        d = self.cfg.recency_decay
        if d < 1.0:
            self._count = [c * d for c in self._count]
        k = self._knee[rung]
        if k is None:
            if not clean:  # establish the floor ONLY from a failure;
                self._knee[rung] = value  # a clean sample leaves it None (= explorable)
        elif clean and value < k:
            self._knee[rung] = k + self.cfg.alpha_relax * (value - k)
        elif (not clean) and value > k:
            self._knee[rung] = k + self.cfg.alpha_tighten * (value - k)
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
    """Facade over KneeModel (SNR axis only), keyed + persisted per drone adapter id
    (radio.adapterId). Keeps the interface policy.py depends on; the live probe stays
    authoritative for promotes — this only warm-starts and feeds the down-only
    predictive demote."""

    def __init__(self, key: str, cfg: LearnedPriorConfig) -> None:
        self.key = key
        self.cfg = cfg
        # The pre-bind sentinel is in-memory only — its learning (before the
        # drone curve binds) is discarded at the connect rekey.
        self._ephemeral = key == UNBOUND_KEY
        self._snr_model = KneeModel(cfg)
        self._since_flush = 0
        self._load()

    def ingest(self, *, snr=None, operating_mcs, operating_clean, settled) -> None:
        if operating_mcs is None or not settled or snr is None:
            return
        self._snr_model.observe(int(operating_mcs), float(snr), bool(operating_clean))
        self._since_flush += 1
        if self._since_flush >= self.cfg.flush_interval_observations:
            self.flush()
            self._since_flush = 0

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

    def snr_predictive_rung_unviable(
        self, snr, slope_db_per_tick, rung, margin: float = 0.0
    ) -> bool:
        """True iff the SNR prior CONFIDENTLY says `rung` is unviable at the
        PROJECTED SNR (snr + slope*horizon), with `margin` dB of hysteresis. A
        cold/unlearned rung -> False (explorable), mirroring snr_rung_unviable
        and the promote-veto. The predictive sibling of snr_rung_unviable:
        gating predict_demote on the CURRENT rung's own knee keeps cold rungs
        explorable, so a clean ladder no longer collapses to MCS0 on a fade."""
        if snr is None or rung is None:
            return False
        projected = snr + slope_db_per_tick * self.cfg.predictive_horizon_ticks
        return self._snr_model.rung_unviable(int(rung), projected, margin)

    def snr_knees_snapshot(self) -> list:
        return self._snr_model.knees_snapshot()

    def to_status(self) -> dict:
        return {"key": self.key, "knees": self._snr_model.knees_snapshot()}

    def _path(self) -> str:
        safe = re.sub(r"[^A-Za-z0-9._-]", "_", self.key)
        return os.path.join(self.cfg.persist_dir, f"{safe}.json")

    def _load(self) -> None:
        if self._ephemeral:
            return  # sentinel: never read a (stale) unbound.json
        try:
            with open(self._path()) as f:
                doc = json.load(f)
        except FileNotFoundError:
            return
        except (ValueError, OSError) as e:
            log.warning("learned_prior: ignoring unreadable %s: %s", self._path(), e)
            return
        # Old flat v2 rssi-model docs have no "snr" key → SNR stays cold (retrain).
        # Combined docs load the "snr" subkey; old rssi-only docs are ignored.
        snr_doc = doc.get("snr")
        if snr_doc is not None and not self._snr_model.load_dict(snr_doc):
            log.info("learned_prior: %s snr ignored (schema/shape) — retraining", self._path())

    def flush(self) -> None:
        if self._ephemeral:
            return  # sentinel: in-memory only, never write unbound.json
        doc = {"key": self.key, "snr": self._snr_model.to_dict()}
        try:
            os.makedirs(self.cfg.persist_dir, exist_ok=True)
            tmp = self._path() + ".tmp"
            with open(tmp, "w") as f:
                json.dump(doc, f)
            os.replace(tmp, self._path())
        except OSError as e:
            log.warning("learned_prior: flush to %s failed: %s", self._path(), e)
