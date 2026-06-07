# Phase 4 — Learned RSSI→Ceiling Prior + Flight Logging Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a GS-local, per-card learned `link-RSSI → viable-ceiling-MCS` prior (binned viability table + derived isotonic floor ladder) that warm-starts the operating MCS and predictively demotes ahead of an RSSI fade, plus structured per-flight JSONL logging and an offline analysis script.

**Architecture:** Pure GS-side addition layered onto the Phase-2 `LeadingSelector`. A new `LearnedPrior` (model + persistence) and `FlightLog` (per-tick JSONL), both owned by `Policy`. `Policy.tick()` ingests one observation per tick, replaces the hand-authored cold-start seed with a confidence-gated warm-start, applies a confidence-gated down-only predictive demote (mirroring the existing cold-start `current_mcs` write), and appends a flight-log record. Persisted per `radioProfile` (= `profile.name`). The live probe stays authoritative; the reactive Channel-B demote is unchanged; everything degrades to today's behavior when the curve is unknown/unconfident. Wire (`{mcs}`-only) and the drone are untouched.

**Tech Stack:** Python 3.13 + pytest (GS). No new runtime deps on the GS (plain JSON). The offline analysis tool may use `matplotlib` (optional, degrades to text).

**Spec:** `docs/superpowers/specs/2026-06-07-phase4-learned-rssi-prior-design.md`.

**Test command:** `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/ -q` (use the venv — a bare `python` lacks pytest). One file: `… tests/unit/test_dl_learned_prior.py -q`. **Baseline: 211 passed.** Git from repo root `/home/gilankpam/Projects/drone/fpvd`.

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `gs/fpvdgs/dynlink/learned_prior.py` | **create** | `LearnedPriorConfig`; `LearnedPrior` (binned table, ingest, ceiling/confidence, isotonic ladder, warm-start/predictive queries, JSON persistence, `to_status`) |
| `gs/fpvdgs/dynlink/flightlog.py` | **create** | `FlightLogConfig`; `FlightLog` (per-session JSONL open/write/close, rotation + size cap, disabled no-op) |
| `gs/fpvdgs/dynlink/policy.py` | modify | add `learned_prior`/`flightlog` to `PolicyConfig`; `Policy` constructs both; `tick()` ingests + warm-starts + predictive-demotes + logs; `Policy.close()` |
| `gs/fpvdgs/dynlink/config_build.py` | modify | build `LearnedPriorConfig` + `FlightLogConfig` from `tuning.learned_prior`; attach to `PolicyConfig` |
| `gs/fpvdgs/dynlink/controller.py` | modify | call `policy.close()` in `_run()`'s `finally` |
| `gs/fpvdgs/supervisor.py` | modify | opportunistic adapter_id↔radioProfile mismatch warning (best-effort, no new fetch) |
| `gs/tools/flightlog_analyze.py` | **create** | offline analysis script (timeline + curve dump + summary; matplotlib optional) |
| `gs/tests/unit/test_dl_learned_prior.py` | **create** | engine unit tests |
| `gs/tests/unit/test_dl_flightlog.py` | **create** | logger unit tests |
| `gs/tests/unit/test_dl_policy_learned.py` | **create** | Policy integration (warm-start, predictive, regression) |
| `gs/tests/unit/test_dl_flightlog_analyze.py` | **create** | analysis-script smoke test |
| `gs/tests/unit/test_dl_config_build.py` | modify | assert new config knobs parse |
| `gs/tests/unit/test_dl_imports.py` | modify | add `learned_prior`, `flightlog` to `MODULES` |

**Canonical API (locked here so every task is consistent):**

```python
# learned_prior.py
@dataclass
class LearnedPriorConfig:
    enabled: bool = True
    bin_width_db: float = 2.0
    rssi_min: float = -90.0
    rssi_max: float = -30.0
    ewma_alpha: float = 0.1
    viable_threshold: float = 0.99          # clean if clean_ewma >= this
    min_samples_warmstart: int = 20
    min_samples_predictive: int = 40        # stricter than warmstart
    warmstart_margin: int = 0               # seed = ceiling - margin
    predictive_horizon_ticks: int = 3       # RSSI projection lookahead
    predictive_debounce_windows: int = 3
    flush_interval_observations: int = 50
    persist_dir: str = "/etc/fpvd/learned"

class LearnedPrior:
    SCHEMA_VERSION = 1
    def __init__(self, key: str, cfg: LearnedPriorConfig) -> None: ...
    def rssi_bin(self, rssi: float) -> int | None        # bin index, None if out of range
    def ingest(self, *, rssi, probed_rung, probe_clean,
               operating_mcs, operating_clean) -> None    # update cells; periodic flush
    def bin_ceiling(self, b: int) -> int | None           # confident ceiling for a bin index
    def ceiling(self, rssi: float) -> int | None          # confident bin → ladder → None
    def warmstart_seed(self, rssi) -> int | None          # ceiling if n>=min_samples_warmstart
    def predictive_ceiling(self, rssi, slope_dbm_per_tick) # ceiling at projected rssi if
                            -> int | None                 #   n>=min_samples_predictive
    def flush(self) -> None                                # atomic write
    def to_status(self) -> dict                            # compact dump for /status
    # MCS rungs handled: 0..7

# flightlog.py
@dataclass
class FlightLogConfig:
    enabled: bool = True
    dir: str = "/etc/fpvd/flightlog"
    max_files: int = 8
    max_mb: float = 4.0

class FlightLog:
    def __init__(self, cfg: FlightLogConfig, *, start_ms: int) -> None: ...
    def write(self, record: dict) -> None      # append one JSON line; no-op if disabled
    def close(self) -> None                     # flush + close; enforce rotation/cap
```

`MAX_MCS = 7` is the rung ceiling everywhere (matches `GateConfig.max_mcs` default and the drone). Order: Tasks 1→12. Each task is one commit; the GS suite stays green at every commit.

---

# Part A — The `LearnedPrior` engine

## Task 1: Config + binning + empty-store skeleton

**Files:** Create `gs/fpvdgs/dynlink/learned_prior.py`; Test `gs/tests/unit/test_dl_learned_prior.py`.

- [ ] **Step 1: Write the failing test**

```python
# gs/tests/unit/test_dl_learned_prior.py
from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig


def _prior(tmp_path, **kw):
    cfg = LearnedPriorConfig(persist_dir=str(tmp_path), **kw)
    return LearnedPrior("m8812eu2", cfg)


def test_rssi_bin_maps_and_rejects_out_of_range(tmp_path):
    p = _prior(tmp_path, bin_width_db=2.0, rssi_min=-90.0, rssi_max=-30.0)
    # -90 is the first bin; -30 is the last edge.
    assert p.rssi_bin(-90.0) == 0
    assert p.rssi_bin(-89.0) == 0
    assert p.rssi_bin(-88.0) == 1
    assert p.rssi_bin(-50.0) == 20
    # out of range / missing → None (not ingested, query unknown)
    assert p.rssi_bin(-91.0) is None
    assert p.rssi_bin(-29.0) is None
    assert p.rssi_bin(None) is None


def test_empty_store_returns_unknown(tmp_path):
    p = _prior(tmp_path)
    assert p.ceiling(-50.0) is None
    assert p.warmstart_seed(-50.0) is None
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: FAIL — `ModuleNotFoundError: fpvdgs.dynlink.learned_prior`.

- [ ] **Step 3: Write minimal implementation**

```python
# gs/fpvdgs/dynlink/learned_prior.py
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

    def bin_ceiling(self, b: int) -> int | None:
        return None  # filled in Task 3

    def ceiling(self, rssi) -> int | None:
        return None  # filled in Task 4

    def warmstart_seed(self, rssi) -> int | None:
        return None  # filled in Task 5
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: PASS (2 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "feat(gs/dynlink): LearnedPrior config + RSSI binning skeleton (Phase 4)"
```

---

## Task 2: Ingest — clean-EWMA + decaying count

**Files:** Modify `gs/fpvdgs/dynlink/learned_prior.py`; Test `gs/tests/unit/test_dl_learned_prior.py`.

The ingest records two boundary observations per tick (spec §4): the probe rung verdict and the operating-rung health. Each updates a cell's `clean_ewma` and increments `n`.

- [ ] **Step 1: Write the failing test**

```python
def test_ingest_raises_clean_and_counts(tmp_path):
    p = _prior(tmp_path, ewma_alpha=0.5, rssi_min=-90.0, rssi_max=-30.0)
    b = p.rssi_bin(-50.0)
    # 3 clean observations of rung 5 at this bin.
    for _ in range(3):
        p.ingest(rssi=-50.0, probed_rung=5, probe_clean=True,
                 operating_mcs=4, operating_clean=True)
    cell5 = p._cells[b][5]
    assert cell5[1] == 3                 # n
    assert cell5[0] is not None and cell5[0] > 0.8   # clean_ewma rose toward 1
    # operating rung 4 also got clean labels.
    assert p._cells[b][4][1] == 3
    assert p._cells[b][4][0] > 0.8


def test_ingest_cliff_lowers_clean(tmp_path):
    p = _prior(tmp_path, ewma_alpha=0.5)
    b = p.rssi_bin(-50.0)
    for _ in range(5):
        p.ingest(rssi=-50.0, probed_rung=6, probe_clean=True,
                 operating_mcs=5, operating_clean=True)
    high = p._cells[b][6][0]
    # now rung 6 cliffs repeatedly
    for _ in range(5):
        p.ingest(rssi=-50.0, probed_rung=6, probe_clean=False,
                 operating_mcs=5, operating_clean=True)
    assert p._cells[b][6][0] < high      # clean_ewma fell


def test_ingest_skips_out_of_range_rssi(tmp_path):
    p = _prior(tmp_path)
    p.ingest(rssi=-200.0, probed_rung=3, probe_clean=True,
             operating_mcs=2, operating_clean=True)
    # nothing recorded
    assert all(cell[1] == 0 for row in p._cells for cell in row)
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: FAIL — `AttributeError: 'LearnedPrior' object has no attribute 'ingest'`.

- [ ] **Step 3: Write the implementation** — add to `LearnedPrior`:

```python
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
```

(The decaying-count refinement is intentionally a plain increment here; recency lives in the EWMA. `n` is a float so Task 6 can persist/scale it without type churn.)

- [ ] **Step 4: Run to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: PASS (5 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "feat(gs/dynlink): LearnedPrior ingest (clean-EWMA + counts)"
```

---

## Task 3: `bin_ceiling` — highest confidently-clean rung

**Files:** Modify `gs/fpvdgs/dynlink/learned_prior.py`; Test `gs/tests/unit/test_dl_learned_prior.py`.

`bin_ceiling(b)` = highest rung with `clean_ewma >= viable_threshold` AND `n >= min_samples_warmstart`; `None` if none qualify. (Rung-monotonicity, spec §3: a clean high rung implies all lower rungs viable, so the highest qualifying rung is the ceiling.)

- [ ] **Step 1: Write the failing test**

```python
def test_bin_ceiling_picks_highest_confident_clean_rung(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, viable_threshold=0.99,
               min_samples_warmstart=3)
    b = p.rssi_bin(-50.0)
    # rungs 0..4 clean, rung 5 cliffed — all with enough samples.
    for _ in range(3):
        for rung in (0, 1, 2, 3, 4):
            p.ingest(rssi=-50.0, probed_rung=rung, probe_clean=True,
                     operating_mcs=rung, operating_clean=True)
        p.ingest(rssi=-50.0, probed_rung=5, probe_clean=False,
                 operating_mcs=4, operating_clean=True)
    assert p.bin_ceiling(b) == 4


def test_bin_ceiling_unknown_until_min_samples(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, viable_threshold=0.99,
               min_samples_warmstart=5)
    b = p.rssi_bin(-50.0)
    for _ in range(2):  # only 2 < 5 samples
        p.ingest(rssi=-50.0, probed_rung=3, probe_clean=True,
                 operating_mcs=3, operating_clean=True)
    assert p.bin_ceiling(b) is None
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py::test_bin_ceiling_picks_highest_confident_clean_rung -q`
Expected: FAIL — returns `None` (stub).

- [ ] **Step 3: Write the implementation** — replace `bin_ceiling`:

```python
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
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: PASS (7 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "feat(gs/dynlink): LearnedPrior bin_ceiling (highest confident clean rung)"
```

---

## Task 4: Derived isotonic floor ladder + `ceiling(rssi)`

**Files:** Modify `gs/fpvdgs/dynlink/learned_prior.py`; Test `gs/tests/unit/test_dl_learned_prior.py`.

`ceiling(rssi)` resolves: confident bin → its `bin_ceiling`; else a derived **isotonic** (monotone-in-RSSI) ladder over all confident bins → the ladder value for this RSSI; else `None`. The isotonic step both denoises (higher RSSI ⇒ ≥ ceiling) and extrapolates into unflown bins between confident ones.

- [ ] **Step 1: Write the failing test**

```python
def _fill_bin(p, rssi, ceiling, samples=5):
    """Make bin(rssi) report `ceiling`: rungs 0..ceiling clean, ceiling+1 cliff."""
    for _ in range(samples):
        for rung in range(ceiling + 1):
            p.ingest(rssi=rssi, probed_rung=rung, probe_clean=True,
                     operating_mcs=rung, operating_clean=True)
        if ceiling + 1 <= 7:
            p.ingest(rssi=rssi, probed_rung=ceiling + 1, probe_clean=False,
                     operating_mcs=ceiling, operating_clean=True)


def test_ceiling_uses_confident_bin_directly(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3)
    _fill_bin(p, -50.0, ceiling=5)
    assert p.ceiling(-50.0) == 5


def test_ceiling_ladder_extrapolates_unflown_bin_monotonically(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3)
    _fill_bin(p, -70.0, ceiling=2)   # weak RSSI bin
    _fill_bin(p, -50.0, ceiling=5)   # strong RSSI bin
    # -60 was never flown; the isotonic ladder must give a value between
    # the two anchors and never below the weaker / above the stronger.
    mid = p.ceiling(-60.0)
    assert mid is not None and 2 <= mid <= 5


def test_ceiling_isotonic_denoises_inversion(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3)
    _fill_bin(p, -70.0, ceiling=5)   # noisy: weak RSSI shows a high ceiling
    _fill_bin(p, -50.0, ceiling=3)   # strong RSSI shows a lower one
    # Monotonicity (more RSSI ⇒ >= ceiling) must hold after the isotonic fit.
    assert p.ceiling(-50.0) >= p.ceiling(-70.0)


def test_ceiling_unknown_with_no_confident_bins(tmp_path):
    p = _prior(tmp_path, min_samples_warmstart=100)
    _fill_bin(p, -50.0, ceiling=5, samples=3)   # below threshold
    assert p.ceiling(-50.0) is None
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -k ceiling -q`
Expected: FAIL — `ceiling` returns `None` (stub).

- [ ] **Step 3: Write the implementation** — replace `ceiling` and add the ladder helper:

```python
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
        # Always resolve through the isotonic ladder (NO short-circuit on the
        # direct bin): a confident bin's value must be monotonicity-corrected
        # against higher-RSSI bins (a noisy low-RSSI bin claiming a high
        # ceiling). For a confident bin with no inversion the ladder value
        # equals its own bin_ceiling, so the "confident bin → its ceiling"
        # semantic still holds in the common case.
        ladder = self._ladder()
        if not ladder:
            return None
        # Extrapolate: highest confident bin <= b, else the lowest anchor.
        best = None
        for lb, c in ladder:
            if lb <= b:
                best = c
            else:
                break
        return best if best is not None else ladder[0][1]
```

(Note: do NOT short-circuit on `bin_ceiling(b)` before the ladder — that returns the raw, un-monotonicized value and fails `test_ceiling_isotonic_denoises_inversion`. Always resolve through `_ladder()`.)

- [ ] **Step 4: Run to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: PASS (11 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "feat(gs/dynlink): LearnedPrior isotonic floor ladder + ceiling(rssi)"
```

---

## Task 5: Confidence-gated queries — `warmstart_seed` + `predictive_ceiling`

**Files:** Modify `gs/fpvdgs/dynlink/learned_prior.py`; Test `gs/tests/unit/test_dl_learned_prior.py`.

`warmstart_seed(rssi)` = `ceiling(rssi) - warmstart_margin` clamped to `[0, MAX_MCS]`, but only when the bin clears `min_samples_warmstart` (which `bin_ceiling`/`ceiling` already enforce). `predictive_ceiling(rssi, slope)` projects RSSI forward `predictive_horizon_ticks` and returns the ceiling there, gated by the stricter `min_samples_predictive`.

- [ ] **Step 1: Write the failing test**

```python
def test_warmstart_seed_applies_margin_and_clamp(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
               warmstart_margin=1)
    _fill_bin(p, -50.0, ceiling=5)
    assert p.warmstart_seed(-50.0) == 4          # 5 - margin(1)
    assert p.warmstart_seed(-91.0) is None        # out of range


def test_warmstart_seed_none_when_unconfident(tmp_path):
    p = _prior(tmp_path, min_samples_warmstart=100)
    _fill_bin(p, -50.0, ceiling=5, samples=3)
    assert p.warmstart_seed(-50.0) is None


def test_predictive_ceiling_projects_and_gates_on_strict_confidence(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
               min_samples_predictive=3, predictive_horizon_ticks=2,
               bin_width_db=2.0)
    _fill_bin(p, -50.0, ceiling=5)   # where we are now
    _fill_bin(p, -56.0, ceiling=2)   # where a -3 dB/tick fade lands in 2 ticks
    # slope -3 dB/tick, horizon 2 -> projected ≈ -50 + (-3*2) = -56 -> ceiling 2
    assert p.predictive_ceiling(-50.0, -3.0) == 2


def test_predictive_ceiling_needs_strict_min_samples(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
               min_samples_predictive=100, predictive_horizon_ticks=2)
    _fill_bin(p, -56.0, ceiling=2, samples=5)   # confident for warmstart, not predictive
    assert p.predictive_ceiling(-50.0, -3.0) is None
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -k "warmstart or predictive" -q`
Expected: FAIL — `warmstart_seed` returns `None` (stub) / no `predictive_ceiling`.

- [ ] **Step 3: Write the implementation** — replace `warmstart_seed`, add `predictive_ceiling` + a confidence-parameterized internal:

```python
    def _confident_ceiling(self, rssi, min_samples) -> int | None:
        """ceiling(rssi) but gated on `min_samples` rather than the
        warmstart default. Temporarily evaluates bins against the stricter
        threshold by reusing the same logic with an override."""
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
```

(Note: `ceiling()` from Task 4 stays as the warmstart-threshold public query; `_confident_ceiling` generalizes it so predictive can use the stricter `min_samples_predictive`. The duplication is deliberate and local — keeping `ceiling()` simple for `to_status`/debugging.)

- [ ] **Step 4: Run to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: PASS (15 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "feat(gs/dynlink): LearnedPrior warmstart_seed + predictive_ceiling (confidence-gated)"
```

---

## Task 6: Persistence — atomic load/flush, schema/bin versioning, `to_status`

**Files:** Modify `gs/fpvdgs/dynlink/learned_prior.py`; Test `gs/tests/unit/test_dl_learned_prior.py`.

Load on construct, flush atomically (temp + `os.replace`), discard a file whose schema version or bin config doesn't match, treat a corrupt file as empty (log + continue). `to_status()` returns a compact dump for `/status`.

- [ ] **Step 1: Write the failing test**

```python
import json


def test_persistence_round_trip(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3)
    _fill_bin(p, -50.0, ceiling=5)
    p.flush()
    # a fresh prior with the same key loads the persisted curve
    p2 = LearnedPrior("m8812eu2", LearnedPriorConfig(
        persist_dir=str(tmp_path), ewma_alpha=1.0, min_samples_warmstart=3))
    assert p2.ceiling(-50.0) == 5


def test_persistence_bin_config_mismatch_discarded(tmp_path):
    p = _prior(tmp_path, bin_width_db=2.0, min_samples_warmstart=3,
               ewma_alpha=1.0)
    _fill_bin(p, -50.0, ceiling=5)
    p.flush()
    # different bin width → stale file ignored, starts empty
    p2 = LearnedPrior("m8812eu2", LearnedPriorConfig(
        persist_dir=str(tmp_path), bin_width_db=4.0, min_samples_warmstart=3))
    assert p2.ceiling(-50.0) is None


def test_corrupt_file_is_ignored(tmp_path):
    (tmp_path / "m8812eu2.json").write_text("{not json")
    p = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path)))
    assert p.ceiling(-50.0) is None      # no crash, empty


def test_key_sanitized_in_filename(tmp_path):
    p = LearnedPrior("bl-m8812eu2/weird", LearnedPriorConfig(persist_dir=str(tmp_path)))
    p.flush()
    files = list(tmp_path.iterdir())
    assert len(files) == 1 and "/" not in files[0].name


def test_to_status_shape(tmp_path):
    p = _prior(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3)
    _fill_bin(p, -50.0, ceiling=5)
    st = p.to_status()
    assert st["key"] == "m8812eu2"
    assert any(entry["ceiling"] == 5 for entry in st["bins"])
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -k "persistence or corrupt or sanitized or to_status" -q`
Expected: FAIL — `flush` is a no-op / no `to_status`; round-trip empty.

- [ ] **Step 3: Write the implementation** — add imports and methods, replace `flush`:

```python
import json
import os
import re
```

```python
    def _path(self) -> str:
        safe = re.sub(r"[^A-Za-z0-9._-]", "_", self.key)
        return os.path.join(self.cfg.persist_dir, f"{safe}.json")

    def _bin_sig(self) -> list:
        return [self.cfg.bin_width_db, self.cfg.rssi_min, self.cfg.rssi_max]

    def _load(self) -> None:
        path = self._path()
        try:
            with open(path) as f:
                doc = json.load(f)
        except FileNotFoundError:
            return
        except (ValueError, OSError) as e:
            log.warning("learned_prior: ignoring unreadable %s: %s", path, e)
            return
        if (doc.get("schema") != self.SCHEMA_VERSION
                or doc.get("bins") != self._bin_sig()):
            log.info("learned_prior: %s schema/bin mismatch — rebuilding", path)
            return
        cells = doc.get("cells")
        if (isinstance(cells, list) and len(cells) == self._nbins):
            self._cells = [
                [[c[0], float(c[1])] for c in row] for row in cells
            ]

    def flush(self) -> None:
        path = self._path()
        doc = {"schema": self.SCHEMA_VERSION, "bins": self._bin_sig(),
               "key": self.key, "cells": self._cells}
        try:
            os.makedirs(self.cfg.persist_dir, exist_ok=True)
            tmp = path + ".tmp"
            with open(tmp, "w") as f:
                json.dump(doc, f)
            os.replace(tmp, path)
        except OSError as e:
            log.warning("learned_prior: flush to %s failed: %s", path, e)

    def to_status(self) -> dict:
        bins = []
        for b in range(self._nbins):
            c = self.bin_ceiling(b)
            if c is None:
                continue
            rssi_lo = self.cfg.rssi_min + b * self.cfg.bin_width_db
            bins.append({"rssi": rssi_lo, "ceiling": c,
                         "n": int(self._cells[b][c][1])})
        return {"key": self.key, "bins": bins}
```

And call `self._load()` at the END of `__init__` (after `self._since_flush = 0`):

```python
        self._since_flush = 0
        self._load()
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: PASS (20 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "feat(gs/dynlink): LearnedPrior persistence (atomic, versioned) + to_status"
```

---

# Part B — Flight logging

## Task 7: `FlightLog` — per-session JSONL with rotation + size cap

**Files:** Create `gs/fpvdgs/dynlink/flightlog.py`; Test `gs/tests/unit/test_dl_flightlog.py`.

- [ ] **Step 1: Write the failing test**

```python
# gs/tests/unit/test_dl_flightlog.py
import json
from fpvdgs.dynlink.flightlog import FlightLog, FlightLogConfig


def test_writes_jsonl_records(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)), start_ms=1000)
    fl.write({"ts": 1.0, "mcs": 5})
    fl.write({"ts": 1.1, "mcs": 4})
    fl.close()
    files = list(tmp_path.glob("*.jsonl"))
    assert len(files) == 1
    lines = files[0].read_text().splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["mcs"] == 5


def test_disabled_is_noop(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), enabled=False), start_ms=1)
    fl.write({"ts": 1.0})
    fl.close()
    assert list(tmp_path.iterdir()) == []


def test_rotation_keeps_max_files(tmp_path):
    # create 5 sessions with max_files=3 → oldest pruned on close
    for i in range(5):
        fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=3),
                       start_ms=1000 + i)
        fl.write({"ts": float(i)})
        fl.close()
    files = sorted(tmp_path.glob("*.jsonl"))
    assert len(files) == 3       # only the 3 newest survive


def test_write_after_close_is_safe(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)), start_ms=1)
    fl.close()
    fl.write({"ts": 1.0})        # no crash
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog.py -q`
Expected: FAIL — `ModuleNotFoundError: fpvdgs.dynlink.flightlog`.

- [ ] **Step 3: Write the implementation**

```python
# gs/fpvdgs/dynlink/flightlog.py
"""Per-flight structured JSONL logger (Phase 4, spec §8).

One file per dynamicLink session, one JSON record per selector tick.
GS-side, dependency-free. Size-capped + rotated. Pulled off-device for
analysis by gs/tools/flightlog_analyze.py."""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass

log = logging.getLogger("fpvdgs.dynlink")


@dataclass
class FlightLogConfig:
    enabled: bool = True
    dir: str = "/etc/fpvd/flightlog"
    max_files: int = 8
    max_mb: float = 4.0


class FlightLog:
    def __init__(self, cfg: FlightLogConfig, *, start_ms: int) -> None:
        self.cfg = cfg
        self._fh = None
        self._bytes = 0
        self._max_bytes = int(cfg.max_mb * 1024 * 1024)
        if not cfg.enabled:
            return
        try:
            os.makedirs(cfg.dir, exist_ok=True)
            self._path = os.path.join(cfg.dir, f"{start_ms}.jsonl")
            self._fh = open(self._path, "w")
        except OSError as e:
            log.warning("flightlog: open failed in %s: %s", cfg.dir, e)
            self._fh = None

    def write(self, record: dict) -> None:
        if self._fh is None:
            return
        if self._bytes >= self._max_bytes:
            return  # this session hit its size cap; stop appending
        try:
            line = json.dumps(record, separators=(",", ":")) + "\n"
            self._fh.write(line)
            self._bytes += len(line)
        except (OSError, TypeError) as e:
            log.warning("flightlog: write failed: %s", e)

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError:
                pass
            self._fh = None
        self._prune()

    def _prune(self) -> None:
        try:
            files = sorted(
                (os.path.join(self.cfg.dir, f) for f in os.listdir(self.cfg.dir)
                 if f.endswith(".jsonl")),
                key=os.path.getmtime,
            )
        except OSError:
            return
        for stale in files[:-self.cfg.max_files] if self.cfg.max_files > 0 else []:
            try:
                os.remove(stale)
            except OSError:
                pass
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog.py -q`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/flightlog.py gs/tests/unit/test_dl_flightlog.py
git commit -m "feat(gs/dynlink): FlightLog per-session JSONL + rotation/size-cap"
```

---

# Part C — Integration

## Task 8: Config build — `LearnedPriorConfig` + `FlightLogConfig` from `tuning`

**Files:** Modify `gs/fpvdgs/dynlink/policy.py`, `gs/fpvdgs/dynlink/config_build.py`; Test `gs/tests/unit/test_dl_config_build.py`.

Add the two configs onto `PolicyConfig` (default-constructed so existing call sites keep working), and parse them from `tuning.learned_prior` (+ nested `flightlog`).

- [ ] **Step 1: Write the failing test** — append to `gs/tests/unit/test_dl_config_build.py`:

```python
def test_learned_prior_config_parsed():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {"learned_prior": {
        "bin_width_db": 3.0, "min_samples_warmstart": 7,
        "flightlog": {"max_files": 2, "enabled": False},
    }}})
    assert cfg.learned_prior.bin_width_db == 3.0
    assert cfg.learned_prior.min_samples_warmstart == 7
    assert cfg.flightlog.max_files == 2
    assert cfg.flightlog.enabled is False


def test_learned_prior_defaults_when_absent():
    from fpvdgs.dynlink.config_build import build_policy_config
    cfg = build_policy_config({"tuning": {}})
    assert cfg.learned_prior.enabled is True
    assert cfg.learned_prior.bin_width_db == 2.0
    assert cfg.flightlog.enabled is True
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_config_build.py -k learned_prior -q`
Expected: FAIL — `PolicyConfig` has no `learned_prior` field.

- [ ] **Step 3a: Extend `PolicyConfig`** — in `gs/fpvdgs/dynlink/policy.py`, add imports + fields:

At the top imports, add:
```python
from .flightlog import FlightLogConfig
from .learned_prior import LearnedPriorConfig
```

In `@dataclass class PolicyConfig`, add two fields (after `selection`):
```python
    learned_prior: LearnedPriorConfig = field(default_factory=LearnedPriorConfig)
    flightlog: FlightLogConfig = field(default_factory=FlightLogConfig)
```

- [ ] **Step 3b: Parse them in `config_build.py`** — in `_build_policy_config`, before the `return PolicyConfig(...)`, add:

```python
    lp_raw = raw.get("learned_prior", {}) or {}
    fl_raw = lp_raw.get("flightlog", {}) or {}
    learned_prior = LearnedPriorConfig(
        enabled=bool(lp_raw.get("enabled", True)),
        bin_width_db=float(lp_raw.get("bin_width_db", 2.0)),
        rssi_min=float(lp_raw.get("rssi_min", -90.0)),
        rssi_max=float(lp_raw.get("rssi_max", -30.0)),
        ewma_alpha=float(lp_raw.get("ewma_alpha", 0.1)),
        viable_threshold=float(lp_raw.get("viable_threshold", 0.99)),
        min_samples_warmstart=int(lp_raw.get("min_samples_warmstart", 20)),
        min_samples_predictive=int(lp_raw.get("min_samples_predictive", 40)),
        warmstart_margin=int(lp_raw.get("warmstart_margin", 0)),
        predictive_horizon_ticks=int(lp_raw.get("predictive_horizon_ticks", 3)),
        predictive_debounce_windows=int(lp_raw.get("predictive_debounce_windows", 3)),
        flush_interval_observations=int(lp_raw.get("flush_interval_observations", 50)),
        persist_dir=str(lp_raw.get("persist_dir", "/etc/fpvd/learned")),
    )
    flightlog = FlightLogConfig(
        enabled=bool(fl_raw.get("enabled", True)),
        dir=str(fl_raw.get("dir", "/etc/fpvd/flightlog")),
        max_files=int(fl_raw.get("max_files", 8)),
        max_mb=float(fl_raw.get("max_mb", 4.0)),
    )
```

Add the imports at the top of `config_build.py`:
```python
from .flightlog import FlightLogConfig
from .learned_prior import LearnedPriorConfig
```

And extend the `return PolicyConfig(...)` to pass them:
```python
    return PolicyConfig(
        leading=leading,
        gate=gate,
        selection=selection,
        starvation_windows=int(policy_raw.get("starvation_windows", 5)),
        learned_prior=learned_prior,
        flightlog=flightlog,
    )
```

- [ ] **Step 4: Run to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_config_build.py -q`
Expected: PASS (existing + 2 new).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/policy.py gs/fpvdgs/dynlink/config_build.py gs/tests/unit/test_dl_config_build.py
git commit -m "feat(gs/dynlink): parse learned_prior + flightlog config knobs"
```

---

## Task 9: Wire `LearnedPrior` + `FlightLog` into `Policy`

**Files:** Modify `gs/fpvdgs/dynlink/policy.py`, `gs/fpvdgs/dynlink/controller.py`; Test `gs/tests/unit/test_dl_policy_learned.py` (create).

`Policy.__init__` builds the prior (keyed by `profile.name`) + the flight log. `tick()`: warm-start replaces the cold-start seed when the curve is confident; a confidence-gated, debounced, **down-only** predictive demote mirrors the existing cold-start `current_mcs` write; every tick ingests one observation and appends a flight-log record. `Policy.close()` flushes + closes; the controller calls it in `_run()`'s `finally`.

- [ ] **Step 1: Write the failing test** — create `gs/tests/unit/test_dl_policy_learned.py`:

```python
"""Phase 4 integration: warm-start + predictive-demote + regression."""
from __future__ import annotations

from pathlib import Path

from fpvdgs.dynlink.policy import Policy, PolicyConfig
from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.profile import load_profile
from fpvdgs.dynlink.signals import Signals

PROFILES = Path(__file__).resolve().parents[2] / "fpvdgs" / "dynlink" / "profiles"


def _profile():
    return load_profile("m8812eu2", [PROFILES])


def _cfg(tmp_path, **lp):
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path), **lp),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def _sig(rssi, ts=1.0):
    return Signals(rssi=rssi, residual_loss_w=0.0, fec_work=0.0,
                   link_starved_w=False, timestamp=ts)


def test_warm_start_seeds_from_persisted_curve(tmp_path):
    # Flight 1: build a confident curve at -50 -> ceiling 5, persist.
    prof = _profile()
    p1 = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3), prof)
    for _ in range(5):
        p1.learned_prior.ingest(rssi=-50.0, probed_rung=5, probe_clean=True,
                                operating_mcs=5, operating_clean=True)
    p1.close()   # flushes
    # Flight 2: a fresh Policy warm-starts to the learned ceiling on tick 1,
    # instead of climbing from MCS 1.
    p2 = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3), prof)
    dec = p2.tick(_sig(-50.0))
    assert dec.mcs == 5
    p2.close()


def test_unknown_curve_falls_back_to_cold_start(tmp_path):
    # Empty store → warm-start unknown → today's coarse_mcs_for_rssi seed.
    p = Policy(_cfg(tmp_path, min_samples_warmstart=100), _profile())
    dec = p.tick(_sig(-50.0))   # coarse table: rssi>=-55 → mcs 5
    assert dec.mcs == 5
    p.close()


def test_predictive_demote_on_confident_fade(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
                    min_samples_predictive=3, predictive_horizon_ticks=2,
                    predictive_debounce_windows=2), prof)
    # learn: strong RSSI -> ceiling 5, the bin a fast fade lands in -> ceiling 2
    for _ in range(5):
        for rung in range(6):
            p.learned_prior.ingest(rssi=-50.0, probed_rung=rung, probe_clean=True,
                                   operating_mcs=rung, operating_clean=True)
        for rung in range(3):
            p.learned_prior.ingest(rssi=-56.0, probed_rung=rung, probe_clean=True,
                                   operating_mcs=rung, operating_clean=True)
        p.learned_prior.ingest(rssi=-56.0, probed_rung=3, probe_clean=False,
                               operating_mcs=2, operating_clean=True)
    # warm-start to 5 at -50, then a -3 dB/tick fade; after debounce the
    # operating MCS pre-demotes toward the projected ceiling (2), down-only.
    p.tick(_sig(-50.0, ts=1.0))
    p.tick(_sig(-53.0, ts=1.1))
    dec = p.tick(_sig(-56.0, ts=1.2))
    assert dec.mcs <= 2
    p.close()
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py -q`
Expected: FAIL — `Policy` has no `learned_prior` attribute / `close`.

- [ ] **Step 3: Implement in `Policy`** (`gs/fpvdgs/dynlink/policy.py`).

3a. Add imports at top:
```python
from .flightlog import FlightLog
from .learned_prior import LearnedPrior
```

3b. In `Policy.__init__`, after `self._starvation_count: int = 0`, add:
```python
        # Phase 4: learned per-card prior + flight log. Keyed by the radio
        # profile name (the operator-set radioProfile). GS-local; the live
        # probe stays authoritative.
        self.learned_prior = (
            LearnedPrior(profile.name, cfg.learned_prior)
            if cfg.learned_prior.enabled else None
        )
        self._prev_rssi: float | None = None
        self._predict_demote_count = 0
        start_ms = int((__import__("time").monotonic()) * 1000)
        self.flightlog = FlightLog(cfg.flightlog, start_ms=start_ms)
```

3c. Replace the cold-start seed block in `tick()` (the `if not self._cold_started and signals.rssi is not None:` block) with a learned-or-coarse warm-start:
```python
        # Warm-start seed (one-shot). Prefer the learned per-card curve; fall
        # back to the coarse hand-table when it's unknown/unconfident. Only
        # raises the boot MCS, runs before select().
        if not self._cold_started and signals.rssi is not None:
            seed = None
            if self.learned_prior is not None:
                seed = self.learned_prior.warmstart_seed(signals.rssi)
            if seed is None:
                seed = coarse_mcs_for_rssi(signals.rssi)
            if seed is not None and seed > self.leading.state.current_mcs:
                self.leading.state.current_mcs = min(seed, self.leading._cap_mcs)
                self.leading.state.tx_power_dBm = self.leading._compute_tx_power(
                    self.leading.state.current_mcs)
            self._cold_started = True
```

3d. Add the predictive-demote block immediately AFTER the warm-start block and BEFORE the `select()` call:
```python
        # Predictive demote (down-only, confidence-gated, debounced). If the
        # curve says the ceiling at the projected RSSI is below where we run,
        # pre-demote ahead of the reactive path. The probe still owns promotes;
        # the reactive Channel-B demote in select() remains the backstop.
        predict_reason = ""
        if (self.learned_prior is not None and signals.rssi is not None):
            slope = (0.0 if self._prev_rssi is None
                     else signals.rssi - self._prev_rssi)
            pc = self.learned_prior.predictive_ceiling(signals.rssi, slope)
            cur = self.leading.state.current_mcs
            if pc is not None and pc < cur:
                self._predict_demote_count += 1
                if (self._predict_demote_count
                        >= self.cfg.learned_prior.predictive_debounce_windows):
                    self.leading.state.current_mcs = max(pc, 0)
                    self.leading.state.tx_power_dBm = (
                        self.leading._compute_tx_power(
                            self.leading.state.current_mcs))
                    self.leading._promote_clean = 0
                    predict_reason = f"predict_demote mcs{cur}->{pc}"
            else:
                self._predict_demote_count = 0
        self._prev_rssi = signals.rssi
```

3e. After `select(...)` returns, merge the predictive reason and ingest + log. Replace the final `return Decision(...)` block with:
```python
        # Ingest one observation for the learned prior (spec §4): the probe
        # rung verdict (current+1) and the operating-rung health.
        if self.learned_prior is not None and signals.rssi is not None:
            target = self.leading.state.current_mcs + 1
            rung = (self._probe_status() if self._probe_status else {}) or {}
            rung = rung.get("mcs", {}).get(str(target)) if target <= 7 else None
            probe_clean = bool(
                rung and rung.get("per") is not None
                and (1.0 - rung["per"]) >= self.cfg.gate.probe_viable_threshold
            )
            operating_clean = signals.residual_loss_w < self.cfg.gate.video_demote_per
            self.learned_prior.ingest(
                rssi=signals.rssi,
                probed_rung=(target if rung is not None else None),
                probe_clean=probe_clean,
                operating_mcs=new_mcs,
                operating_clean=operating_clean,
            )

        reason = "; ".join(
            r for r in ([predict_reason] + self.leading.reasons) if r
        )
        self.flightlog.write({
            "ts": signals.timestamp,
            "rssi": signals.rssi,
            "mcs": new_mcs,
            "reason": reason,
            "residual_loss_w": signals.residual_loss_w,
            "fec_work": signals.fec_work,
            "link_starved": sustained_starved,
            "ceiling": (self.learned_prior.ceiling(signals.rssi)
                        if self.learned_prior and signals.rssi is not None else None),
        })
        return Decision(
            timestamp=signals.timestamp,
            mcs=new_mcs,
            reason=reason,
            signals_snapshot={
                "rssi": signals.rssi,
                "residual_loss_w": signals.residual_loss_w,
                "fec_work": signals.fec_work,
                "link_starved": sustained_starved,
                "mcs": new_mcs,
            },
        )
```

3f. Add `Policy.close()` at the end of the class:
```python
    def close(self) -> None:
        """Flush the learned prior + close the flight log. Called by the
        controller when the dynamicLink loop tears down."""
        if self.learned_prior is not None:
            self.learned_prior.flush()
        self.flightlog.close()
```

- [ ] **Step 4: Wire `policy.close()` in the controller** — in `gs/fpvdgs/dynlink/controller.py` `_run()`, the `try/finally` around `self._stats_loop(on_event)` (lines ~198-204). Add `policy.close()` in the `finally`:

```python
        try:
            await self._stats_loop(on_event)
        finally:
            policy.close()
            if idr_transport is not None:
                idr_transport.close()
            return_link.close()
            self._set(running=False, statsConnected=False, idrListen=None)
```

- [ ] **Step 5: Run to verify**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py tests/unit/test_dl_policy_leading.py tests/unit/test_dl_controller.py -q`
Expected: PASS — the 3 new integration tests + the existing selector/controller tests stay green (regression: empty/unknown prior → today's behavior).

- [ ] **Step 6: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/policy.py gs/fpvdgs/dynlink/controller.py gs/tests/unit/test_dl_policy_learned.py
git commit -m "feat(gs/dynlink): wire LearnedPrior warm-start + predictive-demote + flightlog into Policy"
```

---

## Task 10: Opportunistic adapter_id ↔ radioProfile cross-check warning

**Files:** Modify `gs/fpvdgs/supervisor.py`; Test `gs/tests/unit/test_status.py` (or a new small test).

The supervisor already fetches the drone status in `_dynamic_link_status` (`drone.get_status()`). Reuse that to read `radio.adapterId` and warn **once** if it disagrees with the configured `radioProfile`. Best-effort, never blocks, no new fetch.

- [ ] **Step 1: Write the failing test** — append to `gs/tests/unit/test_status.py`:

```python
def test_adapter_id_mismatch_warns_once(caplog):
    import logging
    from fpvdgs.supervisor import adapter_matches_profile
    # bl-m8812eu2 matches radioProfile m8812eu2; m8731 does not.
    assert adapter_matches_profile("bl-m8812eu2", "m8812eu2") is True
    assert adapter_matches_profile("bl-m8731bu4", "m8812eu2") is False
    assert adapter_matches_profile(None, "m8812eu2") is True   # unknown → no warn
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_status.py::test_adapter_id_mismatch_warns_once -q`
Expected: FAIL — `ImportError: cannot import name 'adapter_matches_profile'`.

- [ ] **Step 3: Implement** — in `gs/fpvdgs/supervisor.py`, add a module-level helper near the top:

```python
def adapter_matches_profile(adapter_id, radio_profile) -> bool:
    """Loose match: the drone's radio-up.sh adapter_id (e.g. 'bl-m8812eu2')
    should contain the configured radioProfile (e.g. 'm8812eu2'). Unknown
    adapter_id (None / "") → treated as a match (no warning)."""
    if not adapter_id:
        return True
    return str(radio_profile) in str(adapter_id)
```

Then in `_dynamic_link_status`, after the `drone_active = ...` block, read the adapter id from the same status and warn once:

```python
    def _dynamic_link_status(reachable):
        eff_dl = store.effective().get("dynamicLink", {})
        st = dynlink.status()
        st["enabled"] = bool(eff_dl.get("enabled"))
        drone_active = None
        adapter_id = None
        try:
            ds = drone.get_status()
            drone_active = ds.get("link", {}).get("dynamicLinkActive")
            adapter_id = ds.get("radio", {}).get("adapterId")
        except Exception:
            pass
        prof = eff_dl.get("radioProfile", "m8812eu2")
        if not adapter_matches_profile(adapter_id, prof) and not _warned["adapter"]:
            log.warning("drone adapter_id %r does not match radioProfile %r — "
                        "the learned prior is keyed by radioProfile; check config",
                        adapter_id, prof)
            _warned["adapter"] = True
        st["drone"] = {"reachable": reachable, "dynamicLinkActive": drone_active}
        return st
```

Add a one-shot guard near where `_dynamic_link_status` is defined (module/function scope as appropriate — a dict so the closure can mutate it):
```python
    _warned = {"adapter": False}
```
and ensure `log = logging.getLogger(...)` exists in `supervisor.py` (it does; reuse it).

(Read the actual `_dynamic_link_status` and surrounding scope first; place `_warned` so the closure can see + mutate it. The exact `drone.get_status()` shape is `{"link": {...}, "radio": {"adapterId": ...}}` — confirmed live.)

- [ ] **Step 4: Run to verify**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_status.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/supervisor.py gs/tests/unit/test_status.py
git commit -m "feat(gs): warn on drone adapter_id <-> radioProfile mismatch (best-effort)"
```

---

## Task 11: Imports guard

**Files:** Modify `gs/tests/unit/test_dl_imports.py`.

- [ ] **Step 1: Add the new modules to the guard** — in `gs/tests/unit/test_dl_imports.py`, add `"learned_prior"` and `"flightlog"` to the `MODULES` list (read the file to match the exact list style).

- [ ] **Step 2: Run to verify**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_imports.py -q`
Expected: PASS (the two new modules import cleanly).

- [ ] **Step 3: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/tests/unit/test_dl_imports.py
git commit -m "test(gs/dynlink): guard learned_prior + flightlog imports"
```

---

# Part D — Offline analysis tool

## Task 12: `flightlog_analyze.py` + smoke test

**Files:** Create `gs/tools/flightlog_analyze.py`; Test `gs/tests/unit/test_dl_flightlog_analyze.py`.

A dev-machine script: read a `<flight>.jsonl`, print summary stats (time-at-each-MCS, demote counts split reactive vs predictive, warm-start hit/fallback, mean MCS vs mean ceiling), and — if `matplotlib` is importable — save a timeline PNG (RSSI / MCS / probe-PER). The smoke test exercises the text path only (no matplotlib dependency in CI).

- [ ] **Step 1: Write the failing test** — create `gs/tests/unit/test_dl_flightlog_analyze.py`:

```python
import json
import importlib.util
from pathlib import Path

TOOL = Path(__file__).resolve().parents[2] / "tools" / "flightlog_analyze.py"


def _load_tool():
    spec = importlib.util.spec_from_file_location("flightlog_analyze", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_summarize_counts_mcs_and_demotes(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    with open(log, "w") as f:
        f.write(json.dumps({"ts": 1.0, "rssi": -50, "mcs": 5, "reason": ""}) + "\n")
        f.write(json.dumps({"ts": 1.1, "rssi": -55, "mcs": 4,
                            "reason": "predict_demote mcs5->4"}) + "\n")
        f.write(json.dumps({"ts": 1.2, "rssi": -60, "mcs": 3,
                            "reason": "video_per_demote loss=0.060"}) + "\n")
    s = mod.summarize(str(log))
    assert s["records"] == 3
    assert s["time_at_mcs"][5] >= 1
    assert s["predictive_demotes"] == 1
    assert s["reactive_demotes"] == 1
```

- [ ] **Step 2: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_analyze.py -q`
Expected: FAIL — file not found / no `summarize`.

- [ ] **Step 3: Write the implementation**

```python
# gs/tools/flightlog_analyze.py
#!/usr/bin/env python3
"""Offline analysis of a Phase-4 flight log (gs/fpvdgs/dynlink/flightlog.py).

Usage:
    python3 flightlog_analyze.py <flight>.jsonl [--plot out.png]

Prints summary stats. With --plot and matplotlib available, also writes a
RSSI / MCS / probe-PER timeline PNG. Dev-machine tool — not deployed."""
from __future__ import annotations

import argparse
import json
from collections import Counter


def _records(path):
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    yield json.loads(line)
                except ValueError:
                    continue


def summarize(path) -> dict:
    recs = list(_records(path))
    time_at_mcs = Counter()
    predictive = reactive = warm_fallback = 0
    ceilings, mcss = [], []
    for r in recs:
        time_at_mcs[r.get("mcs")] += 1
        reason = r.get("reason") or ""
        if "predict_demote" in reason:
            predictive += 1
        if "video_per_demote" in reason or "emergency" in reason:
            reactive += 1
        if r.get("mcs") is not None:
            mcss.append(r["mcs"])
        if r.get("ceiling") is not None:
            ceilings.append(r["ceiling"])
    return {
        "records": len(recs),
        "time_at_mcs": dict(time_at_mcs),
        "predictive_demotes": predictive,
        "reactive_demotes": reactive,
        "mean_mcs": (sum(mcss) / len(mcss)) if mcss else None,
        "mean_ceiling": (sum(ceilings) / len(ceilings)) if ceilings else None,
    }


def _print_summary(s: dict) -> None:
    print(f"records: {s['records']}")
    print(f"time-at-MCS (ticks): {s['time_at_mcs']}")
    print(f"predictive demotes: {s['predictive_demotes']}")
    print(f"reactive demotes:   {s['reactive_demotes']}")
    print(f"mean MCS: {s['mean_mcs']}   mean ceiling: {s['mean_ceiling']}")


def _plot(path, out):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available — skipping plot")
        return
    recs = list(_records(path))
    ts = [r.get("ts") for r in recs]
    fig, ax = plt.subplots(2, 1, sharex=True)
    ax[0].plot(ts, [r.get("rssi") for r in recs], label="rssi")
    ax[0].plot(ts, [r.get("ceiling") for r in recs], label="ceiling")
    ax[0].legend(); ax[0].set_ylabel("RSSI / ceiling")
    ax[1].plot(ts, [r.get("mcs") for r in recs], label="mcs")
    ax[1].legend(); ax[1].set_ylabel("MCS"); ax[1].set_xlabel("ts")
    fig.savefig(out)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()
    _print_summary(summarize(args.logfile))
    if args.plot:
        _plot(args.logfile, args.plot)


if __name__ == "__main__":
    main()
```

- [ ] **Step 4: Run to verify**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_analyze.py -q`
Expected: PASS.

- [ ] **Step 5: Full suite + commit**

Run the whole GS suite to confirm green: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/ -q 2>&1 | tail -3` → green (211 baseline + the new tests; **0 failed**).

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/tools/flightlog_analyze.py gs/tests/unit/test_dl_flightlog_analyze.py
git commit -m "feat(gs/tools): offline flight-log analysis script (Phase 4)"
```

---

## Task 13: Deploy note (no on-hardware step required to land)

**Files:** none (deploy is operator-run, GS-only).

Phase 4 is GS-only and wire-unchanged, so it is **not** a flag-day. When ready, deploy the GS alone: `./deploy/gs/deploy.sh --host 10.18.0.1`. The deploy script already copies `fpvdgs/dynlink/*.py` (so `learned_prior.py` + `flightlog.py` ship); `gs/tools/flightlog_analyze.py` is dev-machine-only and is **not** deployed. The drone is untouched. First flight: empty store → today's behavior + the store fills under `/etc/fpvd/learned/`; pull flight logs from `/etc/fpvd/flightlog/` via `scp` for analysis. Hand this deploy to the operator (do not deploy from the execution session unless told to).

---

## Self-Review

**Spec coverage (`2026-06-07-phase4-learned-rssi-prior-design.md`):**
- §3 model (binned `{clean_ewma,n}`, ceiling, confidence, isotonic ladder) → Tasks 1–4. ✓
- §4 observations (probe rung + operating rung) → Task 2 (ingest) + Task 9 step 3e (wiring the two sources). ✓
- §5 warm-start (confident seed, clamp, coarse fallback) → Task 5 (`warmstart_seed`) + Task 9 step 3c. ✓
- §6 predictive-demote (down-only, debounced, confident, reactive backstop intact) → Task 5 (`predictive_ceiling`) + Task 9 step 3d. ✓
- §7 persistence + keying (`radioProfile`=`profile.name`, atomic, versioned, adapter cross-check) → Task 6 + Task 9 (key) + Task 10 (cross-check). ✓
- §8 flight logging + analysis script → Task 7 (logger) + Task 9 step 3e (records) + Task 12 (tool). ✓
- §9 config knobs → Task 8. ✓
- §10 bootstrap + failure modes (empty→today, corrupt→empty, disabled→noop) → Tasks 6/7 (corrupt/disabled), Task 9 (`enabled` gate + coarse fallback). ✓
- §12 testing (unit engine, flightlog, integration warm-start/predictive/regression, analysis smoke) → Tasks 1–12. ✓

**Placeholder scan:** every code step has complete, runnable code; deletion/wiring steps name exact files + the surrounding anchor (e.g. controller `finally`, the cold-start block). The Task 10 note to "read the actual `_dynamic_link_status` scope first" is a placement seam, not a missing implementation (the helper + the warn call are given in full). No TBDs.

**Type/name consistency:** `LearnedPrior(key, cfg)` / `.ingest(*, rssi, probed_rung, probe_clean, operating_mcs, operating_clean)` / `.warmstart_seed(rssi)` / `.predictive_ceiling(rssi, slope)` / `.ceiling(rssi)` / `.flush()` / `.to_status()` and `FlightLog(cfg, *, start_ms)` / `.write(record)` / `.close()` are used identically across Tasks 1–12. `PolicyConfig.learned_prior` / `.flightlog` (Task 8) match `Policy.__init__`'s reads (Task 9). `profile.name` is the curve key (confirmed `RadioProfile.name` exists). `coarse_mcs_for_rssi` (kept) is the fallback. `MAX_MCS = 7` consistent with `GateConfig.max_mcs`.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-07-phase4-learned-rssi-prior.md`.** Two execution options:

1. **Subagent-Driven (recommended)** — fresh subagent per task, spec + quality review between. GS-only; keep the suite green at every commit (Tasks 1–12 each commit; Task 13 is the operator deploy — STOP before it unless told to deploy).
2. **Inline Execution** — execute via executing-plans with checkpoints.

**Note:** Part A (Tasks 1–6, the engine) and Part B (Task 7, the logger) are self-contained pure units; Part C (8–11) wires them into `Policy`/controller/supervisor; Part D (12) is the offline tool. Task 13 (deploy) is GS-only and operator-run — not a flag-day (wire + drone unchanged).
