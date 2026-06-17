# Learned-Prior Knee-Model Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the binned RSSI→ceiling learned prior with a per-rung RSSI-knee model that is monotone by construction, recency-weighted, and learns only from settled rungs.

**Architecture:** A new `KneeModel` class (per-rung `knee[K]` + decaying `count[K]`, asymmetric-pull update, cumulative-max monotone `ceiling`) lives in `learned_prior.py` alongside a thin `LearnedPrior` facade that keeps the public interface `policy.py` depends on. `policy.py` computes a `settled` flag and feeds operating-rung outcomes only (probe dropped from learning). Predictive-demote policy logic is unchanged.

**Tech Stack:** Python 3.11, pytest. All work under `gs/`. Run tests with `cd gs && .venv/bin/python -m pytest`.

**Spec:** `docs/superpowers/specs/2026-06-16-learned-prior-knee-model-design.md`

---

## File Structure

- `gs/fpvdgs/dynlink/learned_prior.py` — **rewrite**: `LearnedPriorConfig` (new fields), `lsq_slope` (kept verbatim), `KneeModel` (new), `LearnedPrior` facade (new internals, same interface).
- `gs/fpvdgs/dynlink/policy.py` — **modify**: settled-tick tracking, new `ingest` call, flightlog `knees`/`prior_learn`.
- `gs/fpvdgs/dynlink/config_build.py` — **modify**: expose knee-model knobs.
- `gs/tools/flightlog_analyze.py` — **modify**: knee summary line.
- `gs/tests/unit/test_dl_knee_model.py` — **create**: `KneeModel` units.
- `gs/tests/unit/test_dl_learned_prior.py` — **replace**: facade tests (delete bin-based tests).
- `gs/tests/unit/test_dl_policy_learned.py` — **modify**: new ingest signature + knee seeding.
- `gs/tests/unit/test_dl_flightlog_debug_fields.py` — **modify**: new ingest signature + knee fields.
- `gs/tests/unit/test_dl_config_build.py` — **modify**: knee-model knobs; drop binWidthDb-frozen assertion.

Throughout, `MAX_MCS = 7`. The `m8812eu2.json` (schema v1) is ignored on load and retrains.

---

### Task 1: `LearnedPriorConfig` + `KneeModel.observe` (asymmetric pull)

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py`
- Test: `gs/tests/unit/test_dl_knee_model.py` (create)

- [ ] **Step 1: Write the failing test**

Create `gs/tests/unit/test_dl_knee_model.py`:

```python
from fpvdgs.dynlink.learned_prior import KneeModel, LearnedPriorConfig, MAX_MCS


def _model(**kw):
    return KneeModel(LearnedPriorConfig(**kw))


def test_first_sample_seeds_knee_at_rssi():
    m = _model()
    m.observe(rung=4, rssi=-60.0, clean=True)
    assert m._knee[4] == -60.0
    assert m._count[4] == 1.0


def test_clean_below_knee_pulls_down_slowly():
    m = _model(alpha_relax=0.1, recency_decay=1.0)
    m.observe(4, -60.0, clean=True)        # seed -60
    m.observe(4, -70.0, clean=True)        # works even at -70 -> knee toward -70
    # -60 + 0.1*(-70 - -60) = -61.0
    assert m._knee[4] == -61.0


def test_dirty_above_knee_pulls_up_fast():
    m = _model(alpha_tighten=0.5, recency_decay=1.0)
    m.observe(4, -70.0, clean=True)        # seed -70
    m.observe(4, -60.0, clean=False)       # fails at -60 -> knee toward -60
    # -70 + 0.5*(-60 - -70) = -65.0
    assert m._knee[4] == -65.0


def test_tighten_faster_than_relax():
    up = _model(alpha_tighten=0.25, alpha_relax=0.05, recency_decay=1.0)
    up.observe(4, -70.0, True); up.observe(4, -60.0, False)   # dirty pull up
    down = _model(alpha_tighten=0.25, alpha_relax=0.05, recency_decay=1.0)
    down.observe(4, -60.0, True); down.observe(4, -70.0, True)  # clean pull down
    moved_up = abs(up._knee[4] - (-70.0))
    moved_down = abs(down._knee[4] - (-60.0))
    assert moved_up > moved_down           # pessimistic asymmetry


def test_consistent_sample_does_not_move_knee():
    m = _model(recency_decay=1.0)
    m.observe(4, -60.0, clean=True)        # seed -60
    m.observe(4, -50.0, clean=True)        # clean ABOVE knee -> consistent, no move
    assert m._knee[4] == -60.0
    m.observe(4, -70.0, clean=False)       # dirty BELOW knee -> consistent, no move
    assert m._knee[4] == -60.0


def test_observe_ignores_out_of_range_rung():
    m = _model()
    m.observe(rung=99, rssi=-50.0, clean=True)
    assert all(k is None for k in m._knee)
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_knee_model.py -q`
Expected: FAIL — `ImportError: cannot import name 'KneeModel'`.

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/dynlink/learned_prior.py`, keep the module docstring, `import` lines, `log`, and the `lsq_slope` function **verbatim**. Replace `MAX_MCS`, `LearnedPriorConfig`, and everything from `class LearnedPrior` onward with:

```python
MAX_MCS = 7   # rung ceiling (matches SelectorConfig.max_mcs default and the drone)


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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_knee_model.py -q`
Expected: PASS (6 passed).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_knee_model.py
git commit -m "dynlink: KneeModel config + asymmetric-pull observe"
```

---

### Task 2: `KneeModel.ceiling` — monotone + confidence gate

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py` (add methods to `KneeModel`)
- Test: `gs/tests/unit/test_dl_knee_model.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_knee_model.py`:

```python
def _confident(m, rung, knee, *, n=10):
    """Force a confident knee directly (bypass learning dynamics)."""
    m._knee[rung] = knee
    m._count[rung] = float(n)


def test_ceiling_none_when_cold():
    m = _model()
    assert m.ceiling(-50.0) is None


def test_ceiling_highest_confident_rung_at_or_below_rssi():
    m = _model(min_samples=8)
    _confident(m, 1, -80.0)
    _confident(m, 4, -60.0)
    assert m.ceiling(-55.0) == 4     # -60 and -80 both <= -55
    assert m.ceiling(-70.0) == 1     # only -80 <= -70
    assert m.ceiling(-90.0) is None  # nothing low enough


def test_ceiling_ignores_unconfident_knee():
    m = _model(min_samples=8)
    _confident(m, 4, -60.0, n=10)
    m._knee[5] = -55.0
    m._count[5] = 3.0                # below min_samples
    assert m.ceiling(-50.0) == 4     # rung 5 not confident -> ignored


def test_ceiling_enforces_rung_monotonicity_on_inversion():
    # Physically-impossible inversion: rung 4 viable at LOWER rssi than rung 2.
    m = _model(min_samples=8)
    _confident(m, 2, -60.0)
    _confident(m, 4, -70.0)          # inverted
    # cumulative-max raises rung 4's effective knee to -60 (pessimistic).
    assert m.ceiling(-65.0) is None  # neither effective knee (-60) <= -65
    assert m.ceiling(-58.0) == 4
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_knee_model.py -q -k ceiling`
Expected: FAIL — `AttributeError: 'KneeModel' object has no attribute 'ceiling'`.

- [ ] **Step 3: Write minimal implementation**

Add to `KneeModel` in `learned_prior.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_knee_model.py -q`
Expected: PASS (11 passed).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_knee_model.py
git commit -m "dynlink: KneeModel monotone confidence-gated ceiling"
```

---

### Task 3: `KneeModel` recency decay

**Files:**
- Test: `gs/tests/unit/test_dl_knee_model.py` (the decay code already exists in `observe` from Task 1 — this task proves it ages a knee out)

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_knee_model.py`:

```python
def test_recency_decay_ages_out_unreinforced_knee():
    # A knee that stops being reinforced loses confidence as OTHER rungs are
    # observed, and eventually drops below min_samples (no longer in ceiling).
    m = _model(min_samples=8.0, recency_decay=0.9, alpha_relax=0.0,
               alpha_tighten=0.0)
    for _ in range(20):                      # rung 4 becomes confident
        m.observe(4, -60.0, clean=True)
    assert m.ceiling(-50.0) == 4
    for _ in range(60):                      # hammer rung 1; rung 4 decays
        m.observe(1, -80.0, clean=True)
    assert m._count[4] < 8.0                 # rung 4 confidence aged out
    assert m.ceiling(-50.0) == 1             # rung 4 no longer a confident ceiling


def test_recency_decay_one_keeps_confidence_forever():
    m = _model(min_samples=8.0, recency_decay=1.0)
    for _ in range(10):
        m.observe(4, -60.0, clean=True)
    for _ in range(1000):
        m.observe(1, -80.0, clean=True)
    assert m._count[4] == 10.0               # no decay
    assert m.ceiling(-50.0) == 4
```

- [ ] **Step 2: Run test to verify it fails or passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_knee_model.py -q -k recency`
Expected: PASS (the decay logic was implemented in Task 1). If it FAILS, fix `observe` so `recency_decay < 1.0` scales every `_count` entry on each call. This task exists to lock the behavior with a test.

- [ ] **Step 3: Commit**

```bash
git add gs/tests/unit/test_dl_knee_model.py
git commit -m "dynlink: test KneeModel recency decay ages out stale knees"
```

---

### Task 4: `KneeModel` persistence (`to_dict` / `load_dict`)

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py` (add to `KneeModel`)
- Test: `gs/tests/unit/test_dl_knee_model.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_knee_model.py`:

```python
def test_to_dict_round_trips_through_load_dict():
    m = _model()
    _confident(m, 4, -60.0, n=12)
    doc = m.to_dict()
    assert doc["schema"] == 2
    m2 = _model()
    assert m2.load_dict(doc) is True
    assert m2.ceiling(-50.0) == 4


def test_load_dict_rejects_v1_schema():
    m = _model()
    assert m.load_dict({"schema": 1, "bins": [2.0, -90, -30], "cells": []}) is False
    assert m.ceiling(-50.0) is None          # stays empty -> retrains


def test_load_dict_rejects_malformed():
    m = _model()
    assert m.load_dict({"schema": 2, "knees": [1, 2], "counts": []}) is False


def test_knees_snapshot_rounds():
    m = _model()
    m._knee[3] = -64.273
    snap = m.knees_snapshot()
    assert snap[3] == -64.3 and snap[0] is None
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_knee_model.py -q -k "dict or snapshot"`
Expected: FAIL — `AttributeError: ... 'to_dict'`.

- [ ] **Step 3: Write minimal implementation**

Add to `KneeModel` in `learned_prior.py`:

```python
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
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_knee_model.py -q`
Expected: PASS (all knee-model tests).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_knee_model.py
git commit -m "dynlink: KneeModel v2 persistence dict round-trip"
```

---

### Task 5: `LearnedPrior` facade rewrite

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py` (replace the `LearnedPrior` class)
- Test: `gs/tests/unit/test_dl_learned_prior.py` (replace entire file)

- [ ] **Step 1: Write the failing test**

Replace the **entire contents** of `gs/tests/unit/test_dl_learned_prior.py` with:

```python
import json
from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig


def _prior(tmp_path, **kw):
    cfg = LearnedPriorConfig(persist_dir=str(tmp_path), **kw)
    return LearnedPrior("m8812eu2", cfg)


def _settle(p, rung, rssi, clean, n=12):
    for _ in range(n):
        p.ingest(rssi=rssi, operating_mcs=rung, operating_clean=clean, settled=True)


def test_empty_store_returns_unknown(tmp_path):
    p = _prior(tmp_path)
    assert p.ceiling(-50.0) is None
    assert p.warmstart_seed(-50.0) is None
    assert p.predictive_ceiling(-50.0, -1.0) is None


def test_ingest_only_learns_when_settled(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    for _ in range(10):
        p.ingest(rssi=-60.0, operating_mcs=4, operating_clean=True, settled=False)
    assert p.ceiling(-50.0) is None          # nothing learned while unsettled
    _settle(p, 4, -60.0, True, n=5)
    assert p.ceiling(-50.0) == 4


def test_ingest_skips_none_rssi(tmp_path):
    p = _prior(tmp_path, min_samples=1)
    p.ingest(rssi=None, operating_mcs=4, operating_clean=True, settled=True)
    assert p.ceiling(-50.0) is None


def test_ceiling_and_warmstart_seed_from_knees(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 1, -80.0, True)
    _settle(p, 4, -60.0, True)
    assert p.ceiling(-55.0) == 4
    assert p.warmstart_seed(-70.0) == 1


def test_predictive_ceiling_projects_with_slope(tmp_path):
    p = _prior(tmp_path, min_samples=3, predictive_horizon_ticks=3)
    _settle(p, 4, -60.0, True)               # rung4 knee ~ -60
    _settle(p, 1, -80.0, True)               # rung1 knee ~ -80
    # at -58 now, fading -2/tick -> projected -58 + (-2*3) = -64 -> below rung4 knee
    assert p.predictive_ceiling(-58.0, -2.0) == 1


def test_predictive_ceiling_none_rssi(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 4, -60.0, True)
    assert p.predictive_ceiling(None, -2.0) is None


def test_persistence_round_trip_v2(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 4, -60.0, True)
    p.flush()
    p2 = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path),
                                                     min_samples=3))
    assert p2.ceiling(-50.0) == 4


def test_v1_file_ignored_and_retrains(tmp_path):
    (tmp_path / "m8812eu2.json").write_text(json.dumps(
        {"schema": 1, "bins": [2.0, -90, -30], "cells": []}))
    p = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path),
                                                    min_samples=3))
    assert p.ceiling(-50.0) is None          # v1 ignored
    _settle(p, 4, -60.0, True)
    assert p.ceiling(-50.0) == 4             # retrains on v2


def test_corrupt_file_is_ignored(tmp_path):
    (tmp_path / "m8812eu2.json").write_text("{not json")
    p = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path)))
    assert p.ceiling(-50.0) is None


def test_to_status_reports_knees(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 4, -60.0, True)
    st = p.to_status()
    assert st["key"] == "m8812eu2"
    assert isinstance(st["knees"], list) and len(st["knees"]) == 8
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q`
Expected: FAIL — `TypeError: ingest() got an unexpected keyword argument 'settled'` (old facade still present).

- [ ] **Step 3: Write minimal implementation**

Replace the `LearnedPrior` class in `learned_prior.py` with:

```python
class LearnedPrior:
    """Facade over KneeModel, keyed + persisted per radioProfile. Keeps the
    interface policy.py depends on; the live probe stays authoritative for
    promotes — this only warm-starts and feeds the down-only predictive demote."""

    def __init__(self, key: str, cfg: LearnedPriorConfig) -> None:
        self.key = key
        self.cfg = cfg
        self._model = KneeModel(cfg)
        self._since_flush = 0
        self._load()

    def ingest(self, *, rssi, operating_mcs, operating_clean, settled) -> None:
        if rssi is None or operating_mcs is None or not settled:
            return
        self._model.observe(int(operating_mcs), float(rssi), bool(operating_clean))
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
        if not self._model.load_dict(doc):
            log.info("learned_prior: %s ignored (schema/shape) — retraining", self._path())

    def flush(self) -> None:
        doc = self._model.to_dict()
        doc["key"] = self.key
        try:
            os.makedirs(self.cfg.persist_dir, exist_ok=True)
            tmp = self._path() + ".tmp"
            with open(tmp, "w") as f:
                json.dump(doc, f)
            os.replace(tmp, self._path())
        except OSError as e:
            log.warning("learned_prior: flush to %s failed: %s", self._path(), e)
```

The `import` block at the top of `learned_prior.py` must include `json`, `os`, `re`, `logging`, and `from dataclasses import dataclass` (all already present on `main`). `MAX_MCS`, `LearnedPriorConfig`, `KneeModel`, and `lsq_slope` are defined above this class from Tasks 1–4.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py tests/unit/test_dl_knee_model.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "dynlink: LearnedPrior facade over KneeModel (settled ingest, v2)"
```

---

### Task 6: Policy integration — settled tracking, ingest, flightlog

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py`
- Test: `gs/tests/unit/test_dl_policy_learned.py`, `gs/tests/unit/test_dl_flightlog_debug_fields.py`

- [ ] **Step 1: Write the failing tests**

In `gs/tests/unit/test_dl_policy_learned.py`, replace the helper that seeds the prior and the three ingest-based tests. First, the seeding helper — add near the top (after imports):

```python
def _settle_knee(policy, rung, rssi, clean, n=12):
    for _ in range(n):
        policy.learned_prior.ingest(rssi=rssi, operating_mcs=rung,
                                    operating_clean=clean, settled=True)
```

Then replace `test_warm_start_seeds_from_persisted_curve`, `test_predictive_demote_on_confident_fade`, `test_predictive_demote_blocked_when_rssi_flat`, `test_predictive_demote_blocked_when_fade_too_shallow`, and `test_predictive_demote_does_not_misfire_on_detrended_rssi` so every `learned_prior.ingest(rssi=..., probed_rung=..., probe_clean=..., operating_mcs=..., operating_clean=...)` call becomes `_settle_knee(...)` seeding the relevant rung's knee. Use these concrete bodies:

```python
def test_warm_start_seeds_from_persisted_curve(tmp_path):
    prof = _profile()
    p1 = Policy(_cfg(tmp_path, min_samples=3), prof)
    _settle_knee(p1, 5, -50.0, True)
    p1.close()
    p2 = Policy(_cfg(tmp_path, min_samples=3), prof)
    dec = p2.tick(_sig(-50.0, ts=1.0))
    assert dec.mcs == 5                      # warm-started from the persisted knee


def test_unknown_curve_no_seed_stays_at_boot(tmp_path):
    p = Policy(_cfg(tmp_path, min_samples=100), _profile())
    dec = p.tick(_sig(-50.0, ts=1.0))
    assert dec.mcs == 1                      # cold prior -> boot MCS


def test_predictive_demote_on_confident_fade(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=1), prof)
    _settle_knee(p, 1, -80.0, True)          # rung1 viable down to -80
    _settle_knee(p, 2, -62.0, True)          # rung2 viable down to -62
    _settle_knee(p, 5, -50.0, True)          # rung5 viable only >= -50
    p.leading.state.current_mcs = 5
    p.tick(_sig(-50.0, ts=1.0))              # slope 0 -> no demote yet
    dec = p.tick(_sig(-56.0, ts=1.1))        # slope -6, projected -56-18=-74 -> ceiling 1
    assert dec.mcs < 5
    p.close()
```

Replace the three gate tests entirely with these knee-seeded versions (same gate behaviour, no white-box `_cells`/`rssi_bin`):

```python
def test_predictive_demote_blocked_when_rssi_flat(tmp_path):
    """Static prior-vs-probe disagreement at flat RSSI must NOT demote — the
    slope-direction gate suppresses it (the 000010/000012 flapping fix)."""
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=2), prof)
    _settle_knee(p, 2, -50.0, True)          # learned ceiling at -50 = 2
    p.leading.state.current_mcs = 5          # probe pushed above the learned ceiling
    dec = None
    for ts in (1.0, 1.1, 1.2, 1.3, 1.4):
        dec = p.tick(_sig(-50.0, ts=ts))
    assert dec.mcs == 5                       # flat RSSI -> never predict-demoted
    p.close()


def test_predictive_demote_blocked_when_fade_too_shallow(tmp_path):
    """A real but shallow downtrend (projected drop < predictive_min_drop_db)
    must NOT demote."""
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3,
                    predictive_debounce_windows=2, predictive_min_drop_db=1.0), prof)
    _settle_knee(p, 2, -52.0, True)          # ceiling 2 across the band
    p.leading.state.current_mcs = 5
    dec = None
    for rssi, ts in [(-50.0, 1.0), (-50.2, 1.1), (-50.4, 1.2),
                     (-50.6, 1.3), (-50.8, 1.4)]:
        dec = p.tick(_sig(rssi, ts=ts))
    assert dec.mcs == 5                       # 0.2 dB/tick -> 0.6 dB over horizon < 1.0
    p.close()


def test_predictive_demote_does_not_misfire_on_detrended_rssi(tmp_path):
    """Raw RSSI (steps down on a power change) WOULD demote; EIRP-normalized
    RSSI (flat) does NOT. Exercises predictive_ceiling's projection directly."""
    from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig
    lp = LearnedPrior("test-misfire", LearnedPriorConfig(
        persist_dir=str(tmp_path), min_samples=3, predictive_horizon_ticks=3))

    def settle(rung, rssi, n=12):
        for _ in range(n):
            lp.ingest(rssi=rssi, operating_mcs=rung, operating_clean=True, settled=True)

    settle(1, -80.0); settle(2, -78.0); settle(5, -55.0)
    # Raw: rssi -62, slope -6/tick -> projected -80 -> ceiling 1 < 5 (would demote).
    assert lp.predictive_ceiling(-62.0, -6.0) == 1
    # Normalized: rssi -50, slope 0 -> projected -50 -> ceiling 5 (no demote).
    assert lp.predictive_ceiling(-50.0, 0.0) == 5
```

In `gs/tests/unit/test_dl_flightlog_debug_fields.py`, replace `test_record_carries_pc_and_slope` and update `_cfg` usages that pass `ewma_alpha`/`min_samples_warmstart`/`min_samples_predictive` (those kwargs no longer exist). New `test_record_carries_pc_and_slope`:

```python
def test_record_carries_pc_and_slope(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3), prof)
    for _ in range(5):
        p.learned_prior.ingest(rssi=-50.0, operating_mcs=5,
                                operating_clean=True, settled=True)
    p.tick(_sig(-50.0, ts=1.0))
    p.tick(_sig(-52.0, ts=1.1))
    recs = _records(tmp_path)
    assert recs[0]["slope"] == 0.0
    assert recs[1]["slope"] == -2.0
    assert "knees" in recs[1]
    assert recs[1]["prior_learn"] in (True, False)
```

Also update `test_record_carries_predict_gated_flag` and `test_record_pc_and_slope_none_when_prior_cold_or_no_rssi`: drop the removed kwargs from `_cfg(...)` (use `min_samples=...`, `predictive_horizon_ticks=3`, `predictive_debounce_windows=2`) and replace any `ingest(... probed_rung=...)` with `ingest(rssi=..., operating_mcs=..., operating_clean=..., settled=True)`.

- [ ] **Step 2: Run tests to verify they fail**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py tests/unit/test_dl_flightlog_debug_fields.py -q`
Expected: FAIL — `KeyError: 'knees'` and/or `AttributeError`/`TypeError` from the policy still using the old ingest call and config fields.

- [ ] **Step 3: Write minimal implementation**

In `gs/fpvdgs/dynlink/policy.py`:

(a) In `Policy.__init__`, after `self._starvation_count = 0` / `self._loss_count = 0`, add:

```python
        # Knee-prior learning gate: only ingest once the operating rung has
        # been unchanged for settle_ticks (loss from the last change drained).
        self._ticks_at_mcs = 0
        self._last_ingest_mcs: int | None = None
```

(b) Replace the ingest block (the `if signals.rssi is not None:` block that calls `self.learned_prior.ingest(... probed_rung=...)`) with:

```python
        # Learning gate: feed the knee prior ONLY operating-rung outcomes, and
        # only once the rung has been settled for settle_ticks (rejects fast-fade
        # transients where loss is a transition artifact, not rung unviability).
        if new_mcs != self._last_ingest_mcs:
            self._ticks_at_mcs = 0
        else:
            self._ticks_at_mcs += 1
        self._last_ingest_mcs = new_mcs
        prior_settled = self._ticks_at_mcs >= self.cfg.learned_prior.settle_ticks
        prior_learn = signals.rssi is not None and prior_settled
        self.learned_prior.ingest(
            rssi=signals.rssi,
            operating_mcs=new_mcs,
            operating_clean=signals.residual_loss_w < self.cfg.learned_prior.viable_loss,
            settled=prior_settled,
        )
```

(c) In the `self.flightlog.write({...})` dict, add two entries (next to `"pc": pc,`):

```python
            "knees": self.learned_prior.knees_snapshot(),
            "prior_learn": prior_learn,
```

The predictive-demote block, warm-start block, slope computation, and everything else stay unchanged. Note the warm-start block already calls `self.learned_prior.warmstart_seed(signals.rssi)` — that interface is preserved.

- [ ] **Step 4: Run tests to verify they pass**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_learned.py tests/unit/test_dl_flightlog_debug_fields.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_policy_learned.py gs/tests/unit/test_dl_flightlog_debug_fields.py
git commit -m "dynlink: policy feeds knee prior settled operating samples; log knees/prior_learn"
```

---

### Task 7: `config_build` — expose knee-model knobs

**Files:**
- Modify: `gs/fpvdgs/dynlink/config_build.py`
- Test: `gs/tests/unit/test_dl_config_build.py`

- [ ] **Step 1: Write the failing test**

In `gs/tests/unit/test_dl_config_build.py`, replace `test_learned_prior_is_frozen_defaults_regardless_of_config` with:

```python
def test_learned_prior_knobs_tunable():
    cfg = build_policy_config(_block(learnedPrior={
        "settleTicks": 8, "alphaTighten": 0.4, "alphaRelax": 0.02,
        "minSamples": 12, "recencyDecay": 0.999,
    }))
    lp = cfg.learned_prior
    assert lp.settle_ticks == 8
    assert lp.alpha_tighten == 0.4
    assert lp.alpha_relax == 0.02
    assert lp.min_samples == 12
    assert lp.recency_decay == 0.999


def test_learned_prior_defaults_when_absent():
    lp = build_policy_config(_block()).learned_prior
    assert lp.settle_ticks == 5
    assert lp.alpha_tighten == 0.25
    assert lp.alpha_relax == 0.05
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_config_build.py -q -k learned_prior`
Expected: FAIL — assert `5 == 8` (knobs ignored; current code builds `LearnedPriorConfig()` frozen).

- [ ] **Step 3: Write minimal implementation**

In `config_build.py`, replace the frozen `learned_prior=LearnedPriorConfig()` construction with:

```python
    # learned-prior (knee model): expose the learning knobs for in-flight
    # tuning; the predictive-machinery + persist internals stay at defaults.
    lp = block.get("learnedPrior", {}) or {}
    dlp = LearnedPriorConfig()
    learned_prior = LearnedPriorConfig(
        settle_ticks=int(lp.get("settleTicks", dlp.settle_ticks)),
        viable_loss=float(lp.get("viableLoss", dlp.viable_loss)),
        alpha_tighten=float(lp.get("alphaTighten", dlp.alpha_tighten)),
        alpha_relax=float(lp.get("alphaRelax", dlp.alpha_relax)),
        min_samples=float(lp.get("minSamples", dlp.min_samples)),
        recency_decay=float(lp.get("recencyDecay", dlp.recency_decay)),
    )
```

and pass `learned_prior=learned_prior` into the returned `PolicyConfig(...)`. Update the module docstring line that says learned-prior internals are frozen to: "learned-prior exposes its learning knobs (settleTicks/alphaTighten/alphaRelax/minSamples/recencyDecay); predictive + persistence internals stay frozen."

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_config_build.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/config_build.py gs/tests/unit/test_dl_config_build.py
git commit -m "dynlink: expose knee-model learning knobs in config_build"
```

---

### Task 8: `flightlog_analyze` — knee summary

**Files:**
- Modify: `gs/tools/flightlog_analyze.py`
- Test: `gs/tests/unit/test_dl_flightlog_analyze.py`

- [ ] **Step 1: Write the failing test**

Append to `gs/tests/unit/test_dl_flightlog_analyze.py`:

```python
def test_summarize_counts_prior_learn_and_last_knees(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    with open(log, "w") as f:
        f.write(json.dumps({"ts": 1.0, "mcs": 4, "reason": "",
                            "prior_learn": True, "knees": [None, -80, None, None, -60, None, None, None]}) + "\n")
        f.write(json.dumps({"ts": 1.1, "mcs": 4, "reason": "",
                            "prior_learn": False, "knees": [None, -80, None, None, -60, None, None, None]}) + "\n")
        f.write(json.dumps({"ts": 1.2, "mcs": 4, "reason": ""}) + "\n")  # pre-field
    s = mod.summarize(str(log))
    assert s["prior_learn_ticks"] == 1
    assert s["last_knees"] == [None, -80, None, None, -60, None, None, None]
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_analyze.py -q -k prior_learn`
Expected: FAIL — `KeyError: 'prior_learn_ticks'`.

- [ ] **Step 3: Write minimal implementation**

In `flightlog_analyze.py` `summarize()`: add `prior_learn = 0` and `last_knees = None` before the loop; in the loop add `if r.get("prior_learn"): prior_learn += 1` and `if r.get("knees") is not None: last_knees = r["knees"]`; add `"prior_learn_ticks": prior_learn` and `"last_knees": last_knees` to the returned dict. In `_print_summary`, add `print(f"prior-learn ticks:  {s['prior_learn_ticks']}")` and `print(f"last knees:          {s['last_knees']}")`.

- [ ] **Step 4: Run test to verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_analyze.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add gs/tools/flightlog_analyze.py gs/tests/unit/test_dl_flightlog_analyze.py
git commit -m "tools: flightlog_analyze reports prior_learn ticks + last knees"
```

---

### Task 9: Full-suite green + offline validation against flight 000013

**Files:**
- Test: full `gs/tests/` suite
- Validation: ad-hoc script against `000013.jsonl`

- [ ] **Step 1: Run the full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS (all). Fix any stragglers — likely other files still importing removed `LearnedPriorConfig` fields (`ewma_alpha`, `min_samples_warmstart`, `bin_width_db`, etc.) or calling `ingest` with the old signature. Grep first:

```bash
cd gs && grep -rn "ewma_alpha\|min_samples_warmstart\|min_samples_predictive\|bin_width_db\|probed_rung\|probe_clean\|warmstart_margin\|extrapolation_db_per_rung\|_cells\|rssi_bin\|bin_ceiling\|_confident_ceiling\|viable_threshold\|rssi_min\|rssi_max" tests/ fpvdgs/ tools/
```

Update each hit to the knee-model API. Re-run until green.

- [ ] **Step 2: Offline validation against 000013**

Run this (the prior is rebuilt from the log's own settled samples, then we check it would predictive-demote on the −69→−80 fade):

```bash
cd gs && .venv/bin/python - <<'PY'
import json
from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig
recs = [json.loads(l) for l in open("/run/media/gilankpam/DVR/log/dynamic-link/000013.jsonl") if l.strip()]
p = LearnedPrior("val", LearnedPriorConfig(persist_dir="/tmp/knee-val", settle_ticks=5, min_samples=8))
# replay: feed settled operating samples (rung stable >= settle_ticks)
last=None; stable=0
for r in recs:
    m=r.get("mcs"); rssi=r.get("rssi"); loss=r.get("residual_loss_w") or 0
    if m is None or rssi is None: continue
    stable = stable+1 if m==last else 0
    last=m
    p.ingest(rssi=rssi, operating_mcs=m, operating_clean=loss<0.05, settled=stable>=5)
print("learned knees:", p.knees_snapshot())
# the fade: at -69 operating MCS4, fading; does predictive_ceiling drop below 4?
for rssi,slope in [(-69,-0.5),(-72,-0.5),(-76,-0.5),(-80,-0.5)]:
    print(f"  rssi={rssi} slope={slope} -> predictive_ceiling={p.predictive_ceiling(rssi,slope)}")
PY
```

Expected: knees increase with rung; `predictive_ceiling` at the faded RSSIs returns values **< 4**, i.e. the knee model would have demoted on that fade. Record the output in the commit message. (This is a sanity check, not a unit test — if the knees look wrong, revisit `settle_ticks`/`min_samples` defaults before deploying.)

- [ ] **Step 3: Commit**

```bash
git add -A
git commit -m "dynlink: knee-model suite green + 000013 fade validation

<paste knees + predictive_ceiling output here>"
```

---

## Self-Review Notes (addressed)

- **Spec §2 (model):** Tasks 1–2 (knee/count, monotone ceiling). **§3 (learning):** Tasks 1 (asymmetric pull), 3 (decay), 6 (settled gate). **§4 (query/policy):** Tasks 5–6. **§5 (config/persistence/flightlog):** Tasks 4, 5, 6, 7. **§6 (components):** `KneeModel`+`LearnedPrior` in Task 1/5. **§7 (testing):** every task is TDD; Task 9 = offline validation.
- **Interface consistency:** `ingest(rssi, operating_mcs, operating_clean, settled)`, `ceiling(rssi)`, `predictive_ceiling(rssi, slope)`, `warmstart_seed(rssi)`, `knees_snapshot()`, `KneeModel.observe(rung, rssi, clean)`/`ceiling`/`to_dict`/`load_dict` — used identically across tasks.
- **No probe in learning** (Task 6 drops `probed_rung`/`probe_clean`). **Predictive policy logic unchanged** (Task 6 leaves the block intact).
