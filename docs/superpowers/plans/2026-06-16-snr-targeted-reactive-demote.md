# SNR-Targeted Reactive Demote Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the reactive (loss-triggered) demote jump in one move to the rung the live EIRP-normalized SNR supports, instead of stepping one blind rung per tick (which craters the link on a burst).

**Architecture:** Add a smoothed normalized `snr` to `Signals` (mirrors `rssi`). Give `LearnedPrior` a second `KneeModel` keyed on SNR (the class is already signal-agnostic). On sustained loss, `policy` computes `snr_ceiling(snr)` and the selector commits `min(current, target)` in one step. Both new parameters default to `None`, so cold/absent ⇒ today's `current − 1` behaviour (backward-compatible).

**Tech Stack:** Python 3.11, pytest. Work under `gs/`. Run tests: `cd gs && .venv/bin/python -m pytest`.

**Spec:** `docs/superpowers/specs/2026-06-16-snr-targeted-reactive-demote-design.md`
**Branch:** `feat/snr-evm-flightlog` (already has SNR/EVM logging).

---

## File Structure

- `gs/fpvdgs/dynlink/signals.py` — **modify**: add smoothed normalized `Signals.snr`.
- `gs/fpvdgs/dynlink/learned_prior.py` — **modify**: `LearnedPrior` gains a second `KneeModel` (SNR), `snr` ingest, `snr_ceiling`, `snr_knees_snapshot`, combined persistence + v2 back-compat.
- `gs/fpvdgs/dynlink/policy.py` — **modify**: compute `snr_ceiling`, pass target to `select` + `snr` to `ingest`, log `snr_norm`/`snr_ceiling`/`snr_knees`.
- Tests: `gs/tests/unit/test_dl_signals.py`, `test_dl_learned_prior.py`, `test_dl_policy_leading.py`, `test_dl_flightlog_debug_fields.py`.

`MAX_MCS = 7`. `normalize_rssi(value, mcs, cfg)` (existing) adds `P_ref − curve[mcs]`; it is value-semantics-agnostic, so it normalizes SNR with the identical offset.

---

### Task 1: `Signals.snr` — smoothed, EIRP-normalized SNR

**Files:**
- Modify: `gs/fpvdgs/dynlink/signals.py`
- Test: `gs/tests/unit/test_dl_signals.py`

- [ ] **Step 1: Write the failing test** — append to `gs/tests/unit/test_dl_signals.py`:

```python
def test_snr_is_eirp_normalized_and_smoothed():
    # raw SNR 20 at MCS4 (curve 19, P_ref 29) -> +10 offset -> snr_norm 30.
    # First window: EWMA seeds to the value, so s.snr == 30.
    s = _Agg().consume(_evm_rxev([_evm_ant(0, -60, 20, -1, -1)]))  # snr_avg=20, mcs=4
    assert s.snr == 30.0


def test_snr_none_before_any_antenna_data():
    from fpvdgs.dynlink.stats_client import RxEvent
    s = _Agg().consume(RxEvent(timestamp=1.0, id="rx", packets_window={},
                               rx_ant_stats=[], session=None))
    assert s.snr is None
```

(`_Agg`, `_evm_rxev`, `_evm_ant` already exist in this file from the SNR/EVM logging work. `_evm_ant(ant, rssi, snr, evm_min, evm_avg)` builds an `RxAnt` with `mcs=4`.)

- [ ] **Step 2: Run it and verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_signals.py -q -k "snr_is_eirp or snr_none_before"`
Expected: FAIL — `AttributeError: 'Signals' object has no attribute 'snr'` (or `s.snr` is None for the first test).

- [ ] **Step 3: Implement.** In `gs/fpvdgs/dynlink/signals.py`:

(a) In the `Signals` dataclass, find the `# EWMA-smoothed controller inputs` section (it has `rssi`/`rssi_raw`). Add after `rssi_raw`:

```python
    snr: float | None = None              # EWMA of EIRP-normalized SNR (cross-rung control axis)
```

(b) In `SignalAggregator.consume`, find the block `if s.rssi_max_w is not None:` that computes `rssi_norm_w` and EWMAs `s.rssi`. Immediately after the `s.rssi_raw = _ewma(...)` line inside that block, add:

```python
            # SNR shares RSSI's per-MCS TX-power offset (SNR scales 1:1 with
            # TX power, noise unchanged), so reuse normalize_rssi to make SNR
            # cross-rung comparable, then smooth it like rssi.
            if s.snr_w is not None:
                snr_norm_w = normalize_rssi(s.snr_w, s.mcs_w, self.rssi_norm)
                s.snr = _ewma(s.snr, snr_norm_w, self.ewma_alpha_rssi)
```

- [ ] **Step 4: Run it and verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_signals.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/signals.py gs/tests/unit/test_dl_signals.py
git commit -m "dynlink: add smoothed EIRP-normalized Signals.snr"
```

---

### Task 2: `LearnedPrior` — second SNR knee model

**Files:**
- Modify: `gs/fpvdgs/dynlink/learned_prior.py`
- Test: `gs/tests/unit/test_dl_learned_prior.py`

- [ ] **Step 1: Write the failing test** — append to `gs/tests/unit/test_dl_learned_prior.py`:

```python
def _settle_snr(p, rung, snr, clean, n=12):
    for _ in range(n):
        p.ingest(rssi=None, snr=snr, operating_mcs=rung,
                 operating_clean=clean, settled=True)


def test_snr_ceiling_learns_independently_of_rssi(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle_snr(p, 1, 10.0, True)
    _settle_snr(p, 4, 30.0, True)
    assert p.snr_ceiling(35.0) == 4
    assert p.snr_ceiling(12.0) == 1
    assert p.snr_ceiling(5.0) is None
    assert p.ceiling(-50.0) is None          # rssi model untouched (no rssi ingested)


def test_snr_ceiling_none_when_cold_or_none(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    assert p.snr_ceiling(30.0) is None       # cold
    _settle_snr(p, 4, 30.0, True)
    assert p.snr_ceiling(None) is None        # None input


def test_combined_persistence_round_trip(tmp_path):
    p = _prior(tmp_path, min_samples=3)
    _settle(p, 4, -60.0, True)               # rssi knee (existing helper)
    _settle_snr(p, 4, 30.0, True)            # snr knee
    p.flush()
    p2 = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path),
                                                     min_samples=3))
    assert p2.ceiling(-50.0) == 4
    assert p2.snr_ceiling(35.0) == 4


def test_v2_flat_file_loads_rssi_keeps_snr_cold(tmp_path):
    import json
    # a deployed v2 doc is the flat rssi-model dict (no "rssi"/"snr" wrapper)
    p1 = _prior(tmp_path, min_samples=3)
    _settle(p1, 4, -60.0, True)
    flat = p1._model.to_dict(); flat["key"] = "m8812eu2"
    (tmp_path / "m8812eu2.json").write_text(json.dumps(flat))
    p2 = LearnedPrior("m8812eu2", LearnedPriorConfig(persist_dir=str(tmp_path),
                                                     min_samples=3))
    assert p2.ceiling(-50.0) == 4            # rssi knee survived the upgrade
    assert p2.snr_ceiling(35.0) is None       # snr starts cold
```

- [ ] **Step 2: Run it and verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py -q -k "snr_ceiling or combined or v2_flat"`
Expected: FAIL — `TypeError: ingest() got an unexpected keyword argument 'snr'` / `AttributeError: ... 'snr_ceiling'`.

- [ ] **Step 3: Implement.** In `gs/fpvdgs/dynlink/learned_prior.py`, modify the `LearnedPrior` class:

(a) `__init__` — add the second model after `self._model = KneeModel(cfg)`:

```python
        self._snr_model = KneeModel(cfg)
```

(b) Replace the `ingest` method with (adds `snr=None`, feeds both, each guarded):

```python
    def ingest(self, *, rssi, snr=None, operating_mcs, operating_clean, settled) -> None:
        if operating_mcs is None or not settled:
            return
        m = int(operating_mcs)
        clean = bool(operating_clean)
        learned = False
        if rssi is not None:
            self._model.observe(m, float(rssi), clean); learned = True
        if snr is not None:
            self._snr_model.observe(m, float(snr), clean); learned = True
        if learned:
            self._since_flush += 1
            if self._since_flush >= self.cfg.flush_interval_observations:
                self.flush()
                self._since_flush = 0
```

(c) Add `snr_ceiling` and `snr_knees_snapshot` (place next to `ceiling`/`knees_snapshot`):

```python
    def snr_ceiling(self, snr) -> int | None:
        return None if snr is None else self._snr_model.ceiling(float(snr))

    def snr_knees_snapshot(self) -> list:
        return self._snr_model.knees_snapshot()
```

(d) Replace `flush` with the combined doc:

```python
    def flush(self) -> None:
        doc = {"key": self.key,
               "rssi": self._model.to_dict(),
               "snr": self._snr_model.to_dict()}
        try:
            os.makedirs(self.cfg.persist_dir, exist_ok=True)
            tmp = self._path() + ".tmp"
            with open(tmp, "w") as f:
                json.dump(doc, f)
            os.replace(tmp, self._path())
        except OSError as e:
            log.warning("learned_prior: flush to %s failed: %s", self._path(), e)
```

(e) Replace the body of `_load` after the `doc = json.load(f)` / except blocks (keep the `try/except FileNotFoundError/(ValueError, OSError)` exactly as-is) — i.e. replace the final `if not self._model.load_dict(doc): ...` line with:

```python
        # Back-compat: a v2 deploy persisted the flat rssi-model dict (no
        # "rssi"/"snr" wrapper). doc.get("rssi", doc) loads that as the rssi
        # model and leaves snr cold; a v3 combined doc loads both.
        if not self._model.load_dict(doc.get("rssi", doc)):
            log.info("learned_prior: %s rssi ignored (schema/shape) — retraining", self._path())
        snr_doc = doc.get("snr")
        if snr_doc is not None and not self._snr_model.load_dict(snr_doc):
            log.info("learned_prior: %s snr ignored (schema/shape) — retraining", self._path())
```

- [ ] **Step 4: Run it and verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_learned_prior.py tests/unit/test_dl_knee_model.py -q`
Expected: PASS (existing rssi tests still green — `snr` defaults None so they're unaffected).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/learned_prior.py gs/tests/unit/test_dl_learned_prior.py
git commit -m "dynlink: second SNR knee model in LearnedPrior (snr_ceiling, combined persist)"
```

---

### Task 3: `LeadingSelector.select` — jump to the SNR target

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py`
- Test: `gs/tests/unit/test_dl_policy_leading.py`

- [ ] **Step 1: Write the failing test** — append to `gs/tests/unit/test_dl_policy_leading.py`:

```python
def test_loss_demote_jumps_to_target_in_one_move():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    mcs, changed = s.select(probe=_probe(7), loss_rate=0.3, loss_demote=True,
                            loss_demote_target=2, fec_pressure=0.0,
                            link_starved=False, ts_ms=99999.0)
    assert changed and mcs == 2          # 5 -> 2 in one commit, not 5 -> 4


def test_loss_demote_target_at_or_above_current_does_not_demote():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    mcs, changed = s.select(probe=_probe(7), loss_rate=0.3, loss_demote=True,
                            loss_demote_target=5, fec_pressure=0.0,
                            link_starved=False, ts_ms=99999.0)
    assert not changed and mcs == 5      # SNR says 5 is fine -> fluke loss, no demote


def test_loss_demote_cold_target_falls_back_to_one_step():
    s = _selector(max_mcs=5, promote_debounce_windows=1)
    _drive_to_mcs_probe(s, 5)
    mcs, changed = s.select(probe=_probe(7), loss_rate=0.3, loss_demote=True,
                            loss_demote_target=None, fec_pressure=0.0,
                            link_starved=False, ts_ms=99999.0)
    assert changed and mcs == 4          # cold SNR knee -> today's one-step demote
```

- [ ] **Step 2: Run it and verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_leading.py -q -k "jumps_to_target or at_or_above or cold_target"`
Expected: FAIL — `TypeError: select() got an unexpected keyword argument 'loss_demote_target'`.

- [ ] **Step 3: Implement.** In `gs/fpvdgs/dynlink/policy.py`, `LeadingSelector.select`:

(a) Add the parameter to the signature (after `loss_demote: bool = False,`):

```python
        loss_demote_target: int | None = None,
```

(b) Replace the loss branch:

```python
        if loss_demote:
            commit(prev - 1, f"video_per_demote loss={loss_rate:.3f}")
            self._reasons = reasons
            return (st.current_mcs, st.current_mcs != prev)
```

with:

```python
        if loss_demote:
            # Jump straight to the rung the live SNR supports (one move, no
            # overshoot). target None (cold SNR knee) -> today's one-step demote.
            if loss_demote_target is not None:
                tgt = min(prev, int(loss_demote_target))
                commit(tgt, f"video_per_demote loss={loss_rate:.3f} -> mcs{tgt}")
            else:
                commit(prev - 1, f"video_per_demote loss={loss_rate:.3f}")
            self._reasons = reasons
            return (st.current_mcs, st.current_mcs != prev)
```

- [ ] **Step 4: Run it and verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy_leading.py -q`
Expected: PASS (existing loss-demote tests unaffected — `loss_demote_target` defaults None ⇒ `prev − 1`).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_policy_leading.py
git commit -m "dynlink: reactive demote jumps to SNR target in one move"
```

---

### Task 4: `policy.tick` — wire SNR target + ingest + flight log

**Files:**
- Modify: `gs/fpvdgs/dynlink/policy.py`
- Test: `gs/tests/unit/test_dl_flightlog_debug_fields.py`

- [ ] **Step 1: Write the failing test** — append to `gs/tests/unit/test_dl_flightlog_debug_fields.py`:

```python
def test_record_carries_snr_norm_ceiling_knees(tmp_path):
    from fpvdgs.dynlink.signals import Signals
    p = Policy(_cfg(tmp_path, min_samples=3), _profile())
    # warm the SNR knee at MCS5 so snr_ceiling resolves
    for _ in range(12):
        p.learned_prior.ingest(rssi=None, snr=30.0, operating_mcs=5,
                               operating_clean=True, settled=True)
    sig = Signals(rssi=-50.0, residual_loss_w=0.0, fec_work=0.0,
                  link_starved_w=False, timestamp=1.0, snr=35.0)
    p.tick(sig)
    p.close()
    rec = _records(tmp_path)[-1]
    assert rec["snr_norm"] == 35.0
    assert rec["snr_ceiling"] == 5
    assert isinstance(rec["snr_knees"], list) and len(rec["snr_knees"]) == 8


def test_reactive_demote_jumps_to_snr_ceiling(tmp_path):
    from fpvdgs.dynlink.signals import Signals
    p = Policy(_cfg(tmp_path, min_samples=3), _profile())
    # SNR knee: rung1 viable at snr>=10, rung4 at >=30; current snr 12 -> ceiling 1
    for _ in range(12):
        p.learned_prior.ingest(rssi=None, snr=10.0, operating_mcs=1,
                               operating_clean=True, settled=True)
        p.learned_prior.ingest(rssi=None, snr=30.0, operating_mcs=4,
                               operating_clean=True, settled=True)
    p.leading.state.current_mcs = 5

    def sig(ts):
        return Signals(rssi=-60.0, residual_loss_w=0.30, fec_work=0.0,
                       link_starved_w=False, timestamp=ts, snr=12.0)

    p.tick(sig(1.0))            # loss count 1 (default loss_windows=2): no demote
    dec = p.tick(sig(1.1))      # loss count 2 -> sustained -> jump to snr_ceiling(12)=1
    assert dec.mcs == 1         # 5 -> 1 in one move, not 5 -> 4
    p.close()
```

(`_cfg(tmp_path, **lp)` in this file forwards `**lp` to `LearnedPriorConfig`, so only learned-prior kwargs like `min_samples` are valid there. The default `SelectorConfig` has `loss_windows=2` and `video_demote_per=0.05`, so two consecutive `loss=0.30` ticks trigger the sustained-loss demote — no selector override needed. The rssi knee stays cold here, so there is no predictive-demote or warm-start interference.)

- [ ] **Step 2: Run it and verify it fails**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_debug_fields.py -q -k "snr_norm_ceiling or jumps_to_snr"`
Expected: FAIL — `KeyError: 'snr_norm'` and the demote lands at 4 (target not wired).

- [ ] **Step 3: Implement.** In `gs/fpvdgs/dynlink/policy.py`, `Policy.tick`:

(a) Immediately BEFORE the `new_mcs, _changed = self.leading.select(` call, add:

```python
        loss_demote_target = self.learned_prior.snr_ceiling(signals.snr)
```

(b) In that `self.leading.select(...)` call, add the argument (next to `loss_demote=sustained_loss,`):

```python
            loss_demote_target=loss_demote_target,
```

(c) In the `self.learned_prior.ingest(` call, add the argument (next to `rssi=signals.rssi,`):

```python
            snr=signals.snr,
```

(d) In the `self.flightlog.write({...})` dict, after the existing `"snr": signals.snr_w,` line, add:

```python
            "snr_norm": signals.snr,
            "snr_ceiling": loss_demote_target,
            "snr_knees": self.learned_prior.snr_knees_snapshot(),
```

- [ ] **Step 4: Run it and verify it passes**

Run: `cd gs && .venv/bin/python -m pytest tests/unit/test_dl_flightlog_debug_fields.py tests/unit/test_dl_policy_learned.py -q`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/policy.py gs/tests/unit/test_dl_flightlog_debug_fields.py
git commit -m "dynlink: wire SNR-targeted demote into policy.tick + flight log"
```

---

### Task 5: Full suite green + offline 000017 validation

**Files:** full `gs/tests/` suite; ad-hoc validation script.

- [ ] **Step 1: Full suite**

Run: `cd gs && .venv/bin/python -m pytest tests/ -q`
Expected: PASS. If a test calls `ingest`/`select` positionally or asserts old persistence shape, fix it. Grep for risks first:

```bash
cd gs && grep -rn "\.ingest(\|\.flush()\|load_dict\|to_dict()" tests/ fpvdgs/ | grep -i "learned\|prior"
```

The signature changes are keyword-only with defaults, so breakage should be limited to tests that asserted the **flat** persistence doc shape (now wrapped in `{"rssi":..., "snr":...}`) — update those to read `doc["rssi"]`.

- [ ] **Step 2: Offline 000017 validation (go/no-go).** The log predates this change, so it has raw `snr` + `mcs` but no `snr_norm`; reconstruct `snr_norm = snr + (29 − curve[mcs])` and replay:

```bash
cd gs && .venv/bin/python - <<'PY'
import json
from fpvdgs.dynlink.learned_prior import LearnedPrior, LearnedPriorConfig
CURVE=[29,28,25,23,19,19,19,19]; PREF=29
recs=[json.loads(l) for l in open("/run/media/gilankpam/DVR/log/dynamic-link/000017.jsonl") if l.strip()]
i=0
while i<len(recs) and recs[i].get("rssi") is None: i+=1
recs=recs[i:]   # trim boot dead-air
p=LearnedPrior("val", LearnedPriorConfig(persist_dir="/tmp/snrknee-val", settle_ticks=5, min_samples=8))
def snrn(r):
    s=r.get("snr"); m=r.get("mcs")
    return None if s is None or m is None else s+(PREF-CURVE[min(m,7)])
# train the SNR knee from settled samples
last=None; stable=0
for r in recs:
    m=r.get("mcs"); sn=snrn(r); loss=r.get("residual_loss_w") or 0
    if m is None or sn is None: continue
    stable=stable+1 if m==last else 0; last=m
    p.ingest(rssi=None, snr=sn, operating_mcs=m, operating_clean=loss<0.05, settled=stable>=5)
print("learned SNR knees:", p.snr_knees_snapshot())
# replay each logged reactive demote as a jump-to-ceiling
import collections
crater=jump=fluke=cold=0; depth_old=collections.Counter(); depth_new=collections.Counter()
mcs=[r["mcs"] for r in recs]
for k in range(1,len(mcs)):
    if "video_per_demote" in (recs[k].get("reason") or ""):
        prev=mcs[k-1]; sn=snrn(recs[k])
        tgt=p.snr_ceiling(sn) if sn is not None else None
        old_step=prev-mcs[k]                       # how far the live algo stepped that tick
        new_to=min(prev,tgt) if tgt is not None else prev-1
        new_step=prev-new_to
        depth_old[old_step]+=1; depth_new[new_step]+=1
        if tgt is None: cold+=1
        elif new_step==0: fluke+=1
print("logged per-tick demote-step depths:", dict(sorted(depth_old.items())))
print("SNR-jump  demote-step depths:      ", dict(sorted(depth_new.items())))
print(f"fluke (no-demote, SNR ok)={fluke}  cold-fallback={cold}")
PY
```

Expected/go criteria: the SNR knees increase with rung; the SNR-jump step-depths show the demotes landing on a sensible target (not blind −1 every tick), and some fluke-losses demote 0. **Capture the output for the commit message.** If the knees look wrong or every jump is cold, STOP and reconsider before deploy.

- [ ] **Step 3: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add -A
git commit -m "dynlink: SNR-targeted demote suite green + 000017 replay

<paste SNR knees + step-depth comparison here>"
```

---

## Self-Review Notes (addressed)

- **Spec §2 (mechanism):** Task 3 (`min(prev,target)` jump, cold fallback). **§3 (SNR axis):** Task 1 (normalized `snr`), Task 2 (second `KneeModel`, combined persist + v2 back-compat). **§4 (integration):** Task 4 (target compute, ingest snr, flight-log fields). **§6 (testing):** every task TDD; Task 5 = offline replay.
- **Backward compatibility:** `ingest(snr=None)` and `select(loss_demote_target=None)` keep every existing caller/test working (cold ⇒ `prev−1`).
- **Interface consistency:** `ingest(rssi, snr=None, operating_mcs, operating_clean, settled)`, `snr_ceiling(snr)`, `snr_knees_snapshot()`, `select(..., loss_demote_target=None)`, `Signals.snr`, flight-log keys `snr_norm`/`snr_ceiling`/`snr_knees` — used identically across tasks.
- **Out of scope (spec §1):** EVM stays observe-only (no loop change); predictive demote stays on the RSSI model (untouched).
