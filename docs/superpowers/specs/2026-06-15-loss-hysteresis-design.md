# Reactive Loss-Demote Hysteresis + Unified Loss Path Design

**Date:** 2026-06-15
**Status:** Draft for review
**Target:** GS `fpvdgs.dynlink` (GS-local; no drone change, no wire change)
**Refines:** the reactive demote path in `policy.py` (`LeadingSelector.select` / `Policy.tick`)
**Follows:** `2026-06-15-predictive-demote-flapping-fix-design.md` (same "don't react to single-sample noise" principle, applied to the reactive loss path)
**Diagnosed from:** flight log `000014.jsonl` (the fix's own fly-out)

---

## 1. Purpose & scope

The reactive **loss** demote fires on a **single** breaching window. `Policy.tick` passes
`loss_rate=signals.residual_loss_w` raw (no smoothing), and `select()` demotes a rung the
moment `loss_rate >= emergency_loss_rate` (0.05). A lone transient loss window — a multipath
null, one dropped burst, an antenna switch — costs a rung needlessly.

This adds **consecutive-window hysteresis** to the loss path (mirroring the existing
`starvation_windows` hysteresis) and **unifies** the two redundant loss thresholds
(`emergency_loss_rate` and `video_demote_per`, both 0.05) into one gated loss-demote
decision. Pure GS-side; no wire/drone change.

After this change every reactive trigger has appropriate noise rejection:

| trigger | noise rejection |
|---|---|
| **loss** | consecutive-window hysteresis (**new**, `loss_windows`) |
| **fec** | EWMA-smoothed (`fec_work`, existing) |
| **starvation** | consecutive-window hysteresis (`starvation_windows`, existing) |

**Out of scope (separate follow-up):** the `emergency_fec_pressure = 0.80` trigger is
effectively dead — `fec_work` never exceeded ~0.07 in any flight, so it never fires. Left
untouched here.

## 2. Evidence (from `000014`)

The reactive loss path produced 62 loss episodes; the run-length distribution shows most are
transient:

```
1-window episodes (pure transient): 27   ← a single 100 ms blip
2-window:                            15
3-window:                             9
4+ windows (sustained, real):        11
```

Simulating a consecutive-window requirement of depth N on the actual log:

| `loss_windows` (N) | loss-demote events | reduction |
|---|---|---|
| 1 (today) | 62 | — |
| **2 (200 ms)** | **35** | **−44%** |
| 3 (300 ms) | 20 | −68% |
| 4 (400 ms) | 11 | −82% |

The "alternating" gaps between episodes were **all truly-clean windows (<2.5%), none
near-threshold** — i.e. the link genuinely *recovered* between distinct bursts, not flapping
around the threshold. So strict **consecutive** is the correct model (it treats
recover-then-fail as separate episodes); "M-of-N" would only over-merge real recoveries and
barely differs (2-of-3 = 29 vs consecutive-2 = 35).

Separately, 26/36 committed loss demotes occurred within 10 ticks **after** a probe-promote
(over-promotion into a marginal rung) — that's the *promote* side and is **not** addressed
here; it's a future lever.

## 3. Locked design decisions (from brainstorming)

- **Mechanism: consecutive windows** (matches `starvation_windows`; data shows no
  threshold-flapping that would justify M-of-N).
- **Default `loss_windows = 2`** (200 ms, −44%) — lowest risk to video; only filters pure
  single-window transients. Tunable from flight logs.
- **Unify the loss paths**: remove `emergency_loss_rate`; `video_demote_per` (0.05) is the
  single loss threshold. `_emergency_active` becomes fec-or-starved.
- **Add a `loss_gated` diagnostic** (mirrors `predict_gated`) so the next flight can quantify
  suppressed loss demotes.
- **FEC threshold deferred** (§1).

## 4. The changes

### 4.1 Loss hysteresis (Change 1)

In `Policy.__init__`, add `self._loss_count: int = 0`. In `Policy.tick`, mirror the
starvation block:

```python
# Loss hysteresis: residual_loss_w is raw and spikes on a single bad window
# (multipath null, one dropped burst). Require N consecutive breaching windows
# before demoting — 27/62 in-flight loss episodes were single-window transients.
if signals.residual_loss_w >= self.cfg.selector.video_demote_per:
    self._loss_count += 1
else:
    self._loss_count = 0
sustained_loss = self._loss_count >= self.cfg.selector.loss_windows
loss_gated = (signals.residual_loss_w >= self.cfg.selector.video_demote_per
              and not sustained_loss)
```

New config field `SelectorConfig.loss_windows: int = 2` (camelCase `lossWindows`).

**Key property:** the counter only delays the **first** demote of an episode. Once
`sustained_loss` latches, every subsequent breaching window still demotes — a real cliff
loses only `loss_windows × 100 ms` before the first rung-drop, then demotes at full speed.
A transient shorter than `loss_windows` never demotes (the win).

### 4.2 Unify the loss paths (Change 2)

`select()` gains a `loss_demote: bool` param (= `sustained_loss`) and the loss check is
separated from the fec/starved emergency:

```python
# loss demote (hysteresis-gated upstream); reactive, one-step, bypasses promote rate limit
if loss_demote:
    commit(prev - 1, f"video_per_demote loss={loss_rate:.3f}")
    self._reasons = reasons
    return (st.current_mcs, st.current_mcs != prev)
# emergency: fec pressure or sustained starvation (loss removed)
if fec_pressure >= self.cfg.emergency_fec_pressure or link_starved:
    commit(prev - 1, f"emergency fec={fec_pressure:.3f} starved={link_starved}")
    self._reasons = reasons
    return (st.current_mcs, st.current_mcs != prev)
```

- **Remove** `SelectorConfig.emergency_loss_rate`; `_emergency_active` is simplified to
  `fec_pressure >= emergency_fec_pressure or link_starved` (or inlined as above).
- `Policy.tick` passes `loss_rate=signals.residual_loss_w` (raw, for the reason text only)
  and `loss_demote=sustained_loss`.
- **Reason strings now cleanly separate causes:** loss → `video_per_demote loss=…`,
  fec/starved → `emergency fec=… starved=…`. (Today loss masquerades as `emergency loss=…`
  because the emergency branch intercepts first.) Both still match the analyzer's reactive
  counter (`"video_per_demote" or "emergency"`).

### 4.3 Diagnostic (Change 3)

Add `"loss_gated": loss_gated` to the flight-log record (true when loss breached but
hysteresis suppressed the demote). Add a `loss-gated demotes:` count to
`gs/tools/flightlog_analyze.py` (mirrors `predict_gated` / `gated_demotes`).

### 4.4 Config migration

`selector` is a `config.json` block read field-by-field in `config_build.py`. The live GS
`config.json` has `emergencyLossRate: 0.05` (= `videoDemotePer`).

- Add `lossWindows` read: `sel.get("lossWindows", d.loss_windows)`.
- Drop the `emergencyLossRate` read. On the next deploy, the existing `emergencyLossRate`
  key in `/etc/fpvd/config.json` becomes unknown → the tolerant loader **strips it with a
  warning** (no behavior change, since its value equals `videoDemotePer`).
- Update `gs/fpvdgs/config_defaults.py` (add `lossWindows`, drop `emergencyLossRate`).

## 5. What stays the same

The promote path, predictive demote, probe, learned prior, starvation hysteresis, FEC
threshold, the wire, and the drone are all unchanged. No `config.json` schema change beyond
the `selector` block field swap (handled by the existing tolerant loader).

## 6. Testing (TDD)

**Loss hysteresis (`test_dl_policy_*`):**
- 1 breaching window → **no** demote (MCS held)
- 2 consecutive breaching windows → demote
- breach, then a clean window, then breach → no demote (counter reset; consecutive only)
- sustained loss (≥ `loss_windows`) → demotes on each subsequent window (latched)

**Unify:**
- fec breach (`fec_work ≥ emergency_fec_pressure`) → demote with `emergency` reason
- sustained starvation → demote with `emergency` reason
- loss demote → `video_per_demote` reason (not `emergency`)
- config load with a stale `emergencyLossRate` key → loads clean (stripped), no crash
- `lossWindows` read from the `selector` block; default 2 when absent

**Diagnostic:**
- `loss_gated` true when loss breached but `_loss_count < loss_windows`; false otherwise
- analyzer reports `loss-gated demotes`

**Suite:** update `test_dl_config_build.py` and `test_dl_policy_leading.py` references to
`emergency_loss_rate`; whole GS suite green.

## 7. Defaults summary

| knob (`SelectorConfig`) | default | meaning |
|---|---|---|
| `loss_windows` (`lossWindows`) | `2` | consecutive ≥5% residual-loss windows before a loss demote (200 ms @ 10 Hz) |
| `video_demote_per` (`videoDemotePer`) | `0.05` | the single loss-demote threshold (unchanged value; now the only loss knob) |
| ~~`emergency_loss_rate`~~ | — | **removed** (unified into `video_demote_per`) |
| `emergency_fec_pressure` | `0.80` | unchanged (dead trigger; separate follow-up) |
| `starvation_windows` | `5` | unchanged |
