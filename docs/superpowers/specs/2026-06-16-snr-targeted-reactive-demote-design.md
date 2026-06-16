# SNR-Targeted Reactive Demote

**Date:** 2026-06-16
**Status:** Draft for review
**Target:** GS `fpvdgs.dynlink` (`signals.py`, `learned_prior.py`, `policy.py`). GS-local; no drone change, no wire change.
**Builds on:** `2026-06-16-learned-prior-knee-model-design.md` (the per-rung RSSI knee model) and the SNR/EVM flight-log instrumentation (`feat/snr-evm-flightlog`).
**Diagnosed from:** flight `000017.jsonl` — first flight on the knee model with the freeze removed.

---

## 1. Purpose & scope

The reactive (loss-triggered) demote drops **one blind rung per tick**. A single loss burst therefore fires repeatedly and craters the rung to the floor (in `000017`, 17 of 23 demotes were multi-rung, including seven 5→1 craters), then the link climbs back. This is **overshoot** — the demote lands on the wrong rung because it steps blindly instead of targeting the rung the channel actually supports.

The earlier demote-settle "freeze" treated this by *slowing* demotes. That is wrong: if the link needs to demote, it should demote **immediately** — the defect is the *target*, not the *speed*. This design makes the reactive demote **jump directly to the rung the live link quality supports, in one move** — fast and overshoot-proof.

**The signal is SNR.** It is the only available signal that is both *cross-rung* (measured at the current rung, it predicts other rungs' viability) and *interference-aware* (`SNR = signal − noise floor`, so a co-channel hit that RSSI cannot see drops SNR). The 000017 AUC analysis confirms SNR is the robust loss predictor across the range (overall 0.84; 0.79 at the marginal rungs where the craters occurred). RSSI misses interference; EVM is sharp at high MCS but does not map cross-rung, so it cannot pick a target rung.

**Out of scope (deferred):**
- **EVM in the control loop** — EVM stays observe-only (logged). It is the future *proactive* high-MCS interference trip (a separate change); it cannot pick a demote target.
- **Migrating the predictive demote to SNR** — predictive demote stays on the validated RSSI knee. If SNR proves the better axis over more flights, migrate later.
- **A stuck-lossy-with-good-SNR backstop** — if loss persists while SNR says the current rung is fine, v1 demotes nothing (trusts SNR). Recorded as a risk; add a backstop only if flights show it.

## 2. Mechanism

On a caller-hysteresis-gated sustained loss, replace `current − 1` with a single jump to the SNR-supported ceiling:

```
target = snr_ceiling(snr)                 # highest rung the live SNR supports, or None if cold
if target is not None:  commit(min(current, target))   # down-only, one move
else:                   commit(current − 1)             # cold fallback (today's behaviour)
```

- `min(current, target)` is **down-only** — loss never promotes (the probe owns promotes).
- **Overshoot-proof:** a genuine multi-rung drop happens in one tick; a fluke loss with healthy SNR (`target ≥ current`) demotes **nothing**.
- **No timer, no settling.** A genuine fade descends as fast as the channel requires.
- The emergency path (FEC pressure / sustained starvation) and the predictive path are unchanged.

## 3. The SNR axis (extending the knee model)

SNR becomes a second, parallel knee axis; the RSSI knee model is untouched.

**EIRP-normalize SNR (`signals.py`).** `snr_norm_w = snr_raw + (P_ref − curve[mcs])` — the *same* offset RSSI already uses (SNR scales 1:1 with TX power, noise is unchanged), reusing `RssiNormConfig` (curve, `P_ref`, `enabled`). Then `s.snr = _ewma(s.snr, snr_norm_w, ewma_alpha_rssi)` — a smoothed, normalized, cross-rung-comparable control signal, mirroring `s.rssi`. Raw `snr_w` is retained for logging.

**Second `KneeModel` (`learned_prior.py`).** `LearnedPrior` gains `_snr_model = KneeModel(cfg)` alongside the RSSI one. The class is already signal-agnostic (`observe(rung, value, clean)`), so the SNR model learns per-rung SNR knees with the same settle gate, pessimistic asymmetric pull, recency decay, and confidence gate. `snr_ceiling(snr) = _snr_model.ceiling(snr)` (None when cold or `snr` is None).

**Persistence.** Combined doc `{"key", "rssi": <model dict>, "snr": <model dict>}`. Load is back-compatible: a deployed v2 flat doc (RSSI-only, from the just-deployed knee model) loads as the RSSI model with the SNR model cold — the GS's freshly-learned RSSI knees survive the upgrade; SNR learns from zero.

## 4. Integration (`policy.py`)

- `ingest(rssi, snr, operating_mcs, operating_clean, settled)` now feeds **both** models (each None-guarded) on a settled tick.
- `tick` computes `loss_demote_target = learned_prior.snr_ceiling(signals.snr)` and passes it to `LeadingSelector.select(...)`.
- `select` loss branch: `commit(min(prev, target))` when `target` is not None, else `commit(prev − 1)`. Everything else (emergency, predictive, promote, rate-limit) is unchanged.
- Flight log adds `snr_norm` (the control SNR), `snr_ceiling`, and `snr_knees` (the SNR-model snapshot), so the next flight can be validated the same way RSSI knees were.

## 5. Components & boundaries

- **`SignalAggregator`** — computes `snr` (smoothed normalized) alongside `rssi`. One new field + one EWMA line.
- **`KneeModel`** — unchanged; instantiated twice (RSSI, SNR).
- **`LearnedPrior`** — owns both models; `ingest` feeds both; adds `snr_ceiling` + `snr_knees_snapshot`; combined persistence with v2 back-compat.
- **`Policy` / `LeadingSelector`** — `tick` computes the target; `select`'s loss branch jumps to it. No other control logic changes.

## 6. Testing (TDD)

- **signals:** `snr_norm` = raw + correct EIRP offset; `snr` EWMA; norm-disabled ⇒ raw.
- **LearnedPrior:** `ingest` feeds both models; `snr_ceiling` reflects the SNR knees; combined-doc round-trip; **v2 back-compat** (rssi preserved, snr cold).
- **LeadingSelector.select:** loss + target jumps to `min(prev, target)` in one commit (assert a 5→2 jump, not 5→4); `target ≥ prev` ⇒ no demote (fluke case); cold target ⇒ `prev − 1` fallback; emergency path still `prev − 1`.
- **Policy:** reactive demote lands on `snr_ceiling`; cold fallback; flight log carries the new fields.
- **Offline validation (go/no-go before deploy):** replay `000017` — build the SNR knee from its settled samples, re-decide each reactive demote as a jump to `snr_ceiling`, and measure how many of the 17 multi-rung craters collapse to a single correct jump, and whether fluke-losses demote zero rungs.

## 7. Risks

- **SNR-knee cold start.** Until the SNR knees are confident, the reactive demote falls back to `current − 1` (today's behaviour) — no regression, just no improvement yet. It warms within a flight (the RSSI knee did).
- **Stuck-lossy-with-good-SNR.** If loss persists while `snr_ceiling ≥ current`, v1 demotes nothing. Trusting SNR is the intent; monitor for a stuck-lossy case and add a one-rung backstop if observed.
- **EWMA lag.** `snr` is smoothed, so a very sudden interference hit reads with ~1–2 windows of lag; the loss trigger already requires 2 windows, so the jump still reflects the drop. Raw-window SNR was rejected to avoid single-window-noise mis-jumps.
- **SNR-axis validity vs RSSI.** This trusts that EIRP-normalized SNR knees are at least as good a viability predictor as RSSI knees. The offline 000017 replay is the gate; if the jumps look wrong, revisit before deploy.
