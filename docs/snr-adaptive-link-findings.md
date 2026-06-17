# SNR-Driven Adaptive Link — Improvement & Debugging Session

**Date:** 2026-06-16 (single bench + flight session)
**Status:** Implemented + deployed (knee model, SNR-targeted demote, SNR-ceiling fix, flight-log instrumentation). **Live flight-validation of the SNR-ceiling fix still PENDING** (offline-replay-validated only).
**Hardware:** GS aarch64 (OpenIPC), 2× RTL8812EU dongles (`rtl88x2eu`) in diversity; drone OpenIPC SSC338Q; channel 132 (5660 MHz) / 20 MHz, STBC; wfb-ng swfec fork (EVM exposed).
**Code:** all GS-side `gs/fpvdgs/dynlink/` on branch `feat/learned-prior-knee-model` (PR #22). Specs/plans under `docs/superpowers/{specs,plans}/2026-06-16-*`. Analysis tooling: `gs/tools/flightlog_analyze.py`.

---

## TL;DR

We rebuilt the GS adaptive-link control loop across one session, driven by flight logs. The single throughline:

> **Don't dampen or delay a demote — demote to the *correct* rung, immediately, using the signal that actually predicts viability.**

Concretely: replaced the selection-biased binned learned prior with **per-rung viability "knee" models**; made the reactive demote **jump to the rung the live SNR supports** (instead of stepping one blind rung per tick); made the **SNR ceiling authoritative over the probe's optimism** (which killed a 4↔3 oscillation); and instrumented **SNR + EVM** so the signal choices were data-driven, not guessed.

Every step was validated by **offline replay against the real flight log before deploy**.

---

## The arc (chronological)

| # | Flight(s) | What we found → what we did |
|---|---|---|
| 1 | 000007–10 | Reactive demote stepped 1 blind rung/tick → a single ~0.4 s loss burst cratered the rung to the floor (50–77% of demotes multi-rung), then climbed back. Diagnosed as **overshoot**. |
| 2 | 000013 | Built a **demote-settle "freeze"** (rate-limit demotes). Cut multi-rung cascades 77%→11%. *Felt* like a win. |
| 3 | — | **User pushback:** the freeze slowed *genuine* descents and ignored the real problem — the link sat at MCS4 when it should be at 1. Root cause: the **learned prior flat-extrapolated the ceiling** below its lowest confident RSSI anchor, so the predictive demote was **dead on fades**. The freeze was treating a symptom. |
| 4 | 000013 (replay) | **Knee-model redesign:** per-rung RSSI knee (`knee[K]` = RSSI below which rung K is unviable), monotone-by-construction, recency-weighted, **learns only from settled rungs**. Replay: predictive-demotes off MCS4 across the −69→−80 fade the old prior rode at 4. Shipped (subagent-driven TDD). |
| 5 | — | Instrumented **SNR + EVM** into every flight-log record (observe-first). |
| 6 | 000017 | Removing the freeze brought the **cascades back** (17/23 multi-rung). But **EVM validated** as a loss predictor (AUC 0.96–0.98 at MCS4 — catches what RSSI can't). Also: the first 85 s was **boot dead-air**, not algorithm. |
| 7 | 000017 | SNR-vs-EVM AUC: **SNR robust across the range** (0.84 overall), **EVM sharpest at high MCS** (0.96) but collapses low and can't map cross-rung. |
| 8 | 000017 (replay) | **SNR-targeted reactive demote:** jump in one move to `min(current, snr_ceiling)` (a 2nd, EIRP-normalized SNR knee axis). No timer. Replay: **45% of demotes were avoidable flukes** the new logic skips; the rest land correct in one move. Shipped + deployed. |
| 9 | 000018 | **4↔3 oscillation every ~3 s** (66/94 changes). The **probe promoted to MCS4 against the SNR ceiling** — 88% of 3→4 promotes had `snr_ceiling=3` — ate loss, demoted back. |
| 10 | 000018 (replay) | **SNR ceiling made authoritative both directions:** cap the probe promote at `snr_ceiling` + a debounced **proactive SNR demote** before loss + `lossWindows` 2→1. Replay: 4↔3 flaps **66→7**, loss-prone-at-MCS4 ticks **134→4**. Shipped + deployed. |
| 11 | — | **Two flights' logs vanished.** Cause: `flightlog` never `fsync`'d **and** the GS hard-reboots on video loss (loops on the bench when the drone is off) → unsynced vfat writes wiped. Added `fsync`. |

---

## What works (validated)

- **Per-rung knee model.** A monotone, recency-weighted viability curve learned per rung beats the binned table: no flat-extrapolation hole, no selection-biased non-monotonicity. Predictive-demotes correctly on fades (000013 replay).
- **SNR as the operating-ceiling axis.** It's the only signal that's both **cross-rung** (measure at one rung, predict another's viability) and **interference-aware** (`signal − noise`, so co-channel interference RSSI can't see drops it). Jump-to-`snr_ceiling` is **overshoot-proof** (000017 replay: 45% flukes avoided).
- **SNR ceiling capping the probe.** The probe is structurally optimistic; gating its promote at `snr_ceiling` kills the 4↔3 oscillation at its source (000018 replay: 66→7 flaps). Bonus: it also *improves* the knee learning (stops feeding it dirty "promoted-then-lost" samples).
- **Learn only from settled rungs.** Rejects the fast-fade / mid-cascade transients that otherwise poison the knees.
- **EVM as a high-MCS loss predictor.** AUC 0.96–0.98 at MCS4 — the sharpest boundary detector when the link is good (the co-channel case).
- **Process:** instrument-first; offline replay against the real log as the go/no-go before *every* deploy; subagent-driven TDD with two-stage review (the final holistic review caught two real bugs the per-task reviews missed — config knobs being silently stripped by the loader, and a prior-file boot-brick).

## What doesn't (dead ends + open issues)

- **The demote-settle "freeze" — wrong framing, abandoned.** Slowing a demote is the wrong lever; the defect was the *target* rung, not the *speed*. (It did cut cascades, but at the cost of slowing genuine descents and never addressing the optimism.)
- **The flat-extrapolation band-aid — abandoned** for the knee redesign (it only patched how the bad prior was *read*, not how it *learned*).
- **RSSI-keyed predictive demote alone — insufficient.** It's slope-gated, so at a hovering-RSSI boundary (the 4↔3 zone) it's suppressed and can't predict the demote. SNR was needed.
- **EVM as a *target-picker* — can't.** No cross-rung mapping (constellation-density per rung), so it can't say "jump to rung 3." It stays a same-rung sentinel.
- **GS reboot-loop on video loss (system/firmware, outside fpvd).** On the bench with the drone off, the watchdog reboots every ~minute, corrupting the vfat DVR card and making deploys land on a reboot. **Workaround: keep the drone powered.** Taming the watchdog is a separate, unstarted task.
- **Live validation of the SNR-ceiling fix is still pending** — only offline-replay-validated, blocked on a stable GS.

---

## What we learned (durable principles)

1. **Demote to the right rung, fast — don't dampen.** Overshoot (wrong target) was the problem, not demote speed. A signal-targeted one-move jump beats both blind stepping *and* timer-based settling.
2. **Treat the cause, not the symptom.** The demote cascade was a *symptom* of operating-rung optimism (the learned prior). Two fixes (freeze, extrapolation patch) failed because they patched symptoms.
3. **The probe is structurally optimistic.** FEC-off, observe-only, and it measures rung+1 at the *current* rung's TX power — so it reads clean and over-promotes. A real viability signal (SNR) must **cap** it, or you get oscillation.
4. **Pick the signal for the job:**
   - **SNR** — cross-rung *and* interference-aware → the **operating-ceiling axis** (caps promote, picks demote target).
   - **EVM** — sharper at high MCS, but **not cross-rung** → a same-rung **sentinel**, not an axis.
   - **RSSI** — misses interference (sees signal, not noise) → likely **redundant** once SNR drives the loop (candidate to retire).
5. **EVM is per spatial *stream*, not per antenna.** The Realtek driver mislabels it. On Nss=1 the 2nd-stream slot is a `-1`/`0` sentinel; STBC duplicates both slots. **Never naive-average the per-"antenna" EVM** — group per dongle (`ant>>8`), drop sentinels, take the real stream value, combine across dongles. The meaningful spread is *between* dongles.
6. **Normalization depends on use, not signal:** a **cross-rung axis** needs EIRP normalization (add back the per-MCS TX-power offset — SNR scales 1:1 with TX power like RSSI); a **same-rung** judgment needs none. **EVM can't be linearly normalized** (it's a nonlinear quality metric) → use per-rung thresholds.
7. **Learn only from settled state.** Transient (mid-fade) and oscillation (promoted-then-lost) samples poison a learned model; gate learning on the rung being stable.
8. **Instrument before designing the algorithm.** The SNR-vs-EVM *roles* were decided by AUC on real flight data, not intuition. Don't design a control change around a signal you haven't measured.
9. **Offline replay is the cheap go/no-go.** Reconstruct the decision from the logged signals and re-decide under the new rule *before* touching hardware. It caught the cascade-reduction, the fluke rate, and the flap collapse without a flight.
10. **Make telemetry durable.** A log that isn't `fsync`'d is lost on a hard reboot — and this GS hard-reboots a lot. Sync periodically + on roll.

---

## Open items / next steps

- **Flight-validate the SNR-ceiling fix live** (4↔3 flap collapse, `snr_demote` reasons firing) — needs a stable GS (drone powered).
- **Decide RSSI knee retirement** — SNR is a strict superset for viability; keep RSSI 1–2 flights as a fallback/cross-check, then drop to a single axis.
- **EVM proactive trip (deferred)** — at high MCS, EVM (AUC 0.96+) is the sharper detector; a same-rung EVM sentinel could pre-empt the brief loss before SNR catches it. Layer on *only if* residual flap survives the SNR-ceiling fix.
- **Tame the reboot-on-video-loss watchdog** (separate task; lives in OpenIPC firmware, not fpvd) — it makes bench iteration painful and corrupts the DVR card.

---

*Related: `docs/superpowers/specs|plans/2026-06-16-learned-prior-knee-model-*`, `...-snr-targeted-reactive-demote-*`; PR #22; flights 000007–000018.*
