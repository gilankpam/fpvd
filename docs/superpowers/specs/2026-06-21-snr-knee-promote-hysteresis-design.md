# SNR-knee promote/demote hysteresis (MCS-stuck-at-4 fix)

Date: 2026-06-21
Status: implemented (GS-only), tests green (416), verified against live device state; NOT yet flight-validated.

## Symptom

Live link pinned at MCS 4 with both GS and drone up. Probe rung 5 (current+1)
read dead clean and fresh (PER ≈ 5e-20, ageMs ≈ 34, `promote_clean` = 26 ≫
`promoteDebounceWindows` = 3); the RSSI ceiling allowed 5. Yet `snr_ceiling = 4`
and `promote_blocked = true` on 398/400 ticks (flight log 000011).

## Root cause

The SNR promote-veto is a **zero-margin, self-locking knife-edge.**

`KneeModel.rung_unviable(rung, snr)` returned `snr < knee[rung]` (strict). On the
device, rung 5's learned SNR knee was 36.064 dB (confident, decayed count 9.27 ≥
`minSamples` 8) while the live EIRP-normalized SNR sat at 35.998 dB — **0.066 dB
below the knee**, a noise-level margin. So the veto blocked the climb to 5.

It is self-perpetuating: the knee for rung K only relaxes downward from *clean,
settled samples taken while operating at rung K* (`KneeModel.observe`,
`clean and snr < knee` branch). The veto prevents operating at 5, so knee[5] can
never relax below the live SNR, and `recencyDecay` 0.9995 keeps the confidence
count above `minSamples` for ~1400 observations. The knee was seeded near the
operating SNR back when 5 was briefly reachable, the SNR settled just under it,
and the rung fenced itself off permanently. This is the same frontier deadlock as
the 2026-06-16 fix, but that fix only made *cold* (unlearned) rungs explorable;
once a rung is *confident* and the live SNR drifts to just below its knee, the
knife-edge re-locks it.

The probe — authoritative-for-promotes by design — was overridden by the learned
prior at a 0.066 dB whisker.

## Fix: two-margin hysteresis

`rung_unviable(rung, value, margin)` → `value < knee[rung] - margin`. The selector
passes asymmetric margins so the promote and demote gates form a stable dead-band
instead of a single oscillating edge:

- **Promote veto** (target rung): `snr_promote_margin_db` (Pm, default 1.0). Block
  the climb only when SNR is clearly below the target knee.
- **Proactive demote** (current rung): `snr_demote_margin_db` (Dm, default 1.5).
  Bail ahead of loss only when SNR is clearly below the current knee.
- Invariant `Dm > Pm` (schema-enforced) opens the dead-band: promote allowed at
  `knee - Pm`, demote at `knee - Dm`, so a rung climbed onto is not immediately
  yanked back. At the 4↔5 boundary the band is [knee5 − 1.5, knee5 − 1.0] =
  [34.56, 35.06]; the live 35.998 is above it → climbs to 5 and holds.

The landing `snr_ceiling` stays **strict** (no margin) so a real-loss reactive
demote still drops to the rung the SNR actually supports. The proactive-demote
margin (Dm 1.5 < the 1.8 dB 4→5 knee gap) keeps it a clean one-rung step.

Self-correction is restored: once operating at 5 again, clean settled samples
relax knee[5] toward the true floor; if the probe was optimistic and 5 is bad,
the reactive/loss demote backstops and the dirty sample tightens the knee.

Verified against the exact persisted device knees: old code → veto True (ceiling
4); new defaults → promote veto False, proactive-demote False, and a 3 dB fade
still demotes True.

## Alternatives rejected

- **Delete the SNR promote-veto / "probe always wins."** Loses the overshoot
  guard the knee model added (the probe measures rung+1 at the current rung's TX
  power — ~4 dB optimistic at the 3→4 boundary), and *still* needs a demote-side
  margin or it flaps promote↔proactive-demote. Strictly worse.
- **Single shared margin.** Just relocates the knife-edge from `knee` to
  `knee - margin`; no dead-band → still oscillates at the new edge.
- **Fix the knee seeding bias.** Higher-risk and unnecessary: `minSamples`
  confidence gating already washes out the raw seed, and the margin + relax path
  self-corrects an inflated knee once the rung is reachable again.

## Knobs (dynamicLink.selector, flight-tunable)

- `snrPromoteMarginDb` (default 1.0)
- `snrDemoteMarginDb` (default 1.5, must be > promote)

Defaults are baked into `SelectorConfig`, so a code redeploy unsticks the live
link without any config.json change. GS-only — the drone has no SNR knee.

## Next

Deploy to GS, confirm the link climbs 4→5 and holds, then flight-validate the
dead-band widths (Pm/Dm) and watch for any new flap or late proactive-demote.
