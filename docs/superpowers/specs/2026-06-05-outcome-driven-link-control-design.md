# Outcome-Driven Link Control — MCS, Bitrate, and FEC from Measured Outcomes

**Date:** 2026-06-05
**Status:** Draft for review
**Target:** OpenIPC drones (Sigmastar SSC33x, armhf) + GS, fpvd codebase; RTL8812EU link
**Extends:** `2026-05-27-dynamic-link-design.md`, `2026-06-01-dynamic-link-fold-in-design.md`, `2026-06-02-gs-dynamic-link-fold-in-design.md`
**Supersedes:** the SNR-floor MCS selector, the airtime-model bitrate path, and the loss-driven dynamic-FEC escalator in the specs above

---

## 1. Purpose

Replace the adaptive-link controller's **predicted** decision signals with **measured outcomes**.

Today's controller selects MCS against a hand-authored `snr_floor_dB` ladder and sizes bitrate from a fixed-constant airtime model. Live measurement on the deployed hardware (§2) shows both are unsound: the driver's SNR is an EVM-saturated, survivor-biased metric the floor table was never calibrated against, and the airtime constants are 3.6–22× off across the real operating range.

This redesign makes each control loop decide on a signal it can actually trust, measured where that signal physically lives:

- **MCS** selects on **on-air packet-error-rate** (outcome), with a single-step probe to climb. SNR drops out of the control path (observed/logged only); RSSI is a cold-start prior.
- **Bitrate** is a discrete ladder driven by **queue congestion** (`fill_pct`), backing off on latency *before* loss.
- **FEC** ratio and interleaver depth become **constant** — MCS now owns the loss response, so adaptive FEC would double-react.
- **Capacity** per MCS is **measured and remembered** on the drone, not predicted.

A second outcome: the control loops decouple. Each closes locally on its nearest signal, and the only thing crossing the radio for control is the MCS command.

## 2. Background — measured evidence

All figures below were captured live this session on the deployed drone (`192.168.10.152`) + GS (`10.18.0.1`), RTL8812EU (`0bda:a81a`, driver `rtl88x2eu`), MCS0/20 MHz, channel 161.

### 2.1 The SNR metric and `snr_floor_dB` are unsound

- **SNR saturates.** A clean TX-power sweep (controller frozen) moved received RSSI 27 dB (−71 → −44 dBm) while the driver's `snr_avg` moved only 5 dB (21 → 26) and **ceilinged at ~26 dB**. The `snr_avg − rssi` offset is not constant — it shrinks from ~92 (weak) to ~70 (strong). The metric is EVM/AGC-limited, not thermal-SNR. At close range (high-MCS decisions) it has almost no dynamic range.
- **Survivor bias.** `snr_avg`/`rssi_avg` average only *decoded* packets, so they read optimistically at the margin (two clean runs disagreed by ~10 dB at the lowest power).
- **The floor table is synthetic.** `snr_floor_dB` is a uniform 3-dB-per-MCS ladder (40 MHz = 20 MHz + 3 exactly) — a theoretical AWGN table, never calibrated to this metric or hardware.

**Conclusion:** calibrating `snr_floor` can't fix it — saturation is a *resolution* problem, not an accuracy one, and the high-MCS thresholds sit in the saturated band. RSSI keeps full resolution across the same sweep and is the better discriminator.

### 2.2 The airtime / bitrate model is wrong

- `predictor.PredictorConfig.per_packet_airtime_us = 80` is correct only at MCS7/HT40; at the actual MCS0–5/20 MHz operating range, real per-1400 B airtime is **3.6–22× larger**.
- `profile.preamble_us_per_frame = 170` (and `DEFAULT_PREAMBLE_US = 200`) is a lumped fudge mislabeled as a PHY preamble; it derates wire capacity ~2×.
- `data_rate_Mbps_LGI` *is* correct — it's the 802.11n HT standard rate table (verified; the link runs long-GI, `shortGi=false` hardcoded in `drone/src/dynlink/controller.cpp`). But it is **not card-specific** and should be a built-in constant, not a profile field.

### 2.3 The honest, non-survivor-biased loss signal

On-air PER = `(lost + fec_rec) / (out + lost)` = `residual_loss_w + fec_work_rate_w` (both already in `signals.py`). This un-masks FEC (a rate barely surviving on redundancy reads as failing, not as 100% delivered) and counts the packets that *didn't* arrive, so it is not survivor-biased. This is the MCS selector's outcome signal.

### 2.4 Congestion is latency, never drops; `fill_pct` marks the capacity knee

Forcing CBR 5 Mbps at MCS0 (over the ~4 Mbps capacity):

| | uncongested | congested |
|---|---|---|
| frame rate | 43 Hz | 32 Hz (throttled) |
| sender pipeline latency | 0.2 ms | 59 ms (encode 34 + pktz 26) |
| `fill_pct` | 0 | 65 % |
| link RTT (GS, NTP sync) | 25 ms | **370 ms** |
| `transport_drops` | 0 | **0** |

The standing queue backpressures into latency (15× link-RTT inflation) with **zero drops**. A bitrate sweep showed `fill_pct` flat at 0 under capacity, rising sharply through the **knee at ~4–5 Mbps**, then plateauing (~65%) when over — a **threshold signal, not a gradient**, which is exactly what a discrete ladder needs. Encoder water-marks are `VENC_PRESSURE_HIGH_WATER_PCT=75`, `LOW=50`. Note `inPressure` (≥75%) never fires at the 65% equilibrium, and `transport_drops` is sidecar-only — so the controller reads live `fill_pct` and applies its own threshold.

Caveat: VBR makes *actual* bits scene-dependent (a static scene produced ~1.45 Mbps against a 5 Mbps cap), so capacity-probing only registers when the scene is producing near-capacity bits.

### 2.5 The waybeam sidecar already exposes everything needed

The venc (`waybeam_venc`) runs an RTP timing/telemetry **sidecar on UDP `:5602`** (`include/rtp_sidecar.h`). Per encoded frame it emits congestion (`fill_pct`, `transport_drops`, `pressure_drops`), encoder state (frame type, size, QP, scene_change, IDR), and timestamps; it also answers NTP-style clock-sync. **Constraint: single subscriber** (`RtpSidecarSender` holds one `subscriber`, last-SUBSCRIBE-wins). `fill_pct` is also on `GET /api/v1/transport/status` (port 80). fpvd does not consume any of it today.

### 2.6 The drone already detects its adapter

`drone/scripts/radio-up.sh` maps the USB ID to an adapter id (`0bda:a81a → bl-m8812eu2`). The GS ignores this and loads a static `radioProfile`.

## 3. Scope

**In scope:** outcome-driven MCS selection (GS), drone-side rate engine (bitrate ladder + constant FEC/depth + learned capacity), sidecar consumption (drone, loopback), removal of the SNR-floor/airtime/dynamic-FEC machinery, hardware-derived profile selection.

**Out of scope (YAGNI / deferred pending test evidence):**
- **Reach-probe** (multi-step MCS climb) — start single-step; add only if testing shows climb-lag.
- **SNR-collapse interference veto** — interference manifests as loss, which Channel-B already catches; add only if testing shows pre-loss glitches.
- **RSSI/MCS-coupled tx_power loop** — run constant power; revisit as a standalone experiment (does lowering power at close range de-compress the RX and raise effective SNR?).
- **All-rate Minstrel probing** — the monotonicity prior makes single-step boundary probing sufficient.
- **Per-MCS FEC ratio**, **dynamic depth** — single constants for now.
- Changes to the wfb wire format / FEC codec / radiotap.

## 4. Architecture

### 4.1 Division by signal locality

Each loop lives where its signal physically is:

| Loop | Lives on | Decides on (local signal) |
|---|---|---|
| **MCS** (+ constant tx_power) | **GS** (RX) | on-air PER from wfb RX stats; RSSI prior |
| **Bitrate → k → n → depth** (rate engine) | **Drone** (TX) | `fill_pct` (+ sender latency) from local venc sidecar |

The rate engine is a coupled unit (`k` is sized from the live bitrate; `n = k × ratio`), and congestion is TX-local — so it lives wholly on the drone. With FEC ratio and depth constant (§4.4), the rate engine needs **no RX input at all**.

### 4.2 Cross-link surface (control)

| Channel | Direction | Carries |
|---|---|---|
| venc sidecar (loopback) | venc → drone-fpvd | `fill_pct`, sender latency, encoder state |
| dynlink wire | GS → drone | `mcs` (+ initial `bandwidth`; `tx_power` constant) |
| dynlink wire | drone → GS | **removed** (was HELLO) |
| `/air/*` HTTP (existing) | GS → drone | drone `adapter_id` (Phase 3, on demand) |

The decision stream shrinks from `{mcs, k, n, depth, bitrate, tx_power}` to **`{mcs}`**. The drone→GS direction (HELLO: `mtu`, `fps`, `generation_id`) is deleted — the GS no longer needs drone config for FEC (now drone-local) and no longer gates on sync. This also removes the `awaiting_drone_config` stuck-state.

### 4.3 GS — outcome-driven MCS selector (replaces `LeadingSelector`'s SNR path)

Per-MCS statistics and selection:

- **Success EWMA.** For the currently-received MCS each window, `success = 1 − on_air_PER` (§2.3), smoothed (`success_ewma_alpha ≈ 0.25`, ~0.4 s). Attribute to the *received* MCS (`rx_ant_stats.mcs`), not the commanded one, to absorb apply latency.
- **Monotonicity prior.** Success is monotonic in MCS: a working rung implies all lower rungs work; a failing rung implies all higher fail. So only the boundary (current ± 1) is ever probed; the rest is inferred. Stale stats (older than `success_freshness_ms ≈ 1000`) fall back to the prior.
- **Ranking.** `argmax over mcs of HT_rate[mcs] × viable(mcs)` (expected goodput). A 70%-delivering high rung loses to a 99%-delivering mid rung automatically.
- **Single-step climb probe.** After `probe_clean_dwell_windows (≈4)` clean windows, command `current+1` for a short dwell and watch on-air PER; clean → commit, loss → revert and hold off that rung for `probe_failed_holdoff_ms`. Rate-limited by `probe_min_interval_ms (≈2000)`. **This replaces the `snr_floor`+hysteresis climb entirely — without it there is no way up.**
- **RSSI prior.** `rssi_floor_dBm[mcs]` (calibrated ladder) seeds cold-start and bounds the climb when no fresh stats exist. Replaces `snr_floor_dB`.
- **SNR.** Observed and logged only; not used for control.
- **Channel-B emergency** (`emergency_loss_rate`, `emergency_fec_pressure`, starvation): **kept** as the fast one-step drop.

### 4.4 Drone — rate engine (new, fully local)

- **Sidecar consumer.** drone-fpvd subscribes to the local venc sidecar over loopback (`127.0.0.1:5602`), parsing per-frame `fill_pct` (+ sender latency as corroboration). Holds the single subscription.
- **Capacity table (learned + persisted).** `capacity_kbps[mcs]`, seeded from the built-in HT-rate prior (`HT_rate[mcs] × conservative_factor`) and refined from the observed `fill_pct` knee, **persisted to drone state** across boots. First use of an MCS uses the prior; thereafter the learned value.
- **Bitrate ladder.** Discrete rungs `bitrate_ladder_kbps` (≈√2 spacing). Select the highest rung ≤ `capacity_kbps[current_mcs] × (1/ (1+ratio))`. Step **down** one rung when `fill_pct ≥ congestion_fill_pct (≈60)` (or sender latency over threshold) sustained for `rung_down_dwell_ms`; step **up** after a clean dwell `rung_up_dwell_ms`. Hysteretic, discrete — no continuous trim. Bitrate jumps as a feed-forward on MCS change.
- **FEC — constant.** `n = ceil(k × fec_redundancy_ratio)` with `fec_redundancy_ratio` constant (≈1.5, operator-tunable). `k` is sized per rung for the block-fill latency bound (`compute_k`), so it changes only on a rung change. The `NEscalator` and loss-driven escalation are removed.
- **Depth — constant.** `fec_depth` constant (≈1–2, operator-tunable). The `TrailingLoop` bootstrap/step-down logic is removed.

### 4.5 tx_power — constant

Constant operator-set value (default toward max for range). The inverse-MCS coupling (`_compute_tx_power`) and its range/cooldown knobs are removed (the knob is nonlinear and its "dBm" is fictional — §2; coupling deferred as an experiment, §3).

### 4.6 Hardware-derived profile (Phase 3)

The drone reports its `radio-up.sh`-detected `adapter_id` via the existing `/air/*` HTTP status; the GS selects the matching profile (the `rssi_floor_dBm` prior + caps) instead of a static `radioProfile`, default `"auto"`.

## 5. Phased roadmap

Each phase is independently shippable and gets its own implementation plan.

- **Phase 0 — Drone sidecar consumer (observe-only).** drone-fpvd subscribes to the local venc sidecar, surfaces `fill_pct`/latency in status/logs. No control change. Validates the signal (and the §11 backpressure-fidelity question) before anything depends on it. Reuses `/tmp/sidecar_probe.py` semantics.
- **Phase 1 — Drone rate engine.** Bitrate ladder + capacity learn/persist + constant FEC/depth. Move `k`/`n`/`depth`/`bitrate` computation off the GS; shrink the decision wire to `{mcs}`. Retire the airtime model (`PredictorConfig`, `preamble_us_per_frame`, `utilization_factor`).
- **Phase 2 — GS outcome MCS.** Replace the SNR-floor selector with the success-EWMA + single-step-probe selector and RSSI prior; remove `snr_floor_dB`, the SNR-margin gate, `snr_slope`; remove HELLO / the sync-gate. Keep Channel-B.
- **Phase 3 — Hardware profile auto-select.** `adapter_id` via `/air/*`; GS profile selection.

## 6. Schema delta

**Removed:** `RadioProfile.snr_floor_dB`, `.preamble_us_per_frame`; `GateConfig.{snr_safety_margin, snr_predict_horizon_ticks, hysteresis_up_db, hysteresis_down_db, loss_margin_weight, fec_margin_weight, snr_ema_alpha, snr_slope_alpha}`; `signals.snr_slope` + `ewma_alpha_snr_slope`; `BitrateConfig.utilization_factor`; `PredictorConfig` (`per_packet_airtime_us`, `block_duration_ms`, `inter_packet_interval_ms`) and `fit_or_degrade`; `LeadingLoopConfig.{tx_power_min_dBm, tx_power_max_dBm}` + the deprecated `tx_power_*`/`snr_*`/`rssi_*` back-compat block; `DynamicFecConfig.{n_loss_threshold, n_loss_windows, n_loss_step, n_recover_windows, n_recover_step, max_n_escalation, max_redundancy_ratio}` + `NEscalator`; `PolicyConfig.{sustained_loss_windows, clean_windows_for_depth_stepdown}` + the `TrailingLoop` depth logic; the HELLO machinery (`tunnel_listener.py`, drone `hello.cpp`, `drone_config.py` sync-gate, `awaiting_drone_config`).

**Added:** drone sidecar consumer config (`subscribeKeepaliveMs`); MCS selector (`success_ewma_alpha`, `success_climb_threshold ≈0.98`, `success_freshness_ms`, `probe_clean_dwell_windows`, `probe_min_interval_ms`, `probe_failed_holdoff_ms`, `rssi_floor_dBm[mcs]`); rate engine (`bitrate_ladder_kbps`, learned/persisted `capacity_kbps[mcs]`, `congestion_fill_pct`, `congestion_latency_ms`, `rung_up_dwell_ms`, `rung_down_dwell_ms`, `fec_redundancy_ratio`, `fec_depth`); constant `tx_power`; `radioProfile: "auto"`.

**Moved (GS → drone):** `DynamicFecConfig.{k_min, k_max, base_redundancy_ratio→fec_redundancy_ratio, blocks_per_frame}` and `compute_k`.

**Now a built-in constant (not config):** `data_rate_Mbps_LGI` → the 802.11n HT rate table in code.

**Kept:** `emergency_loss_rate`, `emergency_fec_pressure`, `max_mcs`, `bandwidth`, `min_between_changes_ms`, `hold_modes_down_ms`, `starvation_windows`/`starvation_threshold_pps`, `min/max_bitrate_kbps`, fec/burst EWMA alphas.

**Net:** `RadioProfile` collapses from a hand-authored physics model to **caps + one calibrated prior (`rssi_floor_dBm`)**; capacity is learned on the drone; identity is auto-derived (Phase 3).

## 7. Testing

- **Unit:** bitrate ladder (rung selection, hysteresis, capacity ceiling); capacity learner (prior seeding, knee update, persistence round-trip); success-EWMA + monotonicity + probe state machine; sidecar frame/trailer parser; constant-FEC `n`/`depth`.
- **Replay:** drive the GS MCS selector and the drone rate engine against captured telemetry traces (the session's congested/clean sidecar + wfb-stats captures) and assert the decisions.
- **Bench:** reuse this session's probes — TX-power/MCS sweeps, the CBR-congestion `fill_pct` test, the sidecar latency probe — to validate calibration and that the link stays under capacity (no congestion drops observed at the GS).

## 8. Success criteria

1. MCS is selected from measured on-air PER + probe, with `snr_floor_dB` gone; SNR no longer drives any decision.
2. Bitrate backs off on `fill_pct`/latency **before** any `transport_drops` — congestion never reaches the drop regime in steady state, so all GS-observed loss is channel loss.
3. Per-MCS capacity is learned and survives a drone reboot.
4. The decision wire is MCS-only; HELLO and the `awaiting_drone_config` stuck-state are gone.
5. Each control loop is closed on a locally-available signal; the drone rate engine takes no RX input.
6. FEC ratio and depth are constant; no FEC reconfig churn under steady loss.

## 9. Trade-offs

- **Constant FEC overhead.** A fixed ratio pays parity even on a clean link and can't shave it when conditions are great. Accepted: MCS keeps the channel low-PER, hysteresis absorbs blips, and avoiding the MCS↔FEC double-reaction is worth more than the saved parity.
- **Outcome-probe lag vs SNR's instant read.** Climbing requires a probe + dwell, slower than reading an SNR margin — but the SNR margin was unreliable, and the emergency channel still drops instantly. Reach-probe deferred to close the climb-lag gap if measured.
- **Logic on the weaker drone CPU.** The rate engine moves to the SSC33x. It is small (lookup + thresholds + a slow learner), and co-location buys sub-ms congestion feedback.
- **Single-subscriber sidecar.** The drone owns the subscription; GS-side link-RTT becomes a bench tool, not production telemetry.

## 10. Open questions

1. **`fill_pct` fidelity. — RESOLVED (Phase 0, 2026-06-05).** Confirmed on hardware via the drone-local sidecar consumer: varying *only* the radio MCS (encoder untouched) drove `fillPct` **65 → 0 → 65** as MCS went **0 → 5 → 0** (GS confirmed it received MCS 0 then 5). So `fill_pct` faithfully tracks the *radio* queue — backpressure propagates radio → wfb_tx → venc socket — not a socket-scheduling artifact. **Phase 1's bitrate trigger can use `fill_pct`.** Caveat: `fill_pct` saturates coarsely in the over-capacity region (~62–67% across a wide offered-rate range), so it is a crisp *binary* under/over-capacity signal but poor for *proportional* control; the sender-pipeline latency (`encodeMs`/`packetizeMs`) and `fps` are higher-resolution (clean ≈ 4 ms / 60 fps vs congested ≈ 100 ms / 18 fps) and should drive the proportional part of the ladder, with `fill_pct` as the knee detector.
2. **Frame-rate discrepancy. — RESOLVED (Phase 0).** The earlier "43 Hz" was **congestion throttling, not sidecar packet loss**: the drone-local consumer reads **58–60 fps when uncongested** and drops to 15–19 fps under congestion (encoder configured at 60 fps). fps is therefore itself a congestion signal, and capacity math should use the *uncongested* 60 fps.
3. **`rssi_floor_dBm` calibration.** Procedure to derive the per-MCS RSSI prior (conducted sweep vs passive flight-log), and whether it is per-card (Phase 3 selection) or one shared ladder.
4. **Probe glitch budget.** At speed, single-step climb may lag fast recovery; quantify before deciding on the reach-probe.
5. **Capacity-learning convergence. — VALIDATED on hardware (Phase 1 Stage A, 2026-06-05).** The passive learner (drone-local `CapacityTable`: prior-seed = HT-rate × 0.5, then EWMA toward `achievedKbps` when `fillPct ≥ 60`, nudge-up when clear-at-ceiling) was deployed observe-only and exercised on the live drone (192.168.10.152), MCS0/20 MHz/ch161. Driving the encoder over capacity (CBR 8000 → `fillPct` 56–67) pulled `capacity[0]` to the measured radio ceiling **≈ 3200–3260 kbps** and held it stable; persistence round-trips (`/etc/fpvd/capacity.json`, atomic write + `loadFrom` on bootstrap) confirmed via a sentinel reload. Note the *measured* MCS0 ceiling (~3.2 Mbps under current RF) is below Phase 0's ~4 Mbps reference — RF/environment dependent, which is exactly why it is learned not predicted. **Open refinement:** `achievedKbps` (encoder output) can transiently *overshoot* the radio's delivered rate during congestion (observed a 4720 kbps spike against the ~3200 steady ceiling); the α=0.1 EWMA damps these spikes so `capacity[mcs]` stays at the true ceiling, but a Stage B refinement should gate the congested-capacity sample on the *throttled steady state* (low `fps` / high `encodeMs`, per §10.1) rather than any `fillPct ≥ 60` tick, to avoid even transient over-estimation. Separately, at the static CBR-5000 operating point the link sits chronically near-congested (`fillPct` ~50–65) — precisely the condition Stage B's bitrate ladder is meant to relieve.
