# Probe-Driven Link Control — MCS from a Side-Channel Probe, Bitrate from the OpenIPC Calculator

**Date:** 2026-06-06
**Status:** Draft for review — core mechanism hardware-validated (see §2 / `docs/probe-link-mcs-findings.md`)
**Target:** OpenIPC drones (Sigmastar SSC33x, armhf) + GS, fpvd codebase; RTL8812EU link
**Extends:** `2026-05-27-dynamic-link-design.md`, `2026-06-01/02-*-fold-in-design.md`
**Supersedes:** `2026-06-05-outcome-driven-link-control-design.md` — keeps its "outcomes over predictions" thesis but replaces **both** of its decision paths: the success-EWMA + single-step-probe MCS selector becomes a **dedicated side-channel probe link**, and the `fill_pct`/learned-capacity bitrate engine becomes the **deterministic OpenIPC WFB calculator** (the venc sidecar is dropped entirely).

---

## 1. Purpose

Decide the two link knobs from signals we can trust, measured where they live:

- **MCS** is selected from a **dedicated probe link** that continuously measures the raw on-air packet-error-rate of candidate MCS rungs *on a throwaway side-channel* — so the controller learns which rung the channel can carry **without ever risking the video stream**. SNR drops out of control; RSSI is a cold-start prior.
- **Bitrate** is **deterministic**: the drone computes it from the **official OpenIPC WFB calculator** given the current MCS. No measurement, no learning, no sidecar.

Everything else (FEC ratio/depth, tx_power) is constant. The decision crossing the radio shrinks to **`{mcs}`**.

This replaces a hand-authored `snr_floor_dB` ladder, a fixed-constant airtime model, and a loss-driven dynamic-FEC escalator — all shown unsound on the deployed hardware (§2 and the superseded spec's §2).

## 2. Background — measured evidence

All figures captured live on the deployed drone (`192.168.10.152` / tunnel `10.5.0.10`) + GS (`10.18.0.1`), RTL8812EU, ch161/20 MHz, drone txpower min. Full write-up: `docs/probe-link-mcs-findings.md` (raw captures + the throwaway rig are local-only under `tools/probe-mvp/`, git-ignored).

### 2.1 The probe link is feasible and non-disruptive (validated)

A dedicated probe = an extra `wfb_tx` per candidate MCS on its **own `radio_port`**, same card/channel as video, **FEC off** (`k=1 n=1`, raw un-masked loss), MTU-sized (1400 B) throwaway packets at low rate, **mirroring the video PHY** (`-B 20 -S 1 -L 1`, long GI), differing only in `-M`.

- Injects at a **different MCS than the live video** and is received & PER-measured per-MCS. The GS reads per-MCS PER from the probe `wfb_rx` `PKT` counters (`lost/data`) **and** from a seq-gap ground truth — they agree.
- **Per-MCS RSSI/SNR fall out for free** — `wfb_rx` `RX_ANT` rows are keyed by `freq:mcs:bw`, so each probe rung self-reports RSSI/SNR (this is the `rssi_floor_dBm[mcs]` calibration source).
- **Zero video disruption** at 60–75 pps total even at marginal range — video held `lost=0`.

### 2.2 The MCS→PER gradient is monotonic with a sharp cliff (validated)

Sweeping the *video* MCS 2→7 via the GS `/air` proxy at one marginal position (RSSI ≈ −81, SNR ≈ 14) gave a textbook curve: on-air PER **0.47 / 0.23 / 0.83 %** at MCS 2/3/4, then a **cliff to 92 % at MCS5**, 99.6 % at MCS6, dead at MCS7. The viable ceiling was MCS4; the static config sat at MCS2, **wasting two viable rungs**.

Two consequences for the design:
- **Monotonicity holds** — a working rung implies all below work; a failing rung implies all above fail. So the probe never needs to sample the whole table; the boundary is enough.
- **SNR self-destructs at the boundary** — it read 14 when fine, then garbage (8, −111, −127) the instant the link failed, because it averages only the few survivors. PER is the trustworthy signal; SNR is observed/logged only.

### 2.3 The probe faithfully predicts video viability (validated — the crux)

At a marginal position (RSSI ≈ −80, SNR ≈ 15), with the **video pinned at MCS2 (`lost=0` throughout)**, a probe of `{2,4,5}` read:

| probe MCS | probe PER (raw) | video /air-sweep PER (same range) | verdict |
|---|---|---|---|
| 2 | 0.0 % | 0.47 % | clean — matches live video |
| 4 | 1.0 % | 0.83 % | clean — **2 rungs of headroom** |
| 5 | 100 % (0 pkts) | 92 % | **dead — "do not climb"** |

The probe reported "MCS4 viable / MCS5 unusable" on the side-channel **while the video never left MCS2 and never dropped a packet**. The probe is *more conservative* at the cliff (100 % raw vs 92 % with video FEC recovering a few) — desirable for a safety signal.

### 2.4 The airtime/bitrate model is wrong; the OpenIPC table is the fix

(From the superseded spec §2.2, still load-bearing.) `per_packet_airtime_us=80` is right only at MCS7/HT40; across the real MCS0–5/20 MHz range, per-1400 B airtime is 3.6–22× larger, and `preamble_us_per_frame` is a ~2× fudge. Rather than calibrate an airtime model, we adopt the **OpenIPC WFB calculator's effective-rate table** — which is already de-rated to real WFB throughput — and a single utilization factor. No airtime model remains.

### 2.5 Operational facts the design relies on

- `wfb_rx` stats (`RX_ANT` per `freq:mcs:bw`; `PKT` with `data/fec_rec/lost/out`) are exposed as the `:8103` `StatisticsJSONProtocol` and as per-instance stdout. The interval is the `-l` flag (default 1000 ms; live deploy 100 ms; the probe ran at `-l 50`).
- `wfb_tx` MCS/FEC/port are per-instance flags (`-M/-k/-n/-p`); the radio MTU is 1445; `radio-up.sh` already maps the USB id to an `adapter_id`.

## 3. Scope

**In scope:** the drone probe injector; the GS per-MCS PER/RSSI measurement + probe-driven MCS selector; the drone OpenIPC-calculator bitrate engine with latency-sized `k` + fixed FEC ratio/depth; removal of the SNR-floor/airtime/dynamic-FEC/HELLO machinery; constant tx_power; hardware-derived profile.

**Out of scope (YAGNI / deferred):**
- Continuous all-rate (Minstrel-style) probing — monotonicity (§2.2) makes a bounded upward probe sufficient.
- Per-MCS FEC ratio; dynamic depth; tx_power↔MCS coupling.
- Short-GI (link runs long-GI, `shortGi=false` hardcoded).
- Changes to the wfb wire format / FEC codec / radiotap.
- A return (drone→GS) control channel beyond what `/air` already provides.

## 4. Architecture

### 4.1 Division by signal locality

| Loop | Lives on | Decides on |
|---|---|---|
| **MCS** (+ constant tx_power) | **GS** (RX) | probe-PER per rung (promote) + live video-PER (demote); RSSI prior |
| **Bitrate → k → n → depth** | **Drone** (TX) | the commanded MCS, via the deterministic calculator (no live input) |

The probe **TX** is on the drone (it injects on the drone radio); the probe **measurement** and the MCS decision are on the GS (where the RX stats live). The bitrate engine is fully drone-local and takes **no** RX input.

### 4.2 Cross-link surface (control)

| Channel | Direction | Carries |
|---|---|---|
| dynlink wire | GS → drone | `{mcs}` (+ initial `bandwidth`; `tx_power` constant) |
| probe link | drone → GS | per-MCS probe packets (measured at GS; not "control") |
| `/air/*` HTTP (existing) | GS ↔ drone | config/status, incl. drone `adapter_id` (Phase 4) |

The decision stream is **`{mcs}`** only. HELLO (`mtu/fps/generation_id`) and the `awaiting_drone_config` sync-gate are **removed** — the GS no longer needs drone config (FEC is drone-local; bitrate is deterministic).

### 4.3 The probe link

**Drone injector.** For each candidate MCS, a `wfb_tx` on its **own `radio_port`**, FEC off (`k=1 n=1`), MTU-sized throwaway packets at a low fixed rate, PHY mirroring video (`-B`, `-S`, `-L`, long GI). A tiny local feeder generates the packets.

- **Candidate set = a bounded window above current.** Probe `current+1` always; extend to `current+2` while `+1` reads clean (monotonicity makes probing higher pointless once a rung fails). Stop at `max_mcs`. The drone knows `current` from the last `{mcs}` command, so it manages the set **autonomously** — no extra control input. (The MVP exercised a *fixed concurrent set* of rungs; the adaptive upward reach here is the airtime-efficient production form and is an Open Question to confirm under fast range change — §10.2.)
- **Lost-probe attribution.** Each candidate MCS rides its own `radio_port` so a *dropped* probe is still attributable (you can't read MCS off a packet that never arrived). The GS confirms the rung from the `RX_ANT` `mcs` key, which also handles the port→MCS remap as `current` moves.
- **Airtime.** Low rate (target < ~1–few %); upward probes are cheap (higher MCS = less airtime/packet). Validated: 60–75 pps total caused zero video loss.

**GS measurement.** A probe `wfb_rx` per `radio_port` (low `-l`) yields per-MCS PER (`lost/(data+lost)`, FEC-off so raw) + per-MCS RSSI/SNR (`RX_ANT`), smoothed with an EWMA over a freshness window.

### 4.4 GS — probe-driven MCS selector

Two signals, each driving one direction:

- **Promote (explore):** the highest MCS whose **probe success (`1 − probe-PER`) ≥ `probe_viable_threshold`** and is **fresh** (`probe_freshness_ms`). Because the probe already proved it on the side-channel, promotion is **immediate** — no commit-and-watch gamble. Rank by expected goodput `argmax HT_rate[mcs] × viable(mcs)`.
- **Demote (ground truth):** on the **live video on-air PER** breach (`(lost+fec_rec)/(out+lost)`) or **Channel-B emergency** (`emergency_loss_rate`, `emergency_fec_pressure`, starvation) — the fast one-step drop, **kept**. Only the real stream under load can declare the current rung failing; the probe can only promote.
- **RSSI prior:** `rssi_floor_dBm[mcs]` seeds cold-start (before probe stats exist) and bounds the climb. **SNR removed from control** (logged only).
- Rate-limited by `min_between_changes_ms`; `hold_modes_down_ms` after a demote.

This **replaces** `LeadingSelector`'s SNR path, the `snr_floor`+hysteresis climb, and the single-step climb probe entirely.

### 4.5 Drone — bitrate from the OpenIPC calculator

On each `{mcs}` (or bandwidth) change, the drone recomputes the encoder bitrate deterministically, **reserving the probe link's airtime**:

```
probe_kbps   = probe_pps × probe_packet_bytes × 8 / 1000             # constant (≈ 280 at 25 pps, 1400 B)
probe_util   = probe_kbps / baseRate[bw][gi][min(mcs+1, maxMcs)]     # clamps to maxMcs at the ceiling
bitrate_kbps = min( cap, round( baseRate[bw][gi][mcs] × (2/3 - probe_util) × fec_data / fec_total ) )
             clamped to [min_bitrate_kbps, max_bitrate_kbps]
```

- `baseRate` is the **OpenIPC effective-rate table** (built-in constant; already de-rated — *not* the textbook PHY table), MCS 0–7, 20/40 MHz, long/short GI. **Source:** OpenIPC WFB calculator, `OpenIPC/docs` → `src/components/wfb-calculator.astro` (https://github.com/OpenIPC/docs/blob/main/src/components/wfb-calculator.astro). Verbatim (kbps, index = MCS 0–7):
  - 20 MHz — long `[6500, 12000, 15500, 20000, 25000, 42000, 47500, 55000]`; short `[7200, 13400, 18700, 21900, 28300, 43800, 50000, 55200]`
  - 40 MHz — long `[9800, 18600, 30400, 40200, 55800, 80400, 90200, 97000]`; short `[12000, 24000, 36000, 48000, 60000, 91000, 98000, 100000]` (upstream has an obvious typo `980000` at 40/short/MCS6 — use `98000`)
- `2/3` is the utilization factor; `fec_data/fec_total` is the FEC data fraction (`k/n`). Default `8/12` ⇒ factor = `4/9 ≈ 0.444` (e.g. MCS0/20/long → `6500 × 4/9 = 2888 kbps`, comfortably under the ~3.2 Mbps measured ceiling). **Naming gotcha:** the upstream calculator computes `floor((baseRate × 2 × fec_n + floor(3·fec_k/2)) / (3 × fec_k))` where **its `fec_n` is the *data* count and `fec_k` is the *total* count** — opposite to wfb's `-k` (data) / `-n` (total). The result is identical to `baseRate × 2/3 × data/total`.
- **Probe airtime reserve** (added by the probe-plumbing design): the side-channel probe runs continuously while dynamic link is enabled, costing `probe_util` of channel airtime at its rung (`current+1`). Subtracting it from the `2/3` utilization keeps video + probe within the same committed-airtime budget — table-only, no airtime model. The probe is **FEC-off (`k/n = 1/1`)**, so `probe_kbps` is its true on-air rate (no FEC inflation). `probe_util` ≈ 0.7 % (probe on a high rung) up to ≈ 2.3 % at the floor; at the ceiling the single stream **clamps to `maxMcs`** (re-reads the top rung rather than idling), so `probe_util` stays small-but-nonzero (~0.5 %). Net video bitrate drop ≈ 0.5–3.5 %, largest where the channel is tightest. `probe_pps`/`probe_packet_bytes` are fixed constants (see the probe-plumbing design).
- GI = **long** (constant). Bandwidth from the commanded value.

The bitrate jumps as a feed-forward on MCS change. **Open-loop by design** — the only thing that lowers bitrate under bad RF is the GS dropping MCS (which it does on PER), recomputing the formula down. The conservative `0.44` factor is the margin; §2.3 shows it sits safely under the real ceiling.

### 4.6 Drone — FEC (latency-sized k, fixed ratio) and depth (constant)

- The calculator only needs the **ratio**, so `k` can be sized for the block-fill latency bound (`compute_k`, moved GS→drone) while the **redundancy ratio is constant**: `n = ceil(k × fec_redundancy_ratio)` (≈ 1.5, i.e. the same `8/12` data fraction the formula uses — single source of truth feeding both the radio and the bitrate). `k` recomputes only on a bitrate change.
- **Depth** is a constant (≈ 1–2, operator-tunable). The `NEscalator` (dynamic-FEC) and `TrailingLoop` (depth bootstrap/step-down) are removed.

### 4.7 tx_power — constant

Operator-set constant (default toward max for range). The inverse-MCS coupling (`_compute_tx_power`) and its range/cooldown knobs are removed.

### 4.8 Hardware-derived profile (Phase 4)

The drone reports its `radio-up.sh`-detected `adapter_id` via `/air/*`; the GS selects the matching profile (`rssi_floor_dBm` prior + caps) instead of a static `radioProfile`, default `"auto"`.

## 5. Phased roadmap

Each phase is independently shippable and gets its own implementation plan.

- **Phase 1 — Probe link, observe-only.** Productionize the MVP: drone probe injector (bounded upward set, FEC-off, PHY-mirrored) + GS per-MCS PER/RSSI measurement, surfaced in status/logs. No control change yet — validates the selector's inputs in-tree. (`tools/probe-mvp/` is the throwaway reference.)
- **Phase 2 — GS probe-driven MCS selector.** Promote-on-probe / demote-on-video-PER + RSSI prior, replacing the SNR-floor selector. Remove `snr_floor_dB`, the SNR-margin gate, `snr_slope`, HELLO + sync-gate. Keep Channel-B. Shrink the wire to `{mcs}`.
- **Phase 3 — Drone bitrate from the calculator.** OpenIPC table + formula on the drone; move `compute_k` drone-side with a fixed ratio; constant depth. Retire the airtime model (`PredictorConfig`, `preamble_us_per_frame`, `utilization_factor`, `effective_phy_Mbps`) and the dynamic-FEC escalator.
- **Phase 4 — Calibration & auto-profile.** Higher-pps probe pass to derive a stable `rssi_floor_dBm[mcs]` prior (the 20 pps RSSI was noisy); `adapter_id` via `/air/*`; GS profile auto-select.

## 6. Schema delta

**Removed:** `RadioProfile.snr_floor_dB`, `.preamble_us_per_frame`; `GateConfig.{snr_safety_margin, snr_predict_horizon_ticks, hysteresis_up_db, hysteresis_down_db, loss_margin_weight, fec_margin_weight, snr_ema_alpha, snr_slope_alpha}`; `signals.snr_slope` + `ewma_alpha_snr_slope`; `BitrateConfig.utilization_factor`; `PredictorConfig` + `fit_or_degrade`; `LeadingLoopConfig.{tx_power_min_dBm, tx_power_max_dBm}` + the tx_power/snr/rssi back-compat block; `DynamicFecConfig.{n_loss_*, n_recover_*, max_n_escalation, max_redundancy_ratio}` + `NEscalator`; `PolicyConfig.{sustained_loss_windows, clean_windows_for_depth_stepdown}` + `TrailingLoop`; the HELLO machinery (`tunnel_listener.py`, drone `hello.cpp`, `drone_config.py` sync-gate, `awaiting_drone_config`).

**Added:** probe link (drone) — `probe_radio_ports`, `probe_pps`, `probe_packet_bytes`, `probe_reach` (upward window), feeder; MCS selector (GS) — `probe_viable_threshold`, `probe_freshness_ms`, `probe_ewma_alpha`, `video_demote_per`, `rssi_floor_dBm[mcs]`; bitrate (drone) — the OpenIPC `baseRate` table as a built-in constant, `fec_redundancy_ratio`, `fec_depth`, `cap`/`min_bitrate_kbps`/`max_bitrate_kbps`; constant `tx_power`; `radioProfile: "auto"`.

**Moved (GS → drone):** `compute_k` (+ `k_min`, `k_max`, `blocks_per_frame`), `base_redundancy_ratio → fec_redundancy_ratio`.

**Now a built-in constant:** the OpenIPC effective-rate table (replaces `data_rate_Mbps_LGI` *and* the airtime model).

**Kept:** `emergency_loss_rate`, `emergency_fec_pressure`, `max_mcs`, `bandwidth`, `min_between_changes_ms`, `hold_modes_down_ms`, `starvation_windows`/`starvation_threshold_pps`, `min/max_bitrate_kbps`, fec/burst EWMA alphas.

## 7. Testing

- **Unit:** calculator formula (table lookup, FEC fraction, cap/clamp, rounding); `compute_k` + `n = ceil(k×ratio)`; probe per-MCS PER/RSSI parser (PKT/RX_ANT + seq-gap); promote/demote state machine + monotonicity + freshness fallback; RSSI-prior cold-start.
- **Replay:** drive the GS selector against captured probe + video traces (promote the local `tools/probe-mvp/data/*.csv|jsonl` captures to committed test fixtures) and assert decisions (e.g. promote to MCS4, refuse MCS5 at the marginal trace).
- **Bench:** reuse the MVP rig — probe feasibility/airtime, the `/air` video-MCS gradient sweep, and the probe-vs-video proxy comparison — to re-validate after productionizing.

## 8. Success criteria

1. MCS is selected from probe-PER (promote) + video-PER (demote); `snr_floor_dB` gone; SNR drives nothing.
2. The selector climbs into measured headroom (e.g. MCS2→MCS4 at a marginal range) and refuses a dead rung (MCS5) **without the video leaving its safe rung** — i.e. the probe's promotion never glitches the feed.
3. Bitrate is the deterministic OpenIPC-calculator output of the current MCS; the airtime model and dynamic-FEC escalator are gone; no FEC reconfig churn under steady loss.
4. The decision wire is `{mcs}`-only; HELLO and `awaiting_drone_config` are gone.
5. The drone bitrate/FEC engine takes no RX input; the GS needs no drone config.

## 9. Trade-offs

- **Probe airtime.** A side-channel costs a little airtime continuously. Accepted: bounded to a 1–2 rung upward window at low pps (< ~few %), upward probes are cheap, and it buys risk-free exploration (validated: zero video impact).
- **Open-loop bitrate.** A fixed formula can't shave parity on a great link or react within a fixed MCS. Accepted: the `0.44` factor sits under the measured ceiling, and MCS (which *does* react to PER) is the outer loop that pulls bitrate down via the formula. No congestion sidecar to maintain.
- **More moving parts on the drone.** A probe injector + feeder + the calculator move onto the SSC33x. All are small (lookups/thresholds + a trickle feeder); co-location keeps the probe on the exact channel the video uses.
- **Probe vs video packet representativeness.** Probe must stay MTU-sized and PHY-matched or it would mis-predict; a sparse probe can under-sample bursty interference (mitigated by EWMA + adequate pps).

## 10. Open questions

1. **`rssi_floor_dBm[mcs]` calibration (Phase 4).** Derive from a higher-pps probe pass (the 20 pps RSSI/SNR was single-sample-noisy); per-card vs one shared ladder.
2. **Probe reach & cadence.** Confirm `current+1`/`+2` adaptive reach vs a fixed concurrent set under fast range change; pick pps for PER resolution vs airtime.
3. **Promotion debounce.** How many fresh clean probe windows before committing a promote, to avoid chasing a transient clean blip at the cliff.
4. **Probe-stream liveness.** Fallback when the probe goes stale/dies (→ RSSI prior + hold); detection latency.
