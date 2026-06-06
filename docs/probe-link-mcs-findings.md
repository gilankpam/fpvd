# Probe-Link MCS Sounding — MVP Findings

**Date:** 2026-06-06
**Status:** Validated on hardware (single bench/range session)
**Hardware:** OpenIPC drone (Sigmastar SSC338Q, armv7l) + GS (aarch64), RTL8812EU
(`rtl88x2eu`, `0bda:a81a`), channel 161 (5805 MHz) / 20 MHz, wfb-ng. Drone TX power at
minimum, dual-NIC diversity on the GS.
**Code/tooling:** `tools/probe-mvp/` (throwaway scripts). Raw data + terse log in
`tools/probe-mvp/data/`.

---

## TL;DR

We tested whether a small **dedicated wfb "probe" link** — a separate `radio_port`
transmitting throwaway packets at a *higher* MCS than the live video — can tell the
adaptive-link controller **which MCS the video could safely use**, *without* ever risking
the video by probing on it directly.

It works, decisively:

- At a marginal range (RSSI ≈ −80 dBm, SNR ≈ 15 dB) the probe reported **MCS4 = clean
  (1%), MCS5 = dead (100% loss)** — while the **video sat at MCS2 and never lost a
  single packet**. An independent ground-truth sweep of the *real* video MCS at the same
  spot agreed: **MCS4 = 0.8%, MCS5 = 92%**.
- The MCS→packet-error relationship is **monotonic with a sharp cliff**, and the driver's
  **SNR metric collapses to garbage exactly at that cliff** — so packet-error-rate (PER),
  not SNR, is the signal to steer on.
- The conservative static MCS2 was leaving **two viable rungs (MCS3, MCS4) unused** —
  the headroom a probe-driven selector reclaims.

This validates the core of an outcome-driven adaptive-link redesign: pick MCS from
**measured PER on a non-disruptive probe**, and derive the encoder bitrate
deterministically from the chosen MCS.

---

## 1. Motivation

The adaptive-link controller's job is to pick the radio **MCS** (modulation/coding rate)
that maximizes video throughput without losing the link. Two long-standing problems:

1. **SNR is an untrustworthy signal.** On this hardware the driver's `snr_avg` is
   EVM/AGC-limited and survivor-biased (it only averages packets that decoded), so it
   saturates at close range and reads optimistically at the margin — precisely where the
   decision matters.
2. **You can't safely explore by probing the live video.** To learn whether a *higher*
   MCS would work, the classic approach (à la Minstrel rate control) periodically sends
   real traffic at candidate rates and watches it fail. On a video link every failed probe
   is a visible glitch — so controllers stay conservative and leave throughput on the
   table.

The idea tested here removes problem (2) — and, as a bonus, gives a clean read on the
real outcome signal that sidesteps problem (1).

## 2. The idea — a side-channel that sounds the link

Run a **second, throwaway wfb stream** on its own `radio_port`, on the **same card and
channel** as the video, transmitting at a **candidate MCS** (one rung or more above what
the video currently uses). Measure its packet-error-rate at the ground station. Because
the probe carries no real payload, a probe failure costs nothing — so you can safely
measure rungs the video would never dare touch.

This decouples **exploration** (the probe) from **exploitation** (the video):

```
            drone (one RTL8812EU, ch161)                 ground station
        ┌─────────────────────────────────┐         ┌────────────────────────┐
 video ─┤ wfb_tx  port 0  MCS2  FEC k3n5   ├──air────┤ wfb_rx port 0 → video   │  (never touched)
        │                                  │         │                        │
 probe ─┤ wfb_tx port 51  MCS4  FEC off    ├──air────┤ wfb_rx port 51 → PER(4) │  exploration,
 probe ─┤ wfb_tx port 52  MCS5  FEC off    ├──air────┤ wfb_rx port 52 → PER(5) │  zero video risk
        └─────────────────────────────────┘         └────────────────────────┘
```

It is, in effect, **channel sounding / CQI for wfb** — the same separation cellular uses
(sounding reference signals + a channel-quality report so the base station picks MCS
*without* gambling the data stream).

Key design choices, each borne out by the data:

- **Same card / same channel / separate `radio_port`.** The probe must measure the *same*
  channel the video sees; a second card or channel would not.
- **FEC off (`k=1, n=1`).** The probe wants the *raw* per-packet air loss, un-masked by
  error correction — so each lost packet shows as a gap in its sequence numbers.
- **MTU-sized packets (1400 B).** PER scales with packet length; token-sized probes would
  read cleaner than the video and lie about headroom.
- **Mirror the video PHY** (20 MHz, STBC=1, LDPC=1, long GI) so only the MCS differs.
- **Probe only upward.** Monotonicity (below) means everything beneath the current rung
  is already known to work; you only ever test rungs above.

## 3. Setup & tooling

All throwaway, in `tools/probe-mvp/`:

| File | Runs on | Role |
|---|---|---|
| `feeder.c` → `feeder.armv7l` | drone | cross-built static binary; emits seq-numbered (`PRB0`+`u64`) 1400 B UDP packets at a fixed rate to a local `wfb_tx -u` port |
| `probe_drone.sh` | drone | starts one `wfb_tx` (FEC-off, fixed MCS, own `radio_port`) + one feeder **per MCS** in a list |
| `probe_gs.sh` | GS | starts one probe `wfb_rx` per MCS + the logger |
| `probe_log.py` | GS | recovers per-MCS PER from seq gaps (ground truth), taps the wfb-ng `:8103` stats for the **live video** RSSI/SNR/loss, writes a joined CSV + per-packet JSONL |
| `air_mcs_sweep.py` | GS | sweeps the *real video* MCS via the drone config API and logs its on-air PER per rung (ground-truth gradient); always reverts to a safe MCS |

Two independent PER reads were cross-checked and agreed: the **seq-gap** count (our own,
FEC-independent) and **wfb_rx's own `lost`/`data` counters**. wfb_rx's per-antenna stats
are keyed by `freq:mcs:bw`, so each probe stream also yields **per-MCS RSSI/SNR for free**.

## 4. Experiments & results

### 4.1 Feasibility & non-disruption (close range, RSSI ≈ −56)

Probe swept MCS 2/4/6/7 (one at a time) while video ran at MCS2:

| probe MCS | cum PER | per-MCS RSSI/SNR |
|---|---|---|
| 2 | 0.0% | −58 / 25 |
| 4 | 1.6% | −56 / 26 |
| 6 | 0.7% | −56 / 26 |
| 7 | 1.2% | −56 / 25 |

All rungs clean (huge headroom at close range). **Video held `lost=0` throughout** — the
probe (75 pps total across streams) cost no measurable video quality. FEC-off (`k=n=1`)
was accepted by `wfb_tx`/`wfb_rx`, and per-MCS RSSI/SNR landed on the expected
`RX_ANT 5805:<mcs>:20` keys.

### 4.2 Ground-truth MCS gradient (max range, RSSI ≈ −81, SNR ≈ 14)

We flew the drone out and swept the **real video** MCS 2→7 (via the config API, hot-apply,
no restart), measuring the live stream's on-air PER at each rung:

| set MCS | obs MCS | RSSI | SNR | data | fec_rec | lost | on-air PER |
|---:|---:|---:|---:|---:|---:|---:|---:|
| 2 | 2 | −82 | 14 | 9892 | 46 | 0 | **0.47%** |
| 3 | 3 | −81 | 14 | 9841 | 23 | 0 | **0.23%** |
| 4 | 4 | −81 | 14 | 6899 | 57 | 0 | **0.83%** |
| 5 | 5 | −83 | **8** | 252 | 3 | 3044 | **92.45%** |
| 6 | — | −122 | −111 | 11 | 0 | 3057 | **99.64%** |
| 7 | — | −127 | −127 | 0 | 0 | 0 | dead |

- **Monotonic, sharp cliff** between MCS4 (clean) and MCS5 (92%). The viable ceiling here
  is MCS4; everything above fails. This is the **monotonicity prior**: one working rung
  implies all lower work; one failing rung implies all higher fail.
- **SNR self-destructs at the boundary** (14 → 8 → −111/−127). Once the link fails, the
  metric averages only the handful of garbage survivors. It dies exactly where a decision
  is needed — concrete proof that **PER, not SNR, is the signal**.

### 4.3 Probe-as-proxy — the crux (marginal range, RSSI ≈ −80, SNR ≈ 15)

With the drone still out, we deployed the probe **over the wfb tunnel** and ran probe
{2,4,5} @ 20 pps **concurrently** while the video stayed pinned at MCS2:

| probe MCS | pkts recv | probe PER (raw) | video sweep PER (§4.2) | verdict |
|---:|---:|---:|---:|:--|
| 2 | 1569 | **0.0%** | 0.47% | clean — matches live video |
| 4 | 1538 | **1.0%** | 0.83% | clean — **2 rungs of headroom** |
| 5 | 0 | **100%** | 92.45% | **dead — "do not climb"** |

**The probe reported "MCS4 viable, MCS5 unusable" on a throwaway side-channel while the
video never left MCS2 and never lost a packet.** A selector reading these probes would
climb MCS2 → MCS4 and refuse MCS5 — with zero risk to the feed.

The probe reads *more conservative* than the video at the cliff (100% vs 92%) because it
is FEC-off: the video's `k3n5` FEC recovered a few of the survivors. For a safety signal,
erring toward "dead" is the right bias.

## 5. What this validates

1. **A dedicated probe link is feasible and non-disruptive** — a different MCS on a
   separate `radio_port`, same card/channel, costs no measurable video loss.
2. **MCS→PER is monotonic with a sharp viability cliff**, so a controller need only probe
   the boundary (current ± 1) and infer the rest — no expensive all-rate sampling.
3. **PER is the trustworthy signal; SNR is not** — SNR collapses to garbage at the exact
   cliff where the decision lives.
4. **The probe faithfully predicts the video's per-MCS fate** without touching the video.
5. **Conservative static MCS wastes real throughput** — two viable rungs were unused here.

## 6. Limitations & open questions

- **Single session, single range pair, one card.** Conditions were created by physical
  distance with TX power already at minimum; no multipath/flight dynamics were exercised.
- **Per-MCS RSSI/SNR from the probe is noisy at low pps** (20 pps → single-sample SNR of 8
  vs the video's windowed 15). A stable `rssi_floor[mcs]` prior would need higher probe pps
  or longer dwell.
- **Airtime budget not characterized vs pps.** 60–75 pps total was harmless here; the safe
  ceiling vs video bitrate at marginal range is not yet mapped.
- **Probe vs video at the cliff differ in absolute value** (100% vs 92%) due to FEC-off —
  qualitatively identical ("dead"), but not a calibrated 1:1 PER match at extreme loss.
- **Lost-probe attribution** used one `radio_port` per fixed MCS (unambiguous even for
  dropped packets). A single-port time-multiplexed sweep would need a deterministic
  `seq→MCS` schedule instead.

## 7. Implications for the adaptive-link design

This is the de-risking evidence behind the controller redesign in
`docs/superpowers/specs/2026-06-05-outcome-driven-link-control-design.md`. It supports an
outcome-driven controller where:

- **MCS** is selected from **measured probe PER** (promote on a clean probe at the next
  rung; demote on the live video's own PER / an emergency channel). The probe turns MCS
  selection from a risky bandit problem into a **measured lookup**.
- **Bitrate** follows the chosen MCS **deterministically** (e.g. the OpenIPC bitrate
  calculator's effective-rate table × utilization × FEC ratio) — no separate capacity
  learner needed.
- **SNR** is logged only; **RSSI** serves as a cold-start prior and a calibration target
  (§6) — not the control signal.

## 8. Reproduce it

Drone is cross-built once (`nix-shell` provides `armv7l-unknown-linux-musleabihf-gcc`):

```sh
# build the feeder for the drone (from drone/ for shell.nix)
nix-shell --run "armv7l-unknown-linux-musleabihf-gcc -static -Os \
  -o ../tools/probe-mvp/feeder.armv7l ../tools/probe-mvp/feeder.c"
```

Deploy + run (read the live `wfb_tx` args off `ps` to match key/linkId/PHY):

```sh
# push tooling: feeder + probe_drone.sh -> drone ; probe_log.py + probe_gs.sh -> GS
# (the drone is reachable on the LAN when close, or via the GS as a ProxyJump over the
#  wfb tunnel when it is out of LAN range)

ssh GS  '/tmp/probe_gs.sh    start 2,4,5 20'   # one wfb_rx + logger per MCS
ssh DRN '/tmp/probe_drone.sh start 2,4,5 20'   # one FEC-off wfb_tx + feeder per MCS
# watch /tmp/probe_log.console ; artifacts: /tmp/probe_run.joined.csv, *.packets.jsonl
ssh DRN '/tmp/probe_drone.sh stop' ; ssh GS '/tmp/probe_gs.sh stop'

# ground-truth video-MCS gradient (always reverts to the safe MCS):
ssh GS 'python3 /tmp/air_mcs_sweep.py 2,3,4,5,6,7 8 3 2'
```

## Appendix — probe packet wire format

```
offset  size  field
  0      4    magic = "PRB0"
  4      8    big-endian uint64 sequence number
 12     ..    0xA5 fill to the configured size (default 1400 B)
```

PER per window = `1 − received/(max_seq − min_seq + 1)`; cumulative PER spans the run.
Per-packet `{t, seq}` is logged to JSONL so any window/EWMA can be reconstructed offline.
