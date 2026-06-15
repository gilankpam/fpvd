# Dynamic-link tuning reference

This document adds operator-facing semantics to the drone-side `dynamicLink` config knobs.

**Canonical inventory.** The single authoritative list of fields and their current code defaults is `fpvd --dump-config` (prints a full `Config{}` JSON) or `GET /config` on a running daemon. This document does not repeat that list — it explains what the knobs *do* and when to touch them.

**Config model.** There is one config file: `/etc/fpvd/config.json`. It is a sparse overlay; any key that is absent falls back to the code default in `Config{}`. Unknown keys produce a warning on stderr and are otherwise ignored, so old overlays from before a field was removed do not crash the daemon. `GET /defaults` returns the full code-default `Config{}`.

---

## Operational — commonly set

These are the knobs most operators need.

### `dynamicLink.enabled` (bool, default `false`)

Arms the GS→drone dynamic-link control loop. When `false`, the daemon uses the static `link.*` and `video.*` values from config and does not start the dynamic-link applier. When `true`, the applier process starts and takes ownership of `link.mcs`, `link.txPowerDbm`, `link.fec`, `link.width`, `video.bitrate`, `video.qpDelta`, and `video.roi` — PATCH attempts on those paths return `400 dynamic_link_locked`.

### `dynamicLink.safe.*` (object)

The floor the applier falls back to when it loses GS contact (no decision packet received within `healthTimeoutMs`) or is first starting up. `bandwidth` and `txPowerDbm` are **not** fields in `safe` — they are derived at apply time (`bandwidth` from `link.width`, TX power from the per-MCS curve in `txpower_curve.hpp`).

| Field | Default | Valid range | Purpose |
|-------|---------|-------------|---------|
| `safe.mcs` | `1` | 0 – 7 | MCS index for the safe rung |
| `safe.k` | `8` | 1 – 31, k < n | FEC data shards (rs-mode geometry; also used as the safe k for swfec block sizing) |
| `safe.n` | `12` | 2 – 32, n > k | FEC total shards |
| `safe.overheadPct` | `100` | 0 – 255 | swfec repair budget at the safe rung (more repair than normal) |
| `safe.deadlineMs` | `30` | 1 – 255 | swfec recovery window at the safe rung |
| `safe.bitrateKbps` | `2000` | > 0 | Video encoder target at the safe rung |

### `osd.enabled` (bool, default `true`)

Top-level key — a sibling of `dynamicLink`, not nested inside it. Controls whether the daemon writes link-quality data to the msposd overlay message file (`/tmp/MSPOSD.msg`). Runs regardless of whether `dynamicLink.enabled` is true.

---

## Advanced — rarely set

These knobs have sane defaults for the BL-M8812EU2 / ssc338q stack and should only be adjusted if you have a specific reason.

### Timing

| Field | Default | Valid range | Purpose |
|-------|---------|-------------|---------|
| `dynamicLink.healthTimeoutMs` | `10000` | >= 1000 ms | How long without a GS decision packet before the applier falls back to `safe` config |
| `dynamicLink.applyStaggerMs` | `50` | 0 – 500 ms | Delay between applying the radio change and the bitrate/FEC change in a single decision, giving the radio time to settle |
| `dynamicLink.applySubPaceMs` | `5` | 0 – 50 ms | Pacing between individual sub-commands within a single apply batch |

### ROI-QP curve (`dynamicLink.roiQp.*`)

Maps the current video bitrate onto a QP delta for the center-ROI region, trading perceptual quality for headroom at constrained bitrates.

| Field | Default | Valid range | Purpose |
|-------|---------|-------------|---------|
| `roiQp.thresholdKbps` | `6000` | > `lowAnchorKbps` | Bitrate at which the QP delta starts to tighten (no delta above this) |
| `roiQp.lowAnchorKbps` | `2000` | > 0 | Bitrate at which the delta reaches `floor` |
| `roiQp.floor` | `-24` | <= 0 | Most negative QP delta allowed (larger negative = more quality preference) |
| `roiQp.step` | `3` | >= 1 | QP units per curve segment |

### Bitrate and FEC geometry (`dynamicLink.compute.*`)

Controls how the applier derives video bitrate and FEC block size from the current MCS and probe measurement. The probe-based wire-target formula is in `drone/src/dynlink/bitrate.cpp`.

| Field | Default | Valid range | Purpose |
|-------|---------|-------------|---------|
| `compute.minBitrateKbps` | `1000` | > 0, < `maxBitrateKbps` | Hard floor for computed video bitrate |
| `compute.maxBitrateKbps` | `24000` | > `minBitrateKbps` | Hard ceiling for computed video bitrate |
| `compute.baseRedundancyRatio` | `0.5` | > 0 | n/k − 1; 0.5 gives the default 8/12 (data fraction ≈ 67%) |
| `compute.blocksPerFrame` | `2.0` | > 0 | FEC blocks per video frame; raise to reduce block-fill latency at the cost of FEC granularity |
| `compute.kMin` | `2` | >= 1, <= `kMax` | Minimum FEC k (data shards per block) |
| `compute.kMax` | `50` | >= `kMin` | Maximum FEC k |

---

## Frozen constants — no config path

The following values are compile-time constants that are not operator-configurable. They are listed here so that GS-side mirrors (notably `RssiNormConfig.tx_power_dbm_by_mcs` in `gs/fpvdgs/dynlink/signals.py`) can be verified against the drone source.

### `drone/src/dynlink/txpower_curve.hpp`

Per-MCS TX power for the BL-M8812EU2 (indexed MCS 0–7):

```
{ 29, 28, 25, 23, 19, 19, 19, 19 }  // dBm
```

Full power at low MCS for range; backed off on the high-PAPR 64-QAM rungs (MCS 4–7) to keep the PA linear. `safe.bandwidth` and `safe.txPowerDbm` are absent from the `safe` config block because TX power at the safe rung is read from this table at index `safe.mcs`.

### `drone/src/probe/probe_constants.hpp`

Observe-only probe link (FEC-off, tracks one rung above operating MCS):

- Radio port: 50
- Control port: 8001 (wfb_tx -C)
- Feed port: 6700 (feeder → wfb_tx)
- Packets/sec: 25
- Packet bytes: 1400 (mirrors video MTU)
- MCS ceiling: 7

### `drone/src/idr/idr_constants.hpp`

IDR keyframe-request relay from PixelPilot on the GS:

- Listen port: 11223 (UDP)
- Throttle window: 500 ms between encoder IDR requests

### `drone/src/osd/osd_constants.hpp`

OSD overlay message file written by the daemon: `/tmp/MSPOSD.msg`. msposd reads this path; it is not configurable.

### OpenIPC rate table — `drone/src/dynlink/bitrate.cpp`

Long-GI base rates used by the bitrate formula (kbps):

| MCS | 20 MHz | 40 MHz |
|-----|--------|--------|
| 0 | 6 500 | 9 800 |
| 1 | 12 000 | 18 600 |
| 2 | 15 500 | 30 400 |
| 3 | 20 000 | 40 200 |
| 4 | 25 000 | 55 800 |
| 5 | 42 000 | 80 400 |
| 6 | 47 500 | 90 200 |
| 7 | 55 000 | 97 000 |

Source: OpenIPC WFB calculator (long-GI rows).
