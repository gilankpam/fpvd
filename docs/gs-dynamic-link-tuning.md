# GS Dynamic-Link Tuning Reference

This document covers the `fpvdgs` adaptive-link controller knobs exposed in
`/etc/fpvd/config.json` and the frozen constants baked into the code. The
generated `config.json` (from `fpvd --dump-config`) is the canonical inventory
of every exposed knob and its default; this doc adds semantics, valid ranges, and
source-file references for both buckets.

Two config blocks carry the tunable knobs: `dynamicLink.selector` (operational)
and `dynamicLink.smoothing` (advanced). One single-key block exposes a rollback
toggle: `dynamicLink.flightlog.enabled`. Everything else is frozen.

---

## Exposed knobs

### Top-level `dynamicLink` fields

| Key | Type | Default | Valid | Purpose |
|-----|------|---------|-------|---------|
| `enabled` | bool | `false` | — | Arm the in-process control loop. |
| `maxMcs` | int | `5` | 0 – 7 | Operator MCS ceiling — the controller never selects above this rung regardless of probe results. Distinct from the internal `SelectorConfig.max_mcs` HW-ceiling fallback (7). |
| `dronePort` | int | `9999` | 1 – 65535 | Drone dynamic-link UDP **port**; the host is the shared `drone.host` (see below). Matches `drone/src/dynlink/wire.hpp`. |

> **Drone address.** The GS reaches the drone via one top-level `drone.host` (default `10.5.0.10`), reused by all three components — the HTTP `/air` proxy (`drone.apiPort`, default `8080`), the IDR relay (`idrForward.port`), and the dynamic-link decision UDP (`dynamicLink.dronePort`). Only the host is shared; each service keeps its own port.

### `dynamicLink.selector` — operational tuning

The selector implements probe-driven promote + reactive demote with cooldowns.
Source: `gs/fpvdgs/dynlink/policy.py` (`SelectorConfig`).

| Key | Type | Default | Valid | Purpose |
|-----|------|---------|-------|---------|
| `probeViableThreshold` | float | `0.99` | [0, 1] | Minimum EWMA probe-success rate on the `current+1` rung required to begin a promote. |
| `probeFreshnessMs` | float | `500.0` | >= 0 ms | Maximum age of the last probe measurement accepted for a promote decision. Consistent with `PROBE_RX_L=50` ms so a probed rung is never considered stale between wfb_rx stats batches. |
| `promoteDebounceWindows` | int | `3` | positive int | Number of consecutive ticks the probe rung must read clean+fresh before a promote fires. At 10 Hz, the default 3 = 300 ms debounce. |
| `videoDemotePer` | float | `0.05` | [0, 1] | Residual-loss rate (PER) threshold for a loss demote (the single loss-demote threshold). |
| `emergencyFecPressure` | float | `0.80` | [0, 1] | FEC work rate threshold for the Channel-B emergency demote (FEC pressure ≥ this means the link is close to saturation). |
| `holdModesDownMs` | int | `2000` | >= 0 ms | Cooldown after any demote before the selector may promote again. Demotes bypass this cooldown (they are always immediate). |
| `minBetweenChangesMs` | int | `200` | >= 0 ms | Minimum interval between any two MCS changes (promote or demote). |
| `starvationWindows` | int | `5` | positive int | Consecutive windows with packet rate below `smoothing.starvationThresholdPps` before `link_starved` feeds the emergency demote. At 10 Hz, the default 5 = 0.5 s total-blackout failsafe. |
| `lossWindows` | int | `2` | positive int | Consecutive windows with residual loss ≥ `videoDemotePer` before a loss demote. Filters single-window transients (200 ms @ 10 Hz). |

### `dynamicLink.smoothing` — advanced signal-aggregation tuning

Source: `gs/fpvdgs/dynlink/signals.py` (`SignalAggregator`).

| Key | Type | Default | Valid | Purpose |
|-----|------|---------|-------|---------|
| `ewmaAlphaRssi` | float | `0.2` | (0, 1] | EWMA decay weight for the normalized RSSI signal. Higher = faster response; lower = smoother. |
| `ewmaAlphaFec` | float | `0.2` | (0, 1] | EWMA decay weight for the FEC work rate signal. |
| `ewmaAlphaBurst` | float | `0.1` | (0, 1] | EWMA decay weight for the burst rate signal. |
| `starvationThresholdPps` | float | `50.0` | >= 0 | Data fragment rate (packets/sec) below which a window is counted as starved. Well below normal FPV video (~700 – 1500 pps) but above background noise from a stalled stream. |

### `dynamicLink.flightlog`

| Key | Type | Default | Purpose |
|-----|------|---------|---------|
| `flightlog.enabled` | bool | `true` | Write per-tick JSONL flight logs to the frozen storage path. Disable to stop logging without changing other behavior. |

---

## Frozen constants

These values are compiled into the code and cannot be changed via config. A code
change + redeploy is required to modify them.

### Probe measurement constants

Source: `gs/fpvdgs/probe/config_build.py`.

| Constant | Value | Purpose |
|----------|-------|---------|
| `PROBE_PORT` | `50` | wfb radio_port for the observe-only probe wfb_rx/wfb_tx pair. Must match `drone/src/probe/wire.hpp` `kProbeRadioPort`. |
| `PROBE_RX_L` | `50` ms | wfb_rx log interval (`-l`). Consistent with `selector.probeFreshnessMs=500` so a probed rung is never stale between stats batches. |
| `PROBE_EWMA_ALPHA` | `0.25` | Per-MCS PER EWMA smoothing weight inside the probe receiver. |
| `PROBE_BLACKOUT_WINDOWS` | `10` | Consecutive empty windows before the probe treats a rung as fully lost (per=1.0). |

### RSSI normalization curve

Source: `gs/fpvdgs/dynlink/signals.py` (`RssiNormConfig`) + `drone/src/dynlink/txpower_curve.hpp`.

The curve is supplied by the drone at the connect event (`radio.txPowerCurve` in the
`DRONE_CONNECTED` payload) and bound by the controller — the drone is the single source
of truth. Normalization is identity (raw RSSI) until a valid curve arrives.

| Constant | Meaning |
|----------|---------|
| `p_ref_dbm` | Reference TX power (max of the curve). All RSSI readings are normalized to this level. |
| `tx_power_dbm_by_mcs` | Per-MCS TX power (dBm) from the drone's anti-overdrive curve. RSSI is adjusted by `p_ref - curve[mcs]` so comparisons across MCS rungs are EIRP-equivalent. |

### Learned-prior internals

The learned RSSI→MCS-ceiling prior is always-on (constructed unconditionally
whenever dynamic link runs). All internals are frozen; the prior is persisted at
`/etc/fpvd/learned/<adapterId>.json` on the GS (keyed on the drone-reported
`radio.adapterId`, e.g. `bl-m8812eu2`).

Source: `gs/fpvdgs/dynlink/learned_prior.py` (`LearnedPriorConfig`).

| Constant | Value | Purpose |
|----------|-------|---------|
| `bin_width_db` | `2.0` dB | RSSI histogram bin width. |
| `rssi_min` | `-90.0` dBm | Lower edge of the RSSI histogram. |
| `rssi_max` | `-30.0` dBm | Upper edge of the RSSI histogram. |
| `ewma_alpha` | `0.1` | Per-cell EWMA decay for the clean-rate estimate. |
| `viable_threshold` | `0.99` | Minimum clean-rate EWMA to count a bin+rung cell as viable. |
| `min_samples_warmstart` | `20` | Minimum observations per cell before the warm-start seed fires. |
| `min_samples_predictive` | `40` | Stricter threshold for the predictive-demote path. |
| `warmstart_margin` | `0` rungs | Safety margin subtracted from the warm-start ceiling. |
| `predictive_horizon_ticks` | `3` ticks | How many ticks ahead to project RSSI when computing predictive demote. |
| `predictive_debounce_windows` | `3` ticks | Consecutive predictive-demote signals before firing. |
| `flush_interval_observations` | `50` | Persist the prior to disk every N ingested observations. |
| `persist_dir` | `/etc/fpvd/learned` | Directory for the per-profile prior JSON files. |

### Flight-log storage

Source: `gs/fpvdgs/dynlink/flightlog.py` (`FlightLogConfig`).

| Constant | Value | Purpose |
|----------|-------|---------|
| `dir` | `/media/dvr/log/dynamic-link/` | Directory for per-flight JSONL files. |
| `max_files` | `8` | Maximum number of flight files retained (oldest pruned). |
| `max_mb` | `4.0` MB | Maximum size of a single flight file before it is rotated. |
| `flight_gap_s` | `15.0` s | Link gap longer than this causes the next healthy tick to open a new flight file. |

### Video stream identifier

Source: `gs/fpvdgs/dynlink/controller.py`.

| Constant | Value | Purpose |
|----------|-------|---------|
| `videoStreamId` | `"video"` | Substring matched against wfb stats record IDs to select the video rx stream (mavlink/tunnel rx are excluded). Frozen to avoid inadvertently driving the selector on uplink streams. |

---

## Workflow

**Discover current values.** `GET /gs/config` returns the full effective config
with every exposed knob materialized. `fpvd --dump-config` prints the code
defaults (same tree, before any operator overlay is applied).

**Tune live.** `PATCH /gs/config` with the sub-block you want to change, then
`POST /gs/apply`. No wfb restart — `dynamicLink`-only changes are applied in-process.

**Analyze a flight.** Flight logs are one JSONL per flight at
`/media/dvr/log/dynamic-link/` on the GS. Analyze offline with
`gs/tools/flightlog_analyze.py <file>.jsonl [--plot out.png]`.
