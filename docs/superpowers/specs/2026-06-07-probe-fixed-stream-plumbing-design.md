# Probe Link — Fixed Retunable Stream Tied to Dynamic-Link (Plumbing Refactor)

**Date:** 2026-06-07
**Status:** Draft for review
**Target:** OpenIPC drone (Sigmastar SSC33x) + GS, fpvd codebase; RTL8812EU link
**Refines:** `2026-06-06-probe-driven-link-control-design.md` §4.3 (probe link) / Phase 1 — replaces the *config-driven, variable-`mcsList`* probe (Phase 1a/1b) with a single fixed, live-retuned stream.

---

## 1. Purpose

Phase 1a/1b shipped the probe as a **config-driven set**: `probe.mcsList` (1 `wfb_tx`+feeder per MCS) applied via `PATCH /config` + `/apply`. Two problems surfaced:

1. **It glitches video.** On the drone, any `probe` change sets `diffSubsystems().probeChanged` → `needsRebuild` → a full `orch_.stopAll()/startAll()` that restarts **every** process, including `wfb_video_tx` and `waybeam`. Confirmed on hardware: changing `mcsList` flipped all PIDs (`waybeam 780→2218`, all `wfb_tx` re-spawned). This violates the parent spec's success criterion #2 ("the probe's promotion never glitches the feed").
2. **It's the wrong shape for Phase 2.** The selector only needs an **upward boundary probe** (`current+1`); the config-driven variable set is heavier than needed and would reconfigure constantly as the operating MCS moves.

This refactor makes the probe a **single fixed stream that tracks `current+1` by live retune**, owned by the dynamic-link lifecycle, with **no probe config**. It removes the glitch at its root (the `probeChanged → rebuild` path is deleted) and is the foundation Phase 2's selector sits on.

**Scope:** plumbing only. The probe stays **observe-only** (measured, surfaced in `/status`); the promote-on-probe / demote-on-video-PER **selector is a separate later plan** (parent spec Phase 2). The GS continues to choose the operating MCS with its existing selector; the probe simply tracks it.

## 2. Key decisions (from design review)

- **One probe stream**, targeting **`current+1`**. Monotonicity (parent §2.2) makes a single upward boundary probe sufficient for a safe promote; demotion is video-PER-driven and never needs the probe. Dropping the 2nd (`current+2`) stream halves airtime — worst at the floor, where it costs most — at the cost only of climb *speed* (one rung/interval instead of two). The gated 2nd stream is a clean future extension on this same mechanism if climb speed proves too slow.
- **Drone-autonomous retune.** The GS keeps sending only the operating `{mcs}` on the existing dynlink wire — **no wire change, no new control surface**. The drone derives `current+1` and retunes its probe `wfb_tx` live. The GS auto-reads which MCS the probe carries from the `RX_ANT` radiotap key.
- **Fixed `radio_port`s, both ends** — never change during operation.
- **Lifecycle = dynamic-link.** Probe TX (drone) and probe RX (GS) start when `dynamicLink.enabled` and stop when it's disabled. No separate `probe.enabled`.
- **No probe config.** The entire `probe` block is removed from both schemas; probe parameters become hardcoded constants.
- **Ceiling = clamp, no idle.** The probe always sits at `min(current+1, maxMcs)`. At the ceiling it re-reads `maxMcs` (a bonus clean read of the top rung). Zero process churn during operation — the probe processes start/stop only with dynamic-link; in between they are only *retuned*. Cost: ~0.5% redundant airtime at the ceiling.
- **Probe FEC = `k/n = 1/1`** (FEC off) — the GS reads raw, unmasked on-air PER.

## 3. Architecture / data flow

```
GS selector → operating {mcs} ──(existing dynlink wire)──▶ DRONE controller (on {mcs}):
                                                            ├─ retune video tx  → CMD_SET_RADIO (live; already works)
                                                            ├─ retune PROBE tx  → CMD_SET_RADIO to min(mcs+1, maxMcs)  [NEW, live]
                                                            └─ recompute bitrate with probe-airtime reserve (parent §4.5)
   GS probe wfb_rx (fixed port) ◀──── 1 FEC-off stream @ current+1 ───────────┘
        └─ per-MCS PER/RSSI keyed by RX_ANT mcs → /status   (observe-only)
```

No new cross-link surface: the drone manages the probe rung from the `{mcs}` it already receives (parent §4.3 "the drone manages the set autonomously — no extra control input").

## 4. Drone changes (`drone/`, C++)

- **Probe spec:** one long-lived `wfb_tx` + feeder on a fixed `radio_port`, built once at dynamic-link start. Add a **control port** (`-C <probe_ctl_port>`) to the probe `wfb_tx` argv (it currently has none) so it can be retuned. FEC `-k 1 -n 1`. PHY mirrors video (`-B`, `-S`, `-L`, long GI), differing only in `-M`.
- **Lifecycle:** the probe pair is started by `dl_.start()` and stopped by `dl_.stop()` (lifecycle = dynamic-link), not by config apply.
- **Retune:** in the controller's `{mcs}` handler (where it already emits `CMD_SET_RADIO` to the video tx), also send `CMD_SET_RADIO` to the probe `wfb_tx` control port with `mcs = min(current+1, maxMcs)`. Live; no restart.
- **Delete the glitch path:** remove `probe` from the config schema, `diffSubsystems().probeChanged`, the `probeChanged → needsRebuild` term, and `buildProbeSpecs`-from-`mcsList`. With probe no longer in the config diff, a (now-removed) probe change can never trigger `needsRebuild`/`stopAll`.
- **Bitrate reserve:** the drone bitrate calculator (parent §4.5) subtracts `probe_util` from the `2/3` utilization. `probe_util = probe_kbps / baseRate[bw][gi][min(mcs+1, maxMcs)]`, `probe_kbps = probe_pps × probe_packet_bytes × 8 / 1000` (FEC 1/1 ⇒ no inflation).
- **Constants** (replacing config): probe `radio_port`, probe `-C` control port, `probe_pps`, `probe_packet_bytes`. (`maxMcs` already lives in `dynamicLink`.)

## 5. GS changes (`gs/fpvdgs/`, Python)

- The Phase 1b `ProbeController` shrinks to **one `wfb_rx` on the fixed `radio_port`**, started/stopped with **`dynamicLink.enabled`** (hooked into the same place dynlink starts/stops), not `probe.enabled`.
- **Keep** the RX_ANT-keyed `McsAggregator` as-is — it labels MCS from the radiotap, so it transparently follows the probe as it retunes across rungs (parent §4.3 "handles the port→MCS remap as `current` moves").
- **Remove** the `probe` config block + schema validation, and the `/apply` `_route_probe` + `wfb_changed` probe-exclusion added in Phase 1b (no probe config left to route; lifecycle now rides `_route_dynamic_link`/`App.start`).
- `/status` still surfaces the `probe` block (per-MCS `per`/`rssi`/`snr`) — now reflecting the single retuned stream.
- **Constants:** probe `radio_port` (matches the drone's), `rxL`.

## 6. Config / schema delta

- **Removed (both ends):** the entire `probe` block — `enabled`, `basePort`, `maxStreams`, `rxL`, `mcsList`, `pps`, `packetBytes`, `baseFeedPort` — and its schema/validation.
- **New constants:** drone — probe `radio_port` (**50**, reusing the Phase-1a value), probe `-C` control port (**8001**; the video tx uses 8000), `probe_pps` (**25**), `probe_packet_bytes` (**1400**), FEC `1/1`; GS — probe `radio_port` (**50**, must match the drone), `rxL` (**50 ms**). The two `radio_port`s must agree.
- **Lifecycle:** probe (both ends) follows `dynamicLink.enabled`.

## 7. Testing

- **Drone (doctest):** a `{mcs}` change retunes the probe `wfb_tx` (asserts the control-port `CMD_SET_RADIO` to `min(mcs+1, maxMcs)`) and does **not** restart `wfb_video_tx`/`waybeam` (the regression test pinning the old glitch); probe lifecycle starts/stops with dynamic-link; clamp at the ceiling; bitrate reserve math (`(2/3 − probe_util)`, including the ceiling clamp). Build/run per the drone workflow (`cmake --build build -j && ./build/fpvd_tests`).
- **GS (pytest):** one `wfb_rx` on the fixed port; starts/stops with `dynamicLink.enabled`; the aggregator follows a retuned MCS across rungs (RX_ANT key changes → slot moves); `/status` probe block reflects the single stream.
- **On-hardware smoke:** enable dynamic-link; confirm one probe `wfb_tx` (drone) + one `wfb_rx` (GS); drive an MCS change and confirm (a) the probe rung tracks `current+1` in `/status`, (b) `wfb_video_tx`/`waybeam` PIDs are unchanged across the change, (c) video is undisturbed.

## 8. Relationship to the parent spec

This refines parent §4.3 (probe link) and replaces the Phase 1a/1b *config-driven* probe with the fixed/retuned/dynlink-tied form. It is still **Phase 1 (observe-only)** in the parent's roadmap — it does not add the selector (Phase 2) or change the bitrate engine's existence (Phase 3), though it adds the **probe-airtime reserve** to the §4.5 calculator (already recorded there).

## 9. Self-review / open items

- **`maxMcs` source:** the clamp uses `dynamicLink.maxMcs`; the drone controller already has it in its snapshot. Confirm during implementation that it's available at retune time.
- **Cold start:** at dynamic-link enable, "current" is the dynlink safe/initial MCS; the probe starts at `min(safe.mcs+1, maxMcs)`. No special case.
- **Legacy config migration:** deployed `config.json` overlays may still carry a `probe` block (e.g. the drone currently has `probe.mcsList=[2,3,4,5]`). Removing the schema key must **not** hard-fail config load on the now-unknown key — the loader ignores/strips it (and the deploy may prune it). Verify the config loader's unknown-top-level-key behavior on both ends during implementation.
- **Deferred (Phase 2):** the gated `current+2` second stream (for faster climb) — a clean extension on this mechanism (idle its feeder while `+1` is dirty); the promote/demote selector itself.
