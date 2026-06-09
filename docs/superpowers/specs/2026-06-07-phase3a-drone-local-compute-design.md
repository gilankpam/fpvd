# Phase 3a — Drone-Local Compute (bitrate / FEC / depth / tx_power) Design

**Date:** 2026-06-07
**Status:** Draft for review
**Target:** drone `fpvd` (`drone/src/dynlink/`); GS untouched
**Refines:** `2026-06-06-probe-driven-link-control-design.md` §4.5–4.7 (the Phase-3 drone bitrate/FEC/tx_power engine)
**Builds on:** Phase 1 (probe plumbing) + Phase 2 (GS probe-driven MCS selector), both deployed + hardware-validated.

---

## 1. Purpose & scope

Move the **bitrate / FEC / depth / tx_power** decisions onto the drone, computed deterministically from the commanded MCS via the **OpenIPC WFB calculator** — replacing the GS-side airtime model + dynamic-FEC escalator. This is **Phase 3a**: the *drone-local compute* half of Phase 3, deliberately split from the wire-shrink + HELLO removal (Phase 3b) so it ships drone-only and fully back-compatible.

**In scope (drone only):** the drone computes and applies its own `bitrate`, `k`, `n` from `{mcs, bandwidth}`; `depth` and `tx_power` become static constants set at radio bring-up (dropped from the per-decision apply). The drone **ignores** the GS-sent `bitrate/k/n/depth/tx_power` in the Decision packet.

**Out of scope (unchanged this phase):**
- The **wire** stays at v2 — the GS keeps sending the full Decision; the drone simply ignores the now-drone-local fields (it still reads `mcs` + `bandwidth`). No `dl_wire` change, no version bump.
- **HELLO / HelloAck / the sync-gate** — untouched (removed in 3b).
- **PING/PONG timesync** — dormant scaffolding (encode/decode exists; the only consumer is a deferred OSD `debugLatency` flag). Left as-is; `compute_k` uses a **static block-fill latency bound**, not measured RTT.
- **The GS** — no code change. Its bitrate calc + dynamic-FEC escalator keep running and keep populating the (now-ignored) wire fields; they are removed in 3b.
- **The probe plumbing** and the **GS MCS selector** (Phase 1/2) — unchanged.

**Cutover:** direct and unconditional — gated only by the existing `dynamicLink.enabled`. No shadow mode, no feature flag. Rollback = redeploy the drone.

## 2. Locked design decisions (from brainstorming)

- **Stage Phase 3** into 3a (this spec, drone-local compute, wire unchanged) and 3b (wire-shrink to `{mcs}` + HELLO removal + GS cleanup). 3a is independently shippable and hardware-validatable before the 3b flag-day.
- **Drone authoritative for all four** (`bitrate`, `k/n`, `depth`, `tx_power`); GS owns only `mcs` + `bandwidth`.
- **`depth` and `tx_power` are static constants**, **removed from the per-decision apply path** (they are radio constants, not dynamic-loop tuning). `tx_power` reuses the existing static `link.txpower` (no new config); `depth` is a compile-time constant `kInterleaveDepth = 1` (no config field — `link.depth` intentionally not added).
- **Bitrate is open-loop / feed-forward on MCS change** — the only thing that pulls bitrate down under bad RF is the GS dropping MCS (on PER), which the formula re-derives downward. No congestion sidecar.
- **`probe_util` reserve included now**, using the probe's real on-air rung `min(mcs+1, 7)` (7 = the probe's hardware ceiling from the Phase-1 plumbing, independent of the GS selector's `maxMcs`).
- **GI = long, constant.**

## 3. Architecture

Today (Phase 2): the GS computes `bitrate/k/n/depth/tx_power`, ships them in the v2 Decision, and the drone's `dispatchTxApply` applies them verbatim (`wfb_->setFec(d.k,d.n)`, `enc_->apply(d.bitrateKbps,d.fps)`, tx_power + depth via the radio).

Phase 3a: a new drone bitrate/FEC engine recomputes those values from `{mcs, bandwidth}`; `dispatchTxApply` applies the **drone-computed** `bitrate/k/n` (and the commanded `mcs/bandwidth`), and no longer touches tx_power/depth.

```
GS Decision (v2 wire, unchanged) ── mcs, bandwidth ──┐   (bitrate/k/n/depth/tx_power: ignored)
                                                     ▼
                            drone bitrate engine  ── baseRate[bw][mcs] ──► wire_target ─► k ─► n ─► bitrate
                            (OpenIPC table + compute_k + fixed ratio)                              │
                                                                                                   ▼
                                                            dispatchTxApply(mcs, bandwidth, bitrate, k, n)
                                                            tx_power = link.txpower  (static, bring-up)
                                                            depth    = link.depth    (static, bring-up)
```

New drone module(s) under `drone/src/dynlink/` (C++): a `bitrate` unit (OpenIPC table + formula) and a `fec` unit (`compute_k` + `n = ceil(k×ratio)`), ported from the GS `bitrate.py`/`dynamic_fec.py` semantics. Pure functions, unit-tested with doctest.

## 4. Bitrate engine

The OpenIPC effective-rate table is a built-in constant (kbps, already de-rated to real WFB throughput — *not* the textbook PHY table). GI = long (constant), so the table reduces to one row per bandwidth, indexed `baseRate[bw][mcs]` for MCS 0–7:

```
baseRate[20MHz] = [ 6500, 12000, 15500, 20000, 25000, 42000, 47500, 55000]   # long GI
baseRate[40MHz] = [ 9800, 18600, 30400, 40200, 55800, 80400, 90200, 97000]   # long GI
```
*(Source: OpenIPC WFB calculator, `OpenIPC/docs` → `src/components/wfb-calculator.astro`. Short-GI rows exist upstream but are unused — GI is constant long.)*

Per Decision (the result only changes when `mcs` or `bandwidth` changes):

```
probe_kbps   = probe_pps × probe_packet_bytes × 8 / 1000          # constant ≈ 280 (25 pps, 1400 B)
probe_util   = probe_kbps / baseRate[bw][ min(mcs+1, 7) ]         # probe's real on-air rung
wire_target  = baseRate[bw][mcs] × (2/3 − probe_util)             # kbps; replaces effective_phy × utilization
k            = compute_k(wire_target, mtu, fps)                   # block-fill latency sizing (§5)
n            = ceil( k × (1 + base_redundancy_ratio) )            # fixed ratio; base_red = 0.5 ⇒ n/k = 1.5
bitrate_kbps = clamp( round( wire_target × k / n ), [min_bitrate_kbps, max_bitrate_kbps] )
```

- `2/3` is the utilization factor; `k/n` is the FEC data fraction. With the default ratio (`k/n = 8/12 = 2/3`) the effective factor is `4/9 ≈ 0.444` (e.g. MCS0/20 → `6500 × 4/9 ≈ 2888 kbps`, safely under the ~3.2 Mbps measured ceiling — parent §2.3).
- `wire_target × k/n` keeps the wire (video + FEC parity) inside `baseRate × (2/3 − probe_util)`; the int-truncation rounds the wire rate **down**.
- Inputs are all drone-local: `bw, mcs` (wire), `mtu` (`link.mtu`), `fps` (`video.fps`), `probe_pps`/`probe_packet_bytes` (probe constants).

## 5. FEC — latency-sized k, fixed ratio, no escalator

Port the GS `compute_k` semantics (block-fill enforcement, `2026-05-23-block-fill-enforcement-design.md`):

```
anchor_kbps        = wire_target / (1 + base_redundancy_ratio)   # encoder bitrate at n_base
packets_per_frame  = anchor_kbps × 1000 / (fps × mtu × 8)
k                  = clamp( floor(packets_per_frame / blocks_per_frame), [k_min, k_max] )
n                  = ceil( k × (1 + base_redundancy_ratio) )
```

- `k` is anchored on the encoder bitrate at `n_base` (not the wire rate) so block-fill latency does not inflate as parity is added; `blocks_per_frame` (default `1 + base_redundancy_ratio`) keeps `block_fill ≤ frame_period` by construction.
- **Fixed redundancy ratio** — `n = ceil(k × (1 + base_redundancy_ratio))`, the *same* `8/12` data fraction the bitrate formula uses (single source of truth feeding both the radio FEC and the bitrate). **No `NEscalator`**: `n` never escalates on loss in 3a. (The GS dynamic-FEC escalator keeps running and keeps populating the ignored wire `k/n`; both are removed in 3b.)
- The wfb **RX (GS `wfb_rx`) learns `k/n` from the FEC block headers**, so the drone changing `k/n` autonomously needs no GS coordination.
- Recompute is per-Decision; the existing `applyDirection` (Up/Down bitrate ordering vs `lastEnc_`) sequences the radio vs encoder apply and no-ops when nothing changed.

## 6. Static constants — `depth` and `tx_power`

Both leave the dynamic apply path entirely and become static:

- **`tx_power` → reuse `link.txpower`** (existing static field, driver units). 3a drops tx_power from `dispatchTxApply`; power stays at the radio bring-up value. The wire's `tx_power_dBm` and the inverse-MCS coupling (`_compute_tx_power`, GS-side) are ignored. Operator tunes range via `link.txpower`.
- **`depth` → fixed constant `1`** (a compile-time `kInterleaveDepth`; **no config field** — `link.depth` is intentionally *not* added). 3a drops depth from the per-decision apply: the drone sets the interleave depth once (to the constant, when `interleavingSupported`) on the first apply and never modulates it. Safe mode keeps using `dynamicLink.safe.depth` unchanged.

`dispatchTxApply` in 3a therefore touches only: **`mcs`, `bandwidth` (from wire) + computed `bitrate`, `k`, `n`.**

## 7. Config delta (drone)

**Reuse — static (`link`/`video`):**
- `link.txpower` — now the sole tx-power source while dynamicLink is on.
- `link.mtu`, `video.fps` — bitrate/k inputs.

**No new field for `depth`** — it is a compile-time constant (`kInterleaveDepth = 1`); see §6.

**Add — bitrate-engine tuning (stays under `dynamicLink`, the dynamic loop's computation config):**
- `dynamicLink.bitrate`: `min_bitrate_kbps` (1000), `max_bitrate_kbps` (24000).
- `dynamicLink.fec`: `base_redundancy_ratio` (0.5 ⇒ n/k = 1.5), `k_min`, `k_max`, `blocks_per_frame`.

**Built-in code constant:** the OpenIPC `baseRate` table (20/40 MHz, long GI). Probe `pps`/`packet_bytes` come from the existing probe constants.

**Unchanged:** the v2 wire/Decision schema, `dynamicLink.safe.*` (still the degraded/unsynced fallback), `dynamicLink.enabled`, all GS config.

## 8. Removed vs kept

**Removed from the drone's dynamic apply (`dispatchTxApply`):** reading `d.bitrateKbps`, `d.k`, `d.n`, `d.depth`, `d.txPowerDbm` from the Decision. (The fields remain on the wire/struct — ignored.) Per-decision tx_power and depth application.

**Added (drone):** the bitrate engine (table + formula) and the FEC `compute_k`/`n` unit; a `kInterleaveDepth` constant; the `dynamicLink.bitrate`/`dynamicLink.fec` tuning.

**Kept:** the v2 wire + `dl_wire` codec; HELLO/HelloAck/sync-gate; PING/PONG (dormant); the probe plumbing; `mcs`/`bandwidth` apply; `applyDirection` ordered apply; `dynamicLink.safe` fallback; the GS in its entirety (its bitrate/FEC machinery runs but is ignored — removed in 3b).

## 9. Telemetry caveat

During 3a the GS `/status` still shows the GS-computed `bitrate/k/n` (no longer what flies); the drone's `/air/status` reports the truth (the applied, drone-computed values). Fully reconciled in 3b when the GS stops computing them. Note this in the smoke checklist so the GS/drone mismatch isn't read as a bug.

## 10. Testing

- **Drone unit (doctest):**
  - `baseRate` lookup + the bitrate formula: `probe_util` (incl. the `min(mcs+1,7)` clamp), `wire_target`, `wire_target × k/n`, the `[min,max]` clamp, int rounding. Spot values cross-checked against the parent §4.5 worked example (MCS0/20 ≈ 2888 kbps pre-clamp).
  - `compute_k` + `n = ceil(k×(1+ratio))`: port the GS `dynamic_fec` cases (block-fill bound, `k_min/k_max` clamp).
  - Recompute-on-change: same `(mcs,bw)` ⇒ identical output; an MCS step ⇒ bitrate/k/n move monotonically with the table.
- **Cross-check (bench):** sweep `(mcs, bandwidth)` and compare drone-computed `bitrate/k/n` against the GS values — confirm they sit in the expected ballpark (OpenIPC numbers run a bit more conservative than the retired airtime model).
- **On-hardware smoke (drone redeploy only):** enable `dynamicLink`; confirm the drone applies its own `bitrate/k/n` (visible in `/air/status`, independent of the GS-sent values); video healthy across GS-driven MCS changes; `waybeam`/`wfb_video_tx` PIDs unchanged (no runner bounce); tx_power + depth stay at their static `link.*` values throughout.

## 11. Relationship to other phases

- **Phase 3b** (next): shrink the wire to `{mcs}` (+ initial bandwidth), remove HELLO/HelloAck + the `awaiting_drone_config` sync-gate, and delete the now-dead GS machinery (`bitrate.py` airtime model, `PredictorConfig`/`fit_or_degrade`, `NEscalator`/dynamic-FEC, the GS tx_power/depth computation). A coordinated drone+GS wire-version bump (flag-day).
- **Phase 4** (parent §4.8): the learned RSSI/SNR→ceiling prior — orthogonal to 3a.

## 12. Self-review

- **Spec coverage (parent §4.5–4.7):** OpenIPC table + `(2/3 − probe_util) × k/n` bitrate ✓ (§4); latency-sized `k` drone-side + fixed ratio, no escalator ✓ (§5); constant depth + tx_power ✓ (§6). **Deviation:** `probe_util` clamps at the probe's hardware ceiling `7`, not `maxMcs` (parent §4.5) — matches the deployed Phase-1 plumbing; documented §2/§4. **Staging deviation:** the wire-shrink + HELLO removal + GS cleanup the parent bundles into "Phase 3" are deferred to 3b — documented §1/§11.
- **Ambiguity:** `tx_power`/`depth` are static (bring-up), not per-decision — §6. The drone reads only `mcs`+`bandwidth` from the wire — §3. `probe_util` uses the probe's real rung `min(mcs+1,7)` — §4.
- **Scope:** single drone-only implementation; no GS/wire/HELLO change. Independently deployable and reversible (redeploy).
- **Placeholders:** the bitrate-engine constants (`base_redundancy_ratio`, `k_min/k_max`, `blocks_per_frame`, `min/max_bitrate_kbps`) and `kInterleaveDepth` are starting values mirroring the current GS config / `safe.depth` — tune on hardware (depth would need a code change, by design), not blockers.
