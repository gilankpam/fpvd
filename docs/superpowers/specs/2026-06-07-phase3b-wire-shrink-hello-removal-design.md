# Phase 3b — Wire Shrink + HELLO Removal + GS Teardown Design

**Date:** 2026-06-07
**Status:** Draft for review
**Target:** GS `fpvdgs.dynlink` + drone `drone/src/dynlink` (coordinated)
**Refines:** `2026-06-06-probe-driven-link-control-design.md` §5/§9 (the Phase-3 wire shrink + HELLO removal)
**Builds on:** Phase 3a (`2026-06-07-phase3a-drone-local-compute-design.md`) — the drone already computes its own bitrate/k/n/depth/tx_power.

---

## 1. Purpose & scope

The coordinated drone+GS **flag-day** that finishes Phase 3. After 3a the drone computes everything the old wire carried; this phase removes the now-dead carriage and machinery:

- **Shrink the wire** to `{mcs}` (the GS→drone `Decision` drops bitrate/k/n/depth/tx_power/fps/bandwidth).
- **Remove HELLO/HelloAck** and the `awaiting_drone_config` sync-gate.
- **Remove PING/PONG** (dormant timesync) and the drone→GS back-channel.
- **Tear down the dead GS machinery** (bitrate / FEC-escalator / latency-predictor / drone-config), gutting `Policy` to "run the selector, emit `{mcs}`."

**Pure subtraction — no new behavior.** The selector (Phase 2) and the drone-local compute (Phase 3a) are unchanged in function.

**Prerequisite:** Phase 3a deployed **and** hardware-validated. The wire stops carrying bitrate/k/n/depth/tx_power, so the drone MUST already be computing them locally before this lands. A v3 GS talking to a pre-3a drone would starve the drone of those fields.

**Out of scope:** the selector logic, the probe plumbing, the drone-local compute (all unchanged); the Phase-4 learned prior.

## 2. Locked design decisions (from brainstorming)

- **Wire = `{mcs}` only.** Bandwidth leaves the wire too — the drone reads `link.width` (3a's `d.bandwidth` source switches from the wire to config). `timestamp_ms` is dropped (the drone watchdogs on arrival time, not the wire stamp).
- **No sync-gate, no readiness handshake.** The GS emits real `{mcs}` immediately once `dynamicLink.enabled`; the drone's watchdog (no decisions → safe) + dedup-reset cover startup, GS restart, and drone reboot. `generation_id` reboot-detection is dropped.
- **PING/PONG removed** on both sides (no active consumer; the OSD-latency feature that would use them is out of scope). Re-introduce if/when that feature is built.
- **Full GS teardown in this phase** (not deferred to a later pass): delete the dead modules + gut `Policy`.
- **Hard flag-day.** v3 and v2 are mutually incompatible by the version byte; a skewed pair falls to the drone's safe defaults (video up, no adaptation) until both are v3. No transitional dual-decode.

## 3. The v3 wire

The `Decision` packet shrinks from 31 to 15 bytes (big-endian):

```
off  size  field
 0    4    magic    = 0x444C4B31 ('DLK1')   # unchanged protocol-family magic
 4    1    version  = 3                       # was 2 — the cross-version gate
 5    1    flags    (reserved, 0)
 6    4    sequence                           # monotonic; drives drone dedup
10    1    mcs
11    4    crc32(bytes[0..10])
= 15 bytes on-wire (11 payload + 4 CRC)
```

**Dropped fields:** `_pad`, `timestamp_ms`, `bandwidth`, `tx_power_dBm`, `k`, `n`, `depth`, `bitrate_kbps`, `fps`. The magic is unchanged (it identifies the protocol family); the **version byte (2→3) is the compatibility gate**. `decodeDecision` rejects any packet whose version ≠ 3 (`BadVersion`) and any whose length ≠ 15 / CRC mismatches (`Short`/`BadCrc`).

The GS `wire.py` `Encoder` and the drone `wire.cpp` `encode/decodeDecision` change together; the cross-language contract test is updated to the v3 layout.

## 4. HELLO + sync-gate removal

**Removed (GS):** `drone_config.py` (`DroneConfigState`, the sync state machine, `generation_id`, the vanilla-wfb-ng flag tracking); the HELLO listener wiring + `_on_hello`/HelloAck-send in `controller.py`; the `awaiting_drone_config` early-return + `_safe_decision`-until-synced guard in `policy.py`; `Hello`/`HelloAck` in `wire.py` (structs + encode/decode).

**Removed (drone):** `hello.cpp`/`hello.hpp` (`HelloSm` — announce/keepalive/ack); the HELLO send loop, ack handling, and the `generationId` constructor param + plumbing in `controller.cpp`; `Hello`/`HelloAck` in `wire.cpp`/`wire.hpp`.

**Replacement:** none. `Policy.tick()` emits a real `{mcs}` decision from the first tick (the Phase-2 cold-start seed + probe drive it). The drone applies its configured safe defaults on startup until the first decision arrives (UDP loss is fine), and its watchdog returns it to safe whenever decisions stop (GS restart, drone reboot, version skew). The drone's dedup resets on a safe-recovery, so a restarted GS (sequence reset) is re-accepted with no handshake.

## 5. PING/PONG removal

**Removed (both sides):** `Ping`/`Pong` structs, `encodePing`/`decodePing`/`encodePong`/`decodePong`, the `PacketKind::Ping/Pong` arms, and the GS `tunnel_listener.py` PONG dispatch. The drone→GS UDP back-channel is gone entirely — the GS reads drone state via the `/air` HTTP proxy. (`peekKind` reduces to `Decision` vs `Unknown`.)

## 6. GS teardown

- **Delete:** `bitrate.py` (airtime model + `BitrateConfig`/`effective_phy_Mbps`/`compute_*`), `predictor.py` (`PredictorConfig`/`fit_or_degrade`), `dynamic_fec.py` (`NEscalator` + `compute_k`/`compute_n` + `DynamicFecConfig`), `drone_config.py`, `tunnel_listener.py`.
- **Gut `policy.py`:** `Policy.tick()` runs the `LeadingSelector` (Phase 2) + the RSSI cold-start, and emits a `{mcs}`-only `Decision`. Drop the bitrate/k/n/depth/tx_power computation, `_compute_tx_power`, the predictor feed, the `signals_snapshot` bitrate/fec fields tied to the removed pipeline, and the sync-gate. The selector's reactive-demote inputs (`residual_loss_w`, `fec_work`, `link_starved`) stay.
- **Shrink `decision.py`:** the `Decision` dataclass becomes `{mcs}` + header fields (sequence); drop the dropped wire fields.
- **`return_link.py`:** drop the HelloAck send path; keep the decision send.
- **`controller.py`:** drop the HELLO listener + `drone_config` + HelloAck wiring; keep the `SignalAggregator → Policy → wire encode → ReturnLink` core and the probe wiring.
- **Kept:** `signals.py` (feeds the selector), `policy.py`'s `LeadingSelector`/`GateConfig`/cold-start, `profile.py` (RSSI prior, `snr_floor_dB` already deprecated), the probe controller, `config_build.py` (with new deprecations).

## 7. Drone teardown

- **Delete:** `hello.cpp`/`hpp`; the `Ping`/`Pong`/`Hello`/`HelloAck` wire code in `wire.cpp`/`wire.hpp`.
- **`controller.cpp`:** remove the `generationId` param from `start()` and its plumbing, the HELLO emplace + send/ack handling, and the Ping/Pong/Hello/HelloAck branches of the receive switch. The decision branch (`applyLocalCompute` + `dispatchTxApply`) is unchanged except `d.bandwidth` → `cfg` link width (it no longer arrives on the wire).
- **`decodeDecision`:** parse the 15-byte v3; `version != 3` → `BadVersion`.

## 8. Config delta

- **Deprecate (GS, parse-and-ignore with a warning — the Phase-2 pattern):** the bitrate/FEC/predictor/tx_power knobs that moved to the drone or were retired — `tuning.fec.*` (the escalator knobs), `tuning.policy.bitrate.*`, `tuning.policy.starvation`/predictor knobs, the GS tx_power range. The selector knobs (`tuning.gate.*` probe + emergency, `profile_selection.*`) stay.
- **Drone:** the 3a bitrate-engine knobs (`dynamicLink.bitrate`/`dynamicLink.fec`) stay; `dynamicLink.interleavingSupported` stays (the drone still applies the constant depth via it); `dynamicLink.safe.*` stays (the watchdog fallback). No new knobs.
- **Unchanged:** the wfb link config (channel/width/etc.), the probe constants.

## 9. Rollout — the flag-day

v3 ⟷ v2 are mutually incompatible:
- a v2 (31-byte, version=2) decision reaching a v3 drone fails the version check (`BadVersion`);
- a v3 (15-byte) decision reaching a v2 drone fails its size/CRC check.

Either skew ⇒ the receiver gets **no valid decisions** ⇒ the **drone watchdog** drops to safe defaults (`dynamicLink.safe.*`): video stays up at a safe MCS with no adaptation until both ends are v3. Deploy order is therefore irrelevant; the only cost is a brief no-adaptation window during the coordinated deploy. **No transitional back-compat** — the v3 drone does not decode v2.

Recovery if a deploy half-fails (one side v3, the other stuck v2): finish the other side's deploy, or roll both back to the last v2 build. The link never drops video — it sits at safe defaults.

## 10. Testing

- **Unit (wire):** v3 `Decision` encode/decode round-trip (15 bytes, fields, CRC) on both GS (`wire.py`) and drone (`wire.cpp`); a v2-bytes buffer decodes to `BadVersion` on the drone and is rejected by the GS decoder; the cross-language contract test asserts GS-encode == drone-decode for the v3 layout.
- **Unit (GS policy):** `Policy.tick()` with no `drone_config` emits a real `{mcs}` decision immediately (no safe-until-synced); the emitted `Decision` carries only `{mcs}` (+ sequence). The selector/cold-start/demote behavior is unchanged (existing Phase-2 tests, adapted to the `{mcs}`-only `Decision`).
- **Regression (deletes):** remove the tests for the deleted modules (`test_dl_bitrate`/`predictor`/`dynamic_fec`/`drone_config`/`tunnel_listener`/HELLO/PING-PONG on both sides); the remaining suites stay green.
- **Drone unit:** `decodeDecision` v3 + v2-rejection; `controller` decision dispatch with the shrunk decision (bandwidth from `cfg`).
- **Hardware (coordinated deploy):** deploy drone+GS together; confirm adaptation resumes (the drone's locally-computed bitrate/k/n track the GS `{mcs}`), `waybeam`/`wfb_video_tx` PIDs unchanged (no runner bounce), a forced degradation still reactive-demotes, and disable tears down cleanly. Verify a deliberately-skewed pair (one side old) falls to safe defaults with video intact.

## 11. Relationship to other phases

- **Phase 3a** (prereq): drone-local bitrate/k/n/depth/tx_power; this phase removes the wire carriage + GS machinery that 3a made dead.
- **Phase 4** (parent §4.8): the learned RSSI/SNR→ceiling prior — orthogonal; layers onto the selector later.
- After 3b the wire is `{mcs}`-only, the GS holds only the selector + probe + signals, and the drone owns all rate/FEC/power decisions — the end-state of the probe-driven redesign.

## 12. Self-review

- **Spec coverage (parent §5/§9):** wire → `{mcs}` ✓ (§3); HELLO + sync-gate removed ✓ (§4); airtime model / predictor / dynamic-FEC escalator removed ✓ (§6); constant tx_power/depth already in 3a. **Deviation from parent §5:** bandwidth also leaves the wire (parent said "`{mcs}` + initial bandwidth") — bandwidth is static config the drone already holds (`link.width`); documented §2/§3. PING/PONG removal is beyond the parent (they postdate it) — §5.
- **Ambiguity:** "no readiness gate" is explicit — the GS emits immediately; drone watchdog/dedup are the only safety net (§4). Hard flag-day, no dual-decode (§9).
- **Scope:** one coordinated drone+GS plan. Large but cohesive (all subtraction around a single wire-version bump). Could split GS-delete vs drone-delete in the *plan's* task ordering, but it's one flag-day deploy.
- **Placeholders:** the exact deprecated-knob list (§8) is finalized against the live config during planning; the removal set (§6/§7) is enumerated. No TBDs.
