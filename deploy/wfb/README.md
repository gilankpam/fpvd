# wfb-ng swfec deployment + cutover

Builds come from `~/Projects/poc/wfb-ng` branch `swfec`:
- drone: cross-build `make wfb_tx wfb_rx wfb_tun` with the drone toolchain env
- GS: native `make wfb_rx wfb_tx`

## Staged cutover (order matters)

**Per-host ordering is asymmetric** because of the v3 stats contract:
- The new `wfb_rx` emits `contract_version: 3`. The GS `fpvdgs` *consumes*
  the `:8103` feed and hard-aborts (`ContractVersionError`) on a version it
  doesn't know. The currently-deployed fpvdgs only accepts `{1,2}`. So on the
  **GS, the new fpvdgs (accepts `{1,2,3}`) MUST land before the new wfb_rx** —
  fpvdgs first, then wfb. (New fpvdgs tolerates the old v2 feed, so there is
  no reverse hazard.)
- The **drone** fpvd never parses `contract_version`, so its order is free.
  Do wfb binaries first there so the brief old-fpvd→new-wfb_tx CMD-5 rejection
  window (interleave cmd, noisy-not-fatal) is short.

1. **Drone wfb binaries.** With `link.fec.mode` still `"rs"`:
   `./deploy-drone.sh`. Old fpvd keeps running (RS); expect transient CMD-5
   rejection log noise until step 3.
2. **GS new fpvdgs, THEN GS wfb.** `deploy/gs/deploy.sh` first (new fpvdgs,
   accepts v3, still reading the old v2 feed), then `./deploy-gs.sh` (ships
   the binaries AND the fork's complete `wfb_ng` python package — the GS's
   previous interleav-fork `services.py` passes `-X` to every wfb_tx, which
   the swfec binary rejects at spawn, killing the GS uplink, so the whole
   package goes together). After this the feed reports `contract_version: 3`.
   - Verify: video up, GS `:8103` shows `"contract_version": 3`, probe +
     dynamic link still driving MCS.
3. **Drone new fpvd.** `deploy/drone/deploy.sh` (swfec schema, interleaver
   removed). Stops the CMD-5 noise from step 1. `link.fec.mode` defaults to
   `"rs"` (WITH_DEFAULT parse of the existing on-disk config).
3. **Flip the mode.** Via the GS proxy to the drone config API:

       curl -X PATCH http://10.18.0.1:8080/air/config \
            -H 'Content-Type: application/json' \
            -d '{"link":{"fec":{"mode":"swfec"}}}'
       curl -X POST http://10.18.0.1:8080/air/apply

   (The GS `/air/*` proxy strips the `/air` prefix and forwards to the
   drone's `/config` + `/apply` endpoints directly.)

   (Mode flip = full service restart on the drone; expect a brief video
   drop. With dynamicLink enabled, `link.fec` is locked — disable DL,
   flip, re-enable, or flip before arming DL.)
   - Verify: GS stats show `"fec_type": "swfec"`, `"fec_k": 50`,
     `"fec_n": 30`; OSD FEC tuple now reads `(overhead,deadline)` —
     `(50,30)` — not k/n; fec_rec activity under induced loss.
     NOTE: a HOT overheadPct change (no restart) is not reflected in the
     GS-reported fec_k until the next session re-announce (deadline change
     or wfb_tx restart) — by design; verify overhead changes via drone
     config, not GS stats.

## Rollback

- Mode-level: PATCH `{"link":{"fec":{"mode":"rs"}}}` + POST `/air/apply` (config only).
- Binary-level: restore `/root/fpvd-rollback/wfb/*.orig` (drone) /
  `/root/fpvd-gs-rollback/wfb/*` (GS), restart `S99fpvd`.
- fpvd-level: existing `deploy/drone/rollback.sh` / `deploy/gs/rollback.sh`.

## Bench A/B before flight

Flip mode rs↔swfec at fixed MCS/power on the bench; compare
`residual_loss_w` / `fec_rec` under induced loss (see
`docs/superpowers/specs/2026-06-11-swfec-adoption-design.md` §5).
