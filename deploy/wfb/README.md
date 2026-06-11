# wfb-ng swfec deployment + cutover

Builds come from `~/Projects/poc/wfb-ng` branch `swfec`:
- drone: cross-build `make wfb_tx wfb_rx wfb_tun` with the drone toolchain env
- GS: native `make wfb_rx wfb_tx`

## Staged cutover (order matters)

1. **Binaries + full wfb_ng python package first, behavior unchanged.** With
   `link.fec.mode` still `"rs"`: `./deploy-drone.sh` then `./deploy-gs.sh`.
   The GS script ships the binaries AND the fork's complete `wfb_ng` python
   package together — the GS's previous (interleav-fork) `services.py` passes
   `-X` to every wfb_tx it spawns, which the swfec-fork binary rejects at
   spawn, killing the GS uplink. New code, old RS behavior, contract v3 live
   on the stats feed.
   - Verify: video up, GS `:8103` JSON shows `"contract_version": 3`,
     probe + dynamic link still driving MCS.
2. **fpvd both ends.** `deploy/drone/deploy.sh` (new fpvd: swfec schema,
   interleaver removed) and `deploy/gs/deploy.sh` (fpvdgs accepts v3).
   - NOTE: deploy fpvd only AFTER the wfb binaries. OLD fpvd with
     interleavingSupported=true against NEW binaries would push CMD 5,
     which the new wfb_tx rejects with an error response; fpvd logs it on
     every dynlink dispatch — noisy but not fatal. Keep the window short.
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
