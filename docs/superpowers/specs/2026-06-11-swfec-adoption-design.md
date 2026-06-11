# swfec adoption — sliding-window FEC for the video link

**Date:** 2026-06-11
**Status:** approved design
**Branch:** `feat/swfec-adoption` (fpvd) + `swfec` branch in `~/Projects/poc/wfb-ng`

## Summary

Replace Reed–Solomon block FEC (+ block interleaver) on the **video link** with
swfec, a sliding-window FEC already implemented and vector-verified in the
user's wfb-ng fork (`~/Projects/poc/wfb-ng`, branch `swfec`). That fork becomes
the deployed wfb-ng base on both the drone and the GS. The block interleaver is
retired — swfec's deadline-bounded window covers burst loss with less latency.

Tun and telemetry links stay RS k3/n5 (boot-once, robust, unchanged).

## Background / current state

- **Deployed wfb-ng** (drone + GS) is built from
  `~/Projects/drone/wfb-ng-interleav` @ `feat/interleaving_uep`: block
  interleaver (`CMD_SET_INTERLEAVE_DEPTH = 5`, `-J` SESSION fields), and the
  GS stats contract patch (`wfb_ng/protocols.py` emits `interleave_depth` +
  `contract_version` in SESSION; tolerant PKT parsing).
- **swfec fork** (`~/Projects/poc/wfb-ng` @ `swfec`) is based on upstream
  master + inject-retry. It adds:
  - `WFB_FEC_SWFEC = 0x2`; wire format and coefficient PRNG are **frozen
    protocol**, byte-exact against the Rust reference (`crates/swfec` in
    fpv-transport), pinned by committed test vectors.
  - `wfb_tx -z`: swfec mode; `-k` = overhead_pct (0..255), `-n` = deadline_ms
    (1..255). Live-settable through the existing `CMD_SET_FEC` (k=overhead,
    n=deadline). Mode itself is constructor-time (process restart to switch).
  - `wfb_rx`: native swfec decode, session-header driven (RX auto-follows
    `fec_type`; no RX-side config). Stats map into the stock 11-field PKT
    line: seq-gap loss → `lost`, recovered → `fec_rec`, delivered → `out`.
  - It does **not** have: the interleaver, or the GS python contract patch
    (`contract_version` etc.) — `fpvdgs.stats_client._parse_session` raises
    `KeyError` without it, so the python patch must be ported (below).
- **fpvd touchpoints:**
  - Drone spawns `wfb_tx` with `-k/-n` from `link.fec`
    (`drone/src/translate/wfb.cpp`), controls it via
    `drone/src/translate/wfb_control.*` / `wfb_cmd.h` (cmds 1=FEC, 2=RADIO,
    5=INTERLEAVE).
  - Drone-local dynlink compute (`drone/src/dynlink/fec.cpp`) derives k/n from
    a **fixed** `baseRedundancyRatio` (0.5 → n/k = 1.5) + block-fill geometry;
    `bitrate.cpp` budgets `wire_target * k / n`.
  - GS `fpvdgs` parses the wfb-ng JSON stats feed (`:8103`), hard-fails on
    unknown `contract_version` (currently `{1,2}`), and computes
    `residual_loss_w` / `fec_work_rate_w` from `lost`/`fec_rec`/`out`.
  - Deploy scripts ship fpvd only — **nothing currently deploys wfb-ng
    binaries**; they come from the system images.

## Decisions (settled with user)

1. **Fork base:** the swfec fork becomes the new deployed wfb-ng. The
   interleav fork is retired.
2. **FEC policy at adoption:** static config (overheadPct + deadlineMs), parity
   with today's fixed redundancy ratio. Adaptive overhead is explicit future
   work.
3. **Mode switch:** `link.fec.mode: "rs" | "swfec"` stays in config for
   flight-test rollback (one config apply + wfb_tx restart, no redeploy).
4. **Interleaver:** remove from fpvd now (dispatch, `interleavingSupported`,
   `kWfbCmdSetInterleaveDepth`). The new base can't speak CMD 5 even in "rs"
   mode, so it is dead protocol.
5. **Stats contract:** bump `contract_version` to **3**; fpvdgs accepts
   `{1,2,3}`. swfec semantics only ever appear at v3, so skew fails loudly.
6. **Defaults:** `overheadPct=50` (airtime parity with RS n/k=1.5),
   `deadlineMs=30` (~2 frames @60fps). Safe-mode overhead 100%.

## Design

### 1. wfb-ng fork work (`~/Projects/poc/wfb-ng`, branch `swfec`)

Port the GS-side stats contract from `wfb-ng-interleav`:

- `wfb_ng/protocols.py`:
  - Tolerant SESSION parsing (4–6 colon fields); defaults
    `interleave_depth=1`, `contract_version=1` when absent.
  - On-change + once-per-log-interval SESSION emission (dedup before notify).
  - Tolerant PKT parsing (≥11 fields; ignore extras). Do **not** port the
    interleaver counters (`bursts_rec`, `holdoff`, `late_deadline`).
  - `fec_types` mapping gains `2 → 'swfec'`.
- `wfb_ng/services.py` + `wfb_ng/conf/master.cfg`: matching plumbing from the
  interleav fork, minus interleave config keys.
- `src/rx.cpp`: SESSION IPC lines currently emit 4 fields
  (`epoch:fec_type:k:n` — verified; swfec sessions carry
  `epoch:2:overhead_pct:deadline_ms`). Extend both RS and swfec lines with
  the trailing `:1:3` fields (`interleave_depth=1` always,
  `contract_version=3`), matching the deployed interleav-fork convention so
  the C side stays the single source of truth for the contract version and
  the ported python parsing logic is unchanged.
- Tests: add a stats-format check for swfec SESSION/PKT lines alongside the
  existing byte-exact/fuzz suites.

### 2. fpvd drone changes

**Config schema** (`drone/src/config/schema.hpp`, `drone/etc/defaults.json`):

```json
"link": {
  "fec": { "mode": "rs", "k": 3, "n": 5, "overheadPct": 50, "deadlineMs": 30 }
}
```

- `mode` defaults `"rs"` until cutover. `k`/`n` keep RS meaning; the swfec
  keys are ignored in rs mode and vice versa.
- Config-apply classification: `mode` change ⇒ restart wfb_tx (the `-z` flag
  is constructor-time). `overheadPct`/`deadlineMs` changes ⇒ hot apply via
  `CMD_SET_FEC` (k=overhead, n=deadline; both uint8 ⇒ deadline caps at
  255 ms — validated at config load).
- Remove `interleavingSupported` from schema and defaults.

**argv** (`drone/src/translate/wfb.cpp`): in swfec mode the video TX gets
`-z -k <overheadPct> -n <deadlineMs>`; everything else unchanged. Tun/tlm
builders untouched.

**dynlink** (`drone/src/dynlink/`):

- swfec mode skips `computeK`/`computeN`; the Decision's k/n slots carry
  `(overheadPct, deadlineMs)` (static from config — diff-based dispatch means
  `setFec` is pushed once and on config change).
- `computeBitrateKbps`: swfec mode uses
  `wire_target / (1 + overheadPct/100)` in place of `* k/n` (same airtime
  semantics: repairs are extra packets on top of sources).
- Safe recovery: `safe.overheadPct` (default 100) + `safe.deadlineMs`
  (default 30) used in swfec mode; `safe.k/n` in rs mode.
- Remove the `setInterleaveDepth` dispatch and depth from the apply path.

**translate** (`drone/src/translate/wfb_cmd.h`, `wfb_control.*`): remove
`kWfbCmdSetInterleaveDepth` and `setInterleaveDepth`.

**Encoder packet-size guard:** swfec drops inputs >
`MAX_FEC_PAYLOAD - 14` bytes (`SWFEC_MAX_INPUT`, counted as oversize on TX).
Verify waybeam's venc chunking stays under this; enforce/assert at startup or
config load. (RS tolerates slightly larger fragments, so this is a new
constraint when flipping modes.)

### 3. fpvd GS changes

- `gs/fpvdgs/dynlink/stats_client.py`: `CONTRACT_VERSIONS_SUPPORTED =
  {1,2,3}`. `SessionInfo` notes that under `fec_type == 'swfec'`,
  `fec_k`=overheadPct and `fec_n`=deadlineMs.
- Signals/controller/probe: **unchanged**. `residual_loss_w` and
  `fec_work_rate_w` read the same PKT keys, which the swfec RX populates with
  equivalent semantics. The probe path spawns FEC-off RS `wfb_rx` (still
  supported by the new base); the GS→drone decision wire stays v3 `{mcs}`.
- GS unified-config / `/link` API: surface `fec.mode`, `fec.overheadPct`,
  `fec.deadlineMs` as shared keys pushed to the drone, like existing link
  keys.

### 4. Deployment & cutover

New `deploy/wfb/` scripts (drone + GS), following existing conventions
(`scp -O`, rollback dirs, verify step):

- **Drone:** static cross-built `wfb_tx`, `wfb_rx`, `wfb_tun` from the swfec
  fork → `/usr/bin/`, with originals preserved in `/root/fpvd-rollback/`.
- **GS:** `wfb_rx`/`wfb_tx` binaries + the patched `wfb_ng` python package
  into site-packages, originals preserved in `/root/fpvd-gs-rollback/`.

**Cutover order:**

1. Deploy new binaries both ends with `mode:"rs"` — new code, old behavior,
   contract v3 live. Verify video + stats + probe.
2. Deploy fpvd (drone) + fpvdgs (GS) with the schema/dispatch changes.
3. Flip `link.fec.mode` to `"swfec"` via config apply. Bench-verify, then
   flight-test.

**Rollback:** flip mode back to `"rs"` (config-only). Deeper rollback:
restore binaries from the rollback dirs.

### 5. Testing

- **Fork:** existing differential/fuzz tests stay green; new stats-format
  test for swfec sessions.
- **Drone** (`./build/fpvd_tests` from `drone/`, not ctest): argv building
  per mode; bitrate math in swfec mode; dispatch emits `setFec(overhead,
  deadline)` and never an interleave cmd; config validation (deadline ≤ 255,
  mode enum).
- **GS** (pytest): v3 contract acceptance; swfec session parsing; signals
  over swfec-shaped PKT windows; replay validation if a capture exists.
- **Hardware:** bench A/B via the mode switch before flight.

## Out of scope / future work

- Adaptive overhead (loss- or probe-driven) — clean follow-up once static
  swfec is flight-proven.
- Per-MCS overhead curve.
- swfec for tun/telemetry links.
- Removing RS video support after swfec is proven (config surface cleanup).
