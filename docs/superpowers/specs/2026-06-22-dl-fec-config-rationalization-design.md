# Dynamic-link FEC config rationalization

**Date:** 2026-06-22
**Status:** Design — approved in brainstorming, pending spec review
**Scope:** drone (`fpvd`) only. No GS change, no `{mcs}` wire change.

## Problem

Two related warts in how the dynamic link (DL) handles FEC config:

1. **swfec FEC params are locked as if they were adaptive.** While `dynamicLink.enabled`, the
   API locks the whole `link.fec` subtree (`lock.cpp` `{"link","fec"}`) on the theory that the
   controller mutates it. That is only true for **rs**, where the controller *derives* `k`/`n`
   per decision (`computeK`/`computeN` track `wireTarget`, which tracks MCS). For **swfec** the
   controller never computes anything — it passes the static config values straight through
   (`local_compute.cpp:25` sets `d.k = cfg.swfecOverheadPct`, `d.n = cfg.swfecDeadlineMs`, and
   `dispatchTxApply` only re-emits `CMD_SET_FEC` when `d.k`/`d.n` *change*, so after the first
   decision it never fires again). swfec's `mode`/`overheadPct`/`deadlineMs` are therefore static
   link parameters of exactly the same species as `stbc`/`ldpc` — which are **deliberately left
   unlocked** (`lock.cpp:11-17`) and retuned live. The lock is just too coarse.

2. **The failsafe `safe` block hardcodes a Decision, violating "derive everything from MCS."**
   The system's organizing principle is *GS decides MCS; the drone derives bitrate, FEC, and
   power from it*. The one exception is `dynamicLink.safe.{mcs,k,n,overheadPct,deadlineMs,bitrateKbps}`
   — a pre-baked rung the watchdog pushes on a decision-loss trip (`dispatchTxSafe`). It is a
   second, redundant source of truth for quantities the drone already knows how to derive.

## Background: DL FEC today

- `local_compute.cpp` `applyLocalCompute(cfg, d)` fills a `Decision` from `d.mcs`/`d.bandwidth`:
  - **rs:** `k = computeK(wireTarget, …, baseRedundancyRatio, …)`, `n = computeN(k, ratio)` — adaptive.
  - **swfec:** `d.k = cfg.swfecOverheadPct`, `d.n = cfg.swfecDeadlineMs` (both from `link.fec.*`),
    `bitrate = computeBitrateKbpsSwfec(wireTarget, overheadPct, …)` — static pass-through.
  - both: `d.txPowerDbm = txpowerDbmForMcs(d.mcs)` (anti-overdrive curve `{29,28,25,23,19,…}`).
- `lock.cpp` rejects any PATCH that writes inside `kLockedPaths` while DL is enabled, with a
  path-prefix walker plus an ancestor check (so a wholesale `link.fec` overwrite still trips).
- `classifyLinkChange` (`diff.cpp:59-64`) routes a `link.fec` PATCH: a **`mode`** change sets
  `fullRestart` (wfb_tx's `-z` is constructor-time → respawn); an **active-mode param** change
  (rs `k`/`n`, swfec `overheadPct`/`deadlineMs`) sets `videoFec` (hot `CMD_SET_FEC`). They are
  mutually exclusive (`!fecMode && …`).
- On a watchdog trip, `dispatchTxSafe` pushes `cfg.safe.*` unconditionally, then the trip handler
  resets `lastTx_ = Decision{}` + `dedup_.reset()` (`controller.cpp:474-478`) so the next fresh
  decision re-emits everything (this is what restores steady-state values on recovery).

swfec overhead/deadline therefore has **three tiers** today: non-DL static (`link.fec.*`, wfb_tx
ctor), DL steady (`link.fec.*` via `cfg.swfec*`), and DL failsafe (`dynamicLink.safe.*`, default
overhead 100 vs steady 50).

## End state

The `safe` block is gone and swfec params are unlocked static config. FEC is derived from an MCS
in **every** DL regime; the only static FEC inputs are `link.fec.*` (swfec overhead/deadline,
mode, rs non-DL geometry) and `dynamicLink.compute.*` (rs/bitrate derivation params).

| Regime | MCS source | rs `k`/`n` | swfec oh/dl | bitrate / txpower |
|---|---|---|---|---|
| DL steady | GS decision | derive (`computeK`/`computeN`) | `link.fec.overheadPct`/`deadlineMs` | derive |
| DL failsafe | pinned **0** | derive (same) | `link.fec.overheadPct`/`deadlineMs` | derive |
| Non-DL (manual) | `link.mcs` | `link.fec.k`/`n` | `link.fec.overheadPct`/`deadlineMs` | `video.bitrate` etc. |

Consequence: `link.fec.overheadPct`/`deadlineMs` become **the** swfec overhead/deadline knob —
live-tunable under DL (Part 1) and applied identically in steady and failsafe (Part 2). One knob,
no separate failsafe copy.

Lock model after the change: lock only what the controller genuinely *derives* — rs `k`/`n`.
Everything else in `link.fec` (`mode`, `overheadPct`, `deadlineMs`) is static, operator-set,
controller-preserved, and unlocked, exactly like `stbc`/`ldpc`.

---

## Part 1 — Unlock static swfec knobs under DL

### 1.1 Lock granularity — `drone/src/config/lock.cpp`

Replace the single `{"link","fec"}` entry in `kLockedPaths` with:

```cpp
{"link", "fec", "k"},
{"link", "fec", "n"},
```

Verified against the existing walker + ancestor logic:

- `link.fec.mode` / `overheadPct` / `deadlineMs` → **allowed** under DL (none is under or an
  ancestor of `…k`/`…n`).
- `link.fec.k` / `link.fec.n` → **rejected** (`isUnderPrefix` match). Correct: derived under DL+rs;
  irrelevant-but-harmlessly-locked under DL+swfec.
- Wholesale `{"link":{"fec":{}}}` or `{"link":{"fec":null}}` → **still rejected**: the walker emits
  the prefix `link.fec`, which is a strict ancestor of `link.fec.k` (`isAncestorOf`). The
  subtree-wipe guard is preserved.
- Mixed `{"mode":"rs","k":8}` → **rejected** (touches `k`).

Update the comment block to state the rule: rs `k`/`n` are derived → locked; `mode`/`overheadPct`/
`deadlineMs` are static link params the controller preserves → unlocked, like `stbc`/`ldpc`.

### 1.2 Apply routing — `drone/src/daemon.cpp`

A `mode` flip already works under DL: it is `fullRestart` → `needsRebuild` → the rebuild branch
(`daemon.cpp:359`) stops the controller, bounces wfb_tx (re-applying `-z`), and restarts the
controller with a fresh `buildDlSnapshot` carrying the new mode. It returns before the `videoFec`
hot path. The lock was the only thing rejecting it; §1.1 fixes that. **No code change needed for
mode flips.**

The only fix is for live `overheadPct`/`deadlineMs` changes under DL, which take the `videoFec`
hot path and would race the controller (the sole FEC writer under DL):

1. Add `|| link.videoFec` to the DL `setConfig` trigger (`daemon.cpp:422`):

   ```cpp
   else if (enabledOld && enabledNew &&
            (subs.dynamicLink || link.videoRadiotap || link.videoFec))
       dl_.setConfig(dynlink::buildDlSnapshot(effective_, radio_.iface));
   ```

   The controller hot-reloads (`cfg = newCfg`, `controller.cpp:342`). Because swfec
   overhead/deadline *are* the `d.k`/`d.n` dispatch trigger, the next decision (~100 ms) recomputes
   them and `dispatchTxApply` re-emits `CMD_SET_FEC` naturally — no extra restate needed (unlike
   `stbc`/`ldpc`, which ride `setRadio` and need the explicit restate at `:349`). Update the stale
   `// mcs/width/fec are locked` comment at `:423-424`.

2. Gate the direct push so the controller owns FEC while DL is on (`daemon.cpp:454`):

   ```cpp
   if (link.videoFec && !enabledNew) {   // DL off: push directly; DL on: controller owns it
   ```

   This preserves the post-merge-lock case where a single PATCH disables DL *and* writes
   `link.fec` (lock evaluates the merged pending config → unlocked → `enabledNew == false` → direct
   push, as today).

### 1.3 Explicitly unchanged (scope fence)

`local_compute.cpp`, `fec.cpp`, `bitrate.cpp`, `runtime_config` (`swfecOverheadPct` stays live),
`dynamicLink.compute.baseRedundancyRatio` (stays rs-only — **not** folded into swfec), the `{mcs}`
wire, and the entire GS. No validation changes (existing `0..255` / `1..255` / enum bounds suffice).

### 1.4 Tests (`drone/tests/unit/test_lock.cpp`)

Existing fec cases (`k` rejected, wholesale rejected, null-leaf rejected) stay green — they assert
only paths that remain locked, so they double as a regression net. Add:

- DL on + `{"link":{"fec":{"mode":"rs"}}}` → allowed.
- DL on + `{"link":{"fec":{"overheadPct":70}}}` → allowed.
- DL on + `{"link":{"fec":{"deadlineMs":40}}}` → allowed.
- DL on + `{"link":{"fec":{"n":10}}}` → rejected (symmetry with the existing `k` case).
- DL on + `{"link":{"fec":{"mode":"rs","k":8}}}` → rejected (mixed patch touching `k`).

---

## Part 2 — Derive failsafe from MCS 0; remove `dynamicLink.safe`

Failsafe stops carrying a hardcoded rung and instead derives a Decision at **MCS 0** through the
same `applyLocalCompute` path, pinning bandwidth to the operating width (never drop bandwidth on a
trip). MCS 0 (BPSK ½, `curve[0]` = 29 dBm max power) is the robust floor; it yields *more* link
margin and bitrate than today's `safe.mcs=1` rung, and the rs redundancy ratio is unchanged
(today's `safe` rs already uses the steady 0.5 ratio). swfec failsafe overhead becomes the steady
`link.fec.overheadPct` (50) rather than the boosted 100 — accepted: MCS 0 + max power covers the
dominant fade-induced trip; if flight logs later show interference-driven trips need more FEC,
restoring a floor is a one-line constant.

Define `kDlFailsafeMcs = 0` in a shared dynlink header (e.g. `dynlink/local_compute.hpp`), used by
both the controller and the daemon probe seed.

### 2.1 `dispatchTxSafe` rework — `drone/src/dynlink/controller.cpp`

Build the failsafe Decision from MCS 0 and emit it with the existing unconditional-emit +
`lastTx_`/`dedup` reset scaffolding (the recovery mechanism is untouched):

```cpp
void DynamicLinkController::dispatchTxSafe(const DlRuntimeConfig& cfg) {
    Decision d{};
    d.mcs       = kDlFailsafeMcs;     // 0
    d.bandwidth = cfg.linkBandwidth;  // operating width — never change bandwidth on a trip
    applyLocalCompute(cfg, d);        // fills bitrate, k, n, fps, txPowerDbm
    // ... emit unconditionally: setFec(d.k, d.n); setRadio(stbc, ldpc, d.bandwidth, d.mcs);
    //     probe retune at probeRungFor(d.mcs, ceiling); radio_->applySafe(d.txPowerDbm);
}
```

- `setFec(cfg.safe.overheadPct/deadlineMs | safe.k/n)` → `setFec(d.k, d.n)` (derived, mode-correct).
- `setRadio(… cfg.safe.mcs)` → `setRadio(… d.mcs)`; probe `probeRungFor(cfg.safe.mcs, …)` →
  `probeRungFor(d.mcs, …)` (= rung 1).
- `radio_->applySafe(txpowerDbmForMcs(cfg.safe.mcs))` → `radio_->applySafe(d.txPowerDbm)`.
- The encoder safe-bitrate push at `controller.cpp:464` (`enc_->applySafe(cfg.safe.bitrateKbps)`)
  uses the derived `d.bitrateKbps`. Implementation choice (plan decides): have `dispatchTxSafe`
  return the Decision, or move the `enc_->applySafe` call inside it.

`applyLocalCompute` is already included and used at `controller.cpp:391`, so no new dependency.

### 2.2 Remove the `safe` config block

- `drone/src/config/schema.hpp`: delete `struct DynamicLinkSafe`, its
  `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT`, and the `DynamicLink::safe` member.
- `drone/src/dynlink/runtime_config.hpp`: delete `struct SafeDefaults` and `DlRuntimeConfig::safe`.
- `drone/src/dynlink/runtime_config.cpp`: delete the `s.safe = SafeDefaults{…}` population (`:26-33`).
- `drone/src/config/validate.cpp`: delete the `dynamicLink.safe.*` checks (`:132-143`).

Boot-safety: `loadEffective` deep-merges the file over code defaults and `.get<Config>()` **ignores**
unknown keys (`store.cpp:63-67` only *warns*). A deployed `config.json` still carrying
`dynamicLink.safe` loads fine (one warning) and is dropped on the next save. No migration step.
`fpvd --dump-config` stops emitting the block, so fresh seeds are clean.

### 2.3 Probe seed — `drone/src/daemon.cpp`

The initial probe rung (before the first decision retunes it) reads `safe.mcs + 1` at `:151` and
`:254`. Replace both with `std::min(kDlFailsafeMcs + 1, kProbeMcsCeiling)` (= rung 1).

### 2.4 Tests

- `drone/tests/integration/test_dl_controller.cpp` — the watchdog-trip test (`:234,:316,:372`):
  drop the `snap.safe = SafeDefaults{…}` setup; assert the trip now pushes MCS-0-derived values
  (mcs 0, `txpowerDbmForMcs(0)`, derived `k`/`n` for rs and `overheadPct`/`deadlineMs` for swfec,
  derived bitrate), still on the operating bandwidth.
- `drone/tests/unit/test_dl_runtime_config.cpp` — remove the `s.safe.*` assertions
  (`:12,:17,:39-49,:89-92,:137-144`).
- `drone/tests/unit/test_validate.cpp` — remove the `dynamicLink.safe.*` cases
  (`:104-125,:147-151,:314-318`).
- Add a `local_compute` case (or assert via the controller test) that MCS 0 derives sane values
  for both rs and swfec at default config.

---

## Cross-cutting

### Docs

- `CLAUDE.md:56`: "on decision loss the drone falls back to `dynamicLink.safe`" → falls back to a
  derived MCS-0 rung. Update the lock description (`link.fec` is no longer wholesale-locked; only
  `link.fec.k`/`n` are).
- `README.md:73,107`: remove `dynamicLink.safe` from the knob list / "per-airframe failsafe
  ceilings" text.

### Deploy / migration

Drone-only deploy (`deploy/drone/deploy.sh`). Existing `config.json` self-heals (§2.2). The redeploy
briefly drops the video source, which may bounce the GS once (known gotcha).

### Risks

- **swfec failsafe overhead 100 → 50.** Small reduction in burst-loss protection during an
  interference-driven trip; covered for fade/range trips by MCS 0 + max power. Accepted per Option 1;
  reversible via a one-line overhead-floor constant if flight logs warrant.
- **Mode flip = full radio rebuild** ≈ brief video drop. Inherent to the constructor-time `-z` flag;
  identical to flipping mode with DL off; operator-initiated.
- **Failsafe behavior changes** (mcs 1→0, bitrate ~2000→~2600, rs block 8/12→2/3). Bench- and
  flight-validate the trip + recovery before relying on it.

## Implementation phasing

Both parts are independent (Part 2 removes `safe`; Part 1 does not touch it). Land **Part 1 first**
— small, low-risk, bench-validatable in isolation (flip `mode` live; retune `overheadPct` live and
confirm a single clean `CMD_SET_FEC`, no flap). Then **Part 2** — bigger, flight-behavior-changing;
bench the watchdog trip→recovery and flight-validate before trusting the new failsafe.

### Validation

- Part 1 bench: with DL on + swfec, `PATCH link.fec.mode` rs↔swfec (expect brief drop + controller
  resumes); `PATCH link.fec.overheadPct` (expect one `CMD_SET_FEC` at the new value within a tick,
  no race/flap); `PATCH link.fec.k` → `400 dynamic_link_locked`.
- Part 2 bench: force a watchdog trip (stop GS decisions), confirm the radio drops to MCS 0 at
  29 dBm with derived FEC on the operating bandwidth, and that a resumed decision restores steady
  values. Confirm a `config.json` with a leftover `dynamicLink.safe` loads with only a warning.
- Full drone suite stays green (`cd drone && ./build/fpvd_tests`).
