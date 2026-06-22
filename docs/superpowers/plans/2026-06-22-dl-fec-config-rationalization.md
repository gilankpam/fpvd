# Dynamic-link FEC config rationalization — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Under the dynamic link (DL), unlock swfec's static FEC knobs (`link.fec.mode`/`overheadPct`/`deadlineMs`) for live tuning, and make the watchdog failsafe derive from MCS 0 so the `dynamicLink.safe` config block can be deleted.

**Architecture:** Two phased parts on the drone daemon only. Part 1 narrows the API lock from the whole `link.fec` subtree to just the controller-derived rs `k`/`n`, and routes live overhead/deadline changes through the controller instead of the racy direct-push path. Part 2 replaces the hardcoded `dynamicLink.safe` Decision with a derive-from-MCS-0 call through the existing `applyLocalCompute`, then removes the now-dead config block. No GS change, no `{mcs}` wire change.

**Tech Stack:** C++17, doctest, nlohmann::json, CMake.

## Global Constraints

- **Drone-only.** No GS (`gs/`) change, no `{mcs}` wire change (`wire.hpp`).
- **Build:** `cmake -S drone -B drone/build -DCMAKE_BUILD_TYPE=Debug && cmake --build drone/build -j`
- **Test:** from the `drone/` directory, `./build/fpvd_tests` — **never `ctest`** (it runs from `build/` and breaks a fixture path). Filter a case with `./build/fpvd_tests --test-case='*pattern*'`.
- **swfec wire bounds:** `overheadPct` 0..255, `deadlineMs` 1..255 (both uint8 on the control wire).
- **The failsafe MCS is `kDlFailsafeMcs = 0`** (introduced in Task 3).
- TDD throughout; commit after each task.

## File Structure

| File | Responsibility | Tasks |
|---|---|---|
| `drone/src/config/lock.cpp` | DL PATCH lock: which paths are controller-owned | 1 |
| `drone/tests/unit/test_lock.cpp` | Lock unit tests | 1, 4 |
| `drone/src/daemon.cpp` | `apply()` routing; probe seed | 2, 4 |
| `drone/src/dynlink/controller.cpp` / `.hpp` | Control loop; `dispatchTxSafe` failsafe | 2, 3 |
| `drone/src/dynlink/local_compute.hpp` | `applyLocalCompute` decl + `kDlFailsafeMcs` | 3 |
| `drone/tests/integration/test_dl_controller.cpp` | Controller behavior incl. watchdog trip | 2, 3, 4 |
| `drone/src/config/schema.hpp` | Config structs | 4 |
| `drone/src/dynlink/runtime_config.hpp` / `.cpp` | `DlRuntimeConfig` snapshot | 4 |
| `drone/src/config/validate.cpp` | Config validation | 4 |
| `drone/tests/unit/test_dl_runtime_config.cpp`, `test_validate.cpp` | Snapshot + validation tests | 4 |
| `CLAUDE.md`, `README.md` | Docs | 5 |

---

## Task 1: Narrow the DL lock to rs `k`/`n` only

**Files:**
- Modify: `drone/src/config/lock.cpp:8-21` (the `kLockedPaths` table + comment)
- Test: `drone/tests/unit/test_lock.cpp`

**Interfaces:**
- Consumes: `checkDynamicLinkLock(const nlohmann::json& patchBody, const Config& mergedPending) -> LockResult{bool ok; std::vector<std::string> lockedPaths;}` (unchanged signature).
- Produces: nothing new; behavior change only.

- [ ] **Step 1: Add the failing/clarifying lock tests**

Append to `drone/tests/unit/test_lock.cpp` (before the final line):

```cpp
TEST_CASE("lock: DL on + body writes link.fec.mode → allowed (transport choice)") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"mode":"rs"}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL on + body writes link.fec.overheadPct → allowed (static swfec knob)") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"overheadPct":70}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL on + body writes link.fec.deadlineMs → allowed (static swfec knob)") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"deadlineMs":40}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL on + body writes link.fec.n → rejected (rs geometry is derived)") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"n":10}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.fec.n");
}

TEST_CASE("lock: DL on + mixed link.fec {mode,k} → rejected (touches k)") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"mode":"rs","k":8}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.fec.k");
}
```

- [ ] **Step 2: Run the new tests to verify they fail**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case='*link.fec.mode*'`
Expected: FAIL — `link.fec.mode` is currently locked, so `r.ok` is false (the assertion `CHECK(r.ok)` fails).

- [ ] **Step 3: Narrow the lock table**

In `drone/src/config/lock.cpp`, replace the single `{"link", "fec"},` entry in `kLockedPaths` with the two derived-geometry leaves, and update the surrounding comment:

```cpp
// Locked subtree: writes anywhere inside count. The strings here are the
// path *prefixes* the body cannot touch when dynamicLink is enabled.
static const std::vector<std::vector<std::string>> kLockedPaths = {
    {"link", "mcs"},
    // Only rs block geometry is DL-derived (computeK/computeN track the MCS), so
    // only k/n are locked. link.fec.mode / overheadPct / deadlineMs are static
    // swfec params the controller preserves — unlocked, like link.stbc/link.ldpc.
    // A wholesale link.fec overwrite still trips via the ancestor check below.
    {"link", "fec", "k"},
    {"link", "fec", "n"},
    {"link", "width"},
    {"link", "txPowerDbm"},
    // NOTE: link.stbc / link.ldpc are deliberately NOT locked. They are static
    // link parameters the controller preserves on every CMD_SET_RADIO from the
    // config snapshot — the GS decision never carries them — so an operator may
    // retune them while DL is enabled. link.txPowerDbm IS locked: the per-MCS power
    // curve drives tx power per decision, so a manual value would be overridden.
    {"video", "bitrate"},
    {"video", "qpDelta"},
    {"video", "roi"},
};
```

- [ ] **Step 4: Run the lock suite to verify all pass**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case='*lock*'`
Expected: PASS — including the existing `link.fec.k → rejected`, `link.fec wholesale → rejected` (size 2), `link.fec {} → rejected` (`link.fec`), and `link.fec null → rejected` (`link.fec`) cases, which still hold because `k`/`n` stay locked and the ancestor check catches subtree wipes.

- [ ] **Step 5: Commit**

```bash
git add drone/src/config/lock.cpp drone/tests/unit/test_lock.cpp
git commit -m "dynlink: unlock link.fec.mode/overheadPct/deadlineMs under DL; lock only rs k/n"
```

---

## Task 2: Route live overhead/deadline through the controller under DL

**Files:**
- Modify: `drone/src/daemon.cpp:422` (DL setConfig trigger) and `drone/src/daemon.cpp:454` (direct-push gate)
- Test: `drone/tests/integration/test_dl_controller.cpp` (controller-propagation guard test)

**Interfaces:**
- Consumes: `DynamicLinkController::setConfig(const DlRuntimeConfig&)`, `dynlink::buildDlSnapshot(const Config&, const std::string& iface)`, `classifyLinkChange(...).videoFec`, `FakeWfbTx::sawFec(int,int)` (test helper).
- Produces: nothing new.

Background: a `link.fec.mode` flip is `fullRestart` and already rebuilds the controller with a fresh snapshot (no change needed). Only live `overheadPct`/`deadlineMs` changes (`videoFec`) need routing: today they hit the direct `WfbControlClient::setFec` at `daemon.cpp:454`, which races the controller (the sole FEC writer under DL).

- [ ] **Step 1: Write a controller-propagation guard test**

This characterizes the mechanism Part 1 relies on: after `setConfig` changes the swfec overhead, the next decision re-emits `CMD_SET_FEC` with the new value (because swfec overhead/deadline ARE the `d.k`/`d.n` dispatch trigger). Add to `drone/tests/integration/test_dl_controller.cpp`:

```cpp
TEST_CASE("setConfig hot-reloads swfec overhead -> next decision re-emits FEC") {
    FakeWfbTx wfb;
    FakeEnc enc;

    Endpoints ep;
    ep.listenAddr = "127.0.0.1";
    ep.listenPort = 45808;                 // distinct fixed test port
    ep.wfbCtlAddr = "127.0.0.1";
    ep.wfbCtlPort = wfb.port;
    ep.encHost    = "127.0.0.1";
    ep.encPort    = static_cast<uint16_t>(enc.port);
    ep.gsTunnelPort = 0;
    ep.osdUpdateIntervalMs = 1000;

    DlRuntimeConfig snap{};
    snap.healthTimeoutMs = 10000;          // long -> no trip during the test
    snap.applyStaggerMs  = 0;
    snap.applySubPaceMs  = 0;
    snap.roiQp           = RoiCurve{6000, 2000, -24, 3};
    snap.iface           = "wlan-test-nonexistent";
    snap.swfec           = true;           // swfec: d.k=overheadPct, d.n=deadlineMs
    snap.swfecOverheadPct = 50;
    snap.swfecDeadlineMs  = 30;

    DynamicLinkController c(ep);
    c.start(snap);

    auto mkDecision = [](uint32_t seq) {
        Decision d{};
        d.magic = kWireMagic; d.version = kWireVersion;
        d.sequence = seq; d.timestampMs = 1;
        d.mcs = 3; d.bandwidth = 20; d.txPowerDbm = 10;
        d.k = 4; d.n = 6; d.bitrateKbps = 4000; d.fps = 60;
        return d;
    };

    // First decision: swfec pushes overhead/deadline as k/n.
    CHECK(waitFor([&] {
        sendDecision(ep.listenPort, mkDecision(1));
        return wfb.sawFec(50, 30);
    }, 1000));

    // Hot-reload a new overhead; the next (distinct-seq) decision must re-emit it.
    DlRuntimeConfig snap2 = snap;
    snap2.swfecOverheadPct = 70;
    c.setConfig(snap2);

    CHECK(waitFor([&] {
        sendDecision(ep.listenPort, mkDecision(2));
        return wfb.sawFec(70, 30);
    }, 1000));

    c.stop();
}
```

- [ ] **Step 2: Run it — verify it passes (guards existing controller behavior)**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case='*re-emits FEC*'`
Expected: PASS — the controller already hot-reloads `cfg = newCfg` and re-dispatches. This locks in the behavior the daemon change below now depends on.

- [ ] **Step 3: Route `videoFec` through the controller under DL**

In `drone/src/daemon.cpp`, change the `enabledOld && enabledNew` branch condition (currently at `:422`) to also fire on a FEC-param change, and fix the stale comment:

```cpp
        else if (enabledOld && enabledNew &&
                 (subs.dynamicLink || link.videoRadiotap || link.videoFec))
            // stbc/ldpc (videoRadiotap) and swfec overhead/deadline (videoFec) are
            // static params the controller preserves; refresh its snapshot so the
            // loop re-emits them on the next decision. mcs/width/rs-k/n stay locked.
            dl_.setConfig(dynlink::buildDlSnapshot(effective_, radio_.iface));
```

- [ ] **Step 4: Gate the direct FEC push to the DL-off case**

In `drone/src/daemon.cpp`, change the `videoFec` hot-apply guard (currently `if (link.videoFec) {` at `:454`) so the controller owns FEC while DL is on:

```cpp
        if (link.videoFec && !enabledNew) {   // DL off: push directly; DL on: controller owns FEC (setConfig above)
```

(The rest of that block — `WfbControlClient cli(...)`, the swfec/rs `setFec`, the error return — is unchanged.)

- [ ] **Step 5: Build and run the full suite**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS (all cases). The daemon routing change is socket-level control-plane code; it is bench-validated in Task 5's deployment notes (PATCH `link.fec.overheadPct` under DL → one clean `CMD_SET_FEC`, no flap).

- [ ] **Step 6: Commit**

```bash
git add drone/src/daemon.cpp drone/tests/integration/test_dl_controller.cpp
git commit -m "dynlink: route live swfec overhead/deadline through controller under DL"
```

---

## Task 3: Derive the failsafe from MCS 0

**Files:**
- Modify: `drone/src/dynlink/local_compute.hpp` (add `kDlFailsafeMcs`)
- Modify: `drone/src/dynlink/controller.hpp:70` (`dispatchTxSafe` return type)
- Modify: `drone/src/dynlink/controller.cpp:206-228` (`dispatchTxSafe` body) and `:463-464` (trip call site)
- Test: `drone/tests/integration/test_dl_controller.cpp` (the two watchdog-trip tests)

**Interfaces:**
- Consumes: `applyLocalCompute(const DlRuntimeConfig& cfg, Decision& d)`, `DlRuntimeConfig::{linkBandwidth, stbc, ldpc, probeMcsCeiling, applySubPaceMs}`, `Decision::{mcs,bandwidth,k,n,bitrateKbps,txPowerDbm}`, `probeRungFor(int,int)`.
- Produces: `constexpr int fpvd::dynlink::kDlFailsafeMcs = 0;`. `Decision DynamicLinkController::dispatchTxSafe(const DlRuntimeConfig&)` (was `void`).

- [ ] **Step 1: Add the `kDlFailsafeMcs` constant**

In `drone/src/dynlink/local_compute.hpp`, add inside `namespace fpvd::dynlink` (after the `applyLocalCompute` declaration):

```cpp
// Failsafe rung. On a watchdog trip the controller derives a Decision at this
// MCS (BPSK-1/2 floor, max tx power per the anti-overdrive curve) through
// applyLocalCompute, instead of a hardcoded config block.
constexpr int kDlFailsafeMcs = 0;
```

- [ ] **Step 2: Rewrite the watchdog-trip test assertions to expect MCS-0 derive**

In `drone/tests/integration/test_dl_controller.cpp`, add the compute include near the top (after the existing includes):

```cpp
#include "dynlink/local_compute.hpp"
```

In the **watchdog-trip test** (the one starting near `:215` with `snap.linkBandwidth = 40`): delete the `snap.safe = SafeDefaults{...};` setup block, and replace the safe-push assertion block (currently asserting `wfb.sawFec(8, 12) && wfb.sawRadio(1, 40) && ...bitrate=2000`) with a derive-mirror so the expected values track the compute math (MCS-0/bw-40 yields k=2, n=3, bitrate≈4257):

```cpp
    // 2) Go silent past healthTimeoutMs -> watchdog trips -> failsafe derives at
    //    MCS 0 (robust floor), bandwidth pinned to the operating width (40).
    //    Mirror applyLocalCompute so the assertion tracks the math, not magic numbers.
    Decision sf{};
    sf.mcs = fpvd::dynlink::kDlFailsafeMcs;   // 0
    sf.bandwidth = snap.linkBandwidth;        // 40 — never dropped on a trip
    fpvd::dynlink::applyLocalCompute(snap, sf);   // fills k, n, bitrateKbps, txPowerDbm
    CHECK(waitFor([&] {
        return wfb.sawFec(sf.k, sf.n) &&
               wfb.sawRadio(0, snap.linkBandwidth) &&
               enc.sawContaining("video0.bitrate=" + std::to_string(sf.bitrateKbps));
    }, 2000));
```

(Keep the surrounding assertions: `c.status().watchdogTripped == true`, `wfb.allRadioFlags(1, true)`, `wfb.unknownCmds.load() == 0`.)

In the **`setConfig hot-reloads knobs without restart` test** (`:358`): delete the `snap.safe = SafeDefaults{...};` setup and the `snap2.safe.mcs = 5;` reload line. The reload proof is now solely the watchdog-timeout change (10000→400). Replace the safe-push assertion (`wfb.sawFec(8, 12) && wfb.sawRadio(5, 20)`) with the derive mirror (MCS-0/bw-20, default `snap.linkBandwidth==20`):

```cpp
    // After the 400 ms reload trips the watchdog, the failsafe derives at MCS 0
    // on the operating bandwidth (default 20). The trip happening within 2 s (vs
    // the original 10000 ms timeout) is itself the proof the reload took effect.
    Decision sf{};
    sf.mcs = fpvd::dynlink::kDlFailsafeMcs;   // 0
    sf.bandwidth = snap.linkBandwidth;        // 20
    fpvd::dynlink::applyLocalCompute(snap, sf);
    CHECK(waitFor([&] {
        return wfb.sawFec(sf.k, sf.n) && wfb.sawRadio(0, snap.linkBandwidth);
    }, 2000));
```

- [ ] **Step 3: Run the trip tests — verify they fail**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case='*watchdog*,*hot-reload*'`
Expected: compiles (the constant now exists), but FAIL at runtime — `dispatchTxSafe` still pushes the hardcoded `safe` values (mcs 1/5, fec 8/12), not the MCS-0 derive (k=2, n=3, mcs 0).

- [ ] **Step 4: Rework `dispatchTxSafe` to derive, returning the Decision**

In `drone/src/dynlink/controller.hpp:70`, change the declaration:

```cpp
    Decision dispatchTxSafe(const DlRuntimeConfig& cfg);
```

In `drone/src/dynlink/controller.cpp`, replace the whole `dispatchTxSafe` body (`:206-228`) with:

```cpp
Decision DynamicLinkController::dispatchTxSafe(const DlRuntimeConfig& cfg) {
    useconds_t paceUs = static_cast<useconds_t>(cfg.applySubPaceMs) * 1000u;
    // Failsafe derives at the robust MCS-0 floor through the same compute path as
    // a normal decision (GS-decides-MCS, drone-derives-the-rest). Bandwidth is
    // pinned to the operating width — never drop bandwidth on a watchdog trip.
    Decision d{};
    d.mcs       = kDlFailsafeMcs;
    d.bandwidth = cfg.linkBandwidth;
    applyLocalCompute(cfg, d);   // fills k, n, bitrateKbps, fps, txPowerDbm
    wfb_->setFec(d.k, d.n);
    if (paceUs > 0) usleep(paceUs);
    // Preserve the configured stbc/ldpc (robustness coding helps during recovery).
    wfb_->setRadio(/*stbc=*/static_cast<uint8_t>(cfg.stbc ? 1 : 0),
                   /*ldpc=*/cfg.ldpc, /*shortGi=*/false,
                   /*bandwidth=*/d.bandwidth, /*mcs=*/d.mcs,
                   /*vhtMode=*/false, /*vhtNss=*/1);
    // Move the probe down with the video so it never sits above the safe rung.
    if (probeWfb_) {
        int rung = probeRungFor(d.mcs, cfg.probeMcsCeiling);
        probeWfb_->setRadio(static_cast<uint8_t>(cfg.stbc ? 1 : 0), cfg.ldpc, false,
                            d.bandwidth, static_cast<uint8_t>(rung), false, 1);
        lastProbeMcs_ = rung;
    }
    // Low MCS -> high power -> robust recovery (txPowerDbm == curve[0] from derive).
    if (radio_) radio_->applySafe(d.txPowerDbm);
    return d;
}
```

In the watchdog-trip handler (`controller.cpp:463-464`), capture the derived Decision for the encoder safe-bitrate push:

```cpp
                Decision sf = dispatchTxSafe(cfg);
                enc_->applySafe(sf.bitrateKbps);
```

- [ ] **Step 5: Run the trip tests — verify they pass**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case='*watchdog*,*hot-reload*'`
Expected: PASS — both trips now push the MCS-0-derived FEC/radio/bitrate.

- [ ] **Step 6: Run the full suite**

Run: `cd drone && ./build/fpvd_tests`
Expected: PASS. (`dispatchTxSafe` no longer reads `cfg.safe`; the `SafeDefaults` struct still exists and is removed in Task 4.)

- [ ] **Step 7: Commit**

```bash
git add drone/src/dynlink/local_compute.hpp drone/src/dynlink/controller.hpp \
        drone/src/dynlink/controller.cpp drone/tests/integration/test_dl_controller.cpp
git commit -m "dynlink: derive watchdog failsafe from MCS 0 (kDlFailsafeMcs) instead of safe config"
```

---

## Task 4: Remove the `dynamicLink.safe` config block

**Files:**
- Modify: `drone/src/config/schema.hpp` (struct + macro + member)
- Modify: `drone/src/dynlink/runtime_config.hpp:10-17,39` and `runtime_config.cpp:26-33`
- Modify: `drone/src/config/validate.cpp:132-143`
- Modify: `drone/src/daemon.cpp:151,254` (probe seed) + include
- Modify: `drone/tests/unit/test_dl_runtime_config.cpp`, `test_validate.cpp`, `test_lock.cpp`, `test_dl_controller.cpp`

**Interfaces:**
- Consumes: `kDlFailsafeMcs` (Task 3), `kProbeMcsCeiling`.
- Produces: `DynamicLink` and `DlRuntimeConfig` no longer have a `safe` member; `SafeDefaults`/`DynamicLinkSafe` deleted.

- [ ] **Step 1: Strip `safe` from the tests first (red is the missing field, then green after removal)**

`drone/tests/unit/test_dl_runtime_config.cpp`:
- In `"buildDlSnapshot maps schema + derived inputs"`: delete line `c.dynamicLink.safe.mcs = 3;` (keep `c.dynamicLink.healthTimeoutMs = 8000;`) and delete `CHECK(s.safe.mcs == 3);`.
- Delete the entire `TEST_CASE("buildDlSnapshot maps safe defaults correctly")` (the `c.dynamicLink.safe.{mcs,k,n,bitrateKbps}` / `s.safe.*` case).
- In `"buildDlSnapshot default Config produces correct defaults"`: delete the `// Safe defaults` block (the four `CHECK(s.safe.* == ...)` lines).
- In `"buildDlSnapshot: swfec fields from link.fec + safe"`: delete `c.dynamicLink.safe.overheadPct = 120;`, `c.dynamicLink.safe.deadlineMs = 35;`, `CHECK(s.safe.overheadPct == 120);`, `CHECK(s.safe.deadlineMs == 35);`; rename the title to `"buildDlSnapshot: swfec fields from link.fec"`.

`drone/tests/unit/test_validate.cpp`:
- Delete `TEST_CASE("validate: dynamicLink.safe.mcs in [0,7]")`.
- Delete `TEST_CASE("validate: dynamicLink.safe k<n and both in [1,32]")`.
- Delete `TEST_CASE("validate: dynamicLink.safe.bitrateKbps > 0")`.
- In `TEST_CASE("validate: link.fec swfec rules")`, delete the `SUBCASE("safe swfec ranges")` block (the two `dynamicLink.safe.*` assertions).

`drone/tests/unit/test_lock.cpp`: replace the now-defunct `dynamicLink.safe.mcs` allowed-path case with an unlocked-`dynamicLink`-field example that still exists:

```cpp
TEST_CASE("lock: DL on + body writes dynamicLink.compute.baseRedundancyRatio → allowed") {
    auto body = nlohmann::json::parse(
        R"({"dynamicLink":{"compute":{"baseRedundancyRatio":0.6}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
}
```

`drone/tests/integration/test_dl_controller.cpp`: delete the remaining `snap.safe = SafeDefaults{1, 8, 12, 2000};` line in the staggered-dispatch test (the one that does NOT trip the watchdog).

- [ ] **Step 2: Remove the schema struct and member**

In `drone/src/config/schema.hpp`:
- Delete `struct DynamicLinkSafe { ... };` and its `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkSafe, ...)`.
- In `struct DynamicLink`, delete `DynamicLinkSafe   safe{};`.
- In the `DynamicLink` macro, remove `safe`: change `roiQp, safe, compute)` to `roiQp, compute)`.

- [ ] **Step 3: Remove the snapshot struct and population**

In `drone/src/dynlink/runtime_config.hpp`: delete `struct SafeDefaults { ... };` (lines 10-17) and the `SafeDefaults safe;` member in `DlRuntimeConfig`.

In `drone/src/dynlink/runtime_config.cpp`: delete the `s.safe = SafeDefaults{ ... };` assignment and the two follow-on lines `s.safe.overheadPct = ...;` / `s.safe.deadlineMs = ...;`.

- [ ] **Step 4: Remove the validation block**

In `drone/src/config/validate.cpp`, delete the six `dynamicLink.safe.*` checks (the `dl.safe.mcs`, `dl.safe.{k,n}`, `dl.safe.bitrateKbps`, `dl.safe.overheadPct`, `dl.safe.deadlineMs` error pushes). Keep `dl.healthTimeoutMs` and everything after.

- [ ] **Step 5: Fix the probe seed to use `kDlFailsafeMcs`**

In `drone/src/daemon.cpp`, add the include near the other dynlink/probe includes:

```cpp
#include "dynlink/local_compute.hpp"
```

Replace both probe-seed sites (currently `effective_.dynamicLink.safe.mcs + 1` at `:151` and `:254`):

```cpp
        const int probeMcs = std::min(kDlFailsafeMcs + 1, kProbeMcsCeiling);
```

- [ ] **Step 6: Build and run the full suite**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS, clean compile with no references to `safe`/`SafeDefaults`/`DynamicLinkSafe` remaining.

Verify nothing references the removed symbols:

```bash
rg -n "\.safe\b|SafeDefaults|DynamicLinkSafe|dynamicLink\.safe" drone/src drone/tests
```
Expected: no matches.

- [ ] **Step 7: Commit**

```bash
git add drone/src/config/schema.hpp drone/src/dynlink/runtime_config.hpp \
        drone/src/dynlink/runtime_config.cpp drone/src/config/validate.cpp \
        drone/src/daemon.cpp drone/tests/unit/test_dl_runtime_config.cpp \
        drone/tests/unit/test_validate.cpp drone/tests/unit/test_lock.cpp \
        drone/tests/integration/test_dl_controller.cpp
git commit -m "dynlink: remove dynamicLink.safe block (failsafe now derives from MCS 0)"
```

---

## Task 5: Docs + final verification

**Files:**
- Modify: `CLAUDE.md:56`, `README.md` (the `dynamicLink.safe` references)

**Interfaces:** none.

- [ ] **Step 1: Update CLAUDE.md**

In `CLAUDE.md`, in the dynamic-link section:
- Change "on decision loss the drone falls back to `dynamicLink.safe`" to "on decision loss the drone falls back to a derived MCS-0 failsafe rung".
- In the lock description ("the drone API **locks** the fields the controller mutates (`link.mcs`, `link.txpower`, `link.fec`, `link.width`, …)"), change `link.fec` to `link.fec.k`/`link.fec.n` (only rs geometry is locked; `link.fec.mode`/`overheadPct`/`deadlineMs` are operator-tunable under DL).

- [ ] **Step 2: Update README.md**

In `README.md`, remove `dynamicLink.safe` from the knob list (the line listing `dynamicLink.safe`, `dynamicLink.roiQp`, timeouts, …) and delete/adjust the "Per-airframe failsafe ceilings live under `dynamicLink.safe`" sentence to state the failsafe now derives from MCS 0 (no config).

- [ ] **Step 3: Final full-suite verification**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS — entire drone suite green.

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md README.md
git commit -m "docs: DL FEC unlock + MCS-0 failsafe (drop dynamicLink.safe)"
```

---

## Bench validation (post-merge, live hardware)

Deploy drone-only: `./deploy/drone/deploy.sh` (a redeploy briefly drops the video source and may bounce the GS once). Then, with DL enabled:

- **Part 1 — mode flip:** `PATCH /config {"link":{"fec":{"mode":"rs"}}}` then `POST /apply` → expect a brief video drop (full radio rebuild) and the controller resuming on the new mode; flip back to `swfec` likewise.
- **Part 1 — live overhead retune (swfec):** `PATCH /config {"link":{"fec":{"overheadPct":80}}}` + `POST /apply` → expect a single `CMD_SET_FEC` at the new overhead within ~one tick, no flapping. `PATCH {"link":{"fec":{"k":4}}}` → expect `400 dynamic_link_locked`.
- **Part 2 — failsafe:** stop GS decisions (or block the uplink) to force a watchdog trip → confirm the radio drops to MCS 0 at 29 dBm with derived FEC on the operating bandwidth, and that a resumed decision restores steady values. Confirm a `config.json` still carrying a `dynamicLink.safe` block loads with only a warning (`fpvd: warning: unknown config key 'dynamicLink.safe...'`).

---

## Self-Review

**Spec coverage** (against `2026-06-22-dl-fec-config-rationalization-design.md`):
- Part 1 lock granularity → Task 1. ✓
- Part 1 apply routing (setConfig trigger + `!enabledNew` gate) → Task 2. ✓
- Part 1 "explicitly unchanged" (local_compute/baseRedundancyRatio/wire/GS) → respected; no task touches them. ✓
- Part 2 `dispatchTxSafe` MCS-0 derive + `kDlFailsafeMcs` → Task 3. ✓
- Part 2 remove `safe` (schema/runtime_config/validate) + probe seed → Task 4. ✓
- Cross-cutting docs (CLAUDE.md/README) → Task 5. ✓
- Tests (lock cases; trip-test rewrite; runtime_config/validate strip) → Tasks 1,3,4. ✓
- Deploy/risk notes → Bench validation section. ✓

**Placeholder scan:** No TBD/TODO; every code step shows complete code or exact deletion targets. The daemon routing change (Task 2) is honestly marked bench-validated rather than unit-tested, with a concrete bench procedure.

**Type consistency:** `kDlFailsafeMcs` (defined Task 3) is consumed in Tasks 3 and 4. `dispatchTxSafe` return type changes `void`→`Decision` consistently in `controller.hpp` and `controller.cpp` and is used at the trip site. `applyLocalCompute(cfg, d)` signature matches its use in the test mirror and `dispatchTxSafe`. `checkDynamicLinkLock` / `LockResult` unchanged.
