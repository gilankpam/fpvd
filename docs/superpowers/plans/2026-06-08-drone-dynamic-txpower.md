# Drone dynamic per-MCS TX power — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the drone back off TX power on the high-PAPR (64-QAM) MCS rungs so the PA stays linear and the link can hold MCS5+, while keeping full power at low MCS for range.

**Architecture:** A static per-MCS dBm curve (OpenIPC `wlan_adapters.yaml` bl-m8812eu2, level-4 column, rounded to whole dBm) is looked up by the *operating* MCS in `applyLocalCompute`, stored in the existing `Decision.txPowerDbm`, and applied via the existing (diff-based) `RadioTxpower` inside the controller's per-decision MCS-change block. `link.txpower` becomes a dynamic-link-locked config field. Drone-only; GS unchanged.

**Tech Stack:** C++17, doctest (`drone/build/fpvd_tests`), CMake, nlohmann/json, `iw` (via posix_spawn).

**Spec:** `docs/superpowers/specs/2026-06-08-drone-dynamic-txpower-design.md`

**Build/run (from `drone/`):** configure already done (`build/` exists).
- Build: `cmake --build build -j`
- Run all tests: `./build/fpvd_tests`
- Run filtered: `./build/fpvd_tests -tc="<name or glob>"`

---

## File structure

- **Create** `drone/src/dynlink/txpower_curve.hpp` / `.cpp` — the per-MCS dBm table + `txpowerDbmForMcs(int)`. One responsibility: the curve lookup.
- **Create** `drone/tests/unit/test_dl_txpower_curve.cpp` — curve tests.
- **Modify** `drone/src/dynlink/local_compute.cpp` (+ `.hpp` comment) — set `d.txPowerDbm` from the curve.
- **Modify** `drone/tests/unit/test_dl_local_compute.cpp` — assert txpower is now set.
- **Modify** `drone/src/dynlink/controller.cpp` — apply the power in `dispatchTxApply` / `dispatchTxSafe`.
- **Modify** `drone/src/config/lock.cpp` — lock `link.txpower` when DL enabled; update the stale NOTE.
- **Modify** `drone/tests/unit/test_lock.cpp` — flip the `link.txpower`-allowed test to rejected.
- **Modify** `drone/CMakeLists.txt` — register the new source + test.
- **Unchanged:** `radio_txpower.{hpp,cpp}` and `test_dl_radio_txpower.cpp` (this feature is just the first caller of the existing `apply`).

---

### Task 1: Per-MCS tx-power curve module

**Files:**
- Create: `drone/src/dynlink/txpower_curve.hpp`
- Create: `drone/src/dynlink/txpower_curve.cpp`
- Test: `drone/tests/unit/test_dl_txpower_curve.cpp`
- Modify: `drone/CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `drone/tests/unit/test_dl_txpower_curve.cpp`:

```cpp
/* test_dl_txpower_curve.cpp — per-MCS tx power table (bl-m8812eu2 level 4). */
#include "doctest.h"
#include "dynlink/txpower_curve.hpp"
using namespace fpvd::dynlink;

TEST_CASE("txpowerDbmForMcs returns the bl-m8812eu2 level-4 curve") {
    CHECK(txpowerDbmForMcs(0) == 29);
    CHECK(txpowerDbmForMcs(1) == 28);
    CHECK(txpowerDbmForMcs(2) == 25);
    CHECK(txpowerDbmForMcs(3) == 23);
    CHECK(txpowerDbmForMcs(4) == 19);
    CHECK(txpowerDbmForMcs(5) == 19);
    CHECK(txpowerDbmForMcs(6) == 19);
    CHECK(txpowerDbmForMcs(7) == 19);
}

TEST_CASE("txpowerDbmForMcs clamps out-of-range mcs") {
    CHECK(txpowerDbmForMcs(-1) == 29);  // clamps to MCS0
    CHECK(txpowerDbmForMcs(8)  == 19);  // clamps to MCS7
    CHECK(txpowerDbmForMcs(99) == 19);
}
```

- [ ] **Step 2: Register only the test in CMake**

In `drone/CMakeLists.txt`, add the test to the `fpvd_tests` `target_sources` list right after
`tests/unit/test_dl_local_compute.cpp` (line ~97). Do NOT add the `.cpp` source yet — it
doesn't exist, and CMake errors on a missing source file:

```cmake
        tests/unit/test_dl_local_compute.cpp
        tests/unit/test_dl_txpower_curve.cpp
```

- [ ] **Step 3: Run to verify it fails (missing header)**

Run: `cmake --build build -j`
Expected: FAIL — `fatal error: dynlink/txpower_curve.hpp: No such file or directory`.

- [ ] **Step 4: Write the header**

Create `drone/src/dynlink/txpower_curve.hpp`:

```cpp
#pragma once
#include <cstdint>

namespace fpvd::dynlink {

// Per-MCS TX power (dBm) for the BL-M8812EU2 — the level-4 column of the OpenIPC
// adaptive-link wlan_adapters.yaml table, rounded to whole dBm (ref mBm
// 2900/2750/2500/2250/1900..., i.e. 29/27.5/25/22.5/19...; 27.5->28, 22.5->23).
// Full power at low MCS for range; backed off on the high-PAPR 64-QAM rungs
// (MCS4-7) so the PA stays linear. Indexed by MCS 0..7.
inline constexpr int8_t kTxPowerDbmByMcs[8] = { 29, 28, 25, 23, 19, 19, 19, 19 };

// dBm for the given MCS, clamping mcs to [0,7].
int8_t txpowerDbmForMcs(int mcs);

} // namespace fpvd::dynlink
```

- [ ] **Step 5: Write the implementation**

Create `drone/src/dynlink/txpower_curve.cpp`:

```cpp
/* txpower_curve.cpp — per-MCS TX power lookup (bl-m8812eu2 level 4). */
#include "dynlink/txpower_curve.hpp"

namespace fpvd::dynlink {

int8_t txpowerDbmForMcs(int mcs) {
    if (mcs < 0) mcs = 0;
    if (mcs > 7) mcs = 7;
    return kTxPowerDbmByMcs[mcs];
}

} // namespace fpvd::dynlink
```

- [ ] **Step 6: Register the source in CMake**

In `drone/CMakeLists.txt`, add the source to the `fpvd_core` list right after
`src/dynlink/radio_txpower.cpp` (line ~31):

```cmake
    src/dynlink/radio_txpower.cpp
    src/dynlink/txpower_curve.cpp
```

- [ ] **Step 7: Run the test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="txpowerDbmForMcs*"`
Expected: PASS (2 test cases, all assertions green).

- [ ] **Step 8: Commit**

```bash
git add drone/src/dynlink/txpower_curve.hpp drone/src/dynlink/txpower_curve.cpp \
        drone/tests/unit/test_dl_txpower_curve.cpp drone/CMakeLists.txt
git commit -m "feat(drone/dynlink): per-MCS tx power curve (bl-m8812eu2 level 4)"
```

---

### Task 2: `applyLocalCompute` sets `txPowerDbm` from the curve

**Files:**
- Modify: `drone/src/dynlink/local_compute.cpp`
- Modify: `drone/src/dynlink/local_compute.hpp` (comment only)
- Test: `drone/tests/unit/test_dl_local_compute.cpp`

- [ ] **Step 1: Update the existing test + add a dedicated one (the failing test)**

In `drone/tests/unit/test_dl_local_compute.cpp`, the existing case sets `d.txPowerDbm = 27`
and asserts it is *kept*. Change that assertion (MCS in the test is 5 → curve gives 19):

Replace:
```cpp
    CHECK(d.txPowerDbm == 27);
```
with:
```cpp
    CHECK(d.txPowerDbm == 19);   // now SET from the per-MCS curve (mcs5 -> 19 dBm)
```

Then append a dedicated test at the end of the file (before the final newline):

```cpp
TEST_CASE("applyLocalCompute sets txPowerDbm from the per-MCS curve") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    Decision d{};
    d.bandwidth = 20;

    d.mcs = 0; applyLocalCompute(cfg, d); CHECK(d.txPowerDbm == 29);
    d.mcs = 3; applyLocalCompute(cfg, d); CHECK(d.txPowerDbm == 23);
    d.mcs = 4; applyLocalCompute(cfg, d); CHECK(d.txPowerDbm == 19);
    d.mcs = 7; applyLocalCompute(cfg, d); CHECK(d.txPowerDbm == 19);
}
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="applyLocalCompute*"`
Expected: FAIL — the new case and the changed assertion fail (txpower still 27 / 0; curve not wired).

- [ ] **Step 3: Implement — set txPowerDbm in `applyLocalCompute`**

In `drone/src/dynlink/local_compute.cpp`, add the include near the top (after the existing includes):

```cpp
#include "dynlink/txpower_curve.hpp"
```

Then, inside `applyLocalCompute`, after the line `d.fps = sat8(b.fps);`, add:

```cpp
    d.txPowerDbm = txpowerDbmForMcs(d.mcs);   // per-MCS PA-linearity backoff curve
```

- [ ] **Step 4: Update the stale header comment**

In `drone/src/dynlink/local_compute.hpp`, the comment above `applyLocalCompute` says it leaves
`d.txPowerDbm` untouched. Replace:

```cpp
// Overwrites d.bitrateKbps / d.k / d.n / d.depth / d.fps in place from the
// drone-local engine. Leaves d.mcs, d.bandwidth, d.txPowerDbm untouched.
```
with:
```cpp
// Overwrites d.bitrateKbps / d.k / d.n / d.depth / d.fps / d.txPowerDbm in place
// from the drone-local engine (txPowerDbm via the per-MCS curve). Leaves d.mcs,
// d.bandwidth untouched.
```

- [ ] **Step 5: Run the test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="applyLocalCompute*"`
Expected: PASS.

- [ ] **Step 6: Run the full suite (no regressions)**

Run: `./build/fpvd_tests`
Expected: PASS — all cases green (pristine output).

- [ ] **Step 7: Commit**

```bash
git add drone/src/dynlink/local_compute.cpp drone/src/dynlink/local_compute.hpp \
        drone/tests/unit/test_dl_local_compute.cpp
git commit -m "feat(drone/dynlink): applyLocalCompute sets per-MCS tx power"
```

---

### Task 3: Apply the power in the controller dispatch

**Files:**
- Modify: `drone/src/dynlink/controller.cpp`

No unit test: `radio_` runs `iw` via posix_spawn and isn't injectable, so there's no clean
unit seam — verification is a clean build plus on-hardware (covered in the spec's manual check).
`RadioTxpower::apply` itself is already unit-tested.

- [ ] **Step 1: Add the include**

In `drone/src/dynlink/controller.cpp`, add near the other `dynlink/` includes:

```cpp
#include "dynlink/txpower_curve.hpp"
```

- [ ] **Step 2: Apply per-decision power in `dispatchTxApply` (inside the MCS-change block)**

Find, in `dispatchTxApply`, the end of the probe-retune block:

```cpp
                lastProbeMcs_ = rung;
            }
        }
    }
    lastTx_ = d;
```

Replace it with (adds the diff-based power apply inside the MCS-change `if`):

```cpp
                lastProbeMcs_ = rung;
            }
        }
        // Per-MCS tx power (operating-rung coupling): back off on the high-PAPR
        // 64-QAM rungs to keep the PA linear, full power at low MCS for range.
        // RadioTxpower::apply is diff-based, so iw only runs when the value changes.
        if (radio_) radio_->apply(d.txPowerDbm);
    }
    lastTx_ = d;
```

- [ ] **Step 3: Apply safe power in `dispatchTxSafe`**

Find the end of `dispatchTxSafe` (the probe block then the function's closing brace):

```cpp
        probeWfb_->setRadio(static_cast<uint8_t>(cfg.stbc ? 1 : 0), cfg.ldpc, false,
                            cfg.safe.bandwidth, static_cast<uint8_t>(rung), false, 1);
        lastProbeMcs_ = rung;
    }
}
```

Replace it with:

```cpp
        probeWfb_->setRadio(static_cast<uint8_t>(cfg.stbc ? 1 : 0), cfg.ldpc, false,
                            cfg.safe.bandwidth, static_cast<uint8_t>(rung), false, 1);
        lastProbeMcs_ = rung;
    }
    // Safe recovery: drive power for the (low) safe rung unconditionally, matching
    // the other safe sub-commands. Low MCS -> high power -> robust recovery.
    if (radio_) radio_->applySafe(txpowerDbmForMcs(cfg.safe.mcs));
}
```

- [ ] **Step 4: Build and run the full suite**

Run: `cmake --build build -j && ./build/fpvd_tests`
Expected: PASS — clean build, all tests green (this task adds no test but must not break any).

- [ ] **Step 5: Commit**

```bash
git add drone/src/dynlink/controller.cpp
git commit -m "feat(drone/dynlink): apply per-MCS tx power in dispatch (apply + safe)"
```

---

### Task 4: Lock `link.txpower` while dynamicLink is enabled

**Files:**
- Modify: `drone/src/config/lock.cpp`
- Test: `drone/tests/unit/test_lock.cpp`

- [ ] **Step 1: Flip the existing test (the failing test)**

In `drone/tests/unit/test_lock.cpp`, replace the whole existing case that asserts
`link.txpower` is allowed:

```cpp
TEST_CASE("lock: DL on + body writes link.txpower → allowed (operator-owned, not DL-decided)") {
    // The controller stopped deciding txpower in Phase 3a — tx power is constant,
    // applied at radio bring-up / hot-tuned via radio-tune, never written by the
    // decision loop. So an operator may set it while DL is enabled (like stbc/ldpc).
    auto body = nlohmann::json::parse(R"({"link":{"txpower":20}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}
```

with:

```cpp
TEST_CASE("lock: DL on + body writes link.txpower → rejected (curve owns power)") {
    // Since the per-MCS power curve, the controller drives tx power per decision.
    // A manual value would be silently overridden, so it is locked like link.mcs.
    auto body = nlohmann::json::parse(R"({"link":{"txpower":20}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.txpower");
}
```

(Leave the two `DL off + ... link.stbc/ldpc` and `link.stbc/ldpc → allowed` cases as-is —
stbc/ldpc stay unlocked.)

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="lock: DL on + body writes link.txpower*"`
Expected: FAIL — currently allowed, so `CHECK_FALSE(r.ok)` fails.

- [ ] **Step 3: Implement — add the locked path + fix the NOTE**

In `drone/src/config/lock.cpp`, change the `kLockedPaths` initializer and its NOTE. Replace:

```cpp
static const std::vector<std::vector<std::string>> kLockedPaths = {
    {"link", "mcs"},
    {"link", "fec"},
    {"link", "width"},
    // NOTE: link.stbc / link.ldpc / link.txpower are deliberately NOT locked.
    // They are static link parameters, not DL decisions — the GS controller never
    // sends stbc/ldpc (see dl_wire.h: the decision carries only mcs/bandwidth/
    // depth/k/n), and since Phase 3a the in-process controller no longer drives
    // tx power either (it is constant: set at radio bring-up and hot-tuned via
    // radio-tune, never written by the decision loop). stbc/ldpc are preserved on
    // every CMD_SET_RADIO from the config snapshot; txpower is applied directly
    // via iw. So an operator may retune any of them while DL is enabled without
    // the loop ever overriding the choice.
    {"video", "bitrate"},
    {"video", "qpDelta"},
    {"video", "roi"},
};
```

with:

```cpp
static const std::vector<std::vector<std::string>> kLockedPaths = {
    {"link", "mcs"},
    {"link", "fec"},
    {"link", "width"},
    {"link", "txpower"},
    // NOTE: link.stbc / link.ldpc are deliberately NOT locked. They are static
    // link parameters the controller preserves on every CMD_SET_RADIO from the
    // config snapshot — the GS decision never carries them — so an operator may
    // retune them while DL is enabled. link.txpower IS locked: the per-MCS power
    // curve drives tx power per decision, so a manual value would be overridden.
    {"video", "bitrate"},
    {"video", "qpDelta"},
    {"video", "roi"},
};
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="lock:*"`
Expected: PASS — all lock cases green (txpower now rejected, stbc/ldpc still allowed).

- [ ] **Step 5: Run the full suite**

Run: `./build/fpvd_tests`
Expected: PASS — all tests green.

- [ ] **Step 6: Commit**

```bash
git add drone/src/config/lock.cpp drone/tests/unit/test_lock.cpp
git commit -m "feat(drone/config): lock link.txpower while dynamicLink enabled"
```

---

## Done / verification

- [ ] Full suite green: `cmake --build build -j && ./build/fpvd_tests` (no failures, pristine output).
- [ ] Manual/hardware (post-deploy, per spec): confirm `iw dev wlan0 info` txpower tracks the
      operating MCS (e.g. ~29 dBm at MCS0, ~19 dBm at MCS4–5), MCS5 holds at close range, and
      `PATCH /config {"link":{"txpower":N}}` while DL on returns `400 dynamic_link_locked`.
