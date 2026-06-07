# Phase 3a — Drone-Local Compute Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Make the drone compute its own video `bitrate`/`k`/`n` from the commanded `{mcs, bandwidth}` via the OpenIPC WFB calculator (replacing the GS airtime model + dynamic-FEC escalator), with `depth` and `tx_power` becoming static constants — all drone-only, wire unchanged.

**Architecture:** Two pure C++ units (`bitrate` = OpenIPC table + formula; `fec` = block-fill `compute_k`/`n`) plus a pure composer (`local_compute` = `applyLocalCompute(cfg, Decision&)`) that overwrites the GS-sent `bitrate/k/n/depth/fps` on each decoded Decision. The controller calls `applyLocalCompute` right after decode (one line) and stops modulating tx power (drops the dynamic `iw` apply). New bitrate-engine knobs live under `dynamicLink`; `mtu`/`fps` come from `link`/`video`; depth is a code constant.

**Tech Stack:** C++17, doctest (host-only unit tests), nlohmann/json (config), CMake.

**Spec:** `docs/superpowers/specs/2026-06-07-phase3a-drone-local-compute-design.md`.

**Build & test (host, from `drone/`):**
```
cd /home/gilankpam/Projects/drone/fpvd/drone
cmake -B build            # once, if build/ does not exist
cmake --build build -j && ./build/fpvd_tests
```
Filter a single suite: `./build/fpvd_tests --test-case="*bitrate*"`. **NOT `ctest`** (test_daemon copies a fixture via a path relative to `drone/`; ctest runs from `build/` and false-fails). All git commands run from the repo root `/home/gilankpam/Projects/drone/fpvd`.

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `drone/src/dynlink/bitrate.hpp` / `.cpp` | create | OpenIPC base-rate table + `computeWireTargetKbps` (incl. probe_util) + `computeBitrateKbps` |
| `drone/src/dynlink/fec.hpp` / `.cpp` | create | block-fill `computeK` + `computeN` (port of GS `dynamic_fec.compute_k/compute_n`) |
| `drone/src/dynlink/local_compute.hpp` / `.cpp` | create | `kInterleaveDepth` + `applyLocalCompute(const DlRuntimeConfig&, Decision&)` composer |
| `drone/src/config/schema.hpp` | modify | add `DynamicLinkBitrate` + `DynamicLinkFec`; add to `DynamicLink` |
| `drone/src/dynlink/runtime_config.hpp` | modify | add `BitrateEngineConfig` + a `bitrate` field on `DlRuntimeConfig` |
| `drone/src/dynlink/runtime_config.cpp` | modify | map the new config into `buildDlSnapshot` |
| `drone/src/dynlink/controller.cpp` | modify | call `applyLocalCompute(cfg, d)` after decode; drop the dynamic `radio_->apply`/`applySafe` |
| `drone/CMakeLists.txt` | modify | add the new `.cpp` to `fpvd_core` + the new tests to `fpvd_tests` |
| `drone/tests/unit/test_dl_bitrate.cpp` | create | unit-test the bitrate unit |
| `drone/tests/unit/test_dl_fec.cpp` | create | unit-test the fec unit |
| `drone/tests/unit/test_dl_local_compute.cpp` | create | unit-test the composer |
| `drone/tests/unit/test_dl_runtime_config.cpp` | modify | assert the new knobs map through `buildDlSnapshot` |

Execute in order: 1 (bitrate) → 2 (fec) → 3 (config) → 4 (composer) → 5 (controller wire-in + tx_power) → 6 (hardware smoke). Tasks 1–5 are local code + tests (commit each). Task 6 is on-hardware.

---

## Task 1: OpenIPC bitrate table + formula

**Files:** Create `drone/src/dynlink/bitrate.hpp`, `drone/src/dynlink/bitrate.cpp`, `drone/tests/unit/test_dl_bitrate.cpp`; Modify `drone/CMakeLists.txt`.

- [ ] **Step 1: Write the header** — `drone/src/dynlink/bitrate.hpp`:

```cpp
/* bitrate.hpp — OpenIPC WFB-calculator effective-rate table + the
 * deterministic video-bitrate formula (Phase 3a). Pure functions. */
#pragma once
#include <cstdint>

namespace fpvd::dynlink {

// OpenIPC effective-rate table (kbps, long GI), MCS 0-7, already de-rated to
// real WFB throughput. bandwidthMhz is 20 or 40 (the radiotap value). Returns
// 0 for an unknown bandwidth or out-of-range mcs.
uint32_t openIpcBaseRateKbps(int bandwidthMhz, int mcs);

// wire_target_kbps = baseRate[bw][mcs] * (2/3 - probe_util), where
// probe_util = probeKbps / baseRate[bw][min(mcs+1, probeCeiling)].
// Clamps (2/3 - probe_util) to >= 0. Returns 0 if baseRate[bw][mcs] is 0.
double computeWireTargetKbps(int bandwidthMhz, int mcs, int probeCeiling,
                             double probeKbps);

// bitrate = trunc(wire_target * k / n), clamped to [minKbps, maxKbps].
// Truncation (not round) keeps the on-air wire rate <= wire_target.
uint16_t computeBitrateKbps(double wireTargetKbps, int k, int n,
                            int minKbps, int maxKbps);

} // namespace fpvd::dynlink
```

- [ ] **Step 2: Write the failing test** — `drone/tests/unit/test_dl_bitrate.cpp`:

```cpp
/* test_dl_bitrate.cpp — OpenIPC table + bitrate formula (Phase 3a). */
#include "doctest.h"
#include "dynlink/bitrate.hpp"
using namespace fpvd::dynlink;

TEST_CASE("openIpc base rate table endpoints") {
    CHECK(openIpcBaseRateKbps(20, 0) == 6500u);
    CHECK(openIpcBaseRateKbps(20, 7) == 55000u);
    CHECK(openIpcBaseRateKbps(40, 0) == 9800u);
    CHECK(openIpcBaseRateKbps(40, 7) == 97000u);
    CHECK(openIpcBaseRateKbps(20, 5) == 42000u);
    // unknown bandwidth / out-of-range mcs -> 0
    CHECK(openIpcBaseRateKbps(10, 0) == 0u);
    CHECK(openIpcBaseRateKbps(20, 8) == 0u);
    CHECK(openIpcBaseRateKbps(20, -1) == 0u);
}

TEST_CASE("wire target with no probe reserve matches OpenIPC 2/3") {
    // MCS0/20, probeKbps=0 -> 6500 * 2/3 = 4333.33 (parent design worked example)
    CHECK(computeWireTargetKbps(20, 0, 7, 0.0) == doctest::Approx(4333.333).epsilon(0.001));
}

TEST_CASE("wire target subtracts probe airtime at the probe rung") {
    // MCS0/20, probe at min(0+1,7)=1 -> baseRate=12000; probe_util=280/12000.
    double pu = 280.0 / 12000.0;
    double expect = 6500.0 * (2.0 / 3.0 - pu);
    CHECK(computeWireTargetKbps(20, 0, 7, 280.0) == doctest::Approx(expect).epsilon(0.001));
    CHECK(computeWireTargetKbps(20, 0, 7, 280.0) < computeWireTargetKbps(20, 0, 7, 0.0));
}

TEST_CASE("wire target clamps the utilization floor at zero") {
    // A huge probeKbps would drive (2/3 - probe_util) negative; clamp to 0.
    CHECK(computeWireTargetKbps(20, 0, 7, 1.0e9) == doctest::Approx(0.0));
}

TEST_CASE("bitrate truncates wire_target*k/n and clamps") {
    // 4333.33 * 8/12 = 2888.88 -> trunc 2888 (parent: 6500*4/9 = 2888).
    CHECK(computeBitrateKbps(4333.333, 8, 12, 1000, 24000) == 2888);
    // below the floor clamps up
    CHECK(computeBitrateKbps(500.0, 8, 12, 1000, 24000) == 1000);
    // above the ceiling clamps down
    CHECK(computeBitrateKbps(50000.0, 8, 12, 1000, 24000) == 24000);
}
```

- [ ] **Step 3: Wire the test + source into CMake** — in `drone/CMakeLists.txt`, add `src/dynlink/bitrate.cpp` to the `fpvd_core` `target_sources` list (after `src/dynlink/roi_qp.cpp`, line ~32), and add `tests/unit/test_dl_bitrate.cpp` to the `fpvd_tests` `target_sources` list (after `tests/unit/test_dl_roi_qp.cpp`, line ~91).

- [ ] **Step 4: Run to verify it fails** —
```
cd /home/gilankpam/Projects/drone/fpvd/drone && cmake --build build -j 2>&1 | tail -5
```
Expected: FAIL — link error / `undefined reference to openIpcBaseRateKbps` (header declared, no `.cpp` yet).

- [ ] **Step 5: Implement** — `drone/src/dynlink/bitrate.cpp`:

```cpp
/* bitrate.cpp — OpenIPC effective-rate table + bitrate formula (Phase 3a). */
#include "dynlink/bitrate.hpp"
#include <cmath>

namespace fpvd::dynlink {

uint32_t openIpcBaseRateKbps(int bandwidthMhz, int mcs) {
    // Source: OpenIPC WFB calculator (src/components/wfb-calculator.astro),
    // long-GI rows. 40/short/MCS6 upstream typo (980000) is irrelevant — GI is
    // long. Index = MCS 0-7.
    static const uint32_t kRate20[8] =
        {6500, 12000, 15500, 20000, 25000, 42000, 47500, 55000};
    static const uint32_t kRate40[8] =
        {9800, 18600, 30400, 40200, 55800, 80400, 90200, 97000};
    if (mcs < 0 || mcs > 7) return 0;
    if (bandwidthMhz == 20) return kRate20[mcs];
    if (bandwidthMhz == 40) return kRate40[mcs];
    return 0;
}

double computeWireTargetKbps(int bandwidthMhz, int mcs, int probeCeiling,
                             double probeKbps) {
    uint32_t base = openIpcBaseRateKbps(bandwidthMhz, mcs);
    if (base == 0) return 0.0;
    int probeRung = mcs + 1;
    if (probeRung > probeCeiling) probeRung = probeCeiling;
    uint32_t probeBase = openIpcBaseRateKbps(bandwidthMhz, probeRung);
    double probeUtil = (probeBase > 0) ? (probeKbps / static_cast<double>(probeBase)) : 0.0;
    double util = (2.0 / 3.0) - probeUtil;
    if (util < 0.0) util = 0.0;
    return static_cast<double>(base) * util;
}

uint16_t computeBitrateKbps(double wireTargetKbps, int k, int n,
                            int minKbps, int maxKbps) {
    if (k <= 0 || n <= 0)
        return static_cast<uint16_t>(minKbps);
    double raw = wireTargetKbps * static_cast<double>(k) / static_cast<double>(n);
    long v = static_cast<long>(raw);          // truncate toward zero (wire rounds DOWN)
    if (v < minKbps) v = minKbps;
    if (v > maxKbps) v = maxKbps;
    return static_cast<uint16_t>(v);
}

} // namespace fpvd::dynlink
```

- [ ] **Step 6: Run to verify it passes** —
```
cmake --build build -j && ./build/fpvd_tests --test-case="*base rate*,*wire target*,*bitrate truncates*"
```
Expected: PASS. Then the full suite `./build/fpvd_tests 2>&1 | tail -3` — all green.

- [ ] **Step 7: Commit**
```bash
git add drone/src/dynlink/bitrate.hpp drone/src/dynlink/bitrate.cpp drone/tests/unit/test_dl_bitrate.cpp drone/CMakeLists.txt
git commit -m "feat(drone/dynlink): OpenIPC bitrate table + formula (Phase 3a)"
```

---

## Task 2: FEC — block-fill compute_k + compute_n

**Files:** Create `drone/src/dynlink/fec.hpp`, `drone/src/dynlink/fec.cpp`, `drone/tests/unit/test_dl_fec.cpp`; Modify `drone/CMakeLists.txt`.

Port the GS `dynamic_fec.compute_k`/`compute_n` (block-fill enforcement, anchored on the encoder bitrate at `n_base`), with a **fixed** redundancy ratio (no escalation).

- [ ] **Step 1: Write the header** — `drone/src/dynlink/fec.hpp`:

```cpp
/* fec.hpp — latency-sized k (block-fill) + fixed-ratio n (Phase 3a).
 * Port of gs fpvdgs/dynlink/dynamic_fec.compute_k / compute_n. Pure. */
#pragma once

namespace fpvd::dynlink {

// Block size, sized so block_fill stays inside one frame period. Anchored on
// the encoder bitrate at n_base (= wireTarget / (1 + baseRedundancyRatio)).
// Returns kMin for any non-positive input. Result clamped to [kMin, kMax].
int computeK(double wireTargetKbps, int mtuBytes, int fps,
             double baseRedundancyRatio, double blocksPerFrame,
             int kMin, int kMax);

// n = ceil(k * (1 + baseRedundancyRatio)). Fixed ratio; no escalation.
int computeN(int k, double baseRedundancyRatio);

} // namespace fpvd::dynlink
```

- [ ] **Step 2: Write the failing test** — `drone/tests/unit/test_dl_fec.cpp`:

```cpp
/* test_dl_fec.cpp — block-fill compute_k + fixed-ratio compute_n (Phase 3a). */
#include "doctest.h"
#include "dynlink/fec.hpp"
using namespace fpvd::dynlink;

TEST_CASE("computeK sizes for block-fill at a typical wire target") {
    // wireTarget=28000 (MCS5/20 * 2/3), mtu=1500, fps=60, ratio=0.5, bpf=2.0
    // anchor = 28000/1.5 = 18666.67; ppf = 18666.67*1000/(60*1500*8) = 25.93;
    // k = trunc(25.93/2.0) = 12.
    CHECK(computeK(28000.0, 1500, 60, 0.5, 2.0, 2, 50) == 12);
}

TEST_CASE("computeK clamps to [kMin, kMax]") {
    CHECK(computeK(100.0, 1500, 60, 0.5, 2.0, 2, 50) == 2);       // tiny -> kMin
    CHECK(computeK(5.0e6, 1500, 60, 0.5, 2.0, 2, 50) == 50);      // huge -> kMax
    CHECK(computeK(0.0,  1500, 60, 0.5, 2.0, 2, 50) == 2);        // non-positive -> kMin
    CHECK(computeK(28000.0, 0, 60, 0.5, 2.0, 2, 50) == 2);        // bad mtu -> kMin
}

TEST_CASE("computeN is ceil(k * (1 + ratio))") {
    CHECK(computeN(12, 0.5) == 18);   // ceil(18.0) = 18
    CHECK(computeN(8, 0.5) == 12);    // ceil(12.0) = 12
    CHECK(computeN(7, 0.5) == 11);    // ceil(10.5) = 11
    CHECK(computeN(2, 0.5) == 3);     // ceil(3.0) = 3
}
```

- [ ] **Step 3: Wire into CMake** — add `src/dynlink/fec.cpp` to `fpvd_core` and `tests/unit/test_dl_fec.cpp` to `fpvd_tests` in `drone/CMakeLists.txt` (next to the Task 1 entries).

- [ ] **Step 4: Run to verify it fails** — `cmake --build build -j 2>&1 | tail -5` → FAIL (`undefined reference to computeK`).

- [ ] **Step 5: Implement** — `drone/src/dynlink/fec.cpp`:

```cpp
/* fec.cpp — block-fill compute_k + fixed-ratio compute_n (Phase 3a). */
#include "dynlink/fec.hpp"
#include <cmath>

namespace fpvd::dynlink {

int computeK(double wireTargetKbps, int mtuBytes, int fps,
             double baseRedundancyRatio, double blocksPerFrame,
             int kMin, int kMax) {
    if (wireTargetKbps <= 0.0 || mtuBytes <= 0 || fps <= 0 || blocksPerFrame <= 0.0)
        return kMin;
    double anchorKbps = wireTargetKbps / (1.0 + baseRedundancyRatio);
    double packetsPerFrame =
        anchorKbps * 1000.0 / (static_cast<double>(fps) * mtuBytes * 8.0);
    int k = static_cast<int>(packetsPerFrame / blocksPerFrame);  // trunc, matches GS int()
    if (k < kMin) k = kMin;
    if (k > kMax) k = kMax;
    return k;
}

int computeN(int k, double baseRedundancyRatio) {
    return static_cast<int>(std::ceil(static_cast<double>(k) * (1.0 + baseRedundancyRatio)));
}

} // namespace fpvd::dynlink
```

- [ ] **Step 6: Run to verify it passes** — `cmake --build build -j && ./build/fpvd_tests --test-case="*computeK*,*computeN*"` → PASS; then full suite green.

- [ ] **Step 7: Commit**
```bash
git add drone/src/dynlink/fec.hpp drone/src/dynlink/fec.cpp drone/tests/unit/test_dl_fec.cpp drone/CMakeLists.txt
git commit -m "feat(drone/dynlink): block-fill compute_k + fixed-ratio n (Phase 3a)"
```

---

## Task 3: Config — bitrate-engine knobs

**Files:** Modify `drone/src/config/schema.hpp`, `drone/src/dynlink/runtime_config.hpp`, `drone/src/dynlink/runtime_config.cpp`, `drone/tests/unit/test_dl_runtime_config.cpp`.

Add the bitrate-engine tuning under `dynamicLink` (`bitrate` + `fec`) and surface it on `DlRuntimeConfig` via `buildDlSnapshot`, pulling `mtu`/`fps` from `link`/`video`.

- [ ] **Step 1: Write the failing test** — append to `drone/tests/unit/test_dl_runtime_config.cpp`:

```cpp
TEST_CASE("buildDlSnapshot maps the Phase-3a bitrate-engine knobs") {
    fpvd::Config c{};
    c.link.mtu  = 1400;
    c.video.fps = 90;
    c.dynamicLink.bitrate.minBitrateKbps = 1500;
    c.dynamicLink.bitrate.maxBitrateKbps = 20000;
    c.dynamicLink.fec.baseRedundancyRatio = 0.5;
    c.dynamicLink.fec.blocksPerFrame      = 2.0;
    c.dynamicLink.fec.kMin = 3;
    c.dynamicLink.fec.kMax = 40;

    auto s = fpvd::dynlink::buildDlSnapshot(c, "wlan0");

    CHECK(s.bitrate.mtuBytes == 1400);
    CHECK(s.bitrate.fps == 90);
    CHECK(s.bitrate.minBitrateKbps == 1500);
    CHECK(s.bitrate.maxBitrateKbps == 20000);
    CHECK(s.bitrate.baseRedundancyRatio == doctest::Approx(0.5));
    CHECK(s.bitrate.blocksPerFrame == doctest::Approx(2.0));
    CHECK(s.bitrate.kMin == 3);
    CHECK(s.bitrate.kMax == 40);
}
```
(If `test_dl_runtime_config.cpp` lacks the `#include "config/schema.hpp"` / `dynlink/runtime_config.hpp` includes, add them at the top — match the file's existing includes.)

- [ ] **Step 2: Run to verify it fails** — `cmake --build build -j 2>&1 | tail -5` → FAIL (`no member named 'bitrate' in DynamicLink` / `DlRuntimeConfig`).

- [ ] **Step 3: Implement the schema** — in `drone/src/config/schema.hpp`, add these two structs immediately before `struct DynamicLink` (after `DynamicLinkRoiQp`, ~line 134):

```cpp
struct DynamicLinkBitrate {
    int minBitrateKbps{1000};
    int maxBitrateKbps{24000};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkBitrate,
                                               minBitrateKbps, maxBitrateKbps)

struct DynamicLinkFec {
    double baseRedundancyRatio{0.5};   // n/k = 1 + ratio = 1.5 (= 8/12 data fraction)
    double blocksPerFrame{2.0};
    int    kMin{2};
    int    kMax{50};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkFec,
                                               baseRedundancyRatio,
                                               blocksPerFrame, kMin, kMax)
```
Then add two fields to `struct DynamicLink` (after `DynamicLinkSafe safe{};`, ~line 145):
```cpp
    DynamicLinkBitrate bitrate{};
    DynamicLinkFec     fec{};
```
And extend its `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLink, ...)` macro argument list (~line 147-152) to include `bitrate, fec` at the end:
```cpp
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLink, enabled,
                                               healthTimeoutMs,
                                               interleavingSupported,
                                               minIdrIntervalMs, applyStaggerMs,
                                               applySubPaceMs,
                                               osd, roiQp, safe, bitrate, fec)
```

- [ ] **Step 4: Implement the runtime struct** — in `drone/src/dynlink/runtime_config.hpp`, add this struct before `struct DlRuntimeConfig` (after `struct SafeDefaults`, ~line 18):

```cpp
struct BitrateEngineConfig {
    double baseRedundancyRatio{0.5};
    double blocksPerFrame{2.0};
    int    kMin{2};
    int    kMax{50};
    int    minBitrateKbps{1000};
    int    maxBitrateKbps{24000};
    int    mtuBytes{1500};   // from link.mtu
    int    fps{60};          // from video.fps
};
```
Then add a field to `struct DlRuntimeConfig` (e.g. after `SafeDefaults safe;`, ~line 30):
```cpp
    BitrateEngineConfig bitrate;
```

- [ ] **Step 5: Implement the mapping** — in `drone/src/dynlink/runtime_config.cpp`, inside `buildDlSnapshot`, after the `s.safe = SafeDefaults{...};` block (~line 37), add:

```cpp
    s.bitrate = BitrateEngineConfig{
        dl.fec.baseRedundancyRatio,
        dl.fec.blocksPerFrame,
        dl.fec.kMin,
        dl.fec.kMax,
        dl.bitrate.minBitrateKbps,
        dl.bitrate.maxBitrateKbps,
        c.link.mtu,
        c.video.fps,
    };
```

- [ ] **Step 6: Run to verify** — `cmake --build build -j && ./build/fpvd_tests --test-case="*Phase-3a bitrate-engine*"` → PASS. Then the full suite green (the existing `test_schema` round-trip should still pass — `WITH_DEFAULT` keeps old configs loading).

- [ ] **Step 7: Commit**
```bash
git add drone/src/config/schema.hpp drone/src/dynlink/runtime_config.hpp drone/src/dynlink/runtime_config.cpp drone/tests/unit/test_dl_runtime_config.cpp
git commit -m "feat(drone/dynlink): bitrate-engine config knobs (Phase 3a)"
```

---

## Task 4: The composer — `applyLocalCompute`

**Files:** Create `drone/src/dynlink/local_compute.hpp`, `drone/src/dynlink/local_compute.cpp`, `drone/tests/unit/test_dl_local_compute.cpp`; Modify `drone/CMakeLists.txt`.

A pure function that takes a decoded `Decision` and overwrites the drone-local fields (`bitrate`, `k`, `n`, `depth`, `fps`) from the bitrate + fec units. `mcs`, `bandwidth`, and `txPowerDbm` are left untouched. This is where all the Phase-3a math composes — unit-tested directly, no controller harness.

- [ ] **Step 1: Write the header** — `drone/src/dynlink/local_compute.hpp`:

```cpp
/* local_compute.hpp — Phase 3a drone-local decision compute. Overwrites the
 * GS-sent bitrate/k/n/depth/fps on a decoded Decision with values the drone
 * derives from {mcs, bandwidth} via the OpenIPC calculator + block-fill FEC. */
#pragma once
#include "dynlink/wire.hpp"            // Decision
#include "dynlink/runtime_config.hpp"  // DlRuntimeConfig
#include <cstdint>

namespace fpvd::dynlink {

// Constant interleave depth (no config field, by design — see the Phase-3a
// spec). Applied via the existing per-decision depth diff in dispatchTxApply.
inline constexpr uint8_t kInterleaveDepth = 1;

// Overwrites d.bitrateKbps / d.k / d.n / d.depth / d.fps in place from the
// drone-local engine. Leaves d.mcs, d.bandwidth, d.txPowerDbm untouched.
void applyLocalCompute(const DlRuntimeConfig& cfg, Decision& d);

} // namespace fpvd::dynlink
```

- [ ] **Step 2: Write the failing test** — `drone/tests/unit/test_dl_local_compute.cpp`:

```cpp
/* test_dl_local_compute.cpp — Phase 3a decision compose/override. */
#include "doctest.h"
#include "dynlink/local_compute.hpp"
#include "dynlink/bitrate.hpp"
#include "dynlink/fec.hpp"
#include "probe/probe_constants.hpp"
using namespace fpvd::dynlink;

static DlRuntimeConfig cfgWithBitrate() {
    DlRuntimeConfig c{};
    c.probeMcsCeiling = 7;
    c.bitrate = BitrateEngineConfig{0.5, 2.0, 2, 50, 1000, 24000, 1500, 60};
    return c;
}

TEST_CASE("applyLocalCompute overrides bitrate/k/n/depth/fps, keeps mcs/bw/txpower") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    Decision d{};
    d.mcs = 5; d.bandwidth = 20; d.txPowerDbm = 27;
    // GS-sent values that MUST be overridden:
    d.bitrateKbps = 9999; d.k = 99; d.n = 99; d.depth = 9; d.fps = 30;

    applyLocalCompute(cfg, d);

    double probeKbps = static_cast<double>(fpvd::kProbePps) * fpvd::kProbePacketBytes * 8.0 / 1000.0;
    double wt = computeWireTargetKbps(20, 5, 7, probeKbps);
    int    k  = computeK(wt, 1500, 60, 0.5, 2.0, 2, 50);
    int    n  = computeN(k, 0.5);

    CHECK(d.k == static_cast<uint8_t>(k));
    CHECK(d.n == static_cast<uint8_t>(n));
    CHECK(d.bitrateKbps == computeBitrateKbps(wt, k, n, 1000, 24000));
    CHECK(d.depth == kInterleaveDepth);     // constant 1
    CHECK(d.fps == 60);                     // drone video.fps, not the wire 30
    // untouched:
    CHECK(d.mcs == 5);
    CHECK(d.bandwidth == 20);
    CHECK(d.txPowerDbm == 27);
}

TEST_CASE("applyLocalCompute is monotonic in mcs (higher rung -> higher bitrate)") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    Decision lo{}; lo.mcs = 2; lo.bandwidth = 20;
    Decision hi{}; hi.mcs = 5; hi.bandwidth = 20;
    applyLocalCompute(cfg, lo);
    applyLocalCompute(cfg, hi);
    CHECK(hi.bitrateKbps > lo.bitrateKbps);
}
```

- [ ] **Step 3: Wire into CMake** — add `src/dynlink/local_compute.cpp` to `fpvd_core` and `tests/unit/test_dl_local_compute.cpp` to `fpvd_tests` in `drone/CMakeLists.txt`.

- [ ] **Step 4: Run to verify it fails** — `cmake --build build -j 2>&1 | tail -5` → FAIL (`undefined reference to applyLocalCompute`).

- [ ] **Step 5: Implement** — `drone/src/dynlink/local_compute.cpp`:

```cpp
/* local_compute.cpp — Phase 3a drone-local decision compute. */
#include "dynlink/local_compute.hpp"
#include "dynlink/bitrate.hpp"
#include "dynlink/fec.hpp"
#include "probe/probe_constants.hpp"

namespace fpvd::dynlink {

void applyLocalCompute(const DlRuntimeConfig& cfg, Decision& d) {
    const BitrateEngineConfig& b = cfg.bitrate;
    // The probe runs FEC-off at a fixed rate; its true on-air kbps.
    double probeKbps =
        static_cast<double>(fpvd::kProbePps) * fpvd::kProbePacketBytes * 8.0 / 1000.0;
    double wireTarget =
        computeWireTargetKbps(d.bandwidth, d.mcs, cfg.probeMcsCeiling, probeKbps);
    int k = computeK(wireTarget, b.mtuBytes, b.fps,
                     b.baseRedundancyRatio, b.blocksPerFrame, b.kMin, b.kMax);
    int n = computeN(k, b.baseRedundancyRatio);
    d.k           = static_cast<uint8_t>(k);
    d.n           = static_cast<uint8_t>(n);
    d.bitrateKbps = computeBitrateKbps(wireTarget, k, n,
                                       b.minBitrateKbps, b.maxBitrateKbps);
    d.depth       = kInterleaveDepth;
    d.fps         = static_cast<uint8_t>(b.fps);
}

} // namespace fpvd::dynlink
```

- [ ] **Step 6: Run to verify it passes** — `cmake --build build -j && ./build/fpvd_tests --test-case="*applyLocalCompute*"` → PASS; then full suite green.

- [ ] **Step 7: Commit**
```bash
git add drone/src/dynlink/local_compute.hpp drone/src/dynlink/local_compute.cpp drone/tests/unit/test_dl_local_compute.cpp drone/CMakeLists.txt
git commit -m "feat(drone/dynlink): applyLocalCompute decision composer (Phase 3a)"
```

---

## Task 5: Wire into the controller + drop dynamic tx_power

**Files:** Modify `drone/src/dynlink/controller.cpp`.

Call `applyLocalCompute(cfg, d)` immediately after a Decision decodes (so every downstream branch uses the drone-computed values), and remove the dynamic tx-power modulation (`radio_->apply` / `radio_->applySafe`) — tx power now stays at the radio bring-up value (`link.txpower`). Verified by the existing controller/runtime tests staying green plus Task 6.

- [ ] **Step 1: Add the include** — at the top of `drone/src/dynlink/controller.cpp`, with the other `dynlink/*` includes, add:
```cpp
#include "dynlink/local_compute.hpp"
```

- [ ] **Step 2: Call the composer after decode** — in the decision branch, the `else` block that runs after `dedup_.check(d.sequence)` passes (right after `applyState` is forced to `Idle`, before `uint64_t now = nowMonotonicMs();` ~line 462), insert:
```cpp
                        // Phase 3a: the drone computes its own bitrate/k/n
                        // (and a constant depth/fps) from {mcs,bandwidth};
                        // the GS-sent values on the wire are ignored.
                        applyLocalCompute(cfg, d);
```
This must run **before** `applyDirection(lastEnc_.bitrateKbps, d.bitrateKbps, first)` so the direction + all applies use the computed bitrate.

- [ ] **Step 3: Drop the dynamic tx-power applies.** In the same file, remove the four tx-power calls so tx power is no longer modulated per-decision (it stays at the bring-up `link.txpower`):
  - In the **single-shot** branch (`if (!canStagger || dir == ApplyDir::Equal)`, ~line 476-481): delete the line `radio_->apply(d.txPowerDbm);` and the line `lastRadio_ = d;`. Keep `dispatchTxApply(cfg, d);`, `enc_->apply(d.bitrateKbps, d.fps);`, `lastEnc_ = d;`.
  - In the **Up** branch (`else if (dir == ApplyDir::Up)`, ~line 482-491): delete `radio_->apply(d.txPowerDbm);`, `lastRadio_ = d;`, and the `if (subPaceUs > 0) usleep(subPaceUs);` that preceded `dispatchTxApply` (the sub-pace existed only to separate the power-up from the radio push). The branch becomes:
```cpp
                        } else if (dir == ApplyDir::Up) {
                            // Raise capacity (mcs) now; the encoder bitrate
                            // expands after the outer gap. tx power is constant
                            // (set at radio bring-up), so there is no power step.
                            dispatchTxApply(cfg, d);
                            applyPending = d;
                            applyState = ApplyState::UpGap;
                            armGap(gapFd, cfg.applyStaggerMs);
                        }
```
  - In the **Down-gap** handler (`else if (applyState == ApplyState::DownGap)`, ~line 568-574): delete the `if (cfg.applySubPaceMs > 0) { usleep(...); }` block and `radio_->apply(applyPending.txPowerDbm);` and `lastRadio_ = applyPending;`. The branch becomes:
```cpp
                            } else if (applyState == ApplyState::DownGap) {
                                dispatchTxApply(cfg, applyPending);
                            }
```
  - In the **watchdog** safe path (`if (watchdog_->tick(now))`, ~line 527-529): delete `radio_->applySafe(cfg.safe.txPowerDbm);`. Keep `dispatchTxSafe(cfg);` and `enc_->applySafe(cfg.safe.bitrateKbps);`.

  Leave the `radio_` member, `lastRadio_`, and `radio_->setIface(cfg.iface)` (watchdog reset) as-is — they are now vestigial but harmless; removing the member would touch the header + constructor and is out of scope for 3a (cleaned up in 3b). `subPaceUs` may become unused in this scope after the Up-branch edit — if the compiler warns `unused variable 'subPaceUs'`, delete its declaration (`useconds_t subPaceUs = ...`, ~line 466-467).

- [ ] **Step 4: Build + run the full suite** —
```
cd /home/gilankpam/Projects/drone/fpvd/drone && cmake --build build -j && ./build/fpvd_tests 2>&1 | tail -3
```
Expected: PASS (all green). The existing `test_dl_controller` decision-dispatch cases still pass — `dispatchTxApply` now receives the drone-computed `k/n/depth` (FakeWfbTx records them; the existing assertions are on `mcs/bandwidth`, which are unchanged, and on FEC being emitted, which it still is). If any existing controller test asserted a *specific* `k`/`n`/bitrate that came from the GS-sent decision, update it to the drone-computed value (compute via `computeK`/`computeN`/`computeBitrateKbps` for that decision's `mcs`/`bandwidth`) — note any such change in the commit body.

- [ ] **Step 5: Commit**
```bash
git add drone/src/dynlink/controller.cpp
git commit -m "feat(drone/dynlink): drone-local compute in the decision loop; constant tx_power (Phase 3a)"
```

---

## Task 6: On-hardware smoke (needs live drone + GS)

**Files:** none (verification). Redeploy the **drone only** (no GS change).

- [ ] **Step 1:** Build the drone for the target and deploy: `./deploy/drone/deploy.sh` (use the repo's drone deploy path/host). Confirm fpvd restarts cleanly (watch for the `S99fpvd` restart race — recover with `rm -f /var/run/fpvd.pid; /etc/init.d/S99fpvd start` if it hits).
- [ ] **Step 2:** Set a sensible constant `link.txpower` for range **before** enabling (tx power is no longer modulated — it sits at this bring-up value). Enable `dynamicLink` on both ends.
- [ ] **Step 3:** Confirm via the drone `/air/status` (and `/api` wfb state) that the **drone-computed** `bitrate`/`k`/`n` are applied — independent of the GS-sent wire values (the GS `/status` will still show its own computed numbers; that mismatch is expected in 3a, reconciled in 3b). Spot-check a couple of MCS rungs against `computeBitrateKbps`/`computeK`/`computeN`.
- [ ] **Step 4:** Drive MCS changes (GS selector) and confirm video stays healthy across them; `waybeam`/`wfb_video_tx` PIDs **unchanged** (no runner bounce); `depth` stays at 1; tx power stays at `link.txpower` throughout.
- [ ] **Step 5:** Disable `dynamicLink`; confirm clean teardown (safe defaults applied, video healthy).

---

## Self-Review

**Spec coverage (`2026-06-07-phase3a-drone-local-compute-design.md`):**
- §4 OpenIPC table + `(2/3 − probe_util) × k/n` bitrate → Task 1 (`bitrate.cpp`) + Task 4 (compose). ✓
- §5 latency-sized `k` + fixed-ratio `n`, no escalator → Task 2 (`fec.cpp`). ✓
- §6 constant depth (`kInterleaveDepth`, no config) + constant tx_power (drop dynamic apply) → Task 4 (depth) + Task 5 (tx_power). ✓
- §7 config delta (bitrate-engine under `dynamicLink`; mtu/fps reused) → Task 3. ✓
- §3 drone-only, wire v2 unchanged, GS untouched → no GS/wire/HELLO file is modified; the Decision struct is unchanged; the GS-sent fields are simply overwritten post-decode (Task 5). ✓
- §10 testing (unit: table/formula/compute_k/n/compose; hardware smoke) → Tasks 1-4 unit + Task 6 smoke. ✓

**Placeholder scan:** Task 3 Step 1 ("add includes if missing — match the file's existing includes") and Task 5 Step 4 ("if an existing controller test asserted a specific GS k/n, update it") are real seams the implementer resolves against the actual files; the assertion/update logic is specified (recompute via the named functions). No TBDs; every code step has complete code.

**Type/name consistency:** `openIpcBaseRateKbps`/`computeWireTargetKbps`/`computeBitrateKbps` (Task 1) ↔ used in Task 4 + its test. `computeK`/`computeN` (Task 2) ↔ Task 4. `BitrateEngineConfig` fields (Task 3, runtime_config.hpp) ↔ `DlRuntimeConfig.bitrate` use in Task 4 (`b.mtuBytes`, `b.fps`, `b.baseRedundancyRatio`, `b.blocksPerFrame`, `b.kMin/kMax`, `b.minBitrateKbps/maxBitrateKbps`) ↔ mapped from `dl.fec.*`/`dl.bitrate.*`/`c.link.mtu`/`c.video.fps` (Task 3, runtime_config.cpp). `kInterleaveDepth` (Task 4 header) ↔ Task 4 impl/test. `applyLocalCompute(const DlRuntimeConfig&, Decision&)` (Task 4) ↔ called in Task 5. `kProbePps`/`kProbePacketBytes` are in `namespace fpvd` (qualified `fpvd::` in the `fpvd::dynlink` impl/test). `cfg.probeMcsCeiling` is the existing `DlRuntimeConfig` field (=7).

**Note on a spec wording fix:** the spec §4 says "`round(...)`" but also "int-truncation rounds the wire rate down"; the implementation **truncates** (Task 1 `computeBitrateKbps`), matching the GS invariant (wire ≤ target). Truncation is the operative behavior.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-07-phase3a-drone-local-compute.md`.** Two execution options:

1. **Subagent-Driven (recommended)** — fresh subagent per task, spec + quality review between tasks (Tasks 1-5 local; STOP before Task 6 hardware).
2. **Inline Execution** — execute tasks in this session via executing-plans, checkpoints for review.
