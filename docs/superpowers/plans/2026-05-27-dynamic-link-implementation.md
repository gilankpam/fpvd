# wfbng-dynamic-link Integration — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add `dl-applier` (drone-side of wfbng-dynamic-link) to `fpvd` as a first-class supervised subsystem, configured via the existing `PATCH /config` API and driven entirely by CLI args (no conf file).

**Architecture:** New `dynamicLink` top-level schema section + new translator + new lock-check module. Orchestrator gets one more managed process (`dl_applier`) seeded conditionally on `dynamicLink.enabled`. Apply path already rebuilds the orchestrator, so process add/remove is automatic; the only new control-flow is detecting that `dynamicLink.*`, `link.mtu`, or `video.fps` changed so it lands in the `restarted` response.

**Tech Stack:** C++17, nlohmann/json, doctest, CMake. Same toolchain as the existing fpvd codebase.

**Spec:** `docs/superpowers/specs/2026-05-27-dynamic-link-design.md`

---

## File map

**New source files:**
- `src/config/lock.hpp` / `src/config/lock.cpp` — cross-field lock check (paths in PATCH body × pending `dynamicLink.enabled`).
- `src/translate/dynamic_link.hpp` / `src/translate/dynamic_link.cpp` — `Config → dl-applier argv`.

**Modified source files:**
- `src/config/schema.hpp` — add `DynamicLink*` structs, add `dynamicLink` field to `Config`.
- `src/config/validate.cpp` — add range checks for the new section.
- `src/config/diff.hpp` / `src/config/diff.cpp` — add `dynamicLink` flag to `SubsystemDiff`; flag it for `dynamicLink.*` changes plus `link.mtu` / `video.fps`.
- `src/daemon.cpp` — call lock check in `patchPending`; seed `dl_applier` in `seedOrchestrator` when `enabled`; include `dl_applier` in `restarted` list when `subs.dynamicLink`.
- `src/http/handlers.cpp` — surface `dynamic_link_locked` 400 error from `patchPending`.
- `src/daemon.hpp` — add a third `PatchResult::Reason` discriminator so handlers can distinguish lock-fail from schema-fail.
- `etc/defaults.json` — append `dynamicLink` defaults.
- `tests/fixtures/defaults.json` — same.
- `CMakeLists.txt` — list new source and test files.

**New test files:**
- `tests/unit/test_lock.cpp`
- `tests/unit/test_translate_dynamic_link.cpp`
- `tests/unit/test_dl_applier_cli_assumptions.cpp`
- `tests/fixtures/dl_applier_help.txt` — vendored copy of `dl-applier --help` output (operator pastes once).

**Modified test files:**
- `tests/unit/test_schema.cpp` — add round-trip for `dynamicLink`.
- `tests/unit/test_validate.cpp` — add new range checks.
- `tests/unit/test_diff.cpp` — add `dynamicLink` rows.
- `tests/integration/test_http_handlers.cpp` — add `dynamic_link_locked` case.
- `tests/integration/test_daemon.cpp` — add e2e dl_applier startup case (if file structure permits — otherwise add a new fixture file).

---

## Task 1: Schema — add `DynamicLink` types

**Files:**
- Modify: `src/config/schema.hpp` (insert before `struct Config`; add `dynamicLink` field to `Config`)
- Test: `tests/unit/test_schema.cpp`

- [ ] **Step 1: Read the existing `tests/unit/test_schema.cpp` to learn its conventions, then add a failing round-trip test for the new section.**

Append to `tests/unit/test_schema.cpp`:

```cpp
TEST_CASE("schema: dynamicLink round-trips through json") {
    using nlohmann::json;
    fpvd::Config c{};
    c.dynamicLink.enabled = true;
    c.dynamicLink.safe.mcs = 3;
    c.dynamicLink.roiQp.floor = -18;
    c.dynamicLink.osd.debugLatency = true;
    json j = c;
    fpvd::Config c2 = j.get<fpvd::Config>();
    CHECK(c2.dynamicLink.enabled == true);
    CHECK(c2.dynamicLink.safe.mcs == 3);
    CHECK(c2.dynamicLink.roiQp.floor == -18);
    CHECK(c2.dynamicLink.osd.debugLatency == true);
    // unchanged defaults round-trip too
    CHECK(c2.dynamicLink.healthTimeoutMs == 10000);
    CHECK(c2.dynamicLink.interleavingSupported == true);
    CHECK(c2.dynamicLink.mavlinkEnable == true);
}

TEST_CASE("schema: dynamicLink defaults match spec") {
    fpvd::Config c{};
    CHECK(c.dynamicLink.enabled == false);
    CHECK(c.dynamicLink.healthTimeoutMs == 10000);
    CHECK(c.dynamicLink.interleavingSupported == true);
    CHECK(c.dynamicLink.debug == false);
    CHECK(c.dynamicLink.minIdrIntervalMs == 500);
    CHECK(c.dynamicLink.applyStaggerMs == 50);
    CHECK(c.dynamicLink.applySubPaceMs == 5);
    CHECK(c.dynamicLink.mavlinkEnable == true);
    CHECK(c.dynamicLink.osd.enabled == true);
    CHECK(c.dynamicLink.osd.debugLatency == false);
    CHECK(c.dynamicLink.roiQp.thresholdKbps == 6000);
    CHECK(c.dynamicLink.roiQp.lowAnchorKbps == 2000);
    CHECK(c.dynamicLink.roiQp.floor == -24);
    CHECK(c.dynamicLink.roiQp.step == 3);
    CHECK(c.dynamicLink.safe.mcs == 1);
    CHECK(c.dynamicLink.safe.k == 8);
    CHECK(c.dynamicLink.safe.n == 12);
    CHECK(c.dynamicLink.safe.depth == 1);
    CHECK(c.dynamicLink.safe.bandwidth == 20);
    CHECK(c.dynamicLink.safe.txPowerDbm == 20);
    CHECK(c.dynamicLink.safe.bitrateKbps == 2000);
}
```

- [ ] **Step 2: Run the new tests; verify they fail to compile (DynamicLink not declared).**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j 2>&1 | head -40`
Expected: compile error referencing `dynamicLink` not a member of `Config`.

- [ ] **Step 3: Add the structs and the Config field.**

In `src/config/schema.hpp`, insert these blocks **before** `struct Config {`:

```cpp
struct DynamicLinkSafe {
    int mcs{1};
    int k{8};
    int n{12};
    int depth{1};
    int bandwidth{20};
    int txPowerDbm{20};
    int bitrateKbps{2000};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DynamicLinkSafe, mcs, k, n, depth,
                                   bandwidth, txPowerDbm, bitrateKbps)

struct DynamicLinkOsd {
    bool enabled{true};
    bool debugLatency{false};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DynamicLinkOsd, enabled, debugLatency)

struct DynamicLinkRoiQp {
    int thresholdKbps{6000};
    int lowAnchorKbps{2000};
    int floor{-24};
    int step{3};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DynamicLinkRoiQp, thresholdKbps,
                                   lowAnchorKbps, floor, step)

struct DynamicLink {
    bool enabled{false};
    int healthTimeoutMs{10000};
    bool interleavingSupported{true};
    bool debug{false};
    int minIdrIntervalMs{500};
    int applyStaggerMs{50};
    int applySubPaceMs{5};
    bool mavlinkEnable{true};
    DynamicLinkOsd osd{};
    DynamicLinkRoiQp roiQp{};
    DynamicLinkSafe safe{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(DynamicLink, enabled, healthTimeoutMs,
                                   interleavingSupported, debug,
                                   minIdrIntervalMs, applyStaggerMs,
                                   applySubPaceMs, mavlinkEnable, osd,
                                   roiQp, safe)
```

Then in `struct Config { ... }` add the field (between `Snapshot snapshot{};` and `std::map<std::string, Service> services{};`):

```cpp
    DynamicLink dynamicLink{};
```

And update the `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` macro for `Config` to include `dynamicLink`:

```cpp
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Config, link, video, image, telemetry,
                                   recording, snapshot, dynamicLink, services)
```

- [ ] **Step 4: Build and run the two new tests; verify they pass.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='schema: dynamicLink*'`
Expected: 2 test cases pass.

- [ ] **Step 5: Run the full test suite — many tests parse defaults.json into Config and will fail until Task 2 lands.**

Run: `./build/fpvd_tests 2>&1 | tail -20`
Expected: the new tests pass; existing tests that load `tests/fixtures/defaults.json` continue to pass (nlohmann's `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` is **non-strict** — missing keys take their default value, so the absence of `dynamicLink` in the fixture is fine). If any test fails, do not commit; investigate.

- [ ] **Step 6: Commit.**

```bash
git add src/config/schema.hpp tests/unit/test_schema.cpp
git commit -m "feat(schema): add dynamicLink section to Config"
```

---

## Task 2: Defaults JSON — append `dynamicLink` section

**Files:**
- Modify: `etc/defaults.json`
- Modify: `tests/fixtures/defaults.json`
- Test: `tests/unit/test_store.cpp` (already exercises defaults parsing; we just verify the new section survives the round-trip)

- [ ] **Step 1: Update `etc/defaults.json` to its new shape.**

Replace the entire file with:

```json
{
  "link": {"channel": 161, "width": 20, "txpower": 1, "mcs": 2,
           "fec": {"k": 8, "n": 12}, "stbc": false, "ldpc": false,
           "linkId": 7669206, "mtu": 1500, "wlanAdapter": null},
  "video": {"codec": "h265", "resolution": "1920x1080", "fps": 60,
            "bitrate": 8192, "rcMode": "cbr", "gopSize": 1.0, "qpDelta": -4,
            "roi": {"enabled": true, "qp": 0, "center": 0.4, "steps": 2}},
  "image": {"mirror": false, "flip": false, "rotate": 0},
  "telemetry": {"router": "msposd", "serial": "ttyS2", "osdFps": 20, "baud": 115200},
  "recording": {"enabled": false, "dir": "/mnt/mmcblk0p1", "format": "ts",
                "mode": "mirror", "maxSeconds": 300, "maxMB": 500},
  "snapshot": {"enabled": true, "quality": 80},
  "dynamicLink": {
    "enabled": false,
    "healthTimeoutMs": 10000,
    "interleavingSupported": true,
    "debug": false,
    "minIdrIntervalMs": 500,
    "applyStaggerMs": 50,
    "applySubPaceMs": 5,
    "mavlinkEnable": true,
    "osd": {"enabled": true, "debugLatency": false},
    "roiQp": {"thresholdKbps": 6000, "lowAnchorKbps": 2000, "floor": -24, "step": 3},
    "safe": {"mcs": 1, "k": 8, "n": 12, "depth": 1,
             "bandwidth": 20, "txPowerDbm": 20, "bitrateKbps": 2000}
  },
  "services": {}
}
```

- [ ] **Step 2: Update `tests/fixtures/defaults.json` to the same content.**

Write the identical file contents to `tests/fixtures/defaults.json`.

- [ ] **Step 3: Add a regression test that the defaults file actually contains the section (catches future copy-paste mistakes).**

Append to `tests/unit/test_store.cpp`:

```cpp
TEST_CASE("store: defaults file carries dynamicLink section") {
    auto c = fpvd::loadEffective("tests/fixtures/defaults.json",
                                  "/no/such/path");
    CHECK(c.dynamicLink.enabled == false);
    CHECK(c.dynamicLink.safe.mcs == 1);
    CHECK(c.dynamicLink.roiQp.thresholdKbps == 6000);
}
```

- [ ] **Step 4: Build and run.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='store: defaults file*'`
Expected: PASS.

- [ ] **Step 5: Run the full test suite to confirm no regressions.**

Run: `./build/fpvd_tests 2>&1 | tail -5`
Expected: all tests pass.

- [ ] **Step 6: Commit.**

```bash
git add etc/defaults.json tests/fixtures/defaults.json tests/unit/test_store.cpp
git commit -m "feat(defaults): ship dynamicLink baseline (disabled by default)"
```

---

## Task 3: Validation rules for `dynamicLink`

**Files:**
- Modify: `src/config/validate.cpp`
- Test: `tests/unit/test_validate.cpp`

- [ ] **Step 1: Write failing tests covering every rule from spec §5.**

Append to `tests/unit/test_validate.cpp`:

```cpp
TEST_CASE("validate: dynamicLink.safe.mcs in [0,7]") {
    Config c{}; c.dynamicLink.safe.mcs = 8;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.mcs");
}

TEST_CASE("validate: dynamicLink.safe k<n and both in [1,32]") {
    Config c{}; c.dynamicLink.safe.k = 12; c.dynamicLink.safe.n = 8;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.fec");

    Config c2{}; c2.dynamicLink.safe.k = 0;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "dynamicLink.safe.fec");

    Config c3{}; c3.dynamicLink.safe.n = 33;
    auto errs3 = validate(c3);
    REQUIRE(errs3.size() == 1);
    CHECK(errs3[0].path == "dynamicLink.safe.fec");
}

TEST_CASE("validate: dynamicLink.safe.depth in [1,8]") {
    Config c{}; c.dynamicLink.safe.depth = 0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.depth");

    Config c2{}; c2.dynamicLink.safe.depth = 9;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "dynamicLink.safe.depth");
}

TEST_CASE("validate: dynamicLink.safe.bandwidth must be 20 or 40") {
    Config c{}; c.dynamicLink.safe.bandwidth = 80;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.bandwidth");
}

TEST_CASE("validate: dynamicLink.safe.txPowerDbm in [0,30]") {
    Config c{}; c.dynamicLink.safe.txPowerDbm = 31;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.txPowerDbm");

    Config c2{}; c2.dynamicLink.safe.txPowerDbm = -1;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "dynamicLink.safe.txPowerDbm");
}

TEST_CASE("validate: dynamicLink.safe.bitrateKbps > 0") {
    Config c{}; c.dynamicLink.safe.bitrateKbps = 0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.bitrateKbps");
}

TEST_CASE("validate: dynamicLink.healthTimeoutMs >= 1000") {
    Config c{}; c.dynamicLink.healthTimeoutMs = 500;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.healthTimeoutMs");
}

TEST_CASE("validate: dynamicLink.minIdrIntervalMs >= 16") {
    Config c{}; c.dynamicLink.minIdrIntervalMs = 10;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.minIdrIntervalMs");
}

TEST_CASE("validate: dynamicLink.applyStaggerMs in [0,1000]") {
    Config c{}; c.dynamicLink.applyStaggerMs = 1001;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.applyStaggerMs");
}

TEST_CASE("validate: dynamicLink.applySubPaceMs in [0,50]") {
    Config c{}; c.dynamicLink.applySubPaceMs = 51;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.applySubPaceMs");
}

TEST_CASE("validate: dynamicLink.roiQp threshold > lowAnchor > 0") {
    Config c{}; c.dynamicLink.roiQp.thresholdKbps = 1000;
    c.dynamicLink.roiQp.lowAnchorKbps = 2000;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.roiQp");

    Config c2{}; c2.dynamicLink.roiQp.lowAnchorKbps = 0;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "dynamicLink.roiQp");
}

TEST_CASE("validate: dynamicLink.roiQp.floor must be <= 0") {
    Config c{}; c.dynamicLink.roiQp.floor = 1;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.roiQp.floor");
}

TEST_CASE("validate: dynamicLink.roiQp.step >= 1") {
    Config c{}; c.dynamicLink.roiQp.step = 0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.roiQp.step");
}
```

- [ ] **Step 2: Run; verify they fail.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='validate: dynamicLink*'`
Expected: every new test fails (errs is empty, REQUIRE fails).

- [ ] **Step 3: Add the rules in `src/config/validate.cpp`.**

Insert a new block at the end of `validate()`, **before** the `return errs;` line:

```cpp
    // dynamicLink
    {
        const auto& dl = c.dynamicLink;
        if (dl.safe.mcs < 0 || dl.safe.mcs > 7)
            errs.push_back({"dynamicLink.safe.mcs", "must be 0..7"});
        if (dl.safe.k < 1 || dl.safe.k > 32 ||
            dl.safe.n < 1 || dl.safe.n > 32 ||
            dl.safe.k >= dl.safe.n)
            errs.push_back({"dynamicLink.safe.fec", "require 1<=k<n<=32"});
        if (dl.safe.depth < 1 || dl.safe.depth > 8)
            errs.push_back({"dynamicLink.safe.depth", "must be 1..8"});
        if (dl.safe.bandwidth != 20 && dl.safe.bandwidth != 40)
            errs.push_back({"dynamicLink.safe.bandwidth", "must be 20 or 40"});
        if (dl.safe.txPowerDbm < 0 || dl.safe.txPowerDbm > 30)
            errs.push_back({"dynamicLink.safe.txPowerDbm", "must be 0..30"});
        if (dl.safe.bitrateKbps <= 0)
            errs.push_back({"dynamicLink.safe.bitrateKbps", "must be > 0"});

        if (dl.healthTimeoutMs < 1000)
            errs.push_back({"dynamicLink.healthTimeoutMs", "must be >= 1000"});
        if (dl.minIdrIntervalMs < 16)
            errs.push_back({"dynamicLink.minIdrIntervalMs", "must be >= 16"});
        if (dl.applyStaggerMs < 0 || dl.applyStaggerMs > 1000)
            errs.push_back({"dynamicLink.applyStaggerMs", "must be 0..1000"});
        if (dl.applySubPaceMs < 0 || dl.applySubPaceMs > 50)
            errs.push_back({"dynamicLink.applySubPaceMs", "must be 0..50"});

        if (dl.roiQp.thresholdKbps <= 0 ||
            dl.roiQp.lowAnchorKbps <= 0 ||
            dl.roiQp.thresholdKbps <= dl.roiQp.lowAnchorKbps)
            errs.push_back({"dynamicLink.roiQp",
                            "require thresholdKbps > lowAnchorKbps > 0"});
        if (dl.roiQp.floor > 0)
            errs.push_back({"dynamicLink.roiQp.floor", "must be <= 0"});
        if (dl.roiQp.step < 1)
            errs.push_back({"dynamicLink.roiQp.step", "must be >= 1"});
    }
```

- [ ] **Step 4: Run new tests; verify they pass.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='validate: dynamicLink*'`
Expected: every new test passes.

- [ ] **Step 5: Run all tests; verify no regressions.**

Run: `./build/fpvd_tests`
Expected: all PASS.

- [ ] **Step 6: Commit.**

```bash
git add src/config/validate.cpp tests/unit/test_validate.cpp
git commit -m "feat(validate): add dynamicLink range checks"
```

---

## Task 4: Cross-field lock module

**Files:**
- Create: `src/config/lock.hpp`
- Create: `src/config/lock.cpp`
- Create: `tests/unit/test_lock.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add the header.**

Create `src/config/lock.hpp`:

```cpp
#pragma once
#include "config/schema.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace fpvd {

// When dynamicLink.enabled is true in the merged-pending config, the
// PATCH body may not write to any of these paths. The result lists the
// exact dotted paths the body tried to touch.
//
// Evaluation rule: the PATCH body itself is walked; the body's deep
// structure is what's checked (so writing `link.fec` wholesale counts
// as writing the subtree). The "would pending have enabled==true?"
// question is answered by the caller passing the merged pending Config.
struct LockResult {
    bool ok{true};
    std::vector<std::string> lockedPaths{};
};

LockResult checkDynamicLinkLock(const nlohmann::json& patchBody,
                                const Config& mergedPending);

} // namespace fpvd
```

- [ ] **Step 2: Write the failing tests.**

Create `tests/unit/test_lock.cpp`:

```cpp
#include "doctest.h"
#include "config/lock.hpp"
#include <nlohmann/json.hpp>

using fpvd::Config;
using fpvd::checkDynamicLinkLock;

static Config dlOn() {
    Config c{}; c.dynamicLink.enabled = true; return c;
}

TEST_CASE("lock: DL off → any path passes") {
    Config off{};
    auto body = nlohmann::json::parse(R"({"link":{"mcs":5}})");
    auto r = checkDynamicLinkLock(body, off);
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}

TEST_CASE("lock: DL on + body writes link.mcs → rejected") {
    auto body = nlohmann::json::parse(R"({"link":{"mcs":5}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.mcs");
}

TEST_CASE("lock: DL on + body writes link.fec.k → rejected") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"k":4}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.fec.k");
}

TEST_CASE("lock: DL on + body overwrites link.fec wholesale → rejected") {
    auto body = nlohmann::json::parse(R"({"link":{"fec":{"k":4,"n":10}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    // Two children inside the locked subtree; either ordering is acceptable.
    REQUIRE(r.lockedPaths.size() == 2);
}

TEST_CASE("lock: DL on + body writes video.roi.qp → rejected") {
    auto body = nlohmann::json::parse(R"({"video":{"roi":{"qp":-10}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "video.roi.qp");
}

TEST_CASE("lock: DL on + body writes link.channel → allowed (not locked)") {
    auto body = nlohmann::json::parse(R"({"link":{"channel":165}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
}

TEST_CASE("lock: DL on + body writes dynamicLink.safe.mcs → allowed") {
    auto body = nlohmann::json::parse(R"({"dynamicLink":{"safe":{"mcs":3}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
}

TEST_CASE("lock: pending evaluated post-merge — body disables DL and writes locked key → allowed") {
    // Effective state has DL on; the body disables it AND writes link.mcs.
    // The caller must pre-compute the merged pending: in this case DL is off
    // after the merge, so the lock is open.
    Config mergedAfterPatch{}; // DL off (default)
    auto body = nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":false},"link":{"mcs":5}})");
    auto r = checkDynamicLinkLock(body, mergedAfterPatch);
    CHECK(r.ok);
}

TEST_CASE("lock: body enables DL and writes locked key → rejected") {
    // Merged pending has enabled=true; body wrote link.mcs in the same op.
    Config merged = dlOn();
    auto body = nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true},"link":{"mcs":5}})");
    auto r = checkDynamicLinkLock(body, merged);
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.mcs");
}

TEST_CASE("lock: multiple locked paths reported together") {
    auto body = nlohmann::json::parse(
        R"({"link":{"mcs":5,"txpower":10},"video":{"bitrate":1000}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    CHECK(r.lockedPaths.size() == 3);
}

TEST_CASE("lock: body writes link.fec but with no children → still rejected") {
    // Wholesale write of the locked subtree as null/object — implementation
    // detail: it counts the key itself.
    auto body = nlohmann::json::parse(R"({"link":{"fec":{}}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK_FALSE(r.ok);
    REQUIRE(r.lockedPaths.size() == 1);
    CHECK(r.lockedPaths[0] == "link.fec");
}
```

- [ ] **Step 3: Add `tests/unit/test_lock.cpp` and `src/config/lock.cpp` to `CMakeLists.txt`.**

In `CMakeLists.txt`, find the `target_sources(fpvd_core PRIVATE` block and add `src/config/lock.cpp` to it. Find the `target_sources(fpvd_tests PRIVATE` block and add `tests/unit/test_lock.cpp` to it.

The expanded blocks look like:

```cmake
target_sources(fpvd_core PRIVATE
    src/config/store.cpp
    src/config/validate.cpp
    src/config/diff.cpp
    src/config/lock.cpp
    src/translate/waybeam.cpp
    src/translate/wfb.cpp
    src/translate/telemetry.cpp
    src/supervise/process.cpp
    src/supervise/supervisor.cpp
    src/supervise/orchestrator.cpp
    src/supervise/radio.cpp
    src/http/server.cpp
    src/http/handlers.cpp
    src/status.cpp
    src/daemon.cpp
)
```

```cmake
target_sources(fpvd_tests PRIVATE
    tests/unit/test_sanity.cpp
    tests/unit/test_schema.cpp
    tests/unit/test_store.cpp
    tests/unit/test_validate.cpp
    tests/unit/test_diff.cpp
    tests/unit/test_lock.cpp
    tests/unit/test_translate_waybeam.cpp
    tests/unit/test_translate_wfb.cpp
    tests/unit/test_translate_telemetry.cpp
    tests/integration/test_process.cpp
    tests/integration/test_supervisor.cpp
    tests/integration/test_orchestrator.cpp
    tests/integration/test_radio.cpp
    tests/integration/test_daemon.cpp
    tests/integration/test_http_server.cpp
    tests/integration/test_http_handlers.cpp
)
```

- [ ] **Step 4: Add an empty stub for `src/config/lock.cpp` so the build can fail at the assertion stage rather than at link-time.**

Create `src/config/lock.cpp`:

```cpp
#include "config/lock.hpp"

namespace fpvd {

LockResult checkDynamicLinkLock(const nlohmann::json& /*patchBody*/,
                                const Config& /*mergedPending*/) {
    return {true, {}};
}

} // namespace fpvd
```

- [ ] **Step 5: Build and run; verify tests compile but fail.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='lock:*' 2>&1 | tail -20`
Expected: tests compile, several FAIL (stub always returns ok).

- [ ] **Step 6: Implement the real lock check.**

Replace `src/config/lock.cpp` contents with:

```cpp
#include "config/lock.hpp"

namespace fpvd {

// Locked subtree: writes anywhere inside count. The strings here are the
// path *prefixes* the body cannot touch when dynamicLink is enabled.
// `link.fec` covers k, n, and a wholesale subtree overwrite alike.
static const std::vector<std::vector<std::string>> kLockedPaths = {
    {"link", "mcs"},
    {"link", "txpower"},
    {"link", "fec"},
    {"link", "width"},
    {"video", "bitrate"},
    {"video", "qpDelta"},
    {"video", "roi"},
};

// Walk the patch body collecting every dotted path it writes. Object
// children recurse; leaf values (numbers, strings, bools, null, arrays)
// terminate the path. An empty-object child still counts: the path leads
// to that key, even if it's wiping the subtree.
static void collectWrittenPaths(const nlohmann::json& body,
                                 std::vector<std::string>& prefix,
                                 std::vector<std::vector<std::string>>& out) {
    if (!body.is_object()) {
        out.push_back(prefix);
        return;
    }
    if (body.empty()) {
        out.push_back(prefix);
        return;
    }
    for (auto it = body.begin(); it != body.end(); ++it) {
        prefix.push_back(it.key());
        collectWrittenPaths(it.value(), prefix, out);
        prefix.pop_back();
    }
}

static std::string joinDotted(const std::vector<std::string>& p) {
    std::string out;
    for (size_t i = 0; i < p.size(); ++i) {
        if (i) out.push_back('.');
        out += p[i];
    }
    return out;
}

// True iff `path` starts with `prefix` (component-wise).
static bool isUnderPrefix(const std::vector<std::string>& path,
                          const std::vector<std::string>& prefix) {
    if (path.size() < prefix.size()) return false;
    for (size_t i = 0; i < prefix.size(); ++i) {
        if (path[i] != prefix[i]) return false;
    }
    return true;
}

// True iff `path` is a strict ancestor of `prefix` (so a wholesale write
// at or above `link.fec` still trips the `link.fec` lock).
static bool isAncestorOf(const std::vector<std::string>& path,
                         const std::vector<std::string>& prefix) {
    return isUnderPrefix(prefix, path);
}

LockResult checkDynamicLinkLock(const nlohmann::json& patchBody,
                                const Config& mergedPending) {
    if (!mergedPending.dynamicLink.enabled) return {true, {}};
    if (!patchBody.is_object()) return {true, {}};

    std::vector<std::vector<std::string>> written;
    std::vector<std::string> prefix;
    collectWrittenPaths(patchBody, prefix, written);

    LockResult r;
    for (auto& w : written) {
        for (auto& lk : kLockedPaths) {
            if (isUnderPrefix(w, lk) || isAncestorOf(w, lk)) {
                r.lockedPaths.push_back(joinDotted(w));
                break;
            }
        }
    }
    r.ok = r.lockedPaths.empty();
    return r;
}

} // namespace fpvd
```

- [ ] **Step 7: Run lock tests; verify they pass.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='lock:*'`
Expected: all lock tests pass.

- [ ] **Step 8: Run all tests.**

Run: `./build/fpvd_tests`
Expected: all PASS.

- [ ] **Step 9: Commit.**

```bash
git add src/config/lock.hpp src/config/lock.cpp tests/unit/test_lock.cpp CMakeLists.txt
git commit -m "feat(lock): cross-field lock for runtime-owned link/video fields"
```

---

## Task 5: Wire lock check into `Daemon::patchPending` and HTTP

**Files:**
- Modify: `src/daemon.hpp`
- Modify: `src/daemon.cpp`
- Modify: `src/http/handlers.cpp`
- Test: `tests/integration/test_http_handlers.cpp`

The current `patchPending` returns `PatchResult{ok, errors}`. We need handlers to distinguish "validation error" (`error: validation`) from "lock error" (`error: dynamic_link_locked`). Smallest change: add a third field to `PatchResult` carrying the lock paths; when non-empty, handlers emit the new error code.

- [ ] **Step 1: Extend `PatchResult`.**

In `src/daemon.hpp`, change the existing `PatchResult` struct to:

```cpp
struct PatchResult {
    bool ok{true};
    std::vector<ValidationError> errors;
    std::vector<std::string> lockedPaths;  // non-empty => 400 dynamic_link_locked
};
```

- [ ] **Step 2: Add a failing handler-level test for the lock.**

Append to `tests/integration/test_http_handlers.cpp`:

```cpp
TEST_CASE("handlers: PATCH /config rejects locked field when DL enabled") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-lock";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18094);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18094");
    // First enable DL in pending and apply, so effective.dynamicLink.enabled = true.
    auto r1 = c.Patch("/config",
        R"({"dynamicLink":{"enabled":true}})", "application/json");
    REQUIRE(r1); CHECK(r1->status == 200);
    auto r2 = c.Post("/apply", "", "application/json");
    REQUIRE(r2); CHECK(r2->status == 200);

    // Now try to write a locked field.
    auto r3 = c.Patch("/config",
        R"({"link":{"mcs":5}})", "application/json");
    REQUIRE(r3);
    CHECK(r3->status == 400);
    auto j = nlohmann::json::parse(r3->body);
    CHECK(j["error"] == "dynamic_link_locked");
    REQUIRE(j["details"]["locked"].is_array());
    CHECK(j["details"]["locked"].size() == 1);
    CHECK(j["details"]["locked"][0] == "link.mcs");

    // Pending should be unchanged.
    CHECK(d->pending().link.mcs == 2);
    srv.stop(); fs::remove_all(tmp);
}

TEST_CASE("handlers: PATCH that disables DL and writes locked key is allowed") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-lock-unlock";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18095);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18095");
    // Enable + apply.
    c.Patch("/config",
        R"({"dynamicLink":{"enabled":true}})", "application/json");
    c.Post("/apply", "", "application/json");

    // Single PATCH disables DL and writes link.mcs.
    auto r = c.Patch("/config",
        R"({"dynamicLink":{"enabled":false},"link":{"mcs":5}})",
        "application/json");
    REQUIRE(r); CHECK(r->status == 200);
    CHECK(d->pending().dynamicLink.enabled == false);
    CHECK(d->pending().link.mcs == 5);
    srv.stop(); fs::remove_all(tmp);
}
```

- [ ] **Step 3: Run; verify both new tests fail.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='handlers: PATCH*lock*'`
Expected: tests fail (lock not wired in; PATCH succeeds when it should reject).

- [ ] **Step 4: Wire the lock check into `Daemon::patchPending`.**

In `src/daemon.cpp`, add `#include "config/lock.hpp"` near the top, then replace the existing `patchPending` body with:

```cpp
PatchResult Daemon::patchPending(const nlohmann::json& patch) {
    std::lock_guard<std::mutex> g(mu_);
    nlohmann::json next = deepMergeJson(nlohmann::json(pending_), patch);
    Config candidate;
    try { candidate = next.get<Config>(); }
    catch (const nlohmann::json::exception& e) {
        return {false, {{"<root>", e.what()}}, {}};
    }
    auto lockR = checkDynamicLinkLock(patch, candidate);
    if (!lockR.ok) {
        return {false, {}, std::move(lockR.lockedPaths)};
    }
    auto errs = validate(candidate);
    if (!errs.empty()) return {false, std::move(errs), {}};
    pending_ = candidate;
    return {true, {}, {}};
}
```

- [ ] **Step 5: Surface `dynamic_link_locked` in the HTTP handler.**

In `src/http/handlers.cpp`, replace the PATCH handler with:

```cpp
    srv.patch("/config", [&](const httplib::Request& req, httplib::Response& res){
        nlohmann::json body;
        try { body = nlohmann::json::parse(req.body); }
        catch (const nlohmann::json::exception&) {
            res.status = 400;
            res.set_content(errBody("bad_json", "request body not valid JSON").dump(),
                            "application/json");
            return;
        }
        auto pr = d.patchPending(body);
        if (!pr.ok) {
            if (!pr.lockedPaths.empty()) {
                nlohmann::json details = {{"locked", pr.lockedPaths}};
                res.status = 400;
                res.set_content(errBody("dynamic_link_locked",
                    "fields owned by dl-applier while dynamicLink.enabled",
                    details).dump(), "application/json");
                return;
            }
            nlohmann::json details = nlohmann::json::array();
            for (auto& e : pr.errors)
                details.push_back({{"path", e.path}, {"message", e.message}});
            res.status = 400;
            res.set_content(errBody("validation", "schema validation failed", details).dump(),
                            "application/json");
            return;
        }
        res.set_content(nlohmann::json(d.pending()).dump(), "application/json");
    });
```

- [ ] **Step 6: Run new tests; verify they pass.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='handlers: PATCH*lock*'`
Expected: PASS.

- [ ] **Step 7: Run all tests.**

Run: `./build/fpvd_tests`
Expected: all PASS.

- [ ] **Step 8: Commit.**

```bash
git add src/daemon.hpp src/daemon.cpp src/http/handlers.cpp tests/integration/test_http_handlers.cpp
git commit -m "feat(http): surface dynamic_link_locked from PATCH /config"
```

---

## Task 6: Translator — `dynamicLinkArgs(Config, iface)`

**Files:**
- Create: `src/translate/dynamic_link.hpp`
- Create: `src/translate/dynamic_link.cpp`
- Create: `tests/unit/test_translate_dynamic_link.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Add the header.**

Create `src/translate/dynamic_link.hpp`:

```cpp
#pragma once
#include "config/schema.hpp"
#include <string>
#include <vector>

namespace fpvd {

// Build the argv (including argv[0] = /usr/bin/dl-applier) for the
// drone-side dl-applier. `iface` is the wlan device picked by
// radio-up.sh.
std::vector<std::string> dynamicLinkArgs(const Config& c,
                                          const std::string& iface);

} // namespace fpvd
```

- [ ] **Step 2: Write the failing tests.**

Create `tests/unit/test_translate_dynamic_link.cpp`:

```cpp
#include "doctest.h"
#include "config/schema.hpp"
#include "translate/dynamic_link.hpp"
#include <algorithm>
#include <string>
#include <vector>

using fpvd::Config;
using fpvd::dynamicLinkArgs;

static bool has(const std::vector<std::string>& a, const std::string& s) {
    return std::find(a.begin(), a.end(), s) != a.end();
}

static std::string pairAfter(const std::vector<std::string>& a,
                              const std::string& flag) {
    auto it = std::find(a.begin(), a.end(), flag);
    if (it == a.end() || std::next(it) == a.end()) return {};
    return *std::next(it);
}

TEST_CASE("translate.dl: argv[0] is /usr/bin/dl-applier") {
    Config c{};
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(a[0] == "/usr/bin/dl-applier");
}

TEST_CASE("translate.dl: defaults map to expected schema-driven flags") {
    Config c{};
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(pairAfter(a, "--health-timeout-ms") == "10000");
    CHECK(pairAfter(a, "--interleaving-supported") == "1");
    CHECK(pairAfter(a, "--debug-enable") == "0");
    CHECK(pairAfter(a, "--min-idr-interval-ms") == "500");
    CHECK(pairAfter(a, "--apply-stagger-ms") == "50");
    CHECK(pairAfter(a, "--apply-sub-pace-ms") == "5");
    CHECK(pairAfter(a, "--mavlink-enable") == "1");
    CHECK(pairAfter(a, "--osd-enable") == "1");
    CHECK(pairAfter(a, "--osd-debug-latency") == "0");
    CHECK(pairAfter(a, "--roi-qp-threshold-kbps") == "6000");
    CHECK(pairAfter(a, "--roi-qp-low-anchor-kbps") == "2000");
    CHECK(pairAfter(a, "--roi-qp-floor") == "-24");
    CHECK(pairAfter(a, "--roi-qp-step") == "3");
    CHECK(pairAfter(a, "--safe-mcs") == "1");
    CHECK(pairAfter(a, "--safe-k") == "8");
    CHECK(pairAfter(a, "--safe-n") == "12");
    CHECK(pairAfter(a, "--safe-depth") == "1");
    CHECK(pairAfter(a, "--safe-bandwidth") == "20");
    CHECK(pairAfter(a, "--safe-tx-power-d-bm") == "20");
    CHECK(pairAfter(a, "--safe-bitrate-kbps") == "2000");
}

TEST_CASE("translate.dl: derived flags come from link/video/iface") {
    Config c{};
    c.link.mtu = 1400;
    c.video.fps = 90;
    auto a = dynamicLinkArgs(c, "wlx00:11:22");
    CHECK(pairAfter(a, "--hello-mtu-bytes") == "1400");
    CHECK(pairAfter(a, "--hello-fps") == "90");
    CHECK(pairAfter(a, "--wlan-dev") == "wlx00:11:22");
}

TEST_CASE("translate.dl: hard-coded operational defaults present") {
    Config c{};
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(pairAfter(a, "--listen-addr") == "0.0.0.0");
    CHECK(pairAfter(a, "--listen-port") == "5800");
    CHECK(pairAfter(a, "--wfb-tx-ctrl-addr") == "127.0.0.1");
    CHECK(pairAfter(a, "--wfb-tx-ctrl-port") == "8000");
    CHECK(pairAfter(a, "--encoder-kind") == "waybeam");
    CHECK(pairAfter(a, "--encoder-host") == "127.0.0.1");
    CHECK(pairAfter(a, "--encoder-port") == "80");
    CHECK(pairAfter(a, "--idr-listen-addr") == "0.0.0.0");
    CHECK(pairAfter(a, "--idr-listen-port") == "11223");
    CHECK(pairAfter(a, "--mavlink-addr") == "127.0.0.1");
    CHECK(pairAfter(a, "--mavlink-port") == "14551");
    CHECK(pairAfter(a, "--osd-msg-path") == "/tmp/MSPOSD.msg");
    CHECK(pairAfter(a, "--osd-update-interval-ms") == "1000");
}

TEST_CASE("translate.dl: schema toggles propagate as 0/1") {
    Config c{};
    c.dynamicLink.interleavingSupported = false;
    c.dynamicLink.debug = true;
    c.dynamicLink.mavlinkEnable = false;
    c.dynamicLink.osd.enabled = false;
    c.dynamicLink.osd.debugLatency = true;
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(pairAfter(a, "--interleaving-supported") == "0");
    CHECK(pairAfter(a, "--debug-enable") == "1");
    CHECK(pairAfter(a, "--mavlink-enable") == "0");
    CHECK(pairAfter(a, "--osd-enable") == "0");
    CHECK(pairAfter(a, "--osd-debug-latency") == "1");
}

TEST_CASE("translate.dl: schema scalars propagate") {
    Config c{};
    c.dynamicLink.healthTimeoutMs = 7000;
    c.dynamicLink.minIdrIntervalMs = 250;
    c.dynamicLink.applyStaggerMs = 0;
    c.dynamicLink.applySubPaceMs = 0;
    c.dynamicLink.roiQp.thresholdKbps = 5000;
    c.dynamicLink.roiQp.lowAnchorKbps = 1500;
    c.dynamicLink.roiQp.floor = -18;
    c.dynamicLink.roiQp.step = 2;
    c.dynamicLink.safe.mcs = 3;
    c.dynamicLink.safe.bitrateKbps = 8000;
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(pairAfter(a, "--health-timeout-ms") == "7000");
    CHECK(pairAfter(a, "--min-idr-interval-ms") == "250");
    CHECK(pairAfter(a, "--apply-stagger-ms") == "0");
    CHECK(pairAfter(a, "--apply-sub-pace-ms") == "0");
    CHECK(pairAfter(a, "--roi-qp-threshold-kbps") == "5000");
    CHECK(pairAfter(a, "--roi-qp-low-anchor-kbps") == "1500");
    CHECK(pairAfter(a, "--roi-qp-floor") == "-18");
    CHECK(pairAfter(a, "--roi-qp-step") == "2");
    CHECK(pairAfter(a, "--safe-mcs") == "3");
    CHECK(pairAfter(a, "--safe-bitrate-kbps") == "8000");
}

TEST_CASE("translate.dl: no --config and no --config-style flag") {
    // Sanity: we drive everything by CLI; no conf file path leaks in.
    Config c{};
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK_FALSE(has(a, "--config"));
    CHECK_FALSE(has(a, "/etc/dynamic-link/drone.conf"));
}
```

- [ ] **Step 3: Add `src/translate/dynamic_link.cpp` and `tests/unit/test_translate_dynamic_link.cpp` to `CMakeLists.txt`** (insert into the same `target_sources` blocks as before).

- [ ] **Step 4: Add an empty stub `src/translate/dynamic_link.cpp` so the build compiles.**

```cpp
#include "translate/dynamic_link.hpp"

namespace fpvd {

std::vector<std::string> dynamicLinkArgs(const Config&, const std::string&) {
    return {};
}

} // namespace fpvd
```

- [ ] **Step 5: Build and run the new tests; verify they fail.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='translate.dl:*' 2>&1 | tail -20`
Expected: tests fail (empty argv).

- [ ] **Step 6: Implement the translator.**

Replace `src/translate/dynamic_link.cpp` contents with:

```cpp
#include "translate/dynamic_link.hpp"
#include <string>

namespace fpvd {

static const char* b01(bool v) { return v ? "1" : "0"; }

std::vector<std::string> dynamicLinkArgs(const Config& c,
                                          const std::string& iface) {
    using std::to_string;
    const auto& dl = c.dynamicLink;
    std::vector<std::string> a = {
        "/usr/bin/dl-applier",

        // listen endpoint for GS decision packets (over wfb-ng tunnel)
        "--listen-addr", "0.0.0.0",
        "--listen-port", "5800",

        // wfb_tx control socket — pinned to match wfb_video_tx's -C 8000
        "--wfb-tx-ctrl-addr", "127.0.0.1",
        "--wfb-tx-ctrl-port", "8000",

        // encoder
        "--encoder-kind", "waybeam",
        "--encoder-host", "127.0.0.1",
        "--encoder-port", "80",

        // IDR-token listener (PixelPilot_rk)
        "--idr-listen-addr", "0.0.0.0",
        "--idr-listen-port", "11223",

        // MAVLink — port pinned to wfb_tlm_tx's listen socket (14551)
        "--mavlink-addr", "127.0.0.1",
        "--mavlink-port", "14551",

        // OSD output path
        "--osd-msg-path", "/tmp/MSPOSD.msg",
        "--osd-update-interval-ms", "1000",

        // schema-driven scalars and toggles
        "--health-timeout-ms", to_string(dl.healthTimeoutMs),
        "--interleaving-supported", b01(dl.interleavingSupported),
        "--debug-enable", b01(dl.debug),
        "--min-idr-interval-ms", to_string(dl.minIdrIntervalMs),
        "--apply-stagger-ms", to_string(dl.applyStaggerMs),
        "--apply-sub-pace-ms", to_string(dl.applySubPaceMs),
        "--mavlink-enable", b01(dl.mavlinkEnable),
        "--osd-enable", b01(dl.osd.enabled),
        "--osd-debug-latency", b01(dl.osd.debugLatency),

        // ROI-QP curve
        "--roi-qp-threshold-kbps", to_string(dl.roiQp.thresholdKbps),
        "--roi-qp-low-anchor-kbps", to_string(dl.roiQp.lowAnchorKbps),
        "--roi-qp-floor", to_string(dl.roiQp.floor),
        "--roi-qp-step", to_string(dl.roiQp.step),

        // per-airframe safe defaults (failsafe-1 fallback)
        "--safe-mcs", to_string(dl.safe.mcs),
        "--safe-k", to_string(dl.safe.k),
        "--safe-n", to_string(dl.safe.n),
        "--safe-depth", to_string(dl.safe.depth),
        "--safe-bandwidth", to_string(dl.safe.bandwidth),
        "--safe-tx-power-d-bm", to_string(dl.safe.txPowerDbm),
        "--safe-bitrate-kbps", to_string(dl.safe.bitrateKbps),

        // derived from existing fpvd schema
        "--hello-mtu-bytes", to_string(c.link.mtu),
        "--hello-fps", to_string(c.video.fps),

        // radio device picked by radio-up.sh
        "--wlan-dev", iface,
    };
    return a;
}

} // namespace fpvd
```

- [ ] **Step 7: Run translator tests; verify they pass.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='translate.dl:*'`
Expected: PASS.

- [ ] **Step 8: Run all tests.**

Run: `./build/fpvd_tests`
Expected: all PASS.

- [ ] **Step 9: Commit.**

```bash
git add src/translate/dynamic_link.hpp src/translate/dynamic_link.cpp \
        tests/unit/test_translate_dynamic_link.cpp CMakeLists.txt
git commit -m "feat(translate): emit dl-applier argv from schema"
```

---

## Task 7: Diff — flag dl_applier on `dynamicLink.*`, `link.mtu`, `video.fps`

**Files:**
- Modify: `src/config/diff.hpp`
- Modify: `src/config/diff.cpp`
- Test: `tests/unit/test_diff.cpp`

- [ ] **Step 1: Write failing tests.**

Append to `tests/unit/test_diff.cpp`:

```cpp
TEST_CASE("diff: dynamicLink.enabled toggle flags DynamicLink") {
    fpvd::Config a{}, b{};
    b.dynamicLink.enabled = true;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK(r.dynamicLink);
}

TEST_CASE("diff: dynamicLink.safe.mcs change flags DynamicLink only") {
    fpvd::Config a{}, b{};
    b.dynamicLink.safe.mcs = 3;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK(r.dynamicLink);
    CHECK_FALSE(r.radio);
    CHECK_FALSE(r.encoder);
    CHECK_FALSE(r.telemetry);
}

TEST_CASE("diff: link.mtu change flags Radio AND DynamicLink") {
    fpvd::Config a{}, b{};
    b.link.mtu = 1400;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK(r.radio);
    CHECK(r.dynamicLink);
}

TEST_CASE("diff: video.fps change flags Encoder AND DynamicLink") {
    fpvd::Config a{}, b{};
    b.video.fps = 90;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK(r.encoder);
    CHECK(r.dynamicLink);
}

TEST_CASE("diff: video.bitrate change does NOT flag DynamicLink") {
    // bitrate is runtime-managed by dl-applier when enabled; baseline
    // changes restart waybeam (encoder) but not dl-applier itself,
    // since hello-fps/hello-mtu didn't move.
    fpvd::Config a{}, b{};
    b.video.bitrate = 10000;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK(r.encoder);
    CHECK_FALSE(r.dynamicLink);
}

TEST_CASE("diff: link.channel change does NOT flag DynamicLink") {
    fpvd::Config a{}, b{};
    b.link.channel = 165;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK(r.radio);
    CHECK_FALSE(r.dynamicLink);
}
```

- [ ] **Step 2: Run; verify they fail to compile (no `dynamicLink` field on `SubsystemDiff`).**

Run: `cmake --build build -j 2>&1 | head -20`
Expected: compile error referencing `dynamicLink` not a member of `SubsystemDiff`.

- [ ] **Step 3: Add the field to `SubsystemDiff`.**

In `src/config/diff.hpp`, change the struct to:

```cpp
struct SubsystemDiff {
    bool radio{false};
    bool encoder{false};
    bool telemetry{false};
    bool dynamicLink{false};
    std::set<std::string> servicesAffected{};
};
```

- [ ] **Step 4: Compute the flag in `diffSubsystems`.**

Replace `src/config/diff.cpp` contents with:

```cpp
#include "config/diff.hpp"
#include <nlohmann/json.hpp>

namespace fpvd {

SubsystemDiff diffSubsystems(const Config& a, const Config& b) {
    using nlohmann::json;
    SubsystemDiff d;
    json ja = a, jb = b;
    if (ja["link"] != jb["link"]) d.radio = true;
    if (ja["video"] != jb["video"] || ja["image"] != jb["image"] ||
        ja["recording"] != jb["recording"] || ja["snapshot"] != jb["snapshot"])
        d.encoder = true;
    if (ja["telemetry"] != jb["telemetry"]) d.telemetry = true;

    // dynamicLink fires when its own subtree changes, OR when a derived
    // input feeding the translator (link.mtu, video.fps) moves.
    if (ja["dynamicLink"] != jb["dynamicLink"]) d.dynamicLink = true;
    if (ja["link"]["mtu"] != jb["link"]["mtu"]) d.dynamicLink = true;
    if (ja["video"]["fps"] != jb["video"]["fps"]) d.dynamicLink = true;

    // services
    for (auto& [name, sa] : a.services) {
        auto it = b.services.find(name);
        if (it == b.services.end()) { d.servicesAffected.insert(name); continue; }
        json jsa = sa, jsb = it->second;
        if (jsa != jsb) d.servicesAffected.insert(name);
    }
    for (auto& [name, sb] : b.services) {
        (void)sb;
        if (!a.services.count(name)) d.servicesAffected.insert(name);
    }
    return d;
}

} // namespace fpvd
```

- [ ] **Step 5: Run new tests; verify they pass.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='diff:*'`
Expected: all diff tests PASS.

- [ ] **Step 6: Run all tests.**

Run: `./build/fpvd_tests`
Expected: all PASS.

- [ ] **Step 7: Commit.**

```bash
git add src/config/diff.hpp src/config/diff.cpp tests/unit/test_diff.cpp
git commit -m "feat(diff): flag dl_applier on dynamicLink/link.mtu/video.fps"
```

---

## Task 8: Orchestrator wiring — seed `dl_applier` when enabled

**Files:**
- Modify: `src/daemon.cpp` (only `seedOrchestrator` and the `restarted` list inside `apply`)
- Test: `tests/integration/test_daemon.cpp`

The orchestrator is rebuilt from scratch on every apply (see existing `apply` at `src/daemon.cpp:134-147`), so add/remove of `dl_applier` is automatic once we condition the `orch_.add(...)` call on `dynamicLink.enabled`. We also want the `restarted` array in the apply response to include `"dl_applier"` when `subs.dynamicLink` fires.

- [ ] **Step 1: Write a failing integration test.**

Append to `tests/integration/test_daemon.cpp` (read the file first to learn its fixture pattern — `makeTestDaemon`-style helpers may already exist; if not, mirror the one in `test_http_handlers.cpp`):

```cpp
TEST_CASE("daemon: enabling dynamicLink seeds dl_applier in orchestrator") {
    auto tmp = fs::temp_directory_path() / "fpvd-daemon-dl-seed";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json",
                  fs::copy_options::overwrite_existing);
    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // dl_applier is NOT present when disabled.
    auto names = d.orchestrator().names();
    CHECK(std::find(names.begin(), names.end(), "dl_applier") == names.end());

    // Enable + apply (without really restarting; we only need the orch
    // re-seeded).
    auto pr = d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})"));
    CHECK(pr.ok);
    auto ar = d.apply(/*reallyRestart=*/false);
    CHECK(ar.ok);

    // Now dl_applier should be in the orchestrator.
    names = d.orchestrator().names();
    CHECK(std::find(names.begin(), names.end(), "dl_applier") != names.end());

    fs::remove_all(tmp);
}

TEST_CASE("daemon: dl_applier restarted-list reflects subs.dynamicLink") {
    auto tmp = fs::temp_directory_path() / "fpvd-daemon-dl-restarted";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json",
                  fs::copy_options::overwrite_existing);
    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"safe":{"mcs":3}}})"));
    auto ar = d.apply(/*reallyRestart=*/false);
    REQUIRE(ar.ok);
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "dl_applier")
          != ar.restarted.end());

    fs::remove_all(tmp);
}
```

Note: if `tests/integration/test_daemon.cpp` does not yet `#include <algorithm>`, add it.

- [ ] **Step 2: Run; verify the two new tests fail.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='daemon: enabling dynamicLink*' -tc='daemon: dl_applier restarted-list*'`
Expected: both fail.

- [ ] **Step 3: Wire `dl_applier` into `seedOrchestrator`.**

In `src/daemon.cpp`, add `#include "translate/dynamic_link.hpp"` near the top (alongside the other `translate/*` includes).

Append to the end of `seedOrchestrator()`, just before the closing `}`:

```cpp
    if (effective_.dynamicLink.enabled) {
        SupervisedSpec dl{};
        dl.name = "dl_applier";
        dl.argv = dynamicLinkArgs(effective_, iface);
        dl.restart = RestartPolicy::Always;
        dl.startAfter = {"wfb_video_tx", "wfb_tun", "waybeam"};
        // Telemetry router participates only if it's present.
        if (effective_.telemetry.router == "msposd") {
            dl.startAfter.push_back("msposd");
        } else if (effective_.telemetry.router == "mavfwd") {
            dl.startAfter.push_back("mavfwd");
        }
        orch_.add(std::move(dl));
    }
```

- [ ] **Step 4: Include `"dl_applier"` in the apply response when `subs.dynamicLink`.**

In `src/daemon.cpp`, inside `apply()`, find the block:

```cpp
        if (subs.radio) restarted.push_back("radio");
        if (subs.encoder) restarted.push_back("encoder");
        if (subs.telemetry) restarted.push_back("telemetry");
        for (auto& n : subs.servicesAffected) restarted.push_back(n);
```

Add one more line, after the `telemetry` line:

```cpp
        if (subs.dynamicLink) restarted.push_back("dl_applier");
```

- [ ] **Step 5: One subtle thing — `apply()` currently only populates `restarted[]` when `reallyRestart` is true** (the block above lives under that branch). The second new test calls `apply(false)`. Move the `restarted.push_back(...)` block out of the `if (reallyRestart)` branch, since the diff itself is independent of whether we actually restarted.

In `src/daemon.cpp`, restructure the `apply` tail to:

```cpp
    std::vector<std::string> restarted;
    if (subs.radio) restarted.push_back("radio");
    if (subs.encoder) restarted.push_back("encoder");
    if (subs.telemetry) restarted.push_back("telemetry");
    if (subs.dynamicLink) restarted.push_back("dl_applier");
    for (auto& n : subs.servicesAffected) restarted.push_back(n);

    if (reallyRestart) {
        // Subsystem-level restart: rebuild orchestrator (simple v1).
        orch_.stopAll();
        orch_ = Orchestrator{};
        // Re-run radio bring-up if link changed.
        if (subs.radio) {
            auto rr = bringUpRadio(paths_.radioUpScript, effective_);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, {},
                              std::string("radio: ") + rr.stderrText};
                return {false, {}, {}, rr.stderrText, version_};
            }
            radio_ = {rr.driver, rr.iface, rr.adapterId};
        }
        seedOrchestrator();
        orch_.startAll();
    }
    version_++;
    lastApply_ = {nowIso(), true, restarted, std::nullopt};
    return {true, {}, restarted, std::nullopt, version_};
```

Important: when `reallyRestart=false`, the orchestrator isn't rebuilt, so the first new test (which checks `names()` after `apply(false)`) needs the orchestrator re-seed regardless. Add a small fallback: still rebuild the orchestrator on `apply(false)` so `seedOrchestrator()` re-reads `effective_`.

Update the block above to:

```cpp
    if (reallyRestart) {
        orch_.stopAll();
        orch_ = Orchestrator{};
        if (subs.radio) {
            auto rr = bringUpRadio(paths_.radioUpScript, effective_);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, {},
                              std::string("radio: ") + rr.stderrText};
                return {false, {}, {}, rr.stderrText, version_};
            }
            radio_ = {rr.driver, rr.iface, rr.adapterId};
        }
        seedOrchestrator();
        orch_.startAll();
    } else {
        // Re-seed the orchestrator's specs without touching real processes,
        // so introspection (Orchestrator::names()) reflects the new config.
        orch_ = Orchestrator{};
        seedOrchestrator();
    }
```

- [ ] **Step 6: Run new tests; verify they pass.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='daemon: enabling dynamicLink*' -tc='daemon: dl_applier restarted-list*'`
Expected: PASS.

- [ ] **Step 7: Run all tests.**

Run: `./build/fpvd_tests`
Expected: all PASS. Pay attention to the existing `test_daemon.cpp` and `test_http_handlers.cpp` cases that exercise `apply(false)` — the new re-seed behavior could perturb pre-existing assertions about orchestrator state. If any fail, read the failing assertion carefully; the fix is usually in the test (it was asserting on stale orchestrator state) rather than in the daemon.

- [ ] **Step 8: Commit.**

```bash
git add src/daemon.cpp tests/integration/test_daemon.cpp
git commit -m "feat(orchestrator): supervise dl_applier when dynamicLink enabled"
```

---

## Task 9: dl-applier CLI assumptions check

**Files:**
- Create: `tests/fixtures/dl_applier_help.txt`
- Create: `tests/unit/test_dl_applier_cli_assumptions.cpp`
- Modify: `CMakeLists.txt`

This task pins our CLI assumptions to a vendored copy of `dl-applier --help`. If `wfbng-dynamic-link` renames a flag we depend on, this test fails at build time.

- [ ] **Step 1: Capture the current `dl-applier --help` output.**

Run on a machine where the binary is available (e.g. via SSH to a dev drone, or against a host-compiled build of `wfbng-dynamic-link/drone`):

```bash
dl-applier --help > /tmp/dl_applier_help.txt 2>&1
```

Copy the file to `tests/fixtures/dl_applier_help.txt`. If you cannot reach a real binary, ask the user to paste the output; do **not** fabricate it.

- [ ] **Step 2: Add the test.**

Create `tests/unit/test_dl_applier_cli_assumptions.cpp`:

```cpp
#include "doctest.h"
#include <fstream>
#include <sstream>
#include <string>

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    std::stringstream b;
    b << f.rdbuf();
    return b.str();
}

TEST_CASE("dl-applier --help references every flag the translator emits") {
    auto help = readFile("tests/fixtures/dl_applier_help.txt");
    REQUIRE_FALSE(help.empty()); // fixture must exist

    // Schema-driven flags
    const std::vector<std::string> required = {
        "--listen-addr", "--listen-port",
        "--wfb-tx-ctrl-addr", "--wfb-tx-ctrl-port",
        "--encoder-kind", "--encoder-host", "--encoder-port",
        "--idr-listen-addr", "--idr-listen-port",
        "--mavlink-addr", "--mavlink-port", "--mavlink-enable",
        "--osd-msg-path", "--osd-update-interval-ms",
        "--osd-enable", "--osd-debug-latency",
        "--health-timeout-ms",
        "--interleaving-supported",
        "--debug-enable",
        "--min-idr-interval-ms",
        "--apply-stagger-ms", "--apply-sub-pace-ms",
        "--roi-qp-threshold-kbps", "--roi-qp-low-anchor-kbps",
        "--roi-qp-floor", "--roi-qp-step",
        "--safe-mcs", "--safe-k", "--safe-n", "--safe-depth",
        "--safe-bandwidth", "--safe-tx-power-d-bm",
        "--safe-bitrate-kbps",
        "--hello-mtu-bytes", "--hello-fps",
        "--wlan-dev",
    };
    for (auto& f : required) {
        INFO("flag not found in dl-applier --help: " << f);
        CHECK(help.find(f) != std::string::npos);
    }
}
```

- [ ] **Step 3: Add the test file to `CMakeLists.txt`.**

Append `tests/unit/test_dl_applier_cli_assumptions.cpp` to the `target_sources(fpvd_tests PRIVATE ...)` block.

- [ ] **Step 4: Run.**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='dl-applier --help*'`
Expected: PASS. If any flag is missing, **stop**: the spec assumes a name (`--safe-tx-power-d-bm` is the touchy one — the kebab-cased form of `safe_tx_power_dBm`) that the binary doesn't actually accept. In that case, find the correct flag spelling in the help text and update Task 6's translator to match before continuing.

- [ ] **Step 5: Run all tests.**

Run: `./build/fpvd_tests`
Expected: all PASS.

- [ ] **Step 6: Commit.**

```bash
git add tests/fixtures/dl_applier_help.txt \
        tests/unit/test_dl_applier_cli_assumptions.cpp CMakeLists.txt
git commit -m "test: pin dl-applier CLI surface against vendored --help"
```

---

## Task 10: End-to-end smoke — `dl_applier` shows up in `/status`

**Files:**
- Modify: `tests/integration/test_http_handlers.cpp`

This is the headline integration test. It exercises the full chain: PATCH → apply → orchestrator seeded → `/status` lists `dl_applier`.

- [ ] **Step 1: Write the failing test.**

Append to `tests/integration/test_http_handlers.cpp`:

```cpp
TEST_CASE("handlers: enabling dynamicLink surfaces dl_applier in /status") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-dl-e2e";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, /*reallyRestart=*/false);
    srv.listenInBackground("127.0.0.1", 18096);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18096");

    // Before enabling: dl_applier not in /status.processes.
    auto s0 = c.Get("/status");
    REQUIRE(s0); CHECK(s0->status == 200);
    auto j0 = nlohmann::json::parse(s0->body);
    bool found0 = false;
    for (auto& p : j0["processes"]) if (p["name"] == "dl_applier") found0 = true;
    CHECK_FALSE(found0);

    // PATCH + apply.
    c.Patch("/config", R"({"dynamicLink":{"enabled":true}})",
            "application/json");
    auto ap = c.Post("/apply", "", "application/json");
    REQUIRE(ap); CHECK(ap->status == 200);
    auto japp = nlohmann::json::parse(ap->body);
    CHECK(japp["applied"] == true);
    bool restartedDl = false;
    for (auto& r : japp["restarted"])
        if (r == "dl_applier") restartedDl = true;
    CHECK(restartedDl);

    // After: dl_applier visible.
    auto s1 = c.Get("/status");
    auto j1 = nlohmann::json::parse(s1->body);
    bool found1 = false;
    for (auto& p : j1["processes"]) if (p["name"] == "dl_applier") found1 = true;
    CHECK(found1);

    // Flip back off + apply — dl_applier disappears.
    c.Patch("/config", R"({"dynamicLink":{"enabled":false}})",
            "application/json");
    c.Post("/apply", "", "application/json");
    auto s2 = c.Get("/status");
    auto j2 = nlohmann::json::parse(s2->body);
    bool found2 = false;
    for (auto& p : j2["processes"]) if (p["name"] == "dl_applier") found2 = true;
    CHECK_FALSE(found2);

    srv.stop(); fs::remove_all(tmp);
}
```

- [ ] **Step 2: Run; verify it passes (everything is in place from Tasks 1–8).**

Run: `cmake --build build -j && ./build/fpvd_tests -tc='handlers: enabling dynamicLink surfaces dl_applier*'`
Expected: PASS. If it fails, the failure points at the integration gap (typically: `apply(false)` doesn't re-seed the orchestrator, which was supposed to be handled in Task 8 Step 5).

- [ ] **Step 3: Run all tests.**

Run: `./build/fpvd_tests`
Expected: all PASS.

- [ ] **Step 4: Commit.**

```bash
git add tests/integration/test_http_handlers.cpp
git commit -m "test(e2e): dl_applier appears in /status when enabled"
```

---

## Task 11: Documentation — update `README.md`

**Files:**
- Modify: `README.md`

- [ ] **Step 1: Add a short section to the README describing the new domain section.**

After the existing API table in `README.md`, append:

```markdown
## Adaptive link (`dl-applier`)

Set `dynamicLink.enabled = true` to have fpvd supervise the drone-side
of `wfbng-dynamic-link` (`/usr/bin/dl-applier`). Configuration is
driven entirely by fpvd — no `/etc/dynamic-link/drone.conf` is read.

When enabled, these fields become read-only via the API
(`PATCH /config` returns `400 dynamic_link_locked`) because
`dl-applier` mutates them at runtime: `link.mcs`, `link.txpower`,
`link.fec`, `link.width`, `video.bitrate`, `video.qpDelta`,
`video.roi`. To edit a baseline, disable `dynamicLink.enabled`,
PATCH the field, then re-enable.

Per-airframe failsafe ceilings live under `dynamicLink.safe` and the
ROI-QP curve under `dynamicLink.roiQp`. See the design spec at
`docs/superpowers/specs/2026-05-27-dynamic-link-design.md` for the
full list and the lock-rule semantics.
```

- [ ] **Step 2: Commit.**

```bash
git add README.md
git commit -m "docs: dynamicLink integration overview"
```

---

## Verification — final pass

- [ ] **Step 1: Full test run.**

Run: `cmake --build build -j && ./build/fpvd_tests 2>&1 | tail -10`
Expected: all PASS, including the 11 new test cases across schema/validate/lock/translate/diff/daemon/handlers/cli-assumptions.

- [ ] **Step 2: Manual smoke against a fake dl-applier.**

Create a fake binary that exits cleanly:

```bash
cat > /tmp/dl-applier <<'EOF'
#!/bin/sh
exec sleep 9999
EOF
chmod +x /tmp/dl-applier
sudo cp /tmp/dl-applier /usr/bin/dl-applier   # only if you're on the drone
```

On host: run fpvd with the dev defaults and exercise the new flow:

```bash
./build/fpvd --defaults etc/defaults.json \
             --overlay /tmp/fpvd-overlay.json \
             --radio-up /bin/true \
             --waybeam-json /tmp/fpvd-waybeam.json \
             --port 8080 &
curl -sX PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"dynamicLink":{"enabled":true}}'
curl -sX POST  http://127.0.0.1:8080/apply
curl -s  http://127.0.0.1:8080/status | jq '.processes[] | select(.name=="dl_applier")'
```

Expected: the last `curl` prints a JSON object with `"state": "running"` (or `"failed"` if `/usr/bin/dl-applier` doesn't exist on the host — that's fine, it just proves the orchestrator is trying).

- [ ] **Step 3: Confirm the dl-applier process actually gets the right argv.**

```bash
cat /proc/$(pidof dl-applier)/cmdline | tr '\0' ' '
```

Expected: the full flag list emitted by the translator, with `--hello-mtu-bytes 1500 --hello-fps 60 --wlan-dev wlan0` derived from the defaults.

- [ ] **Step 4: Tear down.**

```bash
pkill fpvd
rm -f /tmp/fpvd-overlay.json /tmp/fpvd-waybeam.json /tmp/dl-applier
```
