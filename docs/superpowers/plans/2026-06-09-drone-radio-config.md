# Drone Radio Config Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** On the drone fpvd (C++), rename the adaptive-link `safe` config key to `failsafe`, switch the radio TX power to dBm, make the per-MCS TX power curve config-driven (per-radio default + operator override), and publish the resolved curve in `/status.radio`.

**Architecture:** Drone-local changes only — the drone daemon stays standalone. Config flows `defaults.json` + sparse overlay → `Config` struct (`schema.hpp`) → validation (`validate.cpp`) → runtime snapshot (`DlRuntimeConfig` via `buildDlSnapshot`) consumed by the adaptive-link controller. The per-MCS curve, today a `constexpr` global, becomes a resolved `std::array<int8_t,8>` threaded through `DlRuntimeConfig` and surfaced in `/status`.

**Tech Stack:** C++17, `nlohmann/json` (schema via `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT`), doctest tests. POSIX shell for the radio scripts.

**Build & test:** From `drone/`: `cmake -S . -B build && cmake --build build -j` then run `./build/fpvd_tests` (doctest binary — **not** `ctest`). Filter a single test with `./build/fpvd_tests --test-case="<name>"`. New `tests/*.cpp` files MUST be added to the `target_sources(fpvd_tests …)` list in `drone/CMakeLists.txt` or they won't compile in.

**Spec:** `docs/superpowers/specs/2026-06-09-unified-config-design.md` (sections: Type 2, Type 3, Drone per-MCS TX power curve, `safe`→`failsafe`).

---

## File map

| File | Change |
|---|---|
| `drone/src/config/schema.hpp` | rename `DynamicLink.safe`→`failsafe` key; add `Link.txpowerCurve` optional field |
| `drone/src/config/validate.cpp` | `link.txpower` range 1..63→0..30; `dynamicLink.safe.*`→`failsafe.*` paths; validate `link.txpowerCurve` |
| `drone/src/config/store.cpp` | legacy-`safe` overlay migration shim |
| `drone/etc/defaults.json` | `dynamicLink.safe`→`failsafe`; `link.txpower` 1→20; `link.txpowerCurve: null` |
| `drone/src/dynlink/txpower_curve.hpp` / `.cpp` | curve type, per-radio registry, resolve(+source), MCS lookup taking a curve |
| `drone/src/dynlink/runtime_config.hpp` / `.cpp` | add `txPowerCurve` to `DlRuntimeConfig`; map `dl.failsafe`; resolve curve in `buildDlSnapshot` |
| `drone/src/dynlink/local_compute.cpp` | `txpowerDbmForMcs(cfg.txPowerCurve, d.mcs)` |
| `drone/src/dynlink/controller.cpp` | `txpowerDbmForMcs(cfg.txPowerCurve, cfg.safe.mcs)` |
| `drone/src/status.cpp` | add `radio.txpowerCurve` + `radio.txpowerCurveSource` |
| `drone/scripts/radio-up.sh`, `radio-tune.sh` | txpower → `dBm * 100` (mBm) for all drivers |
| `drone/CMakeLists.txt` | register the new test file(s) |
| `drone/tests/...` | new + updated tests |

> **Internal symbols stay:** the runtime struct `SafeDefaults` and the `DlRuntimeConfig.safe` field keep their names (they read fine as "the failsafe values"). Only the JSON config **key** and the `Link`/`DynamicLink` schema fields change.

---

## Task 1: Rename `safe` → `failsafe` config key

**Files:**
- Modify: `drone/src/config/schema.hpp:154-172`
- Modify: `drone/src/config/validate.cpp:119-133`
- Modify: `drone/src/dynlink/runtime_config.cpp:30-38`
- Modify: `drone/etc/defaults.json`
- Modify: `drone/tests/unit/test_validate.cpp`, `drone/tests/unit/test_schema.cpp`

- [ ] **Step 1: Write the failing test** — add to `drone/tests/unit/test_schema.cpp`:

```cpp
TEST_CASE("DynamicLink parses the failsafe key (renamed from safe)") {
    auto j = nlohmann::json::parse(R"({
        "dynamicLink": {"failsafe": {"mcs": 3, "bitrateKbps": 5000}}
    })");
    auto c = j.get<fpvd::Config>();
    CHECK(c.dynamicLink.failsafe.mcs == 3);
    CHECK(c.dynamicLink.failsafe.bitrateKbps == 5000);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="DynamicLink parses the failsafe key*"`
Expected: FAIL to compile (`failsafe` is not a member of `DynamicLink`).

- [ ] **Step 3: Rename the field in `schema.hpp`**

In `drone/src/config/schema.hpp`, struct `DynamicLink` (line ~163) change `DynamicLinkSafe safe{};` to `DynamicLinkSafe failsafe{};`, and in the macro (line ~167-172) change the member `safe` to `failsafe`:

```cpp
struct DynamicLink {
    bool enabled{false};
    int healthTimeoutMs{10000};
    bool interleavingSupported{true};
    int minIdrIntervalMs{500};
    int applyStaggerMs{50};
    int applySubPaceMs{5};
    DynamicLinkOsd osd{};
    DynamicLinkRoiQp roiQp{};
    DynamicLinkSafe failsafe{};
    DynamicLinkBitrate bitrate{};
    DynamicLinkFec     fec{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLink, enabled,
                                               healthTimeoutMs,
                                               interleavingSupported,
                                               minIdrIntervalMs, applyStaggerMs,
                                               applySubPaceMs,
                                               osd, roiQp, failsafe, bitrate, fec)
```

- [ ] **Step 4: Update `validate.cpp`**

In `drone/src/config/validate.cpp` (lines 119-133), replace every `dl.safe` with `dl.failsafe` and every error path `"dynamicLink.safe.…"` with `"dynamicLink.failsafe.…"`:

```cpp
        if (dl.failsafe.mcs < 0 || dl.failsafe.mcs > 7)
            errs.push_back({"dynamicLink.failsafe.mcs", "must be 0..7"});
        if (dl.failsafe.k < 1 || dl.failsafe.k > 32 ||
            dl.failsafe.n < 1 || dl.failsafe.n > 32 ||
            dl.failsafe.k >= dl.failsafe.n)
            errs.push_back({"dynamicLink.failsafe.fec", "require 1<=k<n<=32"});
        if (dl.failsafe.depth < 1 || dl.failsafe.depth > 8)
            errs.push_back({"dynamicLink.failsafe.depth", "must be 1..8"});
        if (dl.failsafe.bandwidth != 10 && dl.failsafe.bandwidth != 20 &&
            dl.failsafe.bandwidth != 40)
            errs.push_back({"dynamicLink.failsafe.bandwidth", "must be 10, 20, or 40"});
        if (dl.failsafe.txPowerDbm < -10 || dl.failsafe.txPowerDbm > 30)
            errs.push_back({"dynamicLink.failsafe.txPowerDbm", "must be -10..30"});
        if (dl.failsafe.bitrateKbps <= 0)
            errs.push_back({"dynamicLink.failsafe.bitrateKbps", "must be > 0"});
```

- [ ] **Step 5: Update `runtime_config.cpp`**

In `drone/src/dynlink/runtime_config.cpp:30-38`, change the `dl.safe.*` reads (the `s.safe = SafeDefaults{…}` field name stays):

```cpp
    s.safe = SafeDefaults{
        static_cast<uint8_t> (dl.failsafe.mcs),
        static_cast<uint8_t> (dl.failsafe.k),
        static_cast<uint8_t> (dl.failsafe.n),
        static_cast<uint8_t> (dl.failsafe.depth),
        static_cast<uint8_t> (dl.failsafe.bandwidth),
        static_cast<int8_t>  (dl.failsafe.txPowerDbm),
        static_cast<uint16_t>(dl.failsafe.bitrateKbps),
    };
```

- [ ] **Step 6: Update `defaults.json`**

In `drone/etc/defaults.json`, rename the `dynamicLink.safe` object key to `failsafe` (values unchanged):

```json
    "failsafe": {
      "mcs": 1, "k": 8, "n": 12, "depth": 1,
      "bandwidth": 20, "txPowerDbm": 20, "bitrateKbps": 2000
    }
```

- [ ] **Step 7: Update any existing `safe`-key references in tests**

Run: `cd drone && grep -rn '"safe"\|\.safe\b\|dynamicLink.safe' tests/unit/test_validate.cpp tests/unit/test_schema.cpp tests/unit/test_dl_runtime_config.cpp`
For each hit that refers to the **config key** (`dl.safe`, `"safe"` in a JSON literal, `dynamicLink.safe.*` error path), rename to `failsafe`. Do **not** touch references to the runtime `DlRuntimeConfig.safe` / `SafeDefaults` (those keep their name).

- [ ] **Step 8: Run tests to verify they pass**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS (all, including the new failsafe test).

- [ ] **Step 9: Commit**

```bash
git add drone/src/config/schema.hpp drone/src/config/validate.cpp \
        drone/src/dynlink/runtime_config.cpp drone/etc/defaults.json \
        drone/tests/unit/test_schema.cpp drone/tests/unit/test_validate.cpp \
        drone/tests/unit/test_dl_runtime_config.cpp
git commit -m "drone: rename dynamicLink.safe config key to failsafe"
```

---

## Task 2: Legacy `safe` overlay migration shim

A deployed drone's persisted overlay (`/etc/fpvd/config.json`) may still contain `dynamicLink.safe`. Without a shim it would silently fall back to defaults — bad for a failsafe. Migrate the key at load time.

**Files:**
- Modify: `drone/src/config/store.cpp:50-72` (`loadEffective`)
- Test: `drone/tests/unit/test_store.cpp`

- [ ] **Step 1: Write the failing test** — add to `drone/tests/unit/test_store.cpp`:

```cpp
TEST_CASE("loadEffective migrates a legacy dynamicLink.safe overlay key to failsafe") {
    auto dir = std::filesystem::temp_directory_path() / "fpvd-safe-migrate";
    std::filesystem::create_directories(dir);
    auto defaults = dir / "defaults.json";
    auto overlay  = dir / "config.json";
    { std::ofstream f(defaults); f << R"({"dynamicLink":{"failsafe":{"mcs":1}}})"; }
    { std::ofstream f(overlay);  f << R"({"dynamicLink":{"safe":{"mcs":4}}})"; }
    auto c = fpvd::loadEffective(defaults.string(), overlay.string());
    CHECK(c.dynamicLink.failsafe.mcs == 4);   // legacy key honored, not dropped
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="loadEffective migrates*"`
Expected: FAIL — `failsafe.mcs == 1` (default), because the legacy `safe` key was ignored.

- [ ] **Step 3: Add the migration in `loadEffective`**

In `drone/src/config/store.cpp`, inside `loadEffective`, after `overlayJ` is parsed (right before `auto merged = deepMergeJson(...)` at line ~67), insert:

```cpp
    // Back-compat: the adaptive-link failsafe key was renamed safe -> failsafe.
    // Migrate a legacy overlay so a deployed drone's failsafe is preserved.
    if (overlayJ.is_object() && overlayJ.contains("dynamicLink") &&
        overlayJ["dynamicLink"].is_object()) {
        auto& dlj = overlayJ["dynamicLink"];
        if (dlj.contains("safe") && !dlj.contains("failsafe")) {
            dlj["failsafe"] = dlj["safe"];
            dlj.erase("safe");
        }
    }
```

- [ ] **Step 4: Run test to verify it passes**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="loadEffective migrates*"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add drone/src/config/store.cpp drone/tests/unit/test_store.cpp
git commit -m "drone: migrate legacy dynamicLink.safe overlay key to failsafe"
```

---

## Task 3: Radio TX power → dBm

Switch `link.txpower` from driver units (1..63) to dBm (0..30) and make the radio scripts convert dBm→mBm (`* 100`), matching the already-deployed dynamic path (`radio_txpower.cpp` runs `iw set txpower fixed <dBm*100>`).

> **Hardware note:** the deployment radio is `8812eu` (BL-M8812EU2), whose old static scaling was `units * 50` mBm — identical to `dBm * 100` when `units = dBm*2`, so this is a clean change for production. The legacy `88XXau` branch used `units * -100` (a negative-mBm driver quirk); this plan unifies it to `dBm * 100`. **Validate on 88XXau hardware before shipping to those airframes**; the deployment fleet is unaffected.

**Files:**
- Modify: `drone/src/config/schema.hpp:50` (`Link.txpower` default)
- Modify: `drone/src/config/validate.cpp:56-57`
- Modify: `drone/etc/defaults.json` (`link.txpower`)
- Modify: `drone/scripts/radio-up.sh:47-51`, `drone/scripts/radio-tune.sh:21-27`
- Test: `drone/tests/unit/test_validate.cpp`, `drone/tests/integration/test_radio_tune_script.cpp`

- [ ] **Step 1: Write the failing validation test** — add to `drone/tests/unit/test_validate.cpp`:

```cpp
TEST_CASE("link.txpower is validated as dBm 0..30") {
    fpvd::Config c{};
    c.link.txpower = 20;                       // valid dBm
    CHECK(fpvd::validate(c).empty());
    c.link.txpower = 31;                       // above radio max
    auto errs = fpvd::validate(c);
    CHECK(std::any_of(errs.begin(), errs.end(),
        [](const fpvd::ValidationError& e){ return e.path == "link.txpower"; }));
}
```

(If `<algorithm>` isn't already included in the test file, add `#include <algorithm>` at the top.)

- [ ] **Step 2: Run test to verify it fails**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="link.txpower is validated*"`
Expected: FAIL — with the old rule `1..63`, `txpower=20` passes but `31` is still ≤63 so no error is produced.

- [ ] **Step 3: Update the validation range in `validate.cpp:56-57`**

```cpp
    if (c.link.txpower < 0 || c.link.txpower > 30)
        errs.push_back({"link.txpower", "must be 0..30 (dBm)"});
```

- [ ] **Step 4: Update the schema default and defaults.json**

In `drone/src/config/schema.hpp:50` change `int txpower{1};` to `int txpower{20};`.
In `drone/etc/defaults.json` change `"txpower": 1` to `"txpower": 20`.

- [ ] **Step 5: Run validation test to verify it passes**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="link.txpower is validated*"`
Expected: PASS.

- [ ] **Step 6: Update the radio scripts**

In `drone/scripts/radio-up.sh` replace lines 47-51 with a single dBm→mBm conversion:

```sh
# txpower is dBm; iw expects mBm (milli-dBm) = dBm * 100.
iw $WLAN_DEV set txpower fixed $(( ${FPVD_TXPOWER:-20} * 100 ))
```

In `drone/scripts/radio-tune.sh` replace the `txpower)` case body (lines 22-26) with:

```sh
    txpower)
        # txpower is dBm; iw expects mBm = dBm * 100.
        iw "$iface" set txpower fixed $(( ${FPVD_TXPOWER:-20} * 100 ))
        ;;
```

- [ ] **Step 7: Update the script integration test**

In `drone/tests/integration/test_radio_tune_script.cpp`, replace the `TEST_CASE("radio-tune.sh: txpower scaling sign per driver")` body so both drivers now emit `dBm * 100`:

```cpp
TEST_CASE("radio-tune.sh: txpower is dBm rendered as mBm (dBm*100)") {
    auto tmp = fs::temp_directory_path() / "fpvd-rt-txpower";
    fs::remove_all(tmp);
    auto rec = setupStubs(tmp);

    fpvd::Config c{};
    c.link.txpower = 20;   // dBm

    auto r1 = fpvd::tuneRadio("scripts/radio-tune.sh", "txpower", c, "wlan0", "88XXau");
    REQUIRE(r1.ok);
    CHECK(readAllText(rec).find("iw wlan0 set txpower fixed 2000") != std::string::npos);

    fs::remove(rec);
    auto r2 = fpvd::tuneRadio("scripts/radio-tune.sh", "txpower", c, "wlan0", "8812eu");
    REQUIRE(r2.ok);
    CHECK(readAllText(rec).find("iw wlan0 set txpower fixed 2000") != std::string::npos);
}
```

- [ ] **Step 8: Run tests to verify they pass**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS (validation + script tests).

- [ ] **Step 9: Commit**

```bash
git add drone/src/config/schema.hpp drone/src/config/validate.cpp \
        drone/etc/defaults.json drone/scripts/radio-up.sh drone/scripts/radio-tune.sh \
        drone/tests/unit/test_validate.cpp drone/tests/integration/test_radio_tune_script.cpp
git commit -m "drone: switch link.txpower to dBm (iw mBm = dBm*100)"
```

---

## Task 4: Per-MCS curve type, per-radio registry, and resolver

Replace the bare `constexpr kTxPowerDbmByMcs` with a curve type, a per-radio default registry, and a resolver that picks override → radio-default → fallback and reports the source.

**Files:**
- Rewrite: `drone/src/dynlink/txpower_curve.hpp`, `drone/src/dynlink/txpower_curve.cpp`
- Create: `drone/tests/unit/test_dl_txpower_resolve.cpp`
- Modify: `drone/CMakeLists.txt` (register the new test)
- Modify: `drone/tests/unit/test_dl_txpower_curve.cpp` (update to the new signature)

- [ ] **Step 1: Write the failing test** — create `drone/tests/unit/test_dl_txpower_resolve.cpp`:

```cpp
/* test_dl_txpower_resolve.cpp — per-radio default registry + override resolver. */
#include "doctest.h"
#include "dynlink/txpower_curve.hpp"
using namespace fpvd::dynlink;

TEST_CASE("resolveTxpowerCurve: bl-m8812eu2 default when no override") {
    auto r = resolveTxpowerCurve(std::nullopt, std::string("bl-m8812eu2"), "8812eu");
    CHECK(r.source == "bl-m8812eu2");
    CHECK(r.curve == TxPowerCurve{29,28,25,23,19,19,19,19});
}

TEST_CASE("resolveTxpowerCurve: explicit override wins and reports source override") {
    std::vector<int> ov{10,10,10,10,10,10,10,10};
    auto r = resolveTxpowerCurve(ov, std::string("bl-m8812eu2"), "8812eu");
    CHECK(r.source == "override");
    CHECK(r.curve == TxPowerCurve{10,10,10,10,10,10,10,10});
}

TEST_CASE("resolveTxpowerCurve: unknown radio falls back") {
    auto r = resolveTxpowerCurve(std::nullopt, std::nullopt, "8733bu");
    CHECK(r.source == "fallback");
    CHECK(r.curve[7] <= 20);   // conservative tail
}

TEST_CASE("txpowerDbmForMcs clamps mcs into the curve") {
    TxPowerCurve cv{29,28,25,23,19,19,19,19};
    CHECK(txpowerDbmForMcs(cv, -1) == 29);
    CHECK(txpowerDbmForMcs(cv, 3)  == 23);
    CHECK(txpowerDbmForMcs(cv, 99) == 19);
}
```

- [ ] **Step 2: Register the new test in CMake**

In `drone/CMakeLists.txt`, add to the `target_sources(fpvd_tests PRIVATE …)` list (next to `test_dl_txpower_curve.cpp`):

```cmake
        tests/unit/test_dl_txpower_resolve.cpp
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd drone && cmake -S . -B build && cmake --build build -j`
Expected: FAIL to compile — `TxPowerCurve`, `resolveTxpowerCurve`, and the 2-arg `txpowerDbmForMcs` don't exist yet.

- [ ] **Step 4: Rewrite `txpower_curve.hpp`**

```cpp
#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fpvd::dynlink {

using TxPowerCurve = std::array<int8_t, 8>;   // dBm per MCS 0..7

// bl-m8812eu2 default (OpenIPC adaptive-link level-4 column). Full power at low
// MCS for range; backed off on the high-PAPR 64-QAM rungs so the PA stays linear.
inline constexpr TxPowerCurve kCurveM8812eu2 = { 29, 28, 25, 23, 19, 19, 19, 19 };

// Conservative default for radios we have not characterized: modest, flat-ish,
// backed off at the top so an unknown PA is never overdriven.
inline constexpr TxPowerCurve kCurveFallback = { 22, 22, 22, 20, 19, 19, 19, 19 };

struct ResolvedCurve {
    TxPowerCurve curve;
    std::string  source;   // "override" | "<radio>" | "fallback"
};

// Per-radio default, keyed by adapterId then driver; kCurveFallback otherwise.
ResolvedCurve defaultTxpowerCurve(const std::optional<std::string>& adapterId,
                                  const std::string& driver);

// Effective curve: a present, 8-long override wins (source "override"); else the
// per-radio default. A malformed override (wrong length) is ignored (default used).
ResolvedCurve resolveTxpowerCurve(const std::optional<std::vector<int>>& override_,
                                  const std::optional<std::string>& adapterId,
                                  const std::string& driver);

// dBm for the given MCS, clamping mcs to [0,7].
int8_t txpowerDbmForMcs(const TxPowerCurve& curve, int mcs);

// Legacy 1-arg form (uses the bl-m8812eu2 default curve). Kept so the existing
// call sites still compile until Task 5 switches them; removed there.
int8_t txpowerDbmForMcs(int mcs);

} // namespace fpvd::dynlink
```

- [ ] **Step 5: Rewrite `txpower_curve.cpp`**

```cpp
/* txpower_curve.cpp — per-MCS tx power: per-radio defaults + override resolver. */
#include "dynlink/txpower_curve.hpp"

namespace fpvd::dynlink {

ResolvedCurve defaultTxpowerCurve(const std::optional<std::string>& adapterId,
                                  const std::string& driver) {
    if (adapterId && *adapterId == "bl-m8812eu2")
        return {kCurveM8812eu2, "bl-m8812eu2"};
    if (driver == "8812eu")
        return {kCurveM8812eu2, "bl-m8812eu2"};
    return {kCurveFallback, "fallback"};
}

ResolvedCurve resolveTxpowerCurve(const std::optional<std::vector<int>>& override_,
                                  const std::optional<std::string>& adapterId,
                                  const std::string& driver) {
    if (override_ && override_->size() == 8) {
        TxPowerCurve cv{};
        for (size_t i = 0; i < 8; ++i)
            cv[i] = static_cast<int8_t>((*override_)[i]);
        return {cv, "override"};
    }
    return defaultTxpowerCurve(adapterId, driver);
}

int8_t txpowerDbmForMcs(const TxPowerCurve& curve, int mcs) {
    if (mcs < 0) mcs = 0;
    if (mcs > 7) mcs = 7;
    return curve[static_cast<size_t>(mcs)];
}

int8_t txpowerDbmForMcs(int mcs) {        // legacy 1-arg — removed in Task 5
    return txpowerDbmForMcs(kCurveM8812eu2, mcs);
}

} // namespace fpvd::dynlink
```

- [ ] **Step 6: Update the old curve test to the new signature**

Replace the body of `drone/tests/unit/test_dl_txpower_curve.cpp` so it exercises the constant + 2-arg lookup:

```cpp
/* test_dl_txpower_curve.cpp — bl-m8812eu2 level-4 default curve constant. */
#include "doctest.h"
#include "dynlink/txpower_curve.hpp"
using namespace fpvd::dynlink;

TEST_CASE("kCurveM8812eu2 is the bl-m8812eu2 level-4 curve") {
    CHECK(kCurveM8812eu2 == TxPowerCurve{29,28,25,23,19,19,19,19});
    CHECK(txpowerDbmForMcs(kCurveM8812eu2, 0) == 29);
    CHECK(txpowerDbmForMcs(kCurveM8812eu2, 4) == 19);
}
```

- [ ] **Step 7: Run tests to verify they pass**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS (all). The legacy 1-arg `txpowerDbmForMcs(int)` keeps `local_compute.cpp` / `controller.cpp` compiling; the full build stays green. (Those call sites switch to the 2-arg form in Task 5, which then removes the legacy overload.)

- [ ] **Step 8: Commit**

```bash
git add drone/src/dynlink/txpower_curve.hpp drone/src/dynlink/txpower_curve.cpp \
        drone/tests/unit/test_dl_txpower_curve.cpp drone/tests/unit/test_dl_txpower_resolve.cpp \
        drone/CMakeLists.txt
git commit -m "drone: per-radio txpower curve registry + override resolver"
```

---

## Task 5: Thread the resolved curve through `DlRuntimeConfig`

Add the resolved curve to the runtime snapshot and read it at the two call sites, plus expose `link.txpowerCurve` in the schema.

**Files:**
- Modify: `drone/src/config/schema.hpp:47-62` (add `Link.txpowerCurve`)
- Modify: `drone/src/config/validate.cpp` (validate the override)
- Modify: `drone/src/dynlink/runtime_config.hpp:31-52` (add field + signature)
- Modify: `drone/src/dynlink/runtime_config.cpp` (resolve + set)
- Modify: `drone/src/dynlink/local_compute.cpp:31`
- Modify: `drone/src/dynlink/controller.cpp:235`
- Modify: caller of `buildDlSnapshot` (find in `drone/src/daemon.cpp`)
- Test: `drone/tests/unit/test_dl_runtime_config.cpp`, `drone/tests/unit/test_validate.cpp`

- [ ] **Step 1: Add the schema field**

In `drone/src/config/schema.hpp`, struct `Link` (lines 47-62), add the optional override field and the macro member:

```cpp
struct Link {
    int channel{161};
    int width{20};
    int txpower{20};
    int mcs{2};
    Fec fec{};
    bool stbc{true};
    bool ldpc{true};
    long linkId{7669206};
    int mtu{1500};
    std::optional<std::string> wlanAdapter{};
    std::optional<std::vector<int>> txpowerCurve{};   // null => per-radio default
    Beamforming beamforming{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Link, channel, width, txpower,
                                                mcs, fec, stbc, ldpc, linkId,
                                                mtu, wlanAdapter, txpowerCurve,
                                                beamforming)
```

Add `#include <vector>` to `schema.hpp` if not already present (it is used elsewhere, but confirm at the top includes).

- [ ] **Step 2: Write the failing validation test** — add to `drone/tests/unit/test_validate.cpp`:

```cpp
TEST_CASE("link.txpowerCurve override must be 8 entries within 0..30 dBm") {
    fpvd::Config c{};
    c.link.txpowerCurve = std::vector<int>{29,28,25,23,19,19,19,19};
    CHECK(fpvd::validate(c).empty());

    c.link.txpowerCurve = std::vector<int>{29,28,25};            // too short
    CHECK(!fpvd::validate(c).empty());

    c.link.txpowerCurve = std::vector<int>{0,0,0,0,0,0,0,99};    // out of range
    auto errs = fpvd::validate(c);
    CHECK(std::any_of(errs.begin(), errs.end(),
        [](const fpvd::ValidationError& e){ return e.path == "link.txpowerCurve"; }));
}
```

- [ ] **Step 3: Run test to verify it fails**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="link.txpowerCurve override*"`
Expected: FAIL — no validation exists yet (the short/out-of-range curves are accepted).

- [ ] **Step 4: Validate the override in `validate.cpp`**

In `drone/src/config/validate.cpp`, inside the `// link` block (after the `link.txpower` check, ~line 57), add:

```cpp
    if (c.link.txpowerCurve.has_value()) {
        const auto& cv = *c.link.txpowerCurve;
        if (cv.size() != 8)
            errs.push_back({"link.txpowerCurve", "must have exactly 8 entries (MCS 0..7)"});
        else for (int v : cv)
            if (v < 0 || v > 30) {
                errs.push_back({"link.txpowerCurve", "each entry must be 0..30 (dBm)"});
                break;
            }
    }
```

- [ ] **Step 5: Run validation test to verify it passes**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="link.txpowerCurve override*"`
Expected: PASS.

- [ ] **Step 6: Add the field + new signature to `runtime_config.hpp`**

In `drone/src/dynlink/runtime_config.hpp`: add `#include "dynlink/txpower_curve.hpp"` near the top; add a field to `DlRuntimeConfig` (after `linkBandwidth`, ~line 48):

```cpp
    TxPowerCurve txPowerCurve{};   // resolved per-MCS dBm curve (override or radio default)
```

and change the `buildDlSnapshot` declaration (line ~79) to take the radio identity:

```cpp
DlRuntimeConfig buildDlSnapshot(const Config& c, const std::string& iface,
                                const std::optional<std::string>& adapterId,
                                const std::string& driver);
```

Add `#include <optional>` to `runtime_config.hpp` if not present.

- [ ] **Step 7: Resolve the curve in `runtime_config.cpp`**

In `drone/src/dynlink/runtime_config.cpp`: add `#include "dynlink/txpower_curve.hpp"`, change the function signature to match, and set the field before `return s;`:

```cpp
DlRuntimeConfig buildDlSnapshot(const Config& c, const std::string& iface,
                                const std::optional<std::string>& adapterId,
                                const std::string& driver)
{
    // ... existing body unchanged ...
    s.iface         = iface;
    s.txPowerCurve  = resolveTxpowerCurve(c.link.txpowerCurve, adapterId, driver).curve;
    return s;
}
```

- [ ] **Step 8: Update the two call sites**

`drone/src/dynlink/local_compute.cpp:31`:

```cpp
    d.txPowerDbm  = txpowerDbmForMcs(cfg.txPowerCurve, d.mcs);
```

`drone/src/dynlink/controller.cpp:235`:

```cpp
    if (radio_) radio_->applySafe(txpowerDbmForMcs(cfg.txPowerCurve, cfg.safe.mcs));
```

Then remove the now-unused legacy overload: delete the `int8_t txpowerDbmForMcs(int mcs);` declaration from `txpower_curve.hpp` and its definition from `txpower_curve.cpp` (added in Task 4). Verify nothing else uses it:

Run: `cd drone && grep -rn 'txpowerDbmForMcs(' src tests | grep -v 'txPowerCurve\|TxPowerCurve\|curve,'`
Expected: no remaining 1-arg call sites (only the 2-arg form remains).

- [ ] **Step 9: Update the `buildDlSnapshot` caller(s)**

Run: `cd drone && grep -rn 'buildDlSnapshot' src tests`
In each non-test caller (in `drone/src/daemon.cpp`), pass the radio identity, e.g.:

```cpp
buildDlSnapshot(effective_, radio_.iface, radio_.adapterId, radio_.driver)
```

- [ ] **Step 10: Update `test_dl_runtime_config.cpp` for the new signature**

In `drone/tests/unit/test_dl_runtime_config.cpp`, update every `buildDlSnapshot(cfg, "wlan0")` call to pass the radio identity, and add a curve assertion:

```cpp
TEST_CASE("buildDlSnapshot resolves the txpower curve from radio + override") {
    fpvd::Config c{};
    auto s1 = fpvd::dynlink::buildDlSnapshot(c, "wlan0", std::string("bl-m8812eu2"), "8812eu");
    CHECK(s1.txPowerCurve == fpvd::dynlink::TxPowerCurve{29,28,25,23,19,19,19,19});

    c.link.txpowerCurve = std::vector<int>{10,10,10,10,10,10,10,10};
    auto s2 = fpvd::dynlink::buildDlSnapshot(c, "wlan0", std::string("bl-m8812eu2"), "8812eu");
    CHECK(s2.txPowerCurve[0] == 10);
}
```

(Update the other existing `buildDlSnapshot(...)` calls in that file to the 4-arg form, e.g. `buildDlSnapshot(c, "wlan0", std::nullopt, "8812eu")`.)

- [ ] **Step 11: Run the full test suite**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS (all). The build now compiles the two call sites against the 2-arg `txpowerDbmForMcs`.

- [ ] **Step 12: Commit**

```bash
git add drone/src/config/schema.hpp drone/src/config/validate.cpp \
        drone/src/dynlink/runtime_config.hpp drone/src/dynlink/runtime_config.cpp \
        drone/src/dynlink/local_compute.cpp drone/src/dynlink/controller.cpp \
        drone/src/daemon.cpp drone/tests/unit/test_dl_runtime_config.cpp \
        drone/tests/unit/test_validate.cpp
git commit -m "drone: thread resolved txpower curve through DlRuntimeConfig + config override"
```

---

## Task 6: Publish `txpowerCurve` in `/status.radio`

Surface the resolved curve and its source in the drone `/status` radio block.

**Files:**
- Modify: `drone/src/status.cpp:87-93`
- Test: `drone/tests/integration/test_daemon.cpp` (or wherever `buildStatus` is exercised — find first)

- [ ] **Step 1: Find the existing status test**

Run: `cd drone && grep -rln 'buildStatus\|"radio"\|"adapterId"' tests`
Use the file that already constructs a `Daemon` and calls `buildStatus` for Step 2 (likely `tests/integration/test_daemon.cpp` or `tests/integration/test_http_handlers.cpp`).

- [ ] **Step 2: Write the failing test** — add a case to that file asserting the curve appears:

```cpp
TEST_CASE("status.radio carries the resolved txpower curve and source") {
    // Build a Daemon the same way the neighboring status tests in this file do,
    // then:
    auto st = fpvd::buildStatus(d);
    REQUIRE(st["radio"].contains("txpowerCurve"));
    CHECK(st["radio"]["txpowerCurve"].is_array());
    CHECK(st["radio"]["txpowerCurve"].size() == 8);
    REQUIRE(st["radio"].contains("txpowerCurveSource"));
    CHECK(st["radio"]["txpowerCurveSource"].is_string());
}
```

(Mirror the Daemon construction from an existing `buildStatus` test in the same file — reuse its fixture/setup verbatim.)

- [ ] **Step 3: Run test to verify it fails**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="status.radio carries the resolved*"`
Expected: FAIL — `radio` has no `txpowerCurve` key.

- [ ] **Step 4: Emit the curve in `status.cpp`**

In `drone/src/status.cpp`, add the include `#include "dynlink/txpower_curve.hpp"` near the top, and extend the `"radio"` object (lines 87-93):

```cpp
    auto rc = dynlink::resolveTxpowerCurve(d.effective().link.txpowerCurve,
                                           d.radio().adapterId, d.radio().driver);
    nlohmann::json curveJ = nlohmann::json::array();
    for (int8_t v : rc.curve) curveJ.push_back(static_cast<int>(v));
```

(place the two lines just before the big `return { … }`). Then in the `"radio"` block:

```cpp
        {"radio", {
            {"driver", d.radio().driver},
            {"iface", d.radio().iface},
            {"adapterId", d.radio().adapterId.has_value()
                           ? nlohmann::json(d.radio().adapterId.value())
                           : nlohmann::json(nullptr)},
            {"txpowerCurve", curveJ},
            {"txpowerCurveSource", rc.source}
        }},
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests --test-case="status.radio carries the resolved*"`
Expected: PASS.

- [ ] **Step 6: Run the full suite**

Run: `cd drone && ./build/fpvd_tests`
Expected: PASS (all).

- [ ] **Step 7: Commit**

```bash
git add drone/src/status.cpp drone/tests/integration/test_daemon.cpp
git commit -m "drone: publish resolved txpowerCurve + source in /status.radio"
```

---

## Task 7: Docs + final verification

**Files:**
- Modify: `docs/api.md` (drone `link` schema + `/status.radio` + `dynamicLink.failsafe`)

- [ ] **Step 1: Update `docs/api.md`**

In the drone `link` schema table: change `txpower` to `0–30` dBm; add a `txpowerCurve` row (`array[8] | null`, "per-MCS dBm; null = per-radio default"). In the `GET /status` `radio` example, add `txpowerCurve` + `txpowerCurveSource`. In the `dynamicLink` section, rename the `safe` block to `failsafe` (note the legacy-key migration). Search and replace any remaining `dynamicLink.safe` references.

Run: `cd /home/gilankpam/Projects/drone/fpvd && grep -n 'dynamicLink.safe\|"safe"\|txpower.*1.*63\|1 – 63\|driver units' docs/api.md`
Fix each hit.

- [ ] **Step 2: Full clean build + test**

Run: `cd drone && rm -rf build && cmake -S . -B build && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS (all tests), clean build with no warnings about the changed files.

- [ ] **Step 3: Commit**

```bash
git add docs/api.md
git commit -m "docs/api: drone txpower dBm + txpowerCurve + dynamicLink.failsafe"
```

---

## Done criteria

- `./build/fpvd_tests` passes from a clean build.
- `dynamicLink.failsafe` is the config key; a legacy `safe` overlay still applies.
- `link.txpower` is dBm (0..30); radio scripts emit `dBm*100` mBm.
- `link.txpowerCurve` overrides the per-radio default; `/status.radio` reports the resolved curve + source.
- `docs/api.md` matches the new drone schema.
