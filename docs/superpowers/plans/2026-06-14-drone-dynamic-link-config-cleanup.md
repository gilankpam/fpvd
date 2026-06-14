# Drone Dynamic-Link Config Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Slim and de-duplicate the drone's `dynamicLink` config — merge two engine blocks, drop two derivable `safe` fields, and make code the single source of defaults behind one tolerant full `config.json` (no `defaults.json`).

**Architecture:** Drone is C++17 with nlohmann-json structs that already hold every default (`..._WITH_DEFAULT`). We (1) merge `dynamicLink.bitrate`+`fec` into one `compute` block, (2) remove `safe.bandwidth`/`safe.txPowerDbm` and derive them at fallback time, (3) replace the `defaults.json`+merge loader with a `Config{}`-baseline tolerant loader (missing key → default, unknown key → warning) that persists the full config, and add `--dump-config`. Spec: `docs/superpowers/specs/2026-06-14-drone-dynamic-link-config-cleanup-design.md`.

**Tech Stack:** C++17, nlohmann/json (vendored), doctest. Build `cmake --build drone/build -j`; tests run from `drone/` via `./build/fpvd_tests` (NOT `ctest` — see CLAUDE.md).

---

## Prerequisites (once)

- [ ] **Configure + baseline-build**

Run (from repo root):
```bash
cmake -S drone -B drone/build -DCMAKE_BUILD_TYPE=Debug
cmake --build drone/build -j && (cd drone && ./build/fpvd_tests)
```
Expected: PASS (300 cases) — this is the green baseline.

---

## Task 1: Merge `bitrate` + `fec` → `dynamicLink.compute`

Behavior-neutral rename: both blocks already feed one `BitrateEngineConfig`.

**Files:**
- Modify: `drone/src/config/schema.hpp` (DynamicLink* structs)
- Modify: `drone/src/dynlink/runtime_config.cpp:44-53` (snapshot mapping)
- Modify: `drone/src/config/validate.cpp` (add `compute` ranges)
- Test: `drone/tests/unit/test_schema.cpp`, `drone/tests/unit/test_dl_runtime_config.cpp`

- [ ] **Step 1: Write the failing test** — append to `drone/tests/unit/test_dl_runtime_config.cpp`:

```cpp
TEST_CASE("buildDlSnapshot maps dynamicLink.compute -> BitrateEngineConfig") {
    Config c{};
    c.dynamicLink.compute.minBitrateKbps      = 1500;
    c.dynamicLink.compute.maxBitrateKbps      = 20000;
    c.dynamicLink.compute.baseRedundancyRatio = 0.4;
    c.dynamicLink.compute.blocksPerFrame      = 3.0;
    c.dynamicLink.compute.kMin                = 4;
    c.dynamicLink.compute.kMax                = 40;
    auto s = buildDlSnapshot(c, "wlan0");
    CHECK(s.bitrate.minBitrateKbps     == 1500);
    CHECK(s.bitrate.maxBitrateKbps     == 20000);
    CHECK(s.bitrate.baseRedundancyRatio == doctest::Approx(0.4));
    CHECK(s.bitrate.blocksPerFrame     == doctest::Approx(3.0));
    CHECK(s.bitrate.kMin               == 4);
    CHECK(s.bitrate.kMax               == 40);
}
```

- [ ] **Step 2: Run — verify it FAILS to compile**

Run: `cmake --build drone/build -j`
Expected: error — `Config::dynamicLink` has no member `compute`.

- [ ] **Step 3: Replace the two structs with `DynamicLinkCompute`** in `drone/src/config/schema.hpp`. Delete `struct DynamicLinkBitrate {...}` + its macro and `struct DynamicLinkFec {...}` + its macro, and add:

```cpp
struct DynamicLinkCompute {
    int    minBitrateKbps{1000};
    int    maxBitrateKbps{24000};
    double baseRedundancyRatio{0.5};   // n/k = 1 + ratio (= 8/12 data fraction)
    double blocksPerFrame{2.0};
    int    kMin{2};
    int    kMax{50};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkCompute,
    minBitrateKbps, maxBitrateKbps, baseRedundancyRatio, blocksPerFrame, kMin, kMax)
```

In `struct DynamicLink`, replace the `DynamicLinkBitrate bitrate{};` and `DynamicLinkFec fec{};` members with one `DynamicLinkCompute compute{};`, and in the `DynamicLink` macro change the trailing `... roiQp, safe, bitrate, fec)` to `... roiQp, safe, compute)`.

- [ ] **Step 4: Update the snapshot mapping** in `drone/src/dynlink/runtime_config.cpp` — replace the `s.bitrate = BitrateEngineConfig{...}` block with:

```cpp
    s.bitrate = BitrateEngineConfig{
        dl.compute.baseRedundancyRatio,
        dl.compute.blocksPerFrame,
        dl.compute.kMin,
        dl.compute.kMax,
        dl.compute.minBitrateKbps,
        dl.compute.maxBitrateKbps,
        c.link.mtu,
        c.video.fps,
    };
```

- [ ] **Step 5: Add validation** in `drone/src/config/validate.cpp` (inside the `dynamicLink` block, after the roiQp checks):

```cpp
        if (dl.compute.minBitrateKbps <= 0 ||
            dl.compute.maxBitrateKbps <= dl.compute.minBitrateKbps)
            errs.push_back({"dynamicLink.compute",
                            "require 0 < minBitrateKbps < maxBitrateKbps"});
        if (dl.compute.baseRedundancyRatio <= 0.0)
            errs.push_back({"dynamicLink.compute.baseRedundancyRatio", "must be > 0"});
        if (dl.compute.blocksPerFrame <= 0.0)
            errs.push_back({"dynamicLink.compute.blocksPerFrame", "must be > 0"});
        if (dl.compute.kMin < 1 || dl.compute.kMax < dl.compute.kMin)
            errs.push_back({"dynamicLink.compute.k", "require 1 <= kMin <= kMax"});
```

- [ ] **Step 6: Fix the schema round-trip test** in `drone/tests/unit/test_schema.cpp`. In the big JSON literal, replace the two lines
`"bitrate":{"minBitrateKbps":1000,"maxBitrateKbps":24000},` and
`"fec":{"baseRedundancyRatio":0.5,"blocksPerFrame":2.0,"kMin":2,"kMax":50}`
with one line:
`"compute":{"minBitrateKbps":1000,"maxBitrateKbps":24000,"baseRedundancyRatio":0.5,"blocksPerFrame":2.0,"kMin":2,"kMax":50}`
(and remove the trailing comma juggling so it stays valid JSON).

- [ ] **Step 7: Build + run the affected tests**

Run: `cmake --build drone/build -j && (cd drone && ./build/fpvd_tests --test-case='*compute*,*schema*,*runtime_config*')`
Expected: PASS.

- [ ] **Step 8: Full suite + commit**

```bash
cmake --build drone/build -j && (cd drone && ./build/fpvd_tests)
git add drone/src/config/schema.hpp drone/src/dynlink/runtime_config.cpp \
        drone/src/config/validate.cpp drone/tests/unit/test_schema.cpp \
        drone/tests/unit/test_dl_runtime_config.cpp
git commit -m "drone: merge dynamicLink.bitrate+fec into dynamicLink.compute"
```
Expected: 300+ cases PASS.

---

## Task 2: Slim `safe` — drop `bandwidth`/`txPowerDbm`, derive them

`safe.txPowerDbm` is already dead (`dispatchTxSafe` uses `txpowerDbmForMcs(safe.mcs)`); `safe.bandwidth` becomes `cfg.linkBandwidth` (derived from `link.width`).

**Files:**
- Modify: `drone/src/config/schema.hpp` (`DynamicLinkSafe`)
- Modify: `drone/src/dynlink/runtime_config.hpp` (`SafeDefaults`)
- Modify: `drone/src/dynlink/runtime_config.cpp` (snapshot mapping)
- Modify: `drone/src/dynlink/controller.cpp` (`dispatchTxSafe` — `cfg.safe.bandwidth` → `cfg.linkBandwidth`)
- Modify: `drone/src/config/validate.cpp` (drop the two safe checks)
- Test: `drone/tests/integration/test_dl_controller.cpp`, `drone/tests/unit/test_schema.cpp`, `drone/tests/unit/test_dl_runtime_config.cpp`

- [ ] **Step 1: Write the failing test** — add to `drone/tests/unit/test_dl_runtime_config.cpp`:

```cpp
TEST_CASE("safe omits bandwidth/txPowerDbm; SafeDefaults has 6 fields") {
    Config c{};
    c.link.width = 40;                 // operating bandwidth
    c.dynamicLink.safe.mcs = 2;
    auto s = buildDlSnapshot(c, "wlan0");
    // SafeDefaults no longer carries bandwidth/txPowerDbm; the safe rung's
    // bandwidth follows the operating link, derived in dispatchTxSafe.
    CHECK(s.safe.mcs == 2);
    CHECK(s.linkBandwidth == 40);
    // Compile-time guard: these members must not exist anymore.
    // (Uncomment to confirm during development; then delete.)
    // s.safe.bandwidth;  s.safe.txPowerDbm;   // <- must NOT compile
}
```

- [ ] **Step 2: Run — verify it FAILS** (if any existing test still sets `safe.bandwidth`/`txPowerDbm` it will keep compiling; the real gate is Step 5/6 removing those members, which breaks the old asserts). Run `cmake --build drone/build -j` and note current state.

- [ ] **Step 3: Trim the schema** in `drone/src/config/schema.hpp` — `struct DynamicLinkSafe` drops `bandwidth` and `txPowerDbm`:

```cpp
struct DynamicLinkSafe {
    int mcs{1};
    int k{8};
    int n{12};
    int overheadPct{100};
    int deadlineMs{30};
    int bitrateKbps{2000};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkSafe, mcs, k, n,
    overheadPct, deadlineMs, bitrateKbps)
```

- [ ] **Step 4: Trim `SafeDefaults`** in `drone/src/dynlink/runtime_config.hpp`:

```cpp
struct SafeDefaults {
    uint8_t  mcs;
    uint8_t  k;
    uint8_t  n;
    uint16_t bitrateKbps;
    uint8_t  overheadPct{100};
    uint8_t  deadlineMs{30};
};
```

- [ ] **Step 5: Update the snapshot mapping** in `drone/src/dynlink/runtime_config.cpp` — set `s.safe` from the surviving fields only (drop the `bandwidth`/`txPowerDbm` initializers):

```cpp
    s.safe = SafeDefaults{
        static_cast<uint8_t> (dl.safe.mcs),
        static_cast<uint8_t> (dl.safe.k),
        static_cast<uint8_t> (dl.safe.n),
        static_cast<uint16_t>(dl.safe.bitrateKbps),
    };
    s.safe.overheadPct = static_cast<uint8_t>(dl.safe.overheadPct);
    s.safe.deadlineMs  = static_cast<uint8_t>(dl.safe.deadlineMs);
```

- [ ] **Step 6: Derive bandwidth in `dispatchTxSafe`** (`drone/src/dynlink/controller.cpp`) — replace the two `cfg.safe.bandwidth` uses (the `setRadio` call and the probe `setRadio` call) with `cfg.linkBandwidth`. The txpower line already reads `txpowerDbmForMcs(cfg.safe.mcs)` — leave it. Update the nearby comment "safe.bandwidth is the 20/40 radiotap value." → "safe rung uses the operating linkBandwidth."

- [ ] **Step 7: Drop the two validations** in `drone/src/config/validate.cpp` — delete the `dl.safe.bandwidth != 10 && ...` block and the `dl.safe.txPowerDbm < -10 || ... > 30` block.

- [ ] **Step 8: Fix tests that reference the removed fields.**
  - `drone/tests/unit/test_schema.cpp`: in the JSON literal, change the `safe` object to `"safe":{"mcs":1,"k":8,"n":12,"overheadPct":100,"deadlineMs":30,"bitrateKbps":2000}`.
  - `drone/tests/integration/test_dl_controller.cpp`: any positional `SafeDefaults{...}` init now has 6 fields in the new order `{mcs,k,n,bitrateKbps,overheadPct,deadlineMs}` (or use designated/field assignment). Update each.
  - Add to `test_dl_controller.cpp`'s safe-fallback test an assertion that the safe `setRadio` carries the operating bandwidth (e.g. `wfb.sawRadio(safeMcs, 20)` with `snap.linkBandwidth = 20`).

- [ ] **Step 9: Build + targeted tests**

Run: `cmake --build drone/build -j && (cd drone && ./build/fpvd_tests --test-case='*safe*,*controller*,*schema*,*runtime_config*')`
Expected: PASS.

- [ ] **Step 10: Full suite + commit**

```bash
cmake --build drone/build -j && (cd drone && ./build/fpvd_tests)
git add drone/src/config/schema.hpp drone/src/dynlink/runtime_config.hpp \
        drone/src/dynlink/runtime_config.cpp drone/src/dynlink/controller.cpp \
        drone/src/config/validate.cpp drone/tests/
git commit -m "drone: slim dynamicLink.safe — derive bandwidth (link.width) + txPowerDbm (curve)"
```

---

## Task 3: Tolerant loader — `Config{}` baseline + warn-on-unknown

New `loadEffective(configPath)`: base is the serialized code defaults; a missing file → pure defaults; unknown keys warn (skipping the free-form `services` map); a present key wins.

**Files:**
- Modify: `drone/src/config/store.hpp` (signature), `drone/src/config/store.cpp` (impl)
- Test: `drone/tests/unit/test_store.cpp`

- [ ] **Step 1: Write the failing tests** — add to `drone/tests/unit/test_store.cpp`:

```cpp
TEST_CASE("loadEffective: no file yields code defaults") {
    Config c = loadEffective("/no/such/config.json");
    CHECK(c.dynamicLink.healthTimeoutMs == 10000);   // schema default
    CHECK(c.link.width == 20);
}

TEST_CASE("loadEffective: present key overrides, missing key defaults") {
    auto tmp = std::filesystem::temp_directory_path() / "fpvd-cfg-load.json";
    std::ofstream(tmp) << R"({"dynamicLink":{"healthTimeoutMs":7000}})";
    Config c = loadEffective(tmp.string());
    CHECK(c.dynamicLink.healthTimeoutMs == 7000);    // from file
    CHECK(c.dynamicLink.applyStaggerMs == 50);       // missing -> default
    std::filesystem::remove(tmp);
}

TEST_CASE("unknownConfigKeys flags strays but not services entries") {
    nlohmann::json cfg = {
        {"dynamicLink", {{"bogusKnob", 1}}},
        {"services", {{"myproc", {{"exec", "/bin/true"}}}}},
        {"strayTop", true},
    };
    auto unknown = unknownConfigKeys(cfg);
    CHECK(std::find(unknown.begin(), unknown.end(), "dynamicLink.bogusKnob") != unknown.end());
    CHECK(std::find(unknown.begin(), unknown.end(), "strayTop") != unknown.end());
    // services is a free-form map of user-named processes — not flagged.
    CHECK(std::find(unknown.begin(), unknown.end(), "services.myproc") == unknown.end());
}
```
Add includes at the top of the file if missing: `#include <filesystem>`, `#include <fstream>`, `#include <algorithm>`.

- [ ] **Step 2: Run — verify it FAILS to compile** (`unknownConfigKeys` undefined; `loadEffective` arity wrong).

Run: `cmake --build drone/build -j`
Expected: errors.

- [ ] **Step 3: Declare the new API** in `drone/src/config/store.hpp` — replace the `loadEffective(defaultsPath, overlayPath)` declaration and remove `computeOverlay`:

```cpp
// Load the full config from `configPath` merged onto the code defaults
// (Config{}). A missing file yields the code defaults. Unknown keys are
// logged (see unknownConfigKeys) and ignored. Throws StoreError only on
// JSON parse failure or a wrong-typed value for a known key.
Config loadEffective(const std::string& configPath);

// Dotted key-paths present in `cfg` but absent from the code-default
// schema, EXCLUDING the free-form `services` map. Used to warn on
// deprecated/renamed/typo'd keys.
std::vector<std::string> unknownConfigKeys(const nlohmann::json& cfg);
```
(Leave `loadDefaults`, `deepMergeJson`, `atomicWriteJson` as-is; delete the `computeOverlay` declaration.)

- [ ] **Step 4: Implement** in `drone/src/config/store.cpp` — delete `computeOverlay`, replace `loadEffective`, and add the helpers:

```cpp
static void collectUnknown(const nlohmann::json& cfg, const nlohmann::json& ref,
                           const std::string& prefix,
                           std::vector<std::string>& out) {
    if (!cfg.is_object() || !ref.is_object()) return;
    for (auto it = cfg.begin(); it != cfg.end(); ++it) {
        std::string path = prefix.empty() ? it.key() : prefix + "." + it.key();
        if (prefix.empty() && it.key() == "services") continue;  // free-form map
        if (!ref.contains(it.key())) { out.push_back(path); continue; }
        if (it.value().is_object() && ref[it.key()].is_object())
            collectUnknown(it.value(), ref[it.key()], path, out);
    }
}

std::vector<std::string> unknownConfigKeys(const nlohmann::json& cfg) {
    std::vector<std::string> out;
    collectUnknown(cfg, nlohmann::json(Config{}), "", out);
    return out;
}

Config loadEffective(const std::string& configPath) {
    nlohmann::json base = Config{};        // serialize the code defaults
    std::ifstream f(configPath);
    if (!f) return Config{};               // no file -> pure defaults
    std::stringstream buf; buf << f.rdbuf();
    nlohmann::json fileJ;
    try { fileJ = nlohmann::json::parse(buf.str()); }
    catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("config parse error: ") + e.what());
    }
    for (auto& k : unknownConfigKeys(fileJ))
        std::fprintf(stderr,
            "fpvd: warning: unknown config key '%s' (ignored)\n", k.c_str());
    auto merged = deepMergeJson(base, fileJ);
    try { return merged.get<Config>(); }
    catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("config schema: ") + e.what());
    }
}
```
Add `#include <cstdio>` and `#include <vector>` to `store.cpp` if not present.

- [ ] **Step 5: Build + run store tests**

Run: `cmake --build drone/build -j && (cd drone && ./build/fpvd_tests --test-case='*loadEffective*,*unknownConfigKeys*')`
Expected: PASS. (Other targets may still fail to build — daemon callers fixed in Task 4. If the test binary won't link yet, proceed to Task 4 and run these at Task 4 Step 6.)

- [ ] **Step 6: Commit**

```bash
git add drone/src/config/store.hpp drone/src/config/store.cpp drone/tests/unit/test_store.cpp
git commit -m "drone: tolerant config loader (Config{} baseline + warn-on-unknown); drop computeOverlay"
```

---

## Task 4: Daemon wiring — full-config persist, code-sourced defaults, drop `defaultsPath`

**Files:**
- Modify: `drone/src/daemon.hpp` (`DaemonPaths`: drop `defaultsPath`)
- Modify: `drone/src/daemon.cpp` (`defaultsJson`, `bootstrap`, `apply` persist, `reset`)
- Test: `drone/tests/integration/test_daemon.cpp`, `drone/tests/integration/test_http_handlers.cpp`

- [ ] **Step 1: Write the failing test** — add to `drone/tests/integration/test_daemon.cpp`:

```cpp
TEST_CASE("apply persists the FULL config, not a sparse overlay") {
    auto tmp = fs::temp_directory_path() / "fpvd-fullcfg";
    auto paths = makeRoutingPaths(tmp, 46900);   // see Step 4 harness change
    fpvd::Daemon d(paths); d.bootstrap(false);
    auto ar = d.patchPending(nlohmann::json::parse(R"({"dynamicLink":{"enabled":true}})"));
    REQUIRE(ar.ok);
    auto r = d.apply(false);
    REQUIRE(r.ok);
    std::ifstream f(paths.configPath);
    auto written = nlohmann::json::parse(f);
    // Full config: link/video/dynamicLink all present, not just the diff.
    CHECK(written.contains("link"));
    CHECK(written.contains("video"));
    CHECK(written["dynamicLink"]["enabled"] == true);
    CHECK(written["dynamicLink"].contains("safe"));  // a default that wasn't patched
    fs::remove_all(tmp);
}
```

- [ ] **Step 2: Run — verify it FAILS to compile** (`paths.configPath` undefined; harness still uses `defaultsPath`).

- [ ] **Step 3: Update `DaemonPaths`** in `drone/src/daemon.hpp` — remove the `defaultsPath` field and rename `overlayPath` → `configPath`:

```cpp
struct DaemonPaths {
    std::string configPath;      // /etc/fpvd/config.json (full config)
    std::string radioUpScript;
    std::string waybeamJsonPath;
    std::string radioTuneScript{};
    dynlink::Endpoints dlEndpoints{};
    int idrPort{idr::kIdrPort};
    std::string osdMsgPath{osd::kOsdMsgPath};
    int waybeamRestartSettleMs{700};
};
```
(Keep `defaultsJson()` declared — it now returns the code defaults.)

- [ ] **Step 4: Update `daemon.cpp`.**
  - `defaultsJson()` (line ~49): replace the file read with `return nlohmann::json(Config{});`
  - `bootstrap()` (line ~55): `effective_ = loadEffective(paths_.configPath);`
  - `apply()` persistence (lines ~330-333): replace the `defaultsJson()`/`computeOverlay`/`atomicWriteJson(paths_.overlayPath, overlay)` trio with:
    ```cpp
    atomicWriteJson(paths_.configPath, nlohmann::json(pending_));
    ```
  - `reset()` (lines ~548-549): `std::filesystem::remove(paths_.configPath, ec); pending_ = loadEffective("/no/such/path");`

- [ ] **Step 5: Update the test harness** in `drone/tests/integration/test_daemon.cpp` and `drone/tests/integration/test_http_handlers.cpp`:
  - In each path factory (`makeRoutingPaths` ~line 453, the http_handlers factory ~line 356), delete the `paths.defaultsPath = ...` line and rename `paths.overlayPath = ...` → `paths.configPath = (tmp / "etc" / "fpvd" / "config.json").string();`.
  - Replace every other `paths.overlayPath` / `paths.defaultsPath` reference with `paths.configPath`.
  - Remove the `fs::copy_file("tests/fixtures/defaults.json", …)` calls — the daemon now starts from code defaults; tests that need a specific starting value should `patchPending` it or write a `config.json` first.
  - Any test that asserted a value sourced from the old fixture (e.g. a non-default `link.channel`) must either set it via `config.json`/`patchPending` or assert the code default instead.

- [ ] **Step 6: Build + run daemon/store/http tests**

Run: `cmake --build drone/build -j && (cd drone && ./build/fpvd_tests --test-case='*daemon*,*http*,*loadEffective*,*unknownConfigKeys*')`
Expected: PASS.

- [ ] **Step 7: Full suite + commit**

```bash
cmake --build drone/build -j && (cd drone && ./build/fpvd_tests)
git add drone/src/daemon.hpp drone/src/daemon.cpp drone/tests/
git commit -m "drone: full-config persistence; defaults from code; DaemonPaths drops defaultsPath"
```

---

## Task 5: `main.cpp` wiring + `--dump-config`

**Files:**
- Modify: `drone/src/main.cpp`

- [ ] **Step 1: Update arg parsing + paths** in `drone/src/main.cpp`:
  - Delete `defaultsPath` and the `--defaults` arg.
  - Rename `overlayPath` → `configPath` (default stays `/etc/fpvd/config.json`); keep `--overlay` accepted as an alias OR rename the flag to `--config` (update the usage string either way).
  - Add a `--dump-config` flag handled BEFORE constructing the daemon:
    ```cpp
    else if (a == "--dump-config") {
        std::cout << nlohmann::json(fpvd::Config{}).dump(2) << "\n";
        return 0;
    }
    ```
    (Add `#include "config/schema.hpp"` if needed.)
  - Update the `DaemonPaths` aggregate init to the new field order (no `defaultsPath`):
    ```cpp
    fpvd::DaemonPaths paths{configPath, radioUp, waybeamPath, radioTune};
    ```
  - Update the `--help` usage string.

- [ ] **Step 2: Build + smoke-test `--dump-config`**

Run: `cmake --build drone/build -j && ./drone/build/fpvd --dump-config | head -20`
Expected: prints a full JSON config (link/video/.../dynamicLink/services) and exits 0. Sanity-check it parses: `./drone/build/fpvd --dump-config | python3 -c 'import json,sys; json.load(sys.stdin); print("ok")'` → `ok`.

- [ ] **Step 3: Full suite + commit**

```bash
cmake --build drone/build -j && (cd drone && ./build/fpvd_tests)
git add drone/src/main.cpp
git commit -m "drone: main --dump-config; --config path; drop --defaults"
```

---

## Task 6: Remove `defaults.json` + its install rule + stale fixture

**Files:**
- Delete: `drone/etc/defaults.json`, `drone/tests/fixtures/defaults.json` (if no longer referenced)
- Modify: `drone/CMakeLists.txt` (drop the install rule)

- [ ] **Step 1: Confirm no remaining references**

Run: `grep -rn "etc/defaults.json\|fixtures/defaults.json\|defaultsPath\|--defaults" drone/ | grep -v build/`
Expected: only the CMake `install(FILES etc/defaults.json …)` line and possibly nothing in tests. If a test still references the fixture, fix it (Task 4 Step 5) before deleting.

- [ ] **Step 1b: Guard — code defaults must match the retired file (except intended deltas)**

Dropping `defaults.json` makes the *struct* defaults authoritative. Verify they equal the file that was shipped, so nothing silently changes:

```bash
git show HEAD:drone/etc/defaults.json > /tmp/old-defaults.json
./drone/build/fpvd --dump-config > /tmp/new-defaults.json
diff <(python3 -m json.tool /tmp/old-defaults.json) <(python3 -m json.tool /tmp/new-defaults.json)
```
Expected: the ONLY differences are the intended ones from Tasks 1–2 — `dynamicLink.bitrate`+`fec` replaced by `dynamicLink.compute`, and `dynamicLink.safe` losing `bandwidth`/`txPowerDbm`. Any *other* diff means a struct default drifted from the file value: fix the struct default in `schema.hpp` to match the shipped value (or, if the file value was the bug, note the intentional change in the commit). Re-run until clean.

- [ ] **Step 2: Remove the install rule** in `drone/CMakeLists.txt` — delete the block:
```cmake
install(FILES etc/defaults.json DESTINATION ../etc/fpvd)
```
(and its explanatory comment).

- [ ] **Step 3: Delete the files**

```bash
git rm drone/etc/defaults.json drone/tests/fixtures/defaults.json
```
(If `tests/fixtures/defaults.json` is still used by a test, keep it and note why; otherwise remove.)

- [ ] **Step 4: Reconfigure (CMake file list changed) + full suite**

Run: `cmake -S drone -B drone/build -DCMAKE_BUILD_TYPE=Debug && cmake --build drone/build -j && (cd drone && ./build/fpvd_tests)`
Expected: configure OK, build OK, all tests PASS.

- [ ] **Step 5: Commit**

```bash
git add drone/CMakeLists.txt
git commit -m "drone: drop shipped defaults.json + install rule (code is the default source)"
```

---

## Task 7: Tuning-reference doc

**Files:**
- Create: `docs/dynamic-link-tuning.md`

- [ ] **Step 1: Write the doc.** Enumerate every drone `dynamicLink` tunable knob with purpose + valid range, grouped operational vs advanced, plus the frozen constants. Source of truth: the final `schema.hpp` + `validate.cpp`. Include:
  - **Operational:** `dynamicLink.enabled`; `dynamicLink.safe.{mcs,k,n,overheadPct,deadlineMs,bitrateKbps}`; top-level `osd.enabled`.
  - **Advanced:** `dynamicLink.{healthTimeoutMs (≥1000), applyStaggerMs (0–500), applySubPaceMs (0–50)}`; `dynamicLink.roiQp.{thresholdKbps>0, lowAnchorKbps>0 (< threshold), floor≤0, step≥1}`; `dynamicLink.compute.{minBitrateKbps>0, maxBitrateKbps>min, baseRedundancyRatio>0, blocksPerFrame>0, kMin≥1, kMax≥kMin}`.
  - **Frozen (no config):** `txpower_curve.hpp` `{29,28,25,23,19,19,19,19}`; `probe_constants.hpp`; `idr_constants.hpp`; `osd_constants.hpp`; OpenIPC rate table.
  - A note: the canonical inventory is `fpvd --dump-config` / `GET /config`; this doc adds semantics. `safe.bandwidth`/`txPowerDbm` are derived (not config).

- [ ] **Step 2: Commit**

```bash
git add docs/dynamic-link-tuning.md
git commit -m "docs: drone dynamic-link tuning reference"
```

---

## Self-review notes (verify before execution)

- **Spec coverage:** Change 1 → Task 1; Change 2 → Task 2; Change 3 (loader/persist/dump/drop-defaults) → Tasks 3–6; Change 4 → Task 7. ✓
- **Type consistency:** `DynamicLinkCompute` (Task 1) field names match the `runtime_config.cpp` mapping and the Task 7 doc; `SafeDefaults` 6-field shape (Task 2) matches the snapshot init and the test inits; `loadEffective(configPath)` / `unknownConfigKeys` (Task 3) match the daemon callers (Task 4) and `DaemonPaths.configPath`.
- **Watch-outs:** the `services` free-form map must be excluded from warn-on-unknown (Task 3 Step 4); the daemon test harness fixture migration (Task 4 Step 5) is the fiddliest part — run the full suite after Task 4, not just targeted cases.
