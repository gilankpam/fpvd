# waybeam Resilience preset Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Expose waybeam's `video0.resilience` error-resilience preset through fpvd config as `video.resilience` — strict-enum validated, applied by a waybeam process restart, never DL-locked.

**Architecture:** Add one string field to the `Video` config struct. It flows through the existing waybeam config pipeline: `toWaybeamJson()` writes it into `/etc/waybeam.json`; `waybeamConfigDiff()` classifies a change as **restart** (bounce the waybeam process, no live `/api/v1/set` push); the config validator rejects unknown presets. It is deliberately absent from the dynamic-link lock list, so it stays `PATCH`-able while `dynamicLink.enabled`. Drone-only; no GS changes.

**Tech Stack:** C++17, nlohmann/json, doctest. Build with CMake; run tests from `drone/` via `./build/fpvd_tests` (never `ctest` — see CLAUDE.md).

**Design spec:** `docs/superpowers/specs/2026-06-15-waybeam-resilience-preset-design.md`

**Why deployed configs won't break:** `loadEffective()` (`src/config/store.cpp:52`) deep-merges the on-disk overlay onto a fully-serialized `Config{}` base, so a `config.json` whose `video` block omits `resilience` gets `"off"` backfilled from defaults before `get<Config>()`. The only places that parse a full video block *without* deep-merge are two unit tests in `test_schema.cpp`, fixed in Task 1.

**Preset value set (single source of truth):**
`off`, `rescue`, `quality`, `sprint`, `racing`, `endurance`, `patrol`, `rally`, `range`, `fpv` (default `off`).

---

## Pre-flight

- [ ] **Step 0: Build and confirm the suite is green before any change**

Run:
```bash
cmake -S drone -B drone/build -DCMAKE_BUILD_TYPE=Debug
cmake --build drone/build -j
cd drone && ./build/fpvd_tests
```
Expected: build succeeds; all tests pass (`Status: SUCCESS`). If not green, stop and investigate before proceeding.

---

## Task 1: Add `video.resilience` to the config schema

**Files:**
- Modify: `drone/src/config/schema.hpp:78-90` (the `Video` struct + its NLOHMANN macro)
- Test: `drone/tests/unit/test_schema.cpp` (new test + fix two existing fixtures)

- [ ] **Step 1: Write the failing test**

Add to `drone/tests/unit/test_schema.cpp` (after the `sensorBin` test, around line 62):

```cpp
TEST_CASE("schema: video.resilience defaults to off and round-trips") {
    fpvd::Config c{};
    CHECK(c.video.resilience == "off");

    // Serialised default config must carry the new field.
    json def = json(c);
    CHECK(def["video"].contains("resilience"));
    CHECK(def["video"]["resilience"] == "off");

    // A value survives a parse -> serialise round-trip at struct and JSON level.
    c.video.resilience = "fpv";
    json j = c;
    CHECK(j["video"]["resilience"] == "fpv");
    auto c2 = j.get<fpvd::Config>();
    CHECK(c2.video.resilience == "fpv");
}
```

- [ ] **Step 2: Build and verify it fails**

Run:
```bash
cmake --build drone/build -j 2>&1 | tail -20
```
Expected: **compile error** — `'struct fpvd::Video' has no member named 'resilience'`. This is the RED state.

- [ ] **Step 3: Add the field to the `Video` struct**

In `drone/src/config/schema.hpp`, change the `Video` struct (lines 78-90) to add `resilience` after `gopSize` (it is GOP/intra-refresh related), and add it to the macro:

```cpp
struct Video {
    std::string codec{"h265"};
    std::string resolution{"1920x1080"};
    int fps{60};
    int bitrate{8192};
    std::string rcMode{"cbr"};
    double gopSize{1.0};
    // waybeam error-resilience preset. Derives intra-refresh, the SVC-T
    // reference pyramid, and GOP length inside waybeam; gopSize is honored
    // only when resilience == "off". Validated against a fixed preset set in
    // config/validate.cpp.
    std::string resilience{"off"};
    int qpDelta{-4};
    std::string sensorBin{""};   // sensor binning mode (empty = sensor default)
    Roi roi{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Video, codec, resolution, fps, bitrate,
                                   rcMode, gopSize, resilience, qpDelta,
                                   sensorBin, roi)
```

- [ ] **Step 4: Fix the two existing direct-parse fixtures in `test_schema.cpp`**

`Video` is strict (`NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` throws on a missing key). Two tests parse a full video block directly (no deep-merge) and will now throw. Add `"resilience":"off"` to both.

In the round-trip test (currently `test_schema.cpp:14-17`), the video object:
```cpp
        "video":{"codec":"h265","resolution":"1920x1080","fps":60,
                 "bitrate":8192,"rcMode":"cbr","gopSize":1.0,"resilience":"off","qpDelta":-4,
                 "sensorBin":"",
                 "roi":{"enabled":true,"qp":0,"center":0.4,"steps":2}},
```

In the `"schema: Config parses without dynamicLink key"` test (currently `test_schema.cpp:180-183`), the video object:
```cpp
        "video": {"codec":"h265","resolution":"1920x1080","fps":60,
                  "bitrate":8192,"rcMode":"cbr","gopSize":1.0,"resilience":"off","qpDelta":-4,
                  "sensorBin":"",
                  "roi":{"enabled":true,"qp":0,"center":0.4,"steps":2}},
```

(The round-trip test asserts `out == j`, so `j` must include `resilience` to match the now-serialized output. JSON object equality is key-set based, so field order is irrelevant.)

- [ ] **Step 5: Build and run the schema tests**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests --test-case='*schema*'
```
Expected: PASS, including the new resilience test and both fixed fixtures.

- [ ] **Step 6: Run the full suite (catch any other direct-parse fixture)**

Run:
```bash
cd drone && ./build/fpvd_tests
```
Expected: all PASS. If a different test throws `[json.exception] key 'resilience' not found`, add `"resilience":"off"` to that test's video literal too, rebuild, and re-run.

- [ ] **Step 7: Commit**

```bash
git add drone/src/config/schema.hpp drone/tests/unit/test_schema.cpp
git commit -m "feat(drone): add video.resilience field to config schema

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: Translate `video.resilience` into `/etc/waybeam.json`

**Files:**
- Modify: `drone/src/translate/waybeam.cpp:26-42` (the `video0` block in `toWaybeamJson`)
- Test: `drone/tests/unit/test_translate_waybeam.cpp`

- [ ] **Step 1: Write the failing test**

Add to `drone/tests/unit/test_translate_waybeam.cpp`:

```cpp
TEST_CASE("translate.waybeam: video.resilience maps to video0.resilience") {
    fpvd::Config c{};
    // Default is "off".
    CHECK(fpvd::toWaybeamJson(c)["video0"]["resilience"] == "off");

    c.video.resilience = "fpv";
    CHECK(fpvd::toWaybeamJson(c)["video0"]["resilience"] == "fpv");
}
```

- [ ] **Step 2: Build and run to verify it fails**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests --test-case='*video.resilience maps*'
```
Expected: FAIL — `video0` has no `resilience` key (the `CHECK` on `"off"` fails / key access yields null).

- [ ] **Step 3: Emit the field in `toWaybeamJson`**

In `drone/src/translate/waybeam.cpp`, add `resilience` to the `video0` object (insert after the `gopSize` line, currently line 31):

```cpp
        {"video0", {
            {"rcMode", c.video.rcMode},
            {"fps", c.video.fps},
            {"size", c.video.resolution},
            {"bitrate", c.video.bitrate},
            {"gopSize", c.video.gopSize},
            // waybeam ignores gopSize when resilience != "off" (the preset owns
            // intra-refresh + GOP). See waybeamConfigDiff() — resilience is a
            // RESTART-class field.
            {"resilience", c.video.resilience},
            {"qpDelta", c.video.qpDelta},
            {"frameLost", true},
            {"sceneThreshold", 0},
            {"sceneHoldoff", 2},
            {"intraRefreshMode", "off"},
            {"intraRefreshLines", 0},
            {"intraRefreshQp", 0},
            {"zoomPct", 0.0},
            {"zoomX", 0.5},
            {"zoomY", 0.5}
        }},
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests --test-case='*translate.waybeam*'
```
Expected: PASS (new test + existing translate tests).

- [ ] **Step 5: Commit**

```bash
git add drone/src/translate/waybeam.cpp drone/tests/unit/test_translate_waybeam.cpp
git commit -m "feat(drone): emit video0.resilience in waybeam.json

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: Classify a resilience change as RESTART in `waybeamConfigDiff`

**Files:**
- Modify: `drone/src/translate/waybeam.cpp:123-131` (the RESTART section of `waybeamConfigDiff`)
- Test: `drone/tests/unit/test_waybeam_diff.cpp`

- [ ] **Step 1: Write the failing test**

Add to `drone/tests/unit/test_waybeam_diff.cpp`:

```cpp
TEST_CASE("waybeamConfigDiff: resilience is a RESTART field, not live") {
    Config a{}, b{};
    b.video.resilience = "fpv";
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/false);
    CHECK(d.restart.at("video0.resilience") == "fpv");
    CHECK(d.live.empty());

    // Not dynamic-link-owned: still emitted while DL is enabled.
    auto d2 = waybeamConfigDiff(a, b, /*dlEnabled=*/true);
    CHECK(d2.restart.at("video0.resilience") == "fpv");
}
```

- [ ] **Step 2: Build and run to verify it fails**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests --test-case='*resilience is a RESTART*'
```
Expected: FAIL — `d.restart.at("video0.resilience")` throws `out_of_range` (key not emitted).

- [ ] **Step 3: Add the field to the RESTART section**

In `drone/src/translate/waybeam.cpp`, in `waybeamConfigDiff` after the `sensorBin` restart block (currently ending at line 131), add:

```cpp
    // resilience is a named encoder preset (intra-refresh + GOP). waybeam treats
    // it as reboot-class; we apply it by bouncing the waybeam process (RESTART).
    // Not dynamic-link-owned — the controller never writes it.
    if (va.resilience != vb.resilience)
        d.restart["video0.resilience"] = vb.resilience;
```

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests --test-case='*waybeamConfigDiff*'
```
Expected: PASS (new test + all existing diff tests).

- [ ] **Step 5: Commit**

```bash
git add drone/src/translate/waybeam.cpp drone/tests/unit/test_waybeam_diff.cpp
git commit -m "feat(drone): classify video.resilience change as waybeam RESTART

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: Validate `video.resilience` against the preset set

**Files:**
- Modify: `drone/src/config/validate.cpp:86-97` (the `// video` section)
- Test: `drone/tests/unit/test_validate.cpp`

- [ ] **Step 1: Write the failing test**

Add to `drone/tests/unit/test_validate.cpp`:

```cpp
TEST_CASE("validate: video.resilience accepts every known preset") {
    for (const char* p : {"off","rescue","quality","sprint","racing",
                          "endurance","patrol","rally","range","fpv"}) {
        Config c{}; c.video.resilience = p;
        CHECK(validate(c).empty());
    }
}

TEST_CASE("validate: video.resilience rejects an unknown preset") {
    Config c{}; c.video.resilience = "turbo";
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "video.resilience");
}
```

- [ ] **Step 2: Build and run to verify it fails**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests --test-case='*video.resilience*'
```
Expected: FAIL — the unknown-preset case finds 0 errors (`REQUIRE(errs.size() == 1)` fails); no validation exists yet.

- [ ] **Step 3: Add the validation rule**

In `drone/src/config/validate.cpp`, in the `// video` section, after the `rcMode` check (currently lines 96-97), add:

```cpp
    static const std::set<std::string> resiliencePresets{
        "off", "rescue", "quality", "sprint", "racing",
        "endurance", "patrol", "rally", "range", "fpv"};
    if (!resiliencePresets.count(c.video.resilience))
        errs.push_back({"video.resilience",
                        "must be one of off|rescue|quality|sprint|racing|"
                        "endurance|patrol|rally|range|fpv"});
```

(`<set>` is already included at the top of `validate.cpp`.)

- [ ] **Step 4: Run the test to verify it passes**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests --test-case='*validate*'
```
Expected: PASS (both new cases + all existing validate tests, including `validate: default config is valid` — default `"off"` is accepted).

- [ ] **Step 5: Commit**

```bash
git add drone/src/config/validate.cpp drone/tests/unit/test_validate.cpp
git commit -m "feat(drone): validate video.resilience against preset set

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: Regression guard — `video.resilience` is NOT DL-locked

**Files:**
- Test only: `drone/tests/unit/test_lock.cpp` (no production change — `src/config/lock.cpp` deliberately omits resilience)

- [ ] **Step 1: Write the test**

Add to `drone/tests/unit/test_lock.cpp` (the `dlOn()` helper already exists at the top of the file):

```cpp
TEST_CASE("lock: DL on + body writes video.resilience → allowed (operator-owned)") {
    auto body = nlohmann::json::parse(R"({"video":{"resilience":"fpv"}})");
    auto r = checkDynamicLinkLock(body, dlOn());
    CHECK(r.ok);
    CHECK(r.lockedPaths.empty());
}
```

- [ ] **Step 2: Build and run — it should PASS immediately**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests --test-case='*lock*'
```
Expected: PASS. `resilience` is not in `kLockedPaths` (`src/config/lock.cpp:8-21`), so the write is allowed. This test locks in the design decision and will fail loudly if someone later adds resilience to the lock list.

(This task has no RED→GREEN cycle because the desired behavior — "not locked" — is the absence of a rule. The test is a guard, not a driver. If it unexpectedly FAILS, someone has already locked the field; remove that lock entry rather than changing this test.)

- [ ] **Step 3: Commit**

```bash
git add drone/tests/unit/test_lock.cpp
git commit -m "test(drone): guard that video.resilience stays DL-unlocked

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Documentation + full-suite verification

**Files:**
- Modify: `CLAUDE.md` (the drone `fpvd` description bullet)

- [ ] **Step 1: Add a one-line note to CLAUDE.md**

In `CLAUDE.md`, in the `drone/` bullet under "What this is", append a sentence noting the new knob. Change:

```
- `drone/` — **fpvd**, C++17, runs on the OpenIPC ssc338q camera. Supervises waybeam (encoder), wfb-ng (radio), msposd/mavfwd; owns one unified config (defaults + sparse overlay, deep-merged) exposed as `GET/PATCH /config` → `POST /apply` (restarts only affected children).
```
to:
```
- `drone/` — **fpvd**, C++17, runs on the OpenIPC ssc338q camera. Supervises waybeam (encoder), wfb-ng (radio), msposd/mavfwd; owns one unified config (defaults + sparse overlay, deep-merged) exposed as `GET/PATCH /config` → `POST /apply` (restarts only affected children). The encoder's error-resilience profile is a single operator knob, `video.resilience` (waybeam preset; restart-class, never DL-locked).
```

- [ ] **Step 2: Run the full suite**

Run:
```bash
cmake --build drone/build -j && cd drone && ./build/fpvd_tests
```
Expected: all PASS (`Status: SUCCESS`). Note the total assertion/test-case counts.

- [ ] **Step 3: Cross-compile sanity (catches musl/static-only breakage early)**

Run inside the `drone/` nix-shell (per CLAUDE.md):
```bash
cmake -S drone -B drone/build/ssc338q -DCMAKE_TOOLCHAIN_FILE=drone/cmake/toolchain-ssc338q.cmake
cmake --build drone/build/ssc338q --target fpvd -j
```
Expected: `fpvd` links cleanly. (Pure config/string change — no new deps — so this should be a no-op risk, but verify.)

- [ ] **Step 4: Commit**

```bash
git add CLAUDE.md
git commit -m "docs: note video.resilience encoder preset knob

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Bench verification (manual, post-deploy — NOT a code step)

The design's open risk is whether a waybeam **process** restart applies the preset, or whether a full **device** reboot is required (waybeam's own API claims reboot-class). After `./deploy/drone/deploy.sh`:

1. `PATCH /air/config` with `{"video":{"resilience":"<preset>"}}`, then `POST /air/apply`. Confirm the apply response lists `waybeam` in `restarted`.
2. On the camera, confirm `/etc/waybeam.json` now contains `"video0": { ..., "resilience": "<preset>" }`.
3. Confirm the encoder actually adopts intra-refresh **after the process bounce alone** (no full reboot) — via waybeam logs and/or observed error-recovery behavior on the GS video.
4. With `dynamicLink.enabled = true`, confirm `PATCH /air/config {"video":{"resilience":...}}` still succeeds (not `400 dynamic_link_locked`).

If step 3 fails (preset only takes effect after a full reboot), open a follow-up to add an auto-reboot-on-resilience-change path (the "Auto-reboot the camera" option from the design); do not silently ship a knob that doesn't apply.

---

## Self-review notes

- **Spec coverage:** schema field (T1), validation strict-enum (T4), translate→waybeam.json (T2), restart-class apply classification (T3), not-DL-locked (T5), gopSize-ignored doc comment (T2), GS untouched / no deploy change (scope — nothing to do), bench-verification risk (documented). All spec sections map to a task.
- **No new types/functions introduced beyond the `resilience` field** — every reference (`c.video.resilience`, `video0.resilience`, the preset set) is defined within these tasks.
- **Backward-compat:** covered by deep-merge loader; the only break surface (strict direct-parse) is the two `test_schema.cpp` fixtures fixed in T1, with T1 Step 6 as a backstop sweep.
