# swfec Adoption Plan 2/3 — fpvd Drone Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Teach the fpvd drone daemon a `link.fec.mode: "rs"|"swfec"` switch — swfec argv (`-z`), swfec-aware dynlink compute/dispatch, hot overhead/deadline apply — and remove the now-dead block-interleaver support.

**Architecture:** `link.fec` gains `mode`/`overheadPct`/`deadlineMs`. A mode flip is a full service restart (`-z` is constructor-time); overhead/deadline changes hot-apply through the existing `CMD_SET_FEC` (k=overhead, n=deadline). Under dynamic link, `applyLocalCompute` bypasses the RS block-geometry (`computeK/N`) and puts the static swfec params in the Decision's k/n slots; bitrate budget becomes `wire_target × 100/(100+overhead)`. All interleaver code (CMD 5, `interleavingSupported`, `Decision.depth`) is deleted — the new wfb-ng base cannot speak it. Spec: `docs/superpowers/specs/2026-06-11-swfec-adoption-design.md`.

**Tech Stack:** C++17, nlohmann-json, doctest. Build: `cd drone && cmake --build build -j`. Test: `./build/fpvd_tests` **from `drone/`** (NOT ctest — test fixtures use relative paths).

**Repo/branch:** `/home/gilankpam/Projects/drone/fpvd`, branch `feat/swfec-adoption`. Task order is compile-safe: additions first (Tasks 1–6), interleaver removal last (Tasks 7–8) so every task ends green.

---

### Task 1: Schema — swfec keys on `link.fec` and `dynamicLink.safe`

**Files:**
- Modify: `drone/src/config/schema.hpp:35-36` (Fec), `:107-118` (DynamicLinkSafe)
- Modify: `drone/etc/defaults.json:7-10` (link.fec), `:67-75` (dynamicLink.safe)
- Test: `drone/tests/unit/test_schema.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `drone/tests/unit/test_schema.cpp`:

```cpp
TEST_CASE("schema: link.fec swfec keys default and roundtrip") {
    fpvd::Config c{};
    CHECK(c.link.fec.mode == "rs");
    CHECK(c.link.fec.overheadPct == 50);
    CHECK(c.link.fec.deadlineMs == 30);
    nlohmann::json j = c;
    auto back = j.get<fpvd::Config>();
    CHECK(back.link.fec.mode == "rs");
    CHECK(back.link.fec.overheadPct == 50);
    CHECK(back.link.fec.deadlineMs == 30);
}

TEST_CASE("schema: legacy fec object without swfec keys parses with defaults") {
    auto j = nlohmann::json::parse(R"({"link":{"fec":{"k":3,"n":5}}})");
    auto c = j.get<fpvd::Config>();
    CHECK(c.link.fec.k == 3);
    CHECK(c.link.fec.n == 5);
    CHECK(c.link.fec.mode == "rs");
}

TEST_CASE("schema: dynamicLink.safe swfec keys default") {
    fpvd::Config c{};
    CHECK(c.dynamicLink.safe.overheadPct == 100);
    CHECK(c.dynamicLink.safe.deadlineMs == 30);
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests -tc="schema:*"`
Expected: COMPILE FAILURE (`mode` is not a member of `Fec`) — that is the failing state for compiled languages.

- [ ] **Step 3: Implement the schema changes**

In `drone/src/config/schema.hpp`, replace the `Fec` struct (lines 35-36):

```cpp
struct Fec {
    std::string mode{"rs"};   // "rs" | "swfec" — mode flip restarts wfb_tx (-z is constructor-time)
    int k{8};                 // rs-mode block geometry
    int n{12};
    int overheadPct{50};      // swfec-mode repair budget, 0..255 (uint8 on the control wire)
    int deadlineMs{30};       // swfec-mode recovery window, 1..255
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Fec, mode, k, n, overheadPct, deadlineMs)
```

(Note the macro changes from `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` to `..._WITH_DEFAULT` so legacy configs missing the new keys still parse.)

Replace `DynamicLinkSafe` (lines 107-118) — `depth` stays for now (removed in Task 7), swfec keys added:

```cpp
struct DynamicLinkSafe {
    int mcs{1};
    int k{8};
    int n{12};
    int overheadPct{100};   // swfec-mode safe recovery: more repair at the low rung
    int deadlineMs{30};
    int depth{1};
    int bandwidth{20};
    int txPowerDbm{20};
    int bitrateKbps{2000};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkSafe, mcs, k, n,
                                               overheadPct, deadlineMs,
                                               depth, bandwidth, txPowerDbm,
                                               bitrateKbps)
```

In `drone/etc/defaults.json`, change link.fec to:

```json
    "fec": {
      "mode": "rs",
      "k": 8,
      "n": 12,
      "overheadPct": 50,
      "deadlineMs": 30
    },
```

and in `dynamicLink.safe` add `"overheadPct": 100, "deadlineMs": 30` alongside the existing keys.

- [ ] **Step 4: Run tests**

Run: `cmake --build build -j && ./build/fpvd_tests`
Expected: ALL PASS (full suite, not just schema — the WITH_DEFAULT change must not break other fixtures).

- [ ] **Step 5: Commit**

```bash
git add drone/src/config/schema.hpp drone/etc/defaults.json drone/tests/unit/test_schema.cpp
git commit -m "drone/config: link.fec mode/overheadPct/deadlineMs + safe swfec keys"
```

---

### Task 2: Validation rules

**Files:**
- Modify: `drone/src/config/validate.cpp` (after the existing `link.fec` k/n check at lines 58-61, and near the `dynamicLink.safe` checks at ~121-124)
- Test: `drone/tests/unit/test_validate.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `drone/tests/unit/test_validate.cpp` (match the file's existing helper style for finding a field in the error list; if it has none, use this lambda):

```cpp
TEST_CASE("validate: link.fec swfec rules") {
    auto hasErr = [](const std::vector<fpvd::ValidationError>& errs,
                     const std::string& path) {
        for (auto& e : errs) if (e.path == path) return true;
        return false;
    };
    fpvd::Config c{};

    SUBCASE("bad mode rejected") {
        c.link.fec.mode = "raptor";
        CHECK(hasErr(fpvd::validate(c), "link.fec.mode"));
    }
    SUBCASE("overheadPct range") {
        c.link.fec.overheadPct = 256;
        CHECK(hasErr(fpvd::validate(c), "link.fec.overheadPct"));
    }
    SUBCASE("deadlineMs range — uint8 wire cap") {
        c.link.fec.deadlineMs = 0;
        CHECK(hasErr(fpvd::validate(c), "link.fec.deadlineMs"));
        c.link.fec.deadlineMs = 256;
        CHECK(hasErr(fpvd::validate(c), "link.fec.deadlineMs"));
    }
    SUBCASE("safe swfec ranges") {
        c.dynamicLink.safe.overheadPct = -1;
        CHECK(hasErr(fpvd::validate(c), "dynamicLink.safe.overheadPct"));
        c.dynamicLink.safe = {};
        c.dynamicLink.safe.deadlineMs = 300;
        CHECK(hasErr(fpvd::validate(c), "dynamicLink.safe.deadlineMs"));
    }
    SUBCASE("valid swfec config passes") {
        c.link.fec.mode = "swfec";
        c.link.fec.overheadPct = 50;
        c.link.fec.deadlineMs = 30;
        CHECK_FALSE(hasErr(fpvd::validate(c), "link.fec.mode"));
    }
}
```

- [ ] **Step 2: Run to verify failure**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="validate:*"`
Expected: the new SUBCASEs FAIL (no rules yet).

- [ ] **Step 3: Implement**

In `drone/src/config/validate.cpp`, after the existing `link.fec` check (lines 58-61):

```cpp
    if (c.link.fec.mode != "rs" && c.link.fec.mode != "swfec")
        errs.push_back({"link.fec.mode", "must be \"rs\" or \"swfec\""});
    if (c.link.fec.overheadPct < 0 || c.link.fec.overheadPct > 255)
        errs.push_back({"link.fec.overheadPct", "must be 0..255"});
    if (c.link.fec.deadlineMs < 1 || c.link.fec.deadlineMs > 255)
        errs.push_back({"link.fec.deadlineMs", "must be 1..255 (uint8 on the control wire)"});
```

Near the existing `dynamicLink.safe` k/n checks (~lines 121-124):

```cpp
    if (c.dynamicLink.safe.overheadPct < 0 || c.dynamicLink.safe.overheadPct > 255)
        errs.push_back({"dynamicLink.safe.overheadPct", "must be 0..255"});
    if (c.dynamicLink.safe.deadlineMs < 1 || c.dynamicLink.safe.deadlineMs > 255)
        errs.push_back({"dynamicLink.safe.deadlineMs", "must be 1..255"});
```

- [ ] **Step 4: Run tests** — `./build/fpvd_tests` → ALL PASS.

- [ ] **Step 5: Commit**

```bash
git add drone/src/config/validate.cpp drone/tests/unit/test_validate.cpp
git commit -m "drone/config: validate swfec mode/overhead/deadline ranges"
```

---

### Task 3: Video TX argv — `-z` in swfec mode

**Files:**
- Modify: `drone/src/translate/wfb.cpp:6-20` (`commonTx`)
- Test: `drone/tests/unit/test_translate_wfb.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `drone/tests/unit/test_translate_wfb.cpp`:

```cpp
TEST_CASE("translate.wfb: video tx argv in swfec mode") {
    Config c{};
    c.link.fec.mode = "swfec";
    c.link.fec.overheadPct = 60;
    c.link.fec.deadlineMs = 25;
    auto a = wfbArgs(c, fpvd::WfbRole::VideoTx, "wlan0", "/etc/drone.key");
    auto at = [&](const std::string& flag){
        auto it = std::find(a.begin(), a.end(), flag);
        REQUIRE(it != a.end());
        return *(it + 1);
    };
    CHECK(std::find(a.begin(), a.end(), "-z") != a.end());
    CHECK(at("-k") == "60");   // overhead_pct rides -k
    CHECK(at("-n") == "25");   // deadline_ms rides -n
}

TEST_CASE("translate.wfb: tun/tlm tx stay RS even in swfec mode") {
    Config c{};
    c.link.fec.mode = "swfec";
    for (auto role : {fpvd::WfbRole::TunTx, fpvd::WfbRole::TlmTx}) {
        auto tx = wfbArgs(c, role, "wlan0", "/etc/drone.key");
        CHECK(std::find(tx.begin(), tx.end(), "-z") == tx.end());
    }
}
```

- [ ] **Step 2: Run to verify failure** — `./build/fpvd_tests -tc="translate.wfb:*"` → the swfec argv test FAILS (no `-z`).

- [ ] **Step 3: Implement**

Replace `commonTx` in `drone/src/translate/wfb.cpp`:

```cpp
static std::vector<std::string> commonTx(const Config& c, int mcs,
                                          const std::string& /*iface*/,
                                          const std::string& key) {
    std::vector<std::string> a = {
        "/usr/bin/wfb_tx",
        "-K", key,
        "-M", std::to_string(mcs),
        "-B", std::to_string(modulationWidth(c.link.width)),
    };
    if (c.link.fec.mode == "swfec") {
        // swfec repurposes -k/-n: overhead_pct / deadline_ms (see spec).
        a.push_back("-z");
        a.push_back("-k"); a.push_back(std::to_string(c.link.fec.overheadPct));
        a.push_back("-n"); a.push_back(std::to_string(c.link.fec.deadlineMs));
    } else {
        a.push_back("-k"); a.push_back(std::to_string(c.link.fec.k));
        a.push_back("-n"); a.push_back(std::to_string(c.link.fec.n));
    }
    a.insert(a.end(), {
        "-S", c.link.stbc ? "1" : "0",
        "-L", c.link.ldpc ? "1" : "0",
        "-i", std::to_string(c.link.linkId)
    });
    return a;
}
```

(`commonTx` is only called for `VideoTx`; `tunTlmTx` is a separate builder and stays RS — the second test pins that.)

- [ ] **Step 4: Run tests** — `./build/fpvd_tests` → ALL PASS.

- [ ] **Step 5: Commit**

```bash
git add drone/src/translate/wfb.cpp drone/tests/unit/test_translate_wfb.cpp
git commit -m "drone/translate: spawn video wfb_tx with -z overhead/deadline in swfec mode"
```

---

### Task 4: Config-apply routing — mode flip restarts, params hot-apply

**Files:**
- Modify: `drone/src/config/diff.cpp:39-57` (`classifyLinkChange`)
- Modify: `drone/src/daemon.cpp:451-460` (videoFec hot-apply)
- Test: `drone/tests/unit/test_link_classify.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `drone/tests/unit/test_link_classify.cpp`:

```cpp
TEST_CASE("classifyLinkChange: fec mode flip -> fullRestart, not videoFec") {
    Config a{}, b{}; b.link.fec.mode = "swfec";
    auto c = classifyLinkChange(a, b);
    CHECK(c.fullRestart);
    CHECK_FALSE(c.videoFec);
}

TEST_CASE("classifyLinkChange: swfec param change -> videoFec only") {
    Config a{}, b{};
    a.link.fec.mode = b.link.fec.mode = "swfec";
    b.link.fec.overheadPct = 80;
    auto c = classifyLinkChange(a, b);
    CHECK(c.videoFec);
    CHECK_FALSE(c.fullRestart);
}

TEST_CASE("classifyLinkChange: rs k/n change ignored under swfec mode") {
    Config a{}, b{};
    a.link.fec.mode = b.link.fec.mode = "swfec";
    b.link.fec.k = 3;  // rs-only knob; meaningless in swfec mode
    auto c = classifyLinkChange(a, b);
    CHECK_FALSE(c.videoFec);
    CHECK_FALSE(c.fullRestart);
}
```

- [ ] **Step 2: Run to verify failure** — `./build/fpvd_tests -tc="classifyLinkChange*"` → new cases FAIL.

- [ ] **Step 3: Implement diff.cpp**

In `classifyLinkChange` (`drone/src/config/diff.cpp`), replace the `videoFec`/`fullRestart` lines (51-53):

```cpp
    const bool fecMode = la.fec.mode != lb.fec.mode;
    // Per-mode param diff: only the active mode's knobs are live.
    c.videoFec      = !fecMode && (lb.fec.mode == "swfec"
                          ? (la.fec.overheadPct != lb.fec.overheadPct) ||
                            (la.fec.deadlineMs  != lb.fec.deadlineMs)
                          : (la.fec.k != lb.fec.k) || (la.fec.n != lb.fec.n));
    c.fullRestart   = (la.linkId != lb.linkId) ||
                      (la.wlanAdapter != lb.wlanAdapter) ||
                      fecMode;  // -z is constructor-time: wfb_tx must respawn
```

- [ ] **Step 4: Implement daemon.cpp hot-apply**

Replace the `link.videoFec` block (`drone/src/daemon.cpp:451-460`):

```cpp
        if (link.videoFec) {
            WfbControlClient cli("127.0.0.1", kVideoControlPort);
            const auto& f = effective_.link.fec;
            // swfec rides the same CMD_SET_FEC: k=overhead_pct, n=deadline_ms.
            auto rr = (f.mode == "swfec")
                ? cli.setFec(static_cast<uint8_t>(f.overheadPct),
                             static_cast<uint8_t>(f.deadlineMs))
                : cli.setFec(static_cast<uint8_t>(f.k),
                             static_cast<uint8_t>(f.n));
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("fec: ") + rr.error};
                return {false, {}, restarted, rr.error, version_};
            }
        }
```

(Note: under dynamic link, `link.fec` is locked — `drone/src/config/lock.cpp:10` — so this path only runs with DL disabled; the lock list needs no change.)

- [ ] **Step 5: Run tests** — `cmake --build build -j && ./build/fpvd_tests` → ALL PASS.

- [ ] **Step 6: Commit**

```bash
git add drone/src/config/diff.cpp drone/src/daemon.cpp drone/tests/unit/test_link_classify.cpp
git commit -m "drone/config: fec mode flip -> full restart; per-mode hot fec apply"
```

---

### Task 5: Bitrate math — `computeBitrateKbpsSwfec`

**Files:**
- Modify: `drone/src/dynlink/bitrate.hpp`, `drone/src/dynlink/bitrate.cpp`
- Test: `drone/tests/unit/test_dl_bitrate.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `drone/tests/unit/test_dl_bitrate.cpp`:

```cpp
TEST_CASE("computeBitrateKbpsSwfec: 50% overhead = 2/3 of wire target") {
    // 9000 * 100/150 = 6000, exact
    CHECK(fpvd::dynlink::computeBitrateKbpsSwfec(9000.0, 50, 1000, 24000) == 6000);
}

TEST_CASE("computeBitrateKbpsSwfec: 0% overhead passes wire target through") {
    CHECK(fpvd::dynlink::computeBitrateKbpsSwfec(8000.0, 0, 1000, 24000) == 8000);
}

TEST_CASE("computeBitrateKbpsSwfec: truncates toward zero like RS path") {
    // 10000 * 100/130 = 7692.3 -> 7692
    CHECK(fpvd::dynlink::computeBitrateKbpsSwfec(10000.0, 30, 1000, 24000) == 7692);
}

TEST_CASE("computeBitrateKbpsSwfec: clamps to min and max") {
    CHECK(fpvd::dynlink::computeBitrateKbpsSwfec(900.0, 50, 1000, 24000) == 1000);
    CHECK(fpvd::dynlink::computeBitrateKbpsSwfec(90000.0, 50, 1000, 24000) == 24000);
}
```

- [ ] **Step 2: Run to verify failure** — compile failure (function undeclared).

- [ ] **Step 3: Implement**

`drone/src/dynlink/bitrate.hpp` — add below `computeBitrateKbps`:

```cpp
// swfec: sources own wire_target * 100/(100+overheadPct) of the budget
// (repairs are extra packets on top of sources — same airtime semantics as
// the RS n/k de-rate). Same truncation + clamping as computeBitrateKbps.
uint16_t computeBitrateKbpsSwfec(double wireTargetKbps, int overheadPct,
                                 int minKbps, int maxKbps);
```

`drone/src/dynlink/bitrate.cpp` — add:

```cpp
uint16_t computeBitrateKbpsSwfec(double wireTargetKbps, int overheadPct,
                                 int minKbps, int maxKbps) {
    if (overheadPct < 0) overheadPct = 0;
    // Identical math to the RS formula with k=100, n=100+overhead.
    return computeBitrateKbps(wireTargetKbps, 100, 100 + overheadPct,
                              minKbps, maxKbps);
}
```

- [ ] **Step 4: Run tests** — `./build/fpvd_tests` → ALL PASS.

- [ ] **Step 5: Commit**

```bash
git add drone/src/dynlink/bitrate.hpp drone/src/dynlink/bitrate.cpp drone/tests/unit/test_dl_bitrate.cpp
git commit -m "drone/dynlink: swfec bitrate budget (100/(100+overhead) of wire target)"
```

---

### Task 6: Dynlink snapshot + local compute + dispatch in swfec mode

**Files:**
- Modify: `drone/src/dynlink/runtime_config.hpp:31-52` (DlRuntimeConfig), `:10-18` (SafeDefaults), `drone/src/dynlink/runtime_config.cpp` (`buildDlSnapshot`)
- Modify: `drone/src/dynlink/local_compute.cpp:10-32`
- Modify: `drone/src/dynlink/controller.cpp:211-236` (`dispatchTxSafe`)
- Test: `drone/tests/unit/test_dl_runtime_config.cpp`, `drone/tests/unit/test_dl_local_compute.cpp`, `drone/tests/integration/test_dl_controller.cpp`

- [ ] **Step 1: Write the failing tests**

Append to `drone/tests/unit/test_dl_runtime_config.cpp`:

```cpp
TEST_CASE("buildDlSnapshot: swfec fields from link.fec + safe") {
    fpvd::Config c{};
    c.link.fec.mode = "swfec";
    c.link.fec.overheadPct = 70;
    c.link.fec.deadlineMs = 40;
    c.dynamicLink.safe.overheadPct = 120;
    c.dynamicLink.safe.deadlineMs = 35;
    auto s = fpvd::dynlink::buildDlSnapshot(c, "wlan0");
    CHECK(s.swfec);
    CHECK(s.swfecOverheadPct == 70);
    CHECK(s.swfecDeadlineMs == 40);
    CHECK(s.safe.overheadPct == 120);
    CHECK(s.safe.deadlineMs == 35);
}

TEST_CASE("buildDlSnapshot: rs mode -> swfec false") {
    fpvd::Config c{};
    auto s = fpvd::dynlink::buildDlSnapshot(c, "wlan0");
    CHECK_FALSE(s.swfec);
}
```

Append to `drone/tests/unit/test_dl_local_compute.cpp` (reuse the file's existing `cfgWithBitrate()` helper):

```cpp
TEST_CASE("applyLocalCompute swfec: k/n slots carry overhead/deadline, bitrate de-rated") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    cfg.swfec = true;
    cfg.swfecOverheadPct = 50;
    cfg.swfecDeadlineMs = 30;
    Decision d{};
    d.mcs = 2; d.bandwidth = 20;
    applyLocalCompute(cfg, d);
    CHECK(d.k == 50);   // overhead_pct
    CHECK(d.n == 30);   // deadline_ms
    // Cross-check the bitrate against the swfec formula at the same wire target.
    double probeKbps = fpvd::kProbePps * fpvd::kProbePacketBytes * 8.0 / 1000.0;
    double wt = computeWireTargetKbps(20, 2, cfg.probeMcsCeiling, probeKbps);
    CHECK(d.bitrateKbps == computeBitrateKbpsSwfec(wt, 50,
            cfg.bitrate.minBitrateKbps, cfg.bitrate.maxBitrateKbps));
}
```

In `drone/tests/integration/test_dl_controller.cpp`, add an end-to-end swfec case modeled exactly on the existing `"controller applies a decision and trips watchdog to safe"` test (line 214 — same `FakeWfbTx`/`FakeEnc`/`Endpoints`/`waitFor` harness):

```cpp
TEST_CASE("controller swfec mode: decision + safe push carry overhead/deadline") {
    FakeWfbTx wfb;
    FakeEnc enc;

    Endpoints ep;
    ep.listenAddr = "127.0.0.1";
    ep.listenPort = 45807;                 // unique fixed test port
    ep.wfbCtlAddr = "127.0.0.1";
    ep.wfbCtlPort = wfb.port;
    ep.encHost    = "127.0.0.1";
    ep.encPort    = static_cast<uint16_t>(enc.port);
    ep.idrPort    = 0;
    ep.gsTunnelPort = 0;
    ep.osdMsgPath = "/tmp/fpvd_test_osd_swfec.msg";
    ep.osdUpdateIntervalMs = 1000;

    DlRuntimeConfig snap{};
    snap.healthTimeoutMs  = 300;            // watchdog trips fast
    snap.minIdrIntervalMs = 500;
    snap.applyStaggerMs   = 0;
    snap.applySubPaceMs   = 0;
    snap.osdEnabled = false; snap.osdDebugLatency = false; snap.debug = false;
    snap.roiQp = RoiCurve{6000, 2000, -24, 3};
    snap.stbc = true; snap.ldpc = true;
    snap.linkBandwidth = 40;
    snap.iface = "wlan-test-nonexistent";
    snap.swfec = true;
    snap.swfecOverheadPct = 50;
    snap.swfecDeadlineMs  = 30;
    snap.safe = SafeDefaults{/*mcs=*/1, /*k=*/8, /*n=*/12, /*depth=*/0,
                             /*bandwidth=*/20, /*txPowerDbm=*/5,
                             /*bitrateKbps=*/2000};
    snap.safe.overheadPct = 100;            // trailing NSDMI fields, set by name
    snap.safe.deadlineMs  = 35;

    DynamicLinkController c(ep);
    c.start(snap);

    Decision d{};
    d.magic = kWireMagic; d.version = kWireVersion;
    d.sequence = 100; d.timestampMs = 1;
    d.mcs = 7; d.bandwidth = 20; d.txPowerDbm = 10;
    d.k = 4; d.n = 6; d.bitrateKbps = 6000; d.fps = 60;

    // Decision dispatch: the k/n fec slots must carry the STATIC swfec
    // params from the snapshot, never the wire's k/n or RS block math.
    CHECK(waitFor([&] {
        sendDecision(ep.listenPort, d);
        return wfb.sawFec(50, 30);
    }, 1000));

    // Watchdog silence -> safe push uses safe.overheadPct/deadlineMs.
    CHECK(waitFor([&] { return wfb.sawFec(100, 35); }, 2000));

    c.stop();
}
```

(Until Task 7 removes it, `Decision.depth`/`SafeDefaults.depth` still exist — the positional `SafeDefaults{...}` above keeps depth at 0, and the new fields are appended with NSDMIs and set by name, so the file's three existing positional `SafeDefaults{1, 8, 12, 0, 20, 5, 2000}` aggregates keep compiling untouched.)

- [ ] **Step 2: Run to verify failure** — compile failure (`swfec` not a member of `DlRuntimeConfig`).

- [ ] **Step 3: Implement runtime_config**

`drone/src/dynlink/runtime_config.hpp` — in `DlRuntimeConfig`, after `interleavingSupported` (removed in Task 7), add:

```cpp
    bool    swfec{false};        // link.fec.mode == "swfec"
    uint8_t swfecOverheadPct{50};
    uint8_t swfecDeadlineMs{30};
```

In `SafeDefaults`, append at the END (after `bitrateKbps`, with NSDMIs — this keeps the existing positional `SafeDefaults{...}` aggregates in `runtime_config.cpp` and the controller tests compiling unchanged; `depth` stays until Task 7):

```cpp
    uint8_t  overheadPct{100};   // swfec-mode safe recovery params
    uint8_t  deadlineMs{30};
```

`drone/src/dynlink/runtime_config.cpp` — in `buildDlSnapshot`, after the existing `s.safe = SafeDefaults{...};` aggregate add:

```cpp
    s.safe.overheadPct = static_cast<uint8_t>(dl.safe.overheadPct);
    s.safe.deadlineMs  = static_cast<uint8_t>(dl.safe.deadlineMs);

    s.swfec            = (c.link.fec.mode == "swfec");
    s.swfecOverheadPct = static_cast<uint8_t>(c.link.fec.overheadPct);
    s.swfecDeadlineMs  = static_cast<uint8_t>(c.link.fec.deadlineMs);
```

- [ ] **Step 4: Implement local_compute**

Replace the body of `applyLocalCompute` (`drone/src/dynlink/local_compute.cpp:10-32`):

```cpp
void applyLocalCompute(const DlRuntimeConfig& cfg, Decision& d) {
    const BitrateEngineConfig& b = cfg.bitrate;
    // The probe runs FEC-off at a fixed rate; its true on-air kbps.
    double probeKbps =
        static_cast<double>(fpvd::kProbePps) * fpvd::kProbePacketBytes * 8.0 / 1000.0;
    double wireTarget =
        computeWireTargetKbps(d.bandwidth, d.mcs, cfg.probeMcsCeiling, probeKbps);
    auto sat8 = [](int x) -> uint8_t {
        if (x < 0) x = 0;
        if (x > 255) x = 255;
        return static_cast<uint8_t>(x);
    };
    if (cfg.swfec) {
        // swfec: FEC is static config — the k/n Decision slots carry
        // overhead_pct/deadline_ms (pushed via CMD_SET_FEC on change only).
        d.k = cfg.swfecOverheadPct;
        d.n = cfg.swfecDeadlineMs;
        d.bitrateKbps = computeBitrateKbpsSwfec(wireTarget, cfg.swfecOverheadPct,
                                                b.minBitrateKbps, b.maxBitrateKbps);
    } else {
        int k = computeK(wireTarget, b.mtuBytes, b.fps,
                         b.baseRedundancyRatio, b.blocksPerFrame, b.kMin, b.kMax);
        int n = computeN(k, b.baseRedundancyRatio);
        d.k = sat8(k);
        d.n = sat8(n);
        d.bitrateKbps = computeBitrateKbps(wireTarget, k, n,
                                           b.minBitrateKbps, b.maxBitrateKbps);
    }
    d.depth       = kInterleaveDepth;   // removed in Task 7
    d.fps         = sat8(b.fps);
    d.txPowerDbm  = txpowerDbmForMcs(d.mcs);
}
```

(Add `#include "dynlink/bitrate.hpp"` if not already present — it is, for computeK callers.)

- [ ] **Step 5: Implement dispatchTxSafe per-mode fec**

In `drone/src/dynlink/controller.cpp` (`dispatchTxSafe`, line ~213), replace `wfb_->setFec(cfg.safe.k, cfg.safe.n);` with:

```cpp
    if (cfg.swfec) wfb_->setFec(cfg.safe.overheadPct, cfg.safe.deadlineMs);
    else           wfb_->setFec(cfg.safe.k, cfg.safe.n);
```

(`dispatchTxApply` needs no mode branch: it diffs and pushes `d.k/d.n`, which local compute already filled per-mode.)

- [ ] **Step 6: Run tests** — `cmake --build build -j && ./build/fpvd_tests` → ALL PASS.

- [ ] **Step 7: Commit**

```bash
git add drone/src/dynlink/runtime_config.hpp drone/src/dynlink/runtime_config.cpp \
        drone/src/dynlink/local_compute.cpp drone/src/dynlink/controller.cpp \
        drone/tests/unit/test_dl_runtime_config.cpp drone/tests/unit/test_dl_local_compute.cpp \
        drone/tests/integration/test_dl_controller.cpp
git commit -m "drone/dynlink: swfec-mode local compute, snapshot fields, safe dispatch"
```

---

### Task 7: Remove the block interleaver (protocol + schema + Decision.depth)

The new wfb-ng base has no `CMD_SET_INTERLEAVE_DEPTH` (=5); every remnant goes. Complete removal site list (from the audit):

**Files:**
- Modify: `drone/src/translate/wfb_cmd.h:9,26` — delete `kWfbCmdSetInterleaveDepth` and the `set_interleave_depth` / `get_interleave_depth` union members (both req and resp structs)
- Modify: `drone/src/translate/wfb_control.hpp:27`, `wfb_control.cpp:102-111` — delete `setInterleaveDepth`
- Modify: `drone/src/dynlink/controller.cpp:172-176` (apply-path depth block) and `:214-217` (safe-path depth block) — delete both `setInterleaveDepth` dispatches and the `cfg.interleavingSupported` conditions around them
- Modify: `drone/src/dynlink/local_compute.hpp:11-13` — delete `kInterleaveDepth`; `local_compute.cpp` — delete the `d.depth = kInterleaveDepth;` line
- Modify: `drone/src/dynlink/wire.hpp:12-26` — delete `uint8_t depth{};` from `Decision`
- Modify: `drone/src/dynlink/runtime_config.hpp` — delete `bool interleavingSupported;` from `DlRuntimeConfig` and `uint8_t depth;` from `SafeDefaults`; `runtime_config.cpp` — delete the two corresponding copies in `buildDlSnapshot`
- Modify: `drone/src/config/schema.hpp:157,169` — delete `interleavingSupported` from `DynamicLink` (struct + macro); `:107-118` — delete `depth` from `DynamicLinkSafe` (struct + macro)
- Modify: `drone/etc/defaults.json` — delete `"interleavingSupported": true` and `"safe".."depth": 1`
- Tests: `drone/tests/unit/test_wfb_control.cpp` (delete the 3 `setInterleaveDepth` cases), `drone/tests/unit/test_dl_local_compute.cpp` (delete `CHECK(d.depth == kInterleaveDepth)` and any `depth` references), `drone/tests/unit/test_dl_runtime_config.cpp` (delete `interleavingSupported`/`depth` assertions and assignments), `drone/tests/integration/test_dl_controller.cpp` (delete the mock's `kWfbCmdSetInterleaveDepth` parsing + `depth` capture vector + related assertions)

- [ ] **Step 1: Write the regression assertion first**

In `drone/tests/integration/test_dl_controller.cpp`, the `FakeWfbTx` mock (line ~100-110) handles cmds in an if/else chain ending with `kWfbCmdSetInterleaveDepth`. Replace that final branch with an unknown-cmd counter (doctest assertions can't run on the mock's serve thread, so count and assert from the test thread):

```cpp
// In FakeWfbTx's member list (next to `std::vector<uint8_t> depth;`, which
// this task deletes):
std::atomic<int> unknownCmds{0};

// In serve(), replacing the `else if (req.cmd_id == fpvd::kWfbCmdSetInterleaveDepth)`
// branch:
} else {
    // interleave (cmd 5) is retired; any unlisted cmd is a regression.
    unknownCmds.fetch_add(1);
}
```

Then add `CHECK(wfb.unknownCmds.load() == 0);` just before `c.stop();` in BOTH the existing `"controller applies a decision and trips watchdog to safe"` test and the new swfec test from Task 6 — together they prove neither the apply path nor the safe path ever emits cmd 5.

- [ ] **Step 2: Delete all the sites listed above**

Work top-down: wire structs (`wfb_cmd.h`), client (`wfb_control.*`), controller dispatches, local compute, Decision, runtime config, schema, defaults.json, then tests. Mechanical follow-ups the compiler will demand:

- Every positional `SafeDefaults{...}` aggregate loses its `/*depth=*/` element — find them with `grep -rn 'SafeDefaults{' drone/` (three in `test_dl_controller.cpp` like `SafeDefaults{1, 8, 12, 0, 20, 5, 2000}` → `SafeDefaults{1, 8, 12, 20, 5, 2000}`, plus the Task 6 swfec test and the aggregate in `runtime_config.cpp`).
- Every `snap.interleavingSupported = ...` assignment in tests goes — `grep -rn interleavingSupported drone/tests`.
- Every `d.depth = ...` / `CHECK(d.depth ...)` on `Decision` goes — `grep -rn '\.depth' drone/src drone/tests`.

After the deletions run:

```
grep -rni interleave drone/src drone/tests drone/etc
```

Expected: ZERO hits.

- [ ] **Step 3: Build + run full suite**

Run: `cmake --build build -j && ./build/fpvd_tests`
Expected: ALL PASS, including the new `default: FAIL` guard in the controller integration tests.

- [ ] **Step 4: Commit**

```bash
git add -A drone/
git commit -m "drone: retire block-interleaver support (CMD 5, interleavingSupported, Decision.depth)"
```

---

### Task 8: Full verification + live-config sanity

- [ ] **Step 1: Full clean build + suite**

Run: `cd drone && cmake --build build -j && ./build/fpvd_tests`
Expected: ALL PASS.

- [ ] **Step 2: Repo-root config sanity**

The live snapshot `config.drone.json` (repo root) contains `link.fec.{k:3,n:5}` and a `dynamicLink` block — confirm it still parses + validates with the new schema by running the schema/validate test suite (covered by the legacy-parse test in Task 1) and eyeballing that it contains no `interleavingSupported`/`depth` keys (stale keys are ignored by WITH_DEFAULT parsing, so a leftover is harmless but worth noting in the PR).

- [ ] **Step 3: Commit any stragglers + push**

```bash
git push -u origin feat/swfec-adoption
```
