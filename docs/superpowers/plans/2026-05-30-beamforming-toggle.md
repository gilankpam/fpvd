# fpvd Beamforming Toggle + 10MHz Width Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Add a `link.beamforming` on/off toggle that drives the rtl88x2eu monitor-mode beamforming sounding loop in-process, plus first-class `link.width = 10` support.

**Architecture:** A new daemon-owned `BeamformingController` runs a worker thread that writes the driver's `/proc/net/rtl88x2eu/<iface>/bf_monitor_*` nodes in a loop, reconciled after every radio bring-up. It lives outside the `Orchestrator` (which only supervises exec'd processes and is rebuilt each apply). 10MHz is realized as 20MHz modulation on an underclocked channel, centralized in a `modulationWidth()` helper.

**Tech Stack:** C++17, nlohmann/json (vendored), doctest (vendored), CMake. POSIX threads/procfs.

**Spec:** `docs/superpowers/specs/2026-05-30-beamforming-toggle-design.md`

**Deviations from spec (with rationale):**
- The spec lists a BF rule "`link.mcs` in 0..9". The existing global rule already enforces `link.mcs` in `0..7` (a subset), so a BF-specific 0..9 check would be **unreachable dead code**. It is omitted; the global rule already guarantees BF's NSS1 range.
- The spec showed `resolveLocalMac(iface)` called from the daemon. Resolution is moved **into the controller** (it owns the procfs/sysfs base paths), so `BfParams` carries no `localMac`. The daemon stays path-agnostic and the resolver is independently testable.

---

## File Structure

**Create:**
- `src/link_width.hpp` — header-only `modulationWidth()` shared by wfb + BF.
- `src/supervise/beamforming.hpp` / `.cpp` — `BeamformingController`, `BfParams`, `BfState`, `BfStatus`, `resolveLocalMac()`.
- `tests/unit/test_link_width.cpp` — `modulationWidth()` cases.
- `tests/integration/test_beamforming.cpp` — controller + resolver against a faked procfs/sysfs.

**Modify:**
- `src/config/schema.hpp` — `Beamforming` struct + `Link` field.
- `etc/defaults.json` and `tests/fixtures/defaults.json` — default `link.beamforming` block.
- `src/config/validate.cpp` — BF rules; widen `link.width` and `dynamicLink.safe.bandwidth` to allow 10.
- `src/translate/wfb.cpp` — `-B` via `modulationWidth`.
- `src/daemon.hpp` / `src/daemon.cpp` — own `bf_`, `reconcileBeamforming()`, call sites, `"beamforming"` in `restarted`.
- `src/status.cpp` — `beamforming` status block.
- `scripts/radio-up.sh` — width `case` incl. 10MHz channel.
- `CMakeLists.txt` — add new `.cpp` to `fpvd_core` and new test files to `fpvd_tests`.
- `tests/unit/test_validate.cpp`, `tests/unit/test_translate_wfb.cpp`, `tests/integration/test_daemon.cpp` — new coverage.

---

## Task 1: Config schema + defaults

**Files:**
- Modify: `src/config/schema.hpp`
- Modify: `etc/defaults.json`, `tests/fixtures/defaults.json`
- Test: `tests/unit/test_schema.cpp`

- [ ] **Step 1: Write the failing test**

Add to `tests/unit/test_schema.cpp`:

```cpp
TEST_CASE("schema: beamforming defaults and round-trip") {
    fpvd::Config c{};
    CHECK(c.link.beamforming.enabled == false);
    CHECK(c.link.beamforming.remoteMac == "");
    CHECK(c.link.beamforming.ackTimeout == 255);
    CHECK(c.link.beamforming.intervalMs == 100);

    // Round-trips through JSON.
    nlohmann::json j = c;
    auto c2 = j.get<fpvd::Config>();
    CHECK(c2.link.beamforming.ackTimeout == 255);

    // Overlay predating the key still parses (WITH_DEFAULT).
    nlohmann::json old = {{"link", {{"channel", 149}}}};
    auto c3 = old.get<fpvd::Config>();
    CHECK(c3.link.beamforming.enabled == false);
    CHECK(c3.link.channel == 149);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="schema: beamforming defaults and round-trip"`
Expected: compile error — `link` has no member `beamforming`.

- [ ] **Step 3: Implement the schema change**

In `src/config/schema.hpp`, add the struct immediately above `struct Link`:

```cpp
struct Beamforming {
    bool enabled{false};
    std::string remoteMac{};   // ground-station eFuse MAC, required when enabled
    int ackTimeout{255};       // 33..255 us
    int intervalMs{100};       // sounding cadence
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Beamforming, enabled, remoteMac,
                                                ackTimeout, intervalMs)
```

Add the field to `struct Link` (after `wlanAdapter`):

```cpp
    std::optional<std::string> wlanAdapter{};
    Beamforming beamforming{};
```

Update the `Link` macro to include the new field:

```cpp
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Link, channel, width, txpower, mcs, fec,
                                   stbc, ldpc, linkId, mtu, wlanAdapter,
                                   beamforming)
```

- [ ] **Step 4: Add the default to both defaults files**

In `etc/defaults.json` and `tests/fixtures/defaults.json`, inside the `"link"` object, add after `"wlanAdapter": null,` (ensure the preceding line keeps/gets its trailing comma):

```json
    "wlanAdapter": null,
    "beamforming": { "enabled": false, "remoteMac": "", "ackTimeout": 255, "intervalMs": 100 }
```

- [ ] **Step 5: Run test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="schema: beamforming defaults and round-trip"`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/config/schema.hpp etc/defaults.json tests/fixtures/defaults.json tests/unit/test_schema.cpp
git commit -m "feat(schema): add link.beamforming config block"
```

---

## Task 2: `modulationWidth()` helper, width validation, wfb `-B`

**Files:**
- Create: `src/link_width.hpp`
- Modify: `src/config/validate.cpp:43-44`, `src/config/validate.cpp:103-104`
- Modify: `src/translate/wfb.cpp` (`commonTx`)
- Test: `tests/unit/test_link_width.cpp` (new), `tests/unit/test_validate.cpp`, `tests/unit/test_translate_wfb.cpp`
- Modify: `CMakeLists.txt` (register `tests/unit/test_link_width.cpp`)

- [ ] **Step 1: Write the failing helper test**

Create `tests/unit/test_link_width.cpp`:

```cpp
#include "doctest.h"
#include "link_width.hpp"

TEST_CASE("link_width: modulationWidth maps 10 to 20, others unchanged") {
    CHECK(fpvd::modulationWidth(10) == 20);
    CHECK(fpvd::modulationWidth(20) == 20);
    CHECK(fpvd::modulationWidth(40) == 40);
}
```

Register it in `CMakeLists.txt` under the `fpvd_tests` `target_sources` list (alongside the other `tests/unit/*.cpp`):

```cmake
    tests/unit/test_link_width.cpp
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j && ./build/fpvd_tests -tc="link_width: modulationWidth maps 10 to 20, others unchanged"`
Expected: compile error — `link_width.hpp` not found.

- [ ] **Step 3: Create the helper**

Create `src/link_width.hpp`:

```cpp
#pragma once

namespace fpvd {

// This driver realizes a 10MHz channel by underclocking the baseband while
// keeping 20MHz modulation. So the modulation width used by wfb_tx (-B) and
// the BF sounding frame is 20 for a 10MHz link; 20/40 pass through unchanged.
inline int modulationWidth(int linkWidth) {
    return linkWidth == 10 ? 20 : linkWidth;
}

} // namespace fpvd
```

- [ ] **Step 4: Run helper test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="link_width: modulationWidth maps 10 to 20, others unchanged"`
Expected: PASS.

- [ ] **Step 5: Write the failing validation test**

Replace the existing `TEST_CASE("validate: width must be 20 or 40")` in `tests/unit/test_validate.cpp` with:

```cpp
TEST_CASE("validate: width must be 10, 20, or 40") {
    Config c{}; c.link.width = 80;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "link.width");

    Config ok{}; ok.link.width = 10;
    CHECK(validate(ok).empty());
}
```

- [ ] **Step 6: Run validation test to verify it fails**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="validate: width must be 10, 20, or 40"`
Expected: FAIL — `width=10` currently produces a `link.width` error.

- [ ] **Step 7: Widen the width validations**

In `src/config/validate.cpp`, replace lines 43-44:

```cpp
    if (c.link.width != 10 && c.link.width != 20 && c.link.width != 40)
        errs.push_back({"link.width", "must be 10, 20, or 40"});
```

And replace the `dynamicLink.safe.bandwidth` check (lines 103-104):

```cpp
        if (dl.safe.bandwidth != 10 && dl.safe.bandwidth != 20 &&
            dl.safe.bandwidth != 40)
            errs.push_back({"dynamicLink.safe.bandwidth", "must be 10, 20, or 40"});
```

- [ ] **Step 8: Run validation test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="validate: width must be 10, 20, or 40"`
Expected: PASS.

- [ ] **Step 9: Write the failing wfb test**

Add to `tests/unit/test_translate_wfb.cpp`:

```cpp
TEST_CASE("translate.wfb: width=10 injects -B 20 (modulation width)") {
    Config c{}; c.link.width = 10;
    auto a = wfbArgs(c, fpvd::WfbRole::VideoTx, "wlan0", "/etc/drone.key");
    auto idx = std::find(a.begin(), a.end(), "-B");
    REQUIRE(idx != a.end());
    CHECK(*(idx + 1) == "20");
}
```

- [ ] **Step 10: Run wfb test to verify it fails**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="translate.wfb: width=10 injects -B 20 (modulation width)"`
Expected: FAIL — `-B` is `10`.

- [ ] **Step 11: Use modulationWidth in wfb commonTx**

In `src/translate/wfb.cpp`, add the include near the top:

```cpp
#include "translate/wfb.hpp"
#include "link_width.hpp"
```

In `commonTx`, change the `-B` argument:

```cpp
        "-B", std::to_string(modulationWidth(c.link.width)),
```

- [ ] **Step 12: Run the full suite to verify no regressions**

Run: `cmake --build build -j && ./build/fpvd_tests`
Expected: PASS (all cases, including the unchanged `video tx argv` case where width=20 ⇒ `-B 20`).

- [ ] **Step 13: Commit**

```bash
git add src/link_width.hpp src/config/validate.cpp src/translate/wfb.cpp \
        tests/unit/test_link_width.cpp tests/unit/test_validate.cpp \
        tests/unit/test_translate_wfb.cpp CMakeLists.txt
git commit -m "feat(link): support 10MHz width via modulationWidth helper"
```

---

## Task 3: Beamforming validation rules

**Files:**
- Modify: `src/config/validate.cpp` (link section)
- Test: `tests/unit/test_validate.cpp`

- [ ] **Step 1: Write the failing tests**

Add to `tests/unit/test_validate.cpp`:

```cpp
TEST_CASE("validate: beamforming off ignores stale fields") {
    Config c{};
    c.link.beamforming.enabled = false;
    c.link.beamforming.remoteMac = "";   // empty is fine when disabled
    c.link.stbc = true;                  // irrelevant when disabled
    CHECK(validate(c).empty());
}

TEST_CASE("validate: beamforming on requires stbc off") {
    Config c{};
    c.link.beamforming.enabled = true;
    c.link.beamforming.remoteMac = "00:c0:ca:aa:bb:cc";
    c.link.stbc = true;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "link.beamforming");
}

TEST_CASE("validate: beamforming on requires a valid remoteMac") {
    Config c{};
    c.link.beamforming.enabled = true;

    c.link.beamforming.remoteMac = "";
    REQUIRE(validate(c).size() == 1);
    CHECK(validate(c)[0].path == "link.beamforming.remoteMac");

    c.link.beamforming.remoteMac = "not-a-mac";
    REQUIRE(validate(c).size() == 1);
    CHECK(validate(c)[0].path == "link.beamforming.remoteMac");

    c.link.beamforming.remoteMac = "00:c0:ca:aa:bb:cc";
    CHECK(validate(c).empty());
}

TEST_CASE("validate: beamforming ackTimeout and intervalMs ranges") {
    Config c{};
    c.link.beamforming.enabled = true;
    c.link.beamforming.remoteMac = "00:c0:ca:aa:bb:cc";

    c.link.beamforming.ackTimeout = 32;     // below 33
    REQUIRE(validate(c).size() == 1);
    CHECK(validate(c)[0].path == "link.beamforming.ackTimeout");

    c.link.beamforming.ackTimeout = 255;
    c.link.beamforming.intervalMs = 0;      // below 1
    REQUIRE(validate(c).size() == 1);
    CHECK(validate(c)[0].path == "link.beamforming.intervalMs");
}
```

- [ ] **Step 2: Run tests to verify they fail**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="validate: beamforming*"`
Expected: FAIL — no BF validation yet (the "on requires stbc off" / range cases find 0 errors).

- [ ] **Step 3: Implement the BF validation**

In `src/config/validate.cpp`, add a static MAC validator above `validate(...)` (near `parseResolution`):

```cpp
static bool isValidMac(const std::string& s) {
    if (s.size() != 17) return false;
    for (size_t i = 0; i < s.size(); ++i) {
        if (i % 3 == 2) { if (s[i] != ':') return false; }
        else if (!std::isxdigit(static_cast<unsigned char>(s[i]))) return false;
    }
    return true;
}
```

Add `#include <cctype>` to the includes at the top of the file.

In `validate(...)`, in the `// link` section after the `channel` check, add:

```cpp
    if (c.link.beamforming.enabled) {
        const auto& bf = c.link.beamforming;
        // Driver requires STBC off under monitor beamforming. (The MCS/NSS1
        // requirement is already covered by the global link.mcs 0..7 rule.)
        if (c.link.stbc)
            errs.push_back({"link.beamforming",
                            "requires link.stbc=false"});
        if (!isValidMac(bf.remoteMac))
            errs.push_back({"link.beamforming.remoteMac",
                            "must be a valid MAC (aa:bb:cc:dd:ee:ff)"});
        if (bf.ackTimeout < 33 || bf.ackTimeout > 255)
            errs.push_back({"link.beamforming.ackTimeout", "must be 33..255"});
        if (bf.intervalMs < 1)
            errs.push_back({"link.beamforming.intervalMs", "must be >= 1"});
    }
```

- [ ] **Step 4: Run tests to verify they pass**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="validate: beamforming*"`
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/config/validate.cpp tests/unit/test_validate.cpp
git commit -m "feat(validate): enforce beamforming constraints (stbc/mac/timeouts)"
```

---

## Task 4: `BeamformingController` + `resolveLocalMac`

**Files:**
- Create: `src/supervise/beamforming.hpp`, `src/supervise/beamforming.cpp`
- Create: `tests/integration/test_beamforming.cpp`
- Modify: `CMakeLists.txt` (add `src/supervise/beamforming.cpp` to `fpvd_core`; add `tests/integration/test_beamforming.cpp` to `fpvd_tests`)

- [ ] **Step 1: Write the header**

Create `src/supervise/beamforming.hpp`:

```cpp
#pragma once
#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace fpvd {

struct BfParams {
    std::string iface;      // e.g. "wlan0"
    std::string driver;     // RadioResult.driver, for the unsupported reason text
    std::string remoteMac;  // ground-station MAC
    int width{20};          // link.width; controller derives modulation bw
    int ackTimeout{255};
    int intervalMs{100};

    bool operator==(const BfParams& o) const {
        return iface == o.iface && driver == o.driver &&
               remoteMac == o.remoteMac && width == o.width &&
               ackTimeout == o.ackTimeout && intervalMs == o.intervalMs;
    }
    bool operator!=(const BfParams& o) const { return !(*this == o); }
};

enum class BfState { Disabled, Unsupported, Active, Error };

struct BfStatus {
    bool requested{false};
    BfState state{BfState::Disabled};
    std::string reason;
    std::string localMac;   // resolved drone reference for the GS
    std::string remoteMac;
    int bw{0};
    long soundingCount{0};
    std::optional<std::string> lastCbr;
};

// Resolve the chip's MAC: parse the first MAC-like token from
// <procBase>/<iface>/mac_addr; fall back to <sysBase>/<iface>/address.
// Returns "" if neither is readable.
std::string resolveLocalMac(const std::string& procBase,
                            const std::string& sysBase,
                            const std::string& iface);

class BeamformingController {
public:
    explicit BeamformingController(
        std::string procBase = "/proc/net/rtl88x2eu",
        std::string sysBase  = "/sys/class/net");
    ~BeamformingController();

    // Idempotent: starts, stops, or restarts the sounding loop to match the
    // desired (enabled, params) state.
    void reconcile(bool enabled, const BfParams& p);
    void stop();                  // stop loop + reset driver state
    BfStatus status() const;

private:
    bool supported(const std::string& iface) const;  // bf_monitor_conf exists?
    bool writeNode(const std::string& iface, const std::string& node,
                   const std::string& content);       // returns false on error
    std::string readNode(const std::string& iface, const std::string& node) const;
    void startLoop();
    void loop();
    void stopLoopThread();        // join without driver reset

    std::string procBase_;
    std::string sysBase_;

    mutable std::mutex mu_;       // guards status_ + params_
    BfStatus status_;
    BfParams params_;
    bool running_{false};

    std::thread thr_;
    std::atomic<bool> stopFlag_{false};
};

} // namespace fpvd
```

- [ ] **Step 2: Write the implementation**

Create `src/supervise/beamforming.cpp`:

```cpp
#include "supervise/beamforming.hpp"
#include "link_width.hpp"
#include <cctype>
#include <chrono>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

namespace fpvd {

static std::string extractMac(const std::string& text) {
    // Find the first aa:bb:cc:dd:ee:ff token.
    for (size_t i = 0; i + 17 <= text.size(); ++i) {
        bool ok = true;
        for (size_t j = 0; j < 17; ++j) {
            char ch = text[i + j];
            if (j % 3 == 2) { if (ch != ':') { ok = false; break; } }
            else if (!std::isxdigit(static_cast<unsigned char>(ch))) { ok = false; break; }
        }
        if (ok) return text.substr(i, 17);
    }
    return "";
}

std::string resolveLocalMac(const std::string& procBase,
                            const std::string& sysBase,
                            const std::string& iface) {
    {
        std::ifstream f(procBase + "/" + iface + "/mac_addr");
        if (f) {
            std::stringstream ss; ss << f.rdbuf();
            auto mac = extractMac(ss.str());
            if (!mac.empty()) return mac;
        }
    }
    {
        std::ifstream f(sysBase + "/" + iface + "/address");
        if (f) {
            std::string line; std::getline(f, line);
            auto mac = extractMac(line);
            if (!mac.empty()) return mac;
        }
    }
    return "";
}

BeamformingController::BeamformingController(std::string procBase,
                                             std::string sysBase)
    : procBase_(std::move(procBase)), sysBase_(std::move(sysBase)) {}

BeamformingController::~BeamformingController() { stop(); }

bool BeamformingController::supported(const std::string& iface) const {
    struct stat st{};
    std::string p = procBase_ + "/" + iface + "/bf_monitor_conf";
    return ::stat(p.c_str(), &st) == 0;
}

bool BeamformingController::writeNode(const std::string& iface,
                                      const std::string& node,
                                      const std::string& content) {
    std::ofstream f(procBase_ + "/" + iface + "/" + node);
    if (!f) return false;
    f << content;
    f.flush();
    return static_cast<bool>(f);
}

std::string BeamformingController::readNode(const std::string& iface,
                                            const std::string& node) const {
    std::ifstream f(procBase_ + "/" + iface + "/" + node);
    if (!f) return "";
    std::stringstream ss; ss << f.rdbuf();
    return ss.str();
}

void BeamformingController::reconcile(bool enabled, const BfParams& p) {
    if (!enabled) {
        stop();
        std::lock_guard<std::mutex> g(mu_);
        status_ = BfStatus{};   // clean Disabled, requested=false
        running_ = false;
        return;
    }

    // Already running with identical params => no-op.
    {
        std::lock_guard<std::mutex> g(mu_);
        if (running_ && params_ == p && status_.state == BfState::Active)
            return;
    }
    // Any change while running => restart cleanly.
    stop();

    BfStatus s;
    s.requested = true;
    s.remoteMac = p.remoteMac;
    s.bw = modulationWidth(p.width);
    s.localMac = resolveLocalMac(procBase_, sysBase_, p.iface);

    if (!supported(p.iface)) {
        s.state = BfState::Unsupported;
        s.reason = "no bf_monitor proc node on " + p.iface +
                   " (driver " + p.driver + ")";
        std::lock_guard<std::mutex> g(mu_);
        status_ = s; params_ = p; running_ = false;
        return;
    }

    // Init sequence (synchronous so errors surface immediately).
    bool ok = writeNode(p.iface, "bf_monitor_conf",
                         "1 " + p.remoteMac + " 0 0");
    ok = writeNode(p.iface, "ack_timeout", std::to_string(p.ackTimeout)) && ok;
    if (!ok) {
        s.state = BfState::Error;
        s.reason = "failed to write bf_monitor init nodes";
        std::lock_guard<std::mutex> g(mu_);
        status_ = s; params_ = p; running_ = false;
        return;
    }

    s.state = BfState::Active;
    {
        std::lock_guard<std::mutex> g(mu_);
        status_ = s; params_ = p; running_ = true;
    }
    startLoop();
}

void BeamformingController::startLoop() {
    stopFlag_.store(false);
    thr_ = std::thread([this] { loop(); });
}

void BeamformingController::loop() {
    int token = 0;
    BfParams p;
    { std::lock_guard<std::mutex> g(mu_); p = params_; }
    const int bw = modulationWidth(p.width);

    while (!stopFlag_.load()) {
        std::string trig = resolveLocalMac(procBase_, sysBase_, p.iface);
        trig += " " + p.remoteMac + " 0 0 " + std::to_string(token) +
                " " + std::to_string(bw);
        if (!writeNode(p.iface, "bf_monitor_trig", trig)) {
            std::lock_guard<std::mutex> g(mu_);
            status_.state = BfState::Error;
            status_.reason = "bf_monitor_trig write failed";
            return;
        }
        token = (token + 1) % 64;
        std::string cbr = readNode(p.iface, "bf_monitor_trig");
        {
            std::lock_guard<std::mutex> g(mu_);
            status_.soundingCount++;
            status_.lastCbr = cbr.empty() ? std::nullopt
                                          : std::optional<std::string>(cbr);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(p.intervalMs));
        if (stopFlag_.load()) break;
        writeNode(p.iface, "bf_monitor_en", "1");
    }
}

void BeamformingController::stopLoopThread() {
    stopFlag_.store(true);
    if (thr_.joinable()) thr_.join();
}

void BeamformingController::stop() {
    std::string iface;
    bool wasRunning;
    {
        std::lock_guard<std::mutex> g(mu_);
        wasRunning = running_;
        iface = params_.iface;
    }
    stopLoopThread();
    if (wasRunning && !iface.empty()) {
        writeNode(iface, "bf_monitor_conf", "0 00:00:00:00:00:00 0 0");
        writeNode(iface, "ack_timeout", "33");
    }
    std::lock_guard<std::mutex> g(mu_);
    running_ = false;
    if (status_.state == BfState::Active) {
        status_.state = BfState::Disabled;
        status_.requested = false;
    }
}

BfStatus BeamformingController::status() const {
    std::lock_guard<std::mutex> g(mu_);
    return status_;
}

} // namespace fpvd
```

- [ ] **Step 3: Register in CMake**

In `CMakeLists.txt`, add to `fpvd_core` `target_sources` (with the other `src/supervise/*.cpp`):

```cmake
    src/supervise/beamforming.cpp
```

Add to `fpvd_tests` `target_sources` (with the other `tests/integration/*.cpp`):

```cmake
    tests/integration/test_beamforming.cpp
```

- [ ] **Step 4: Write the failing tests**

Create `tests/integration/test_beamforming.cpp`:

```cpp
#include "doctest.h"
#include "supervise/beamforming.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <thread>

namespace fs = std::filesystem;
using namespace std::chrono_literals;

static fs::path makeIface(const fs::path& procBase, const std::string& iface,
                          bool withBfNode) {
    fs::create_directories(procBase / iface);
    std::ofstream(procBase / iface / "mac_addr") << "mac_addr=00:c0:ca:11:22:33\n";
    if (withBfNode) std::ofstream(procBase / iface / "bf_monitor_conf") << "";
    return procBase / iface;
}

static std::string readFile(const fs::path& p) {
    std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}

TEST_CASE("beamforming: resolveLocalMac prefers proc mac_addr, falls back to sysfs") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-mac";
    fs::remove_all(tmp);
    auto proc = tmp / "proc"; auto sys = tmp / "sys";
    fs::create_directories(proc / "wlan0");
    fs::create_directories(sys / "wlan0");
    std::ofstream(proc / "wlan0" / "mac_addr") << "addr=00:c0:ca:aa:bb:cc";
    std::ofstream(sys / "wlan0" / "address") << "de:ad:be:ef:00:01\n";
    CHECK(fpvd::resolveLocalMac(proc.string(), sys.string(), "wlan0")
          == "00:c0:ca:aa:bb:cc");

    fs::remove(proc / "wlan0" / "mac_addr");  // fall back to sysfs
    CHECK(fpvd::resolveLocalMac(proc.string(), sys.string(), "wlan0")
          == "de:ad:be:ef:00:01");
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: unsupported when bf_monitor_conf absent") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-unsup";
    fs::remove_all(tmp);
    makeIface(tmp / "proc", "wlan0", /*withBfNode=*/false);
    fpvd::BeamformingController bf((tmp / "proc").string(),
                                   (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "88XXau";
    p.remoteMac = "00:c0:ca:dd:ee:ff";
    bf.reconcile(true, p);
    auto s = bf.status();
    CHECK(s.requested == true);
    CHECK(s.state == fpvd::BfState::Unsupported);
    CHECK(s.localMac == "00:c0:ca:11:22:33");
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: active writes init sequence; stop resets driver") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-active";
    fs::remove_all(tmp);
    auto ifd = makeIface(tmp / "proc", "wlan0", /*withBfNode=*/true);
    fpvd::BeamformingController bf((tmp / "proc").string(),
                                   (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "8812eu";
    p.remoteMac = "00:c0:ca:dd:ee:ff"; p.width = 10;
    p.ackTimeout = 255; p.intervalMs = 5;
    bf.reconcile(true, p);

    auto s = bf.status();
    CHECK(s.state == fpvd::BfState::Active);
    CHECK(s.bw == 20);                                   // width 10 => modulation 20
    CHECK(readFile(ifd / "bf_monitor_conf") == "1 00:c0:ca:dd:ee:ff 0 0");
    CHECK(readFile(ifd / "ack_timeout") == "255");

    // Loop produces soundings.
    for (int i = 0; i < 100 && bf.status().soundingCount == 0; ++i)
        std::this_thread::sleep_for(5ms);
    CHECK(bf.status().soundingCount > 0);

    bf.stop();
    CHECK(readFile(ifd / "bf_monitor_conf") == "0 00:00:00:00:00:00 0 0");
    CHECK(readFile(ifd / "ack_timeout") == "33");
    CHECK(bf.status().state == fpvd::BfState::Disabled);
    fs::remove_all(tmp);
}

TEST_CASE("beamforming: reconcile is idempotent and disables on enabled=false") {
    auto tmp = fs::temp_directory_path() / "fpvd-bf-idem";
    fs::remove_all(tmp);
    makeIface(tmp / "proc", "wlan0", /*withBfNode=*/true);
    fpvd::BeamformingController bf((tmp / "proc").string(),
                                   (tmp / "sys").string());
    fpvd::BfParams p; p.iface = "wlan0"; p.driver = "8812eu";
    p.remoteMac = "00:c0:ca:dd:ee:ff"; p.intervalMs = 5;
    bf.reconcile(true, p);
    CHECK(bf.status().state == fpvd::BfState::Active);

    bf.reconcile(true, p);                 // identical => still active
    CHECK(bf.status().state == fpvd::BfState::Active);

    bf.reconcile(false, p);                // disable
    CHECK(bf.status().state == fpvd::BfState::Disabled);
    fs::remove_all(tmp);
}
```

- [ ] **Step 5: Run tests to verify they pass (build first to confirm they compiled and fail without impl already done)**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j && ./build/fpvd_tests -tc="beamforming:*"`
Expected: PASS (the implementation from Steps 1-2 satisfies them). If any controller bug surfaces, fix it in `beamforming.cpp` and re-run until green.

- [ ] **Step 6: Commit**

```bash
git add src/supervise/beamforming.hpp src/supervise/beamforming.cpp \
        tests/integration/test_beamforming.cpp CMakeLists.txt
git commit -m "feat(supervise): BeamformingController monitor-mode sounding loop"
```

---

## Task 5: Daemon integration

**Files:**
- Modify: `src/daemon.hpp` (include, member, method decl)
- Modify: `src/daemon.cpp` (`reconcileBeamforming`, call sites, `restarted` reporting)
- Test: `tests/integration/test_daemon.cpp`

- [ ] **Step 1: Write the failing test**

Add to `tests/integration/test_daemon.cpp`:

```cpp
TEST_CASE("daemon: apply reports beamforming when its config changes") {
    auto tmp = fs::temp_directory_path() / "fpvd-test-bf-apply";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");

    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    nlohmann::json patch = {{"link", {{"beamforming",
        {{"enabled", true}, {"remoteMac", "00:c0:ca:dd:ee:ff"}}}}}};
    auto pr = d.patchPending(patch);
    REQUIRE(pr.ok);

    auto ar = d.apply(false);
    REQUIRE(ar.ok);
    auto& r = ar.restarted;
    CHECK(std::find(r.begin(), r.end(), "beamforming") != r.end());
    fs::remove_all(tmp);
}
```

- [ ] **Step 2: Run test to verify it fails**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="daemon: apply reports beamforming when its config changes"`
Expected: FAIL — `"beamforming"` not present in `restarted`.

- [ ] **Step 3: Add the member and method to the header**

In `src/daemon.hpp`, add the include with the other `supervise/` includes:

```cpp
#include "supervise/beamforming.hpp"
```

In `class Daemon`, under `private:`, declare the method (after `seedOrchestrator`):

```cpp
    void seedOrchestrator();
    void reconcileBeamforming();
    void rewriteWaybeamJson();
```

And add the member (after `Orchestrator orch_;`):

```cpp
    Orchestrator orch_;
    BeamformingController bf_;
```

- [ ] **Step 4: Implement reconcileBeamforming and wire call sites**

In `src/daemon.cpp`, add the helper (place it just after `seedOrchestrator()`'s closing brace):

```cpp
void Daemon::reconcileBeamforming() {
    const auto& bfc = effective_.link.beamforming;
    BfParams p;
    p.iface      = radio_.iface.empty() ? "wlan0" : radio_.iface;
    p.driver     = radio_.driver;
    p.remoteMac  = bfc.remoteMac;
    p.width      = effective_.link.width;
    p.ackTimeout = bfc.ackTimeout;
    p.intervalMs = bfc.intervalMs;
    bf_.reconcile(bfc.enabled, p);
}
```

In `bootstrap()`, after `orch_.startAll();`:

```cpp
        seedOrchestrator();
        orch_.startAll();
        reconcileBeamforming();
```

In `apply()`'s **deferred-retune** detached thread, after its `orch_.startAll();`:

```cpp
            seedOrchestrator();
            orch_.startAll();
            reconcileBeamforming();
        }).detach();
```

In `apply()`'s **synchronous restart** branch, after its `orch_.startAll();` (the one inside `if (reallyRestart) { ... }`):

```cpp
        seedOrchestrator();
        orch_.startAll();
        reconcileBeamforming();
    } else {
```

- [ ] **Step 5: Add `"beamforming"` to the restarted report**

In `apply()`, locate where `restarted` is populated (after `for (auto& n : subs.servicesAffected) restarted.push_back(n);`). Just below the existing diff was computed (`auto subs = diffSubsystems(...)`), the function already overwrites `effective_ = pending_` later. Add the change detection **before** `effective_ = pending_;`, right after the `deferRadioRetune` computation:

```cpp
    const bool bfChanged =
        nlohmann::json(effective_.link.beamforming) !=
            nlohmann::json(pending_.link.beamforming) ||
        effective_.link.width != pending_.link.width;
```

Then, in the `restarted` assembly block (after the services loop), add:

```cpp
    for (auto& n : subs.servicesAffected) restarted.push_back(n);
    if (bfChanged) restarted.push_back("beamforming");
```

Ensure `#include <nlohmann/json.hpp>` is available in `daemon.cpp` (it is via existing includes; if not, add it).

- [ ] **Step 6: Run the daemon test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="daemon: apply reports beamforming when its config changes"`
Expected: PASS.

- [ ] **Step 7: Run the full suite**

Run: `cmake --build build -j && ./build/fpvd_tests`
Expected: PASS — existing daemon tests still green (a `video.bitrate`-only apply must NOT report `"beamforming"`, since `bfChanged` is false there).

- [ ] **Step 8: Commit**

```bash
git add src/daemon.hpp src/daemon.cpp tests/integration/test_daemon.cpp
git commit -m "feat(daemon): own and reconcile BeamformingController on apply"
```

---

## Task 6: `/status` beamforming block

**Files:**
- Modify: `src/status.cpp`
- Modify: `src/daemon.hpp` (expose `bf_` status read)
- Test: `tests/integration/test_daemon.cpp` (status assertion) or `tests/integration/test_http_handlers.cpp`

- [ ] **Step 1: Expose the controller status from the daemon**

In `src/daemon.hpp`, add a public accessor (near `const RadioInfo& radio()`):

```cpp
    const RadioInfo& radio() const { return radio_; }
    BfStatus beamformingStatus() const { return bf_.status(); }
```

`BfStatus` is already in scope via the `supervise/beamforming.hpp` include added in Task 5. The method is `const`; `BeamformingController::status()` is already `const`.

- [ ] **Step 2: Write the failing test**

Add to `tests/integration/test_daemon.cpp`:

```cpp
TEST_CASE("status: includes beamforming block") {
    auto tmp = fs::temp_directory_path() / "fpvd-test-bf-status";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");
    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    auto j = fpvd::buildStatus(d);
    REQUIRE(j.contains("beamforming"));
    CHECK(j["beamforming"]["state"] == "disabled");
    CHECK(j["beamforming"].contains("localMac"));
    fs::remove_all(tmp);
}
```

Add `#include "status.hpp"` to the test file's includes if not already present.

- [ ] **Step 3: Run test to verify it fails**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="status: includes beamforming block"`
Expected: FAIL — no `beamforming` key in status.

- [ ] **Step 4: Implement the status block**

In `src/status.cpp`, add a state-name helper near `stateName(ProcState)`:

```cpp
static const char* bfStateName(BfState s) {
    switch (s) {
        case BfState::Disabled:    return "disabled";
        case BfState::Unsupported: return "unsupported";
        case BfState::Active:      return "active";
        case BfState::Error:       return "error";
    }
    return "unknown";
}
```

In `buildStatus(...)`, before the final `return {`, capture the snapshot:

```cpp
    auto bf = d.beamformingStatus();
```

Add a `beamforming` entry to the returned object (after the `radio` block, before `processes`):

```cpp
        {"beamforming", {
            {"requested", bf.requested},
            {"state", bfStateName(bf.state)},
            {"reason", bf.reason},
            {"localMac", bf.localMac},
            {"remoteMac", bf.remoteMac},
            {"bw", bf.bw},
            {"soundingCount", bf.soundingCount},
            {"lastCbr", bf.lastCbr.has_value()
                         ? nlohmann::json(bf.lastCbr.value())
                         : nlohmann::json(nullptr)}
        }},
```

`BfState`/`BfStatus` are visible in `status.cpp` because `status.hpp` includes `daemon.hpp`, which now includes `supervise/beamforming.hpp`.

- [ ] **Step 5: Run test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="status: includes beamforming block"`
Expected: PASS.

- [ ] **Step 6: Commit**

```bash
git add src/status.cpp src/daemon.hpp tests/integration/test_daemon.cpp
git commit -m "feat(status): report beamforming state and resolved localMac"
```

---

## Task 7: `radio-up.sh` 10MHz channel bring-up

**Files:**
- Modify: `scripts/radio-up.sh`

- [ ] **Step 1: Replace the width line with a case**

In `scripts/radio-up.sh`, replace this block:

```sh
iw $WLAN_DEV set monitor none
[ "${FPVD_WIDTH:-20}" = "40" ] && width=HT40+ || width=HT20
iw $WLAN_DEV set channel "${FPVD_CHANNEL:-161}" "$width"
```

with:

```sh
iw $WLAN_DEV set monitor none
# 10MHz uses a dedicated channel-width token (baseband underclocked, 20MHz
# modulation); 40 => HT40+; everything else => HT20.
case "${FPVD_WIDTH:-20}" in
    10) iw $WLAN_DEV set channel "${FPVD_CHANNEL:-161}" 10MHz ;;
    40) iw $WLAN_DEV set channel "${FPVD_CHANNEL:-161}" HT40+ ;;
    *)  iw $WLAN_DEV set channel "${FPVD_CHANNEL:-161}" HT20 ;;
esac
```

- [ ] **Step 2: Syntax-check the script**

Run: `sh -n scripts/radio-up.sh && echo OK`
Expected: `OK` (no syntax errors).

- [ ] **Step 3: Verify the 10MHz branch is reachable (dry trace)**

Run: `FPVD_WIDTH=10 FPVD_CHANNEL=149 sh -x scripts/radio-up.sh 2>&1 | grep -m1 '10MHz' || echo "trace stops earlier (no adapter) — inspect manually"`
Expected: Either a line containing `iw wlan0 set channel 149 10MHz`, or the fallback message (the script exits early on a host with no supported USB adapter — that's fine; the `case` itself was syntax-checked in Step 2). Confirm by reading the edited `case` block that the `10)` branch emits the `10MHz` token.

- [ ] **Step 4: Commit**

```bash
git add scripts/radio-up.sh
git commit -m "feat(radio): tune 10MHz channel width in radio-up.sh"
```

---

## Final verification

- [ ] **Build clean and run the whole suite**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j && ./build/fpvd_tests`
Expected: all tests PASS, no warnings from the new files (`-Wall -Wextra -Wpedantic` is on).

- [ ] **Manual smoke (host)**

Run:
```bash
./build/fpvd --defaults etc/defaults.json --overlay /tmp/fpvd-overlay.json \
  --radio-up /bin/true --waybeam-json /tmp/fpvd-waybeam.json --port 8080 &
sleep 1
curl -s localhost:8080/status | python3 -m json.tool | grep -A8 beamforming
# Enable BF (will be 'unsupported' on a dev host with no driver proc node):
curl -s -X PATCH localhost:8080/config -H 'content-type: application/json' \
  -d '{"link":{"beamforming":{"enabled":true,"remoteMac":"00:c0:ca:dd:ee:ff"}}}'
curl -s -X POST localhost:8080/apply
curl -s localhost:8080/status | python3 -m json.tool | grep -A8 beamforming
kill %1
```
Expected: first status shows `"state": "disabled"`; after apply, `"state": "unsupported"` with a reason mentioning the iface (no real driver on the dev host). A bad config (e.g. `stbc:true` with BF on) should make the PATCH return a 400 validation error.

---

## Self-review notes

- **Spec coverage:** schema (T1), validation incl. width/safe.bandwidth + BF rules (T2/T3), `modulationWidth` + wfb `-B` (T2), controller w/ support-detection, loop, idempotent reconcile, error/unsupported states, faked-procfs tests, `resolveLocalMac` (T4), daemon ownership/reconcile/restarted reporting (T5), `/status` block (T6), radio-up.sh 10MHz (T7). All spec sections map to a task.
- **mcs 0..9 rule:** intentionally omitted as dead code (global `link.mcs` 0..7 is stricter) — documented in the Deviations note.
- **localMac:** resolved inside the controller, not configurable — matches the revised spec.
- **Type consistency:** `BfParams`/`BfState`/`BfStatus`/`reconcile`/`stop`/`status`/`resolveLocalMac`/`reconcileBeamforming`/`modulationWidth`/`bfChanged` are named identically across all tasks.
