# Waybeam API-driven config apply — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Apply fpvd-managed waybeam encoder config through waybeam's HTTP API (live `/api/v1/set` for live fields; an fpvd-owned waybeam-only process restart for restart-class fields) instead of bouncing the whole orchestrator, so encoder changes never drop the radio link.

**Architecture:** A shared `WaybeamClient` transport (cpp-httplib) is reused by the dynamic-link `EncoderClient` and a new config-apply path in `Daemon::apply()`. A `waybeamConfigDiff()` buckets changed fields into LIVE (pushed via a batched `/api/v1/set`, transactionally before commit) and RESTART (waybeam.json rewrite + `Orchestrator::restart("waybeam")`). `subs.encoder` no longer forces a full rebuild.

**Tech Stack:** C++17, cpp-httplib (vendored), nlohmann/json (vendored), doctest (vendored), CMake.

**Reference spec:** `docs/superpowers/specs/2026-06-02-waybeam-api-config-apply-design.md`

**Build & test commands (host):**
```bash
cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host --target fpvd_tests -j
./build/host/fpvd_tests                              # whole suite
./build/host/fpvd_tests --test-case="<pattern>"      # one case (doctest filter)
```

---

### Task 1: `WaybeamClient` shared transport

A thin, stateless cpp-httplib wrapper for waybeam's HTTP API. A fresh `httplib::Client` is built per call, so one instance is safe to share across threads.

**Files:**
- Create: `src/waybeam/client.hpp`
- Create: `src/waybeam/client.cpp`
- Modify: `CMakeLists.txt` (register source + test)
- Test: `tests/unit/test_waybeam_client.cpp`

- [ ] **Step 1: Write the header**

Create `src/waybeam/client.hpp`:

```cpp
#pragma once
#include <cstdint>
#include <map>
#include <string>

namespace fpvd {

// Thin transport for waybeam's HTTP API (cpp-httplib). Stateless: a fresh
// httplib::Client is created per call, so a single instance is safe to share
// across threads. Shared by the dynamic-link EncoderClient and the daemon's
// config-apply path.
class WaybeamClient {
public:
    WaybeamClient(std::string host, uint16_t port,
                  int connectTimeoutMs = 300, int readTimeoutMs = 500);

    // GET /api/v1/set?k1=v1&k2=v2 … (keys/values percent-encoded). true on 2xx.
    // An empty map is a no-op that returns true.
    bool setFields(const std::map<std::string, std::string>& fields);

    // GET an already-formed path (e.g. "/request/idr"). true on 2xx.
    bool get(const std::string& path);

private:
    std::string host_;
    uint16_t    port_;
    int         connectTimeoutMs_;
    int         readTimeoutMs_;
};

} // namespace fpvd
```

- [ ] **Step 2: Write the implementation**

Create `src/waybeam/client.cpp`:

```cpp
#include "waybeam/client.hpp"
#include <httplib.h>
#include <cctype>

namespace fpvd {

static std::string urlEncode(const std::string& s) {
    static const char* hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(s.size());
    for (unsigned char c : s) {
        if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            out.push_back(static_cast<char>(c));
        } else {
            out.push_back('%');
            out.push_back(hex[c >> 4]);
            out.push_back(hex[c & 0x0F]);
        }
    }
    return out;
}

WaybeamClient::WaybeamClient(std::string host, uint16_t port,
                             int connectTimeoutMs, int readTimeoutMs)
    : host_(std::move(host)), port_(port),
      connectTimeoutMs_(connectTimeoutMs), readTimeoutMs_(readTimeoutMs) {}

bool WaybeamClient::get(const std::string& path) {
    httplib::Client cli(host_, static_cast<int>(port_));
    cli.set_connection_timeout(0, connectTimeoutMs_ * 1000);  // µs
    cli.set_read_timeout(0, readTimeoutMs_ * 1000);
    auto res = cli.Get(path.c_str());
    return res && res->status / 100 == 2;
}

bool WaybeamClient::setFields(const std::map<std::string, std::string>& fields) {
    if (fields.empty()) return true;
    std::string path = "/api/v1/set?";
    bool first = true;
    for (const auto& [k, v] : fields) {
        if (!first) path.push_back('&');
        first = false;
        path += urlEncode(k);
        path.push_back('=');
        path += urlEncode(v);
    }
    return get(path);
}

} // namespace fpvd
```

- [ ] **Step 3: Register the source and the test in CMake**

In `CMakeLists.txt`, add to the `fpvd_core` `target_sources(...)` block (after the `src/translate/waybeam.cpp` line):

```cmake
    src/waybeam/client.cpp
```

In the `fpvd_tests` `target_sources(...)` block (after the `tests/unit/test_translate_waybeam.cpp` line), add:

```cmake
        tests/unit/test_waybeam_client.cpp
```

- [ ] **Step 4: Write the failing test**

Create `tests/unit/test_waybeam_client.cpp`:

```cpp
#include "doctest.h"
#include "waybeam/client.hpp"
#include <httplib.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace fpvd;

// Minimal fake waybeam HTTP server (mirrors test_dl_encoder_client.cpp).
struct FakeWb {
    httplib::Server srv;
    std::vector<std::string> hits;
    std::mutex mu;
    int port{0};
    std::thread th;

    FakeWb() {
        srv.Get("/api/v1/set", [&](const httplib::Request& r, httplib::Response& res) {
            std::lock_guard<std::mutex> lk(mu);
            hits.push_back(r.target);
            res.set_content("ok", "text/plain");
        });
        srv.Get("/request/idr", [&](const httplib::Request& r, httplib::Response& res) {
            (void)r;
            std::lock_guard<std::mutex> lk(mu);
            hits.push_back("/request/idr");
            res.set_content("ok", "text/plain");
        });
        port = srv.bind_to_any_port("127.0.0.1");
        th = std::thread([&] { srv.listen_after_bind(); });
        srv.wait_until_ready();
    }
    ~FakeWb() { srv.stop(); th.join(); }
    size_t count() { std::lock_guard<std::mutex> lk(mu); return hits.size(); }
    std::string last() { std::lock_guard<std::mutex> lk(mu); return hits.back(); }
};

TEST_CASE("WaybeamClient::setFields builds /api/v1/set and returns true on 2xx") {
    FakeWb f;
    WaybeamClient c("127.0.0.1", static_cast<uint16_t>(f.port));
    std::map<std::string, std::string> fields{
        {"video0.bitrate", "6000"}, {"fpv.roi_enabled", "true"}};
    CHECK(c.setFields(fields));
    REQUIRE(f.count() == 1);
    CHECK(f.last().find("video0.bitrate=6000") != std::string::npos);
    CHECK(f.last().find("fpv.roi_enabled=true") != std::string::npos);
}

TEST_CASE("WaybeamClient::get hits the path and returns true on 2xx") {
    FakeWb f;
    WaybeamClient c("127.0.0.1", static_cast<uint16_t>(f.port));
    CHECK(c.get("/request/idr"));
    REQUIRE(f.count() == 1);
    CHECK(f.last() == "/request/idr");
}

TEST_CASE("WaybeamClient::setFields empty map is a no-op success") {
    FakeWb f;
    WaybeamClient c("127.0.0.1", static_cast<uint16_t>(f.port));
    CHECK(c.setFields({}));
    CHECK(f.count() == 0);
}

TEST_CASE("WaybeamClient returns false when connection refused") {
    httplib::Server dead;
    int dead_port = dead.bind_to_any_port("127.0.0.1");
    dead.stop();  // refuse connections
    WaybeamClient c("127.0.0.1", static_cast<uint16_t>(dead_port));
    CHECK_FALSE(c.setFields({{"video0.bitrate", "6000"}}));
    CHECK_FALSE(c.get("/request/idr"));
}
```

- [ ] **Step 5: Build and run — verify it passes**

```bash
cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host --target fpvd_tests -j
./build/host/fpvd_tests --test-case="WaybeamClient*"
```
Expected: all 4 `WaybeamClient*` cases PASS.

- [ ] **Step 6: Commit**

```bash
git add src/waybeam/client.hpp src/waybeam/client.cpp tests/unit/test_waybeam_client.cpp CMakeLists.txt
git commit -m "feat(waybeam): add shared WaybeamClient HTTP transport"
```

---

### Task 2: Refactor `EncoderClient` onto `WaybeamClient`

Move transport out of the dynamic-link `EncoderClient` into the shared `WaybeamClient`; keep all DL policy (ROI curve, dedup, IDR throttle). Behavior (query strings, timeouts, dedup) is unchanged.

**Files:**
- Modify: `src/dynlink/encoder_client.hpp`
- Modify: `src/dynlink/encoder_client.cpp`
- Modify: `src/dynlink/controller.hpp:42` (add `wb_` member)
- Modify: `src/dynlink/controller.cpp:106-107` (init `wb_`) and `:124` (emplace)
- Test: `tests/unit/test_dl_encoder_client.cpp` (construct via `WaybeamClient`)

- [ ] **Step 1: Update the EncoderClient header**

Replace the whole body of `src/dynlink/encoder_client.hpp` with:

```cpp
#pragma once
#include "waybeam/client.hpp"
#include <cstdint>

namespace fpvd::dynlink {

struct RoiCurve {
    uint16_t thresholdKbps;
    uint16_t lowAnchorKbps;
    int8_t   floor;
    uint8_t  step;
};

class EncoderClient {
public:
    // Transport is injected (non-owning); the referenced WaybeamClient must
    // outlive this EncoderClient.
    EncoderClient(WaybeamClient& client, uint32_t minIdrIntervalMs, RoiCurve roi);

    // GET /api/v1/set?video0.bitrate=&fpv.roiQp=&[video0.fps=]. Diff-based.
    // bitrate==0 is a no-op sentinel. Returns 0 ok/no-op, -1 HTTP fail.
    int apply(uint16_t bitrateKbps, uint8_t fps);

    // GET /request/idr, throttled by minIdrIntervalMs. 0 sent, 1 throttled, -1 fail.
    int requestIdr(uint64_t nowMs);

    // Push safe bitrate (roiQp recomputed, fps unchanged). Returns 0/-1.
    int applySafe(uint16_t safeBitrateKbps);

    void setRoiCurve(RoiCurve roi) { roi_ = roi; }
    void setMinIdrInterval(uint32_t ms) { minIdrIntervalMs_ = ms; }

private:
    WaybeamClient* client_;
    uint32_t       minIdrIntervalMs_;
    RoiCurve       roi_;

    bool     lastValid_{false};
    uint16_t lastBitrate_{0};
    int8_t   lastRoiQp_{0};
    uint8_t  lastFps_{0};

    bool     idrEverSent_{false};
    uint64_t lastIdrMs_{0};
};

} // namespace fpvd::dynlink
```

- [ ] **Step 2: Update the EncoderClient implementation**

Replace the whole body of `src/dynlink/encoder_client.cpp` with:

```cpp
/* encoder_client.cpp — dynamic-link encoder policy over the shared
 * WaybeamClient transport. Query strings and dedup are unchanged from the
 * original dl_backend_enc port. */
#include "dynlink/encoder_client.hpp"
#include "dynlink/roi_qp.hpp"

#include <map>
#include <string>

namespace fpvd::dynlink {

EncoderClient::EncoderClient(WaybeamClient& client, uint32_t minIdrIntervalMs,
                             RoiCurve roi)
    : client_(&client), minIdrIntervalMs_(minIdrIntervalMs), roi_(roi) {}

int EncoderClient::apply(uint16_t bitrateKbps, uint8_t fps) {
    if (bitrateKbps == 0) return 0;  // sentinel: don't push

    auto roiQp = static_cast<int8_t>(
        computeRoiQp(bitrateKbps, roi_.thresholdKbps, roi_.lowAnchorKbps,
                     roi_.floor, roi_.step));

    if (lastValid_ && lastBitrate_ == bitrateKbps &&
        lastRoiQp_ == roiQp && lastFps_ == fps) {
        return 0;  // no-op: nothing changed
    }

    std::map<std::string, std::string> fields{
        {"video0.bitrate", std::to_string(static_cast<unsigned>(bitrateKbps))},
        {"fpv.roiQp",      std::to_string(static_cast<int>(roiQp))},
    };
    if (fps != 0)
        fields["video0.fps"] = std::to_string(static_cast<unsigned>(fps));

    bool ok = client_->setFields(fields);
    if (ok) {
        lastBitrate_ = bitrateKbps;
        lastRoiQp_   = roiQp;
        lastFps_     = fps;
        lastValid_   = true;
    }
    return ok ? 0 : -1;
}

int EncoderClient::requestIdr(uint64_t nowMs) {
    if (idrEverSent_ &&
        (nowMs - lastIdrMs_) < static_cast<uint64_t>(minIdrIntervalMs_)) {
        return 1;  // throttled
    }
    bool ok = client_->get("/request/idr");
    lastIdrMs_   = nowMs;       // arm throttle on ANY attempt (even failure)
    idrEverSent_ = true;
    return ok ? 0 : -1;
}

int EncoderClient::applySafe(uint16_t safeBitrateKbps) {
    auto roiQp = static_cast<int8_t>(
        computeRoiQp(safeBitrateKbps, roi_.thresholdKbps, roi_.lowAnchorKbps,
                     roi_.floor, roi_.step));

    std::map<std::string, std::string> fields{
        {"video0.bitrate", std::to_string(static_cast<unsigned>(safeBitrateKbps))},
        {"fpv.roiQp",      std::to_string(static_cast<int>(roiQp))},
    };  // fps omitted (matches original apply_safe behaviour)

    bool ok = client_->setFields(fields);
    if (ok) {
        lastBitrate_ = safeBitrateKbps;
        lastRoiQp_   = roiQp;
        lastFps_     = 0;
        lastValid_   = true;
    }
    return ok ? 0 : -1;
}

} // namespace fpvd::dynlink
```

- [ ] **Step 3: Give the controller a `WaybeamClient` member**

In `src/dynlink/controller.hpp`, find:

```cpp
    Endpoints ep_;
    std::thread thread_;
```
Replace with:

```cpp
    Endpoints ep_;
    WaybeamClient wb_;            // transport for enc_; built from ep_ in the ctor
    std::thread thread_;
```
(`encoder_client.hpp` is already included by `controller.hpp`, so `WaybeamClient` is visible.)

- [ ] **Step 4: Construct `wb_` and inject it into `enc_`**

In `src/dynlink/controller.cpp`, find:

```cpp
DynamicLinkController::DynamicLinkController(Endpoints ep)
    : ep_(std::move(ep)) {}
```
Replace with:

```cpp
DynamicLinkController::DynamicLinkController(Endpoints ep)
    : ep_(std::move(ep)), wb_(ep_.encHost, ep_.encPort) {}
```

Then find (around line 124):

```cpp
    enc_.emplace(ep_.encHost, ep_.encPort, snap.minIdrIntervalMs, snap.roiQp);
```
Replace with:

```cpp
    enc_.emplace(wb_, snap.minIdrIntervalMs, snap.roiQp);
```

- [ ] **Step 5: Update the EncoderClient test to construct via `WaybeamClient`**

In `tests/unit/test_dl_encoder_client.cpp`, add the include after the existing `#include "dynlink/encoder_client.hpp"`:

```cpp
#include "waybeam/client.hpp"
```

Then replace every constructor of the form:

```cpp
    EncoderClient enc("127.0.0.1", static_cast<uint16_t>(f.port),
                      /*minIdrIntervalMs=*/500,
                      RoiCurve{6000, 2000, -24, 3});
```
with a two-line form that builds the transport first:

```cpp
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, /*minIdrIntervalMs=*/500, RoiCurve{6000, 2000, -24, 3});
```

There are 9 such constructions (TEST_CASEs at the original lines 54, 78, 90, 101, 117, 131, 141, 154, 183 use `f.port`; the one at line 171 uses `dead_port`). For the `dead_port` case (the "IDR throttle arms on any attempt including failure" test), use:

```cpp
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(dead_port));
    EncoderClient enc(wb, 500, RoiCurve{6000, 2000, -24, 3});
    enc.setMinIdrInterval(500);
```

The assertions on `f.last()` (query substrings) are unchanged — the queries are identical.

- [ ] **Step 6: Build and run the DL encoder + controller tests — verify they pass**

```bash
cmake --build build/host --target fpvd_tests -j
./build/host/fpvd_tests --test-case="EncoderClient*,*Controller*,*dl*,*DL*"
```
Expected: all dynamic-link cases PASS (identical behavior after the refactor).

- [ ] **Step 7: Commit**

```bash
git add src/dynlink/encoder_client.hpp src/dynlink/encoder_client.cpp \
        src/dynlink/controller.hpp src/dynlink/controller.cpp \
        tests/unit/test_dl_encoder_client.cpp
git commit -m "refactor(dynlink): EncoderClient uses shared WaybeamClient transport"
```

---

### Task 3: `waybeamConfigDiff` + drop `video0.codec` from translator

Add the single field→waybeam-name→mutability mapping as a diff function, and stop writing the retired `video0.codec`.

**Files:**
- Modify: `src/translate/waybeam.hpp`
- Modify: `src/translate/waybeam.cpp`
- Modify: `tests/unit/test_translate_waybeam.cpp` (remove codec assertions)
- Create: `tests/unit/test_waybeam_diff.cpp`
- Modify: `CMakeLists.txt` (register the new test)

- [ ] **Step 1: Extend the translator header**

In `src/translate/waybeam.hpp`, add the includes `#include <map>` and `#include <string>` after the existing includes, then add before the closing `} // namespace fpvd`:

```cpp
// Changed waybeam fields between two configs, bucketed by mutability.
//   live    — push via GET /api/v1/set (applied instantly, no restart)
//   restart — require a waybeam process restart to take effect
// Values are pre-formatted waybeam field values (snake_case field names).
struct WaybeamFieldDiff {
    std::map<std::string, std::string> live;
    std::map<std::string, std::string> restart;
};

// Diff the waybeam-relevant fields of `oldc` vs `newc`. `video.codec` is never
// emitted (retired in waybeam — hardcoded H.265). When `dlEnabled`, the
// dynamic-link-owned fields (video.bitrate, video.qpDelta, video.roi.*,
// video.fps) are omitted: the dynamic-link controller is their sole writer.
WaybeamFieldDiff waybeamConfigDiff(const Config& oldc, const Config& newc,
                                   bool dlEnabled);
```

- [ ] **Step 2: Drop codec from `toWaybeamJson` and implement the diff**

In `src/translate/waybeam.cpp`, delete this line from the `video0` object:

```cpp
            {"codec", c.video.codec},
```

Then add, before the closing `} // namespace fpvd`:

```cpp
static std::string fmtBool(bool b) { return b ? "true" : "false"; }

static std::string fmtDouble(double d) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", d);
    return buf;
}

WaybeamFieldDiff waybeamConfigDiff(const Config& a, const Config& b,
                                   bool dlEnabled) {
    WaybeamFieldDiff d;
    const auto& va = a.video; const auto& vb = b.video;

    // LIVE — dynamic-link-owned fields are skipped while DL is enabled.
    if (!dlEnabled) {
        if (va.bitrate != vb.bitrate)
            d.live["video0.bitrate"]  = std::to_string(vb.bitrate);
        if (va.fps != vb.fps)
            d.live["video0.fps"]      = std::to_string(vb.fps);
        if (va.qpDelta != vb.qpDelta)
            d.live["video0.qp_delta"] = std::to_string(vb.qpDelta);
        if (va.roi.enabled != vb.roi.enabled)
            d.live["fpv.roi_enabled"] = fmtBool(vb.roi.enabled);
        if (va.roi.qp != vb.roi.qp)
            d.live["fpv.roi_qp"]      = std::to_string(vb.roi.qp);
        if (va.roi.steps != vb.roi.steps)
            d.live["fpv.roi_steps"]   = std::to_string(vb.roi.steps);
        if (va.roi.center != vb.roi.center)
            d.live["fpv.roi_center"]  = fmtDouble(vb.roi.center);
    }
    // gopSize is LIVE but NOT dynamic-link-owned (the controller never writes
    // gop), so it is pushed regardless of DL state.
    if (va.gopSize != vb.gopSize)
        d.live["video0.gop_size"] = fmtDouble(vb.gopSize);

    // RESTART
    if (va.resolution != vb.resolution)
        d.restart["video0.size"]    = vb.resolution;
    if (va.rcMode != vb.rcMode)
        d.restart["video0.rc_mode"] = vb.rcMode;

    const auto& ia = a.image; const auto& ib = b.image;
    if (ia.mirror != ib.mirror) d.restart["image.mirror"] = fmtBool(ib.mirror);
    if (ia.flip   != ib.flip)   d.restart["image.flip"]   = fmtBool(ib.flip);
    if (ia.rotate != ib.rotate) d.restart["image.rotate"] = std::to_string(ib.rotate);

    const auto& ra = a.recording; const auto& rb = b.recording;
    if (ra.enabled    != rb.enabled)    d.restart["record.enabled"]     = fmtBool(rb.enabled);
    if (ra.format     != rb.format)     d.restart["record.format"]      = rb.format;
    if (ra.mode       != rb.mode)       d.restart["record.mode"]        = rb.mode;
    if (ra.maxSeconds != rb.maxSeconds) d.restart["record.max_seconds"] = std::to_string(rb.maxSeconds);
    if (ra.maxMB      != rb.maxMB)      d.restart["record.max_mb"]      = std::to_string(rb.maxMB);

    // video.codec intentionally not mapped (retired; pinned to h265).
    return d;
}
```

Add `#include <cstdio>` and `#include <string>` to the top of `src/translate/waybeam.cpp` if not already present (needed for `snprintf`/`std::to_string`).

- [ ] **Step 3: Remove codec assertions from the existing translator test**

In `tests/unit/test_translate_waybeam.cpp`, delete these three lines:

```cpp
    CHECK(out["video0"]["codec"] == "h265");
```
```cpp
    c.video.codec = "h264";
```
```cpp
    CHECK(out["video0"]["codec"] == "h264");
```

- [ ] **Step 4: Register and write the diff test**

In `CMakeLists.txt`, add to the `fpvd_tests` `target_sources(...)` block (after `tests/unit/test_translate_waybeam.cpp`):

```cmake
        tests/unit/test_waybeam_diff.cpp
```

Create `tests/unit/test_waybeam_diff.cpp`:

```cpp
#include "doctest.h"
#include "translate/waybeam.hpp"

using namespace fpvd;

TEST_CASE("waybeamConfigDiff: live bitrate change when DL disabled") {
    Config a{}, b{};
    b.video.bitrate = 4096;
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/false);
    CHECK(d.live.at("video0.bitrate") == "4096");
    CHECK(d.restart.empty());
}

TEST_CASE("waybeamConfigDiff: restart fields are bucketed separately") {
    Config a{}, b{};
    b.video.resolution = "1280x720";
    b.image.flip = true;
    b.recording.enabled = true;
    auto d = waybeamConfigDiff(a, b, false);
    CHECK(d.restart.at("video0.size") == "1280x720");
    CHECK(d.restart.at("image.flip") == "true");
    CHECK(d.restart.at("record.enabled") == "true");
    CHECK(d.live.empty());
}

TEST_CASE("waybeamConfigDiff: codec is never emitted") {
    Config a{}, b{};
    b.video.codec = "h264";   // (invalid, but must never reach waybeam)
    auto d = waybeamConfigDiff(a, b, false);
    CHECK(d.live.empty());
    CHECK(d.restart.empty());
}

TEST_CASE("waybeamConfigDiff: DL-owned fields excluded when DL enabled") {
    Config a{}, b{};
    b.video.bitrate = 4096;     // DL-owned
    b.video.qpDelta = -8;       // DL-owned
    b.video.roi.qp = -10;       // DL-owned
    b.video.fps = 30;           // DL-owned
    b.video.gopSize = 2.0;      // NOT DL-owned
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/true);
    CHECK(d.live.find("video0.bitrate") == d.live.end());
    CHECK(d.live.find("video0.qp_delta") == d.live.end());
    CHECK(d.live.find("fpv.roi_qp") == d.live.end());
    CHECK(d.live.find("video0.fps") == d.live.end());
    CHECK(d.live.at("video0.gop_size") == "2");   // gop still pushed
}

TEST_CASE("waybeamConfigDiff: DL-owned fields included when DL disabled") {
    Config a{}, b{};
    b.video.bitrate = 4096;
    b.video.fps = 30;
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/false);
    CHECK(d.live.at("video0.bitrate") == "4096");
    CHECK(d.live.at("video0.fps") == "30");
}

TEST_CASE("waybeamConfigDiff: no change yields empty diff") {
    Config a{}, b{};
    auto d = waybeamConfigDiff(a, b, false);
    CHECK(d.live.empty());
    CHECK(d.restart.empty());
}
```

- [ ] **Step 5: Build and run — verify it passes**

```bash
cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Debug   # re-run: new test file added
cmake --build build/host --target fpvd_tests -j
./build/host/fpvd_tests --test-case="waybeamConfigDiff*,*toWaybeamJson*,*translate*"
```
Expected: all `waybeamConfigDiff*` and translator cases PASS.

- [ ] **Step 6: Commit**

```bash
git add src/translate/waybeam.hpp src/translate/waybeam.cpp \
        tests/unit/test_translate_waybeam.cpp tests/unit/test_waybeam_diff.cpp \
        CMakeLists.txt
git commit -m "feat(translate): add waybeamConfigDiff, drop retired video0.codec"
```

---

### Task 4: Pin `video.codec` to h265 in validation

waybeam hardcodes H.265, so fpvd must reject any other codec.

**Files:**
- Modify: `src/config/validate.cpp:82-83`
- Test: `tests/unit/test_validate.cpp:49-53`

- [ ] **Step 1: Update the failing test first**

In `tests/unit/test_validate.cpp`, replace the existing case:

```cpp
TEST_CASE("validate: video.codec must be h264 or h265") {
    Config c{}; c.video.codec = "av1";
```
…through its closing brace (the block that checks `errs[0].path == "video.codec"`) with:

```cpp
TEST_CASE("validate: video.codec must be h265 (hardware is H.265-only)") {
    {
        Config c{}; c.video.codec = "h265";   // the only accepted value
        auto errs = fpvd::validate(c);
        for (auto& e : errs) CHECK(e.path != "video.codec");
    }
    {
        Config c{}; c.video.codec = "h264";   // previously valid, now rejected
        auto errs = fpvd::validate(c);
        bool found = false;
        for (auto& e : errs) if (e.path == "video.codec") found = true;
        CHECK(found);
    }
    {
        Config c{}; c.video.codec = "av1";
        auto errs = fpvd::validate(c);
        bool found = false;
        for (auto& e : errs) if (e.path == "video.codec") found = true;
        CHECK(found);
    }
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cmake --build build/host --target fpvd_tests -j
./build/host/fpvd_tests --test-case="validate: video.codec*"
```
Expected: FAIL — `h264` currently passes validation, so the `found` check for h264 fails.

- [ ] **Step 3: Tighten the validation**

In `src/config/validate.cpp`, replace:

```cpp
    if (c.video.codec != "h264" && c.video.codec != "h265")
        errs.push_back({"video.codec", "must be h264 or h265"});
```
with:

```cpp
    if (c.video.codec != "h265")
        errs.push_back({"video.codec", "must be h265 (hardware is H.265-only)"});
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cmake --build build/host --target fpvd_tests -j
./build/host/fpvd_tests --test-case="validate: video.codec*"
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/config/validate.cpp tests/unit/test_validate.cpp
git commit -m "feat(validate): pin video.codec to h265 (waybeam is H.265-only)"
```

---

### Task 5: `Orchestrator::restart(name)`

Restart a single supervised process (clean `shutdown()` + `start()`), leaving the others running.

**Files:**
- Modify: `src/supervise/orchestrator.hpp:18` (declaration)
- Modify: `src/supervise/orchestrator.cpp` (implementation)
- Test: `tests/integration/test_orchestrator.cpp` (new case)

- [ ] **Step 1: Write the failing test**

Append to `tests/integration/test_orchestrator.cpp`:

```cpp
TEST_CASE("orchestrator: restart bounces one process, leaves others running") {
    fpvd::Orchestrator orch;
    orch.add({"a", {"/bin/sh", "-c", "sleep 30"}, {}, fpvd::RestartPolicy::Always, {}});
    orch.add({"b", {"/bin/sh", "-c", "sleep 30"}, {}, fpvd::RestartPolicy::Always, {}});
    orch.startAll();
    std::this_thread::sleep_for(100ms);

    pid_t aBefore = orch.get("a")->pid();
    pid_t bBefore = orch.get("b")->pid();
    REQUIRE(aBefore > 0);
    REQUIRE(bBefore > 0);

    orch.restart("a");
    std::this_thread::sleep_for(100ms);

    CHECK(orch.get("a")->state() == fpvd::ProcState::Running);
    CHECK(orch.get("a")->pid() != aBefore);     // new process
    CHECK(orch.get("b")->pid() == bBefore);     // untouched

    orch.restart("does-not-exist");             // no-op, must not throw

    orch.stopAll();
}
```

- [ ] **Step 2: Run the test to verify it fails**

```bash
cmake --build build/host --target fpvd_tests -j
```
Expected: FAIL to compile — `Orchestrator` has no member `restart`.

- [ ] **Step 3: Declare and implement `restart`**

In `src/supervise/orchestrator.hpp`, after the line:

```cpp
    void remove(const std::string& name);  // shuts down if running
```
add:

```cpp
    void restart(const std::string& name);  // bounce one process; no-op if absent
```

In `src/supervise/orchestrator.cpp`, after the `remove(...)` function, add:

```cpp
void Orchestrator::restart(const std::string& name) {
    auto it = sups_.find(name);
    if (it == sups_.end()) return;
    it->second->shutdown();   // SIGTERM + join (no reinit flag → no self-respawn)
    it->second->start();      // fresh supervision loop, immediate restart
}
```

- [ ] **Step 4: Run the test to verify it passes**

```bash
cmake --build build/host --target fpvd_tests -j
./build/host/fpvd_tests --test-case="orchestrator: restart*"
```
Expected: PASS.

- [ ] **Step 5: Commit**

```bash
git add src/supervise/orchestrator.hpp src/supervise/orchestrator.cpp \
        tests/integration/test_orchestrator.cpp
git commit -m "feat(supervise): Orchestrator::restart bounces a single process"
```

---

### Task 6: Wire `WaybeamClient` into `Daemon` and restructure `apply()`

Drop `subs.encoder` from `needsRebuild`; push LIVE fields transactionally before commit; bounce only waybeam for RESTART fields.

**Files:**
- Modify: `src/daemon.hpp` (include + `waybeam_` member)
- Modify: `src/daemon.cpp` (ctor init + `apply()` restructure)
- Test: `tests/integration/test_daemon.cpp` (rewrite the encoder-while-enabled case + add 3 cases)

- [ ] **Step 1: Add the `WaybeamClient` member to the daemon**

In `src/daemon.hpp`, add the include after `#include "dynlink/runtime_config.hpp"`:

```cpp
#include "waybeam/client.hpp"
```

Then, in the private member list, find:

```cpp
    RadioInfo radio_;
    Orchestrator orch_;
```
Replace with:

```cpp
    RadioInfo radio_;
    WaybeamClient waybeam_;   // declared before dl_/orch_ for init order
    Orchestrator orch_;
```

- [ ] **Step 2: Initialize `waybeam_` in the constructor**

In `src/daemon.cpp`, replace:

```cpp
Daemon::Daemon(DaemonPaths paths)
    : paths_(std::move(paths)),
      dl_(paths_.dlEndpoints),
      dlGenerationId_(std::random_device{}()),
      startedAt_(std::chrono::steady_clock::now()) {
}
```
with:

```cpp
Daemon::Daemon(DaemonPaths paths)
    : paths_(std::move(paths)),
      waybeam_(paths_.dlEndpoints.encHost, paths_.dlEndpoints.encPort),
      dl_(paths_.dlEndpoints),
      dlGenerationId_(std::random_device{}()),
      startedAt_(std::chrono::steady_clock::now()) {
}
```
(Member declaration order is `paths_ … radio_, waybeam_, orch_ … dl_`, so this init order is valid.)

- [ ] **Step 3: Restructure `apply()` — compute the encoder plan + transactional live push**

In `src/daemon.cpp::apply()`, find the block that ends the pre-commit section (the `bfChanged` computation) followed by the overlay persist:

```cpp
    const bool bfChanged =
        nlohmann::json(effective_.link.beamforming) !=
            nlohmann::json(pending_.link.beamforming) ||
        effective_.link.width != pending_.link.width;

    // Persist overlay (sparse diff vs defaults).
    auto defaultsJ = defaultsJson();
```
Replace with:

```cpp
    const bool bfChanged =
        nlohmann::json(effective_.link.beamforming) !=
            nlohmann::json(pending_.link.beamforming) ||
        effective_.link.width != pending_.link.width;

    // Encoder reconcile (computed from the pre-commit diff). codec is excluded;
    // dynamic-link-owned fields are excluded while DL is enabled.
    auto wbDiff = waybeamConfigDiff(effective_, pending_, enabledNew);
    const bool encRestart = !wbDiff.restart.empty();   // any restart field ⇒ restart
    const bool encLive    = !encRestart && !wbDiff.live.empty();
    const bool encChanged = encRestart || encLive;

    // A full orchestrator rebuild is needed only for non-encoder subsystems.
    const bool needsRebuild = subs.telemetry ||
        !subs.servicesAffected.empty() || link.fullRestart;

    // Transactional LIVE push: apply before committing so a failed push fails the
    // apply with nothing changed and the radio link untouched. Skipped under a
    // full rebuild (it restarts waybeam + reloads the file) and on the dry path.
    if (reallyRestart && !needsRebuild && encLive) {
        if (!waybeam_.setFields(wbDiff.live)) {
            lastApply_ = {nowIso(), false, {},
                          std::string("waybeam: /api/v1/set failed")};
            return {false, {}, {}, std::string("waybeam: /api/v1/set failed"),
                    version_};
        }
    }

    // Persist overlay (sparse diff vs defaults).
    auto defaultsJ = defaultsJson();
```

- [ ] **Step 4: Report `encoder` from the new plan, and drop the old `needsRebuild`**

In the same function, find:

```cpp
    std::vector<std::string> restarted;
    if (subs.radio) restarted.push_back("radio");
    if (subs.encoder) restarted.push_back("encoder");
    if (subs.telemetry) restarted.push_back("telemetry");
```
Replace the encoder line so it reads:

```cpp
    std::vector<std::string> restarted;
    if (subs.radio) restarted.push_back("radio");
    if (encChanged) restarted.push_back("encoder");
    if (subs.telemetry) restarted.push_back("telemetry");
```

Then delete the now-duplicate old declaration further down:

```cpp
    const bool needsRebuild = subs.encoder || subs.telemetry ||
        !subs.servicesAffected.empty() || link.fullRestart;
```
(It has been moved up and no longer includes `subs.encoder`. Keep the large explanatory comment block that precedes it.)

- [ ] **Step 5: Bounce only waybeam for restart-class encoder changes (hot path)**

In the same function, find the start of the hot path:

```cpp
    if (reallyRestart) {
        // Hot path: no wfb restart. Route the in-process controller FIRST, so it
```
Insert the waybeam bounce right after the `if (reallyRestart) {` line, before that comment:

```cpp
    if (reallyRestart) {
        // Encoder restart-class change: waybeam.json was rewritten above; bounce
        // ONLY waybeam (wfb stays up, radio link preserved). On Star6E a /set-
        // driven reinit would self-respawn and race our supervisor, so fpvd owns
        // the restart. No-op if waybeam is not currently supervised.
        if (encRestart) orch_.restart("waybeam");
        // Hot path: no wfb restart. Route the in-process controller FIRST, so it
```

- [ ] **Step 6: Rewrite the obsolete encoder-rebuild test**

In `tests/integration/test_daemon.cpp`, replace the entire case `TEST_CASE("apply: encoder change while enabled rebuilds + restart-around")` (it uses the now-invalid `video.codec:"h264"`) with:

```cpp
TEST_CASE("apply: restart-class encoder change while enabled bounces only waybeam") {
    auto tmp = fs::temp_directory_path() / "fpvd-route-encoder";
    auto paths = makeRoutingPaths(tmp, 46802);
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // Enable DL and apply so the controller is running.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})")).ok);
    REQUIRE(d.apply(/*reallyRestart=*/true).ok);
    REQUIRE(d.dynamicLinkStatus().running);

    auto namesBefore = d.orchestrator().names();

    // A resolution change is a RESTART-class encoder field, NOT dynamic-link-
    // locked and NOT a dynamicLink input — so it no longer rebuilds the
    // orchestrator and the controller is never bounced.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"video":{"resolution":"1280x720"}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "encoder")
          != ar.restarted.end());
    CHECK(d.orchestrator().names() == namesBefore);   // no full rebuild
    CHECK(d.dynamicLinkStatus().running);             // controller untouched

    fs::remove_all(tmp);
}
```

- [ ] **Step 7: Add the LIVE-push, RESTART-rewrite, and failure cases**

Append to `tests/integration/test_daemon.cpp` (the `FakeWb` fixture mirrors the one in test_waybeam_client.cpp; include `<httplib.h>` near the top of the file if not already present):

```cpp
// Fake waybeam HTTP server for the encoder-apply path.
namespace {
struct FakeWbDaemon {
    httplib::Server srv;
    std::vector<std::string> hits;
    std::mutex mu;
    int port{0};
    std::thread th;
    FakeWbDaemon() {
        srv.Get("/api/v1/set", [&](const httplib::Request& r, httplib::Response& res) {
            std::lock_guard<std::mutex> lk(mu);
            hits.push_back(r.target);
            res.set_content("ok", "text/plain");
        });
        port = srv.bind_to_any_port("127.0.0.1");
        th = std::thread([&] { srv.listen_after_bind(); });
        srv.wait_until_ready();
    }
    ~FakeWbDaemon() { srv.stop(); th.join(); }
    size_t count() { std::lock_guard<std::mutex> lk(mu); return hits.size(); }
    std::string last() { std::lock_guard<std::mutex> lk(mu); return hits.back(); }
};
} // namespace

TEST_CASE("apply: LIVE encoder change pushes /api/v1/set, no rebuild") {
    FakeWbDaemon wb;
    auto tmp = fs::temp_directory_path() / "fpvd-enc-live";
    auto paths = makeRoutingPaths(tmp, 46810);
    paths.dlEndpoints.encPort = static_cast<uint16_t>(wb.port);  // point at fake waybeam
    fpvd::Daemon d(paths);
    d.bootstrap(false);
    auto namesBefore = d.orchestrator().names();

    // DL disabled → bitrate is fpvd-owned and LIVE.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"video":{"bitrate":4096}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);

    REQUIRE(wb.count() == 1);
    CHECK(wb.last().find("video0.bitrate=4096") != std::string::npos);
    CHECK(d.orchestrator().names() == namesBefore);   // no rebuild
    CHECK(d.effective().video.bitrate == 4096);

    fs::remove_all(tmp);
}

TEST_CASE("apply: RESTART encoder change rewrites file, issues no /set") {
    FakeWbDaemon wb;
    auto tmp = fs::temp_directory_path() / "fpvd-enc-restart";
    auto paths = makeRoutingPaths(tmp, 46811);
    paths.dlEndpoints.encPort = static_cast<uint16_t>(wb.port);
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"video":{"resolution":"1280x720"}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);

    // Restart-class path uses the file + waybeam bounce, never /api/v1/set.
    CHECK(wb.count() == 0);
    std::ifstream wf(paths.waybeamJsonPath);
    nlohmann::json wj; wf >> wj;
    CHECK(wj["video0"]["size"] == "1280x720");
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "encoder")
          != ar.restarted.end());

    fs::remove_all(tmp);
}

TEST_CASE("apply: failed /api/v1/set fails the apply with effective unchanged") {
    auto tmp = fs::temp_directory_path() / "fpvd-enc-fail";
    auto paths = makeRoutingPaths(tmp, 46812);
    // encPort 0 → connection refused; the LIVE push must fail.
    paths.dlEndpoints.encPort = 0;
    fpvd::Daemon d(paths);
    d.bootstrap(false);
    int before = d.effective().video.bitrate;

    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"video":{"bitrate":4096}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    CHECK_FALSE(ar.ok);
    CHECK(d.effective().video.bitrate == before);   // not committed
    CHECK(d.version() == 0);                         // no version bump

    fs::remove_all(tmp);
}
```

Ensure these includes are present at the top of `tests/integration/test_daemon.cpp` (add any that are missing): `#include <httplib.h>`, `#include <mutex>`, `#include <vector>`.

- [ ] **Step 8: Build and run the daemon suite — verify it passes**

```bash
cmake -S . -B build/host -DCMAKE_BUILD_TYPE=Debug
cmake --build build/host --target fpvd_tests -j
./build/host/fpvd_tests --test-case="daemon*,apply*"
```
Expected: all `daemon*` / `apply*` cases PASS, including the three new encoder cases and the rewritten restart-class case.

- [ ] **Step 9: Run the FULL suite**

```bash
./build/host/fpvd_tests
```
Expected: entire suite green (no regressions in DL, translate, validate, orchestrator, http).

- [ ] **Step 10: Commit**

```bash
git add src/daemon.hpp src/daemon.cpp tests/integration/test_daemon.cpp
git commit -m "feat(daemon): apply encoder config via waybeam API instead of rebuild"
```

---

## Self-Review

**Spec coverage:**
- LIVE batched `/api/v1/set` → Task 1 (`setFields`) + Task 6 Step 3 (pre-commit push). ✓
- RESTART → file rewrite + waybeam-only restart → Task 5 (`Orchestrator::restart`) + Task 6 Step 5. ✓
- `subs.encoder` removed from `needsRebuild` → Task 6 Step 4. ✓
- Transactional "fail apply, keep link up" → Task 6 Step 3 (push before commit, early return) + test in Step 7. ✓
- Shared `WaybeamClient` reused by `EncoderClient` → Tasks 1 + 2. ✓
- `waybeamConfigDiff` mapping table + codec dropped → Task 3. ✓
- Codec pinned to h265 → Task 4. ✓
- DL-owned field exclusion when enabled → Task 3 (diff) + test; Task 6 uses `enabledNew`. ✓
- Tests (unit diff, validate, orchestrator restart, daemon integration incl. /set failure) → Tasks 1,3,4,5,6. ✓

**Placeholder scan:** none — every step has concrete code/commands.

**Type consistency:** `WaybeamClient::setFields(const std::map<std::string,std::string>&)` / `get(const std::string&)` are used identically in `EncoderClient` (Task 2) and `Daemon` (Task 6). `WaybeamFieldDiff{live,restart}` defined in Task 3 and consumed in Task 6. `Orchestrator::restart(const std::string&)` defined Task 5, called Task 6. `encChanged`/`encRestart`/`encLive` defined once in Task 6 Step 3 and used in Steps 4–5.

**Out-of-plan note:** after the suite is green, run the `verify` skill for the full dual-backend / cross build before opening the PR.
