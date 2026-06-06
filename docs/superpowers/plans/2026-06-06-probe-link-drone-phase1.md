# Probe Link — Drone Injector (Phase 1a) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** The drone fpvd daemon spawns, per configured probe MCS, a dedicated FEC-off `wfb_tx` on its own `radio_port` (mirroring the video PHY) plus a paced feeder that injects throwaway MTU-sized packets — observe-only, no control change — and surfaces the probe in `/status`.

**Architecture:** A new `probe` config block lists candidate MCS rungs. A pure function `buildProbeSpecs()` turns that config into orchestrator `SupervisedSpec` entries (one `wfb_tx` + one feeder per MCS), seeded in `Daemon::seedOrchestrator()` exactly like the existing wfb/video specs. The feeder is the validated `feeder.c` from the MVP, cross-compiled by `deploy.sh` and installed to `/usr/libexec/fpvd/probe-feeder`. The GS-side per-MCS PER/RSSI measurement is a **separate** plan (Phase 1b) — this plan ends at "drone transmits measurable probe streams," verifiable with the MVP GS rig (`tools/probe-mvp/`).

**Tech Stack:** C++17, nlohmann/json (vendored), doctest (vendored), CMake + nix cross-toolchain (`armv7l-unknown-linux-musleabihf`), OpenIPC/busybox drone.

**Scope decisions (from `docs/superpowers/specs/2026-06-06-probe-driven-link-control-design.md`):**
- Phase 1a uses a **fixed configured `probe.mcsList`** (not the adaptive "window above current MCS" — that needs current-MCS tracking and lands in Phase 2). YAGNI.
- The feeder is a **standalone binary** spawned by the orchestrator (matches the existing helper-spawn pattern; isolates the real-time send loop). Process-count cost is accepted for an operator-enabled observe-only feature.
- Probe `wfb_tx` mirrors the video PHY (`-B`, `-S`, `-L`, long GI) and differs only in `-M <mcs>` and `-k 1 -n 1` (FEC off → raw on-air loss).

---

## File Structure

| File | Change | Responsibility |
|---|---|---|
| `drone/src/config/schema.hpp` | modify | add `Probe` struct + `Config.probe` |
| `drone/src/config/validate.cpp` | modify | bounds + port-collision validation |
| `drone/src/probe/probe_specs.hpp` | create | `buildProbeSpecs()` declaration |
| `drone/src/probe/probe_specs.cpp` | create | `buildProbeSpecs()` pure function |
| `drone/src/probe/feeder.c` | create | paced seq+MTU UDP sender (from MVP) |
| `drone/src/daemon.cpp` | modify | seed probe specs into the orchestrator |
| `drone/src/status.cpp` | modify | add `probe` object to `/status` |
| `drone/CMakeLists.txt` | modify | add `probe_specs.cpp` + the two new tests |
| `drone/tests/unit/test_probe_schema.cpp` | create | schema + validation tests |
| `drone/tests/unit/test_probe_specs.cpp` | create | `buildProbeSpecs()` argv tests |
| `deploy/drone/deploy.sh` | modify | cross-build + install `probe-feeder` |

**Reserved `radio_port`s (must not collide):** video=0, tlm=16, tun=32, tlm-rx=144, tun-rx=160. Probe defaults start at `basePort=50`.

**Build/test commands (run from `drone/`):**
- Host tests: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j && ctest --test-dir build --output-on-failure`
- Run one test: `./build/fpvd_tests --test-case="<name>"`

---

## Task 1: Probe config schema

**Files:**
- Modify: `drone/src/config/schema.hpp` (add `Probe` struct before `struct Config`; add field + macro arg to `Config`)
- Test: `drone/tests/unit/test_probe_schema.cpp` (create)
- Modify: `drone/CMakeLists.txt` (register the new test)

- [ ] **Step 1: Write the failing test**

Create `drone/tests/unit/test_probe_schema.cpp`:

```cpp
#include "doctest.h"
#include "config/schema.hpp"

using fpvd::Config;
using nlohmann::json;

TEST_CASE("probe schema: absent block uses defaults") {
    Config c = json::parse(R"({"link":{}})").get<Config>();
    CHECK(c.probe.enabled == false);
    CHECK(c.probe.mcsList.empty());
    CHECK(c.probe.pps == 25);
    CHECK(c.probe.packetBytes == 1400);
    CHECK(c.probe.basePort == 50);
    CHECK(c.probe.baseFeedPort == 6700);
}

TEST_CASE("probe schema: parses and round-trips") {
    json j = json::parse(R"({"probe":{"enabled":true,"mcsList":[3,5,7],
        "pps":20,"packetBytes":1400,"basePort":50,"baseFeedPort":6700}})");
    Config c = j.get<Config>();
    CHECK(c.probe.enabled);
    CHECK(c.probe.mcsList == std::vector<int>{3, 5, 7});
    CHECK(c.probe.pps == 20);
    json out = c;
    CHECK(out["probe"] == j["probe"]);
}
```

- [ ] **Step 2: Register the test and run it to verify it fails**

In `drone/CMakeLists.txt`, add to the `target_sources(fpvd_tests PRIVATE ...)` list (after `tests/unit/test_schema.cpp`):

```cmake
        tests/unit/test_probe_schema.cpp
```

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j 2>&1 | tail -20`
Expected: FAIL — compile error, `Config` has no member `probe`.

- [ ] **Step 3: Add the schema struct**

In `drone/src/config/schema.hpp`, add immediately before `struct Config {` (after the `Service` macro, line ~163):

```cpp
struct Probe {
    bool enabled{false};
    std::vector<int> mcsList{};   // candidate MCS rungs; each gets its own radio_port
    int pps{25};                  // packets/sec per stream
    int packetBytes{1400};        // datagram size (mirror video MTU)
    int basePort{50};             // radio_port for mcsList[0]; +i per stream
    int baseFeedPort{6700};       // wfb_tx -u feed port for mcsList[0]; +i per stream
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Probe, enabled, mcsList, pps,
                                                packetBytes, basePort, baseFeedPort)
```

Then add `Probe probe{};` to `struct Config` (after `std::map<std::string, Service> services{};`) and add `probe` to its macro:

```cpp
struct Config {
    Link link{};
    Video video{};
    Image image{};
    Telemetry telemetry{};
    Recording recording{};
    DynamicLink dynamicLink{};
    std::map<std::string, Service> services{};
    Probe probe{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Config, link, video, image,
                                               telemetry, recording,
                                               dynamicLink, services, probe)
```

- [ ] **Step 4: Run the test to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="probe schema*"`
Expected: PASS (2 test cases).

- [ ] **Step 5: Commit**

```bash
git add drone/src/config/schema.hpp drone/tests/unit/test_probe_schema.cpp drone/CMakeLists.txt
git commit -m "feat(drone/probe): add probe config schema"
```

---

## Task 2: Probe config validation

**Files:**
- Modify: `drone/src/config/validate.cpp` (add a `probe` block inside `validate()`)
- Test: `drone/tests/unit/test_probe_schema.cpp` (append cases)

- [ ] **Step 1: Write the failing tests**

Append to `drone/tests/unit/test_probe_schema.cpp`:

```cpp
#include "config/validate.hpp"
using fpvd::validate;

static bool has_path(const std::vector<fpvd::ValidationError>& e, const std::string& p) {
    for (auto& v : e) if (v.path == p) return true;
    return false;
}

TEST_CASE("probe validate: disabled probe is always valid") {
    Config c{};                    // probe.enabled defaults false
    c.probe.mcsList = {99};        // garbage ignored while disabled
    CHECK(validate(c).empty());
}

TEST_CASE("probe validate: rejects bad enabled config") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {8};         // out of 0..7
    c.probe.pps = 0;               // < 1
    auto errs = validate(c);
    CHECK(has_path(errs, "probe.mcsList"));
    CHECK(has_path(errs, "probe.pps"));
}

TEST_CASE("probe validate: rejects reserved radio_port collision") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {5};
    c.probe.basePort = 32;         // collides with the tun radio_port
    CHECK(has_path(validate(c), "probe.basePort"));
}

TEST_CASE("probe validate: accepts a sane enabled config") {
    Config c{};
    c.probe.enabled = true;
    c.probe.mcsList = {3, 5, 7};
    auto errs = validate(c);
    for (auto& e : errs) CHECK(e.path.rfind("probe", 0) != 0);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="probe validate*"`
Expected: FAIL — `probe validate: rejects bad enabled config` fails (no probe validation yet).

- [ ] **Step 3: Add the validation block**

In `drone/src/config/validate.cpp`, add `#include <set>` is already present. Insert before `return errs;` (line ~156):

```cpp
    // probe
    {
        const auto& pr = c.probe;
        if (pr.enabled) {
            if (pr.mcsList.empty())
                errs.push_back({"probe.mcsList", "must be non-empty when enabled"});
            for (int m : pr.mcsList)
                if (m < 0 || m > 7)
                    errs.push_back({"probe.mcsList", "each MCS must be 0..7"});
            if (pr.pps < 1 || pr.pps > 1000)
                errs.push_back({"probe.pps", "must be 1..1000"});
            if (pr.packetBytes < 12 || pr.packetBytes > 1445)
                errs.push_back({"probe.packetBytes", "must be 12..1445"});
            static const std::set<int> reserved{0, 16, 32, 144, 160};
            for (size_t i = 0; i < pr.mcsList.size(); ++i) {
                int port = pr.basePort + static_cast<int>(i);
                if (reserved.count(port)) {
                    errs.push_back({"probe.basePort",
                        "probe radio_port collides with a reserved port (0/16/32/144/160)"});
                    break;
                }
            }
        }
    }
```

- [ ] **Step 4: Run to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="probe validate*"`
Expected: PASS (4 test cases).

- [ ] **Step 5: Commit**

```bash
git add drone/src/config/validate.cpp drone/tests/unit/test_probe_schema.cpp
git commit -m "feat(drone/probe): validate probe config bounds + port collisions"
```

---

## Task 3: Probe spec builder

**Files:**
- Create: `drone/src/probe/probe_specs.hpp`, `drone/src/probe/probe_specs.cpp`
- Test: `drone/tests/unit/test_probe_specs.cpp`
- Modify: `drone/CMakeLists.txt` (add source to `fpvd_core`, test to `fpvd_tests`)

- [ ] **Step 1: Write the failing test**

Create `drone/tests/unit/test_probe_specs.cpp`:

```cpp
#include "doctest.h"
#include "probe/probe_specs.hpp"

using namespace fpvd;

static std::string joined(const std::vector<std::string>& a) {
    std::string s;
    for (auto& x : a) { s += x; s += ' '; }
    return s;
}

TEST_CASE("buildProbeSpecs: empty when disabled") {
    Config c{};
    CHECK(buildProbeSpecs(c, "wlan0", "/etc/drone.key", "/usr/libexec/fpvd/probe-feeder").empty());
}

TEST_CASE("buildProbeSpecs: one wfb_tx + one feeder per MCS, PHY-mirrored, FEC off") {
    Config c{};
    c.link.width = 20; c.link.stbc = true; c.link.ldpc = true; c.link.linkId = 7669206;
    c.probe.enabled = true;
    c.probe.mcsList = {5, 7};
    c.probe.pps = 20; c.probe.packetBytes = 1400;
    c.probe.basePort = 50; c.probe.baseFeedPort = 6700;

    auto specs = buildProbeSpecs(c, "wlan0", "/etc/drone.key", "/usr/libexec/fpvd/probe-feeder");
    REQUIRE(specs.size() == 4);   // tx+feed for each of 2 MCS

    // stream 0 = MCS5 on port 50, feed 6700
    CHECK(specs[0].name == "probe-tx-mcs5");
    std::string tx0 = joined(specs[0].argv);
    CHECK(tx0.find("/usr/bin/wfb_tx ") == 0);
    CHECK(tx0.find("-M 5 ") != std::string::npos);
    CHECK(tx0.find("-B 20 ") != std::string::npos);
    CHECK(tx0.find("-S 1 ") != std::string::npos);
    CHECK(tx0.find("-L 1 ") != std::string::npos);
    CHECK(tx0.find("-k 1 ") != std::string::npos);
    CHECK(tx0.find("-n 1 ") != std::string::npos);
    CHECK(tx0.find("-i 7669206 ") != std::string::npos);
    CHECK(tx0.find("-p 50 ") != std::string::npos);
    CHECK(tx0.find("-u 6700 ") != std::string::npos);
    CHECK(tx0.find(" wlan0 ") != std::string::npos);

    CHECK(specs[1].name == "probe-feed-mcs5");
    CHECK(specs[1].argv == std::vector<std::string>{
        "/usr/libexec/fpvd/probe-feeder", "6700", "20", "1400"});
    REQUIRE(specs[1].startAfter.size() == 1);
    CHECK(specs[1].startAfter[0] == "probe-tx-mcs5");

    // stream 1 = MCS7 on port 51, feed 6701
    CHECK(specs[2].name == "probe-tx-mcs7");
    CHECK(joined(specs[2].argv).find("-p 51 ") != std::string::npos);
    CHECK(specs[3].argv[1] == "6701");
}
```

- [ ] **Step 2: Register source + test, run to verify it fails**

In `drone/CMakeLists.txt`:
- add `src/probe/probe_specs.cpp` to the `target_sources(fpvd_core PRIVATE ...)` list (after `src/translate/wfb.cpp`)
- add `tests/unit/test_probe_specs.cpp` to the `target_sources(fpvd_tests PRIVATE ...)` list

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j 2>&1 | tail -20`
Expected: FAIL — `probe/probe_specs.hpp` not found.

- [ ] **Step 3: Write the header**

Create `drone/src/probe/probe_specs.hpp`:

```cpp
#pragma once
#include "config/schema.hpp"
#include "supervise/supervisor.hpp"
#include <string>
#include <vector>

namespace fpvd {

// Observe-only probe link: one FEC-off wfb_tx + one feeder per probe MCS,
// mirroring the video PHY (width/stbc/ldpc, long GI). Returns empty when the
// probe is disabled. index i -> radio_port basePort+i, feed port baseFeedPort+i.
std::vector<SupervisedSpec> buildProbeSpecs(const Config& c,
                                            const std::string& iface,
                                            const std::string& key,
                                            const std::string& feederPath);

} // namespace fpvd
```

- [ ] **Step 4: Write the implementation**

Create `drone/src/probe/probe_specs.cpp`:

```cpp
#include "probe/probe_specs.hpp"
#include "link_width.hpp"

namespace fpvd {

std::vector<SupervisedSpec> buildProbeSpecs(const Config& c,
                                            const std::string& iface,
                                            const std::string& key,
                                            const std::string& feederPath) {
    std::vector<SupervisedSpec> out;
    if (!c.probe.enabled) return out;
    const auto& p = c.probe;
    for (size_t i = 0; i < p.mcsList.size(); ++i) {
        const int mcs  = p.mcsList[i];
        const int port = p.basePort + static_cast<int>(i);
        const int feed = p.baseFeedPort + static_cast<int>(i);
        const std::string txName = "probe-tx-mcs" + std::to_string(mcs);
        const std::string fdName = "probe-feed-mcs" + std::to_string(mcs);

        SupervisedSpec tx{};
        tx.name = txName;
        tx.argv = {
            "/usr/bin/wfb_tx", "-K", key,
            "-M", std::to_string(mcs),
            "-B", std::to_string(modulationWidth(c.link.width)),
            "-S", c.link.stbc ? "1" : "0",
            "-L", c.link.ldpc ? "1" : "0",
            "-k", "1", "-n", "1",
            "-i", std::to_string(c.link.linkId),
            "-p", std::to_string(port),
            "-u", std::to_string(feed),
            iface,
        };
        tx.restart = RestartPolicy::Always;
        out.push_back(std::move(tx));

        SupervisedSpec fd{};
        fd.name = fdName;
        fd.argv = {feederPath, std::to_string(feed),
                   std::to_string(p.pps), std::to_string(p.packetBytes)};
        fd.restart = RestartPolicy::Always;
        fd.startAfter = {txName};
        out.push_back(std::move(fd));
    }
    return out;
}

} // namespace fpvd
```

- [ ] **Step 5: Run to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="buildProbeSpecs*"`
Expected: PASS (2 test cases).

- [ ] **Step 6: Commit**

```bash
git add drone/src/probe/probe_specs.hpp drone/src/probe/probe_specs.cpp \
        drone/tests/unit/test_probe_specs.cpp drone/CMakeLists.txt
git commit -m "feat(drone/probe): build per-MCS wfb_tx + feeder supervised specs"
```

---

## Task 4: Feeder source in-tree

**Files:**
- Create: `drone/src/probe/feeder.c` (the validated MVP feeder; not a CMake target — cross-built by deploy.sh in Task 7)

- [ ] **Step 1: Add the feeder source**

Create `drone/src/probe/feeder.c` with exactly this content (this is the hardware-validated MVP feeder):

```c
// feeder.c — paced probe traffic generator for the fpvd probe link.
// Sends seq-numbered UDP datagrams to 127.0.0.1:<port> at a fixed rate, for a
// dedicated FEC-off wfb_tx to inject at a fixed MCS on its own radio_port.
//
// Usage: probe-feeder <port> <pps> <size> [duration_s]
// Wire: [0..3]='PRB0'  [4..11]=big-endian uint64 seq  [12..]=0xA5 fill
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <unistd.h>
#include <time.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>

int main(int argc, char **argv) {
    if (argc < 4) {
        fprintf(stderr, "usage: %s <port> <pps> <size> [duration_s]\n", argv[0]);
        return 2;
    }
    int port = atoi(argv[1]);
    int pps  = atoi(argv[2]);
    int size = atoi(argv[3]);
    long dur = argc > 4 ? atol(argv[4]) : 0;
    if (size < 12) size = 12;
    if (pps  < 1)  pps  = 1;

    int fd = socket(AF_INET, SOCK_DGRAM, 0);
    if (fd < 0) { perror("socket"); return 1; }
    struct sockaddr_in a;
    memset(&a, 0, sizeof a);
    a.sin_family = AF_INET;
    a.sin_port   = htons(port);
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (connect(fd, (struct sockaddr *)&a, sizeof a) < 0) { perror("connect"); return 1; }

    unsigned char *buf = malloc(size);
    memset(buf, 0xA5, size);
    memcpy(buf, "PRB0", 4);

    long ns = 1000000000L / pps;
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    uint64_t seq = 0;
    time_t t0 = time(NULL);

    for (;;) {
        for (int i = 0; i < 8; i++) buf[4 + i] = (seq >> (56 - 8 * i)) & 0xff;
        (void)send(fd, buf, size, 0);
        seq++;
        t.tv_nsec += ns;
        while (t.tv_nsec >= 1000000000L) { t.tv_nsec -= 1000000000L; t.tv_sec++; }
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
        if (dur && (time(NULL) - t0) >= dur) break;
    }
    return 0;
}
```

- [ ] **Step 2: Host smoke test (compiles + runs + rate is right)**

Run:
```bash
cc -O2 -o /tmp/probe-feeder drone/src/probe/feeder.c
timeout 2 /tmp/probe-feeder 9999 50 1400 2; echo "exit=$?"
```
Expected: exit=0 within ~2 s (sends to a dead local port harmlessly; verifies it builds and the pacing loop runs).

- [ ] **Step 3: Commit**

```bash
git add drone/src/probe/feeder.c
git commit -m "feat(drone/probe): in-tree probe feeder source"
```

---

## Task 5: Seed probe specs into the orchestrator

**Files:**
- Modify: `drone/src/daemon.cpp` (`#include "probe/probe_specs.hpp"`; seed in `seedOrchestrator()`)
- Test: `drone/tests/integration/test_daemon.cpp` (append a case using its existing harness)

- [ ] **Step 1: Write the failing test**

Open `drone/tests/integration/test_daemon.cpp`, read how it constructs a `Daemon` and triggers `seedOrchestrator()` (it builds a Daemon with a fake radio-up script and calls the bootstrap/apply path; mirror that exact setup). Append a case that enables the probe and asserts the specs landed in the orchestrator:

```cpp
TEST_CASE("daemon: seeds probe streams when probe.enabled") {
    // --- mirror the existing test_daemon harness setup above this case ---
    // Build a Daemon `d` with the fake radio-up fixture, set its effective
    // config to enable the probe, then run the same bootstrap/apply call the
    // other cases use so seedOrchestrator() runs.
    //   d.mutableEffectiveForTest().probe.enabled = true;
    //   d.mutableEffectiveForTest().probe.mcsList = {5};
    //   <bootstrap/apply call as in the harness>
    CHECK(d.orchestrator().get("probe-tx-mcs5")  != nullptr);
    CHECK(d.orchestrator().get("probe-feed-mcs5") != nullptr);
}
```

> If the harness has no test seam to mutate `effective_`, the smallest seam is to point its config fixture file at a JSON that includes `"probe":{"enabled":true,"mcsList":[5]}`. Match whatever the adjacent cases already do — do not invent a new constructor.

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="daemon: seeds probe*"`
Expected: FAIL — `get("probe-tx-mcs5")` is `nullptr` (not seeded yet).

- [ ] **Step 2: Seed the specs**

In `drone/src/daemon.cpp`, add the include near the other config/translate includes at the top:

```cpp
#include "probe/probe_specs.hpp"
```

In `Daemon::seedOrchestrator()`, add at the very end of the function (after the `effective_.services` loop, before the closing brace at line ~134):

```cpp
    // Observe-only probe link: extra FEC-off wfb_tx + feeder per probe MCS.
    static const std::string kProbeFeeder = "/usr/libexec/fpvd/probe-feeder";
    for (auto& s : buildProbeSpecs(effective_, iface, key, kProbeFeeder))
        orch_.add(std::move(s));
```

- [ ] **Step 3: Run to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="daemon: seeds probe*"`
Expected: PASS.

- [ ] **Step 4: Run the full suite (no regressions)**

Run: `ctest --test-dir build --output-on-failure`
Expected: all pass.

- [ ] **Step 5: Commit**

```bash
git add drone/src/daemon.cpp drone/tests/integration/test_daemon.cpp
git commit -m "feat(drone/probe): seed probe streams into the orchestrator"
```

---

## Task 6: Probe status in /status

**Files:**
- Modify: `drone/src/status.cpp` (`buildStatus()` — add a `probe` object)
- Test: `drone/tests/integration/test_http_handlers.cpp` (append, using its existing harness) OR a focused status test

- [ ] **Step 1: Write the failing test**

The per-stream process state already appears in the existing `processes` array (probe specs are normal supervised processes). Add a top-level `probe` summary so consumers can find the streams without name-matching. Append to `drone/tests/integration/test_http_handlers.cpp` (mirror its setup for building a `Daemon`/status JSON):

```cpp
TEST_CASE("status: includes probe summary") {
    // --- mirror the existing handler-test harness to get `buildStatus(d)` ---
    //   d.mutableEffectiveForTest().probe.enabled = true;
    //   d.mutableEffectiveForTest().probe.mcsList = {3, 5};
    auto j = buildStatus(d);
    REQUIRE(j.contains("probe"));
    CHECK(j["probe"]["enabled"] == true);
    CHECK(j["probe"]["mcsList"] == nlohmann::json::array({3, 5}));
}
```

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="status: includes probe*"`
Expected: FAIL — `j` has no `probe` key.

- [ ] **Step 2: Add the probe object**

In `drone/src/status.cpp`, in the final `return { ... };` of `buildStatus()`, add a `probe` entry after `{"dynamicLink", dlj}` (note: comma after `dlj`):

```cpp
        {"dynamicLink", dlj},
        {"probe", {
            {"enabled", d.effective().probe.enabled},
            {"mcsList", d.effective().probe.mcsList}
        }}
```

- [ ] **Step 3: Run to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="status: includes probe*"`
Expected: PASS.

- [ ] **Step 4: Commit**

```bash
git add drone/src/status.cpp drone/tests/integration/test_http_handlers.cpp
git commit -m "feat(drone/probe): surface probe summary in /status"
```

---

## Task 7: Deploy the feeder + on-hardware smoke

**Files:**
- Modify: `deploy/drone/deploy.sh` (cross-build `probe-feeder` via nix, install to `/usr/libexec/fpvd/probe-feeder`)

- [ ] **Step 1: Add the cross-build + install**

In `deploy/drone/deploy.sh`, in the build section (after the fpvd strip step, ~line 56), add:

```bash
# Cross-build the probe feeder (tiny static C binary; not part of the CMake CXX target).
PROBE_FEEDER="$(mktemp)"
( cd "$CPP" && nix-shell --run \
  "armv7l-unknown-linux-musleabihf-gcc -static -Os -o '$PROBE_FEEDER' src/probe/feeder.c && \
   armv7l-unknown-linux-musleabihf-strip -s '$PROBE_FEEDER'" )
echo "[build] probe-feeder: $(stat -c %s "$PROBE_FEEDER") bytes"
```

Add `rm -f "$PROBE_FEEDER"` to the existing `trap '... rm -f ...' EXIT` line.

In the push section (after the `radio-tune.sh` copy, ~line 67), add:

```bash
copy "$PROBE_FEEDER" /usr/libexec/fpvd/probe-feeder
```

And add `/usr/libexec/fpvd/probe-feeder` to the existing `chmod +x` remote command (~line 99):

```bash
remote 'chmod +x /usr/bin/fpvd.new /usr/libexec/fpvd/radio-up.sh /usr/libexec/fpvd/radio-tune.sh /usr/libexec/fpvd/probe-feeder /etc/init.d/S99fpvd'
```

- [ ] **Step 2: Deploy to the drone**

Run: `./deploy/drone/deploy.sh --host 192.168.10.152`
Expected: `[done] fpvd deployed`. Verify the binary landed:
```bash
ssh -o BatchMode=yes root@192.168.10.152 'ls -la /usr/libexec/fpvd/probe-feeder'
```

- [ ] **Step 3: Enable the probe and verify it transmits (uses the MVP GS rig)**

Enable on the drone (observe-only; static MCS2 video unaffected):
```bash
ssh -o BatchMode=yes root@192.168.10.152 \
  "curl -s -X PATCH http://127.0.0.1:8080/config -H 'Content-Type: application/json' \
   -d '{\"probe\":{\"enabled\":true,\"mcsList\":[3,5,7]}}' && curl -s -X POST http://127.0.0.1:8080/apply"
```
Confirm the supervised probe processes are up:
```bash
ssh -o BatchMode=yes root@192.168.10.152 \
  'pidof wfb_tx | tr " " "\n" | wc -l; ps w | grep -c "[p]robe-feeder"; curl -s http://127.0.0.1:8080/status | tr "," "\n" | grep -A3 probe'
```
Expected: wfb_tx count rises by 3 (probe-tx-mcs3/5/7), 3 probe-feeder processes, `/status` shows `"probe":{"enabled":true,...}`.

Confirm reception at the GS with the MVP receiver (it already parses these ports' seq stream):
```bash
ssh -o BatchMode=yes root@10.18.0.1 '/tmp/probe_gs.sh start 3,5,7 25; sleep 8; tail -4 /tmp/probe_log.console; /tmp/probe_gs.sh stop'
```
Expected: the GS console shows per-MCS recv/PER for m3/m5/m7 and **video `lost=0`** — i.e. the in-tree probe behaves exactly like the validated MVP injector.

- [ ] **Step 4: Disable the probe + commit**

```bash
ssh -o BatchMode=yes root@192.168.10.152 \
  "curl -s -X PATCH http://127.0.0.1:8080/config -d '{\"probe\":{\"enabled\":false}}' && curl -s -X POST http://127.0.0.1:8080/apply"
git add deploy/drone/deploy.sh
git commit -m "build(drone/probe): cross-build + install probe-feeder in deploy"
```

---

## Self-Review

**Spec coverage (Phase 1a portion of §4.3 / §5):**
- "drone probe injector ... own radio_port, FEC off, MTU-sized, PHY-mirroring video" → Tasks 1–4 (config + `buildProbeSpecs` + feeder).
- "fpvd spawns/supervises them" → Task 5 (orchestrator seeding, auto-restart via `RestartPolicy::Always`).
- "surfaced in status/logs, no control change" → Task 6 (`/status` probe object; processes array). Observe-only: nothing reads probe results on the drone; the GS-side measurement is Phase 1b.
- Deploy/ship → Task 7.
- **Deferred (correctly out of Phase 1a):** adaptive "window above current MCS" (needs current-MCS tracking → Phase 2); GS per-MCS PER/RSSI measurement → **Phase 1b plan**.

**Placeholder scan:** Tasks 5 & 6 reference the existing `test_daemon.cpp` / `test_http_handlers.cpp` harnesses for the `Daemon` setup rather than reproducing it (their construction is non-trivial and codebase-specific); the assertion code is given in full. The implementer must read those two files for the setup idiom — this is the one place exact code can't be pre-written without the harness in front of you. Everything else is complete.

**Type consistency:** `Probe` fields (`enabled`, `mcsList`, `pps`, `packetBytes`, `basePort`, `baseFeedPort`) are identical across schema (Task 1), validation (Task 2), builder (Task 3), and status (Task 6). `buildProbeSpecs` signature matches between header (Task 3 Step 3), impl (Step 4), and call site (Task 5). Process names `probe-tx-mcs<N>` / `probe-feed-mcs<N>` match between builder and tests. Feeder CLI `<port> <pps> <size>` matches between `feeder.c` (Task 4) and the feeder argv in `buildProbeSpecs` (Task 3).

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-06-probe-link-drone-phase1.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — I dispatch a fresh subagent per task, review between tasks, fast iteration.

**2. Inline Execution** — Execute tasks in this session using executing-plans, batch execution with checkpoints.

**Which approach?**
