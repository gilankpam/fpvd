# Probe Fixed-Stream Plumbing Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace the config-driven, variable-`mcsList` probe (Phase 1a/1b) with a single fixed probe stream that tracks `current+1` by live `CMD_SET_RADIO` retune, owned by the dynamic-link lifecycle, with no probe config — removing the drone video-glitch (`probeChanged → full orchestrator rebuild`) at its root.

**Architecture:** GS sends only the operating `{mcs}` on the existing dynlink wire (no wire change). The drone derives `min(mcs+1, 7)` and live-retunes one long-lived FEC-off probe `wfb_tx` (now with a `-C` control port) — no process restart. The GS runs one probe `wfb_rx` on the matching fixed `radio_port`; its RX_ANT-keyed aggregator follows the retuned rung automatically. Both ends start/stop with `dynamicLink.enabled`. Probe config is deleted from both schemas; parameters become constants.

**Tech Stack:** Drone — C++17, doctest, nlohmann/json, CMake. GS — Python 3.13, asyncio, pytest.

**Out of scope (deferred):**
- **Bitrate airtime reserve** (`(2/3 − probe_util)`): the drone OpenIPC bitrate calculator does **not exist yet** (it is Phase 3 — today the GS computes `bitrateKbps` and sends it in the `Decision` packet; the drone only consumes it). The reserve is already recorded in `docs/superpowers/specs/2026-06-06-probe-driven-link-control-design.md` §4.5 for when Phase 3 builds that calculator. The probe's ~1–2% airtime is absorbed by the GS calc's existing `utilization_factor` margin in the meantime.
- The promote-on-probe / demote-on-video-PER **selector** (parent spec Phase 2). Probe stays observe-only.
- The gated `current+2` second stream.

**Spec:** `docs/superpowers/specs/2026-06-07-probe-fixed-stream-plumbing-design.md`.

**Run tests:**
- Drone: from `/home/gilankpam/Projects/drone/fpvd/drone`: `cmake --build build -j && ./build/fpvd_tests` (filter: `./build/fpvd_tests -tc="*probe*"`). New test `.cpp` files must be added to `target_sources(fpvd_tests ...)` in `CMakeLists.txt` (~lines 71-110).
- GS: from `/home/gilankpam/Projects/drone/fpvd/gs`: `python -m pytest tests/ -q` (one file: `python -m pytest tests/unit/test_probe_controller.py -q`).

**Constants (both ends must agree on the port):** probe `radio_port = 50`, probe control port `8001` (video tx uses 8000), `probe_pps = 25`, `probe_packet_bytes = 1400`, probe feed UDP port `6700`, FEC `k/n = 1/1`, probe MCS ceiling `7`.

---

## File Structure

**Drone (`drone/`):**
| File | Change | Responsibility |
|---|---|---|
| `src/probe/probe_constants.hpp` | create | the probe constants (port/ctl/pps/bytes/feed/ceiling) |
| `src/probe/probe_specs.hpp` / `.cpp` | modify | build ONE fixed FEC-off `wfb_tx` (+`-C`) + feeder at an initial MCS |
| `src/config/schema.hpp` | modify | delete the `Probe` struct + `probe` field |
| `src/config/diff.cpp` / `diff.hpp` | modify | delete `probeChanged` |
| `src/daemon.cpp` | modify | seed/add/remove probe on `dynamicLink.enabled`; drop probe from `needsRebuild` |
| `src/dynlink/controller.{hpp,cpp}` | modify | a probe `WfbControlClient`; retune to `min(mcs+1,7)` in `dispatchTxApply` |
| `src/dynlink/runtime_config.{hpp,cpp}` | modify | carry the probe control port + ceiling into `DlRuntimeConfig` |
| `src/status.cpp` | modify | probe status reflects the dynlink-tied fixed stream |
| `tests/unit/test_probe_specs.cpp` | modify | single-stream + `-C` argv assertions |
| `tests/integration/test_daemon.cpp` | modify | replace probe-rebuild test with no-rebuild + dynlink-lifecycle tests |

**GS (`gs/`):**
| File | Change | Responsibility |
|---|---|---|
| `fpvdgs/schema.py` | modify | delete `probe` from `CONFIG_TOP_KEYS` + `_validate_probe` |
| `etc/defaults.json` | modify | delete the `probe` block |
| `fpvdgs/probe/config_build.py` | modify | single fixed port; derive from effective (no probe config) |
| `fpvdgs/probe/controller.py` | modify | spawn ONE `wfb_rx` on the fixed port (drop basePort/maxStreams loop) |
| `fpvdgs/supervisor.py` | modify | tie probe lifecycle + status to `dynamicLink.enabled` |
| `fpvdgs/api.py` | modify | delete `_route_probe` + the `probe` arg + the `wfb_changed` probe-exclusion |
| `fpvdgs/config.py` | verify | tolerate a legacy `probe` key in a deployed overlay (no hard-fail) |
| `tests/unit/test_probe_controller.py` | modify | single-stream tests |
| `tests/unit/test_probe_schema.py` | delete | probe schema is gone |
| `tests/unit/test_api.py` | modify | delete the probe routing tests + helper |
| `tests/integration/test_supervisor_e2e.py` | modify | probe block tied to dynamicLink |

Execute **Part A (drone) first** — it removes the glitch and is the harder half — then **Part B (GS)**. Each part is independently buildable and testable.

---

# Part A — Drone (C++)

## Task A1: Probe constants + remove the `Probe` config struct

**Files:** Create `drone/src/probe/probe_constants.hpp`; Modify `drone/src/config/schema.hpp`

- [ ] **Step 1: Create the constants header** — `drone/src/probe/probe_constants.hpp`:

```cpp
#pragma once
namespace fpvd {
// Fixed, observe-only probe link (one FEC-off wfb_tx tracking current+1).
// Both GS and drone must agree on kProbeRadioPort.
constexpr int     kProbeRadioPort   = 50;     // wfb radio_port (matches GS)
constexpr int     kProbeControlPort = 8001;   // wfb_tx -C control port (video uses 8000)
constexpr int     kProbeFeedPort    = 6700;   // wfb_tx -u feed port (feeder -> tx)
constexpr int     kProbePps         = 25;     // feeder packets/sec
constexpr int     kProbePacketBytes = 1400;   // feeder datagram size (mirror video MTU)
constexpr int     kProbeMcsCeiling  = 7;      // hardware MCS ceiling; clamp current+1 here
} // namespace fpvd
```

- [ ] **Step 2: Remove the `Probe` struct from the config.** In `drone/src/config/schema.hpp`, delete the `Probe` struct (lines ~165-174) and its `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Probe, ...)` line. Then remove `probe` from the top-level `Config`:

Change:
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
to:
```cpp
struct Config {
    Link link{};
    Video video{};
    Image image{};
    Telemetry telemetry{};
    Recording recording{};
    DynamicLink dynamicLink{};
    std::map<std::string, Service> services{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Config, link, video, image,
                                               telemetry, recording,
                                               dynamicLink, services)
```

> **Legacy-config tolerance:** `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT` ignores unknown JSON keys on deserialize, so a deployed `config.json` overlay that still carries a `"probe": {...}` block will load fine (the key is silently dropped). No migration code needed on the drone — but verify in Step 4 that the build + existing config-load tests stay green.

- [ ] **Step 3: Build to find all references** — `cd /home/gilankpam/Projects/drone/fpvd/drone && cmake --build build -j 2>&1 | head -40`
Expected: FAIL — compile errors at every `c.probe` / `effective_.probe` / `pending_.probe` use (in `probe_specs.cpp`, `diff.cpp`, `daemon.cpp`, `status.cpp`, tests). These are fixed by Tasks A2–A6. (This is the worklist for the rest of Part A.)

- [ ] **Step 4: Commit (after A2–A6 compile)** — defer the commit until `buildProbeSpecs` (A2) and the call sites compile; A1 alone does not build. Proceed to A2.

---

## Task A2: `buildProbeSpecs` → one fixed FEC-off `wfb_tx` (+ `-C`) + feeder

**Files:** Modify `drone/src/probe/probe_specs.hpp` / `.cpp`; Test `drone/tests/unit/test_probe_specs.cpp`

The new signature drops the `Config`-probe dependency and takes the initial probe MCS + the link PHY params it mirrors. The single `wfb_tx` gets a `-C <kProbeControlPort>` control port (so the controller can retune it) and FEC `1/1`.

- [ ] **Step 1: Rewrite the test** — replace `drone/tests/unit/test_probe_specs.cpp` with:

```cpp
#include "doctest.h"
#include "probe/probe_specs.hpp"
#include "probe/probe_constants.hpp"
#include <string>

using namespace fpvd;

static std::string joined(const std::vector<std::string>& v) {
    std::string s;
    for (auto& a : v) { s += a; s += ' '; }
    return s;
}

TEST_CASE("probe specs: one fec-off wfb_tx with control port + one feeder") {
    Config c{};
    c.link.linkId = 7669206;
    c.link.width = 20;
    c.link.stbc = true;
    c.link.ldpc = true;
    auto specs = buildProbeSpecs(c, "wlan0", "/etc/drone.key",
                                 "/usr/libexec/fpvd/probe-feeder", /*mcs=*/4);
    REQUIRE(specs.size() == 2);

    // wfb_tx
    const auto& tx = specs[0];
    CHECK(tx.name == "probe-tx");
    auto j = joined(tx.argv);
    CHECK(j.find("/usr/bin/wfb_tx ") != std::string::npos);
    CHECK(j.find(" -M 4 ") != std::string::npos);          // initial mcs
    CHECK(j.find(" -B 20 ") != std::string::npos);          // modulationWidth(20)
    CHECK(j.find(" -S 1 ") != std::string::npos);           // stbc
    CHECK(j.find(" -L 1 ") != std::string::npos);           // ldpc
    CHECK(j.find(" -k 1 ") != std::string::npos);           // FEC off
    CHECK(j.find(" -n 1 ") != std::string::npos);
    CHECK(j.find(" -C 8001 ") != std::string::npos);        // control port (retune)
    CHECK(j.find(" -i 7669206 ") != std::string::npos);
    CHECK(j.find(" -p 50 ") != std::string::npos);          // fixed radio_port
    CHECK(j.find(" -u 6700 ") != std::string::npos);        // feed port
    CHECK(j.find(" wlan0 ") != std::string::npos);

    // feeder
    const auto& fd = specs[1];
    CHECK(fd.name == "probe-feed");
    CHECK(fd.argv == std::vector<std::string>{
        "/usr/libexec/fpvd/probe-feeder", "6700", "25", "1400"});
    CHECK(fd.startAfter == std::vector<std::string>{"probe-tx"});
}

TEST_CASE("probe specs: mcs is clamped to the ceiling") {
    Config c{};
    auto specs = buildProbeSpecs(c, "wlan0", "/etc/drone.key", "/feeder", /*mcs=*/9);
    REQUIRE(specs.size() == 2);
    CHECK(joined(specs[0].argv).find(" -M 7 ") != std::string::npos);  // clamped to kProbeMcsCeiling
}
```

- [ ] **Step 2: Run to verify it fails** — `cmake --build build -j && ./build/fpvd_tests -tc="probe specs*"`
Expected: FAIL/compile error — old `buildProbeSpecs` signature + behavior.

- [ ] **Step 3: Implement.** Replace `drone/src/probe/probe_specs.hpp` body:

```cpp
#pragma once
#include "config/schema.hpp"
#include "supervise/supervisor.hpp"
#include <string>
#include <vector>

namespace fpvd {

// Observe-only probe link: ONE FEC-off wfb_tx (with a -C control port for live
// MCS retune) + one feeder, mirroring the video PHY (width/stbc/ldpc, long GI),
// FEC off (k=1 n=1). `mcs` is the initial rung; it is clamped to kProbeMcsCeiling.
// Lifecycle is owned by the caller (seed when dynamicLink is enabled).
std::vector<SupervisedSpec> buildProbeSpecs(const Config& c,
                                            const std::string& iface,
                                            const std::string& key,
                                            const std::string& feederPath,
                                            int mcs);

} // namespace fpvd
```

Replace `drone/src/probe/probe_specs.cpp`:

```cpp
#include "probe/probe_specs.hpp"
#include "probe/probe_constants.hpp"
#include "link_width.hpp"
#include <algorithm>

namespace fpvd {

std::vector<SupervisedSpec> buildProbeSpecs(const Config& c,
                                            const std::string& iface,
                                            const std::string& key,
                                            const std::string& feederPath,
                                            int mcs) {
    std::vector<SupervisedSpec> out;
    const int rung = std::clamp(mcs, 0, kProbeMcsCeiling);

    SupervisedSpec tx{};
    tx.name = "probe-tx";
    tx.argv = {
        "/usr/bin/wfb_tx", "-K", key,
        "-M", std::to_string(rung),
        "-B", std::to_string(modulationWidth(c.link.width)),
        "-S", c.link.stbc ? "1" : "0",
        "-L", c.link.ldpc ? "1" : "0",
        "-k", "1", "-n", "1",
        "-C", std::to_string(kProbeControlPort),
        "-i", std::to_string(c.link.linkId),
        "-p", std::to_string(kProbeRadioPort),
        "-u", std::to_string(kProbeFeedPort),
        iface,
    };
    tx.restart = RestartPolicy::Always;
    out.push_back(std::move(tx));

    SupervisedSpec fd{};
    fd.name = "probe-feed";
    fd.argv = {feederPath, std::to_string(kProbeFeedPort),
               std::to_string(kProbePps), std::to_string(kProbePacketBytes)};
    fd.restart = RestartPolicy::Always;
    fd.startAfter = {"probe-tx"};
    out.push_back(std::move(fd));
    return out;
}

} // namespace fpvd
```

> Confirm `wfb_tx` accepts `-C <port>` for the TX control socket. The video tx is launched with `-C 8000` (`src/translate/wfb.cpp:46`), so the flag is supported; the probe just uses a distinct port.

- [ ] **Step 4: Run to verify it passes** — `cmake --build build -j && ./build/fpvd_tests -tc="probe specs*"` → PASS (2 cases).

- [ ] **Step 5: Commit (with A1)**
```bash
git add drone/src/probe/probe_constants.hpp drone/src/probe/probe_specs.hpp drone/src/probe/probe_specs.cpp drone/src/config/schema.hpp drone/tests/unit/test_probe_specs.cpp
git commit -m "feat(drone/probe): single fixed FEC-off probe wfb_tx with -C control port"
```

---

## Task A3: Remove `probeChanged` from the config diff

**Files:** Modify `drone/src/config/diff.hpp`, `drone/src/config/diff.cpp`

- [ ] **Step 1: Remove the field** — in `drone/src/config/diff.hpp`, delete `bool probeChanged{false};` from `struct SubsystemDiff`.

- [ ] **Step 2: Remove the comparison** — in `drone/src/config/diff.cpp`, delete:
```cpp
    // probe
    if (ja["probe"] != jb["probe"]) d.probeChanged = true;
```

- [ ] **Step 3: Build** — `cmake --build build -j 2>&1 | head -20`
Expected: FAIL — `daemon.cpp` still references `subs.probeChanged` (fixed in A4). Proceed to A4 (commit A3+A4 together).

---

## Task A4: Tie probe lifecycle to `dynamicLink.enabled` (seed + transition add/remove); drop probe from `needsRebuild`

**Files:** Modify `drone/src/daemon.cpp`

The probe processes are orchestrator-managed: **seeded when `dynamicLink.enabled`** (at boot/rebuild), and **added/removed on the live dynamicLink on↔off transition** (targeted ops — no video bounce). The initial probe MCS = `min(dynamicLink.safe.mcs + 1, kProbeMcsCeiling)`.

- [ ] **Step 1: Add an integration test** — append to `drone/tests/integration/test_daemon.cpp` (and **delete** the obsolete `TEST_CASE("apply: probe change triggers orchestrator rebuild ...")` at lines ~732-753, and the two probe tests at lines ~672-730 which patch `probe.enabled`/`mcsList` — those config knobs no longer exist):

```cpp
TEST_CASE("daemon: probe stream seeded only when dynamicLink is enabled") {
    auto tmp = fs::temp_directory_path() / "fpvd-probe-dl-seed";
    auto paths = makeRoutingPaths(tmp, 46850);
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // dynamicLink disabled by default -> no probe specs.
    CHECK(d.orchestrator().get("probe-tx") == nullptr);
    CHECK(d.orchestrator().get("probe-feed") == nullptr);

    // Enabling dynamicLink adds the probe pair WITHOUT a full rebuild: the video
    // tx keeps its identity (no stopAll/startAll).
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);
    CHECK(d.orchestrator().get("probe-tx")   != nullptr);
    CHECK(d.orchestrator().get("probe-feed") != nullptr);

    // Disabling removes them again.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":false}})")).ok);
    REQUIRE(d.apply(/*reallyRestart=*/true).ok);
    CHECK(d.orchestrator().get("probe-tx") == nullptr);

    fs::remove_all(tmp);
}
```

- [ ] **Step 2: Run to verify it fails** — `cmake --build build -j && ./build/fpvd_tests -tc="*probe*"`
Expected: FAIL (probe specs not seeded on dynamicLink enable; also still references removed `probe` config).

- [ ] **Step 3: Implement.**

(a) In `seedOrchestrator()` (`daemon.cpp:84-140`), replace the probe-seeding block at the end:
```cpp
    // Observe-only probe link: extra FEC-off wfb_tx + feeder per probe MCS.
    static const std::string kProbeFeeder = "/usr/libexec/fpvd/probe-feeder";
    for (auto& s : buildProbeSpecs(effective_, iface, key, kProbeFeeder))
        orch_.add(std::move(s));
```
with (gate on dynamicLink; single stream at the safe rung+1):
```cpp
    // Observe-only probe link: ONE FEC-off wfb_tx tracking current+1, owned by
    // the dynamic-link lifecycle. Seeded only when dynamicLink is enabled; the
    // controller retunes it live on each {mcs} (see DynamicLinkController).
    if (effective_.dynamicLink.enabled) {
        static const std::string kProbeFeeder = "/usr/libexec/fpvd/probe-feeder";
        const int probeMcs = std::min(effective_.dynamicLink.safe.mcs + 1,
                                      kProbeMcsCeiling);
        for (auto& s : buildProbeSpecs(effective_, iface, key, kProbeFeeder, probeMcs))
            orch_.add(std::move(s));
    }
```
Add `#include "probe/probe_constants.hpp"` and `#include <algorithm>` to `daemon.cpp` if not present.

(b) In `Daemon::apply()` (`daemon.cpp:276`), drop `subs.probeChanged` from `needsRebuild`:
```cpp
    const bool needsRebuild = subs.telemetry ||
        !subs.servicesAffected.empty() || link.fullRestart;
```

(c) In the **hot path** of `apply()` add probe add/remove on the dynamicLink transition. Locate the existing transition block (`daemon.cpp:378-388`):
```cpp
        if (!enabledOld && enabledNew)
            startController();
        else if (enabledOld && !enabledNew) {
            dl_.stop();
            restateStaticLink();   // revert radio + encoder to the static config
        }
        else if (enabledOld && enabledNew && (subs.dynamicLink || link.videoRadiotap))
            dl_.setConfig(dynlink::buildDlSnapshot(effective_, radio_.iface));
```
Replace with (add a helper-driven probe add/remove alongside the controller transition):
```cpp
        if (!enabledOld && enabledNew) {
            addProbeStream();      // targeted orch_.add + start (no video bounce)
            startController();
        }
        else if (enabledOld && !enabledNew) {
            dl_.stop();
            removeProbeStream();   // targeted orch_.remove (no video bounce)
            restateStaticLink();   // revert radio + encoder to the static config
        }
        else if (enabledOld && enabledNew && (subs.dynamicLink || link.videoRadiotap))
            dl_.setConfig(dynlink::buildDlSnapshot(effective_, radio_.iface));
```

(d) Add the two private helpers to `daemon.cpp` (and declare them in `daemon.hpp` near `seedOrchestrator()`):
```cpp
void Daemon::addProbeStream() {
    const std::string iface = radio_.iface.empty() ? "wlan0" : radio_.iface;
    static const std::string kProbeFeeder = "/usr/libexec/fpvd/probe-feeder";
    const int probeMcs = std::min(effective_.dynamicLink.safe.mcs + 1,
                                  kProbeMcsCeiling);
    for (auto& s : buildProbeSpecs(effective_, iface, "/etc/drone.key",
                                   kProbeFeeder, probeMcs)) {
        const std::string name = s.name;
        orch_.add(std::move(s));
        orch_.restart(name);   // add() registers; restart() starts a not-running spec
    }
}

void Daemon::removeProbeStream() {
    orch_.remove("probe-feed");   // remove feeder first (it startAfter the tx)
    orch_.remove("probe-tx");
}
```
> **Verify the orchestrator start semantics:** confirm in `src/supervise/orchestrator.cpp` whether `add()` auto-starts. If it does, drop the `orch_.restart(name)` line. If `add()` only registers (likely), `restart(name)` = shutdown(no-op) + start, which starts it. `remove()` "shuts down if running" per its header doc. These are targeted ops — they never touch `wfb_video_tx`/`waybeam`.

- [ ] **Step 4: Run to verify it passes** — `cmake --build build -j && ./build/fpvd_tests -tc="*probe*"` → PASS; then full `./build/fpvd_tests` → no regressions (watch the dynamicLink + apply tests).

- [ ] **Step 5: Commit (A3 + A4)**
```bash
git add drone/src/config/diff.hpp drone/src/config/diff.cpp drone/src/daemon.cpp drone/src/daemon.hpp drone/tests/integration/test_daemon.cpp
git commit -m "feat(drone/probe): own probe lifecycle via dynamicLink; drop probeChanged rebuild"
```

---

## Task A5: Live-retune the probe to `min(mcs+1, 7)` on each `{mcs}`

**Files:** Modify `drone/src/dynlink/runtime_config.{hpp,cpp}`, `drone/src/dynlink/controller.{hpp,cpp}`

The controller already emits `CMD_SET_RADIO` to the video tx (`wfb_`) in `dispatchTxApply`. Add a second `WfbControlClient` for the probe and retune it to `min(d.mcs+1, ceiling)` whenever the video MCS changes.

- [ ] **Step 1: Carry the probe control port + ceiling into the runtime config.** In `drone/src/dynlink/runtime_config.hpp`, add fields near `wfbCtlAddr`/`wfbCtlPort` (line ~58):
```cpp
    uint16_t probeCtlPort{0};   // probe wfb_tx -C port; 0 disables probe retune
    int      probeMcsCeiling{7};
```
In `drone/src/dynlink/runtime_config.cpp` `buildDlSnapshot(...)`, set them from the probe constants:
```cpp
    cfg.probeCtlPort = static_cast<uint16_t>(kProbeControlPort);
    cfg.probeMcsCeiling = kProbeMcsCeiling;
```
Add `#include "probe/probe_constants.hpp"` to `runtime_config.cpp`.

- [ ] **Step 2: Add the probe control client to the controller.** In `drone/src/dynlink/controller.hpp`, next to `wfb_` (line ~59), declare:
```cpp
    std::unique_ptr<WfbControlClient> probeWfb_;   // probe tx retune (nullptr if disabled)
    int lastProbeMcs_{-1};                          // last rung pushed to the probe
```
In `controller.cpp` `start()` (where `wfb_` is constructed, ~line 123), construct `probeWfb_` when a port is configured:
```cpp
    if (cfg.probeCtlPort != 0)
        probeWfb_ = std::make_unique<WfbControlClient>("127.0.0.1", cfg.probeCtlPort);
```

- [ ] **Step 3: Write the test.** The controller is hard to unit-test in isolation (it binds sockets + threads). Add a focused unit test of the clamp helper instead. In `controller.hpp`, expose a tiny static helper:
```cpp
    static int probeRungFor(int mcs, int ceiling) {
        return mcs + 1 < ceiling ? mcs + 1 : ceiling;
    }
```
Create `drone/tests/unit/test_probe_rung.cpp`:
```cpp
#include "doctest.h"
#include "dynlink/controller.hpp"
using fpvd::dynlink::DynamicLinkController;

TEST_CASE("probe rung tracks current+1, clamped to ceiling") {
    CHECK(DynamicLinkController::probeRungFor(2, 7) == 3);
    CHECK(DynamicLinkController::probeRungFor(5, 7) == 6);
    CHECK(DynamicLinkController::probeRungFor(6, 7) == 7);
    CHECK(DynamicLinkController::probeRungFor(7, 7) == 7);   // ceiling
    CHECK(DynamicLinkController::probeRungFor(9, 7) == 7);   // above ceiling
}
```
Add `tests/unit/test_probe_rung.cpp` to `target_sources(fpvd_tests ...)` in `drone/CMakeLists.txt`.

- [ ] **Step 4: Run to verify it fails** — `cmake --build build -j && ./build/fpvd_tests -tc="probe rung*"`
Expected: FAIL (helper not defined) → then PASS once `probeRungFor` is added.

- [ ] **Step 5: Retune in `dispatchTxApply`.** In `controller.cpp` `dispatchTxApply` (lines 177-202), inside the `if (first || lastTx_.mcs != d.mcs || lastTx_.bandwidth != d.bandwidth)` block, after the video `wfb_->setRadio(...)` call, add the probe retune:
```cpp
        // Retune the observe-only probe to current+1 (clamped), mirroring the
        // video PHY flags. Best-effort: a soft failure (probe not yet up) is
        // retried on the next decision. The probe rides its own radio_port, so
        // this never touches the video stream.
        if (probeWfb_) {
            int rung = probeRungFor(d.mcs, cfg.probeMcsCeiling);
            if (rung != lastProbeMcs_) {
                probeWfb_->setRadio(static_cast<uint8_t>(cfg.stbc ? 1 : 0),
                                    cfg.ldpc, /*shortGi=*/false,
                                    /*bandwidth=*/d.bandwidth,
                                    /*mcs=*/static_cast<uint8_t>(rung),
                                    /*vhtMode=*/false, /*vhtNss=*/1);
                lastProbeMcs_ = rung;
            }
        }
```
Also retune in `dispatchTxSafe` (lines 206-220) so a watchdog safe-recovery moves the probe down with the video (set the probe to `min(cfg.safe.mcs+1, ceiling)`):
```cpp
    if (probeWfb_) {
        int rung = probeRungFor(cfg.safe.mcs, cfg.probeMcsCeiling);
        probeWfb_->setRadio(static_cast<uint8_t>(cfg.stbc ? 1 : 0), cfg.ldpc, false,
                            cfg.safe.bandwidth, static_cast<uint8_t>(rung), false, 1);
        lastProbeMcs_ = rung;
    }
```

- [ ] **Step 6: Run to verify** — `cmake --build build -j && ./build/fpvd_tests` → PASS (the new clamp test + no regressions). Add `#include <memory>` to `controller.hpp` if needed for `unique_ptr`.

- [ ] **Step 7: Commit**
```bash
git add drone/src/dynlink/runtime_config.hpp drone/src/dynlink/runtime_config.cpp drone/src/dynlink/controller.hpp drone/src/dynlink/controller.cpp drone/tests/unit/test_probe_rung.cpp drone/CMakeLists.txt
git commit -m "feat(drone/probe): live-retune probe wfb_tx to current+1 on each decision"
```

---

## Task A6: Drone status reflects the dynlink-tied probe

**Files:** Modify `drone/src/status.cpp`; Test `drone/tests/integration/test_daemon.cpp`

The old status read `probe.enabled` / `probe.mcsList` from config (now removed). Replace with a summary derived from dynamicLink + the orchestrator.

- [ ] **Step 1: Find the probe block in `buildStatus`** — `grep -n "probe" drone/src/status.cpp`. It currently emits `{"enabled": effective_.probe.enabled, "mcsList": effective_.probe.mcsList}` (or similar). 

- [ ] **Step 2: Write/replace the status test** — in `test_daemon.cpp`, replace the old `TEST_CASE("status: includes probe summary")` with:
```cpp
TEST_CASE("status: probe summary reflects dynamicLink + running tx") {
    auto tmp = fs::temp_directory_path() / "fpvd-probe-status";
    auto paths = makeRoutingPaths(tmp, 46851);
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    auto j0 = fpvd::buildStatus(d);
    REQUIRE(j0.contains("probe"));
    CHECK(j0["probe"]["enabled"] == false);   // dynamicLink off

    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})")).ok);
    REQUIRE(d.apply(/*reallyRestart=*/true).ok);
    auto j1 = fpvd::buildStatus(d);
    CHECK(j1["probe"]["enabled"] == true);
    CHECK(j1["probe"]["running"] == (d.orchestrator().get("probe-tx") != nullptr));

    fs::remove_all(tmp);
}
```

- [ ] **Step 3: Implement.** In `status.cpp`, replace the probe block with:
```cpp
    j["probe"] = {
        {"enabled", d.config().dynamicLink.enabled},
        {"running", d.orchestrator().get("probe-tx") != nullptr},
    };
```
> Adjust to the actual accessors `buildStatus` already uses (it has a `Daemon&`; use the same `effective_`/`config()`/`orchestrator()` accessor the surrounding code uses — match the existing style).

- [ ] **Step 4: Run** — `cmake --build build -j && ./build/fpvd_tests -tc="*probe*status*"` → PASS; full `./build/fpvd_tests` → green.

- [ ] **Step 5: Commit**
```bash
git add drone/src/status.cpp drone/tests/integration/test_daemon.cpp
git commit -m "feat(drone/probe): status probe summary from dynamicLink + orchestrator"
```

---

# Part B — GS (Python)

## Task B1: Remove probe from the GS schema

**Files:** Modify `gs/fpvdgs/schema.py`; Delete `gs/tests/unit/test_probe_schema.py`

- [ ] **Step 1: Delete the obsolete schema test** — `git rm gs/tests/unit/test_probe_schema.py` (it validates `probe.basePort/maxStreams/rxL`, all removed).

- [ ] **Step 2: Edit `schema.py`.** Remove `"probe"` from `CONFIG_TOP_KEYS` (line 6):
```python
CONFIG_TOP_KEYS = {"wfb", "drone", "dynamicLink", "pixelpilot"}   # link is excluded on purpose
```
Remove the probe call in `validate_effective` (lines 54-56):
```python
    probe = cfg.get("probe")
    if probe is not None:
        _validate_probe(probe)
```
Delete the entire `_validate_probe` function (lines 126-139).

> After this, a `PATCH /config {"probe": ...}` is rejected (`unknown config keys: ['probe']`) — intended; probe is no longer operator-configurable. A deployed `config.json` overlay carrying a legacy `probe` block is handled in B6.

- [ ] **Step 3: Run** — `cd /home/gilankpam/Projects/drone/fpvd/gs && python -m pytest tests/unit/test_schema.py -q` → PASS (no probe tests remain). Full suite will fail until B2-B5 (the supervisor/api/defaults still reference probe) — that's expected; proceed.

- [ ] **Step 4: Commit (with B2)** — defer; commit B1+B2 together.

---

## Task B2: Remove the `probe` block from defaults

**Files:** Modify `gs/etc/defaults.json`

- [ ] **Step 1: Edit `defaults.json`** — delete the last top-level entry (line 59) and the trailing comma on the `pixelpilot` block's close so the JSON stays valid:

Change the end from:
```json
    "extraArgs": []
  },
  "probe": { "enabled": false, "basePort": 50, "maxStreams": 4, "rxL": 50 }
}
```
to:
```json
    "extraArgs": []
  }
}
```

- [ ] **Step 2: Verify JSON** — `python -c "import json; json.load(open('etc/defaults.json'))"` → no error.

- [ ] **Step 3: Commit (B1 + B2)**
```bash
git add gs/fpvdgs/schema.py gs/etc/defaults.json gs/tests/unit/test_probe_schema.py
git commit -m "feat(gs/probe): remove probe config block + schema (lifecycle moves to dynamicLink)"
```

---

## Task B3: `make_probe_snapshot` → single fixed port from effective

**Files:** Modify `gs/fpvdgs/probe/config_build.py`

- [ ] **Step 1: Rewrite the snapshot builder.** Replace `gs/fpvdgs/probe/config_build.py`:
```python
"""Build the self-contained snapshot the ProbeController consumes. One fixed
probe wfb_rx on kProbePort (matching the drone's probe radio_port). No probe
config — the probe lifecycle follows dynamicLink."""
from __future__ import annotations

from ..runner_supervisor import resolve_wlans

GS_KEY = "/etc/gs.key"
PROBE_PORT = 50    # wfb radio_port; MUST match the drone's kProbeRadioPort
PROBE_RX_L = 50    # wfb_rx -l log interval (ms)


def make_probe_snapshot(effective: dict) -> dict:
    """The snapshot for the single probe wfb_rx: fixed port + key/linkId/wlans."""
    return {
        "port": PROBE_PORT,
        "rxL": PROBE_RX_L,
        "key": GS_KEY,
        "linkId": effective.get("link", {}).get("linkId"),
        "wlans": resolve_wlans(effective),
    }
```

- [ ] **Step 2: No standalone test** (covered by B4 controller tests + B5 e2e). Proceed.

- [ ] **Step 3: Commit (with B4)** — defer; commit B3+B4 together.

---

## Task B4: `ProbeController` → one `wfb_rx` on the fixed port

**Files:** Modify `gs/fpvdgs/probe/controller.py`; Test `gs/tests/unit/test_probe_controller.py`

Drop the `enabled`/`basePort`/`maxStreams` loop: when started, spawn exactly one `wfb_rx` on `snap["port"]`. Lifecycle (start/stop) is driven by the supervisor on `dynamicLink.enabled`. The threaded-asyncio scaffold, `set_config`, and the RX_ANT-keyed aggregator stay as-is.

- [ ] **Step 1: Rewrite the tests** — replace `gs/tests/unit/test_probe_controller.py`:
```python
import asyncio
from fpvdgs.probe.controller import ProbeController

def _snap(**over):
    s = {"port": 50, "rxL": 50, "key": "/etc/gs.key",
         "linkId": 7669206, "wlans": ["wlanA", "wlanB"]}
    s.update(over)
    return s

class _FakeProc:
    """Emits scripted stdout lines then idles until killed."""
    def __init__(self, lines):
        self._lines = list(lines)
        self.stdout = self
        self.killed = False
    async def readline(self):
        if self._lines:
            return (self._lines.pop(0) + "\n").encode()
        await asyncio.sleep(3600)
    def kill(self): self.killed = True
    async def wait(self): return 0

def _wait_until(pred, timeout=2.0):
    import time
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if pred():
            return True
        time.sleep(0.01)
    return False

def test_builds_one_wfb_rx_on_fixed_port():
    cmds = []
    def spawn(cmd):
        cmds.append(cmd)
        return _FakeProc([])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        assert len(cmds) == 1
        cmd = cmds[0]
        assert "/usr/bin/wfb_rx" in cmd[0]
        assert "-p" in cmd and "50" in cmd
        assert "-K" in cmd and "/etc/gs.key" in cmd
        assert "-i" in cmd and "7669206" in cmd
        assert "wlanA" in cmd and "wlanB" in cmd
        assert c.status()["running"] is True and c.status()["streams"] == 1
    finally:
        c.stop()
    assert c.status()["running"] is False

def test_measures_per_mcs_from_stdout():
    def spawn(cmd):
        return _FakeProc(["1\tRX_ANT\t5805:5:20\t0\t1:-80:-80:-80:8:8:8",
                          "1\tPKT\t10:0:0:0:1:1:0:9:0:1:0:0:0:0"])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        assert _wait_until(lambda: "5" in c.status()["mcs"])
        mcs = c.status()["mcs"]
        assert abs(mcs["5"]["per"] - 0.9) < 1e-9 and mcs["5"]["snr"] == 8
    finally:
        c.stop()

def test_retune_followed_via_rx_ant_key():
    # The same fixed port carries a different MCS after the drone retunes; the
    # aggregator keys by RX_ANT mcs, so a new slot appears.
    def spawn(cmd):
        return _FakeProc(["1\tRX_ANT\t5805:3:20\t0\t9:-55:-55:-55:28:28:28",
                          "1\tPKT\t9:0:0:0:9:9:0:0:0:9:0:0:0:0",
                          "2\tRX_ANT\t5805:4:20\t0\t9:-56:-56:-56:27:27:27",
                          "2\tPKT\t9:0:0:0:9:9:0:0:0:9:0:0:0:0"])
    c = ProbeController(_snap(), spawn=spawn)
    c.start()
    try:
        assert _wait_until(lambda: {"3", "4"} <= set(c.status()["mcs"]))
        snap = c.status()["mcs"]
        assert snap["3"]["per"] == 0.0 and snap["4"]["per"] == 0.0
    finally:
        c.stop()
```

- [ ] **Step 2: Run to verify it fails** — `python -m pytest tests/unit/test_probe_controller.py -q` → FAIL (snapshot keys / multi-stream behavior changed).

- [ ] **Step 3: Implement.** In `gs/fpvdgs/probe/controller.py`, replace `_build_cmd` to use the single `port`:
```python
    def _build_cmd(self, port: int, sink: int) -> list[str]:
        # wfb_rx (rx.cpp getopt "K:fa:c:u:U:p:l:i:e:R:s:") — -l is the log_interval.
        snap = self._snap
        return [WFB_RX, "-K", str(snap["key"]), "-i", str(snap["linkId"]),
                "-p", str(port), "-c", "127.0.0.1", "-u", str(sink),
                "-l", str(snap.get("rxL", 50)), *list(snap["wlans"])]
```
(unchanged signature). Then replace `_run` to spawn exactly one stream:
```python
    async def _run(self):
        self._stop_event = asyncio.Event()
        snap = self._snap
        procs, tasks = [], []
        try:
            cmd = self._build_cmd(int(snap["port"]), 7000)   # 7000 = throwaway sink
            res = self._spawn(cmd)
            proc = await res if asyncio.iscoroutine(res) else res
            procs.append(proc)
            tasks.append(asyncio.ensure_future(self._read_stream(proc)))
            self._set(running=True, streams=len(procs))
            self._started.set()
            await self._stop_event.wait()
        finally:
            # Runs on normal stop AND on a spawn failure, so the wfb_rx is never
            # orphaned holding the radio_port.
            self._set(running=False, streams=0)
            self._started.set()
            for p in procs:
                try:
                    p.kill()
                except Exception:
                    pass
            for t in tasks:
                t.cancel()
            for p in procs:
                try:
                    await p.wait()
                except Exception:
                    pass
```
Update `status()` to drop the `enabled` key (the supervisor owns "enabled" now):
```python
    def status(self):
        with self._lock:
            st = dict(self._status)
            st["mcs"] = {str(m): v for m, v in self._agg.snapshot().items()}
        return st
```

- [ ] **Step 4: Run to verify it passes** — `python -m pytest tests/unit/test_probe_controller.py -q` → PASS (3 cases).

- [ ] **Step 5: Commit (B3 + B4)**
```bash
git add gs/fpvdgs/probe/config_build.py gs/fpvdgs/probe/controller.py gs/tests/unit/test_probe_controller.py
git commit -m "feat(gs/probe): single fixed-port wfb_rx; snapshot derived from effective"
```

---

## Task B5: Tie GS probe lifecycle + status to `dynamicLink.enabled`

**Files:** Modify `gs/fpvdgs/supervisor.py`; Test `gs/tests/integration/test_supervisor_e2e.py`

- [ ] **Step 1: Update the e2e test.** Replace `test_status_has_probe_block` in `gs/tests/integration/test_supervisor_e2e.py` (it enabled `probe.enabled`; now drive it via `dynamicLink.enabled`). Keep the module's existing `_FakeProc`/harness:
```python
def test_status_probe_tied_to_dynamiclink(tmp_path, monkeypatch):
    import json
    from fpvdgs import supervisor
    monkeypatch.setattr(supervisor, "resolve_wlans", lambda cfg: ["wlan0"])
    monkeypatch.setattr("fpvdgs.probe.config_build.resolve_wlans",
                        lambda cfg: ["wlan0"])

    class _StubDl:
        def __init__(self, *a, **k): self.started = False
        def start(self): self.started = True
        def stop(self): self.started = False
        def set_config(self, snap): pass
        def status(self): return {"running": self.started, "hello": "none"}
    monkeypatch.setattr(supervisor, "DynamicLinkController", _StubDl)

    spawned = []
    def fake_spawn(cmd):
        spawned.append(cmd)
        class _P:
            stdout = type("S", (), {"readline": staticmethod(
                lambda: __import__("asyncio").sleep(3600))})()
            def kill(self): pass
            async def wait(self): return 0
        return _P()

    defaults = tmp_path / "defaults.json"
    defaults.write_text(json.dumps({
        "link": {"channel": 132, "width": 40, "region": "US", "linkId": 7669206},
        "wfb": {"profile": "gs", "raw": {}},
        "drone": {"endpoint": "http://127.0.0.1:1"},
        "pixelpilot": {"enabled": False},
        "dynamicLink": {"enabled": True, "maxMcs": 5, "bandwidth": 20,
                        "txpower": {"min": 18, "max": 28},
                        "radioProfile": "m8812eu2", "dronePort": 9999,
                        "tuning": {}}}))
    cfg_out = tmp_path / "wfb.cfg"
    app = supervisor.build_app(str(defaults), str(tmp_path / "config.json"),
                               str(cfg_out), "127.0.0.1", 0,
                               runner_cmd=["true"], probe_spawn=fake_spawn)
    app.start()
    try:
        code, body = app.api.handle("GET", "/status", {}, b"")
        assert code == 200
        assert body["probe"]["enabled"] is True
        assert len(spawned) == 1            # one wfb_rx, started with dynamicLink
    finally:
        app.shutdown()
```

- [ ] **Step 2: Run to verify it fails** — `python -m pytest tests/integration/test_supervisor_e2e.py -q` → FAIL (probe still gated on `probe.enabled`).

- [ ] **Step 3: Implement.** In `gs/fpvdgs/supervisor.py`:

In `App.start()` (lines 39-41), gate the probe on `dynamicLink.enabled`:
```python
        if (self.probe is not None
                and self.store.effective().get("dynamicLink", {}).get("enabled")):
            self.probe.start()
```
In `_probe_status()` (lines 116-119), gate on `dynamicLink.enabled` and add the `enabled` key the drone-side status no longer comes from probe config:
```python
    def _probe_status():
        if not store.effective().get("dynamicLink", {}).get("enabled"):
            return {"enabled": False, "running": False}
        return {"enabled": True, **probe_ctrl.status()}
```

> `App.shutdown()` already calls `self.probe.stop()` unconditionally (null-safe) — no change. `make_probe_snapshot(effective)` is already called in `build_app` to construct `probe_ctrl` — no change (B3 made it config-free).

- [ ] **Step 4: Run to verify it passes** — `python -m pytest tests/integration/test_supervisor_e2e.py -q` → PASS; full `python -m pytest tests/ -q` (will still fail on the api probe routing tests until B6).

- [ ] **Step 5: Commit (with B6)** — defer; commit B5+B6 together.

---

## Task B6: Remove probe `/apply` routing; tie probe to the dynamicLink transition

**Files:** Modify `gs/fpvdgs/api.py`; Test `gs/tests/unit/test_api.py`

With no probe config, there is nothing to route on a probe change. The probe must instead start/stop **with the dynamicLink transition** in `_route_dynamic_link`.

- [ ] **Step 1: Update the tests.** In `gs/tests/unit/test_api.py`, **delete** `_api_with_probe` and the 4 probe routing tests (`test_enable_probe_starts_controller_without_bouncing_runner`, `test_disable_probe_stops_controller`, `test_probe_tuning_change_while_enabled_calls_set_config`, `test_wfb_change_does_not_touch_probe`). Then extend the dynlink helper so a fake probe is passed, and add a test that the **dynamicLink** transition drives the probe. Add to `test_api.py`:
```python
class _FakeProbe:
    def __init__(self): self.started = False; self.cfgs = []
    def start(self): self.started = True
    def stop(self): self.started = False
    def set_config(self, snap): self.cfgs.append(snap)
    def status(self): return {"running": self.started, "streams": 1, "mcs": {}}

def _api_with_dl_and_probe(tmp_path):
    # mirror _api_with_dynlink, but also pass a fake probe and stub resolve_wlans
    import json
    from fpvdgs import api as api_mod, schema, render as render_mod
    from fpvdgs.config import ConfigStore
    defaults = tmp_path / "defaults.json"
    defaults.write_text(json.dumps({
        "link": {"channel": 132, "width": 40, "region": "US", "linkId": 7669206,
                 "wlans": ["wlan0"]},
        "wfb": {"profile": "gs", "raw": {}},
        "drone": {"endpoint": "http://127.0.0.1:1"},
        "dynamicLink": {"enabled": False, "maxMcs": 5, "bandwidth": 20,
                        "txpower": {"min": 18, "max": 28},
                        "radioProfile": "m8812eu2", "dronePort": 9999, "tuning": {}}}))
    store = ConfigStore.load(str(defaults), str(tmp_path / "config.json"))
    ctrl = _FakeController()          # existing fake dynlink controller in this file
    probe = _FakeProbe()
    runner = _FakeRunner()
    cfg_out = tmp_path / "wfb.cfg"
    api = api_mod.Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
                      drone=None, link=None, status_fn=lambda: {}, cfg_out=str(cfg_out),
                      dynlink=ctrl, probe=probe)
    return api, store, ctrl, probe, runner

def test_enable_dynamiclink_starts_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert ctrl.started is True and probe.started is True
    assert runner.restarts == 0           # no video bounce

def test_disable_dynamiclink_stops_probe(tmp_path):
    api, store, ctrl, probe, runner = _api_with_dl_and_probe(tmp_path)
    store.patch({"dynamicLink": {"enabled": True}}); api.handle("POST", "/apply", {}, b"")
    store.patch({"dynamicLink": {"enabled": False}})
    code, _ = api.handle("POST", "/apply", {}, b"")
    assert code == 200
    assert probe.started is False and runner.restarts == 0
```
> Match the exact harness style of the existing `_api_with_dynlink` in this file (imports, `_FakeController`, `_FakeRunner`). If `resolve_wlans` is reached via `make_probe_snapshot`, monkeypatch `fpvdgs.probe.config_build.resolve_wlans` in the helper (the dynlink config has `link.wlans` set, so it returns without hardware).

- [ ] **Step 2: Run to verify it fails** — `python -m pytest tests/unit/test_api.py -q` → FAIL (probe not driven by dynamicLink transition; old probe tests removed).

- [ ] **Step 3: Implement.** In `gs/fpvdgs/api.py`:

Add `make_probe_snapshot` import is already present (line 11) — keep it. In `_apply_gs`, **remove** the `_route_probe` call (lines 105-106):
```python
        self._route_probe(effective.get("probe", {}),
                          pending.get("probe", {}), pending)
```
**Delete** the `_route_probe` method (lines 124-136). Remove `"probe"` from the `wfb_changed` exclusion (lines 90-91):
```python
        # Anything outside dynamicLink/pixelpilot (link already equal) needs the
        # runner. (probe carries no config now; its lifecycle rides dynamicLink.)
        wfb_changed = (self._without(pending, "dynamicLink", "pixelpilot")
                       != self._without(effective, "dynamicLink", "pixelpilot"))
```
In `_route_dynamic_link`, drive the probe alongside the controller:
```python
    def _route_dynamic_link(self, dl_old, dl_new, pending):
        """Start/stop/reconfigure the in-process controller AND the observe-only
        probe (they share a lifecycle). Never bounces the wfb runner."""
        if self.dynlink is None:
            return
        was, now = bool(dl_old.get("enabled")), bool(dl_new.get("enabled"))
        if not was and now:
            self.dynlink.set_config(make_dl_snapshot(pending))
            self.dynlink.start()
            if self.probe is not None:
                self.probe.set_config(make_probe_snapshot(pending))
                self.probe.start()
        elif was and not now:
            self.dynlink.stop()
            if self.probe is not None:
                self.probe.stop()
        elif was and now and dl_old != dl_new:
            self.dynlink.set_config(make_dl_snapshot(pending))
            if self.probe is not None:
                self.probe.set_config(make_probe_snapshot(pending))
```
Keep the `probe=None` param on `Api.__init__` (the controller is still injected for the lifecycle; just no longer routed on a `probe` config change).

- [ ] **Step 4: Run to verify it passes** — `python -m pytest tests/unit/test_api.py -q` → PASS; then full `python -m pytest tests/ -q` → green.

- [ ] **Step 5: Commit (B5 + B6)**
```bash
git add gs/fpvdgs/supervisor.py gs/fpvdgs/api.py gs/tests/integration/test_supervisor_e2e.py gs/tests/unit/test_api.py
git commit -m "feat(gs/probe): probe lifecycle follows dynamicLink; drop probe /apply routing"
```

---

## Task B7: Verify legacy-config tolerance + deploy script

**Files:** Verify `gs/fpvdgs/config.py`; Modify `deploy/gs/deploy.sh` (only if it pushed probe defaults)

- [ ] **Step 1: Confirm the loader tolerates a legacy `probe` key.** A deployed `/etc/fpvd/config.json` overlay may still carry `"probe": {...}`. Inspect `gs/fpvdgs/config.py` `ConfigStore.load`/`patch`/`effective`: confirm an unknown top-level overlay key is carried/ignored, NOT rejected (validation only runs `validate_config_patch` on PATCH bodies and `validate_effective` on apply — neither runs on the persisted overlay at boot). Add a test to `gs/tests/unit/test_config.py`:
```python
def test_legacy_probe_key_in_overlay_does_not_break_load(tmp_path):
    import json
    from fpvdgs.config import ConfigStore
    (tmp_path / "defaults.json").write_text(json.dumps({"link": {"channel": 1}}))
    (tmp_path / "config.json").write_text(json.dumps({"probe": {"enabled": True}}))
    store = ConfigStore.load(str(tmp_path / "defaults.json"),
                             str(tmp_path / "config.json"))
    eff = store.effective()           # must not raise
    assert eff["link"]["channel"] == 1
```
Run: `python -m pytest tests/unit/test_config.py -q`. If it raises, the loader needs to tolerate/strip unknown keys — fix minimally and re-run. If it passes, no loader change needed.

- [ ] **Step 2: Drone defaults** — confirm the drone `tests/fixtures/defaults.json` and any deployed drone defaults no longer require a `probe` block (the C++ `Config` no longer has the field; nlohmann ignores extra keys, so a stale block is harmless). No change needed unless a test asserts on `probe`.

- [ ] **Step 3: Commit (if any change)**
```bash
git add gs/tests/unit/test_config.py
git commit -m "test(gs/probe): legacy probe overlay key tolerated on config load"
```

---

# Part C — On-hardware smoke (needs live GS 10.18.0.1 + drone)

**Files:** none (verification). Deploy both ends.

- [ ] **Step 1: Deploy** — drone: `./deploy/drone/deploy.sh` (cross-build); GS: `./deploy/gs/deploy.sh --host 10.18.0.1`. Both `[done]`, fpvd up on each.
- [ ] **Step 2: Enable dynamic-link** (which now also starts the probe both ends): `curl -X PATCH http://10.18.0.1:8080/config -d '{"dynamicLink":{"enabled":true}}' && curl -X POST http://10.18.0.1:8080/apply -d '{}'`. Confirm on the drone (`ssh root@192.168.10.152`) exactly one `probe-tx` (with `-C 8001`) + `probe-feed`, and on the GS one probe `wfb_rx` on port 50.
- [ ] **Step 3: Capture video PIDs** on the drone (`pidof waybeam`, `pidof wfb_tx`). Drive an MCS change (let the link controller move it, or force one). Confirm: (a) `/status` probe block tracks `current+1` across the change, (b) `waybeam`/`wfb_video_tx` PIDs are **unchanged** (no glitch — the regression), (c) video is undisturbed.
- [ ] **Step 4: Disable dynamic-link** — confirm the probe `wfb_tx`/`wfb_rx` are gone on both ends and video is healthy.

---

## Self-Review

**Spec coverage (`2026-06-07-probe-fixed-stream-plumbing-design.md`):**
- §2 "one probe stream, current+1" → A2 (single spec), A5 (`probeRungFor` = mcs+1). ✓
- §2 "drone-autonomous retune, no wire change" → A5 (controller derives rung from `d.mcs`, no `wire.*` change). ✓
- §2 "fixed ports both ends" → A1 (`kProbeRadioPort=50`), B3 (`PROBE_PORT=50`). ✓
- §2 "lifecycle = dynamicLink" → A4 (seed/add/remove on `dynamicLink.enabled`), B5 (`App.start`/`_probe_status`), B6 (`_route_dynamic_link`). ✓
- §2 "no probe config" → A1 (drone schema), B1 (GS schema), B2 (defaults). ✓
- §2 "ceiling = clamp to MCS 7" → A2/A5 (`std::clamp`/`probeRungFor` to `kProbeMcsCeiling`). ✓
- §2 "FEC 1/1" → A2 (`-k 1 -n 1`). ✓
- §1 "delete probeChanged→rebuild" → A3 + A4(b). ✓
- §9 "legacy config migration" → A1 note (nlohmann ignores), B7 (GS loader test). ✓
- §5 bitrate reserve → **deferred** (Phase 3; out of scope, noted in header). ✓ (intentional gap)

**Placeholder scan:** A6 Step 3 and A4 Step 3(d) note "match the existing accessor"/"verify orchestrator start semantics" — these are intentional verification steps against internal APIs the implementer must read (`status.cpp`'s `Daemon&` accessors, `orchestrator.cpp`'s `add`/`restart`), not missing code; the new code is given in full. All test bodies and new functions are complete.

**Type/name consistency:** process names `probe-tx`/`probe-feed` consistent A2↔A4↔A6↔C. `buildProbeSpecs(c, iface, key, feederPath, mcs)` consistent A2↔A4. `kProbeRadioPort=50` == GS `PROBE_PORT=50`. `probeRungFor(mcs, ceiling)` consistent A5 def↔use. Snapshot keys `{port,rxL,key,linkId,wlans}` consistent B3↔B4. `_probe_status` returns `{enabled, running}` (disabled) / `{enabled:True, running, streams, mcs}` (enabled) consistent B5↔drone A6 shape.

---

## Execution Handoff

**Plan complete and saved to `docs/superpowers/plans/2026-06-07-probe-fixed-stream-plumbing.md`. Two execution options:**

**1. Subagent-Driven (recommended)** — fresh subagent per task, two-stage review (spec then quality) between tasks.

**2. Inline Execution** — execute here with checkpoints.

**Which approach?**
