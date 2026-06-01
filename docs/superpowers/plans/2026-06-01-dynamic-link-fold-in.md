# Dynamic-Link Fold-In Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fold the drone-side `dl-applier` adaptive-link controller into the fpvd process as an in-process `DynamicLinkController` thread, with hot config reload (no restart) and I/O unified onto fpvd's existing `wfb_tx`/`iw`/encoder clients.

**Architecture:** A new `src/dynlink/` C++ module ports the C control loop (`wfbng-dynamic-link/drone/src/dl_*`) one-to-one. It runs on one dedicated thread owned by `Daemon`, reading an atomically-published `DlRuntimeConfig` snapshot and waking on an `eventfd` for reload/stop. Decisions reach hardware via `WfbControlClient` (extended with interleave-depth), a new `EncoderClient`, and a new `RadioTxpower` `iw` helper. `/etc/dynamic-link/drone.conf`, the argv translator, and MAVLink are removed.

**Tech Stack:** C++17, CMake, doctest (vendored), cpp-httplib (vendored, client+server), nlohmann/json (vendored), Linux syscalls (`timerfd`, `eventfd`, `poll`, `posix_spawnp`). Cross-target: armv7l/musl/static (ssc338q).

**Spec:** `docs/superpowers/specs/2026-06-01-dynamic-link-fold-in-design.md`
**Branch:** `feat/dynamic-link-fold-in` (already checked out)

---

## How to use this plan

Many tasks **port an existing, working C module to C++**. For those, the named C file
**is the complete reference implementation** — the task gives the exact C++ interface
(header), the source path to translate, the mechanical transforms, and the **ported
tests** (the byte/behaviour parity guarantee). "Port `dl_wire.c`'s `dl_wire_decode`
preserving the big-endian byte layout" is a concrete instruction, not a placeholder:
the bytes are pinned by the golden-vector tests you copy from the C test file.

New code (the controller, snapshot, `EncoderClient`, `RadioTxpower`, depth command,
`apply()` routing, status block, schema edit, toolchain) is shown in full.

Reference checkout of the C sources: `/home/gilankpam/Projects/drone/dynamic-link/drone/src/`
and tests at `/home/gilankpam/Projects/drone/dynamic-link/tests/drone/`.

Build/test from the fpvd repo root:
```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j && ./build/fpvd_tests
```
Run a single test case: `./build/fpvd_tests -tc="<case name>"`.

---

## File structure

**New files (fpvd):**
- `cmake/toolchain-ssc338q.cmake` — cross toolchain (Phase 0)
- `src/dynlink/wire.{hpp,cpp}` — `Decision`/`Ping`/`Pong`/`Hello`/`HelloAck` + decode/encode/crc32/peek
- `src/dynlink/dedup.{hpp,cpp}` — `Dedup`
- `src/dynlink/watchdog.{hpp,cpp}` — `Watchdog`
- `src/dynlink/apply_direction.hpp` — `applyDirection()` (header-only)
- `src/dynlink/roi_qp.{hpp,cpp}` — `computeRoiQp()`
- `src/dynlink/encoder_client.{hpp,cpp}` — `EncoderClient` (waybeam HTTP)
- `src/dynlink/radio_txpower.{hpp,cpp}` — `RadioTxpower` (`iw` via posix_spawnp)
- `src/dynlink/hello.{hpp,cpp}` — `HelloSm`
- `src/dynlink/osd.{hpp,cpp}` — `OsdWriter`
- `src/dynlink/idr_listen.{hpp,cpp}` — `IdrListener`
- `src/dynlink/runtime_config.hpp` — `DlRuntimeConfig`, `DlStatus`, `Endpoints`, `buildDlSnapshot()`
- `src/dynlink/controller.{hpp,cpp}` — `DynamicLinkController`

**Modified (fpvd):**
- `src/translate/wfb_cmd.h` (+ interleave-depth), `src/translate/wfb_control.{hpp,cpp}` (+ `setInterleaveDepth`)
- `src/config/schema.hpp`, `etc/defaults.json` (remove `mavlinkEnable`)
- `src/daemon.{hpp,cpp}` (own controller; apply routing; `seedOrchestrator` drops the `dl_applier` child; bootstrap)
- `src/status.cpp` (`dynamicLink` block)
- `CMakeLists.txt`, `shell.nix`

**Deleted (fpvd):**
- `src/translate/dynamic_link.{hpp,cpp}`
- `tests/unit/test_translate_dynamic_link.cpp`, `tests/unit/test_dl_applier_cli_assumptions.cpp`

---

# Phase 0 — Build infrastructure

### Task 1: ssc338q cross-compile toolchain + nix dep

**Files:**
- Create: `cmake/toolchain-ssc338q.cmake`
- Modify: `shell.nix`
- Modify: `CMakeLists.txt` (guard tests to host builds)

- [ ] **Step 1: Add the toolchain file**

Create `cmake/toolchain-ssc338q.cmake`:
```cmake
# Cross toolchain for OpenIPC SSC338Q (armv7l, musl, NEON-VFPv4, static).
# Mirrors wfbng-dynamic-link/drone/Makefile's `ssc338q` target.
set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR armv7l)
set(CMAKE_C_COMPILER   armv7l-unknown-linux-musleabihf-gcc)
set(CMAKE_CXX_COMPILER armv7l-unknown-linux-musleabihf-g++)
set(_ssc "-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -Os")
set(CMAKE_C_FLAGS_INIT   "${_ssc}")
set(CMAKE_CXX_FLAGS_INIT "${_ssc}")
set(CMAKE_EXE_LINKER_FLAGS_INIT "-static -static-libstdc++ -static-libgcc")
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
```

- [ ] **Step 2: Add the cross toolchain to the nix dev shell**

Edit `shell.nix` to:
```nix
{ pkgs ? import <nixpkgs> {} }:

pkgs.mkShell {
  packages = [
    pkgs.cmake
    pkgs.ninja
    pkgs.pkg-config
    pkgs.pkgsCross.armv7l-hf-multiplatform.pkgsMusl.stdenv.cc   # ssc338q gcc/g++
  ];
}
```

- [ ] **Step 3: Guard the test executable to host builds**

In `CMakeLists.txt`, wrap the `enable_testing()` / `fpvd_tests` block so it only
builds when not cross-compiling. Replace `enable_testing()` and everything through
`add_test(NAME fpvd_tests COMMAND fpvd_tests)` with:
```cmake
# Tests build host-only (doctest runs on the dev machine / qemu-arm).
if(NOT CMAKE_CROSSCOMPILING)
    enable_testing()
    add_executable(fpvd_tests tests/test_main.cpp)
    target_link_libraries(fpvd_tests PRIVATE fpvd_core)
    target_sources(fpvd_tests PRIVATE
        # ... existing list unchanged ...
    )
    add_test(NAME fpvd_tests COMMAND fpvd_tests)
endif()
```

- [ ] **Step 4: Verify host build still works**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j && ./build/fpvd_tests`
Expected: configure + build succeed; all existing tests PASS.

- [ ] **Step 5: Verify the cross build produces an armv7l binary**

Run (inside `nix-shell`):
```bash
cmake -S . -B build/ssc338q -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-ssc338q.cmake
cmake --build build/ssc338q --target fpvd -j
file build/ssc338q/fpvd
```
Expected: `build/ssc338q/fpvd: ELF 32-bit LSB executable, ARM, EABI5 ... statically linked`.
(If the cross toolchain isn't on PATH, this step is documented-manual; the host build in Step 4 must still pass.)

- [ ] **Step 6: Commit**
```bash
git add cmake/toolchain-ssc338q.cmake shell.nix CMakeLists.txt
git commit -m "build: ssc338q cross toolchain + host-only test guard"
```

---

# Phase 1 — Pure-logic module ports

All Phase 1 modules live in `src/dynlink/`, depend on nothing in fpvd, and port
one-to-one from the C sources. After creating each `.cpp`, add it to `fpvd_core`'s
`target_sources` and the test `.cpp` to `fpvd_tests`'s `target_sources` in
`CMakeLists.txt` (called out in each task's commit step).

### Task 2: `Wire` — decode/encode/crc32/peek

**Files:**
- Create: `src/dynlink/wire.hpp`, `src/dynlink/wire.cpp`
- Test: `tests/unit/test_dl_wire.cpp`
- Reference: `dynamic-link/drone/src/dl_wire.{h,c}`, tests `dynamic-link/tests/drone/test_wire.c`

- [ ] **Step 1: Write the C++ interface**

Create `src/dynlink/wire.hpp` mirroring `dl_wire.h` exactly (same constants, same
field order/types). Use `enum class` and `std::optional` for decode results:
```cpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <optional>

namespace fpvd::dynlink {

inline constexpr uint32_t kWireMagic    = 0x444C4B31u;  // "DLK1"
inline constexpr int      kWireVersion  = 2;
inline constexpr size_t   kWireOnWire   = 31;           // 27 payload + 4 CRC
inline constexpr uint32_t kPingMagic    = 0x444C5047u;
inline constexpr size_t   kPingOnWire   = 24;
inline constexpr uint32_t kPongMagic    = 0x444C504Eu;
inline constexpr size_t   kPongOnWire   = 40;
inline constexpr uint32_t kHelloMagic   = 0x444C4845u;
inline constexpr size_t   kHelloOnWire  = 32;
inline constexpr uint32_t kHelloAckMagic= 0x444C4841u;
inline constexpr size_t   kHelloAckOnWire = 32;
inline constexpr uint8_t  kHelloFlagVanillaWfbNg = 0x01u;

struct Decision {
    uint32_t magic{};  uint8_t version{}; uint8_t flags{};
    uint32_t sequence{}; uint32_t timestampMs{};
    uint8_t mcs{}; uint8_t bandwidth{}; int8_t txPowerDbm{};
    uint8_t k{}; uint8_t n{}; uint8_t depth{};
    uint16_t bitrateKbps{}; uint8_t fps{};
};
struct Ping { uint32_t magic{}; uint8_t version{}; uint8_t flags{};
              uint32_t gsSeq{}; uint64_t gsMonoUs{}; };
struct Pong { uint32_t magic{}; uint8_t version{}; uint8_t flags{};
              uint32_t gsSeq{}; uint64_t gsMonoUsEcho{};
              uint64_t droneMonoRecvUs{}; uint64_t droneMonoSendUs{}; };
struct Hello { uint32_t magic{}; uint8_t version{}; uint8_t flags{};
               uint32_t generationId{}; uint16_t mtuBytes{}; uint16_t fps{};
               uint32_t applierBuildSha{}; };
struct HelloAck { uint32_t magic{}; uint8_t version{}; uint32_t generationIdEcho{}; };

enum class DecodeResult { Ok, Short, BadMagic, BadVersion, BadCrc };
enum class PacketKind { Unknown, Decision, Ping, Pong, Hello, HelloAck };

uint32_t crc32(const uint8_t* buf, size_t len);
PacketKind peekKind(const uint8_t* buf, size_t len);

size_t encodeDecision(const Decision&, uint8_t* buf, size_t buflen);
DecodeResult decodeDecision(const uint8_t* buf, size_t len, Decision&);
size_t encodePing(const Ping&, uint8_t* buf, size_t buflen);
DecodeResult decodePing(const uint8_t* buf, size_t len, Ping&);
size_t encodePong(const Pong&, uint8_t* buf, size_t buflen);
DecodeResult decodePong(const uint8_t* buf, size_t len, Pong&);
size_t encodeHello(const Hello&, uint8_t* buf, size_t buflen);
DecodeResult decodeHello(const uint8_t* buf, size_t len, Hello&);
size_t encodeHelloAck(const HelloAck&, uint8_t* buf, size_t buflen);
DecodeResult decodeHelloAck(const uint8_t* buf, size_t len, HelloAck&);

} // namespace fpvd::dynlink
```

- [ ] **Step 2: Write the failing test (golden vectors copied from `test_wire.c`)**

Create `tests/unit/test_dl_wire.cpp`. Port every case from `dynamic-link/tests/drone/test_wire.c`,
**keeping the exact byte vectors and CRC values** — they are the GS-interop contract.
Worked example (roundtrip + a fixed-CRC check; copy the remaining `test_wire.c` cases
verbatim, translating `assert` → `CHECK`):
```cpp
#include <doctest/doctest.h>
#include "dynlink/wire.hpp"
using namespace fpvd::dynlink;

TEST_CASE("decision encode/decode roundtrip") {
    Decision d{}; d.magic = kWireMagic; d.version = kWireVersion;
    d.sequence = 42; d.timestampMs = 1000; d.mcs = 3; d.bandwidth = 20;
    d.txPowerDbm = 25; d.k = 8; d.n = 12; d.depth = 1;
    d.bitrateKbps = 6000; d.fps = 60;
    uint8_t buf[kWireOnWire];
    CHECK(encodeDecision(d, buf, sizeof(buf)) == kWireOnWire);
    Decision out{};
    CHECK(decodeDecision(buf, kWireOnWire, out) == DecodeResult::Ok);
    CHECK(out.sequence == 42); CHECK(out.mcs == 3); CHECK(out.bitrateKbps == 6000);
    CHECK(out.txPowerDbm == 25); CHECK(out.fps == 60);
}

TEST_CASE("decode rejects bad magic / version / crc / short") {
    Decision d{}; d.magic = kWireMagic; d.version = kWireVersion; d.sequence = 1;
    uint8_t buf[kWireOnWire];
    encodeDecision(d, buf, sizeof(buf));
    Decision out{};
    CHECK(decodeDecision(buf, kWireOnWire - 1, out) == DecodeResult::Short);
    uint8_t bad[kWireOnWire]; for (size_t i=0;i<kWireOnWire;i++) bad[i]=buf[i];
    bad[0] ^= 0xFF;
    CHECK(decodeDecision(bad, kWireOnWire, out) == DecodeResult::BadMagic);
    uint8_t badcrc[kWireOnWire]; for (size_t i=0;i<kWireOnWire;i++) badcrc[i]=buf[i];
    badcrc[kWireOnWire-1] ^= 0xFF;
    CHECK(decodeDecision(badcrc, kWireOnWire, out) == DecodeResult::BadCrc);
}

TEST_CASE("crc32 known vector matches dl_wire") {
    // Copy the exact (input bytes -> expected crc) vector from test_wire.c.
    const uint8_t v[] = {0x44,0x4C,0x4B,0x31};
    CHECK(crc32(v, sizeof(v)) == /* value from test_wire.c */ 0u);  // replace 0u
}

// ... port peekKind, ping/pong, hello/hello_ack roundtrip + golden-byte cases
//     from test_wire.c here, verbatim vectors ...
```
> Replace the `crc32` expected value and add the remaining cases by copying
> `test_wire.c`. Do not invent vectors — transcribe them so byte-parity is proven.

- [ ] **Step 3: Run test to verify it fails**

Run: `cmake --build build -j` then `./build/fpvd_tests -tc="decision encode/decode roundtrip"`
Expected: link error (no `wire.cpp`) — implement next.

- [ ] **Step 4: Implement `wire.cpp`**

Port `dl_wire.c` to `src/dynlink/wire.cpp` inside `namespace fpvd::dynlink`. Preserve
the **big-endian field packing and CRC-32 (poly 0xEDB88320, init/final 0xFFFFFFFF)
byte-for-byte**. Mechanical transforms: `dl_decision_t` → `Decision`, return
`DecodeResult::X` instead of `DL_DECODE_X`, `dl_wire_peek_kind` → `peekKind`. Keep the
payload offsets identical to the C source.

- [ ] **Step 5: Run tests to verify they pass**

Run: `cmake --build build -j && ./build/fpvd_tests -tc="*"` (run all wire cases)
Expected: PASS.

- [ ] **Step 6: Commit**
```bash
# add src/dynlink/wire.cpp to fpvd_core target_sources and
# tests/unit/test_dl_wire.cpp to fpvd_tests target_sources in CMakeLists.txt first
git add src/dynlink/wire.hpp src/dynlink/wire.cpp tests/unit/test_dl_wire.cpp CMakeLists.txt
git commit -m "dynlink: port wire codec (byte-identical to dl_wire)"
```

### Task 3: `Dedup`

**Files:**
- Create: `src/dynlink/dedup.hpp`, `src/dynlink/dedup.cpp`
- Test: `tests/unit/test_dl_dedup.cpp`
- Reference: `dl_dedup.{h,c}`, `tests/drone/test_dedup.c`

- [ ] **Step 1: Write the failing test**
```cpp
#include <doctest/doctest.h>
#include "dynlink/dedup.hpp"
using namespace fpvd::dynlink;

TEST_CASE("dedup accepts fresh, drops stale/equal, reset reseeds") {
    Dedup d;
    CHECK(d.check(10) == false);   // first accept
    CHECK(d.check(10) == true);    // equal -> drop
    CHECK(d.check(9)  == true);    // older -> drop
    CHECK(d.check(11) == false);   // newer -> accept
    d.reset();
    CHECK(d.check(5)  == false);   // post-reset accepts unconditionally
}
```

- [ ] **Step 2: Run to verify it fails** — Run: `cmake --build build -j` Expected: link error.

- [ ] **Step 3: Implement** — `src/dynlink/dedup.hpp` + `.cpp`:
```cpp
#pragma once
#include <cstdint>
namespace fpvd::dynlink {
class Dedup {
public:
    bool check(uint32_t seq);   // true => drop (stale/dup). port dl_dedup_check
    void reset();               // port dl_dedup_reset
private:
    uint32_t lastSeq_{0};
    bool ever_{false};
};
} // namespace fpvd::dynlink
```
Port `dl_dedup_check` (signed-32 delta ≤ 0 → drop) and `dl_dedup_reset` exactly.

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="dedup*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/dedup.hpp src/dynlink/dedup.cpp tests/unit/test_dl_dedup.cpp CMakeLists.txt
git commit -m "dynlink: port sequence dedup"
```

### Task 4: `Watchdog`

**Files:**
- Create: `src/dynlink/watchdog.hpp`, `src/dynlink/watchdog.cpp`
- Test: `tests/unit/test_dl_watchdog.cpp`
- Reference: `dl_watchdog.{h,c}`, `tests/drone/test_watchdog.c`

- [ ] **Step 1: Write the failing test**
```cpp
#include <doctest/doctest.h>
#include "dynlink/watchdog.hpp"
using namespace fpvd::dynlink;

TEST_CASE("watchdog trips once per silent window") {
    Watchdog w(1000);                 // 1000 ms timeout
    w.notifyDecision(0);
    CHECK(w.tick(500)  == false);     // within window
    CHECK(w.tick(1500) == true);      // first stale tick -> trip (one-shot)
    CHECK(w.tick(2000) == false);     // still silent -> no re-trip
    CHECK(w.isTripped() == true);
    w.notifyDecision(2500);           // fresh decision clears latch
    CHECK(w.isTripped() == false);
}
```

- [ ] **Step 2: Run to verify it fails** — Expected: link error.

- [ ] **Step 3: Implement** — header mirrors `dl_watchdog_t`:
```cpp
#pragma once
#include <cstdint>
namespace fpvd::dynlink {
class Watchdog {
public:
    explicit Watchdog(uint32_t timeoutMs);
    void setTimeout(uint32_t timeoutMs);          // for hot reconcile
    void notifyDecision(uint64_t nowMs);          // port dl_watchdog_notify_decision
    bool tick(uint64_t nowMs);                    // port dl_watchdog_tick (one-shot)
    bool isTripped() const { return tripped_; }
private:
    uint64_t lastDecisionMs_{0};
    uint32_t timeoutMs_;
    bool everSeen_{false};
    bool tripped_{false};
};
} // namespace fpvd::dynlink
```
Port `dl_watchdog_*` exactly; `setTimeout` just updates `timeoutMs_` (new, for reconcile).

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="watchdog*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/watchdog.hpp src/dynlink/watchdog.cpp tests/unit/test_dl_watchdog.cpp CMakeLists.txt
git commit -m "dynlink: port GS-link watchdog"
```

### Task 5: `applyDirection` + `computeRoiQp`

**Files:**
- Create: `src/dynlink/apply_direction.hpp` (header-only), `src/dynlink/roi_qp.hpp`, `src/dynlink/roi_qp.cpp`
- Test: `tests/unit/test_dl_apply_direction.cpp`, `tests/unit/test_dl_roi_qp.cpp`
- Reference: `dl_apply.h`, `dl_backend_enc.c` (`dl_compute_roi_qp_raw`), `tests/drone/test_apply_stagger.c`, `tests/drone/test_roi_qp.c`

- [ ] **Step 1: Write the failing tests**

`tests/unit/test_dl_apply_direction.cpp`:
```cpp
#include <doctest/doctest.h>
#include "dynlink/apply_direction.hpp"
using namespace fpvd::dynlink;
TEST_CASE("apply direction up/down/equal/first") {
    CHECK(applyDirection(0, 6000, true)  == ApplyDir::Equal);  // first
    CHECK(applyDirection(4000, 6000, false) == ApplyDir::Up);
    CHECK(applyDirection(6000, 4000, false) == ApplyDir::Down);
    CHECK(applyDirection(6000, 6000, false) == ApplyDir::Equal);
}
```
`tests/unit/test_dl_roi_qp.cpp` (copy exact pairs from `test_roi_qp.c`; design defaults: 6000→0, 4000→-12, 2000→-24):
```cpp
#include <doctest/doctest.h>
#include "dynlink/roi_qp.hpp"
using namespace fpvd::dynlink;
TEST_CASE("roi qp linear ramp, quantized, clamped") {
    // threshold=6000 anchor=2000 floor=-24 step=3
    CHECK(computeRoiQp(6000, 6000, 2000, -24, 3) == 0);
    CHECK(computeRoiQp(7000, 6000, 2000, -24, 3) == 0);    // >= threshold
    CHECK(computeRoiQp(2000, 6000, 2000, -24, 3) == -24);  // <= anchor
    CHECK(computeRoiQp(1000, 6000, 2000, -24, 3) == -24);  // clamp
    CHECK(computeRoiQp(4000, 6000, 2000, -24, 3) == -12);  // midpoint
    // ... port remaining test_roi_qp.c vectors verbatim ...
}
```

- [ ] **Step 2: Run to verify they fail** — Expected: missing headers/symbols.

- [ ] **Step 3: Implement**

`src/dynlink/apply_direction.hpp` (header-only, port `dl_apply.h`):
```cpp
#pragma once
#include <cstdint>
namespace fpvd::dynlink {
enum class ApplyDir { Equal, Up, Down };
inline ApplyDir applyDirection(uint16_t prevBitrateKbps, uint16_t newBitrateKbps,
                               bool firstDecision) {
    if (firstDecision) return ApplyDir::Equal;
    if (newBitrateKbps > prevBitrateKbps) return ApplyDir::Up;
    if (newBitrateKbps < prevBitrateKbps) return ApplyDir::Down;
    return ApplyDir::Equal;
}
} // namespace fpvd::dynlink
```
`src/dynlink/roi_qp.hpp`:
```cpp
#pragma once
#include <cstdint>
namespace fpvd::dynlink {
// Port of dl_compute_roi_qp_raw. Returns a delta in [floor, 0].
int computeRoiQp(uint16_t bitrateKbps, uint16_t thresholdKbps,
                 uint16_t lowAnchorKbps, int8_t floor, uint8_t step);
} // namespace fpvd::dynlink
```
`src/dynlink/roi_qp.cpp` — port `dl_compute_roi_qp_raw` arithmetic exactly (integer
truncation toward zero, clamp to `[floor, 0]`).

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="apply direction*"` and `-tc="roi qp*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/apply_direction.hpp src/dynlink/roi_qp.hpp src/dynlink/roi_qp.cpp \
        tests/unit/test_dl_apply_direction.cpp tests/unit/test_dl_roi_qp.cpp CMakeLists.txt
git commit -m "dynlink: port apply-direction + roi-qp curve"
```

---

# Phase 2 — I/O clients

### Task 6: Extend `WfbControlClient` with `setInterleaveDepth`

**Files:**
- Modify: `src/translate/wfb_cmd.h`, `src/translate/wfb_control.hpp`, `src/translate/wfb_control.cpp`
- Test: `tests/unit/test_wfb_control.cpp` (existing — add a case)
- Reference: `dl_backend_tx.c` (`send_depth`, `CMD_SET_INTERLEAVE_DEPTH`), `dynamic-link/drone/src/vendored/tx_cmd.h`

- [ ] **Step 1: Write the failing test**

Append to `tests/unit/test_wfb_control.cpp` (uses the file's existing localhost fake
UDP server harness — follow the existing `setRadio`/`setFec` test's pattern for naming):
```cpp
TEST_CASE("WfbControlClient setInterleaveDepth wire format + rc") {
    // Reuse this file's fake wfb_tx UDP server fixture. Assert the request:
    //   cmd_id == kWfbCmdSetInterleaveDepth (5), payload byte == depth.
    // Echo rc=0 -> ok; rc=EINVAL -> error; silence -> 500ms timeout -> error.
    FakeWfbTx srv;                       // existing fixture in this file
    WfbControlClient cli("127.0.0.1", srv.port());
    srv.expectAndReplyRc(0);
    auto r = cli.setInterleaveDepth(2);
    CHECK(r.ok);
    CHECK(srv.lastCmdId() == 5);
    CHECK(srv.lastDepth() == 2);
}
```
> Match the existing fixture's method names in `test_wfb_control.cpp`; the asserts above
> are illustrative of that fixture's shape.

- [ ] **Step 2: Run to verify it fails** — Expected: no member `setInterleaveDepth`.

- [ ] **Step 3: Add the command to `wfb_cmd.h`**

In `src/translate/wfb_cmd.h`, add the opcode and union member (matching wfb-ng's
`tx_cmd.h`):
```cpp
constexpr uint8_t kWfbCmdSetFec             = 1;
constexpr uint8_t kWfbCmdSetRadio           = 2;
constexpr uint8_t kWfbCmdSetInterleaveDepth = 5;
```
Add to the `WfbCmdReq::u` union: `struct { uint8_t depth; } set_interleave_depth;`
(and the symmetric member in `WfbCmdResp::u` if the file mirrors them).

- [ ] **Step 4: Implement `setInterleaveDepth`**

In `wfb_control.hpp` add the declaration; in `wfb_control.cpp` implement, mirroring
`setFec`/`setRadio`:
```cpp
WfbCtlResult WfbControlClient::setInterleaveDepth(uint8_t depth) {
    uint32_t id = reqId_++;
    WfbCmdReq req{};
    req.req_id = htonl(id);
    req.cmd_id = kWfbCmdSetInterleaveDepth;
    req.u.set_interleave_depth.depth = depth;
    return sendAndRecv(&req,
        offsetof(WfbCmdReq, u) + sizeof(req.u.set_interleave_depth),
        id, "set_interleave_depth");
}
```

- [ ] **Step 5: Run to verify pass** — `./build/fpvd_tests -tc="*nterleave*"` Expected: PASS (plus existing wfb_control cases still PASS).

- [ ] **Step 6: Commit**
```bash
git add src/translate/wfb_cmd.h src/translate/wfb_control.hpp src/translate/wfb_control.cpp tests/unit/test_wfb_control.cpp
git commit -m "wfb_control: add setInterleaveDepth (CMD_SET_INTERLEAVE_DEPTH)"
```

### Task 7: `EncoderClient` (waybeam HTTP)

**Files:**
- Create: `src/dynlink/encoder_client.hpp`, `src/dynlink/encoder_client.cpp`
- Test: `tests/unit/test_dl_encoder_client.cpp`
- Reference: `dl_backend_enc.c` (`apply_set`, `dl_backend_enc_apply`, `_request_idr`, `_apply_safe`)

- [ ] **Step 1: Write the failing test (against a localhost fake HTTP server)**
```cpp
#include <doctest/doctest.h>
#include "dynlink/encoder_client.hpp"
#include <httplib.h>
#include <thread>
using namespace fpvd::dynlink;

TEST_CASE("EncoderClient applies bitrate+roiQp+fps, diffs, throttles IDR") {
    httplib::Server srv;
    std::vector<std::string> hits;
    srv.Get("/api/v1/set", [&](const httplib::Request& r, httplib::Response& res){
        hits.push_back(r.target); res.set_content("ok","text/plain"); });
    srv.Get("/request/idr", [&](const httplib::Request& r, httplib::Response& res){
        hits.push_back("/request/idr"); res.set_content("ok","text/plain"); });
    int port = srv.bind_to_any_port("127.0.0.1");
    std::thread th([&]{ srv.listen_after_bind(); });
    srv.wait_until_ready();

    EncoderClient enc("127.0.0.1", static_cast<uint16_t>(port),
                      /*minIdrIntervalMs=*/500,
                      RoiCurve{6000,2000,-24,3});
    // bitrate 6000 -> roiQp 0; fps 60
    CHECK(enc.apply(6000, 60) == 0);
    CHECK(hits.back().find("video0.bitrate=6000") != std::string::npos);
    CHECK(hits.back().find("fpv.roiQp=0") != std::string::npos);
    CHECK(hits.back().find("video0.fps=60") != std::string::npos);
    size_t n = hits.size();
    CHECK(enc.apply(6000, 60) == 0);          // identical -> diffed out, no new hit
    CHECK(hits.size() == n);
    CHECK(enc.requestIdr(1000) == 0);          // first IDR sent
    CHECK(enc.requestIdr(1100) == 1);          // throttled (<500ms)
    CHECK(enc.requestIdr(1700) == 0);          // window elapsed -> sent

    srv.stop(); th.join();
}
```

- [ ] **Step 2: Run to verify it fails** — Expected: missing header.

- [ ] **Step 3: Implement**

`src/dynlink/encoder_client.hpp`:
```cpp
#pragma once
#include <cstdint>
#include <string>
namespace fpvd::dynlink {
struct RoiCurve { uint16_t thresholdKbps; uint16_t lowAnchorKbps; int8_t floor; uint8_t step; };
class EncoderClient {
public:
    EncoderClient(std::string host, uint16_t port, uint32_t minIdrIntervalMs, RoiCurve roi);
    // GET /api/v1/set?video0.bitrate=&fpv.roiQp=&[video0.fps=]. Diff-based.
    // bitrate==0 is a no-op sentinel. Returns 0 ok/no-op, -1 HTTP fail.
    int apply(uint16_t bitrateKbps, uint8_t fps);
    // GET /request/idr, throttled by minIdrIntervalMs. 0 sent, 1 throttled, -1 fail.
    int requestIdr(uint64_t nowMs);
    // Push safe bitrate (roiQp recomputed, fps unchanged). Returns 0/-1.
    int applySafe(uint16_t safeBitrateKbps);
    void setRoiCurve(RoiCurve roi) { roi_ = roi; }        // hot reconcile
    void setMinIdrInterval(uint32_t ms) { minIdrIntervalMs_ = ms; } // hot reconcile
private:
    int httpGet(const std::string& path);
    std::string host_; uint16_t port_;
    uint32_t minIdrIntervalMs_; RoiCurve roi_;
    // diff state (port dl_backend_enc's last_*):
    bool lastValid_{false}; uint16_t lastBitrate_{0}; int8_t lastRoiQp_{0}; uint8_t lastFps_{0};
    bool idrEverSent_{false}; uint64_t lastIdrMs_{0};
};
} // namespace fpvd::dynlink
```
`src/dynlink/encoder_client.cpp` — port `apply_set`/`dl_backend_enc_apply`/
`_request_idr`/`_apply_safe`. Compute roiQp via `computeRoiQp(...)` (Task 5). Build the
query string exactly as `apply_set` (always emit `fpv.roiQp`, emit `video0.fps` only
when `fps != 0`). Use `httplib::Client cli(host_, port_); cli.Get(path)` for `httpGet`;
return 0 when `res && res->status/100 == 2`, else -1. Arm the IDR throttle on **any**
attempt (matches the C comment).

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="EncoderClient*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/encoder_client.hpp src/dynlink/encoder_client.cpp tests/unit/test_dl_encoder_client.cpp CMakeLists.txt
git commit -m "dynlink: EncoderClient (waybeam HTTP) replacing dl_backend_enc"
```

### Task 8: `RadioTxpower` (`iw` via posix_spawnp)

**Files:**
- Create: `src/dynlink/radio_txpower.hpp`, `src/dynlink/radio_txpower.cpp`
- Test: `tests/unit/test_dl_radio_txpower.cpp`
- Reference: `dl_backend_radio.c` (`run_iw`, `dl_backend_radio_apply`)

- [ ] **Step 1: Write the failing test (stub `iw` on PATH)**
```cpp
#include <doctest/doctest.h>
#include "dynlink/radio_txpower.hpp"
#include <cstdlib>
#include <fstream>
using namespace fpvd::dynlink;

TEST_CASE("RadioTxpower runs iw with dBm*100 mBm, diffs on unchanged") {
    // Put a fake `iw` on PATH that records argv to a temp file and exits 0.
    // (Create build/fake-bin/iw as a shell script; prepend build/fake-bin to PATH.)
    RadioTxpower r("wlan0");
    CHECK(r.apply(/*dBm=*/20) == 0);     // first -> runs iw, records "...fixed 2000"
    CHECK(r.apply(/*dBm=*/20) == 0);     // unchanged -> no iw run (diffed out)
    CHECK(r.apply(/*dBm=*/22) == 0);     // changed -> runs iw "...fixed 2200"
    // assert recorded argv from the temp file: last call has "2200".
}
```
> Build the fake-`iw` harness the same way `tests/unit/test_tune_radio.cpp` stubs
> `iw`/`ip` (follow that file's PATH-prepend pattern).

- [ ] **Step 2: Run to verify it fails** — Expected: missing header.

- [ ] **Step 3: Implement**

`src/dynlink/radio_txpower.hpp`:
```cpp
#pragma once
#include <cstdint>
#include <string>
namespace fpvd::dynlink {
// Sets NIC txpower via `iw dev <iface> set txpower fixed <dBm*100>` (posix_spawnp).
// Diff-based: only runs iw when the dBm value changes. Port of dl_backend_radio.
class RadioTxpower {
public:
    explicit RadioTxpower(std::string iface) : iface_(std::move(iface)) {}
    void setIface(std::string iface) { iface_ = std::move(iface); current_.reset(); }
    int apply(int8_t dBm);        // 0 ok/no-op, -1 iw failure
    int applySafe(int8_t dBm);    // unconditional run (watchdog fallback)
private:
    int runIw(int8_t dBm);        // port run_iw
    std::string iface_;
    std::optional<int8_t> current_{};
};
} // namespace fpvd::dynlink
```
(Add `#include <optional>`.) `radio_txpower.cpp` — port `run_iw` using `posix_spawnp`
with argv `{"iw","dev",iface,"set","txpower","fixed", "<dBm*100>", NULL}` and
`waitpid`; `apply` diffs against `current_`; `applySafe` always runs. Drop the
`dl_latency`/`dl_dbg` calls (debug suite deferred); keep the same success/failure rc.

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="RadioTxpower*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/radio_txpower.hpp src/dynlink/radio_txpower.cpp tests/unit/test_dl_radio_txpower.cpp CMakeLists.txt
git commit -m "dynlink: RadioTxpower iw helper replacing dl_backend_radio"
```

---

# Phase 3 — Stateful side-effect modules

### Task 9: `HelloSm`

**Files:**
- Create: `src/dynlink/hello.hpp`, `src/dynlink/hello.cpp`
- Test: `tests/unit/test_dl_hello.cpp`
- Reference: `dl_hello.{h,c}`, `tests/drone/test_dl_hello.c`

> Difference from the C version: `HelloSm` takes `mtu`, `fps`, and `generationId`
> **directly** (the C `dl_hello_init` read them from `/etc/wfb.yaml`/`majestic.yaml`).
> fpvd is authoritative, so there is no file read. `generationId` is supplied by the
> caller (Daemon generates it once via `std::random_device`).

- [ ] **Step 1: Write the failing test** — port `test_dl_hello.c` cases: ANNOUNCING
cadence (initial vs steady), ANNOUNCING→KEEPALIVE on matching ACK, KEEPALIVE→ANNOUNCING
after 3 missed keepalives, mismatched-generation ACK ignored.
```cpp
#include <doctest/doctest.h>
#include "dynlink/hello.hpp"
using namespace fpvd::dynlink;

TEST_CASE("hello announce -> keepalive on matching ack") {
    HelloSm h(/*generationId=*/0xABCD, /*mtu=*/1400, /*fps=*/60, HelloCadence{});
    CHECK(h.state() == HelloState::Announcing);
    uint8_t buf[kHelloOnWire];
    CHECK(h.buildAnnounce(buf, sizeof(buf)) == kHelloOnWire);
    HelloAck ack{}; ack.magic = kHelloAckMagic; ack.generationIdEcho = 0xABCD;
    h.onAck(ack);
    CHECK(h.state() == HelloState::Keepalive);
    HelloAck bad{}; bad.generationIdEcho = 0x9999;
    // mismatched ack does not change state (port test_dl_hello.c assertion)
}
// ... port cadence + drop-back cases from test_dl_hello.c ...
```

- [ ] **Step 2: Run to verify it fails** — Expected: missing header.

- [ ] **Step 3: Implement**

`src/dynlink/hello.hpp`:
```cpp
#pragma once
#include "dynlink/wire.hpp"
#include <cstddef>
#include <cstdint>
namespace fpvd::dynlink {
enum class HelloState { Init, Announcing, Keepalive, Disabled };
struct HelloCadence {
    uint32_t announceInitialMs{500};
    uint32_t announceSteadyMs{5000};
    uint32_t keepaliveMs{10000};
    uint32_t announceInitialCount{60};
};
class HelloSm {
public:
    HelloSm(uint32_t generationId, uint16_t mtuBytes, uint16_t fps, HelloCadence cad);
    HelloState state() const { return state_; }
    size_t buildAnnounce(uint8_t* buf, size_t buflen);   // port dl_hello_build_announce
    uint32_t nextDelayMs() const;                        // port dl_hello_next_delay_ms
    void onAck(const HelloAck& ack);                     // port dl_hello_on_ack
    void onKeepaliveTick();                              // port dl_hello_on_keepalive_tick
    void setMtuFps(uint16_t mtu, uint16_t fps);          // hot reconcile; re-announce
    void setVanilla(bool vanilla);                       // interleavingSupported flag bit
private:
    HelloState state_{HelloState::Announcing};
    uint32_t generationId_; uint16_t mtu_; uint16_t fps_;
    uint32_t announceCount_{0}; uint32_t keepalivesWithoutAck_{0};
    uint8_t flags_{0}; HelloCadence cad_;
};
} // namespace fpvd::dynlink
```
`hello.cpp` — port the state machine. `buildAnnounce` fills a `Hello{}` (magic, version,
`flags_`, `generationId_`, `mtu_`, `fps_`, `applierBuildSha=0`) and calls
`encodeHello`. `setMtuFps`/`setVanilla` are new (reconcile): update fields; if the
vanilla flag bit changed, reset to `Announcing` so the GS relearns the capability.

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="hello*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/hello.hpp src/dynlink/hello.cpp tests/unit/test_dl_hello.cpp CMakeLists.txt
git commit -m "dynlink: port HELLO state machine (mtu/fps injected, no file reads)"
```

### Task 10: `OsdWriter`

**Files:**
- Create: `src/dynlink/osd.hpp`, `src/dynlink/osd.cpp`
- Test: `tests/unit/test_dl_osd.cpp`
- Reference: `dl_osd.{h,c}`, `tests/drone/test_osd.c`

- [ ] **Step 1: Write the failing test** — port `test_osd.c`: writing status to a temp
path produces the expected msposd line; event line + IDR counter behave as in C.
```cpp
#include <doctest/doctest.h>
#include "dynlink/osd.hpp"
#include "dynlink/wire.hpp"
#include <fstream>
#include <sstream>
using namespace fpvd::dynlink;

TEST_CASE("osd writes status line atomically") {
    std::string path = "/tmp/fpvd_test_osd.msg";
    OsdWriter osd(path, /*enabled=*/true, /*updateIntervalMs=*/1000, /*debugLatency=*/false);
    Decision d{}; d.mcs = 3; d.bitrateKbps = 6000; d.k = 8; d.n = 12;
    osd.writeStatus(d, /*rssiDbm=*/-60);
    std::ifstream f(path); std::stringstream ss; ss << f.rdbuf();
    CHECK(ss.str().find(/* expected token from test_osd.c, e.g. */ "6000") != std::string::npos);
    // ... port remaining test_osd.c assertions (event line, idr bump) ...
}
```

- [ ] **Step 2: Run to verify it fails** — Expected: missing header.

- [ ] **Step 3: Implement** — `osd.hpp`:
```cpp
#pragma once
#include "dynlink/wire.hpp"
#include <cstdint>
#include <string>
namespace fpvd::dynlink {
class OsdWriter {
public:
    OsdWriter(std::string msgPath, bool enabled, uint32_t updateIntervalMs, bool debugLatency);
    void writeStatus(const Decision& d, int rssiDbm);   // port dl_osd_write_status
    void writeEvent(const std::string& text);           // port dl_osd_write_event
    void bumpIdr();                                      // port dl_osd_bump_idr
    void eventWatchdog();                                // port dl_osd_event_watchdog
    void setEnabled(bool e) { enabled_ = e; }            // hot reconcile
private:
    std::string msgPath_; bool enabled_; uint32_t updateIntervalMs_; bool debugLatency_;
    uint64_t idrCount_{0};
    // ... any cached event-line state from dl_osd ...
};
} // namespace fpvd::dynlink
```
`osd.cpp` — port `dl_osd_*`. Keep the **exact msposd directive format** (e.g. the
`&L..&F..` prefix from `dl_osd.c`) and the atomic tmpfile+rename write. Drop the
`debugLatency` per-call latency rendering for now (debug suite deferred) — accept the
flag but treat it as a no-op, noted in a comment. When `enabled_` is false, write
nothing (matches C `dl_osd_open` returning NULL → calls are no-ops).

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="osd*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/osd.hpp src/dynlink/osd.cpp tests/unit/test_dl_osd.cpp CMakeLists.txt
git commit -m "dynlink: port OSD msposd status writer"
```

### Task 11: `IdrListener`

**Files:**
- Create: `src/dynlink/idr_listen.hpp`, `src/dynlink/idr_listen.cpp`
- Test: `tests/unit/test_dl_idr_listen.cpp`
- Reference: `dl_idr_listen.{h,c}`, `tests/drone/test_idr_listen.c`

- [ ] **Step 1: Write the failing test** — port `test_idr_listen.c`: open on an
ephemeral port, send N datagrams, `drain()` returns N; `fd()` valid; port 0 → disabled
(`fd() == -1`, `drain() == 0`).
```cpp
#include <doctest/doctest.h>
#include "dynlink/idr_listen.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
using namespace fpvd::dynlink;

TEST_CASE("idr listener drains datagrams; port 0 disables") {
    IdrListener l("127.0.0.1", 0);
    CHECK(l.fd() == -1);
    CHECK(l.drain() == 0);
    // ... port the bound-socket + send + drain==N case from test_idr_listen.c ...
}
```

- [ ] **Step 2: Run to verify it fails** — Expected: missing header.

- [ ] **Step 3: Implement** — `idr_listen.hpp`:
```cpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <string>
namespace fpvd::dynlink {
class IdrListener {
public:
    IdrListener(const std::string& bindAddr, uint16_t port);  // port==0 disables
    ~IdrListener();
    IdrListener(const IdrListener&) = delete;
    IdrListener& operator=(const IdrListener&) = delete;
    int fd() const { return fd_; }     // -1 if disabled
    size_t drain();                    // recvfrom until EAGAIN; returns count
private:
    int fd_{-1};
};
} // namespace fpvd::dynlink
```
`idr_listen.cpp` — port `dl_idr_listen_open`/`_fd`/`_drain`/`_close` (non-blocking
`AF_INET` `SOCK_DGRAM`, `SO_REUSEADDR`, bind; `drain` loops `recvfrom` until `EAGAIN`).

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="idr*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/idr_listen.hpp src/dynlink/idr_listen.cpp tests/unit/test_dl_idr_listen.cpp CMakeLists.txt
git commit -m "dynlink: port PixelPilot IDR-token listener"
```

---

# Phase 4 — Snapshot + controller

### Task 12: `DlRuntimeConfig` snapshot + `DlStatus` + `buildDlSnapshot`

**Files:**
- Create: `src/dynlink/runtime_config.hpp`, `src/dynlink/runtime_config.cpp`
- Test: `tests/unit/test_dl_runtime_config.cpp`
- Reference: spec §5.1; `src/config/schema.hpp` (`DynamicLink`, `Config`)

- [ ] **Step 1: Write the failing test**
```cpp
#include <doctest/doctest.h>
#include "dynlink/runtime_config.hpp"
#include "config/schema.hpp"
using namespace fpvd;
using namespace fpvd::dynlink;

TEST_CASE("buildDlSnapshot maps schema + derived inputs") {
    Config c{};                       // defaults
    c.link.mtu = 1400; c.video.fps = 90;
    c.dynamicLink.safe.mcs = 3; c.dynamicLink.healthTimeoutMs = 8000;
    auto s = buildDlSnapshot(c, "wlan1");
    CHECK(s.iface == "wlan1");
    CHECK(s.helloMtuBytes == 1400);
    CHECK(s.helloFps == 90);
    CHECK(s.safe.mcs == 3);
    CHECK(s.healthTimeoutMs == 8000);
    CHECK(s.roiQp.thresholdKbps == 6000);   // default carried through
}
```

- [ ] **Step 2: Run to verify it fails** — Expected: missing header.

- [ ] **Step 3: Implement** — `runtime_config.hpp`:
```cpp
#pragma once
#include "dynlink/encoder_client.hpp"   // RoiCurve
#include <atomic>
#include <cstdint>
#include <string>
namespace fpvd { struct Config; }
namespace fpvd::dynlink {

struct SafeDefaults { uint8_t mcs; uint8_t k; uint8_t n; uint8_t depth;
                      uint8_t bandwidth; int8_t txPowerDbm; uint16_t bitrateKbps; };

struct DlRuntimeConfig {
    uint32_t healthTimeoutMs; uint32_t minIdrIntervalMs;
    uint32_t applyStaggerMs;  uint32_t applySubPaceMs;
    bool interleavingSupported; bool osdEnabled; bool osdDebugLatency; bool debug;
    RoiCurve roiQp; SafeDefaults safe;
    uint16_t helloMtuBytes; uint16_t helloFps;
    std::string iface;
};

enum class HelloPub { Disabled, Announcing, Keepalive };
struct DlStatus {                         // published by the loop, read by HTTP thread
    bool running{false};
    bool watchdogTripped{false};
    long lastDecisionAgeMs{-1};           // -1 => none yet
    HelloPub hello{HelloPub::Disabled};
};

// Pinned production endpoints; overridable in tests.
struct Endpoints {
    std::string listenAddr{"0.0.0.0"};   uint16_t listenPort{5800};
    std::string wfbCtlAddr{"127.0.0.1"}; uint16_t wfbCtlPort{8000};
    std::string encHost{"127.0.0.1"};    uint16_t encPort{80};
    std::string idrAddr{"0.0.0.0"};      uint16_t idrPort{11223};
    std::string gsTunnelAddr{"10.5.0.1"};uint16_t gsTunnelPort{5801};
    std::string osdMsgPath{"/tmp/MSPOSD.msg"}; uint32_t osdUpdateIntervalMs{1000};
};

DlRuntimeConfig buildDlSnapshot(const Config& c, const std::string& iface);

} // namespace fpvd::dynlink
```
`runtime_config.cpp` — implement `buildDlSnapshot`: copy `c.dynamicLink.*` into the
struct, set `roiQp = {thresholdKbps, lowAnchorKbps, floor, step}`, `safe = {...}`,
`helloMtuBytes = c.link.mtu`, `helloFps = c.video.fps`, `iface = iface`. (Include
`config/schema.hpp` in the `.cpp`.)

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="buildDlSnapshot*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/runtime_config.hpp src/dynlink/runtime_config.cpp tests/unit/test_dl_runtime_config.cpp CMakeLists.txt
git commit -m "dynlink: DlRuntimeConfig snapshot + DlStatus + Endpoints"
```

### Task 13: `DynamicLinkController` skeleton (lifecycle: start/stop/thread/eventfd/poll)

**Files:**
- Create: `src/dynlink/controller.hpp`, `src/dynlink/controller.cpp`
- Test: `tests/integration/test_dl_controller.cpp`
- Reference: `dl_applier.c` (socket/timer setup, poll skeleton, signal/stop handling)

- [ ] **Step 1: Write the failing lifecycle test**
```cpp
#include <doctest/doctest.h>
#include "dynlink/controller.hpp"
#include "dynlink/runtime_config.hpp"
#include <thread>
#include <chrono>
using namespace fpvd::dynlink;

static Endpoints ephemeral() {
    Endpoints e; e.listenPort = 0; e.idrPort = 0;   // ask OS for free ports / disable
    return e;
}

TEST_CASE("controller starts and stops cleanly") {
    DlRuntimeConfig snap{};                 // zero-ish; fields not exercised here
    snap.healthTimeoutMs = 10000; snap.iface = "wlan0";
    DynamicLinkController c(ephemeral());
    c.start(snap, /*generationId=*/0x1234);
    CHECK(c.status().running == true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    c.stop();
    CHECK(c.status().running == false);
    c.start(snap, 0x1234);                  // restartable
    CHECK(c.status().running == true);
    c.stop();
}
```
> If `listenPort == 0` can't be supported by the bind path, give the test a fixed
> high port (e.g. 0 → pick 45800) — but keep `Endpoints` injectable.

- [ ] **Step 2: Run to verify it fails** — Expected: missing header.

- [ ] **Step 3: Implement the skeleton**

`src/dynlink/controller.hpp`:
```cpp
#pragma once
#include "dynlink/runtime_config.hpp"
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>
namespace fpvd::dynlink {
class DynamicLinkController {
public:
    explicit DynamicLinkController(Endpoints ep = {});
    ~DynamicLinkController();
    DynamicLinkController(const DynamicLinkController&) = delete;
    DynamicLinkController& operator=(const DynamicLinkController&) = delete;

    void start(const DlRuntimeConfig& snap, uint32_t generationId);
    void stop();                                  // idempotent; joins the thread
    bool running() const { return running_.load(); }
    void setConfig(const DlRuntimeConfig& snap);  // hot reload (Task 17)
    DlStatus status() const;                       // snapshot of published status

private:
    void run();                                    // the poll(2) loop (Tasks 14-17)
    void publishStatus(const DlStatus&);

    Endpoints ep_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopFlag_{false};
    int eventFd_{-1};                              // reload/stop wake
    uint32_t generationId_{0};
    std::shared_ptr<const DlRuntimeConfig> cfg_;   // atomically swapped
    mutable std::mutex statusMu_;
    DlStatus status_{};
};
} // namespace fpvd::dynlink
```
`controller.cpp` skeleton — `start()`: store `cfg_` (via `std::atomic_store` on the
shared_ptr or a mutex), create `eventFd_` (`eventfd(0, EFD_NONBLOCK|EFD_CLOEXEC)`),
clear `stopFlag_`, set `running_ = true`, launch `thread_ = std::thread([this]{ run(); })`.
`stop()`: set `stopFlag_`, write 1 to `eventFd_`, `join()`, close fds, `running_ = false`
(idempotent if not running). `run()` for now: open the listen socket from `ep_`, build
a `pollfd` set of `{listen_fd, eventfd}`, loop `poll(-1)`; on eventfd readable drain it
and break if `stopFlag_`. `status()`: lock `statusMu_`, return `status_` with
`running = running_`. Mirror `dl_applier.c`'s socket helpers (`open_listen_socket`,
timer creation) — bring them over but leave decision handling for Task 14.

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="controller starts*"` Expected: PASS (no leaks/hangs; thread joins).

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/controller.hpp src/dynlink/controller.cpp tests/integration/test_dl_controller.cpp CMakeLists.txt
git commit -m "dynlink: DynamicLinkController lifecycle skeleton"
```

### Task 14: Decision dispatch (tx/radio/enc + stagger/sub-pace + watchdog/OSD tick)

**Files:**
- Modify: `src/dynlink/controller.{hpp,cpp}`
- Test: `tests/integration/test_dl_controller.cpp` (add an end-to-end case)
- Reference: `dl_applier.c` main loop (decision branch, gap timer, watchdog/OSD tick), `dl_backend_tx.c` (`dl_backend_tx_apply` diff order: FEC→DEPTH→RADIO; `apply_safe`)

- [ ] **Step 1: Write the failing end-to-end test**

Stand up a fake `wfb_tx` UDP control server + fake encoder HTTP server (reuse the Task 6
/ Task 7 fixtures) on injected `Endpoints`. Inject a decision datagram into the listen
port; assert the controller issues the expected `setFec`/`setRadio` and encoder
`apply`. Then go silent past `healthTimeoutMs` and assert safe-defaults are pushed.
```cpp
TEST_CASE("controller applies a decision and trips watchdog to safe") {
    // ep points listen/wfbCtl/enc at local fakes; healthTimeoutMs small.
    // 1) send one encoded Decision -> expect fake wfb_tx sees setFec(k,n) + setRadio(mcs,bw)
    //    and fake encoder sees video0.bitrate=...
    // 2) wait > healthTimeoutMs -> expect safe-defaults push (safe.mcs/k/n + safe bitrate)
    //    and status().watchdogTripped == true
}
```
> Build this against the same fakes used in Tasks 6/7; assert via their recorded calls.

- [ ] **Step 2: Run to verify it fails** — Expected: controller ignores decisions (no dispatch yet).

- [ ] **Step 3: Implement decision dispatch**

In `controller.cpp`, add owned members: `WfbControlClient`, `EncoderClient`,
`RadioTxpower`, `OsdWriter`, `Watchdog`, `Dedup`, and per-backend prev-state
(`Decision lastTx_`, `lastRadio_`, `lastEnc_`). Construct them in `start()` from `cfg_`
+ `ep_`. In `run()`:
- **Listen fd readable:** `peekKind`; for `Decision`, `decodeDecision`; `dedup_.check`;
  compute `applyDirection(lastEnc_.bitrateKbps, d.bitrateKbps, first)`; dispatch with
  stagger/sub-pace exactly as `dl_applier.c`:
  - tx apply = diff FEC (`setFec`), DEPTH (`setInterleaveDepth`, only if
    `cfg_->interleavingSupported` and depth changed), RADIO (`setRadio(stbc=0, ldpc=0,
    shortGi=false, bandwidth=d.bandwidth, mcs=d.mcs, vhtMode=false, vhtNss=1)`);
    `usleep(applySubPaceMs*1000)` between sub-commands. Pass `d.bandwidth` (already the
    20/40 radiotap value on the wire) **directly** — do not run it through
    `modulationWidth` (that only matters for `link.width=10`, which never appears on the
    decision wire). This matches `dl_backend_tx.c send_radio`.
  - radio apply = `RadioTxpower::apply(d.txPowerDbm)` (diffed).
  - enc apply = `EncoderClient::apply(d.bitrateKbps, d.fps)`.
  - Direction Up/Down → split across the gap timer (port `APPLY_UP_GAP`/`APPLY_DOWN_GAP`
    state and `arm_gap`/`disarm_gap`). Equal/stagger==0 → single shot.
  - `watchdog_.notifyDecision(nowMs)`; `osd_.writeStatus(d, 0)`; update `lastEnc_`/etc.
- **Tick timer:** `if (watchdog_.tick(nowMs))` → push safe defaults
  (`setFec(safe.k,safe.n)`, depth if interleaving, `setRadio(stbc=0, ldpc=0,
  shortGi=false, bandwidth=safe.bandwidth, mcs=safe.mcs, vhtMode=false, vhtNss=1)`,
  `RadioTxpower::applySafe(safe.txPowerDbm)`, `EncoderClient::applySafe(safe.bitrateKbps)`),
  `osd_.eventWatchdog()`, reset `lastTx_/lastRadio_/lastEnc_` + `dedup_.reset()`; publish
  `status_.watchdogTripped`. Also OSD periodic refresh on the tick.
- **Gap timer:** port the phase-2 apply from `dl_applier.c`.

(`safe.bandwidth` is also the 20/40 radiotap value — passed directly, no `modulationWidth`.)

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="controller applies*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/controller.hpp src/dynlink/controller.cpp tests/integration/test_dl_controller.cpp
git commit -m "dynlink: decision dispatch + stagger/sub-pace + watchdog safe-defaults"
```

### Task 15: HELLO timer + GS-tunnel socket

**Files:**
- Modify: `src/dynlink/controller.{hpp,cpp}`
- Test: `tests/integration/test_dl_controller.cpp` (add a HELLO case)
- Reference: `dl_applier.c` (hello timer branch, `open_gs_tunnel_socket`, ack handling)

- [ ] **Step 1: Write the failing test** — point `ep_.gsTunnelAddr/Port` at a local UDP
sink; after `start()`, assert at least one `DLHE` (peekKind == Hello) arrives; send a
matching `HelloAck` into the listen port; assert `status().hello` becomes `Keepalive`.

- [ ] **Step 2: Run to verify it fails** — Expected: no HELLO emitted.

- [ ] **Step 3: Implement** — add a `HelloSm hello_` member and a hello timerfd to the
poll set. Open the GS-tunnel UDP socket (`open_gs_tunnel_socket`: unconnected, `sendto`
per packet). On the hello timer: `if keepalive` → `onKeepaliveTick()`; build + `sendto`
the announce; re-arm via `nextDelayMs()` (coerce 0→1ms). On listen fd `HelloAck` →
`decodeHelloAck` → `hello_.onAck`. Publish `status_.hello`. Construct `hello_` in
`start()` with `generationId_`, `cfg_->helloMtuBytes`, `cfg_->helloFps`, and
`setVanilla(!cfg_->interleavingSupported)`.

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="*HELLO*"` (or the case name) Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/controller.hpp src/dynlink/controller.cpp tests/integration/test_dl_controller.cpp
git commit -m "dynlink: HELLO announce/keepalive + GS-tunnel socket"
```

### Task 16: IDR listener integration

**Files:**
- Modify: `src/dynlink/controller.{hpp,cpp}`
- Test: `tests/integration/test_dl_controller.cpp` (add an IDR case)
- Reference: `dl_applier.c` (idr listen branch)

- [ ] **Step 1: Write the failing test** — with `ep_.idrPort` set to a fixed test port
and a fake encoder, send a datagram to the IDR port; assert the controller calls
`EncoderClient::requestIdr` (fake encoder sees `GET /request/idr`), throttled by
`minIdrIntervalMs`.

- [ ] **Step 2: Run to verify it fails** — Expected: IDR datagram ignored.

- [ ] **Step 3: Implement** — add an `IdrListener idr_` member; add its `fd()` to the
poll set when enabled. On readable: `idr_.drain()`; if `> 0` → `osd_.bumpIdr()` and
`enc_.requestIdr(nowMs)`.

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="*IDR*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/controller.hpp src/dynlink/controller.cpp tests/integration/test_dl_controller.cpp
git commit -m "dynlink: IDR-token listener -> encoder IDR"
```

### Task 17: `setConfig` + eventfd reconcile (hot reload)

**Files:**
- Modify: `src/dynlink/controller.{hpp,cpp}`
- Test: `tests/integration/test_dl_controller.cpp` (add a hot-reload case)
- Reference: spec §5.2

- [ ] **Step 1: Write the failing test**
```cpp
TEST_CASE("setConfig hot-reloads knobs without restart") {
    // start with healthTimeoutMs=10000, safe.mcs=1; capture thread id / running.
    // call setConfig(snap with healthTimeoutMs=2000, safe.mcs=5).
    // assert: still running (no restart); watchdog now trips at ~2000ms;
    //         a subsequent watchdog safe-push uses mcs=5.
}
```

- [ ] **Step 2: Run to verify it fails** — Expected: new config not picked up.

- [ ] **Step 3: Implement** — `setConfig(snap)`: `std::atomic_store(&cfg_, make_shared<const DlRuntimeConfig>(snap))`
then `write(eventFd_, &one, 8)`. In `run()`, when the eventfd is readable and not
stopping: load the new `cfg_`, run `reconcile(old, new)`:
- `watchdog_.setTimeout(new.healthTimeoutMs)`;
- re-arm the tick timer if `osdUpdateIntervalMs`/`healthTimeoutMs` changed (tick =
  `min(osdUpdateIntervalMs, healthTimeoutMs/2)`, floor 100 ms — same formula as
  `dl_applier.c`);
- `enc_.setRoiCurve(new.roiQp); enc_.setMinIdrInterval(new.minIdrIntervalMs);`
- `osd_.setEnabled(new.osdEnabled);`
- `hello_.setMtuFps(new.helloMtuBytes, new.helloFps); hello_.setVanilla(!new.interleavingSupported);`
- keep `safe`/pacing in `cfg_` (read on next use). Store `old = new` for the next diff.

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="setConfig hot-reloads*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/dynlink/controller.hpp src/dynlink/controller.cpp tests/integration/test_dl_controller.cpp
git commit -m "dynlink: setConfig + eventfd reconcile (hot reload)"
```

---

# Phase 5 — Daemon integration

### Task 18: Remove `mavlinkEnable` from schema + defaults

**Files:**
- Modify: `src/config/schema.hpp:147,152-156`, `etc/defaults.json:63`
- Test: `tests/unit/test_schema.cpp` (existing)

- [ ] **Step 1: Update the failing expectation** — in `tests/unit/test_schema.cpp`,
remove any assertion referencing `mavlinkEnable`; add a guard that a JSON body with an
unknown `dynamicLink.mavlinkEnable` key is ignored/handled per the existing unknown-key
policy (match how other removed keys behave in that test file).

- [ ] **Step 2: Run to verify it fails** — `./build/fpvd_tests -tc="*schema*"` Expected: compile error (field still referenced) or assertion mismatch.

- [ ] **Step 3: Implement** — In `src/config/schema.hpp`: delete `bool mavlinkEnable{true};`
(line 147) and remove `mavlinkEnable` from the `NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLink, ...)`
field list (line ~156). In `etc/defaults.json`, delete the `"mavlinkEnable": true,` line
(63).

- [ ] **Step 4: Run to verify pass** — `cmake --build build -j && ./build/fpvd_tests -tc="*schema*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/config/schema.hpp etc/defaults.json tests/unit/test_schema.cpp
git commit -m "schema: drop dynamicLink.mavlinkEnable (MAVLink removed)"
```

### Task 19: Wire the controller into `Daemon`; delete translator + dl_applier child

**Files:**
- Modify: `src/daemon.hpp`, `src/daemon.cpp` (`seedOrchestrator`, `bootstrap`)
- Delete: `src/translate/dynamic_link.hpp`, `src/translate/dynamic_link.cpp`,
  `tests/unit/test_translate_dynamic_link.cpp`, `tests/unit/test_dl_applier_cli_assumptions.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Remove the deleted sources from the build**

In `CMakeLists.txt`, delete from `fpvd_core` `target_sources`: `src/translate/dynamic_link.cpp`.
Delete from `fpvd_tests` `target_sources`: `tests/unit/test_translate_dynamic_link.cpp`
and `tests/unit/test_dl_applier_cli_assumptions.cpp`. Delete the four files:
```bash
git rm src/translate/dynamic_link.hpp src/translate/dynamic_link.cpp \
       tests/unit/test_translate_dynamic_link.cpp tests/unit/test_dl_applier_cli_assumptions.cpp
```

- [ ] **Step 2: Add the controller member to `Daemon`**

In `src/daemon.hpp`: `#include "dynlink/controller.hpp"`. Add private members:
```cpp
    dynlink::DynamicLinkController dl_;
    uint32_t dlGenerationId_{0};   // random per boot, set in ctor
```
Add a public accessor `dynlink::DlStatus dynamicLinkStatus() const { return dl_.status(); }`.

- [ ] **Step 3: Drop the `dl_applier` child from `seedOrchestrator`**

In `src/daemon.cpp::seedOrchestrator`, delete the entire
`if (effective_.dynamicLink.enabled) { SupervisedSpec dl{}; ... orch_.add(std::move(dl)); }`
block (it referenced the now-deleted `dynamicLinkArgs`). Remove the
`#include "translate/dynamic_link.hpp"` at the top of `daemon.cpp`.

- [ ] **Step 4: Start/stop the controller in `bootstrap` and shutdown**

In the `Daemon` constructor, generate `dlGenerationId_` once via `std::random_device`.
In `bootstrap(startProcesses)`, after `seedOrchestrator()` + `orch_.startAll()`, if
`startProcesses && effective_.dynamicLink.enabled`:
```cpp
    dl_.start(dynlink::buildDlSnapshot(effective_, radio_.iface), dlGenerationId_);
```
Ensure `Daemon`'s destructor / shutdown path calls `dl_.stop()` before the orchestrator
tears down (add `dl_.stop();` at the start of the shutdown sequence).

- [ ] **Step 5: Build & run the full suite**

Run: `cmake -S . -B build && cmake --build build -j && ./build/fpvd_tests`
Expected: configures without the deleted files; all tests PASS (no references to
`dynamicLinkArgs`).

- [ ] **Step 6: Commit**
```bash
git add -A
git commit -m "daemon: own DynamicLinkController; delete argv translator + dl_applier child"
```

### Task 20: `apply()` routing — controller start/stop/hot-reload/restart-around

**Files:**
- Modify: `src/daemon.cpp` (`apply`)
- Test: `tests/integration/test_daemon.cpp` (or new `tests/integration/test_dl_apply_routing.cpp`)
- Reference: spec §8.3; existing `apply()` in `src/daemon.cpp`

- [ ] **Step 1: Write the failing routing tests**

Add cases (use the existing `test_daemon.cpp` fixture with a fake radio-up/orchestrator;
the controller can run with injected ephemeral `Endpoints` — expose a test seam to set
`Daemon`'s controller `Endpoints`, or assert on `dynamicLinkStatus().running`):
```cpp
TEST_CASE("apply: dynamicLink knob change hot-reloads, no orchestrator rebuild") {
    // enabled=true already applied. PATCH dynamicLink.safe.mcs=3 + apply.
    // assert: orchestrator NOT rebuilt (process pids unchanged / no stopAll),
    //         controller still running (status().running stays true).
}
TEST_CASE("apply: enabled false->true starts controller; true->false stops it") { /* ... */ }
TEST_CASE("apply: encoder change while enabled rebuilds + restart-around") {
    // PATCH video.codec while enabled -> needsRebuild true; controller stopped before
    // stopAll and started after startAll (status().running ends true).
}
```

- [ ] **Step 2: Run to verify it fails** — Expected: dynamicLink change still rebuilds (old behavior) / no controller calls.

- [ ] **Step 3: Implement the routing**

In `src/daemon.cpp::apply`, after computing `subs`/`link` and `wasDlEnabled`:
- Add `const bool enabledNew = effective_.dynamicLink.enabled;` (note `effective_` is
  assigned `pending_` mid-function — capture `enabledOld = wasDlEnabled` before, read
  `enabledNew` after the assignment).
- **Remove `dlAffects` from `needsRebuild`.** New:
  ```cpp
  const bool needsRebuild = subs.encoder || subs.telemetry ||
      !subs.servicesAffected.empty() || link.fullRestart;
  ```
- In the **full-rebuild branch** (`reallyRestart && needsRebuild`): before `orch_.stopAll()`,
  add `if (enabledOld) dl_.stop();`. After `orch_.startAll()` (and `bringUpRadio` has set
  `radio_`), add:
  ```cpp
  if (enabledNew)
      dl_.start(dynlink::buildDlSnapshot(effective_, radio_.iface), dlGenerationId_);
  ```
- In the **hot/no-rebuild branch** (`reallyRestart`, after the existing link hot-apply
  block, before the `version_++`/return): add controller routing:
  ```cpp
  if (!enabledOld && enabledNew)
      dl_.start(dynlink::buildDlSnapshot(effective_, radio_.iface), dlGenerationId_);
  else if (enabledOld && !enabledNew)
      dl_.stop();
  else if (enabledOld && enabledNew && subs.dynamicLink)
      dl_.setConfig(dynlink::buildDlSnapshot(effective_, radio_.iface));
  ```
  (Place this so it also runs on the plain no-link-change apply path; the `nicChannel`
  deferred-return branch must run the controller routing **before** detaching its worker.)
- Keep `if (dlAffects) restarted.push_back("dl_applier");` but rename the pushed label to
  `"dynamicLink"` and base it on `subs.dynamicLink && (enabledOld || enabledNew)`.

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="apply: *dynamicLink*"` and the other two cases Expected: PASS; existing `test_daemon`/`test_http_handlers` still PASS.

- [ ] **Step 5: Commit**
```bash
git add src/daemon.cpp tests/integration/test_daemon.cpp
git commit -m "daemon: route dynamicLink apply to controller (hot reload, no bounce)"
```

### Task 21: `/status` `dynamicLink` block

**Files:**
- Modify: `src/status.cpp` (`buildStatus`)
- Test: `tests/integration/test_http_handlers.cpp` or `tests/integration/test_daemon.cpp`
- Reference: spec §8.4; existing `buildStatus`

- [ ] **Step 1: Write the failing test**
```cpp
TEST_CASE("status exposes dynamicLink block; no dl_applier process row") {
    // disabled: status["dynamicLink"] == {enabled:false, running:false}
    // enabled+running: dynamicLink.running == true, has watchdogTripped/lastDecisionAgeMs/hello
    // processes[] never contains a name == "dl_applier"
}
```

- [ ] **Step 2: Run to verify it fails** — Expected: no `dynamicLink` key in status JSON.

- [ ] **Step 3: Implement** — in `buildStatus(Daemon& d)`, add a `dynamicLink` object to
the returned JSON:
```cpp
    auto dls = d.dynamicLinkStatus();
    nlohmann::json dlj;
    if (!d.effective().dynamicLink.enabled) {
        dlj = {{"enabled", false}, {"running", false}};
    } else {
        const char* hello = dls.hello == dynlink::HelloPub::Keepalive ? "keepalive"
                          : dls.hello == dynlink::HelloPub::Announcing ? "announcing"
                          : "disabled";
        dlj = {
            {"enabled", true},
            {"running", dls.running},
            {"watchdogTripped", dls.watchdogTripped},
            {"lastDecisionAgeMs", dls.lastDecisionAgeMs < 0
                ? nlohmann::json(nullptr) : nlohmann::json(dls.lastDecisionAgeMs)},
            {"hello", hello}
        };
    }
```
Add `{"dynamicLink", dlj}` to the top-level returned object. (`#include "dynlink/runtime_config.hpp"`.)

- [ ] **Step 4: Run to verify pass** — `./build/fpvd_tests -tc="status exposes dynamicLink*"` Expected: PASS.

- [ ] **Step 5: Commit**
```bash
git add src/status.cpp tests/integration/test_http_handlers.cpp
git commit -m "status: add dynamicLink block, drop dl_applier process row"
```

---

# Phase 6 — Verification

### Task 22: Full-suite, cross-build, and on-hardware verification

**Files:** none (verification only); update `README.md` adaptive-link section.

- [ ] **Step 1: Run the complete host test suite**

Run: `cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug && cmake --build build -j && ./build/fpvd_tests`
Expected: ALL tests PASS, including every new `test_dl_*` case and the existing suite.

- [ ] **Step 2: Cross-build the ssc338q binary**

Run (in `nix-shell`):
```bash
cmake -S . -B build/ssc338q -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-ssc338q.cmake
cmake --build build/ssc338q --target fpvd -j
file build/ssc338q/fpvd
```
Expected: statically-linked ARM EABI5 executable (success criterion #8).

- [ ] **Step 3: Update README**

In `README.md`, update the "Adaptive link (`dl-applier`)" section: it is no longer a
supervised child — `dynamicLink.enabled` now starts an in-process controller; the
locked-fields list and `dynamicLink.*` knobs are unchanged but **hot-reloadable**;
`/status` exposes a `dynamicLink` block (not a `dl_applier` process row); MAVLink status
is removed. Add the ssc338q cross-build command from Step 2.

- [ ] **Step 4: On-hardware smoke checklist (manual; not automated)**

Document the run against a drone (maps to spec §14 success criteria):
1. Boot `enabled:false` → `GET /status` has `dynamicLink:{enabled:false,running:false}`,
   no `dl_applier` process, no `/etc/dynamic-link/drone.conf` read.
2. `PATCH {"dynamicLink":{"enabled":true}}` + `/apply` → `dynamicLink.running:true`, GS
   sees HELLO, decisions apply; `pidof wfb_video_tx waybeam` unchanged (no bounce).
3. `PATCH {"dynamicLink":{"safe":{"mcs":3}}}` + `/apply` → applied live; all
   `wfb`/`waybeam` pids unchanged.
4. `PATCH {"link":{"mtu":1400}}` + `/apply` → HELLO re-announces mtu=1400; no bounce.
5. `PATCH {"dynamicLink":{"enabled":false}}` + `/apply` → `running:false`; pids unchanged.
6. `PATCH {"video":{"codec":"h264"}}` + `/apply` while enabled → wfb/waybeam bounce
   (expected), controller restart-around, GS re-HELLO.

- [ ] **Step 5: Commit**
```bash
git add README.md
git commit -m "docs: README adaptive-link is now in-process + hot-reloadable; ssc338q build"
```

---

## Self-review notes (author check — see spec coverage)

- Spec §4.1 module port map → Tasks 2–5, 9–11, 13–16. §4.2 I/O unification → Tasks 6,7,8.
  §4.3 deletions → Tasks 18,19. §5 snapshot/reconcile → Tasks 12,17. §6 concurrency is
  satisfied by the existing `checkDynamicLinkLock` (unchanged) + the loop reading only
  its snapshot (Tasks 13–17). §7 iw helper → Task 8. §8.1/8.2 schema/translator →
  Tasks 18,19. §8.3 apply routing → Task 20. §8.4 status → Task 21. §9 error handling is
  implemented across Tasks 13–17 (soft-fail clients, watchdog) and surfaced in §status.
  §10 testing is embedded per task. §11 build + §14 #8 → Tasks 1, 22.
- **Deferred (out of scope, no task — by design):** MAVLink (removed, Task 18), debug
  suite (`dl_dbg`/`dl_latency`/SD log), `dl-inject`, encoder/`video.*` hot-apply,
  `/status` apply_fail/watchdog counters, deleting `drone/` from the dynamic-link repo,
  Buildroot recipe edits (live in `openipc-builder`).
- **Known seam to confirm during execution:** Task 20 needs the `Daemon` controller to
  use injectable `Endpoints` in tests (ephemeral ports). If `Daemon` hardcodes
  production `Endpoints`, add a test-only setter or constructor parameter in Task 19.
