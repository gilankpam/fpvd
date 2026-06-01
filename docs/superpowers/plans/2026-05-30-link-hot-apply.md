# Link Hot-Apply Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Apply `link.*` changes at runtime without restarting the wfb stack — NIC params (channel/width/txpower/mtu) via `iw`/`ip`, radiotap/FEC (mcs/fec/stbc/ldpc/bandwidth) via the video `wfb_tx` UDP control socket, with tun/tlm decoupled into boot-once constants.

**Architecture:** A pure `classifyLinkChange()` diff routes each changed link field to one of three mechanisms: a granular `radio-tune.sh` (run via `tuneRadio()`), a `WfbControlClient` (UDP `CMD_SET_RADIO`/`CMD_SET_FEC` to `127.0.0.1:8000`), or — only for `linkId`/`wlanAdapter` — the existing full-restart path. `apply()` gates the hot path so it runs only for purely hot-applicable link-only changes; everything else keeps today's behavior. channel/width retunes are deferred ~200 ms so the HTTP response flushes before the air link drops.

**Tech Stack:** C++17, doctest (single `fpvd_tests` binary), CMake, POSIX sockets, nlohmann/json, POSIX shell (`iw`/`ip`).

**Reference design:** `docs/superpowers/specs/2026-05-30-link-hot-apply-design.md`

---

## Build & test commands (used throughout)

- Configure once: `cmake -S . -B build`
- Build: `cmake --build build -j`
- Run one test: `./build/fpvd_tests --test-case="<name>"`  (run from repo root — tests use relative paths like `tests/fixtures/...`)
- Run all: `./build/fpvd_tests`

doctest returns non-zero on any failure.

## File map

- Create: `src/translate/wfb_cmd.h` — vendored packed wire structs + cmd ids
- Create: `src/translate/wfb_control.hpp` / `.cpp` — `WfbControlClient`
- Create: `scripts/radio-tune.sh` — single-action `iw`/`ip` tuner
- Create: `tests/fixtures/fake_radio_tune.sh` — records argv/env for tests
- Create: `tests/unit/test_link_classify.cpp` — `classifyLinkChange` tests
- Create: `tests/unit/test_wfb_control.cpp` — `WfbControlClient` tests
- Modify: `src/translate/wfb.hpp` — add `kVideoControlPort`
- Modify: `src/translate/wfb.cpp` — tun/tlm fixed constants; use the port constant
- Modify: `src/config/diff.hpp` / `diff.cpp` — `LinkChange` + `classifyLinkChange`
- Modify: `src/supervise/radio.hpp` / `radio.cpp` — `tuneRadio()`
- Modify: `src/daemon.hpp` — `DaemonPaths.radioTuneScript`
- Modify: `src/daemon.cpp` — gated hot path in `apply()`
- Modify: `src/main.cpp` — `--radio-tune` flag + wiring
- Modify: `tests/unit/test_translate_wfb.cpp` — tun/tlm new constants
- Modify: `tests/integration/test_radio.cpp` — `tuneRadio` tests
- Modify: `tests/integration/test_daemon.cpp` — hot-path tests
- Modify: `CMakeLists.txt` — new source, new test files, install rule

---

## Task 1: tun/tlm boot-once constants + video control-port constant

Decouple tun/tlm `wfb_tx` from `c.link.*` (fixed `mcs=0`, fec `3/5`, `-B 20`, stbc/ldpc off; only `linkId` shared). Introduce `kVideoControlPort` so the control port lives in one place.

**Files:**
- Modify: `src/translate/wfb.hpp`
- Modify: `src/translate/wfb.cpp`
- Test: `tests/unit/test_translate_wfb.cpp`

- [ ] **Step 1: Update the tunnel/telemetry test expectations to the new constants**

In `tests/unit/test_translate_wfb.cpp`, replace the `TunTx` assertions in the
`"translate.wfb: tunnel rx and tx argv"` test (the block currently checking
`-M`/`1`) with fixed-constant checks:

```cpp
    auto tx = wfbArgs(c, fpvd::WfbRole::TunTx, "wlan0", "/etc/drone.key");
    CHECK(tx[0] == "/usr/bin/wfb_tx");
    CHECK(contains(tx, "-p")); CHECK(contains(tx, "32"));
    CHECK(contains(tx, "-u")); CHECK(contains(tx, "5801"));
    // tun/tlm are boot-once with fixed robust params, independent of link.*
    auto at = [&](const std::string& flag){
        auto it = std::find(tx.begin(), tx.end(), flag);
        REQUIRE(it != tx.end());
        return *(it + 1);
    };
    CHECK(at("-M") == "0");   // robust mcs=0
    CHECK(at("-k") == "3");   // fec 3/5
    CHECK(at("-n") == "5");
    CHECK(at("-B") == "20");  // HT20
    CHECK(at("-S") == "0");
    CHECK(at("-L") == "0");
    CHECK(at("-i") == "7669206");  // shared linkId
```

Add a fixed-constant check to the `"translate.wfb: telemetry rx and tx argv"`
test too, after the existing `tx` checks:

```cpp
    auto att = [&](const std::string& flag){
        auto it = std::find(tx.begin(), tx.end(), flag);
        REQUIRE(it != tx.end());
        return *(it + 1);
    };
    CHECK(att("-M") == "0");
    CHECK(att("-k") == "3");
    CHECK(att("-n") == "5");
```

- [ ] **Step 2: Run the test to verify it fails**

Run: `cmake -S . -B build && cmake --build build -j && ./build/fpvd_tests --test-case="translate.wfb: tunnel rx and tx argv"`
Expected: FAIL — tun tx still emits `-M 1`, `-k 8`, `-n 12`.

- [ ] **Step 3: Add `kVideoControlPort` to `wfb.hpp`**

In `src/translate/wfb.hpp`, inside `namespace fpvd {`, above `enum class WfbRole`:

```cpp
// wfb_tx control socket port for the video instance (-C). Bound to 127.0.0.1.
constexpr int kVideoControlPort = 8000;
```

- [ ] **Step 4: Implement fixed tun/tlm tx in `wfb.cpp`**

In `src/translate/wfb.cpp`, add a helper above `wfbArgs` (after `commonTx`):

```cpp
// tun/tlm are boot-once processes with fixed, robust radiotap/FEC, decoupled
// from c.link.* (except the shared linkId). See
// docs/superpowers/specs/2026-05-30-link-hot-apply-design.md.
static std::vector<std::string> tunTlmTx(const Config& c, const std::string& key) {
    return {
        "/usr/bin/wfb_tx",
        "-K", key,
        "-M", "0",
        "-B", "20",
        "-k", "3",
        "-n", "5",
        "-S", "0",
        "-L", "0",
        "-i", std::to_string(c.link.linkId)
    };
}
```

Change the `VideoTx` case to use the port constant — replace
`a.push_back("-C"); a.push_back("8000");` with:

```cpp
            a.push_back("-C"); a.push_back(std::to_string(kVideoControlPort));
```

Change the `TunTx` case body — replace `auto a = commonTx(c, 1, iface, key);`
with `auto a = tunTlmTx(c, key);` (keep the rest of that case: the `-p 32`,
`-u 5801`, and `iface` pushes).

Change the `TlmTx` case body — replace `auto a = commonTx(c, 1, iface, key);`
with `auto a = tunTlmTx(c, key);` (keep `-p 16`, `-u 14551`, `iface`).

- [ ] **Step 5: Run the wfb translate tests to verify they pass**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="translate.wfb: *"`
Expected: PASS (all wfb translate cases).

- [ ] **Step 6: Commit**

```bash
git add src/translate/wfb.hpp src/translate/wfb.cpp tests/unit/test_translate_wfb.cpp
git commit -m "feat(wfb): tun/tlm boot-once constants + kVideoControlPort

tun/tlm wfb_tx now use fixed mcs=0, fec 3/5, HT20, stbc/ldpc off,
decoupled from link.* (linkId still shared). Centralize the video
control port (8000) as kVideoControlPort.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 2: `classifyLinkChange()` diff

Pure function classifying which mechanism each changed link field needs.

**Files:**
- Modify: `src/config/diff.hpp`
- Modify: `src/config/diff.cpp`
- Create: `tests/unit/test_link_classify.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Write the failing test**

Create `tests/unit/test_link_classify.cpp`:

```cpp
#include "doctest.h"
#include "config/diff.hpp"

using fpvd::Config;
using fpvd::classifyLinkChange;

TEST_CASE("classifyLinkChange: no change -> all false") {
    Config a{}, b{};
    auto c = classifyLinkChange(a, b);
    CHECK_FALSE(c.nicChannel);
    CHECK_FALSE(c.nicWidth);
    CHECK_FALSE(c.nicTxpower);
    CHECK_FALSE(c.nicMtu);
    CHECK_FALSE(c.videoRadiotap);
    CHECK_FALSE(c.videoFec);
    CHECK_FALSE(c.fullRestart);
}

TEST_CASE("classifyLinkChange: txpower only") {
    Config a{}, b{}; b.link.txpower = a.link.txpower + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.nicTxpower);
    CHECK_FALSE(c.nicChannel);
    CHECK_FALSE(c.videoRadiotap);
}

TEST_CASE("classifyLinkChange: channel -> nicChannel only (not nicWidth)") {
    Config a{}, b{}; b.link.channel = a.link.channel + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.nicChannel);
    CHECK_FALSE(c.nicWidth);
    CHECK_FALSE(c.videoRadiotap);
}

TEST_CASE("classifyLinkChange: width -> nicChannel + nicWidth + videoRadiotap") {
    Config a{}, b{}; b.link.width = 40;  // default 20
    auto c = classifyLinkChange(a, b);
    CHECK(c.nicChannel);
    CHECK(c.nicWidth);
    CHECK(c.videoRadiotap);
}

TEST_CASE("classifyLinkChange: mcs -> videoRadiotap only") {
    Config a{}, b{}; b.link.mcs = a.link.mcs + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.videoRadiotap);
    CHECK_FALSE(c.nicChannel);
    CHECK_FALSE(c.videoFec);
}

TEST_CASE("classifyLinkChange: fec -> videoFec only") {
    Config a{}, b{}; b.link.fec.k = a.link.fec.k + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.videoFec);
    CHECK_FALSE(c.videoRadiotap);
}

TEST_CASE("classifyLinkChange: linkId -> fullRestart") {
    Config a{}, b{}; b.link.linkId = a.link.linkId + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.fullRestart);
}

TEST_CASE("classifyLinkChange: wlanAdapter -> fullRestart") {
    Config a{}, b{}; b.link.wlanAdapter = std::string("bl-m8812eu2");
    auto c = classifyLinkChange(a, b);
    CHECK(c.fullRestart);
}
```

- [ ] **Step 2: Register the test file in CMake**

In `CMakeLists.txt`, in the `fpvd_tests` `target_sources` list, add after
`tests/unit/test_diff.cpp`:

```cmake
    tests/unit/test_link_classify.cpp
```

- [ ] **Step 3: Run to verify it fails**

Run: `cmake -S . -B build && cmake --build build -j`
Expected: FAIL to compile — `classifyLinkChange` / `LinkChange` undeclared.

- [ ] **Step 4: Declare `LinkChange` + `classifyLinkChange` in `diff.hpp`**

In `src/config/diff.hpp`, after the `SubsystemDiff` struct and before
`diffSubsystems`:

```cpp
// Per-field routing for a link change. nicChannel/nicWidth drive iw retune
// (channel and width both reconfigure the NIC; width also bumps the video
// radiotap bandwidth). videoRadiotap/videoFec drive wfb_tx control commands.
// fullRestart fields (linkId/wlanAdapter) cannot be hot-applied.
struct LinkChange {
    bool nicChannel{false};    // channel || width  (NIC retune; drops air link)
    bool nicWidth{false};      // width specifically (radiotap bandwidth follows)
    bool nicTxpower{false};    // txpower
    bool nicMtu{false};        // mtu
    bool videoRadiotap{false}; // mcs || stbc || ldpc || width
    bool videoFec{false};      // fec.k || fec.n
    bool fullRestart{false};   // linkId || wlanAdapter
};

LinkChange classifyLinkChange(const Config& a, const Config& b);
```

- [ ] **Step 5: Implement `classifyLinkChange` in `diff.cpp`**

In `src/config/diff.cpp`, inside `namespace fpvd {`, after `diffSubsystems`:

```cpp
LinkChange classifyLinkChange(const Config& a, const Config& b) {
    const auto& la = a.link;
    const auto& lb = b.link;
    LinkChange c;
    const bool channel = la.channel != lb.channel;
    const bool width   = la.width   != lb.width;
    c.nicChannel    = channel || width;
    c.nicWidth      = width;
    c.nicTxpower    = la.txpower != lb.txpower;
    c.nicMtu        = la.mtu != lb.mtu;
    c.videoRadiotap = (la.mcs != lb.mcs) || (la.stbc != lb.stbc) ||
                      (la.ldpc != lb.ldpc) || width;
    c.videoFec      = (la.fec.k != lb.fec.k) || (la.fec.n != lb.fec.n);
    c.fullRestart   = (la.linkId != lb.linkId) ||
                      (la.wlanAdapter != lb.wlanAdapter);
    return c;
}
```

- [ ] **Step 6: Run to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="classifyLinkChange: *"`
Expected: PASS (all 8 cases).

- [ ] **Step 7: Commit**

```bash
git add src/config/diff.hpp src/config/diff.cpp tests/unit/test_link_classify.cpp CMakeLists.txt
git commit -m "feat(config): classifyLinkChange routing diff

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 3: `WfbControlClient` (UDP wire client)

UDP client for a `wfb_tx` control socket. Mirrors `wfbng-dynamic-link/drone/src/dl_backend_tx.c`: connected UDP, `req_id` htonl + match-on-recv, 500 ms recv timeout, drain stale replies.

**Files:**
- Create: `src/translate/wfb_cmd.h`
- Create: `src/translate/wfb_control.hpp`
- Create: `src/translate/wfb_control.cpp`
- Create: `tests/unit/test_wfb_control.cpp`
- Modify: `CMakeLists.txt`

- [ ] **Step 1: Create the vendored wire header `wfb_cmd.h`**

Create `src/translate/wfb_cmd.h` (packed structs copied from
`wfb-ng/src/tx_cmd.h`):

```cpp
#pragma once
#include <cstddef>
#include <cstdint>

namespace fpvd {

constexpr uint8_t kWfbCmdSetFec   = 1;
constexpr uint8_t kWfbCmdSetRadio = 2;

#pragma pack(push, 1)
struct WfbCmdReq {
    uint32_t req_id;   // network byte order
    uint8_t  cmd_id;
    union {
        struct { uint8_t k; uint8_t n; } set_fec;
        struct {
            uint8_t stbc;
            bool    ldpc;
            bool    short_gi;
            uint8_t bandwidth;
            uint8_t mcs_index;
            bool    vht_mode;
            uint8_t vht_nss;
        } set_radio;
    } u;
};

struct WfbCmdResp {
    uint32_t req_id;   // network byte order
    uint32_t rc;       // network byte order
    union {
        struct { uint8_t k; uint8_t n; } get_fec;
        struct {
            uint8_t stbc;
            bool    ldpc;
            bool    short_gi;
            uint8_t bandwidth;
            uint8_t mcs_index;
            bool    vht_mode;
            uint8_t vht_nss;
        } get_radio;
    } u;
};
#pragma pack(pop)

} // namespace fpvd
```

- [ ] **Step 2: Create the header `wfb_control.hpp`**

Create `src/translate/wfb_control.hpp`:

```cpp
#pragma once
#include <cstdint>
#include <string>

namespace fpvd {

struct WfbCtlResult {
    bool ok{false};
    std::string error;
};

// Minimal UDP client for a wfb_tx control socket (-C port, bound to
// 127.0.0.1). Connected UDP; req_id htonl + match-on-recv; 500 ms recv
// timeout; drains stale replies before each send.
class WfbControlClient {
public:
    WfbControlClient(const std::string& addr, uint16_t port);
    ~WfbControlClient();

    WfbControlClient(const WfbControlClient&) = delete;
    WfbControlClient& operator=(const WfbControlClient&) = delete;

    WfbCtlResult setRadio(uint8_t stbc, bool ldpc, bool shortGi,
                          uint8_t bandwidth, uint8_t mcs,
                          bool vhtMode, uint8_t vhtNss);
    WfbCtlResult setFec(uint8_t k, uint8_t n);

private:
    WfbCtlResult sendAndRecv(const void* req, size_t reqLen,
                             uint32_t reqId, const char* label);
    int fd_{-1};
    uint32_t reqId_{1};
    std::string openError_;
};

} // namespace fpvd
```

- [ ] **Step 3: Write the failing test**

Create `tests/unit/test_wfb_control.cpp`:

```cpp
#include "doctest.h"
#include "translate/wfb_control.hpp"
#include "translate/wfb_cmd.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstddef>
#include <cstring>
#include <thread>

namespace {
// Bind a UDP server on 127.0.0.1:<ephemeral>; return fd and learned port.
int bindServer(uint16_t& port) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(fd >= 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    REQUIRE(::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
    socklen_t len = sizeof(a);
    REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len) == 0);
    port = ntohs(a.sin_port);
    return fd;
}
} // namespace

TEST_CASE("wfb_control: setRadio sends correct wire bytes, parses ok reply") {
    uint16_t port = 0;
    int srv = bindServer(port);

    fpvd::WfbCtlResult res;
    std::thread client([&] {
        fpvd::WfbControlClient c("127.0.0.1", port);
        res = c.setRadio(/*stbc=*/0, /*ldpc=*/false, /*shortGi=*/false,
                         /*bandwidth=*/40, /*mcs=*/5, /*vhtMode=*/false,
                         /*vhtNss=*/1);
    });

    fpvd::WfbCmdReq req{};
    sockaddr_in from{};
    socklen_t flen = sizeof(from);
    ssize_t n = ::recvfrom(srv, &req, sizeof(req), 0,
                           reinterpret_cast<sockaddr*>(&from), &flen);
    REQUIRE(n == static_cast<ssize_t>(offsetof(fpvd::WfbCmdReq, u) +
                                      sizeof(req.u.set_radio)));
    CHECK(req.cmd_id == fpvd::kWfbCmdSetRadio);
    CHECK(req.u.set_radio.bandwidth == 40);
    CHECK(req.u.set_radio.mcs_index == 5);
    CHECK(req.u.set_radio.vht_nss == 1);

    fpvd::WfbCmdResp resp{};
    resp.req_id = req.req_id;   // echo as-is (already network order)
    resp.rc = htonl(0);
    ::sendto(srv, &resp, offsetof(fpvd::WfbCmdResp, u), 0,
             reinterpret_cast<sockaddr*>(&from), flen);

    client.join();
    CHECK(res.ok);
    ::close(srv);
}

TEST_CASE("wfb_control: setFec sends k/n and non-zero rc surfaces as error") {
    uint16_t port = 0;
    int srv = bindServer(port);

    fpvd::WfbCtlResult res;
    std::thread client([&] {
        fpvd::WfbControlClient c("127.0.0.1", port);
        res = c.setFec(3, 5);
    });

    fpvd::WfbCmdReq req{};
    sockaddr_in from{};
    socklen_t flen = sizeof(from);
    ::recvfrom(srv, &req, sizeof(req), 0,
               reinterpret_cast<sockaddr*>(&from), &flen);
    CHECK(req.cmd_id == fpvd::kWfbCmdSetFec);
    CHECK(req.u.set_fec.k == 3);
    CHECK(req.u.set_fec.n == 5);

    fpvd::WfbCmdResp resp{};
    resp.req_id = req.req_id;
    resp.rc = htonl(22);   // EINVAL
    ::sendto(srv, &resp, offsetof(fpvd::WfbCmdResp, u), 0,
             reinterpret_cast<sockaddr*>(&from), flen);

    client.join();
    CHECK_FALSE(res.ok);
    CHECK(res.error.find("rc=22") != std::string::npos);
    ::close(srv);
}

TEST_CASE("wfb_control: silence yields timeout error") {
    uint16_t port = 0;
    int srv = bindServer(port);
    fpvd::WfbControlClient c("127.0.0.1", port);
    auto res = c.setFec(3, 5);   // server never replies (~500 ms)
    CHECK_FALSE(res.ok);
    CHECK(res.error.find("timeout") != std::string::npos);
    ::close(srv);
}
```

- [ ] **Step 4: Register the source + test in CMake**

In `CMakeLists.txt`, in the `fpvd_core` `target_sources`, add after
`src/translate/wfb.cpp`:

```cmake
    src/translate/wfb_control.cpp
```

In the `fpvd_tests` `target_sources`, add after `tests/unit/test_translate_wfb.cpp`:

```cmake
    tests/unit/test_wfb_control.cpp
```

- [ ] **Step 5: Run to verify it fails**

Run: `cmake -S . -B build && cmake --build build -j`
Expected: FAIL to link/compile — `WfbControlClient` methods undefined.

- [ ] **Step 6: Implement `wfb_control.cpp`**

Create `src/translate/wfb_control.cpp`:

```cpp
#include "translate/wfb_control.hpp"
#include "translate/wfb_cmd.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cerrno>
#include <cstddef>
#include <cstring>

namespace fpvd {

WfbControlClient::WfbControlClient(const std::string& addr, uint16_t port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        openError_ = std::string("socket: ") + std::strerror(errno);
        return;
    }
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    if (::inet_pton(AF_INET, addr.c_str(), &dst.sin_addr) != 1) {
        openError_ = "bad address: " + addr;
        ::close(fd_);
        fd_ = -1;
        return;
    }
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) < 0) {
        openError_ = std::string("connect: ") + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return;
    }
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 500000;   // 500 ms
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

WfbControlClient::~WfbControlClient() {
    if (fd_ >= 0) ::close(fd_);
}

WfbCtlResult WfbControlClient::sendAndRecv(const void* req, size_t reqLen,
                                           uint32_t reqId, const char* label) {
    if (fd_ < 0) return {false, std::string(label) + ": " + openError_};

    // Drain stale replies left over from a prior timed-out request.
    WfbCmdResp scratch;
    while (::recv(fd_, &scratch, sizeof(scratch), MSG_DONTWAIT) > 0) {}

    ssize_t nsent = ::send(fd_, req, reqLen, 0);
    if (nsent < 0 || static_cast<size_t>(nsent) != reqLen)
        return {false, std::string(label) + ": send failed"};

    for (;;) {
        WfbCmdResp resp;
        ssize_t nrecv = ::recv(fd_, &resp, sizeof(resp), 0);
        if (nrecv < 0)
            return {false, std::string(label) + ": timeout"};
        if (static_cast<size_t>(nrecv) < offsetof(WfbCmdResp, u))
            return {false, std::string(label) + ": short reply"};
        if (ntohl(resp.req_id) != reqId) continue;   // stale; keep waiting
        uint32_t rc = ntohl(resp.rc);
        if (rc != 0)
            return {false, std::string(label) + ": rc=" + std::to_string(rc)};
        return {true, {}};
    }
}

WfbCtlResult WfbControlClient::setRadio(uint8_t stbc, bool ldpc, bool shortGi,
                                        uint8_t bandwidth, uint8_t mcs,
                                        bool vhtMode, uint8_t vhtNss) {
    uint32_t id = reqId_++;
    WfbCmdReq req{};
    req.req_id = htonl(id);
    req.cmd_id = kWfbCmdSetRadio;
    req.u.set_radio.stbc = stbc;
    req.u.set_radio.ldpc = ldpc;
    req.u.set_radio.short_gi = shortGi;
    req.u.set_radio.bandwidth = bandwidth;
    req.u.set_radio.mcs_index = mcs;
    req.u.set_radio.vht_mode = vhtMode;
    req.u.set_radio.vht_nss = vhtNss;
    return sendAndRecv(&req, offsetof(WfbCmdReq, u) + sizeof(req.u.set_radio),
                       id, "set_radio");
}

WfbCtlResult WfbControlClient::setFec(uint8_t k, uint8_t n) {
    uint32_t id = reqId_++;
    WfbCmdReq req{};
    req.req_id = htonl(id);
    req.cmd_id = kWfbCmdSetFec;
    req.u.set_fec.k = k;
    req.u.set_fec.n = n;
    return sendAndRecv(&req, offsetof(WfbCmdReq, u) + sizeof(req.u.set_fec),
                       id, "set_fec");
}

} // namespace fpvd
```

- [ ] **Step 7: Run to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="wfb_control: *"`
Expected: PASS (3 cases; the timeout case takes ~500 ms).

- [ ] **Step 8: Commit**

```bash
git add src/translate/wfb_cmd.h src/translate/wfb_control.hpp src/translate/wfb_control.cpp tests/unit/test_wfb_control.cpp CMakeLists.txt
git commit -m "feat(translate): WfbControlClient for wfb_tx CMD_SET_RADIO/FEC

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 4: `radio-tune.sh` + `tuneRadio()`

A granular tune script (one `iw`/`ip` action) and the C++ runner that execs it.

**Files:**
- Create: `scripts/radio-tune.sh`
- Create: `tests/fixtures/fake_radio_tune.sh`
- Modify: `src/supervise/radio.hpp`
- Modify: `src/supervise/radio.cpp`
- Test: `tests/integration/test_radio.cpp`
- Modify: `CMakeLists.txt` (install rule)

- [ ] **Step 1: Create `radio-tune.sh`**

Create `scripts/radio-tune.sh`:

```sh
#!/bin/sh
# radio-tune.sh — apply ONE live radio change without restarting wfb.
# Usage: radio-tune.sh <channel|txpower|mtu>
# Inputs (env): FPVD_IFACE, FPVD_DRIVER, FPVD_CHANNEL, FPVD_WIDTH,
#               FPVD_TXPOWER, FPVD_MTU
set -eu

action="${1:-}"
iface="${FPVD_IFACE:-wlan0}"

case "$action" in
    channel)
        # 10MHz uses a dedicated token (baseband underclock, 20MHz modulation);
        # 40 => HT40+; everything else => HT20. Mirrors radio-up.sh.
        case "${FPVD_WIDTH:-20}" in
            10) iw "$iface" set channel "${FPVD_CHANNEL:-161}" 10MHz ;;
            40) iw "$iface" set channel "${FPVD_CHANNEL:-161}" HT40+ ;;
            *)  iw "$iface" set channel "${FPVD_CHANNEL:-161}" HT20 ;;
        esac
        ;;
    txpower)
        if [ "${FPVD_DRIVER:-}" = "88XXau" ]; then
            iw "$iface" set txpower fixed $(( ${FPVD_TXPOWER:-1} * -100 ))
        else
            iw "$iface" set txpower fixed $(( ${FPVD_TXPOWER:-1} *  50 ))
        fi
        ;;
    mtu)
        ip link set "$iface" mtu "${FPVD_MTU:-1500}"
        ;;
    *)
        echo "radio-tune.sh: unknown action '$action'" >&2
        exit 2
        ;;
esac
```

Make it executable:

```bash
chmod +x scripts/radio-tune.sh
```

- [ ] **Step 2: Create the test fixture `fake_radio_tune.sh`**

Create `tests/fixtures/fake_radio_tune.sh` (records argv + env to a file the
test names via `FPVD_TEST_RECORD`, inherited by the forked child):

```sh
#!/bin/sh
echo "action=$1 iface=${FPVD_IFACE} channel=${FPVD_CHANNEL} width=${FPVD_WIDTH} txpower=${FPVD_TXPOWER} mtu=${FPVD_MTU}" >> "$FPVD_TEST_RECORD"
exit 0
```

Make it executable:

```bash
chmod +x tests/fixtures/fake_radio_tune.sh
```

- [ ] **Step 3: Write the failing test**

In `tests/integration/test_radio.cpp`, add at the end (the file already
includes `"supervise/radio.hpp"`):

```cpp
#include <cstdlib>
#include <filesystem>
#include <fstream>

TEST_CASE("radio: tuneRadio passes action and env to script") {
    namespace fs = std::filesystem;
    auto rec = fs::temp_directory_path() / "fpvd-tune-record.txt";
    fs::remove(rec);
    ::setenv("FPVD_TEST_RECORD", rec.string().c_str(), 1);

    fpvd::Config c{};
    c.link.channel = 100;
    c.link.width = 40;
    c.link.txpower = 5;
    c.link.mtu = 1400;
    auto r = fpvd::tuneRadio("tests/fixtures/fake_radio_tune.sh", "txpower",
                             c, "wlan0", "8812eu");
    REQUIRE(r.ok);

    std::ifstream f(rec);
    std::string line;
    std::getline(f, line);
    CHECK(line.find("action=txpower") != std::string::npos);
    CHECK(line.find("iface=wlan0") != std::string::npos);
    CHECK(line.find("txpower=5") != std::string::npos);
    fs::remove(rec);
}

TEST_CASE("radio: tuneRadio surfaces non-zero exit + stderr") {
    fpvd::Config c{};
    auto r = fpvd::tuneRadio("tests/fixtures/fake_radio_up_fail.sh", "channel",
                             c, "wlan0", "8812eu");
    CHECK_FALSE(r.ok);
    CHECK(r.exitCode == 3);
    CHECK(r.stderrText.find("missing modules") != std::string::npos);
}
```

- [ ] **Step 4: Run to verify it fails**

Run: `cmake -S . -B build && cmake --build build -j`
Expected: FAIL to compile — `tuneRadio` undeclared.

- [ ] **Step 5: Declare `tuneRadio` in `radio.hpp`**

In `src/supervise/radio.hpp`, after the `bringUpRadio` declaration:

```cpp
// Apply a single live radio change via `scriptPath <action>` (channel,
// txpower, or mtu). Passes the relevant link.* fields plus the already-known
// iface/driver as env vars. Captures stderr; RadioResult.driver/iface unused.
RadioResult tuneRadio(const std::string& scriptPath, const std::string& action,
                      const Config& c, const std::string& iface,
                      const std::string& driver);
```

- [ ] **Step 6: Implement `tuneRadio` in `radio.cpp`**

In `src/supervise/radio.cpp`, inside `namespace fpvd {`, after `bringUpRadio`
(reuses the file-local `readAll`):

```cpp
RadioResult tuneRadio(const std::string& scriptPath, const std::string& action,
                      const Config& c, const std::string& iface,
                      const std::string& driver) {
    RadioResult r{};
    int errPipe[2];
    if (::pipe(errPipe) < 0) { r.ok = false; r.exitCode = -1; return r; }
    pid_t pid = ::fork();
    if (pid == 0) {
        ::dup2(errPipe[1], 2);
        ::close(errPipe[0]); ::close(errPipe[1]);
        setenv("FPVD_IFACE",   iface.c_str(),  1);
        setenv("FPVD_DRIVER",  driver.c_str(), 1);
        setenv("FPVD_CHANNEL", std::to_string(c.link.channel).c_str(), 1);
        setenv("FPVD_WIDTH",   std::to_string(c.link.width).c_str(),   1);
        setenv("FPVD_TXPOWER", std::to_string(c.link.txpower).c_str(), 1);
        setenv("FPVD_MTU",     std::to_string(c.link.mtu).c_str(),     1);
        ::execl(scriptPath.c_str(), scriptPath.c_str(), action.c_str(),
                static_cast<char*>(nullptr));
        _exit(127);
    }
    ::close(errPipe[1]);
    readAll(errPipe[0], r.stderrText);
    ::close(errPipe[0]);

    int status = 0;
    ::waitpid(pid, &status, 0);
    r.exitCode = WIFEXITED(status) ? WEXITSTATUS(status) : 128;
    r.ok = (r.exitCode == 0);
    return r;
}
```

- [ ] **Step 7: Run to verify it passes**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="radio: tuneRadio*"`
Expected: PASS (both new radio cases).

- [ ] **Step 8: Add the install rule for `radio-tune.sh`**

In `CMakeLists.txt`, after the `radio-up.sh` install line:

```cmake
install(PROGRAMS scripts/radio-tune.sh DESTINATION libexec/fpvd)
```

- [ ] **Step 9: Commit**

```bash
git add scripts/radio-tune.sh tests/fixtures/fake_radio_tune.sh src/supervise/radio.hpp src/supervise/radio.cpp tests/integration/test_radio.cpp CMakeLists.txt
git commit -m "feat(supervise): radio-tune.sh + tuneRadio for live iw/ip changes

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 5: `DaemonPaths.radioTuneScript` + main wiring

Add the tune-script path to `DaemonPaths` and wire a `--radio-tune` flag.

**Files:**
- Modify: `src/daemon.hpp`
- Modify: `src/main.cpp`

- [ ] **Step 1: Add the field to `DaemonPaths`**

In `src/daemon.hpp`, in `struct DaemonPaths`, add the new field **last**, after
`waybeamJsonPath` (NOT after `radioUpScript`). Appending last is what keeps the
existing 4-field positional initializers correct — they map to the first four
fields unchanged, and the new field value-initializes to `""`:

```cpp
struct DaemonPaths {
    std::string defaultsPath;    // /rom/etc/fpvd/defaults.json
    std::string overlayPath;     // /etc/fpvd/config.json
    std::string radioUpScript;   // /usr/libexec/fpvd/radio-up.sh
    std::string waybeamJsonPath; // /etc/waybeam.json
    std::string radioTuneScript; // /usr/libexec/fpvd/radio-tune.sh
};
```

> Why last and not next to `radioUpScript`: every existing test constructs
> `DaemonPaths` positionally with 4 args `{defaults, overlay, radioUp, waybeam}`.
> Inserting mid-struct would shift the waybeam path into `radioTuneScript`. New
> 5-arg call sites pass the tune script as the **5th** argument.

- [ ] **Step 2: Wire the flag + path in `main.cpp`**

In `src/main.cpp`, after the `radioUp` declaration (line ~19):

```cpp
    std::string radioTune    = "/usr/libexec/fpvd/radio-tune.sh";
```

In the arg-parse loop, after the `--radio-up` branch:

```cpp
        else if (a == "--radio-tune" && i + 1 < argc) radioTune = argv[++i];
```

Update the usage string to include `[--radio-tune PATH]` (add it next to
`[--radio-up PATH]`).

Replace the `DaemonPaths paths{...}` construction with the 5-field form
(tune script is the **5th** / last argument, matching the struct order):

```cpp
    fpvd::DaemonPaths paths{defaultsPath, overlayPath, radioUp, waybeamPath,
                            radioTune};
```

- [ ] **Step 3: Build to verify it compiles**

Run: `cmake -S . -B build && cmake --build build -j`
Expected: PASS (compiles; `radioTuneScript` populated in main, defaulted in tests).

- [ ] **Step 4: Run the full suite to confirm nothing regressed**

Run: `./build/fpvd_tests`
Expected: PASS (all existing tests still green).

- [ ] **Step 5: Commit**

```bash
git add src/daemon.hpp src/main.cpp
git commit -m "feat(daemon): DaemonPaths.radioTuneScript + --radio-tune flag

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 6: Gated hot path in `apply()`

Rewrite `apply()` dispatch: classify the link change, take the hot path for
link-only changes, keep the full-restart path for subsystem / linkId / wlanAdapter
changes, defer channel/width.

**Files:**
- Modify: `src/daemon.cpp`
- Test: `tests/integration/test_daemon.cpp`

- [ ] **Step 1: Write the failing hot-path tests**

In `tests/integration/test_daemon.cpp`, add at the end (file already includes
`<algorithm>`, `<filesystem>`, `<fstream>`; add `<thread>` to the includes at
the top if not present):

```cpp
TEST_CASE("daemon: txpower change takes hot path (tuneRadio, no rebuild)") {
    auto tmp = fs::temp_directory_path() / "fpvd-hot-txpower";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");
    auto rec = tmp / "tune-record.txt";
    ::setenv("FPVD_TEST_RECORD", rec.string().c_str(), 1);

    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string(),
        "tests/fixtures/fake_radio_tune.sh"
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"link":{"txpower":5}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);

    std::ifstream f(rec);
    std::string line;
    std::getline(f, line);
    CHECK(line.find("action=txpower") != std::string::npos);
    CHECK(line.find("txpower=5") != std::string::npos);
    fs::remove_all(tmp);
}

TEST_CASE("daemon: width change defers channel retune via tune script") {
    auto tmp = fs::temp_directory_path() / "fpvd-hot-width";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");
    auto rec = tmp / "tune-record.txt";
    ::setenv("FPVD_TEST_RECORD", rec.string().c_str(), 1);

    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string(),
        "tests/fixtures/fake_radio_tune.sh"
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"link":{"width":40}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);   // returns immediately; channel retune is deferred

    // The detached worker sleeps 200ms, runs the channel tune, then attempts a
    // video setRadio to 127.0.0.1:8000 which is not listening (~500ms timeout).
    // Wait long enough for the worker to finish before `d` is destroyed, so the
    // thread (capturing `this`) does not outlive the Daemon.
    std::this_thread::sleep_for(std::chrono::milliseconds(900));

    std::ifstream f(rec);
    std::string line;
    std::getline(f, line);
    CHECK(line.find("action=channel") != std::string::npos);
    CHECK(line.find("width=40") != std::string::npos);
    fs::remove_all(tmp);
}
```

- [ ] **Step 2: Run to verify it fails**

Run: `cmake -S . -B build && cmake --build build -j && ./build/fpvd_tests --test-case="daemon: txpower change*"`
Expected: FAIL — current `apply()` rebuilds the orchestrator instead of calling `tuneRadio`; the tune-record file is never written.

- [ ] **Step 3: Add includes to `daemon.cpp`**

In `src/daemon.cpp`, with the other `#include "translate/..."` lines, add:

```cpp
#include "translate/wfb_control.hpp"
#include "link_width.hpp"
```

(`config/diff.hpp` is already pulled in via `daemon.hpp`.)

- [ ] **Step 4: Replace the body of `apply()`**

Replace the entire `apply()` function (from `ApplyResult Daemon::apply(bool reallyRestart) {`
through its closing `}`) with:

```cpp
ApplyResult Daemon::apply(bool reallyRestart) {
    std::lock_guard<std::mutex> g(mu_);
    auto errs = validate(pending_);
    if (!errs.empty()) return {false, std::move(errs), {}, std::nullopt, version_};

    auto subs = diffSubsystems(effective_, pending_);
    auto link = classifyLinkChange(effective_, pending_);
    const bool wasDlEnabled = effective_.dynamicLink.enabled;

    // Beamforming is reconciled (not exec-supervised); report it as restarted
    // when its own block or the derived modulation width changed.
    const bool bfChanged =
        nlohmann::json(effective_.link.beamforming) !=
            nlohmann::json(pending_.link.beamforming) ||
        effective_.link.width != pending_.link.width;

    // Persist overlay (sparse diff vs defaults).
    auto defaultsJ = defaultsJson();
    auto pendingJ = nlohmann::json(pending_);
    auto overlay = computeOverlay(defaultsJ, pendingJ);
    atomicWriteJson(paths_.overlayPath, overlay);

    effective_ = pending_;
    rewriteWaybeamJson();

    std::vector<std::string> restarted;
    if (subs.radio) restarted.push_back("radio");
    if (subs.encoder) restarted.push_back("encoder");
    if (subs.telemetry) restarted.push_back("telemetry");
    const bool dlAffects =
        subs.dynamicLink && (wasDlEnabled || effective_.dynamicLink.enabled);
    if (dlAffects) restarted.push_back("dl_applier");
    for (auto& n : subs.servicesAffected) restarted.push_back(n);
    if (bfChanged) restarted.push_back("beamforming");

    // A rebuild bounces the whole orchestrator (including wfb). It is needed
    // only when a non-link subsystem changes, or when a link change cannot be
    // hot-applied (linkId / wlanAdapter). dlAffects keeps an mtu-only change
    // on the hot path when DL is off, but rebuilds when DL consumes it.
    const bool needsRebuild = subs.encoder || subs.telemetry || dlAffects ||
        !subs.servicesAffected.empty() || link.fullRestart;

    if (reallyRestart && needsRebuild) {
        // Full-restart path (unchanged): rebuild orchestrator + radio bring-up.
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
        reconcileBeamforming();
        version_++;
        lastApply_ = {nowIso(), true, restarted, std::nullopt};
        return {true, {}, restarted, std::nullopt, version_};
    }

    if (reallyRestart) {
        // Hot path: a purely hot-applicable link change — no wfb restart.
        // (A) Immediate, non-link-dropping changes.
        if (link.nicTxpower) {
            auto rr = tuneRadio(paths_.radioTuneScript, "txpower", effective_,
                                radio_.iface, radio_.driver);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("txpower: ") + rr.stderrText};
                return {false, {}, restarted, rr.stderrText, version_};
            }
        }
        if (link.nicMtu) {
            auto rr = tuneRadio(paths_.radioTuneScript, "mtu", effective_,
                                radio_.iface, radio_.driver);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("mtu: ") + rr.stderrText};
                return {false, {}, restarted, rr.stderrText, version_};
            }
        }
        if (link.videoFec) {
            WfbControlClient cli("127.0.0.1", kVideoControlPort);
            auto rr = cli.setFec(static_cast<uint8_t>(effective_.link.fec.k),
                                 static_cast<uint8_t>(effective_.link.fec.n));
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("fec: ") + rr.error};
                return {false, {}, restarted, rr.error, version_};
            }
        }
        if (link.videoRadiotap && !link.nicWidth) {
            // mcs/stbc/ldpc with no width change — push now (no link drop).
            WfbControlClient cli("127.0.0.1", kVideoControlPort);
            auto rr = cli.setRadio(
                static_cast<uint8_t>(effective_.link.stbc ? 1 : 0),
                effective_.link.ldpc, false,
                static_cast<uint8_t>(modulationWidth(effective_.link.width)),
                static_cast<uint8_t>(effective_.link.mcs), false, 1);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("radio: ") + rr.error};
                return {false, {}, restarted, rr.error, version_};
            }
        }

        // (B) Link-dropping change (channel and/or width) — defer ~200ms so the
        // HTTP response flushes before the air link (and wfb_tun session) drops.
        if (link.nicChannel) {
            version_++;
            lastApply_ = {nowIso(), true, restarted, std::nullopt};
            const bool pushWidth = link.nicWidth;
            std::thread([this, restarted, pushWidth] {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                std::lock_guard<std::mutex> g2(mu_);
                auto rr = tuneRadio(paths_.radioTuneScript, "channel", effective_,
                                    radio_.iface, radio_.driver);
                if (!rr.ok) {
                    lastApply_.ok = false;
                    lastApply_.error = std::string("channel: ") + rr.stderrText;
                    return;
                }
                if (pushWidth) {
                    // NIC retuned first; now bump the video radiotap bandwidth.
                    WfbControlClient cli("127.0.0.1", kVideoControlPort);
                    auto cr = cli.setRadio(
                        static_cast<uint8_t>(effective_.link.stbc ? 1 : 0),
                        effective_.link.ldpc, false,
                        static_cast<uint8_t>(modulationWidth(effective_.link.width)),
                        static_cast<uint8_t>(effective_.link.mcs), false, 1);
                    if (!cr.ok) {
                        lastApply_.ok = false;
                        lastApply_.error = std::string("radio: ") + cr.error;
                    }
                }
            }).detach();
            return {true, {}, restarted, std::nullopt, version_};
        }

        version_++;
        lastApply_ = {nowIso(), true, restarted, std::nullopt};
        return {true, {}, restarted, std::nullopt, version_};
    }

    // reallyRestart == false: re-seed orchestrator specs only (dry config load).
    orch_ = Orchestrator{};
    seedOrchestrator();
    version_++;
    lastApply_ = {nowIso(), true, restarted, std::nullopt};
    return {true, {}, restarted, std::nullopt, version_};
}
```

- [ ] **Step 5: Run the new hot-path tests to verify they pass**

Run: `cmake --build build -j && ./build/fpvd_tests --test-case="daemon: txpower change*" && ./build/fpvd_tests --test-case="daemon: width change*"`
Expected: PASS (the width case takes ~900 ms by design).

- [ ] **Step 6: Run the whole suite to confirm no regressions**

Run: `./build/fpvd_tests`
Expected: PASS — all prior daemon/dl/beamforming/http tests still green (they
use `apply(false)`, which keeps the re-seed branch).

- [ ] **Step 7: Commit**

```bash
git add src/daemon.cpp tests/integration/test_daemon.cpp
git commit -m "feat(daemon): gated hot path for link changes in apply()

Link-only changes apply in place: txpower/mtu via tuneRadio, mcs/fec/
stbc/ldpc via WfbControlClient, channel/width deferred ~200ms. Subsystem
and linkId/wlanAdapter changes keep the full-restart path.

Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>"
```

---

## Task 7: Final verification

- [ ] **Step 1: Clean rebuild + full suite**

Run: `rm -rf build && cmake -S . -B build && cmake --build build -j && ./build/fpvd_tests`
Expected: PASS (all tests, clean build with `-Wall -Wextra -Wpedantic`).

- [ ] **Step 2: Lint the shell scripts (if `shellcheck` available)**

Run: `command -v shellcheck >/dev/null && shellcheck scripts/radio-tune.sh tests/fixtures/fake_radio_tune.sh || echo "shellcheck not installed — skip"`
Expected: no errors (or skip message).

- [ ] **Step 3: Confirm executables are tracked as executable**

Run: `git ls-files --stage scripts/radio-tune.sh tests/fixtures/fake_radio_tune.sh`
Expected: mode `100755` for both. If `100644`, run
`git update-index --chmod=+x scripts/radio-tune.sh tests/fixtures/fake_radio_tune.sh` and commit.

- [ ] **Step 4: Manual on-hardware checklist (documented, not automated)**

Not runnable in CI — perform on a real drone when available:
- `txpower` change: `iw dev wlan0 get txpower` reflects it; video uninterrupted; `pidof wfb_tx` stable (no restart).
- `mcs` change (DL off): wfb_tx log shows "Radiotap updated …"; link stays up.
- `channel`/`width` change: both ends retune; HTTP `/apply` response returns before the drop; `GET /status` shows `lastApply.ok` (or the deferred error).

---

## Self-review notes

- **Spec coverage:** field→mechanism matrix (Tasks 4+6), `WfbControlClient` wire format (Task 3), `radio-tune.sh` actions incl. 10MHz/HT20/HT40+ + txpower scaling (Task 4), tun/tlm constants mcs=0/3-5/HT20 (Task 1), gated hot path + defer + ordering (Task 6), best-effort stop-on-first-error + deferred-worker reporting (Task 6), tests 1–5 (Tasks 2,3,4,6) + manual checklist (Task 7).
- **DL coordination:** unchanged — `checkDynamicLinkLock` already rejects DL-owned link writes at PATCH (`src/config/lock.cpp`); the hot path adds `dlAffects` so an mtu-only change rebuilds only when DL consumes it.
- **Type consistency:** `LinkChange` fields (`nicChannel`/`nicWidth`/`nicTxpower`/`nicMtu`/`videoRadiotap`/`videoFec`/`fullRestart`) and `WfbCtlResult{ok,error}` / `WfbControlClient::setRadio(7 args)`/`setFec(2 args)` / `tuneRadio(scriptPath,action,Config,iface,driver)` are used identically across Tasks 2/3/4/6. `kVideoControlPort` defined in Task 1, consumed in Task 6.
- **Note vs spec:** the implemented `LinkChange` adds an explicit `nicWidth` flag (not in the spec's struct sketch) so the deferred worker can order the NIC retune before the video bandwidth push. Behavior matches the spec's intent.
