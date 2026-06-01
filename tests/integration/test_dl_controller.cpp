#include "doctest.h"
#include "dynlink/controller.hpp"
#include "dynlink/runtime_config.hpp"
#include "dynlink/wire.hpp"
#include "translate/wfb_cmd.h"

#include <httplib.h>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstring>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

using namespace fpvd::dynlink;

static Endpoints ephemeral() {
    Endpoints e; e.listenPort = 45800; e.idrPort = 0;   // fixed test port; idr disabled
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

// ---------------------------------------------------------------------------
// End-to-end: decision dispatch + watchdog safe-defaults
// ---------------------------------------------------------------------------

namespace {

// Fake wfb_tx UDP control server. Binds 127.0.0.1:<ephemeral>, echoes every
// request (req_id + rc=0) and records the parsed commands for assertions.
// (Replicates the Task 6 fixture from test_wfb_control.cpp.)
struct FakeWfbTx {
    int fd{-1};
    uint16_t port{0};
    std::thread th;
    std::atomic<bool> stop{false};

    std::mutex mu;
    std::vector<std::pair<uint8_t, uint8_t>> fec;     // (k, n)
    std::vector<std::pair<uint8_t, uint8_t>> radio;    // (mcs, bandwidth)
    std::vector<uint8_t> depth;

    FakeWfbTx() {
        fd = ::socket(AF_INET, SOCK_DGRAM, 0);
        REQUIRE(fd >= 0);
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        a.sin_port = 0;
        REQUIRE(::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
        socklen_t len = sizeof(a);
        REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len) == 0);
        port = ntohs(a.sin_port);

        // 200 ms recv timeout so the thread can observe `stop`.
        timeval tv{0, 200'000};
        ::setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

        th = std::thread([this] { serve(); });
    }

    ~FakeWfbTx() {
        stop.store(true);
        if (th.joinable()) th.join();
        if (fd >= 0) ::close(fd);
    }

    void serve() {
        while (!stop.load()) {
            fpvd::WfbCmdReq req{};
            sockaddr_in from{};
            socklen_t flen = sizeof(from);
            ssize_t n = ::recvfrom(fd, &req, sizeof(req), 0,
                                   reinterpret_cast<sockaddr*>(&from), &flen);
            if (n < 0) continue;  // timeout / EAGAIN

            {
                std::lock_guard<std::mutex> lk(mu);
                if (req.cmd_id == fpvd::kWfbCmdSetFec) {
                    fec.emplace_back(req.u.set_fec.k, req.u.set_fec.n);
                } else if (req.cmd_id == fpvd::kWfbCmdSetRadio) {
                    radio.emplace_back(req.u.set_radio.mcs_index,
                                       req.u.set_radio.bandwidth);
                } else if (req.cmd_id == fpvd::kWfbCmdSetInterleaveDepth) {
                    depth.push_back(req.u.set_interleave_depth.depth);
                }
            }

            fpvd::WfbCmdResp resp{};
            resp.req_id = req.req_id;   // echo (already network order)
            resp.rc = htonl(0);
            ::sendto(fd, &resp, offsetof(fpvd::WfbCmdResp, u), 0,
                     reinterpret_cast<sockaddr*>(&from), flen);
        }
    }

    bool sawFec(uint8_t k, uint8_t n) {
        std::lock_guard<std::mutex> lk(mu);
        for (auto& f : fec) if (f.first == k && f.second == n) return true;
        return false;
    }
    bool sawRadio(uint8_t mcs, uint8_t bw) {
        std::lock_guard<std::mutex> lk(mu);
        for (auto& r : radio) if (r.first == mcs && r.second == bw) return true;
        return false;
    }
};

// Fake encoder HTTP server (replicates the Task 7 fixture from
// test_dl_encoder_client.cpp): records every /api/v1/set target.
struct FakeEnc {
    httplib::Server srv;
    std::vector<std::string> hits;
    std::mutex mu;
    int port{0};
    std::thread th;

    FakeEnc() {
        srv.Get("/api/v1/set", [&](const httplib::Request& r, httplib::Response& res) {
            std::lock_guard<std::mutex> lk(mu);
            hits.push_back(r.target);
            res.set_content("ok", "text/plain");
        });
        srv.Get("/request/idr", [&](const httplib::Request&, httplib::Response& res) {
            std::lock_guard<std::mutex> lk(mu);
            hits.push_back("/request/idr");
            res.set_content("ok", "text/plain");
        });
        port = srv.bind_to_any_port("127.0.0.1");
        th = std::thread([&] { srv.listen_after_bind(); });
        srv.wait_until_ready();
    }
    ~FakeEnc() {
        srv.stop();
        th.join();
    }

    bool sawContaining(const std::string& needle) {
        std::lock_guard<std::mutex> lk(mu);
        for (auto& h : hits) if (h.find(needle) != std::string::npos) return true;
        return false;
    }
};

// Inject one encoded Decision datagram into the controller's listen port.
void sendDecision(uint16_t listenPort, const Decision& d) {
    uint8_t buf[64];
    size_t n = encodeDecision(d, buf, sizeof(buf));
    REQUIRE(n > 0);

    int s = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(s >= 0);
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    dst.sin_port = htons(listenPort);
    ssize_t w = ::sendto(s, buf, n, 0,
                         reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    CHECK(w == static_cast<ssize_t>(n));
    ::close(s);
}

// Poll a predicate until true or deadline.
template <typename F>
bool waitFor(F pred, int timeoutMs) {
    auto deadline = std::chrono::steady_clock::now() +
                    std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline) {
        if (pred()) return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

}  // namespace

TEST_CASE("controller applies a decision and trips watchdog to safe") {
    FakeWfbTx wfb;
    FakeEnc enc;

    Endpoints ep;
    ep.listenAddr = "127.0.0.1";
    ep.listenPort = 45801;                 // fixed test port
    ep.wfbCtlAddr = "127.0.0.1";
    ep.wfbCtlPort = wfb.port;
    ep.encHost    = "127.0.0.1";
    ep.encPort    = static_cast<uint16_t>(enc.port);
    ep.idrPort    = 0;                      // idr disabled (Task 16)
    ep.gsTunnelPort = 0;
    ep.osdMsgPath = "/tmp/fpvd_test_osd.msg";
    ep.osdUpdateIntervalMs = 1000;

    DlRuntimeConfig snap{};
    snap.healthTimeoutMs       = 300;       // small -> watchdog trips fast
    snap.minIdrIntervalMs      = 500;
    snap.applyStaggerMs        = 0;         // single-shot dispatch
    snap.applySubPaceMs        = 0;
    snap.interleavingSupported = false;
    snap.osdEnabled            = false;
    snap.osdDebugLatency       = false;
    snap.debug                 = false;
    snap.roiQp                 = RoiCurve{6000, 2000, -24, 3};
    snap.iface                 = "wlan-test-nonexistent";  // iw will fail, not hang
    snap.safe = SafeDefaults{
        /*mcs=*/1, /*k=*/8, /*n=*/12, /*depth=*/0,
        /*bandwidth=*/20, /*txPowerDbm=*/5, /*bitrateKbps=*/2000};

    DynamicLinkController c(ep);
    c.start(snap, /*generationId=*/0x1234);
    CHECK(c.status().running == true);

    // 1) Inject one decision. mcs=7, bw=40, k=4, n=6, bitrate=6000, fps=60.
    Decision d{};
    d.magic       = kWireMagic;
    d.version     = kWireVersion;
    d.sequence    = 100;
    d.timestampMs = 1;
    d.mcs         = 7;
    d.bandwidth   = 40;
    d.txPowerDbm  = 10;
    d.k           = 4;
    d.n           = 6;
    d.depth       = 0;
    d.bitrateKbps = 6000;
    d.fps         = 60;
    // Resend until phase-1 dispatch is observed: the controller binds its
    // listen socket asynchronously after start(), and a UDP datagram sent
    // before the bind is silently dropped. Resends carry the same sequence,
    // so the dedup drops all but the first accepted copy.
    bool gotFec = waitFor([&] {
        sendDecision(ep.listenPort, d);
        return wfb.sawFec(4, 6);
    }, 1000);
    CHECK(gotFec);
    CHECK(waitFor([&] { return wfb.sawRadio(7, 40); }, 1000));
    CHECK(waitFor([&] { return enc.sawContaining("video0.bitrate=6000"); }, 1000));

    // 2) Go silent past healthTimeoutMs -> watchdog trips -> safe-defaults push.
    CHECK(waitFor([&] {
        return wfb.sawFec(8, 12) &&
               wfb.sawRadio(1, 20) &&
               enc.sawContaining("video0.bitrate=2000");
    }, 2000));
    CHECK(waitFor([&] { return c.status().watchdogTripped == true; }, 1000));

    c.stop();
    CHECK(c.status().running == false);
}

// Exercises the staggered gap state machine: an UP-direction decision applies
// tx+radio immediately and the encoder bitrate only after the gap timer fires.
TEST_CASE("controller staggers an UP decision across the gap timer") {
    FakeWfbTx wfb;
    FakeEnc enc;

    Endpoints ep;
    ep.listenAddr = "127.0.0.1";
    ep.listenPort = 45802;                 // distinct fixed test port
    ep.wfbCtlAddr = "127.0.0.1";
    ep.wfbCtlPort = wfb.port;
    ep.encHost    = "127.0.0.1";
    ep.encPort    = static_cast<uint16_t>(enc.port);
    ep.idrPort    = 0;
    ep.gsTunnelPort = 0;
    ep.osdMsgPath = "/tmp/fpvd_test_osd2.msg";
    ep.osdUpdateIntervalMs = 1000;

    DlRuntimeConfig snap{};
    snap.healthTimeoutMs       = 5000;      // large -> watchdog won't trip mid-test
    snap.minIdrIntervalMs      = 500;
    snap.applyStaggerMs        = 120;       // non-zero -> staggered dispatch
    snap.applySubPaceMs        = 0;
    snap.interleavingSupported = false;
    snap.osdEnabled            = false;
    snap.osdDebugLatency       = false;
    snap.debug                 = false;
    snap.roiQp                 = RoiCurve{6000, 2000, -24, 3};
    snap.iface                 = "wlan-test-nonexistent";
    snap.safe = SafeDefaults{1, 8, 12, 0, 20, 5, 2000};

    DynamicLinkController c(ep);
    c.start(snap, 0x2222);

    auto mkDecision = [](uint32_t seq, uint8_t mcs, uint16_t br) {
        Decision d{};
        d.magic = kWireMagic; d.version = kWireVersion;
        d.sequence = seq; d.timestampMs = 1;
        d.mcs = mcs; d.bandwidth = 20; d.txPowerDbm = 10;
        d.k = 4; d.n = 6; d.depth = 0;
        d.bitrateKbps = br; d.fps = 60;
        return d;
    };

    // 1) First decision (seq 1) — first => Equal => single shot. bitrate 4000.
    Decision d1 = mkDecision(1, /*mcs=*/3, /*br=*/4000);
    bool got1 = waitFor([&] { sendDecision(ep.listenPort, d1); return wfb.sawRadio(3, 20); }, 1000);
    CHECK(got1);
    CHECK(waitFor([&] { return enc.sawContaining("video0.bitrate=4000"); }, 1000));

    // 2) Second decision (seq 2) — higher bitrate => UP. tx+radio apply now;
    //    encoder bitrate expands only after the ~120 ms gap timer.
    Decision d2 = mkDecision(2, /*mcs=*/5, /*br=*/6000);
    bool gotRadio = waitFor([&] { sendDecision(ep.listenPort, d2); return wfb.sawRadio(5, 20); }, 1000);
    CHECK(gotRadio);
    // Encoder bitrate=6000 must eventually arrive (phase 2 over the gap timer).
    CHECK(waitFor([&] { return enc.sawContaining("video0.bitrate=6000"); }, 1000));

    c.stop();
    CHECK(c.status().running == false);
}

// ---------------------------------------------------------------------------
// IDR token listener -> encoder requestIdr
// ---------------------------------------------------------------------------

namespace {

// Send a short UDP datagram to localhost:<port> (simulates a PixelPilot IDR token).
void sendIdrToken(uint16_t port) {
    int s = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(s >= 0);
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    dst.sin_port = htons(port);
    const char tok[] = "IDR";  // 3-byte token (mimics PixelPilot_rk)
    ssize_t w = ::sendto(s, tok, sizeof(tok) - 1, 0,
                         reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    CHECK(w == 3);
    ::close(s);
}

} // namespace (anonymous, IDR helpers)

TEST_CASE("controller IDR: datagram -> requestIdr; second immediate datagram is throttled") {
    FakeWfbTx wfb;
    FakeEnc enc;

    const uint16_t kIdrPort = 41223;

    Endpoints ep;
    ep.listenAddr        = "127.0.0.1";
    ep.listenPort        = 45804;           // fixed test port
    ep.wfbCtlAddr        = "127.0.0.1";
    ep.wfbCtlPort        = wfb.port;
    ep.encHost           = "127.0.0.1";
    ep.encPort           = static_cast<uint16_t>(enc.port);
    ep.idrAddr           = "127.0.0.1";
    ep.idrPort           = kIdrPort;        // IDR listener enabled
    ep.gsTunnelPort      = 0;
    ep.osdMsgPath        = "/tmp/fpvd_test_osd_idr.msg";
    ep.osdUpdateIntervalMs = 1000;

    DlRuntimeConfig snap{};
    snap.healthTimeoutMs       = 10000;     // won't trip during test
    snap.minIdrIntervalMs      = 500;       // 500 ms throttle window
    snap.applyStaggerMs        = 0;
    snap.applySubPaceMs        = 0;
    snap.interleavingSupported = false;
    snap.osdEnabled            = false;
    snap.osdDebugLatency       = false;
    snap.debug                 = false;
    snap.roiQp                 = RoiCurve{6000, 2000, -24, 3};
    snap.iface                 = "wlan-test-nonexistent";
    snap.safe = SafeDefaults{1, 8, 12, 0, 20, 5, 2000};

    DynamicLinkController c(ep);
    c.start(snap, /*generationId=*/0x5555);
    CHECK(c.status().running == true);

    // 1) Send an IDR token; assert the controller forwards it to the encoder.
    //    Re-send in a loop (controller binds idr socket asynchronously).
    bool gotIdr = waitFor([&] {
        sendIdrToken(kIdrPort);
        return enc.sawContaining("/request/idr");
    }, 2000);
    CHECK(gotIdr);

    // 2) Count how many /request/idr hits arrived so far.
    auto countIdr = [&] {
        std::lock_guard<std::mutex> lk(enc.mu);
        int n = 0;
        for (auto& h : enc.hits) if (h == "/request/idr") ++n;
        return n;
    };
    int firstCount = countIdr();
    CHECK(firstCount >= 1);

    // 3) Send a second token immediately — must be throttled (minIdrIntervalMs=500 ms).
    //    Wait 100 ms and assert no new /request/idr was added.
    sendIdrToken(kIdrPort);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    CHECK(countIdr() == firstCount);   // throttled: encoder count unchanged

    c.stop();
    CHECK(c.status().running == false);
}

// ---------------------------------------------------------------------------
// HELLO announce/keepalive + GS-tunnel socket
// ---------------------------------------------------------------------------

TEST_CASE("controller HELLO: sends DLHE to gs-tunnel sink and transitions to Keepalive on ACK") {
    FakeWfbTx wfb;
    FakeEnc enc;

    // GS-tunnel sink: bind an ephemeral UDP socket on loopback to receive DLHE
    // datagrams that the controller sends via the gs-tunnel socket.
    int sinkFd = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(sinkFd >= 0);
    sockaddr_in sinkAddr{};
    sinkAddr.sin_family = AF_INET;
    sinkAddr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    sinkAddr.sin_port = 0;  // let the kernel assign
    REQUIRE(::bind(sinkFd, reinterpret_cast<sockaddr*>(&sinkAddr), sizeof(sinkAddr)) == 0);
    socklen_t sinkLen = sizeof(sinkAddr);
    REQUIRE(::getsockname(sinkFd, reinterpret_cast<sockaddr*>(&sinkAddr), &sinkLen) == 0);
    uint16_t sinkPort = ntohs(sinkAddr.sin_port);
    // 500 ms recv timeout so the sink thread can poll.
    timeval tv{0, 500'000};
    ::setsockopt(sinkFd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));

    // Collect received DLHE packets in a thread.
    std::atomic<int> helloCount{0};
    std::atomic<bool> sinkStop{false};
    std::thread sinkThread([&] {
        while (!sinkStop.load()) {
            uint8_t buf[64];
            ssize_t n = ::recv(sinkFd, buf, sizeof(buf), 0);
            if (n <= 0) continue;
            if (peekKind(buf, static_cast<size_t>(n)) == PacketKind::Hello) {
                helloCount.fetch_add(1);
            }
        }
    });

    Endpoints ep;
    ep.listenAddr       = "127.0.0.1";
    ep.listenPort       = 45803;
    ep.wfbCtlAddr       = "127.0.0.1";
    ep.wfbCtlPort       = wfb.port;
    ep.encHost          = "127.0.0.1";
    ep.encPort          = static_cast<uint16_t>(enc.port);
    ep.idrPort          = 0;
    ep.gsTunnelAddr     = "127.0.0.1";
    ep.gsTunnelPort     = sinkPort;
    ep.osdMsgPath       = "/tmp/fpvd_test_osd3.msg";
    ep.osdUpdateIntervalMs = 1000;

    const uint32_t genId = 0xDEADBEEF;

    DlRuntimeConfig snap{};
    snap.healthTimeoutMs       = 10000;   // won't trip during test
    snap.minIdrIntervalMs      = 500;
    snap.applyStaggerMs        = 0;
    snap.applySubPaceMs        = 0;
    snap.interleavingSupported = false;
    snap.osdEnabled            = false;
    snap.osdDebugLatency       = false;
    snap.debug                 = false;
    snap.roiQp                 = RoiCurve{6000, 2000, -24, 3};
    snap.iface                 = "wlan-test-nonexistent";
    snap.safe = SafeDefaults{1, 8, 12, 0, 20, 5, 2000};
    snap.helloMtuBytes         = 1400;    // non-zero -> HelloSm enters ANNOUNCING
    snap.helloFps              = 60;

    DynamicLinkController c(ep);
    c.start(snap, genId);
    CHECK(c.status().running == true);

    // 1) Assert at least one DLHE arrives at the sink within 2 s.
    bool gotHello = waitFor([&] { return helloCount.load() >= 1; }, 2000);
    CHECK(gotHello);

    // 2) Send a matching HelloAck back into the controller's listen port.
    HelloAck ack{};
    ack.magic             = kHelloAckMagic;
    ack.version           = kWireVersion;
    ack.generationIdEcho  = genId;
    uint8_t ackBuf[64];
    size_t ackLen = encodeHelloAck(ack, ackBuf, sizeof(ackBuf));
    REQUIRE(ackLen > 0);

    int s2 = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(s2 >= 0);
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    dst.sin_port = htons(ep.listenPort);
    ssize_t w = ::sendto(s2, ackBuf, ackLen, 0,
                         reinterpret_cast<sockaddr*>(&dst), sizeof(dst));
    CHECK(w == static_cast<ssize_t>(ackLen));
    ::close(s2);

    // 3) Assert status().hello becomes Keepalive.
    bool gotKeepalive = waitFor([&] {
        return c.status().hello == HelloPub::Keepalive;
    }, 2000);
    CHECK(gotKeepalive);

    c.stop();
    CHECK(c.status().running == false);

    sinkStop.store(true);
    sinkThread.join();
    ::close(sinkFd);
}

// ---------------------------------------------------------------------------
// Hot config reload: setConfig applies new knobs without restart
// ---------------------------------------------------------------------------

TEST_CASE("setConfig hot-reloads knobs without restart") {
    FakeWfbTx wfb;
    FakeEnc enc;

    Endpoints ep;
    ep.listenAddr        = "127.0.0.1";
    ep.listenPort        = 45805;           // fixed test port
    ep.wfbCtlAddr        = "127.0.0.1";
    ep.wfbCtlPort        = wfb.port;
    ep.encHost           = "127.0.0.1";
    ep.encPort           = static_cast<uint16_t>(enc.port);
    ep.idrPort           = 0;
    ep.gsTunnelPort      = 0;
    ep.osdMsgPath        = "/tmp/fpvd_test_osd_hotreload.msg";
    ep.osdUpdateIntervalMs = 1000;

    // Start with a long watchdog timeout and safe.mcs=1
    DlRuntimeConfig snap{};
    snap.healthTimeoutMs       = 10000;     // long -> won't trip during setup
    snap.minIdrIntervalMs      = 500;
    snap.applyStaggerMs        = 0;
    snap.applySubPaceMs        = 0;
    snap.interleavingSupported = false;
    snap.osdEnabled            = false;
    snap.osdDebugLatency       = false;
    snap.debug                 = false;
    snap.roiQp                 = RoiCurve{6000, 2000, -24, 3};
    snap.iface                 = "wlan-test-nonexistent";
    snap.safe = SafeDefaults{
        /*mcs=*/1, /*k=*/8, /*n=*/12, /*depth=*/0,
        /*bandwidth=*/20, /*txPowerDbm=*/5, /*bitrateKbps=*/2000};

    DynamicLinkController c(ep);
    c.start(snap, /*generationId=*/0x6666);
    CHECK(c.status().running == true);

    // Inject one decision so the watchdog's everSeen_ becomes true. We must
    // send repeatedly until the controller's listen socket is bound.
    {
        Decision d{};
        d.magic = kWireMagic; d.version = kWireVersion;
        d.sequence = 1; d.timestampMs = 1;
        d.mcs = 3; d.bandwidth = 20; d.txPowerDbm = 10;
        d.k = 4; d.n = 6; d.depth = 0;
        d.bitrateKbps = 4000; d.fps = 60;
        bool gotFec = waitFor([&] {
            sendDecision(ep.listenPort, d);
            return wfb.sawFec(4, 6);
        }, 1000);
        CHECK(gotFec);  // decision was accepted; watchdog.everSeen_ = true
    }

    // Assert still running with original long timeout (watchdog won't trip yet)
    CHECK(c.status().running == true);

    // Hot-reload: shorten watchdog timeout to 400 ms and change safe.mcs=5
    DlRuntimeConfig snap2 = snap;
    snap2.healthTimeoutMs = 400;             // shorter -> will trip faster
    snap2.safe.mcs = 5;                      // new safe mcs

    c.setConfig(snap2);

    // Assert still running (no restart)
    CHECK(c.status().running == true);

    // After the reload, the watchdog should trip at ~400 ms (not 10000 ms),
    // and the safe push must use mcs=5 (not the original mcs=1).
    // The watchdog timeout is now 400 ms, tick = min(1000, 200) = 200 ms,
    // so it should trip within ~400 + 200 ms = ~600 ms from last decision.
    //
    // Wait up to 2 s for safe FEC (8/12) and safe radio (mcs=5, bw=20).
    CHECK(waitFor([&] {
        return wfb.sawFec(8, 12) && wfb.sawRadio(5, 20);
    }, 2000));

    // Confirm watchdog actually tripped in status
    CHECK(waitFor([&] { return c.status().watchdogTripped == true; }, 1000));

    // Confirm still running (setConfig did NOT restart the loop)
    CHECK(c.status().running == true);

    c.stop();
    CHECK(c.status().running == false);
}
