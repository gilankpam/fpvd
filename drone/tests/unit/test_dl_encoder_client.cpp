/* test_dl_encoder_client.cpp — port of test_dl_backend_enc.c + task spec tests.
 * Defaults: threshold=6000, anchor=2000, floor=-24, step=3. */
#include "doctest.h"
#include "dynlink/encoder_client.hpp"
#include "waybeam/client.hpp"
#include <httplib.h>
#include <mutex>
#include <thread>
using namespace fpvd::dynlink;

// ---------------------------------------------------------------------------
// Shared fake-server fixture
// ---------------------------------------------------------------------------
struct FakeSrv {
    httplib::Server srv;
    std::vector<std::string> hits;
    std::mutex mu;
    int port{0};
    std::thread th;

    FakeSrv() {
        srv.Get("/api/v1/set", [&](const httplib::Request& r, httplib::Response& res) {
            std::lock_guard<std::mutex> lk(mu);
            hits.push_back(r.target);
            res.set_content("ok", "text/plain");
        });
        port = srv.bind_to_any_port("127.0.0.1");
        th = std::thread([&] { srv.listen_after_bind(); });
        srv.wait_until_ready();
    }

    ~FakeSrv() {
        srv.stop();
        th.join();
    }

    // Returns request count
    size_t count() {
        std::lock_guard<std::mutex> lk(mu);
        return hits.size();
    }

    std::string last() {
        std::lock_guard<std::mutex> lk(mu);
        return hits.back();
    }
};

// ---------------------------------------------------------------------------
// Task spec test
// ---------------------------------------------------------------------------
TEST_CASE("EncoderClient applies bitrate+roiQp+fps, diffs") {
    FakeSrv f;

    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    // bitrate 6000 -> roiQp 0; fps 60
    CHECK(enc.apply(6000, 60) == 0);
    CHECK(f.last().find("video0.bitrate=6000") != std::string::npos);
    CHECK(f.last().find("fpv.roiQp=0") != std::string::npos);
    CHECK(f.last().find("video0.fps=60") != std::string::npos);

    size_t n = f.count();
    CHECK(enc.apply(6000, 60) == 0); // identical -> diffed out, no new hit
    CHECK(f.count() == n);
}

// ---------------------------------------------------------------------------
// Ported from test_dl_backend_enc.c
// ---------------------------------------------------------------------------
TEST_CASE("EncoderClient emits signed roiQp when starved") {
    FakeSrv f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    CHECK(enc.apply(4000, 60) == 0);
    // At 4000 kbps with defaults, roiQp = -12
    REQUIRE(f.count() > 0);
    CHECK(f.last().find("fpv.roiQp=-12") != std::string::npos);
    CHECK(f.last().find("video0.bitrate=4000") != std::string::npos);
}

TEST_CASE("EncoderClient emits roiQp=0 above threshold") {
    FakeSrv f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    CHECK(enc.apply(8000, 60) == 0);
    // Bug-fix assertion: at 8000 kbps roiQp = 0, still send fpv.roiQp=0
    REQUIRE(f.count() > 0);
    CHECK(f.last().find("fpv.roiQp=0") != std::string::npos);
}

TEST_CASE("EncoderClient deduplicates repeat apply") {
    FakeSrv f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    enc.apply(4000, 60);
    size_t n1 = f.count();
    enc.apply(4000, 60); // identical -> no HTTP
    CHECK(f.count() == n1);
}

TEST_CASE("EncoderClient different bitrate same roiQp is NOT deduped") {
    // Two different raw bitrates that compute to the same roiQp=-12 under
    // curve {threshold=6000, anchor=2000, floor=-24, step=3}:
    //   4000: span=4000, delta=2000, raw=(-24*2000)/4000=-12, q=-12
    //   3950: span=4000, delta=1950, raw=(-24*2050)/4000=-12 (truncated), q=-12
    // The dedup key includes raw bitrate, so BOTH must produce HTTP hits.
    FakeSrv f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    enc.apply(4000, 60);
    size_t n1 = f.count();
    REQUIRE(n1 > 0);
    // Different raw bitrate (3950) -> must NOT be deduped even though roiQp is same
    enc.apply(3950, 60);
    CHECK(f.count() == n1 + 1);
    CHECK(f.last().find("video0.bitrate=3950") != std::string::npos);
}

TEST_CASE("EncoderClient bitrate=0 is no-op sentinel") {
    FakeSrv f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    CHECK(enc.apply(0, 60) == 0);
    CHECK(f.count() == 0); // no HTTP request
}

TEST_CASE("EncoderClient applySafe uses compute formula") {
    FakeSrv f;
    // safe bitrate 2000 -> floor -24
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    CHECK(enc.applySafe(2000) == 0);
    REQUIRE(f.count() > 0);
    CHECK(f.last().find("video0.bitrate=2000") != std::string::npos);
    CHECK(f.last().find("fpv.roiQp=-24") != std::string::npos);
    // fps=0 means no video0.fps in query
    CHECK(f.last().find("video0.fps") == std::string::npos);
}

TEST_CASE("EncoderClient fps=0 omits video0.fps from query") {
    FakeSrv f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    // fps=0 -> video0.fps not emitted
    CHECK(enc.apply(6000, 0) == 0);
    REQUIRE(f.count() > 0);
    CHECK(f.last().find("video0.fps") == std::string::npos);
    CHECK(f.last().find("video0.bitrate=6000") != std::string::npos);
}

TEST_CASE("EncoderClient setRoiCurve hot reconcile") {
    FakeSrv f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    EncoderClient enc(wb, RoiCurve{6000, 2000, -24, 3});

    enc.apply(4000, 60); // roiQp=-12 cached
    size_t n1 = f.count();

    // Changing the curve changes computed roiQp -> new request goes out
    enc.setRoiCurve(RoiCurve{8000, 3000, -18, 2});
    // at 4000 kbps with new curve: span=5000, delta=1000, raw=-18*4000/5000=-14.4 -> -14
    // quantize step=2: -14/2=-7 *2 = -14 -> roiQp=-14
    CHECK(enc.apply(4000, 60) == 0);
    CHECK(f.count() > n1);
}
