/* test_idr_relay.cpp — unit tests for the always-on idr::IdrRelay. */
#include "doctest.h"
#include "idr/relay.hpp"
#include "waybeam/client.hpp"

#include <arpa/inet.h>
#include <chrono>
#include <httplib.h>
#include <mutex>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <unistd.h>

using namespace fpvd::idr;

// ---------------------------------------------------------------------------
// Fake waybeam server: records /request/idr hits.
// ---------------------------------------------------------------------------
struct FakeEnc {
    httplib::Server srv;
    std::vector<std::string> hits;
    std::mutex mu;
    int port{0};
    std::thread th;

    FakeEnc() {
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
    ~FakeEnc() {
        srv.stop();
        th.join();
    }

    size_t count() {
        std::lock_guard<std::mutex> lk(mu);
        return hits.size();
    }
};

static void sendDatagram(uint16_t port) {
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(s >= 0);
    struct sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);
    const char msg[] = "abc\n";
    sendto(s, msg, sizeof(msg) - 1, 0, reinterpret_cast<struct sockaddr*>(&dst), sizeof(dst));
    close(s);
}

// Spin up to ~1s waiting for a predicate.
template <typename F> static bool waitFor(F pred) {
    for (int i = 0; i < 200; ++i) {
        if (pred())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    return pred();
}

TEST_CASE("IdrRelay: requestIdr throttles within the window") {
    FakeEnc f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    IdrRelay relay(wb, "127.0.0.1", /*port=*/0, /*minIntervalMs=*/500); // no socket

    CHECK(relay.requestIdr(1000) == 0); // first sent
    CHECK(relay.requestIdr(1100) == 1); // throttled (<500ms)
    CHECK(relay.requestIdr(1700) == 0); // window elapsed -> sent
}

TEST_CASE("IdrRelay: count increments per logical request, not per throttled attempt") {
    FakeEnc f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    IdrRelay relay(wb, "127.0.0.1", /*port=*/0, /*minIntervalMs=*/500); // no socket

    CHECK(relay.count() == 0);
    CHECK(relay.requestIdr(1000) == 0); // sent -> counts
    CHECK(relay.count() == 1);
    CHECK(relay.requestIdr(1100) == 1); // throttled -> does NOT count
    CHECK(relay.count() == 1);
    CHECK(relay.requestIdr(1700) == 0); // window elapsed -> sent -> counts
    CHECK(relay.count() == 2);
}

TEST_CASE("IdrRelay: throttle arms on any attempt including failure") {
    // Dead server: bind a free port then stop before listen so connects refuse.
    httplib::Server dead;
    int dead_port = dead.bind_to_any_port("127.0.0.1");
    dead.stop();

    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(dead_port));
    IdrRelay relay(wb, "127.0.0.1", /*port=*/0, /*minIntervalMs=*/500);

    (void)relay.requestIdr(1000);       // may fail (-1) but arms throttle + counts
    CHECK(relay.count() == 1);          // a failed send is still a logical request
    CHECK(relay.requestIdr(1100) == 1); // throttled regardless of prior result
    CHECK(relay.count() == 1);          // throttled attempt does not count
}

TEST_CASE("IdrRelay: always-on socket path forwards to the encoder") {
    FakeEnc f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    const uint16_t PORT = 51124;
    IdrRelay relay(wb, "127.0.0.1", PORT, /*minIntervalMs=*/500);
    relay.start(); // no dynamic-link controller involved — IDR is independent

    sendDatagram(PORT);

    CHECK(waitFor([&] { return f.count() >= 1; }));
    CHECK(relay.count() >= 1); // logical-request counter (drives the OSD "I")

    relay.stop();
}

TEST_CASE("IdrRelay: port 0 disables (no thread, count stays 0)") {
    FakeEnc f;
    fpvd::WaybeamClient wb("127.0.0.1", static_cast<uint16_t>(f.port));
    IdrRelay relay(wb, "127.0.0.1", /*port=*/0, /*minIntervalMs=*/500);
    relay.start(); // no-op when disabled
    CHECK(relay.count() == 0);
    relay.stop(); // no-op, must not hang/crash
    CHECK(relay.count() == 0);
}
