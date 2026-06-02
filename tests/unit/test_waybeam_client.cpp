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
