#include "doctest.h"
#include "daemon.hpp"
#include "http/handlers.hpp"
#include "http/server.hpp"
#include <filesystem>
#include <fstream>
#include <httplib.h>

namespace fs = std::filesystem;

static std::unique_ptr<fpvd::Daemon> makeTestDaemon(const fs::path& tmp) {
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json",
                  fs::copy_options::overwrite_existing);
    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    auto d = std::make_unique<fpvd::Daemon>(paths);
    d->bootstrap(false);
    return d;
}

TEST_CASE("handlers: GET /config returns effective") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-get";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18081);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18081");
    auto r = c.Get("/config");
    REQUIRE(r);
    CHECK(r->status == 200);
    auto j = nlohmann::json::parse(r->body);
    CHECK(j["video"]["bitrate"] == 8192);
    srv.stop();
    fs::remove_all(tmp);
}

TEST_CASE("handlers: GET /defaults returns defaults") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-defaults";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18082);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18082");
    auto r = c.Get("/defaults");
    REQUIRE(r);
    CHECK(r->status == 200);
    auto j = nlohmann::json::parse(r->body);
    CHECK(j["video"]["bitrate"] == 8192);
    srv.stop();
    fs::remove_all(tmp);
}

TEST_CASE("handlers: GET /healthz returns 200") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-health";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18083);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18083");
    auto r = c.Get("/healthz");
    REQUIRE(r);
    CHECK(r->status == 200);
    srv.stop();
    fs::remove_all(tmp);
}

TEST_CASE("handlers: PATCH /config stages a change") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-patch";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18084);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18084");
    auto r = c.Patch("/config",
        R"({"video":{"bitrate":12345}})", "application/json");
    REQUIRE(r);
    CHECK(r->status == 200);
    CHECK(d->pending().video.bitrate == 12345);
    CHECK(d->effective().video.bitrate == 8192);  // not applied
    srv.stop();
    fs::remove_all(tmp);
}

TEST_CASE("handlers: PATCH /config rejects malformed JSON") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-badjson";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18085);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18085");
    auto r = c.Patch("/config", "not json", "application/json");
    REQUIRE(r);
    CHECK(r->status == 400);
    auto j = nlohmann::json::parse(r->body);
    CHECK(j["error"] == "bad_json");
    srv.stop();
    fs::remove_all(tmp);
}

TEST_CASE("handlers: PATCH /config rejects validation errors") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-validate";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18086);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18086");
    auto r = c.Patch("/config",
        R"({"link":{"width":80}})", "application/json");
    REQUIRE(r);
    CHECK(r->status == 400);
    auto j = nlohmann::json::parse(r->body);
    CHECK(j["error"] == "validation");
    srv.stop();
    fs::remove_all(tmp);
}
