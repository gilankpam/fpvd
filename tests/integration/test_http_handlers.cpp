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

TEST_CASE("handlers: POST /apply commits pending and increments version") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-apply";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18091);
    srv.waitUntilReady(std::chrono::seconds(2));
    httplib::Client c("http://127.0.0.1:18091");
    c.Patch("/config", R"({"video":{"bitrate":9999}})", "application/json");
    auto r = c.Post("/apply", "", "application/json");
    REQUIRE(r); CHECK(r->status == 200);
    auto j = nlohmann::json::parse(r->body);
    CHECK(j["applied"] == true);
    CHECK(j["version"] == 1);
    CHECK(d->effective().video.bitrate == 9999);
    srv.stop(); fs::remove_all(tmp);
}

TEST_CASE("handlers: POST /reset removes overlay") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-reset";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    // Stage and apply something so an overlay file exists.
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18092);
    srv.waitUntilReady(std::chrono::seconds(2));
    httplib::Client c("http://127.0.0.1:18092");
    c.Patch("/config", R"({"video":{"bitrate":5555}})", "application/json");
    c.Post("/apply", "", "application/json");
    REQUIRE(fs::exists(tmp / "etc" / "fpvd" / "config.json"));
    auto r = c.Post("/reset", "", "application/json");
    REQUIRE(r); CHECK(r->status == 200);
    CHECK_FALSE(fs::exists(tmp / "etc" / "fpvd" / "config.json"));
    srv.stop(); fs::remove_all(tmp);
}

TEST_CASE("handlers: PATCH /config rejects locked field when DL enabled") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-lock";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18094);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18094");
    // First enable DL in pending and apply, so effective.dynamicLink.enabled = true.
    auto r1 = c.Patch("/config",
        R"({"dynamicLink":{"enabled":true}})", "application/json");
    REQUIRE(r1); CHECK(r1->status == 200);
    auto r2 = c.Post("/apply", "", "application/json");
    REQUIRE(r2); CHECK(r2->status == 200);

    // Now try to write a locked field.
    auto r3 = c.Patch("/config",
        R"({"link":{"mcs":5}})", "application/json");
    REQUIRE(r3);
    CHECK(r3->status == 400);
    auto j = nlohmann::json::parse(r3->body);
    CHECK(j["error"] == "dynamic_link_locked");
    REQUIRE(j["details"]["locked"].is_array());
    CHECK(j["details"]["locked"].size() == 1);
    CHECK(j["details"]["locked"][0] == "link.mcs");

    // Pending should be unchanged.
    CHECK(d->pending().link.mcs == 2);
    srv.stop(); fs::remove_all(tmp);
}

TEST_CASE("handlers: PATCH that disables DL and writes locked key is allowed") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-lock-unlock";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18095);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18095");
    // Enable + apply.
    c.Patch("/config",
        R"({"dynamicLink":{"enabled":true}})", "application/json");
    c.Post("/apply", "", "application/json");

    // Single PATCH disables DL and writes link.mcs.
    auto r = c.Patch("/config",
        R"({"dynamicLink":{"enabled":false},"link":{"mcs":5}})",
        "application/json");
    REQUIRE(r); CHECK(r->status == 200);
    CHECK(d->pending().dynamicLink.enabled == false);
    CHECK(d->pending().link.mcs == 5);
    srv.stop(); fs::remove_all(tmp);
}

TEST_CASE("handlers: GET /status returns expected shape") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-status";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18093);
    srv.waitUntilReady(std::chrono::seconds(2));
    httplib::Client c("http://127.0.0.1:18093");
    auto r = c.Get("/status");
    REQUIRE(r); CHECK(r->status == 200);
    auto j = nlohmann::json::parse(r->body);
    CHECK(j.contains("uptime"));
    CHECK(j.contains("version"));
    CHECK(j.contains("processes"));
    CHECK(j["processes"].is_array());
    srv.stop(); fs::remove_all(tmp);
}

TEST_CASE("handlers: enabling dynamicLink surfaces dl_applier in /status") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-dl-e2e";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, /*reallyRestart=*/false);
    srv.listenInBackground("127.0.0.1", 18096);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18096");

    // Before enabling: dl_applier not in /status.processes.
    auto s0 = c.Get("/status");
    REQUIRE(s0); CHECK(s0->status == 200);
    auto j0 = nlohmann::json::parse(s0->body);
    bool found0 = false;
    for (auto& p : j0["processes"]) if (p["name"] == "dl_applier") found0 = true;
    CHECK_FALSE(found0);

    // PATCH + apply.
    c.Patch("/config", R"({"dynamicLink":{"enabled":true}})",
            "application/json");
    auto ap = c.Post("/apply", "", "application/json");
    REQUIRE(ap); CHECK(ap->status == 200);
    auto japp = nlohmann::json::parse(ap->body);
    CHECK(japp["applied"] == true);
    bool restartedDl = false;
    for (auto& r : japp["restarted"])
        if (r == "dl_applier") restartedDl = true;
    CHECK(restartedDl);

    // After: dl_applier visible.
    auto s1 = c.Get("/status");
    auto j1 = nlohmann::json::parse(s1->body);
    bool found1 = false;
    for (auto& p : j1["processes"]) if (p["name"] == "dl_applier") found1 = true;
    CHECK(found1);

    // Flip back off + apply — dl_applier disappears.
    c.Patch("/config", R"({"dynamicLink":{"enabled":false}})",
            "application/json");
    c.Post("/apply", "", "application/json");
    auto s2 = c.Get("/status");
    auto j2 = nlohmann::json::parse(s2->body);
    bool found2 = false;
    for (auto& p : j2["processes"]) if (p["name"] == "dl_applier") found2 = true;
    CHECK_FALSE(found2);

    srv.stop(); fs::remove_all(tmp);
}
