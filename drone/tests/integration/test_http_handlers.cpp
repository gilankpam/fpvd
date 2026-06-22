#include "daemon.hpp"
#include "doctest.h"
#include "http/handlers.hpp"
#include "http/server.hpp"
#include <filesystem>
#include <fstream>
#include <httplib.h>

namespace fs = std::filesystem;

static std::unique_ptr<fpvd::Daemon> makeTestDaemon(const fs::path& tmp) {
    fs::create_directories(tmp / "etc" / "fpvd");
    fpvd::DaemonPaths paths{(tmp / "etc" / "fpvd" / "config.json").string(),
                            "tests/fixtures/fake_radio_up_ok.sh",
                            (tmp / "etc" / "waybeam.json").string()};
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
    auto r = c.Patch("/config", R"({"video":{"bitrate":12345}})", "application/json");
    REQUIRE(r);
    CHECK(r->status == 200);
    CHECK(d->pending().video.bitrate == 12345);
    CHECK(d->effective().video.bitrate == 8192); // not applied
    srv.stop();
    fs::remove_all(tmp);
}

TEST_CASE("handlers: video.sensorBin is exposed over PATCH + GET /config") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-sensorbin";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, false);
    srv.listenInBackground("127.0.0.1", 18099);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18099");

    // GET exposes the field (default empty).
    auto g0 = c.Get("/config");
    REQUIRE(g0);
    CHECK(nlohmann::json::parse(g0->body)["video"]["sensorBin"] == "");

    // PATCH stages it into pending.
    auto r = c.Patch("/config", R"({"video":{"sensorBin":"2x2"}})", "application/json");
    REQUIRE(r);
    CHECK(r->status == 200);
    CHECK(d->pending().video.sensorBin == "2x2");
    CHECK(nlohmann::json::parse(r->body)["video"]["sensorBin"] == "2x2");

    // GET ?pending=true reflects the staged value.
    auto g1 = c.Get("/config?pending=true");
    REQUIRE(g1);
    CHECK(nlohmann::json::parse(g1->body)["video"]["sensorBin"] == "2x2");

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
    auto r = c.Patch("/config", R"({"link":{"width":80}})", "application/json");
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
    REQUIRE(r);
    CHECK(r->status == 200);
    auto j = nlohmann::json::parse(r->body);
    CHECK(j["applied"] == true);
    CHECK(j["version"] == 1);
    CHECK(d->effective().video.bitrate == 9999);
    srv.stop();
    fs::remove_all(tmp);
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
    REQUIRE(r);
    CHECK(r->status == 200);
    CHECK_FALSE(fs::exists(tmp / "etc" / "fpvd" / "config.json"));
    srv.stop();
    fs::remove_all(tmp);
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
    auto r1 = c.Patch("/config", R"({"dynamicLink":{"enabled":true}})", "application/json");
    REQUIRE(r1);
    CHECK(r1->status == 200);
    auto r2 = c.Post("/apply", "", "application/json");
    REQUIRE(r2);
    CHECK(r2->status == 200);

    // Now try to write a locked field.
    auto r3 = c.Patch("/config", R"({"link":{"mcs":5}})", "application/json");
    REQUIRE(r3);
    CHECK(r3->status == 400);
    auto j = nlohmann::json::parse(r3->body);
    CHECK(j["error"] == "dynamic_link_locked");
    REQUIRE(j["details"]["locked"].is_array());
    CHECK(j["details"]["locked"].size() == 1);
    CHECK(j["details"]["locked"][0] == "link.mcs");

    // Pending should be unchanged.
    CHECK(d->pending().link.mcs == 2);
    srv.stop();
    fs::remove_all(tmp);
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
    c.Patch("/config", R"({"dynamicLink":{"enabled":true}})", "application/json");
    c.Post("/apply", "", "application/json");

    // Single PATCH disables DL and writes link.mcs.
    auto r = c.Patch("/config", R"({"dynamicLink":{"enabled":false},"link":{"mcs":5}})",
                     "application/json");
    REQUIRE(r);
    CHECK(r->status == 200);
    CHECK(d->pending().dynamicLink.enabled == false);
    CHECK(d->pending().link.mcs == 5);
    srv.stop();
    fs::remove_all(tmp);
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
    REQUIRE(r);
    CHECK(r->status == 200);
    auto j = nlohmann::json::parse(r->body);
    CHECK(j.contains("uptime"));
    CHECK(j.contains("version"));
    CHECK(j.contains("processes"));
    CHECK(j["processes"].is_array());
    srv.stop();
    fs::remove_all(tmp);
}

TEST_CASE("handlers: enabling dynamicLink reports dynamicLink in apply restarted; not in "
          "/status.processes") {
    auto tmp = fs::temp_directory_path() / "fpvd-handlers-dl-e2e";
    fs::remove_all(tmp);
    auto d = makeTestDaemon(tmp);
    fpvd::HttpServer srv;
    fpvd::registerHandlers(srv, *d, /*reallyRestart=*/false);
    srv.listenInBackground("127.0.0.1", 18096);
    srv.waitUntilReady(std::chrono::seconds(2));

    httplib::Client c("http://127.0.0.1:18096");

    // Before enabling: the controller is never surfaced as a /status process.
    auto s0 = c.Get("/status");
    REQUIRE(s0);
    CHECK(s0->status == 200);
    auto j0 = nlohmann::json::parse(s0->body);
    bool found0 = false;
    for (auto& p : j0["processes"])
        if (p["name"] == "dl_applier" || p["name"] == "dynamicLink")
            found0 = true;
    CHECK_FALSE(found0);

    // PATCH + apply — "dynamicLink" appears in restarted list (config reported).
    c.Patch("/config", R"({"dynamicLink":{"enabled":true}})", "application/json");
    auto ap = c.Post("/apply", "", "application/json");
    REQUIRE(ap);
    CHECK(ap->status == 200);
    auto japp = nlohmann::json::parse(ap->body);
    CHECK(japp["applied"] == true);
    bool restartedDl = false;
    for (auto& r : japp["restarted"])
        if (r == "dynamicLink")
            restartedDl = true;
    CHECK(restartedDl);

    // After: the controller is in-process, NOT in /status.processes.
    auto s1 = c.Get("/status");
    auto j1 = nlohmann::json::parse(s1->body);
    bool found1 = false;
    for (auto& p : j1["processes"])
        if (p["name"] == "dl_applier" || p["name"] == "dynamicLink")
            found1 = true;
    CHECK_FALSE(found1);

    // Flip back off + apply — still not in processes.
    c.Patch("/config", R"({"dynamicLink":{"enabled":false}})", "application/json");
    c.Post("/apply", "", "application/json");
    auto s2 = c.Get("/status");
    auto j2 = nlohmann::json::parse(s2->body);
    bool found2 = false;
    for (auto& p : j2["processes"])
        if (p["name"] == "dl_applier" || p["name"] == "dynamicLink")
            found2 = true;
    CHECK_FALSE(found2);

    srv.stop();
    fs::remove_all(tmp);
}

// Helper: create a Daemon with ephemeral DL endpoints so the real
// DynamicLinkController can be started (unreachable backends tolerated).
static std::unique_ptr<fpvd::Daemon> makeTestDaemonWithDlEndpoints(const fs::path& tmp,
                                                                   uint16_t listenPort) {
    fs::remove_all(tmp);
    fs::create_directories(tmp / "etc" / "fpvd");

    fpvd::DaemonPaths paths{};
    paths.configPath = (tmp / "etc" / "fpvd" / "config.json").string();
    paths.radioUpScript = "tests/fixtures/fake_radio_up_ok.sh";
    paths.waybeamJsonPath = (tmp / "etc" / "waybeam.json").string();
    paths.dlEndpoints.listenAddr = "127.0.0.1";
    paths.dlEndpoints.listenPort = listenPort;
    paths.dlEndpoints.wfbCtlAddr = "127.0.0.1";
    paths.dlEndpoints.wfbCtlPort = 0;
    paths.dlEndpoints.encHost = "127.0.0.1";
    paths.dlEndpoints.encPort = 0;
    paths.idrPort = 0;
    paths.dlEndpoints.gsTunnelPort = 0;
    paths.osdMsgPath = (tmp / "MSPOSD.msg").string();

    auto d = std::make_unique<fpvd::Daemon>(paths);
    d->bootstrap(false);
    return d;
}

TEST_CASE("status exposes dynamicLink block; no dl_applier process row") {
    // ---- Case 1: disabled (default) -----------------------------------------
    {
        auto tmp = fs::temp_directory_path() / "fpvd-status-dl-disabled";
        fs::remove_all(tmp);
        auto d = makeTestDaemon(tmp);
        fpvd::HttpServer srv;
        fpvd::registerHandlers(srv, *d, false);
        srv.listenInBackground("127.0.0.1", 18097);
        srv.waitUntilReady(std::chrono::seconds(2));

        httplib::Client c("http://127.0.0.1:18097");
        auto r = c.Get("/status");
        REQUIRE(r);
        CHECK(r->status == 200);
        auto j = nlohmann::json::parse(r->body);

        // dynamicLink block must exist
        REQUIRE(j.contains("dynamicLink"));
        auto dl = j["dynamicLink"];
        CHECK(dl["enabled"] == false);
        CHECK(dl["running"] == false);
        // disabled block must NOT have watchdogTripped / lastDecisionAgeMs / hello
        CHECK_FALSE(dl.contains("watchdogTripped"));
        CHECK_FALSE(dl.contains("lastDecisionAgeMs"));
        CHECK_FALSE(dl.contains("hello"));

        // processes[] must NOT contain dl_applier or dynamicLink
        bool found = false;
        for (auto& p : j["processes"])
            if (p["name"] == "dl_applier" || p["name"] == "dynamicLink")
                found = true;
        CHECK_FALSE(found);

        srv.stop();
        fs::remove_all(tmp);
    }

    // ---- Case 2: enabled + running ------------------------------------------
    {
        auto tmp = fs::temp_directory_path() / "fpvd-status-dl-enabled";
        auto d = makeTestDaemonWithDlEndpoints(tmp, 46810);
        fpvd::HttpServer srv;
        fpvd::registerHandlers(srv, *d, /*reallyRestart=*/true);
        srv.listenInBackground("127.0.0.1", 18098);
        srv.waitUntilReady(std::chrono::seconds(2));

        httplib::Client c("http://127.0.0.1:18098");

        // Enable DL and apply so the controller actually starts.
        auto pr = c.Patch("/config", R"({"dynamicLink":{"enabled":true}})", "application/json");
        REQUIRE(pr);
        CHECK(pr->status == 200);
        auto ap = c.Post("/apply", "", "application/json");
        REQUIRE(ap);
        CHECK(ap->status == 200);

        // Fetch status and verify dynamicLink block.
        auto r = c.Get("/status");
        REQUIRE(r);
        CHECK(r->status == 200);
        auto j = nlohmann::json::parse(r->body);

        REQUIRE(j.contains("dynamicLink"));
        auto dl = j["dynamicLink"];
        CHECK(dl["enabled"] == true);
        CHECK(dl["running"] == true);
        // All 5 fields must be present when enabled
        CHECK(dl.contains("watchdogTripped"));
        CHECK(dl.contains("lastDecisionAgeMs"));
        CHECK(dl.contains("hello"));
        // hello must be a valid string
        std::string helloVal = dl["hello"];
        CHECK((helloVal == "disabled" || helloVal == "announcing" || helloVal == "keepalive"));

        // processes[] must NOT contain dl_applier or dynamicLink
        bool found = false;
        for (auto& p : j["processes"])
            if (p["name"] == "dl_applier" || p["name"] == "dynamicLink")
                found = true;
        CHECK_FALSE(found);

        srv.stop();
        fs::remove_all(tmp);
    }
}
