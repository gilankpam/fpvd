#include "doctest.h"
#include "daemon.hpp"
#include "status.hpp"
#include <httplib.h>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <thread>
#include <vector>

namespace fs = std::filesystem;

TEST_CASE("daemon: bootstraps from defaults file when no overlay") {
    auto tmp = fs::temp_directory_path() / "fpvd-test-bootstrap";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");

    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(/*startProcesses=*/false);  // no real children in tests

    CHECK(d.effective().video.bitrate == 8192);
    CHECK(d.version() == 0);
    fs::remove_all(tmp);
}

TEST_CASE("daemon: PATCH then apply updates effective and overlay file") {
    auto tmp = fs::temp_directory_path() / "fpvd-test-apply";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");

    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    nlohmann::json patch = {{"video", {{"bitrate", 12345}}}};
    auto patchRes = d.patchPending(patch);
    REQUIRE(patchRes.ok);

    auto applyRes = d.apply(false);  // don't actually restart processes
    REQUIRE(applyRes.ok);
    CHECK(d.effective().video.bitrate == 12345);
    CHECK(d.version() == 1);

    // Overlay written; should contain only the diff.
    std::ifstream f(paths.overlayPath);
    nlohmann::json saved;
    f >> saved;
    CHECK(saved == nlohmann::json{{"video", {{"bitrate", 12345}}}});

    // /etc/waybeam.json rewritten.
    std::ifstream wf(paths.waybeamJsonPath);
    nlohmann::json wj;
    wf >> wj;
    CHECK(wj["video0"]["bitrate"] == 12345);

    fs::remove_all(tmp);
}

TEST_CASE("daemon: reset clears overlay") {
    auto tmp = fs::temp_directory_path() / "fpvd-test-reset";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");
    // Pre-existing overlay.
    std::ofstream(tmp / "etc" / "fpvd" / "config.json")
        << R"({"video":{"bitrate":11111}})";

    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);
    CHECK(d.effective().video.bitrate == 11111);

    d.reset();
    CHECK(d.pending().video.bitrate == 8192);
    CHECK_FALSE(fs::exists(paths.overlayPath));

    fs::remove_all(tmp);
}

TEST_CASE("daemon: dl_applier never in orchestrator (DynamicLinkController is in-process)") {
    auto tmp = fs::temp_directory_path() / "fpvd-daemon-dl-seed";
    fs::remove_all(tmp);
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
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // dl_applier is never a supervised process; the controller is in-process.
    auto names = d.orchestrator().names();
    CHECK(std::find(names.begin(), names.end(), "dl_applier") == names.end());

    // Enable + apply — dl_applier still NOT in orchestrator.
    auto pr = d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})"));
    CHECK(pr.ok);
    auto ar = d.apply(/*reallyRestart=*/false);
    CHECK(ar.ok);

    names = d.orchestrator().names();
    CHECK(std::find(names.begin(), names.end(), "dl_applier") == names.end());

    fs::remove_all(tmp);
}

TEST_CASE("daemon: dynamicLink in restarted-list when safe.* changes while DL is enabled") {
    auto tmp = fs::temp_directory_path() / "fpvd-daemon-dl-restarted";
    fs::remove_all(tmp);
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
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // Enable DL first and apply, so effective.dynamicLink.enabled = true.
    d.patchPending(nlohmann::json::parse(R"({"dynamicLink":{"enabled":true}})"));
    REQUIRE(d.apply(/*reallyRestart=*/false).ok);

    // Now change a safe.* knob: dynamicLink should appear in restarted.
    d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"safe":{"mcs":3}}})"));
    auto ar = d.apply(/*reallyRestart=*/false);
    REQUIRE(ar.ok);
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "dynamicLink")
          != ar.restarted.end());

    fs::remove_all(tmp);
}

TEST_CASE("daemon: dynamicLink NOT in restarted-list when safe.* changes while DL is disabled") {
    auto tmp = fs::temp_directory_path() / "fpvd-daemon-dl-not-restarted";
    fs::remove_all(tmp);
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
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // DL stays disabled. Change a safe knob: dynamicLink should NOT be reported.
    d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"safe":{"mcs":3}}})"));
    auto ar = d.apply(/*reallyRestart=*/false);
    REQUIRE(ar.ok);
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "dynamicLink")
          == ar.restarted.end());

    fs::remove_all(tmp);
}

TEST_CASE("daemon: dynamicLink IN restarted-list when DL is being disabled (transition)") {
    auto tmp = fs::temp_directory_path() / "fpvd-daemon-dl-disable-restart";
    fs::remove_all(tmp);
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
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // Enable DL and apply.
    d.patchPending(nlohmann::json::parse(R"({"dynamicLink":{"enabled":true}})"));
    REQUIRE(d.apply(/*reallyRestart=*/false).ok);

    // Now disable DL — the apply still touches dynamicLink (it stops).
    d.patchPending(nlohmann::json::parse(R"({"dynamicLink":{"enabled":false}})"));
    auto ar = d.apply(/*reallyRestart=*/false);
    REQUIRE(ar.ok);
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "dynamicLink")
          != ar.restarted.end());

    fs::remove_all(tmp);
}

TEST_CASE("daemon: apply reports beamforming when its config changes") {
    auto tmp = fs::temp_directory_path() / "fpvd-test-bf-apply";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");

    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    nlohmann::json patch = {{"link", {{"beamforming",
        {{"enabled", true}, {"remoteMac", "00:c0:ca:dd:ee:ff"}}}}}};
    auto pr = d.patchPending(patch);
    REQUIRE(pr.ok);

    auto ar = d.apply(false);
    REQUIRE(ar.ok);
    auto& r = ar.restarted;
    CHECK(std::find(r.begin(), r.end(), "beamforming") != r.end());
    fs::remove_all(tmp);
}

TEST_CASE("status: includes beamforming block") {
    auto tmp = fs::temp_directory_path() / "fpvd-test-bf-status";
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json");
    fpvd::DaemonPaths paths{
        (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string(),
        (tmp / "etc" / "fpvd" / "config.json").string(),
        "tests/fixtures/fake_radio_up_ok.sh",
        (tmp / "etc" / "waybeam.json").string()
    };
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    auto j = fpvd::buildStatus(d);
    REQUIRE(j.contains("beamforming"));
    CHECK(j["beamforming"]["state"] == "disabled");
    CHECK(j["beamforming"].contains("localMac"));
    fs::remove_all(tmp);
}

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

// ---- apply() controller routing (Task 20) ----------------------------------
//
// These exercise POST /apply with reallyRestart=true so the in-process
// DynamicLinkController is actually started/stopped/hot-reloaded. The endpoints
// are pinned to ephemeral test ports so the controller thread binds without
// colliding with prod ports; unreachable backends are tolerated (decisions fail
// soft). Assertions are on dynamicLinkStatus().running and orchestrator process
// identity (orchestrator().names()), not on real wfb_tx dispatch.

// Build a DaemonPaths whose dlEndpoints use harmless ephemeral test ports.
static fpvd::DaemonPaths makeRoutingPaths(const fs::path& tmp, uint16_t listenPort) {
    fs::remove_all(tmp);
    fs::create_directories(tmp / "rom" / "etc" / "fpvd");
    fs::create_directories(tmp / "etc" / "fpvd");
    fs::copy_file("tests/fixtures/defaults.json",
                  tmp / "rom" / "etc" / "fpvd" / "defaults.json",
                  fs::copy_options::overwrite_existing);

    fpvd::DaemonPaths paths{};
    paths.defaultsPath = (tmp / "rom" / "etc" / "fpvd" / "defaults.json").string();
    paths.overlayPath = (tmp / "etc" / "fpvd" / "config.json").string();
    paths.radioUpScript = "tests/fixtures/fake_radio_up_ok.sh";
    paths.waybeamJsonPath = (tmp / "etc" / "waybeam.json").string();
    paths.radioTuneScript = "tests/fixtures/fake_radio_tune.sh";
    // Ephemeral test endpoints: listen on a free high port, IDR disabled (0),
    // GS tunnel off (0), backends point at unbound ports (connect fails soft).
    paths.dlEndpoints.listenAddr = "127.0.0.1";
    paths.dlEndpoints.listenPort = listenPort;
    paths.dlEndpoints.wfbCtlAddr = "127.0.0.1";
    paths.dlEndpoints.wfbCtlPort = 0;
    paths.dlEndpoints.encHost = "127.0.0.1";
    paths.dlEndpoints.encPort = 0;
    paths.dlEndpoints.idrPort = 0;
    paths.dlEndpoints.gsTunnelPort = 0;
    paths.dlEndpoints.osdMsgPath = (tmp / "MSPOSD.msg").string();
    return paths;
}

TEST_CASE("apply: dynamicLink knob change hot-reloads, no orchestrator rebuild") {
    auto tmp = fs::temp_directory_path() / "fpvd-route-knob";
    auto paths = makeRoutingPaths(tmp, 46800);
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // Enable DL and apply for real so the controller is running.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})")).ok);
    REQUIRE(d.apply(/*reallyRestart=*/true).ok);
    REQUIRE(d.dynamicLinkStatus().running);

    // Record orchestrator process identity before the knob change.
    auto namesBefore = d.orchestrator().names();

    // Change a safe.* knob — a pure dynamicLink change.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"safe":{"mcs":3}}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);

    // Orchestrator NOT rebuilt: process set unchanged.
    auto namesAfter = d.orchestrator().names();
    CHECK(namesAfter == namesBefore);
    // Controller still running (hot setConfig, not stop/start).
    CHECK(d.dynamicLinkStatus().running);

    fs::remove_all(tmp);
}

TEST_CASE("apply: enabled false->true starts controller; true->false stops it") {
    auto tmp = fs::temp_directory_path() / "fpvd-route-toggle";
    auto paths = makeRoutingPaths(tmp, 46801);
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // Initially disabled, not running.
    CHECK_FALSE(d.dynamicLinkStatus().running);
    auto namesBefore = d.orchestrator().names();

    // false -> true: starts the controller, no full rebuild needed for this alone.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})")).ok);
    REQUIRE(d.apply(/*reallyRestart=*/true).ok);
    CHECK(d.dynamicLinkStatus().running);
    CHECK(d.orchestrator().names() == namesBefore);  // no rebuild

    // true -> false: stops the controller.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":false}})")).ok);
    REQUIRE(d.apply(/*reallyRestart=*/true).ok);
    CHECK_FALSE(d.dynamicLinkStatus().running);
    CHECK(d.orchestrator().names() == namesBefore);  // still no rebuild

    fs::remove_all(tmp);
}

TEST_CASE("apply: restart-class encoder change while enabled bounces only waybeam") {
    auto tmp = fs::temp_directory_path() / "fpvd-route-encoder";
    auto paths = makeRoutingPaths(tmp, 46802);
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // Enable DL and apply so the controller is running.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})")).ok);
    REQUIRE(d.apply(/*reallyRestart=*/true).ok);
    REQUIRE(d.dynamicLinkStatus().running);

    // Seed the orchestrator with fakes so the waybeam-only bounce is observable:
    // a real full rebuild would wipe/replace these; a hot apply preserves them,
    // and the restart-class path bounces only "waybeam".
    auto& orch = d.orchestrator();
    orch.add({"wfb_video_tx", {"/bin/sh", "-c", "sleep 30"}, {},
              fpvd::RestartPolicy::Always, {}});
    orch.add({"waybeam", {"/bin/sh", "-c", "sleep 30"}, {},
              fpvd::RestartPolicy::Always, {}});
    orch.startAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto namesBefore = orch.names();
    pid_t wfbPid = orch.get("wfb_video_tx")->pid();
    pid_t wbPid  = orch.get("waybeam")->pid();
    REQUIRE(wfbPid > 0);
    REQUIRE(wbPid > 0);

    // A resolution change is a RESTART-class encoder field, NOT dynamic-link-
    // locked and NOT a dynamicLink input -- it bounces only waybeam, no rebuild.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"video":{"resolution":"1280x720"}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "encoder")
          != ar.restarted.end());
    CHECK(orch.names() == namesBefore);                    // no full rebuild
    CHECK(orch.get("waybeam")->pid() != wbPid);            // waybeam bounced
    CHECK(orch.get("wfb_video_tx")->pid() == wfbPid);      // wfb untouched
    CHECK(d.dynamicLinkStatus().running);                  // controller untouched

    orch.stopAll();
    fs::remove_all(tmp);
}

// Fake waybeam HTTP server for the encoder-apply path.
namespace {
struct FakeWbDaemon {
    httplib::Server srv;
    std::vector<std::string> hits;
    std::mutex mu;
    int port{0};
    std::thread th;
    FakeWbDaemon() {
        srv.Get("/api/v1/set", [&](const httplib::Request& r, httplib::Response& res) {
            std::lock_guard<std::mutex> lk(mu);
            hits.push_back(r.target);
            res.set_content("ok", "text/plain");
        });
        port = srv.bind_to_any_port("127.0.0.1");
        th = std::thread([&] { srv.listen_after_bind(); });
        srv.wait_until_ready();
    }
    ~FakeWbDaemon() { srv.stop(); th.join(); }
    size_t count() { std::lock_guard<std::mutex> lk(mu); return hits.size(); }
    std::string last() { std::lock_guard<std::mutex> lk(mu); return hits.back(); }
};
} // namespace

TEST_CASE("apply: LIVE encoder change pushes /api/v1/set, no rebuild") {
    FakeWbDaemon wb;
    auto tmp = fs::temp_directory_path() / "fpvd-enc-live";
    auto paths = makeRoutingPaths(tmp, 46810);
    paths.dlEndpoints.encPort = static_cast<uint16_t>(wb.port);  // point at fake waybeam
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    // Seed a fake "waybeam" so we can prove the LIVE path does NOT bounce it.
    auto& orch = d.orchestrator();
    orch.add({"waybeam", {"/bin/sh", "-c", "sleep 30"}, {},
              fpvd::RestartPolicy::Always, {}});
    orch.startAll();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto namesBefore = orch.names();
    pid_t wbPid = orch.get("waybeam")->pid();
    REQUIRE(wbPid > 0);

    // DL disabled -> bitrate is fpvd-owned and LIVE.
    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"video":{"bitrate":4096}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);

    REQUIRE(wb.count() == 1);
    CHECK(wb.last().find("video0.bitrate=4096") != std::string::npos);
    CHECK(orch.names() == namesBefore);                 // no rebuild
    CHECK(orch.get("waybeam")->pid() == wbPid);         // LIVE push does not bounce waybeam
    CHECK(d.effective().video.bitrate == 4096);

    orch.stopAll();
    fs::remove_all(tmp);
}

TEST_CASE("apply: RESTART encoder change rewrites file, issues no /set") {
    FakeWbDaemon wb;
    auto tmp = fs::temp_directory_path() / "fpvd-enc-restart";
    auto paths = makeRoutingPaths(tmp, 46811);
    paths.dlEndpoints.encPort = static_cast<uint16_t>(wb.port);
    fpvd::Daemon d(paths);
    d.bootstrap(false);

    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"video":{"resolution":"1280x720"}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    REQUIRE(ar.ok);

    // Restart-class path uses the file + waybeam bounce, never /api/v1/set.
    CHECK(wb.count() == 0);
    std::ifstream wf(paths.waybeamJsonPath);
    nlohmann::json wj; wf >> wj;
    CHECK(wj["video0"]["size"] == "1280x720");
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "encoder")
          != ar.restarted.end());

    fs::remove_all(tmp);
}

TEST_CASE("apply: failed /api/v1/set fails the apply with effective unchanged") {
    auto tmp = fs::temp_directory_path() / "fpvd-enc-fail";
    auto paths = makeRoutingPaths(tmp, 46812);
    // encPort 0 -> connection refused; the LIVE push must fail.
    paths.dlEndpoints.encPort = 0;
    fpvd::Daemon d(paths);
    d.bootstrap(false);
    int before = d.effective().video.bitrate;

    REQUIRE(d.patchPending(nlohmann::json::parse(
        R"({"video":{"bitrate":4096}})")).ok);
    auto ar = d.apply(/*reallyRestart=*/true);
    CHECK_FALSE(ar.ok);
    CHECK(d.effective().video.bitrate == before);   // not committed
    CHECK(d.version() == 0);                         // no version bump

    fs::remove_all(tmp);
}
