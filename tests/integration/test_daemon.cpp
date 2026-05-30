#include "doctest.h"
#include "daemon.hpp"
#include "status.hpp"
#include <algorithm>
#include <filesystem>
#include <fstream>

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

TEST_CASE("daemon: enabling dynamicLink seeds dl_applier in orchestrator") {
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

    // dl_applier is NOT present when disabled.
    auto names = d.orchestrator().names();
    CHECK(std::find(names.begin(), names.end(), "dl_applier") == names.end());

    // Enable + apply (without really restarting; we only need the orch
    // re-seeded).
    auto pr = d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"enabled":true}})"));
    CHECK(pr.ok);
    auto ar = d.apply(/*reallyRestart=*/false);
    CHECK(ar.ok);

    // Now dl_applier should be in the orchestrator.
    names = d.orchestrator().names();
    CHECK(std::find(names.begin(), names.end(), "dl_applier") != names.end());

    fs::remove_all(tmp);
}

TEST_CASE("daemon: dl_applier in restarted-list when safe.* changes while DL is enabled") {
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

    // Now change a safe.* knob: dl_applier should appear in restarted.
    d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"safe":{"mcs":3}}})"));
    auto ar = d.apply(/*reallyRestart=*/false);
    REQUIRE(ar.ok);
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "dl_applier")
          != ar.restarted.end());

    fs::remove_all(tmp);
}

TEST_CASE("daemon: dl_applier NOT in restarted-list when safe.* changes while DL is disabled") {
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

    // DL stays disabled. Change a safe knob: dl_applier should NOT be reported.
    d.patchPending(nlohmann::json::parse(
        R"({"dynamicLink":{"safe":{"mcs":3}}})"));
    auto ar = d.apply(/*reallyRestart=*/false);
    REQUIRE(ar.ok);
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "dl_applier")
          == ar.restarted.end());

    fs::remove_all(tmp);
}

TEST_CASE("daemon: dl_applier IN restarted-list when DL is being disabled (transition)") {
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

    // Now disable DL — the apply still touches dl_applier (it stops).
    d.patchPending(nlohmann::json::parse(R"({"dynamicLink":{"enabled":false}})"));
    auto ar = d.apply(/*reallyRestart=*/false);
    REQUIRE(ar.ok);
    CHECK(std::find(ar.restarted.begin(), ar.restarted.end(), "dl_applier")
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
