#include "doctest.h"
#include "daemon.hpp"
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
