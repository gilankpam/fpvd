#include "doctest.h"
#include "config/store.hpp"
#include <nlohmann/json.hpp>
#include <filesystem>
#include <fstream>
namespace fs = std::filesystem;

TEST_CASE("store: load defaults from file") {
    auto cfg = fpvd::loadDefaults("tests/fixtures/defaults.json");
    CHECK(cfg.link.channel == 161);
    CHECK(cfg.video.bitrate == 8192);
    CHECK(cfg.telemetry.router == "msposd");
}

TEST_CASE("store: loadDefaults throws on missing file") {
    CHECK_THROWS_AS(fpvd::loadDefaults("/no/such/file.json"),
                    fpvd::StoreError);
}

TEST_CASE("store: loadDefaults throws on malformed JSON") {
    CHECK_THROWS_AS(fpvd::loadDefaults("tests/fixtures/malformed.json"),
                    fpvd::StoreError);
}

TEST_CASE("store: sparse overlay merges into defaults") {
    auto cfg = fpvd::loadEffective(
        "tests/fixtures/defaults.json",
        "tests/fixtures/overlay_bitrate_only.json");
    // Overlay only changed video.bitrate.
    CHECK(cfg.video.bitrate == 12000);
    // Everything else from defaults.
    CHECK(cfg.video.fps == 60);
    CHECK(cfg.link.channel == 161);
}

TEST_CASE("store: missing overlay returns defaults unchanged") {
    auto cfg = fpvd::loadEffective(
        "tests/fixtures/defaults.json",
        "/no/such/overlay.json");
    CHECK(cfg.video.bitrate == 8192);
}

TEST_CASE("store: deepMergeJson merges nested objects") {
    using nlohmann::json;
    json base = {{"a", {{"x", 1}, {"y", 2}}}, {"b", 3}};
    json over = {{"a", {{"y", 20}}}};
    json m = fpvd::deepMergeJson(base, over);
    CHECK(m["a"]["x"] == 1);
    CHECK(m["a"]["y"] == 20);
    CHECK(m["b"] == 3);
}

TEST_CASE("store: deepMergeJson replaces arrays wholesale") {
    using nlohmann::json;
    json base = {{"arr", {1, 2, 3}}};
    json over = {{"arr", {9}}};
    json m = fpvd::deepMergeJson(base, over);
    CHECK(m["arr"] == json::array({9}));
}

TEST_CASE("store: computeOverlay returns only diff") {
    using nlohmann::json;
    json defaults = {{"a", 1}, {"b", {{"x", 1}, {"y", 2}}}};
    json effective = {{"a", 1}, {"b", {{"x", 1}, {"y", 99}}}};
    auto ov = fpvd::computeOverlay(defaults, effective);
    CHECK(ov == json{{"b", {{"y", 99}}}});
}

TEST_CASE("store: computeOverlay handles arrays as wholesale replace") {
    using nlohmann::json;
    json defaults = {{"arr", {1, 2, 3}}};
    json effective = {{"arr", {1, 2, 4}}};
    auto ov = fpvd::computeOverlay(defaults, effective);
    CHECK(ov == json{{"arr", {1, 2, 4}}});
}

TEST_CASE("store: atomicWriteJson writes file and survives") {
    auto tmp = fs::temp_directory_path() / "fpvd_atomic_test.json";
    fs::remove(tmp);
    nlohmann::json j = {{"k", "v"}};
    fpvd::atomicWriteJson(tmp.string(), j);
    std::ifstream in(tmp);
    nlohmann::json round;
    in >> round;
    CHECK(round == j);
    CHECK_FALSE(fs::exists(tmp.string() + ".tmp"));
    fs::remove(tmp);
}

TEST_CASE("store: defaults file carries dynamicLink section") {
    auto c = fpvd::loadEffective("tests/fixtures/defaults.json",
                                  "/no/such/path");
    CHECK(c.dynamicLink.enabled == false);
    CHECK(c.dynamicLink.failsafe.mcs == 1);
    CHECK(c.dynamicLink.roiQp.thresholdKbps == 6000);
}

TEST_CASE("loadEffective migrates a legacy dynamicLink.safe overlay key to failsafe") {
    auto dir = std::filesystem::temp_directory_path() / "fpvd-safe-migrate";
    std::filesystem::create_directories(dir);
    auto defaults = dir / "defaults.json";
    auto overlay  = dir / "config.json";
    { std::ofstream f(defaults); f << R"({"dynamicLink":{"failsafe":{"mcs":1}}})"; }
    { std::ofstream f(overlay);  f << R"({"dynamicLink":{"safe":{"mcs":4}}})"; }
    auto c = fpvd::loadEffective(defaults.string(), overlay.string());
    CHECK(c.dynamicLink.failsafe.mcs == 4);   // legacy key honored, not dropped
}
