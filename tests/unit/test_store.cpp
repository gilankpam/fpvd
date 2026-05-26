#include "doctest.h"
#include "config/store.hpp"
#include <nlohmann/json.hpp>

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
