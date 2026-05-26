#include "doctest.h"
#include "config/store.hpp"

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
