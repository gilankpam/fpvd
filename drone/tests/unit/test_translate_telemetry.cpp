#include "doctest.h"
#include "config/schema.hpp"
#include "translate/telemetry.hpp"
#include <algorithm>

using fpvd::Config;

TEST_CASE("translate.telemetry: msposd argv from defaults") {
    Config c{};
    auto a = fpvd::telemetryArgs(c);
    REQUIRE(!a.empty());
    CHECK(a[0] == "/usr/bin/msposd");
    auto contains = [&](const std::string& s){
        return std::find(a.begin(), a.end(), s) != a.end();
    };
    CHECK(contains("-b")); CHECK(contains("115200"));
    CHECK(contains("-r")); CHECK(contains("20"));
    CHECK(contains("-m")); CHECK(contains("/dev/ttyS2"));
    CHECK(contains("-o")); CHECK(contains("127.0.0.1:14551"));
    CHECK(contains("-z")); CHECK(contains("1920x1080"));
}

TEST_CASE("translate.telemetry: msposd -z tracks video.resolution") {
    Config c{}; c.video.resolution = "1280x720";
    auto a = fpvd::telemetryArgs(c);
    // -z must carry the *current* resolution, so a rebuilt msposd sizes its OSD
    // canvas to the new video size after a resolution change.
    auto it = std::find(a.begin(), a.end(), "-z");
    REQUIRE(it != a.end());
    REQUIRE(std::next(it) != a.end());
    CHECK(*std::next(it) == "1280x720");
}

TEST_CASE("translate.telemetry: mavfwd argv") {
    Config c{}; c.telemetry.router = "mavfwd";
    auto a = fpvd::telemetryArgs(c);
    CHECK(a[0] == "/usr/bin/mavfwd");
    auto contains = [&](const std::string& s){
        return std::find(a.begin(), a.end(), s) != a.end();
    };
    CHECK(contains("-i")); CHECK(contains("127.0.0.1:14550"));
    CHECK(contains("-o")); CHECK(contains("127.0.0.1:14551"));
}

TEST_CASE("translate.telemetry: none returns empty argv") {
    Config c{}; c.telemetry.router = "none";
    auto a = fpvd::telemetryArgs(c);
    CHECK(a.empty());
}
