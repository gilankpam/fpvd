#include "doctest.h"
#include "config/validate.hpp"

using fpvd::Config;
using fpvd::validate;

TEST_CASE("validate: default config is valid") {
    Config c{};
    auto errs = validate(c);
    CHECK(errs.empty());
}

TEST_CASE("validate: width must be 20 or 40") {
    Config c{}; c.link.width = 80;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "link.width");
}

TEST_CASE("validate: fec.k must be less than fec.n") {
    Config c{}; c.link.fec.k = 12; c.link.fec.n = 8;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "link.fec");
}

TEST_CASE("validate: video.fps must be in (0, 120]") {
    Config c{}; c.video.fps = 0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "video.fps");

    Config c2{}; c2.video.fps = 200;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "video.fps");
}

TEST_CASE("validate: video.resolution must parse as WxH") {
    Config c{}; c.video.resolution = "1080";
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "video.resolution");
}

TEST_CASE("validate: video.codec must be h264 or h265") {
    Config c{}; c.video.codec = "av1";
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "video.codec");
}

TEST_CASE("validate: image.rotate must be 0/90/180/270") {
    Config c{}; c.image.rotate = 45;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "image.rotate");
}

TEST_CASE("validate: telemetry.router must be msposd|mavfwd|none") {
    Config c{}; c.telemetry.router = "garbage";
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "telemetry.router");
}

TEST_CASE("validate: service.startAfter cycles are rejected") {
    Config c{};
    fpvd::Service a{}; a.exec = "/bin/true"; a.startAfter = {"b"};
    fpvd::Service b{}; b.exec = "/bin/true"; b.startAfter = {"a"};
    c.services["a"] = a; c.services["b"] = b;
    auto errs = validate(c);
    REQUIRE(errs.size() >= 1);
    CHECK(errs[0].path.rfind("services", 0) == 0);
}

TEST_CASE("validate: service.restart must be always|on-failure|never") {
    Config c{};
    fpvd::Service s{}; s.exec = "/bin/true"; s.restart = "sometimes";
    c.services["x"] = s;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "services.x.restart");
}
