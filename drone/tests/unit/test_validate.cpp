#include "doctest.h"
#include "config/validate.hpp"
#include <algorithm>

using fpvd::Config;
using fpvd::validate;

TEST_CASE("validate: default config is valid") {
    Config c{};
    auto errs = validate(c);
    CHECK(errs.empty());
}

TEST_CASE("validate: width must be 10, 20, or 40") {
    Config c{}; c.link.width = 80;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "link.width");

    Config ok{}; ok.link.width = 10;
    CHECK(validate(ok).empty());
}

TEST_CASE("validate: fec.k must be less than fec.n") {
    Config c{}; c.link.fec.k = 12; c.link.fec.n = 8;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "link.fec");
}

TEST_CASE("link.txpowerCurve override must be 8 entries within 0..30 dBm") {
    fpvd::Config c{};
    c.link.txpowerCurve = std::vector<int>{29,28,25,23,19,19,19,19};
    CHECK(fpvd::validate(c).empty());

    c.link.txpowerCurve = std::vector<int>{29,28,25};            // too short
    CHECK(!fpvd::validate(c).empty());

    c.link.txpowerCurve = std::vector<int>{0,0,0,0,0,0,0,99};    // out of range
    auto errs = fpvd::validate(c);
    CHECK(std::any_of(errs.begin(), errs.end(),
        [](const fpvd::ValidationError& e){ return e.path == "link.txpowerCurve"; }));
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

TEST_CASE("validate: video.codec must be h265 (hardware is H.265-only)") {
    {
        Config c{}; c.video.codec = "h265";   // the only accepted value
        auto errs = validate(c);
        for (auto& e : errs) CHECK(e.path != "video.codec");
    }
    {
        Config c{}; c.video.codec = "h264";   // previously valid, now rejected
        auto errs = validate(c);
        bool found = false;
        for (auto& e : errs) if (e.path == "video.codec") found = true;
        CHECK(found);
    }
    {
        Config c{}; c.video.codec = "av1";
        auto errs = validate(c);
        bool found = false;
        for (auto& e : errs) if (e.path == "video.codec") found = true;
        CHECK(found);
    }
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

TEST_CASE("validate: dynamicLink.failsafe.mcs in [0,7]") {
    Config c{}; c.dynamicLink.failsafe.mcs = 8;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.failsafe.mcs");
}

TEST_CASE("validate: dynamicLink.failsafe k<n and both in [1,32]") {
    Config c{}; c.dynamicLink.failsafe.k = 12; c.dynamicLink.failsafe.n = 8;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.failsafe.fec");

    Config c2{}; c2.dynamicLink.failsafe.k = 0;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "dynamicLink.failsafe.fec");

    Config c3{}; c3.dynamicLink.failsafe.n = 33;
    auto errs3 = validate(c3);
    REQUIRE(errs3.size() == 1);
    CHECK(errs3[0].path == "dynamicLink.failsafe.fec");
}

TEST_CASE("validate: dynamicLink.failsafe.depth in [1,8]") {
    Config c{}; c.dynamicLink.failsafe.depth = 0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.failsafe.depth");

    Config c2{}; c2.dynamicLink.failsafe.depth = 9;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "dynamicLink.failsafe.depth");
}

TEST_CASE("validate: dynamicLink.failsafe.bandwidth must be 10, 20, or 40") {
    Config c{}; c.dynamicLink.failsafe.bandwidth = 80;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.failsafe.bandwidth");

    Config ok{}; ok.dynamicLink.failsafe.bandwidth = 10;
    CHECK(validate(ok).empty());
}

TEST_CASE("validate: dynamicLink.failsafe.bitrateKbps > 0") {
    Config c{}; c.dynamicLink.failsafe.bitrateKbps = 0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.failsafe.bitrateKbps");
}

TEST_CASE("validate: dynamicLink.healthTimeoutMs >= 1000") {
    Config c{}; c.dynamicLink.healthTimeoutMs = 500;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.healthTimeoutMs");
}

TEST_CASE("validate: dynamicLink.minIdrIntervalMs >= 16") {
    Config c{}; c.dynamicLink.minIdrIntervalMs = 10;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.minIdrIntervalMs");
}

TEST_CASE("validate: dynamicLink.applyStaggerMs in [0,500]") {
    Config c{}; c.dynamicLink.applyStaggerMs = 501;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.applyStaggerMs");
}

TEST_CASE("validate: dynamicLink.applySubPaceMs in [0,50]") {
    Config c{}; c.dynamicLink.applySubPaceMs = 51;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.applySubPaceMs");
}

TEST_CASE("validate: dynamicLink.roiQp threshold > lowAnchor > 0") {
    Config c{}; c.dynamicLink.roiQp.thresholdKbps = 1000;
    c.dynamicLink.roiQp.lowAnchorKbps = 2000;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.roiQp");

    Config c2{}; c2.dynamicLink.roiQp.lowAnchorKbps = 0;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "dynamicLink.roiQp");
}

TEST_CASE("validate: dynamicLink.roiQp.floor must be <= 0") {
    Config c{}; c.dynamicLink.roiQp.floor = 1;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.roiQp.floor");
}

TEST_CASE("validate: dynamicLink.roiQp.step >= 1") {
    Config c{}; c.dynamicLink.roiQp.step = 0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.roiQp.step");
}

TEST_CASE("link.txpower is validated as dBm 0..30") {
    fpvd::Config c{};
    c.link.txpower = 20;                       // valid dBm
    CHECK(fpvd::validate(c).empty());
    c.link.txpower = 31;                       // above radio max
    auto errs = fpvd::validate(c);
    CHECK(std::any_of(errs.begin(), errs.end(),
        [](const fpvd::ValidationError& e){ return e.path == "link.txpower"; }));
}

TEST_CASE("validate: beamforming off ignores stale fields") {
    Config c{};
    c.link.beamforming.enabled = false;
    c.link.beamforming.remoteMac = "";   // empty is fine when disabled
    c.link.stbc = true;                  // irrelevant when disabled
    CHECK(validate(c).empty());
}

TEST_CASE("validate: beamforming on requires stbc off") {
    Config c{};
    c.link.beamforming.enabled = true;
    c.link.beamforming.remoteMac = "00:c0:ca:aa:bb:cc";
    c.link.stbc = true;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "link.beamforming");
}

TEST_CASE("validate: beamforming on requires a valid remoteMac") {
    Config c{};
    c.link.beamforming.enabled = true;
    c.link.stbc = false;                 // stbc defaults true; clear it to isolate the remoteMac rule

    c.link.beamforming.remoteMac = "";
    REQUIRE(validate(c).size() == 1);
    CHECK(validate(c)[0].path == "link.beamforming.remoteMac");

    c.link.beamforming.remoteMac = "not-a-mac";
    REQUIRE(validate(c).size() == 1);
    CHECK(validate(c)[0].path == "link.beamforming.remoteMac");

    c.link.beamforming.remoteMac = "00:c0:ca:aa:bb:cc";
    CHECK(validate(c).empty());
}

TEST_CASE("validate: beamforming ackTimeout and intervalMs ranges") {
    Config c{};
    c.link.beamforming.enabled = true;
    c.link.stbc = false;                 // stbc defaults true; clear it to isolate the range rules
    c.link.beamforming.remoteMac = "00:c0:ca:aa:bb:cc";

    c.link.beamforming.ackTimeout = 32;     // below 33
    REQUIRE(validate(c).size() == 1);
    CHECK(validate(c)[0].path == "link.beamforming.ackTimeout");

    c.link.beamforming.ackTimeout = 255;
    c.link.beamforming.intervalMs = 0;      // below 1
    REQUIRE(validate(c).size() == 1);
    CHECK(validate(c)[0].path == "link.beamforming.intervalMs");
}
