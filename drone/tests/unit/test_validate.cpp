#include "doctest.h"
#include "config/validate.hpp"

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

TEST_CASE("validate: dynamicLink.safe.mcs in [0,7]") {
    Config c{}; c.dynamicLink.safe.mcs = 8;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.mcs");
}

TEST_CASE("validate: dynamicLink.safe k<n and both in [1,32]") {
    Config c{}; c.dynamicLink.safe.k = 12; c.dynamicLink.safe.n = 8;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.fec");

    Config c2{}; c2.dynamicLink.safe.k = 0;
    auto errs2 = validate(c2);
    REQUIRE(errs2.size() == 1);
    CHECK(errs2[0].path == "dynamicLink.safe.fec");

    Config c3{}; c3.dynamicLink.safe.n = 33;
    auto errs3 = validate(c3);
    REQUIRE(errs3.size() == 1);
    CHECK(errs3[0].path == "dynamicLink.safe.fec");
}

TEST_CASE("validate: link.txPowerDbm in [-10,30]") {
    Config c{}; c.link.txPowerDbm = 31;
    auto errs = validate(c);
    REQUIRE(errs.size() >= 1);
    bool found = false;
    for (auto& e : errs) if (e.path == "link.txPowerDbm") found = true;
    CHECK(found);

    Config c2{}; c2.link.txPowerDbm = -11;
    auto errs2 = validate(c2);
    bool found2 = false;
    for (auto& e : errs2) if (e.path == "link.txPowerDbm") found2 = true;
    CHECK(found2);

    Config c3{}; c3.link.txPowerDbm = 20;   // in range
    auto errs3 = validate(c3);
    for (auto& e : errs3) CHECK(e.path != "link.txPowerDbm");
}

TEST_CASE("validate: dynamicLink.safe.bitrateKbps > 0") {
    Config c{}; c.dynamicLink.safe.bitrateKbps = 0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.safe.bitrateKbps");
}

TEST_CASE("validate: dynamicLink.healthTimeoutMs >= 1000") {
    Config c{}; c.dynamicLink.healthTimeoutMs = 500;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.healthTimeoutMs");
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

TEST_CASE("validate: dynamicLink.compute maxBitrateKbps > minBitrateKbps") {
    Config c{}; c.dynamicLink.compute.maxBitrateKbps = c.dynamicLink.compute.minBitrateKbps;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.compute");
}

TEST_CASE("validate: dynamicLink.compute kMax >= kMin") {
    Config c{}; c.dynamicLink.compute.kMax = c.dynamicLink.compute.kMin - 1;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.compute.k");
}

TEST_CASE("validate: dynamicLink.compute.baseRedundancyRatio > 0") {
    Config c{}; c.dynamicLink.compute.baseRedundancyRatio = 0.0;
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "dynamicLink.compute.baseRedundancyRatio");
}

TEST_CASE("validate: video.resilience accepts every known preset") {
    for (const char* p : {"off","rescue","quality","sprint","racing",
                          "endurance","patrol","rally","range","fpv"}) {
        Config c{}; c.video.resilience = p;
        CHECK_MESSAGE(validate(c).empty(), "unexpected error for preset: " << p);
    }
}

TEST_CASE("validate: video.resilience rejects an unknown preset") {
    Config c{}; c.video.resilience = "turbo";
    auto errs = validate(c);
    REQUIRE(errs.size() == 1);
    CHECK(errs[0].path == "video.resilience");
}

TEST_CASE("validate: link.fec swfec rules") {
    auto hasErr = [](const std::vector<fpvd::ValidationError>& errs,
                     const std::string& path) {
        for (auto& e : errs) if (e.path == path) return true;
        return false;
    };
    fpvd::Config c{};

    SUBCASE("bad mode rejected") {
        c.link.fec.mode = "raptor";
        CHECK(hasErr(fpvd::validate(c), "link.fec.mode"));
    }
    SUBCASE("overheadPct range") {
        c.link.fec.overheadPct = 256;
        CHECK(hasErr(fpvd::validate(c), "link.fec.overheadPct"));
    }
    SUBCASE("deadlineMs range — uint8 wire cap") {
        c.link.fec.deadlineMs = 0;
        CHECK(hasErr(fpvd::validate(c), "link.fec.deadlineMs"));
        c = {};
        c.link.fec.deadlineMs = 256;
        CHECK(hasErr(fpvd::validate(c), "link.fec.deadlineMs"));
    }
    SUBCASE("safe swfec ranges") {
        c.dynamicLink.safe.overheadPct = -1;
        CHECK(hasErr(fpvd::validate(c), "dynamicLink.safe.overheadPct"));
        c.dynamicLink.safe = {};
        c.dynamicLink.safe.deadlineMs = 300;
        CHECK(hasErr(fpvd::validate(c), "dynamicLink.safe.deadlineMs"));
    }
    SUBCASE("valid swfec config passes") {
        c.link.fec.mode = "swfec";
        c.link.fec.overheadPct = 50;
        c.link.fec.deadlineMs = 30;
        CHECK(fpvd::validate(c).empty());
    }
}
