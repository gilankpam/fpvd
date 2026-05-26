#include "doctest.h"
#include "config/diff.hpp"

TEST_CASE("diff: changing link.channel flags Radio") {
    fpvd::Config a{}, b{};
    b.link.channel = 165;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK(r.radio);
    CHECK_FALSE(r.encoder);
    CHECK_FALSE(r.telemetry);
    CHECK(r.servicesAffected.empty());
}

TEST_CASE("diff: video changes flag Encoder") {
    fpvd::Config a{}, b{};
    b.video.bitrate = 10000;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK_FALSE(r.radio);
    CHECK(r.encoder);
}

TEST_CASE("diff: telemetry changes flag Telemetry") {
    fpvd::Config a{}, b{};
    b.telemetry.osdFps = 30;
    auto r = fpvd::diffSubsystems(a, b);
    CHECK(r.telemetry);
}

TEST_CASE("diff: service add/remove/modify") {
    fpvd::Config a{}, b{};
    fpvd::Service s{}; s.exec = "/bin/true";
    b.services["x"] = s;
    auto r1 = fpvd::diffSubsystems(a, b);
    CHECK(r1.servicesAffected.count("x"));

    fpvd::Config c = b;
    c.services["x"].args = {"--flag"};
    auto r2 = fpvd::diffSubsystems(b, c);
    CHECK(r2.servicesAffected.count("x"));

    fpvd::Config d = b;
    d.services.erase("x");
    auto r3 = fpvd::diffSubsystems(b, d);
    CHECK(r3.servicesAffected.count("x"));
}
