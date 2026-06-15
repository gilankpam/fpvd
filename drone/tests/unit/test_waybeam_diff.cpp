#include "doctest.h"
#include "translate/waybeam.hpp"

using namespace fpvd;

TEST_CASE("waybeamConfigDiff: live bitrate change when DL disabled") {
    Config a{}, b{};
    b.video.bitrate = 4096;
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/false);
    CHECK(d.live.at("video0.bitrate") == "4096");
    CHECK(d.restart.empty());
}

TEST_CASE("waybeamConfigDiff: restart fields are bucketed separately") {
    Config a{}, b{};
    b.video.resolution = "1280x720";
    b.image.flip = true;
    b.recording.enabled = true;
    auto d = waybeamConfigDiff(a, b, false);
    CHECK(d.restart.at("video0.size") == "1280x720");
    CHECK(d.restart.at("image.flip") == "true");
    CHECK(d.restart.at("record.enabled") == "true");
    CHECK(d.live.empty());
}

TEST_CASE("waybeamConfigDiff: codec is never emitted") {
    Config a{}, b{};
    b.video.codec = "h264";   // (invalid, but must never reach waybeam)
    auto d = waybeamConfigDiff(a, b, false);
    CHECK(d.live.empty());
    CHECK(d.restart.empty());
}

TEST_CASE("waybeamConfigDiff: DL-owned fields excluded when DL enabled") {
    Config a{}, b{};
    b.video.bitrate = 4096;     // DL-owned
    b.video.qpDelta = -8;       // DL-owned
    b.video.roi.qp = -10;       // DL-owned
    b.video.fps = 30;           // DL-owned
    b.video.gopSize = 2.0;      // NOT DL-owned
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/true);
    CHECK(d.live.find("video0.bitrate") == d.live.end());
    CHECK(d.live.find("video0.qp_delta") == d.live.end());
    CHECK(d.live.find("fpv.roi_qp") == d.live.end());
    CHECK(d.live.find("video0.fps") == d.live.end());
    CHECK(d.live.at("video0.gop_size") == "2");   // gop still pushed
}

TEST_CASE("waybeamConfigDiff: DL-owned fields included when DL disabled") {
    Config a{}, b{};
    b.video.bitrate = 4096;
    b.video.fps = 30;
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/false);
    CHECK(d.live.at("video0.bitrate") == "4096");
    CHECK(d.live.at("video0.fps") == "30");
}

TEST_CASE("waybeamConfigDiff: no change yields empty diff") {
    Config a{}, b{};
    auto d = waybeamConfigDiff(a, b, false);
    CHECK(d.live.empty());
    CHECK(d.restart.empty());
}

TEST_CASE("waybeamConfigDiff: RESTART field still emitted when DL enabled") {
    Config a{}, b{};
    b.video.resolution = "1280x720";   // RESTART, not DL-owned
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/true);
    CHECK(d.restart.at("video0.size") == "1280x720");
}

TEST_CASE("waybeamConfigDiff: sensorBin is a RESTART field, not DL-owned") {
    Config a{}, b{};
    b.video.sensorBin = "2x2";
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/false);
    CHECK(d.restart.at("isp.sensor_bin") == "2x2");
    CHECK(d.live.empty());

    // Not dynamic-link-owned: still emitted while DL is enabled.
    auto d2 = waybeamConfigDiff(a, b, /*dlEnabled=*/true);
    CHECK(d2.restart.at("isp.sensor_bin") == "2x2");
}

TEST_CASE("waybeamConfigDiff: resilience is a RESTART field, not live") {
    Config a{}, b{};
    b.video.resilience = "fpv";
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/false);
    CHECK(d.restart.at("video0.resilience") == "fpv");
    CHECK(d.live.empty());

    // Not dynamic-link-owned: still emitted while DL is enabled.
    auto d2 = waybeamConfigDiff(a, b, /*dlEnabled=*/true);
    CHECK(d2.restart.at("video0.resilience") == "fpv");
}

TEST_CASE("waybeamConfigDiff: fractional roi.center formats with a decimal") {
    Config a{}, b{};
    a.video.roi.center = 0.0;   // default center is 0.4, so explicitly set a to 0.0 for a genuine change
    b.video.roi.center = 0.4;
    auto d = waybeamConfigDiff(a, b, /*dlEnabled=*/false);
    CHECK(d.live.at("fpv.roi_center") == "0.4");
}
