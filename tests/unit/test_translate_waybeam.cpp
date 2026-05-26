#include "doctest.h"
#include "config/schema.hpp"
#include "translate/waybeam.hpp"
#include <fstream>
#include <sstream>

TEST_CASE("translate.waybeam: default config produces expected JSON") {
    fpvd::Config c{};
    auto out = fpvd::toWaybeamJson(c);

    // Spot checks on translated fields.
    CHECK(out["video0"]["codec"] == "h265");
    CHECK(out["video0"]["fps"] == 60);
    CHECK(out["video0"]["size"] == "1920x1080");
    CHECK(out["video0"]["bitrate"] == 8192);
    CHECK(out["video0"]["rcMode"] == "cbr");
    CHECK(out["video0"]["qpDelta"] == -4);
    CHECK(out["image"]["mirror"] == false);
    CHECK(out["fpv"]["roiEnabled"] == true);
    CHECK(out["fpv"]["roiCenter"] == 0.4);
    CHECK(out["fpv"]["roiSteps"] == 2);
    CHECK(out["outgoing"]["enabled"] == true);
    CHECK(out["outgoing"]["server"] == "unix://venc_wfb");
    CHECK(out["outgoing"]["streamMode"] == "rtp");
    CHECK(out["snapshot"]["enabled"] == true);
    CHECK(out["snapshot"]["quality"] == 80);
    CHECK(out["record"]["enabled"] == false);
}

TEST_CASE("translate.waybeam: changes propagate") {
    fpvd::Config c{};
    c.video.codec = "h264";
    c.video.fps = 30;
    c.video.bitrate = 4000;
    c.video.resolution = "1280x720";
    c.image.rotate = 180;
    c.snapshot.enabled = false;

    auto out = fpvd::toWaybeamJson(c);
    CHECK(out["video0"]["codec"] == "h264");
    CHECK(out["video0"]["fps"] == 30);
    CHECK(out["video0"]["bitrate"] == 4000);
    CHECK(out["video0"]["size"] == "1280x720");
    CHECK(out["image"]["rotate"] == 180);
    CHECK(out["snapshot"]["enabled"] == false);
}
