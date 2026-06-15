#include "doctest.h"
#include "config/schema.hpp"
#include "translate/waybeam.hpp"
#include <fstream>
#include <sstream>

TEST_CASE("translate.waybeam: default config produces expected JSON") {
    fpvd::Config c{};
    auto out = fpvd::toWaybeamJson(c);

    // Spot checks on translated fields.
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
    CHECK(out["record"]["enabled"] == false);
}

TEST_CASE("translate.waybeam: changes propagate") {
    fpvd::Config c{};
    c.video.fps = 30;
    c.video.bitrate = 4000;
    c.video.resolution = "1280x720";
    c.image.rotate = 180;

    auto out = fpvd::toWaybeamJson(c);
    CHECK(out["video0"]["fps"] == 30);
    CHECK(out["video0"]["bitrate"] == 4000);
    CHECK(out["video0"]["size"] == "1280x720");
    CHECK(out["image"]["rotate"] == 180);
}

TEST_CASE("translate.waybeam: video.sensorBin maps to isp.sensorBin") {
    fpvd::Config c{};
    // Default is empty.
    CHECK(fpvd::toWaybeamJson(c)["isp"]["sensorBin"] == "");

    c.video.sensorBin = "2x2";
    CHECK(fpvd::toWaybeamJson(c)["isp"]["sensorBin"] == "2x2");
}

TEST_CASE("translate.waybeam: video.resilience maps to video0.resilience") {
    fpvd::Config c{};
    // Default is "off".
    CHECK(fpvd::toWaybeamJson(c)["video0"]["resilience"] == "off");

    c.video.resilience = "fpv";
    CHECK(fpvd::toWaybeamJson(c)["video0"]["resilience"] == "fpv");
}
