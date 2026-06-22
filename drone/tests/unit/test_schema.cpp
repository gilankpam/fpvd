#include "doctest.h"
#include "config/schema.hpp"
#include <nlohmann/json.hpp>

using fpvd::Config;
using nlohmann::json;

TEST_CASE("schema: round-trip a minimal config through json") {
    json j = json::parse(R"({
        "link":{"channel":161,"width":20,"txPowerDbm":20,"mcs":2,
                "fec":{"mode":"rs","k":8,"n":12,"overheadPct":50,"deadlineMs":30},"stbc":false,"ldpc":false,
                "linkId":7669206,"mtu":1500,"wlanAdapter":null,
                "beamforming":{"enabled":false,"remoteMac":"","ackTimeout":255,"intervalMs":100}},
        "video":{"codec":"h265","resolution":"1920x1080","fps":60,
                 "bitrate":8192,"rcMode":"cbr","gopSize":1.0,"resilience":"off","qpDelta":-4,
                 "sensorBin":"",
                 "roi":{"enabled":true,"qp":0,"center":0.4,"steps":2}},
        "image":{"mirror":false,"flip":false,"rotate":0},
        "telemetry":{"router":"msposd","serial":"ttyS2","osdFps":20,"baud":115200},
        "recording":{"enabled":false,"format":"ts",
                     "mode":"mirror","maxSeconds":300,"maxMB":500},
        "osd":{"enabled":true},
        "dynamicLink":{
            "enabled":false,"healthTimeoutMs":10000,
            "applyStaggerMs":50,"applySubPaceMs":5,
            "roiQp":{"thresholdKbps":6000,"lowAnchorKbps":2000,
                     "floor":-24,"step":3},
            "compute":{"minBitrateKbps":1000,"maxBitrateKbps":24000,"baseRedundancyRatio":0.5,"blocksPerFrame":2.0,"kMin":2,"kMax":50}
        },
        "services":{}
    })");

    Config c = j.get<Config>();
    CHECK(c.link.channel == 161);
    CHECK(c.video.fps == 60);
    CHECK(c.telemetry.router == "msposd");
    CHECK(c.services.empty());

    json out = c;
    // Idempotent round-trip.
    CHECK(out == j);
}

TEST_CASE("schema: video.sensorBin defaults empty and round-trips") {
    // Default is empty (sensor's own default binning).
    fpvd::Config c{};
    CHECK(c.video.sensorBin == "");

    // Serialised default config must carry the new field.
    json def = json(c);
    CHECK(def["video"].contains("sensorBin"));
    CHECK(def["video"]["sensorBin"] == "");

    // A value survives a parse -> serialise round-trip, both at the struct
    // and JSON level.
    c.video.sensorBin = "4lane";
    json j = c;
    CHECK(j["video"]["sensorBin"] == "4lane");
    auto c2 = j.get<fpvd::Config>();
    CHECK(c2.video.sensorBin == "4lane");
}

TEST_CASE("schema: video.resilience defaults to off and round-trips") {
    fpvd::Config c{};
    CHECK(c.video.resilience == "off");

    // Serialised default config must carry the new field.
    json def = json(c);
    CHECK(def["video"].contains("resilience"));
    CHECK(def["video"]["resilience"] == "off");

    // A value survives a parse -> serialise round-trip at struct and JSON level.
    c.video.resilience = "fpv";
    json j = c;
    CHECK(j["video"]["resilience"] == "fpv");
    auto c2 = j.get<fpvd::Config>();
    CHECK(c2.video.resilience == "fpv");
}

TEST_CASE("schema: service entry round-trips") {
    json j = json::parse(R"({
        "enabled":true,
        "exec":"/usr/bin/adaptive-link",
        "args":["--port","9601"],
        "env":{"LOG":"info"},
        "startAfter":["wfb_tlm_rx"],
        "restart":"always"
    })");
    auto s = j.get<fpvd::Service>();
    CHECK(s.enabled);
    CHECK(s.exec == "/usr/bin/adaptive-link");
    CHECK(s.args.size() == 2);
    CHECK(s.env.at("LOG") == "info");
    CHECK(s.startAfter == std::vector<std::string>{"wfb_tlm_rx"});
    CHECK(s.restart == "always");
    json out = s;
    CHECK(out == j);
}

TEST_CASE("schema: dynamicLink round-trips through json") {
    fpvd::Config c{};
    c.dynamicLink.enabled = true;
    c.dynamicLink.roiQp.floor = -18;
    c.osd.enabled = false;
    json j = c;
    fpvd::Config c2 = j.get<fpvd::Config>();
    CHECK(c2.dynamicLink.enabled == true);
    CHECK(c2.dynamicLink.roiQp.floor == -18);
    CHECK(c2.osd.enabled == false);
    // unchanged defaults round-trip too
    CHECK(c2.dynamicLink.healthTimeoutMs == 10000);
    // mavlinkEnable was removed from schema — must NOT appear in serialised output
    CHECK(j.at("dynamicLink").contains("mavlinkEnable") == false);
    // a stray mavlinkEnable in input is silently ignored (NLOHMANN_WITH_DEFAULT behaviour)
    nlohmann::json jStray = j;
    jStray["dynamicLink"]["mavlinkEnable"] = true;
    fpvd::Config c3 = jStray.get<fpvd::Config>();  // must not throw
    nlohmann::json jOut3 = c3;
    CHECK(jOut3.at("dynamicLink").contains("mavlinkEnable") == false);
}

TEST_CASE("schema: dynamicLink defaults match spec") {
    fpvd::Config c{};
    CHECK(c.dynamicLink.enabled == false);
    CHECK(c.dynamicLink.healthTimeoutMs == 10000);
    CHECK(c.dynamicLink.applyStaggerMs == 50);
    CHECK(c.dynamicLink.applySubPaceMs == 5);
    CHECK(c.osd.enabled == true);
    CHECK(c.dynamicLink.roiQp.thresholdKbps == 6000);
    CHECK(c.dynamicLink.roiQp.lowAnchorKbps == 2000);
    CHECK(c.dynamicLink.roiQp.floor == -24);
    CHECK(c.dynamicLink.roiQp.step == 3);
}

TEST_CASE("schema: beamforming defaults and round-trip") {
    fpvd::Config c{};
    CHECK(c.link.beamforming.enabled == false);
    CHECK(c.link.beamforming.remoteMac.empty());
    CHECK(c.link.beamforming.ackTimeout == 255);
    CHECK(c.link.beamforming.intervalMs == 100);

    // Round-trips through JSON.
    nlohmann::json j = c;
    auto c2 = j.get<fpvd::Config>();
    CHECK(c2.link.beamforming.ackTimeout == 255);
    CHECK(c2.link.beamforming.enabled == false);
    CHECK(c2.link.beamforming.intervalMs == 100);
    CHECK(c2.link.beamforming.remoteMac.empty());

    // Overlay predating the key still parses (WITH_DEFAULT).
    nlohmann::json old = {{"link", {{"channel", 149}}}};
    auto c3 = old.get<fpvd::Config>();
    CHECK(c3.link.beamforming.enabled == false);
    CHECK(c3.link.channel == 149);
}

TEST_CASE("schema: link.fec swfec keys default and round-trip") {
    fpvd::Config c{};
    CHECK(c.link.fec.mode == "swfec");
    CHECK(c.link.fec.overheadPct == 50);
    CHECK(c.link.fec.deadlineMs == 30);
    nlohmann::json j = c;
    auto back = j.get<fpvd::Config>();
    CHECK(back.link.fec.mode == "swfec");
    CHECK(back.link.fec.overheadPct == 50);
    CHECK(back.link.fec.deadlineMs == 30);
}

TEST_CASE("schema: legacy fec object without swfec keys parses with defaults") {
    // no "mode" key -> code default (swfec) fills in
    auto j = nlohmann::json::parse(R"({"link":{"fec":{"k":3,"n":5}}})");
    auto c = j.get<fpvd::Config>();
    CHECK(c.link.fec.k == 3);
    CHECK(c.link.fec.n == 5);
    CHECK(c.link.fec.mode == "swfec");
}

TEST_CASE("schema: Config parses without dynamicLink key — defaults applied") {
    using nlohmann::json;
    json j = json::parse(R"({
        "link": {"channel":161,"width":20,"txPowerDbm":20,"mcs":2,
                 "fec":{"k":8,"n":12},"stbc":false,"ldpc":false,
                 "linkId":7669206,"mtu":1500,"wlanAdapter":null},
        "video": {"codec":"h265","resolution":"1920x1080","fps":60,
                  "bitrate":8192,"rcMode":"cbr","gopSize":1.0,"resilience":"off","qpDelta":-4,
                  "sensorBin":"",
                  "roi":{"enabled":true,"qp":0,"center":0.4,"steps":2}},
        "image": {"mirror":false,"flip":false,"rotate":0},
        "telemetry": {"router":"msposd","serial":"ttyS2","osdFps":20,"baud":115200},
        "recording": {"enabled":false,"format":"ts",
                      "mode":"mirror","maxSeconds":300,"maxMB":500},
        "services": {}
    })");
    fpvd::Config c = j.get<fpvd::Config>();  // must not throw
    CHECK(c.dynamicLink.enabled == false);
    CHECK(c.dynamicLink.healthTimeoutMs == 10000);
}
