#include "doctest.h"
#include "config/schema.hpp"
#include <nlohmann/json.hpp>

using fpvd::Config;
using nlohmann::json;

TEST_CASE("schema: round-trip a minimal config through json") {
    json j = json::parse(R"({
        "link":{"channel":161,"width":20,"txpower":1,"mcs":2,
                "fec":{"k":8,"n":12},"stbc":false,"ldpc":false,
                "linkId":7669206,"mtu":1500,"wlanAdapter":null,
                "beamforming":{"enabled":false,"remoteMac":"","ackTimeout":255,"intervalMs":100}},
        "video":{"codec":"h265","resolution":"1920x1080","fps":60,
                 "bitrate":8192,"rcMode":"cbr","gopSize":1.0,"qpDelta":-4,
                 "roi":{"enabled":true,"qp":0,"center":0.4,"steps":2}},
        "image":{"mirror":false,"flip":false,"rotate":0},
        "telemetry":{"router":"msposd","serial":"ttyS2","osdFps":20,"baud":115200},
        "recording":{"enabled":false,"format":"ts",
                     "mode":"mirror","maxSeconds":300,"maxMB":500},
        "dynamicLink":{
            "enabled":false,"healthTimeoutMs":10000,
            "interleavingSupported":true,
            "minIdrIntervalMs":500,"applyStaggerMs":50,"applySubPaceMs":5,
            "osd":{"enabled":true,"debugLatency":false},
            "roiQp":{"thresholdKbps":6000,"lowAnchorKbps":2000,
                     "floor":-24,"step":3},
            "safe":{"mcs":1,"k":8,"n":12,"depth":1,
                    "bandwidth":20,"txPowerDbm":20,"bitrateKbps":2000}
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
    c.dynamicLink.safe.mcs = 3;
    c.dynamicLink.roiQp.floor = -18;
    c.dynamicLink.osd.debugLatency = true;
    json j = c;
    fpvd::Config c2 = j.get<fpvd::Config>();
    CHECK(c2.dynamicLink.enabled == true);
    CHECK(c2.dynamicLink.safe.mcs == 3);
    CHECK(c2.dynamicLink.roiQp.floor == -18);
    CHECK(c2.dynamicLink.osd.debugLatency == true);
    // unchanged defaults round-trip too
    CHECK(c2.dynamicLink.healthTimeoutMs == 10000);
    CHECK(c2.dynamicLink.interleavingSupported == true);
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
    CHECK(c.dynamicLink.interleavingSupported == true);
    CHECK(c.dynamicLink.minIdrIntervalMs == 500);
    CHECK(c.dynamicLink.applyStaggerMs == 50);
    CHECK(c.dynamicLink.applySubPaceMs == 5);
    CHECK(c.dynamicLink.osd.enabled == true);
    CHECK(c.dynamicLink.osd.debugLatency == false);
    CHECK(c.dynamicLink.roiQp.thresholdKbps == 6000);
    CHECK(c.dynamicLink.roiQp.lowAnchorKbps == 2000);
    CHECK(c.dynamicLink.roiQp.floor == -24);
    CHECK(c.dynamicLink.roiQp.step == 3);
    CHECK(c.dynamicLink.safe.mcs == 1);
    CHECK(c.dynamicLink.safe.k == 8);
    CHECK(c.dynamicLink.safe.n == 12);
    CHECK(c.dynamicLink.safe.depth == 1);
    CHECK(c.dynamicLink.safe.bandwidth == 20);
    CHECK(c.dynamicLink.safe.txPowerDbm == 20);
    CHECK(c.dynamicLink.safe.bitrateKbps == 2000);
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

TEST_CASE("schema: Config parses without dynamicLink key — defaults applied") {
    using nlohmann::json;
    json j = json::parse(R"({
        "link": {"channel":161,"width":20,"txpower":1,"mcs":2,
                 "fec":{"k":8,"n":12},"stbc":false,"ldpc":false,
                 "linkId":7669206,"mtu":1500,"wlanAdapter":null},
        "video": {"codec":"h265","resolution":"1920x1080","fps":60,
                  "bitrate":8192,"rcMode":"cbr","gopSize":1.0,"qpDelta":-4,
                  "roi":{"enabled":true,"qp":0,"center":0.4,"steps":2}},
        "image": {"mirror":false,"flip":false,"rotate":0},
        "telemetry": {"router":"msposd","serial":"ttyS2","osdFps":20,"baud":115200},
        "recording": {"enabled":false,"format":"ts",
                      "mode":"mirror","maxSeconds":300,"maxMB":500},
        "services": {}
    })");
    fpvd::Config c = j.get<fpvd::Config>();  // must not throw
    CHECK(c.dynamicLink.enabled == false);
    CHECK(c.dynamicLink.safe.mcs == 1);
    CHECK(c.dynamicLink.healthTimeoutMs == 10000);
}
