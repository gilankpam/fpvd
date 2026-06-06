#include "doctest.h"
#include "probe/probe_specs.hpp"

using namespace fpvd;

static std::string joined(const std::vector<std::string>& a) {
    std::string s;
    for (auto& x : a) { s += x; s += ' '; }
    return s;
}

TEST_CASE("buildProbeSpecs: empty when disabled") {
    Config c{};
    CHECK(buildProbeSpecs(c, "wlan0", "/etc/drone.key", "/usr/libexec/fpvd/probe-feeder").empty());
}

TEST_CASE("buildProbeSpecs: one wfb_tx + one feeder per MCS, PHY-mirrored, FEC off") {
    Config c{};
    c.link.width = 20; c.link.stbc = true; c.link.ldpc = true; c.link.linkId = 7669206;
    c.probe.enabled = true;
    c.probe.mcsList = {5, 7};
    c.probe.pps = 20; c.probe.packetBytes = 1400;
    c.probe.basePort = 50; c.probe.baseFeedPort = 6700;

    auto specs = buildProbeSpecs(c, "wlan0", "/etc/drone.key", "/usr/libexec/fpvd/probe-feeder");
    REQUIRE(specs.size() == 4);   // tx+feed for each of 2 MCS

    // stream 0 = MCS5 on port 50, feed 6700
    CHECK(specs[0].name == "probe-tx-mcs5");
    std::string tx0 = joined(specs[0].argv);
    CHECK(tx0.find("/usr/bin/wfb_tx ") == 0);
    CHECK(tx0.find("-M 5 ") != std::string::npos);
    CHECK(tx0.find("-B 20 ") != std::string::npos);
    CHECK(tx0.find("-S 1 ") != std::string::npos);
    CHECK(tx0.find("-L 1 ") != std::string::npos);
    CHECK(tx0.find("-k 1 ") != std::string::npos);
    CHECK(tx0.find("-n 1 ") != std::string::npos);
    CHECK(tx0.find("-i 7669206 ") != std::string::npos);
    CHECK(tx0.find("-p 50 ") != std::string::npos);
    CHECK(tx0.find("-u 6700 ") != std::string::npos);
    CHECK(tx0.find(" wlan0 ") != std::string::npos);

    CHECK(specs[1].name == "probe-feed-mcs5");
    CHECK(specs[1].argv == std::vector<std::string>{
        "/usr/libexec/fpvd/probe-feeder", "6700", "20", "1400"});
    REQUIRE(specs[1].startAfter.size() == 1);
    CHECK(specs[1].startAfter[0] == "probe-tx-mcs5");

    // stream 1 = MCS7 on port 51, feed 6701
    CHECK(specs[2].name == "probe-tx-mcs7");
    CHECK(joined(specs[2].argv).find("-p 51 ") != std::string::npos);
    CHECK(specs[3].argv[1] == "6701");
}
