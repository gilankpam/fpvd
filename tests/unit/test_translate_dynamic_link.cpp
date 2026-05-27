#include "doctest.h"
#include "config/schema.hpp"
#include "translate/dynamic_link.hpp"
#include <algorithm>
#include <string>
#include <vector>

using fpvd::Config;
using fpvd::dynamicLinkArgs;

static bool has(const std::vector<std::string>& a, const std::string& s) {
    return std::find(a.begin(), a.end(), s) != a.end();
}

static std::string pairAfter(const std::vector<std::string>& a,
                              const std::string& flag) {
    auto it = std::find(a.begin(), a.end(), flag);
    if (it == a.end() || std::next(it) == a.end()) return {};
    return *std::next(it);
}

TEST_CASE("translate.dl: argv[0] is /usr/bin/dl-applier") {
    Config c{};
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(a[0] == "/usr/bin/dl-applier");
}

TEST_CASE("translate.dl: defaults map to expected schema-driven flags") {
    Config c{};
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(pairAfter(a, "--health-timeout-ms") == "10000");
    CHECK(pairAfter(a, "--interleaving-supported") == "1");
    CHECK(pairAfter(a, "--debug-enable") == "0");
    CHECK(pairAfter(a, "--min-idr-interval-ms") == "500");
    CHECK(pairAfter(a, "--apply-stagger-ms") == "50");
    CHECK(pairAfter(a, "--apply-sub-pace-ms") == "5");
    CHECK(pairAfter(a, "--mavlink-enable") == "1");
    CHECK(pairAfter(a, "--osd-enable") == "1");
    CHECK(pairAfter(a, "--osd-debug-latency") == "0");
    CHECK(pairAfter(a, "--roi-qp-threshold-kbps") == "6000");
    CHECK(pairAfter(a, "--roi-qp-low-anchor-kbps") == "2000");
    CHECK(pairAfter(a, "--roi-qp-floor") == "-24");
    CHECK(pairAfter(a, "--roi-qp-step") == "3");
    CHECK(pairAfter(a, "--safe-mcs") == "1");
    CHECK(pairAfter(a, "--safe-k") == "8");
    CHECK(pairAfter(a, "--safe-n") == "12");
    CHECK(pairAfter(a, "--safe-depth") == "1");
    CHECK(pairAfter(a, "--safe-bandwidth") == "20");
    CHECK(pairAfter(a, "--safe-tx-power-dBm") == "20");
    CHECK(pairAfter(a, "--safe-bitrate-kbps") == "2000");
}

TEST_CASE("translate.dl: derived flags come from link/video/iface") {
    Config c{};
    c.link.mtu = 1400;
    c.video.fps = 90;
    auto a = dynamicLinkArgs(c, "wlx00:11:22");
    CHECK(pairAfter(a, "--hello-mtu-bytes") == "1400");
    CHECK(pairAfter(a, "--hello-fps") == "90");
    CHECK(pairAfter(a, "--wlan-dev") == "wlx00:11:22");
}

TEST_CASE("translate.dl: hard-coded operational defaults present") {
    Config c{};
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(pairAfter(a, "--listen-addr") == "0.0.0.0");
    CHECK(pairAfter(a, "--listen-port") == "5800");
    CHECK(pairAfter(a, "--wfb-tx-ctrl-addr") == "127.0.0.1");
    CHECK(pairAfter(a, "--wfb-tx-ctrl-port") == "8000");
    CHECK(pairAfter(a, "--encoder-kind") == "waybeam");
    CHECK(pairAfter(a, "--encoder-host") == "127.0.0.1");
    CHECK(pairAfter(a, "--encoder-port") == "80");
    CHECK(pairAfter(a, "--idr-listen-addr") == "0.0.0.0");
    CHECK(pairAfter(a, "--idr-listen-port") == "11223");
    CHECK(pairAfter(a, "--mavlink-addr") == "127.0.0.1");
    CHECK(pairAfter(a, "--mavlink-port") == "14551");
    CHECK(pairAfter(a, "--osd-msg-path") == "/tmp/MSPOSD.msg");
    CHECK(pairAfter(a, "--osd-update-interval-ms") == "1000");
}

TEST_CASE("translate.dl: schema toggles propagate as 0/1") {
    Config c{};
    c.dynamicLink.interleavingSupported = false;
    c.dynamicLink.debug = true;
    c.dynamicLink.mavlinkEnable = false;
    c.dynamicLink.osd.enabled = false;
    c.dynamicLink.osd.debugLatency = true;
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(pairAfter(a, "--interleaving-supported") == "0");
    CHECK(pairAfter(a, "--debug-enable") == "1");
    CHECK(pairAfter(a, "--mavlink-enable") == "0");
    CHECK(pairAfter(a, "--osd-enable") == "0");
    CHECK(pairAfter(a, "--osd-debug-latency") == "1");
}

TEST_CASE("translate.dl: schema scalars propagate") {
    Config c{};
    c.dynamicLink.healthTimeoutMs = 7000;
    c.dynamicLink.minIdrIntervalMs = 250;
    c.dynamicLink.applyStaggerMs = 0;
    c.dynamicLink.applySubPaceMs = 0;
    c.dynamicLink.roiQp.thresholdKbps = 5000;
    c.dynamicLink.roiQp.lowAnchorKbps = 1500;
    c.dynamicLink.roiQp.floor = -18;
    c.dynamicLink.roiQp.step = 2;
    c.dynamicLink.safe.mcs = 3;
    c.dynamicLink.safe.bitrateKbps = 8000;
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK(pairAfter(a, "--health-timeout-ms") == "7000");
    CHECK(pairAfter(a, "--min-idr-interval-ms") == "250");
    CHECK(pairAfter(a, "--apply-stagger-ms") == "0");
    CHECK(pairAfter(a, "--apply-sub-pace-ms") == "0");
    CHECK(pairAfter(a, "--roi-qp-threshold-kbps") == "5000");
    CHECK(pairAfter(a, "--roi-qp-low-anchor-kbps") == "1500");
    CHECK(pairAfter(a, "--roi-qp-floor") == "-18");
    CHECK(pairAfter(a, "--roi-qp-step") == "2");
    CHECK(pairAfter(a, "--safe-mcs") == "3");
    CHECK(pairAfter(a, "--safe-bitrate-kbps") == "8000");
}

TEST_CASE("translate.dl: no --config and no --config-style flag") {
    // Sanity: we drive everything by CLI; no conf file path leaks in.
    Config c{};
    auto a = dynamicLinkArgs(c, "wlan0");
    CHECK_FALSE(has(a, "--config"));
    CHECK_FALSE(has(a, "/etc/dynamic-link/drone.conf"));
}
