/* test_dl_runtime_config.cpp — unit tests for buildDlSnapshot */
#include "doctest.h"
#include "dynlink/runtime_config.hpp"
#include "config/schema.hpp"
using namespace fpvd;
using namespace fpvd::dynlink;

TEST_CASE("buildDlSnapshot maps schema + derived inputs") {
    Config c{};                       // defaults
    c.link.mtu = 1400; c.video.fps = 90;
    c.link.stbc = false; c.link.ldpc = true;   // preserved through, not DL-decided
    c.dynamicLink.failsafe.mcs = 3; c.dynamicLink.healthTimeoutMs = 8000;
    auto s = buildDlSnapshot(c, "wlan1", std::nullopt, "8812eu");
    CHECK(s.iface == "wlan1");
    CHECK(s.stbc == false);
    CHECK(s.ldpc == true);
    CHECK(s.safe.mcs == 3);
    CHECK(s.healthTimeoutMs == 8000);
    CHECK(s.roiQp.thresholdKbps == 6000);   // default carried through
}

TEST_CASE("buildDlSnapshot maps all DynamicLink fields") {
    Config c{};
    c.dynamicLink.healthTimeoutMs    = 5000;
    c.dynamicLink.interleavingSupported = false;
    c.dynamicLink.minIdrIntervalMs   = 200;
    c.dynamicLink.applyStaggerMs     = 25;
    c.dynamicLink.applySubPaceMs     = 10;
    c.dynamicLink.osd.enabled        = false;
    c.dynamicLink.osd.debugLatency   = true;

    auto s = buildDlSnapshot(c, "wlan0", std::nullopt, "8812eu");

    CHECK(s.healthTimeoutMs     == 5000u);
    CHECK(s.interleavingSupported == false);
    CHECK(s.minIdrIntervalMs    == 200u);
    CHECK(s.applyStaggerMs      == 25u);
    CHECK(s.applySubPaceMs      == 10u);
    CHECK(s.osdEnabled          == false);
    CHECK(s.osdDebugLatency     == true);
    // debug field is absent from schema — must default to false
    CHECK(s.debug               == false);
}

TEST_CASE("buildDlSnapshot maps safe defaults correctly") {
    Config c{};
    c.dynamicLink.failsafe.mcs         = 2;
    c.dynamicLink.failsafe.k           = 6;
    c.dynamicLink.failsafe.n           = 10;
    c.dynamicLink.failsafe.depth       = 2;
    c.dynamicLink.failsafe.bandwidth   = 40;
    c.dynamicLink.failsafe.txPowerDbm  = 15;
    c.dynamicLink.failsafe.bitrateKbps = 3000;

    auto s = buildDlSnapshot(c, "wlan0", std::nullopt, "8812eu");

    CHECK(s.safe.mcs         == 2u);
    CHECK(s.safe.k           == 6u);
    CHECK(s.safe.n           == 10u);
    CHECK(s.safe.depth       == 2u);
    CHECK(s.safe.bandwidth   == 40u);
    CHECK(s.safe.txPowerDbm  == 15);
    CHECK(s.safe.bitrateKbps == 3000u);
}

TEST_CASE("buildDlSnapshot maps roiQp curve correctly") {
    Config c{};
    c.dynamicLink.roiQp.thresholdKbps = 8000;
    c.dynamicLink.roiQp.lowAnchorKbps = 3000;
    c.dynamicLink.roiQp.floor         = -18;
    c.dynamicLink.roiQp.step          = 2;

    auto s = buildDlSnapshot(c, "wlan0", std::nullopt, "8812eu");

    CHECK(s.roiQp.thresholdKbps == 8000u);
    CHECK(s.roiQp.lowAnchorKbps == 3000u);
    CHECK(s.roiQp.floor         == -18);
    CHECK(s.roiQp.step          == 2u);
}

TEST_CASE("buildDlSnapshot default Config produces correct defaults") {
    Config c{};
    auto s = buildDlSnapshot(c, "wlan2", std::nullopt, "8812eu");

    // DynamicLink defaults from schema
    CHECK(s.healthTimeoutMs      == 10000u);
    CHECK(s.interleavingSupported == true);
    CHECK(s.minIdrIntervalMs     == 500u);
    CHECK(s.applyStaggerMs       == 50u);
    CHECK(s.applySubPaceMs       == 5u);
    CHECK(s.osdEnabled           == true);
    CHECK(s.osdDebugLatency      == false);
    CHECK(s.debug                == false);

    // Link/video defaults
    CHECK(s.stbc          == true);   // link defaults now enable stbc/ldpc
    CHECK(s.ldpc          == true);
    CHECK(s.iface         == "wlan2");

    // RoiQp defaults
    CHECK(s.roiQp.thresholdKbps == 6000u);
    CHECK(s.roiQp.lowAnchorKbps == 2000u);
    CHECK(s.roiQp.floor         == -24);
    CHECK(s.roiQp.step          == 3u);

    // Safe defaults
    CHECK(s.safe.mcs         == 1u);
    CHECK(s.safe.k           == 8u);
    CHECK(s.safe.n           == 12u);
    CHECK(s.safe.depth       == 1u);
    CHECK(s.safe.bandwidth   == 20u);
    CHECK(s.safe.txPowerDbm  == 20);
    CHECK(s.safe.bitrateKbps == 2000u);
}

TEST_CASE("buildDlSnapshot maps the Phase-3a bitrate-engine knobs") {
    fpvd::Config c{};
    c.link.mtu  = 1400;
    c.video.fps = 90;
    c.dynamicLink.bitrate.minBitrateKbps = 1500;
    c.dynamicLink.bitrate.maxBitrateKbps = 20000;
    c.dynamicLink.fec.baseRedundancyRatio = 0.5;
    c.dynamicLink.fec.blocksPerFrame      = 2.0;
    c.dynamicLink.fec.kMin = 3;
    c.dynamicLink.fec.kMax = 40;

    auto s = fpvd::dynlink::buildDlSnapshot(c, "wlan0", std::nullopt, "8812eu");

    CHECK(s.bitrate.mtuBytes == 1400);
    CHECK(s.bitrate.fps == 90);
    CHECK(s.bitrate.minBitrateKbps == 1500);
    CHECK(s.bitrate.maxBitrateKbps == 20000);
    CHECK(s.bitrate.baseRedundancyRatio == doctest::Approx(0.5));
    CHECK(s.bitrate.blocksPerFrame == doctest::Approx(2.0));
    CHECK(s.bitrate.kMin == 3);
    CHECK(s.bitrate.kMax == 40);
}

TEST_CASE("buildDlSnapshot maps link.width to linkBandwidth (radiotap value)") {
    fpvd::Config c{};
    c.link.width = 20;
    auto s20 = fpvd::dynlink::buildDlSnapshot(c, "wlan0", std::nullopt, "8812eu");
    CHECK(s20.linkBandwidth == 20);
    c.link.width = 40;
    auto s40 = fpvd::dynlink::buildDlSnapshot(c, "wlan0", std::nullopt, "8812eu");
    CHECK(s40.linkBandwidth == 40);
    // modulationWidth maps HT40-as-20 (width=10) to the 20 MHz radiotap value
    c.link.width = 10;
    auto s10 = fpvd::dynlink::buildDlSnapshot(c, "wlan0", std::nullopt, "8812eu");
    CHECK(s10.linkBandwidth == 20);
}

TEST_CASE("buildDlSnapshot resolves the txpower curve from radio + override") {
    fpvd::Config c{};
    auto s1 = fpvd::dynlink::buildDlSnapshot(c, "wlan0", std::string("bl-m8812eu2"), "8812eu");
    CHECK(s1.txPowerCurve == fpvd::dynlink::TxPowerCurve{29,28,25,23,19,19,19,19});

    c.link.txpowerCurve = std::vector<int>{10,10,10,10,10,10,10,10};
    auto s2 = fpvd::dynlink::buildDlSnapshot(c, "wlan0", std::string("bl-m8812eu2"), "8812eu");
    CHECK(s2.txPowerCurve[0] == 10);
}
