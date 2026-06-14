#include "doctest.h"
#include "config/diff.hpp"

using fpvd::Config;
using fpvd::classifyLinkChange;

TEST_CASE("classifyLinkChange: no change -> all false") {
    Config a{}, b{};
    auto c = classifyLinkChange(a, b);
    CHECK_FALSE(c.nicChannel);
    CHECK_FALSE(c.nicWidth);
    CHECK_FALSE(c.nicTxpower);
    CHECK_FALSE(c.nicMtu);
    CHECK_FALSE(c.videoRadiotap);
    CHECK_FALSE(c.videoFec);
    CHECK_FALSE(c.fullRestart);
}

TEST_CASE("classifyLinkChange: txpower only") {
    Config a{}, b{}; b.link.txPowerDbm = a.link.txPowerDbm + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.nicTxpower);
    CHECK_FALSE(c.nicChannel);
    CHECK_FALSE(c.videoRadiotap);
}

TEST_CASE("classifyLinkChange: mtu only -> nicMtu only") {
    Config a{}, b{}; b.link.mtu = a.link.mtu + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.nicMtu);
    CHECK_FALSE(c.nicChannel);
    CHECK_FALSE(c.videoRadiotap);
    CHECK_FALSE(c.videoFec);
}

TEST_CASE("classifyLinkChange: channel -> nicChannel only (not nicWidth)") {
    Config a{}, b{}; b.link.channel = a.link.channel + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.nicChannel);
    CHECK_FALSE(c.nicWidth);
    CHECK_FALSE(c.videoRadiotap);
}

TEST_CASE("classifyLinkChange: width -> nicChannel + nicWidth + videoRadiotap") {
    Config a{}, b{}; b.link.width = 40;  // default 20
    auto c = classifyLinkChange(a, b);
    CHECK(c.nicChannel);
    CHECK(c.nicWidth);
    CHECK(c.videoRadiotap);
}

TEST_CASE("classifyLinkChange: mcs -> videoRadiotap only") {
    Config a{}, b{}; b.link.mcs = a.link.mcs + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.videoRadiotap);
    CHECK_FALSE(c.nicChannel);
    CHECK_FALSE(c.videoFec);
}

TEST_CASE("classifyLinkChange: fec -> videoFec only") {
    // With default mode=swfec, the live knobs are overheadPct/deadlineMs.
    Config a{}, b{}; b.link.fec.overheadPct = a.link.fec.overheadPct + 10;
    auto c = classifyLinkChange(a, b);
    CHECK(c.videoFec);
    CHECK_FALSE(c.videoRadiotap);
}

TEST_CASE("classifyLinkChange: linkId -> fullRestart") {
    Config a{}, b{}; b.link.linkId = a.link.linkId + 1;
    auto c = classifyLinkChange(a, b);
    CHECK(c.fullRestart);
}

TEST_CASE("classifyLinkChange: wlanAdapter -> fullRestart") {
    Config a{}, b{}; b.link.wlanAdapter = std::string("bl-m8812eu2");
    auto c = classifyLinkChange(a, b);
    CHECK(c.fullRestart);
}

TEST_CASE("classifyLinkChange: fec mode flip -> fullRestart, not videoFec") {
    // Flip from swfec (default) to rs triggers fullRestart.
    Config a{}, b{}; b.link.fec.mode = "rs";
    auto c = classifyLinkChange(a, b);
    CHECK(c.fullRestart);
    CHECK_FALSE(c.videoFec);
}

TEST_CASE("classifyLinkChange: swfec param change -> videoFec only") {
    Config a{}, b{};
    a.link.fec.mode = b.link.fec.mode = "swfec";
    b.link.fec.overheadPct = 80;
    auto c = classifyLinkChange(a, b);
    CHECK(c.videoFec);
    CHECK_FALSE(c.fullRestart);
}

TEST_CASE("classifyLinkChange: rs k/n change ignored under swfec mode") {
    Config a{}, b{};
    a.link.fec.mode = b.link.fec.mode = "swfec";
    b.link.fec.k = 3;  // rs-only knob; meaningless in swfec mode
    auto c = classifyLinkChange(a, b);
    CHECK_FALSE(c.videoFec);
    CHECK_FALSE(c.fullRestart);
}
