#include "doctest.h"
#include "config/schema.hpp"
#include "translate/wfb.hpp"
#include <algorithm>

using fpvd::Config;
using fpvd::wfbArgs;

TEST_CASE("translate.wfb: video tx argv") {
    Config c{};
    auto a = wfbArgs(c, fpvd::WfbRole::VideoTx, "wlan0", "/etc/drone.key");
    CHECK(a[0] == "/usr/bin/wfb_tx");
    CHECK(a.back() == "wlan0");
    auto contains = [&](const std::string& s){
        return std::find(a.begin(), a.end(), s) != a.end();
    };
    CHECK(contains("-K")); CHECK(contains("/etc/drone.key"));
    CHECK(contains("-M")); CHECK(contains("2"));
    CHECK(contains("-B")); CHECK(contains("20"));
    CHECK(contains("-k")); CHECK(contains("8"));
    CHECK(contains("-n")); CHECK(contains("12"));
    CHECK(contains("-U")); CHECK(contains("venc_wfb"));
    CHECK(contains("-i")); CHECK(contains("7669206"));
    CHECK(contains("-C")); CHECK(contains("8000"));
    CHECK(contains("-J")); CHECK(contains("10"));
    CHECK(contains("-E")); CHECK(contains("5000"));
}

TEST_CASE("translate.wfb: tunnel rx and tx argv") {
    Config c{};
    auto rx = wfbArgs(c, fpvd::WfbRole::TunRx, "wlan0", "/etc/drone.key");
    CHECK(rx[0] == "/usr/bin/wfb_rx");
    auto contains = [](auto& v, const std::string& s){
        return std::find(v.begin(), v.end(), s) != v.end();
    };
    CHECK(contains(rx, "-p")); CHECK(contains(rx, "160"));
    CHECK(contains(rx, "-u")); CHECK(contains(rx, "5800"));

    auto tx = wfbArgs(c, fpvd::WfbRole::TunTx, "wlan0", "/etc/drone.key");
    CHECK(tx[0] == "/usr/bin/wfb_tx");
    CHECK(contains(tx, "-p")); CHECK(contains(tx, "32"));
    CHECK(contains(tx, "-u")); CHECK(contains(tx, "5801"));
    // tunnel uses M=1 not user mcs
    CHECK(contains(tx, "-M")); CHECK(contains(tx, "1"));
}

TEST_CASE("translate.wfb: telemetry rx and tx argv") {
    Config c{};
    auto rx = wfbArgs(c, fpvd::WfbRole::TlmRx, "wlan0", "/etc/drone.key");
    auto contains = [](auto& v, const std::string& s){
        return std::find(v.begin(), v.end(), s) != v.end();
    };
    CHECK(contains(rx, "144"));
    CHECK(contains(rx, "14550"));

    auto tx = wfbArgs(c, fpvd::WfbRole::TlmTx, "wlan0", "/etc/drone.key");
    CHECK(contains(tx, "16"));
    CHECK(contains(tx, "14551"));
}

TEST_CASE("translate.wfb: wfb_tun argv") {
    auto a = fpvd::wfbTunArgs();
    CHECK(a[0] == "/usr/bin/wfb_tun");
    CHECK(std::find(a.begin(), a.end(), "10.5.0.10/24") != a.end());
}

TEST_CASE("translate.wfb: stbc and ldpc reflected") {
    Config c{}; c.link.stbc = true; c.link.ldpc = true;
    auto a = wfbArgs(c, fpvd::WfbRole::VideoTx, "wlan0", "/etc/drone.key");
    auto idxS = std::find(a.begin(), a.end(), "-S");
    REQUIRE(idxS != a.end());
    CHECK(*(idxS + 1) == "1");
    auto idxL = std::find(a.begin(), a.end(), "-L");
    REQUIRE(idxL != a.end());
    CHECK(*(idxL + 1) == "1");
}
