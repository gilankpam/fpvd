#include "config/schema.hpp"
#include "doctest.h"
#include "translate/wfb.hpp"
#include <algorithm>

using fpvd::Config;
using fpvd::wfbArgs;

TEST_CASE("translate.wfb: video tx argv") {
    Config c{};
    auto a = wfbArgs(c, fpvd::WfbRole::VideoTx, "wlan0", "/etc/drone.key");
    CHECK(a[0] == "/usr/bin/wfb_tx");
    CHECK(a.back() == "wlan0");
    auto contains = [&](const std::string& s) {
        return std::find(a.begin(), a.end(), s) != a.end();
    };
    CHECK(contains("-K"));
    CHECK(contains("/etc/drone.key"));
    CHECK(contains("-M"));
    CHECK(contains("2"));
    CHECK(contains("-B"));
    CHECK(contains("20"));
    // Default mode is swfec: -z flag + overhead_pct=50 / deadline_ms=30
    CHECK(contains("-z"));
    CHECK(contains("-k"));
    CHECK(contains("50"));
    CHECK(contains("-n"));
    CHECK(contains("30"));
    CHECK(contains("-U"));
    CHECK(contains("venc_wfb"));
    CHECK(contains("-i"));
    CHECK(contains("7669206"));
    CHECK(contains("-C"));
    CHECK(contains("8000"));
    CHECK(contains("-J"));
    CHECK(contains("10"));
    CHECK(contains("-E"));
    CHECK(contains("5000"));
}

TEST_CASE("translate.wfb: tunnel rx and tx argv") {
    Config c{};
    auto rx = wfbArgs(c, fpvd::WfbRole::TunRx, "wlan0", "/etc/drone.key");
    CHECK(rx[0] == "/usr/bin/wfb_rx");
    auto contains = [](auto& v, const std::string& s) {
        return std::find(v.begin(), v.end(), s) != v.end();
    };
    CHECK(contains(rx, "-p"));
    CHECK(contains(rx, "160"));
    CHECK(contains(rx, "-u"));
    CHECK(contains(rx, "5800"));

    auto tx = wfbArgs(c, fpvd::WfbRole::TunTx, "wlan0", "/etc/drone.key");
    CHECK(tx[0] == "/usr/bin/wfb_tx");
    CHECK(contains(tx, "-p"));
    CHECK(contains(tx, "32"));
    CHECK(contains(tx, "-u"));
    CHECK(contains(tx, "5801"));
    // tun/tlm are boot-once with fixed robust params, independent of link.*
    auto at = [&](const std::string& flag) {
        auto it = std::find(tx.begin(), tx.end(), flag);
        REQUIRE(it != tx.end());
        return *(it + 1);
    };
    CHECK(at("-M") == "0"); // robust mcs=0
    CHECK(at("-k") == "3"); // fec 3/5
    CHECK(at("-n") == "5");
    CHECK(at("-B") == "20"); // HT20
    CHECK(at("-S") == "0");
    CHECK(at("-L") == "0");
    CHECK(at("-i") == "7669206"); // shared linkId
}

TEST_CASE("translate.wfb: telemetry rx and tx argv") {
    Config c{};
    auto rx = wfbArgs(c, fpvd::WfbRole::TlmRx, "wlan0", "/etc/drone.key");
    auto contains = [](auto& v, const std::string& s) {
        return std::find(v.begin(), v.end(), s) != v.end();
    };
    CHECK(contains(rx, "144"));
    CHECK(contains(rx, "14550"));

    auto tx = wfbArgs(c, fpvd::WfbRole::TlmTx, "wlan0", "/etc/drone.key");
    CHECK(contains(tx, "16"));
    CHECK(contains(tx, "14551"));
    auto att = [&](const std::string& flag) {
        auto it = std::find(tx.begin(), tx.end(), flag);
        REQUIRE(it != tx.end());
        return *(it + 1);
    };
    CHECK(att("-M") == "0");
    CHECK(att("-k") == "3");
    CHECK(att("-n") == "5");
}

TEST_CASE("translate.wfb: wfb_tun argv") {
    auto a = fpvd::wfbTunArgs();
    CHECK(a[0] == "/usr/bin/wfb_tun");
    CHECK(std::find(a.begin(), a.end(), "10.5.0.10/24") != a.end());
}

TEST_CASE("translate.wfb: width=10 injects -B 20 (modulation width)") {
    Config c{};
    c.link.width = 10;
    auto a = wfbArgs(c, fpvd::WfbRole::VideoTx, "wlan0", "/etc/drone.key");
    auto idx = std::find(a.begin(), a.end(), "-B");
    REQUIRE(idx != a.end());
    CHECK(*(idx + 1) == "20");
}

TEST_CASE("translate.wfb: stbc and ldpc reflected") {
    Config c{};
    c.link.stbc = true;
    c.link.ldpc = true;
    auto a = wfbArgs(c, fpvd::WfbRole::VideoTx, "wlan0", "/etc/drone.key");
    auto idxS = std::find(a.begin(), a.end(), "-S");
    REQUIRE(idxS != a.end());
    CHECK(*(idxS + 1) == "1");
    auto idxL = std::find(a.begin(), a.end(), "-L");
    REQUIRE(idxL != a.end());
    CHECK(*(idxL + 1) == "1");
}

TEST_CASE("translate.wfb: video tx argv in swfec mode") {
    Config c{};
    c.link.fec.mode = "swfec";
    c.link.fec.overheadPct = 60;
    c.link.fec.deadlineMs = 25;
    auto a = wfbArgs(c, fpvd::WfbRole::VideoTx, "wlan0", "/etc/drone.key");
    auto at = [&](const std::string& flag) {
        auto it = std::find(a.begin(), a.end(), flag);
        REQUIRE(it != a.end());
        return *(it + 1);
    };
    CHECK(std::find(a.begin(), a.end(), "-z") != a.end());
    CHECK(at("-k") == "60"); // overhead_pct rides -k
    CHECK(at("-n") == "25"); // deadline_ms rides -n
}

TEST_CASE("translate.wfb: tun/tlm tx stay RS even in swfec mode") {
    Config c{};
    c.link.fec.mode = "swfec";
    for (auto role : {fpvd::WfbRole::TunTx, fpvd::WfbRole::TlmTx}) {
        auto tx = wfbArgs(c, role, "wlan0", "/etc/drone.key");
        CHECK(std::find(tx.begin(), tx.end(), "-z") == tx.end());
    }
}
