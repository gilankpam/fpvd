/* test_dl_osd.cpp — OSD status-line writer (ported from test_osd.c). */
#include "doctest.h"
#include "dynlink/osd.hpp"
#include "dynlink/wire.hpp"
#include <fstream>
#include <sstream>
#include <cstdio>   // unlink
using namespace fpvd::dynlink;

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

TEST_CASE("osd: status includes IDR counter") {
    std::string path = "/tmp/fpvd-osd-test.msg";

    OsdWriter osd(path, /*enabled=*/true, /*updateIntervalMs=*/1000, /*debugLatency=*/false);

    Decision d{};
    d.mcs          = 5;
    d.bitrateKbps  = 12000;
    d.k            = 8;
    d.n            = 14;
    d.txPowerDbm   = 18;

    /* Zero counter is rendered as I0. */
    osd.writeStatus(d, /*rssiDbm=*/-50, 0);
    std::string buf = readFile(path);
    CHECK(buf.find(" I0 |") != std::string::npos);

    /* Three bumps -> I3. */
    osd.bumpIdr();
    osd.bumpIdr();
    osd.bumpIdr();
    osd.writeStatus(d, -50, 0);
    buf = readFile(path);
    CHECK(buf.find(" I3 |") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("osd: status line contains expected fields") {
    std::string path = "/tmp/fpvd-osd-test2.msg";
    OsdWriter osd(path, /*enabled=*/true, /*updateIntervalMs=*/1000, /*debugLatency=*/false);

    Decision d{};
    d.mcs         = 3;
    d.bitrateKbps = 6000;
    d.k           = 8;
    d.n           = 12;
    d.txPowerDbm  = 20;

    osd.writeStatus(d, -60, 0);
    std::string buf = readFile(path);

    /* Prefix present */
    CHECK(buf.find("&L50&F30") != std::string::npos);
    /* MCS field */
    CHECK(buf.find("MCS3") != std::string::npos);
    /* Bitrate in Mbps: 6000 kbps -> 6 M */
    CHECK(buf.find("6M") != std::string::npos);
    /* FEC tuple */
    CHECK(buf.find("(8,12)") != std::string::npos);
    /* TX power */
    CHECK(buf.find("TX20") != std::string::npos);
    /* RSSI */
    CHECK(buf.find("R-60") != std::string::npos);
    /* msposd placeholders present */
    CHECK(buf.find("&B") != std::string::npos);
    CHECK(buf.find("&T") != std::string::npos);
    CHECK(buf.find("CPU&C") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("osd: disabled writes nothing") {
    std::string path = "/tmp/fpvd-osd-test3.msg";
    std::remove(path.c_str());

    OsdWriter osd(path, /*enabled=*/false, /*updateIntervalMs=*/1000, /*debugLatency=*/false);
    Decision d{};
    d.mcs = 3; d.bitrateKbps = 3000;

    osd.writeStatus(d, -70, 0);
    osd.writeEvent("test");
    osd.bumpIdr();

    /* File must not exist. */
    std::ifstream f(path);
    CHECK(!f.good());
}

TEST_CASE("osd: event line written before status line") {
    std::string path = "/tmp/fpvd-osd-test4.msg";
    OsdWriter osd(path, /*enabled=*/true, /*updateIntervalMs=*/1000, /*debugLatency=*/false);

    osd.writeEvent("WATCHDOG safe_defaults");
    std::string buf = readFile(path);
    /* Event prefix + text */
    CHECK(buf.find("&L50&F30") != std::string::npos);
    CHECK(buf.find("WATCHDOG safe_defaults") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("osd: writeStatus clears stale event line") {
    std::string path = "/tmp/fpvd-osd-test5.msg";
    OsdWriter osd(path, /*enabled=*/true, /*updateIntervalMs=*/1000, /*debugLatency=*/false);

    /* Set an event first. */
    osd.writeEvent("WATCHDOG safe_defaults");
    {
        std::string buf = readFile(path);
        CHECK(buf.find("WATCHDOG safe_defaults") != std::string::npos);
    }

    /* writeStatus should clear the event line. */
    Decision d{};
    d.mcs = 2; d.bitrateKbps = 4000; d.k = 4; d.n = 8;
    osd.writeStatus(d, -55, 0);
    {
        std::string buf = readFile(path);
        /* Event toast must be gone — writeStatus clears event_line. */
        CHECK(buf.find("WATCHDOG") == std::string::npos);
        CHECK(buf.find("MCS2") != std::string::npos);
    }

    std::remove(path.c_str());
}

TEST_CASE("osd: eventWatchdog writes expected text") {
    std::string path = "/tmp/fpvd-osd-test6.msg";
    OsdWriter osd(path, /*enabled=*/true, /*updateIntervalMs=*/1000, /*debugLatency=*/false);

    osd.eventWatchdog();
    std::string buf = readFile(path);
    CHECK(buf.find("WATCHDOG safe_defaults") != std::string::npos);

    std::remove(path.c_str());
}
