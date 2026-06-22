/* test_osd_writer.cpp — osd::OsdWriter status/base-line writer. */
#include "doctest.h"
#include "dynlink/wire.hpp"
#include "osd/writer.hpp"
#include <cstdio> // unlink
#include <fstream>
#include <sstream>
using namespace fpvd::osd;
using fpvd::dynlink::Decision;

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

TEST_CASE("osd: status includes IDR counter") {
    std::string path = "/tmp/fpvd-osd-test.msg";

    OsdWriter osd(path, /*enabled=*/true);

    Decision d{};
    d.mcs = 5;
    d.bitrateKbps = 12000;
    d.k = 8;
    d.n = 14;
    d.txPowerDbm = 18;

    /* Zero count is rendered as I0. */
    osd.writeStatus(d, 0, /*idrCount=*/0);
    std::string buf = readFile(path);
    CHECK(buf.find(" I0 |") != std::string::npos);

    /* The count is owned by the always-on relay and passed in -> I3. */
    osd.writeStatus(d, 0, /*idrCount=*/3);
    buf = readFile(path);
    CHECK(buf.find(" I3 |") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("osd: status line contains expected fields") {
    std::string path = "/tmp/fpvd-osd-test2.msg";
    OsdWriter osd(path, /*enabled=*/true);

    Decision d{};
    d.mcs = 3;
    d.bitrateKbps = 6000;
    d.k = 8;
    d.n = 12;
    d.txPowerDbm = 20;

    osd.writeStatus(d, 0, /*idrCount=*/0);
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
    /* msposd placeholders present */
    CHECK(buf.find("&B") != std::string::npos);
    CHECK(buf.find("&T") != std::string::npos);
    CHECK(buf.find("CPU&C") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("osd: writeBaseLine renders system stats + BF token, no decision data") {
    std::string path = "/tmp/fpvd-osd-base.msg";
    OsdWriter osd(path, /*enabled=*/true);

    osd.writeBaseLine(/*bfCode=*/2);
    std::string buf = readFile(path);
    CHECK(buf.find("&L50&F30") != std::string::npos); // prefix
    CHECK(buf.find("&B  T&T  W&W  CPU&C") != std::string::npos);
    CHECK(buf.find(" B+") != std::string::npos); // working BF token
    CHECK(buf.find("MCS") == std::string::npos); // no link decision data

    std::remove(path.c_str());
}

TEST_CASE("osd: disabled writes nothing (status AND base line)") {
    std::string path = "/tmp/fpvd-osd-test3.msg";
    std::remove(path.c_str());

    OsdWriter osd(path, /*enabled=*/false);
    Decision d{};
    d.mcs = 3;
    d.bitrateKbps = 3000;

    osd.writeStatus(d, 0, /*idrCount=*/0);
    osd.writeEvent("test");
    osd.writeBaseLine(0);

    /* File must not exist. */
    std::ifstream f(path);
    CHECK(!f.good());
}

TEST_CASE("osd: setEnabled gates writes at runtime") {
    std::string path = "/tmp/fpvd-osd-toggle.msg";
    std::remove(path.c_str());

    OsdWriter osd(path, /*enabled=*/false);
    osd.setEnabled(true);
    osd.writeBaseLine(0);
    CHECK(readFile(path).find("CPU&C") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("osd: setEnabled(false) clears the overlay") {
    std::string path = "/tmp/fpvd-osd-toggle-off.msg";
    std::remove(path.c_str());

    OsdWriter osd(path, /*enabled=*/true);
    osd.writeBaseLine(0);
    CHECK(readFile(path).find("CPU&C") != std::string::npos); // rendered

    /* Toggling OSD off must actively clear the msg file: msposd holds + re-
     * renders the last bytes forever, so flipping the flag alone leaves a stale
     * overlay on screen. The file must be emptied. */
    osd.setEnabled(false);
    CHECK(readFile(path).empty());

    std::remove(path.c_str());
}

TEST_CASE("osd: event line written before status line") {
    std::string path = "/tmp/fpvd-osd-test4.msg";
    OsdWriter osd(path, /*enabled=*/true);

    osd.writeEvent("WATCHDOG safe_defaults");
    std::string buf = readFile(path);
    /* Event prefix + text */
    CHECK(buf.find("&L50&F30") != std::string::npos);
    CHECK(buf.find("WATCHDOG safe_defaults") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("osd: writeStatus clears stale event line") {
    std::string path = "/tmp/fpvd-osd-test5.msg";
    OsdWriter osd(path, /*enabled=*/true);

    /* Set an event first. */
    osd.writeEvent("WATCHDOG safe_defaults");
    {
        std::string buf = readFile(path);
        CHECK(buf.find("WATCHDOG safe_defaults") != std::string::npos);
    }

    /* writeStatus should clear the event line. */
    Decision d{};
    d.mcs = 2;
    d.bitrateKbps = 4000;
    d.k = 4;
    d.n = 8;
    osd.writeStatus(d, 0, /*idrCount=*/0);
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
    OsdWriter osd(path, /*enabled=*/true);

    osd.eventWatchdog();
    std::string buf = readFile(path);
    CHECK(buf.find("WATCHDOG safe_defaults") != std::string::npos);

    std::remove(path.c_str());
}
