/* test_osd_writer.cpp — osd::OsdWriter status/base-line writer. */
#include "doctest.h"
#include "dynlink/wire.hpp"
#include "osd/osd_constants.hpp"
#include "osd/writer.hpp"
#include <cstdio> // unlink
#include <fstream>
#include <sstream>
using namespace fpvd::osd;
using fpvd::dynlink::Decision;

TEST_CASE("osd: glyph constants are non-empty multibyte UTF-8") {
    const char* glyphs[] = {kGlyphSignal1, kGlyphSignal2, kGlyphSignal3, kGlyphSpeed,
                            kGlyphShield,  kGlyphFlash,   kGlyphAntenna, kGlyphRefresh,
                            kGlyphFilm,    kGlyphThermo,  kGlyphWifi,    kGlyphCpu};
    for (const char* g : glyphs) {
        REQUIRE(g != nullptr);
        std::string s(g);
        CHECK(s.size() >= 2);                                  // multibyte (PUA glyph)
        CHECK((static_cast<unsigned char>(s[0]) & 0x80) != 0); // leading byte has high bit
    }
}

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

TEST_CASE("osd: status renders a per-line glyph column") {
    std::string path = "/tmp/fpvd-osd-col.msg";
    OsdWriter osd(path, /*enabled=*/true);

    Decision d{};
    d.mcs = 5;
    d.bitrateKbps = 18000;
    d.k = 12;
    d.n = 8;
    d.txPowerDbm = 19;

    osd.writeStatus(d, /*bfCode=*/2, /*idrCount=*/3);
    std::string buf = readFile(path);

    // Signal line: green (mcs>=kMcsGood) + signal-3 glyph + "MCS5".
    CHECK(buf.find("&L30&F30") != std::string::npos);
    CHECK(buf.find(std::string(kGlyphSignal3) + "MCS5") != std::string::npos);
    // Cap line: green, speedometer, "18Mbps".
    CHECK(buf.find(std::string(kGlyphSpeed) + "18Mbps") != std::string::npos);
    // FEC line: white, shield, "12/8".
    CHECK(buf.find("&L00&F30") != std::string::npos);
    CHECK(buf.find(std::string(kGlyphShield) + "12/8") != std::string::npos);
    // TX line: flash, "19dBm".
    CHECK(buf.find(std::string(kGlyphFlash) + "19dBm") != std::string::npos);
    // Beamforming working: cyan antenna.
    CHECK(buf.find("&L70&F30 " + std::string(kGlyphAntenna)) != std::string::npos);
    // IDR line: refresh, "3".
    CHECK(buf.find(std::string(kGlyphRefresh) + "3") != std::string::npos);
    // msposd placeholders carried on their own lines.
    CHECK(buf.find(std::string(kGlyphFilm) + "&B") != std::string::npos);
    CHECK(buf.find(std::string(kGlyphThermo) + "&T°C") != std::string::npos);
    CHECK(buf.find(std::string(kGlyphWifi) + "&W°C") != std::string::npos);
    CHECK(buf.find(std::string(kGlyphCpu) + "&C") != std::string::npos);

    std::remove(path.c_str());
}

TEST_CASE("osd: MCS tier drives signal/cap color and signal glyph") {
    std::string path = "/tmp/fpvd-osd-tier.msg";
    OsdWriter osd(path, true);
    Decision d{};
    d.bitrateKbps = 3000;

    d.mcs = 0; // failsafe rung -> red, weakest glyph
    osd.writeStatus(d, 0, 0);
    {
        std::string b = readFile(path);
        CHECK(b.find("&L20&F30 " + std::string(kGlyphSignal1) + "MCS0") != std::string::npos);
        CHECK(b.find("&L20&F30 " + std::string(kGlyphSpeed)) != std::string::npos); // cap red too
    }

    d.mcs = 2; // mid -> yellow, signal-2
    osd.writeStatus(d, 0, 0);
    {
        std::string b = readFile(path);
        CHECK(b.find("&L50&F30 " + std::string(kGlyphSignal2) + "MCS2") != std::string::npos);
    }

    d.mcs = 4; // good -> green, signal-3
    osd.writeStatus(d, 0, 0);
    {
        std::string b = readFile(path);
        CHECK(b.find("&L30&F30 " + std::string(kGlyphSignal3) + "MCS4") != std::string::npos);
    }
    std::remove(path.c_str());
}

TEST_CASE("osd: beamforming line by code (omitted/white/cyan)") {
    std::string path = "/tmp/fpvd-osd-bf.msg";
    OsdWriter osd(path, true);
    Decision d{};
    d.mcs = 3;

    osd.writeStatus(d, /*bfCode=*/0, 0);
    CHECK(readFile(path).find(kGlyphAntenna) == std::string::npos); // omitted when off

    osd.writeStatus(d, /*bfCode=*/1, 0);
    CHECK(readFile(path).find("&L00&F30 " + std::string(kGlyphAntenna)) !=
          std::string::npos); // armed=white

    osd.writeStatus(d, /*bfCode=*/2, 0);
    CHECK(readFile(path).find("&L70&F30 " + std::string(kGlyphAntenna)) !=
          std::string::npos); // active=cyan
    std::remove(path.c_str());
}

TEST_CASE("osd: writeBaseLine renders the system subset, no link lines") {
    std::string path = "/tmp/fpvd-osd-base.msg";
    OsdWriter osd(path, true);

    osd.writeBaseLine(/*bfCode=*/2);
    std::string b = readFile(path);
    CHECK(b.find(std::string(kGlyphFilm) + "&B") != std::string::npos);
    CHECK(b.find(std::string(kGlyphCpu) + "&C") != std::string::npos);
    CHECK(b.find("&L70&F30 " + std::string(kGlyphAntenna)) != std::string::npos); // BF active
    CHECK(b.find("MCS") == std::string::npos);       // no link decision data
    CHECK(b.find(kGlyphSpeed) == std::string::npos); // no cap line
    std::remove(path.c_str());
}

TEST_CASE("osd: event line is red, prepended, and cleared by writeStatus") {
    std::string path = "/tmp/fpvd-osd-evt.msg";
    OsdWriter osd(path, true);

    osd.writeEvent("WATCHDOG safe_defaults");
    {
        std::string b = readFile(path);
        CHECK(b.find("&L20&F30 WATCHDOG safe_defaults") != std::string::npos); // red toast
        // The toast is the first line of the file (prepended above any column).
        CHECK(b.find("&L20&F30 WATCHDOG safe_defaults") == 0);
    }

    Decision d{};
    d.mcs = 2;
    d.bitrateKbps = 4000;
    osd.writeStatus(d, 0, 0);
    {
        std::string b = readFile(path);
        CHECK(b.find("WATCHDOG") == std::string::npos); // cleared
        CHECK(b.find("MCS2") != std::string::npos);
    }
    std::remove(path.c_str());
}

TEST_CASE("osd: eventWatchdog writes expected text") {
    std::string path = "/tmp/fpvd-osd-wd.msg";
    OsdWriter osd(path, true);
    osd.eventWatchdog();
    CHECK(readFile(path).find("WATCHDOG safe_defaults") != std::string::npos);
    std::remove(path.c_str());
}

TEST_CASE("osd: disabled writes nothing; setEnabled gates and clears") {
    std::string path = "/tmp/fpvd-osd-en.msg";
    std::remove(path.c_str());

    OsdWriter osd(path, /*enabled=*/false);
    Decision d{};
    d.mcs = 3;
    osd.writeStatus(d, 0, 0);
    osd.writeBaseLine(0);
    {
        std::ifstream f(path);
        CHECK(!f.good()); // nothing written while disabled
    }

    osd.setEnabled(true);
    osd.writeBaseLine(0);
    CHECK(readFile(path).find("&C") != std::string::npos); // rendered when enabled

    osd.setEnabled(false);
    CHECK(readFile(path).empty()); // on->off actively clears the overlay
    std::remove(path.c_str());
}
