#include "doctest.h"
#include "supervise/radio.hpp"

TEST_CASE("radio: bring up captures driver/iface from stdout") {
    fpvd::Config c{};
    auto r = fpvd::bringUpRadio("tests/fixtures/fake_radio_up_ok.sh", c);
    REQUIRE(r.ok);
    CHECK(r.driver == "8812eu");
    CHECK(r.iface == "wlan0");
    CHECK(r.adapterId.value_or("") == "bl-m8812eu2");
}

TEST_CASE("radio: failure surfaces exit code and stderr") {
    fpvd::Config c{};
    auto r = fpvd::bringUpRadio("tests/fixtures/fake_radio_up_fail.sh", c);
    CHECK_FALSE(r.ok);
    CHECK(r.exitCode == 3);
    CHECK(r.stderrText.find("missing modules") != std::string::npos);
}

#include <cstdlib>
#include <filesystem>
#include <fstream>

TEST_CASE("radio: tuneRadio passes action and env to script") {
    namespace fs = std::filesystem;
    auto rec = fs::temp_directory_path() / "fpvd-tune-record.txt";
    fs::remove(rec);
    ::setenv("FPVD_TEST_RECORD", rec.string().c_str(), 1);

    fpvd::Config c{};
    c.link.channel = 100;
    c.link.width = 40;
    c.link.txPowerDbm = 5;
    c.link.mtu = 1400;
    auto r = fpvd::tuneRadio("tests/fixtures/fake_radio_tune.sh", "txpower",
                             c, "wlan0", "8812eu");
    REQUIRE(r.ok);

    std::ifstream f(rec);
    std::string line;
    std::getline(f, line);
    CHECK(line.find("action=txpower") != std::string::npos);
    CHECK(line.find("iface=wlan0") != std::string::npos);
    CHECK(line.find("txpower=5") != std::string::npos);
    fs::remove(rec);
}

TEST_CASE("radio: tuneRadio surfaces non-zero exit + stderr") {
    fpvd::Config c{};
    auto r = fpvd::tuneRadio("tests/fixtures/fake_radio_up_fail.sh", "channel",
                             c, "wlan0", "8812eu");
    CHECK_FALSE(r.ok);
    CHECK(r.exitCode == 3);
    CHECK(r.stderrText.find("missing modules") != std::string::npos);
}
