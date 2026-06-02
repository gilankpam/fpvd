#include "doctest.h"
#include "supervise/radio.hpp"
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {
// Create a temp bin dir with stub `iw` and `ip` that record their args, and
// prepend it to PATH. Returns the record-file path. Stubs write "<tool> <args>".
fs::path setupStubs(const fs::path& tmp) {
    fs::create_directories(tmp);
    auto rec = tmp / "cmds.txt";
    fs::remove(rec);
    for (const char* tool : {"iw", "ip"}) {
        auto p = tmp / tool;
        std::ofstream s(p);
        s << "#!/bin/sh\n"
          << "echo \"" << tool << " $*\" >> \"" << rec.string() << "\"\n"
          << "exit 0\n";
        s.close();
        fs::permissions(p, fs::perms::owner_all | fs::perms::group_read |
                              fs::perms::group_exec | fs::perms::others_read |
                              fs::perms::others_exec);
    }
    std::string path = tmp.string() + ":" + (std::getenv("PATH") ? std::getenv("PATH") : "");
    ::setenv("PATH", path.c_str(), 1);
    return rec;
}

std::string readAllText(const fs::path& p) {
    std::ifstream f(p);
    std::string out, line;
    while (std::getline(f, line)) { out += line; out += "\n"; }
    return out;
}
} // namespace

TEST_CASE("radio-tune.sh: txpower scaling sign per driver") {
    auto tmp = fs::temp_directory_path() / "fpvd-rt-txpower";
    fs::remove_all(tmp);
    auto rec = setupStubs(tmp);

    fpvd::Config c{};
    c.link.txpower = 5;

    // 88XXau driver: txpower * -100  => -500
    auto r1 = fpvd::tuneRadio("scripts/radio-tune.sh", "txpower", c, "wlan0", "88XXau");
    REQUIRE(r1.ok);
    auto out1 = readAllText(rec);
    CHECK(out1.find("iw wlan0 set txpower fixed -500") != std::string::npos);

    // other driver: txpower * 50 => 250
    fs::remove(rec);
    auto r2 = fpvd::tuneRadio("scripts/radio-tune.sh", "txpower", c, "wlan0", "8812eu");
    REQUIRE(r2.ok);
    auto out2 = readAllText(rec);
    CHECK(out2.find("iw wlan0 set txpower fixed 250") != std::string::npos);

    fs::remove_all(tmp);
}

TEST_CASE("radio-tune.sh: channel width tokens") {
    auto tmp = fs::temp_directory_path() / "fpvd-rt-channel";
    fs::remove_all(tmp);
    auto rec = setupStubs(tmp);

    fpvd::Config c{};
    c.link.channel = 100;

    c.link.width = 10;
    fs::remove(rec);
    REQUIRE(fpvd::tuneRadio("scripts/radio-tune.sh", "channel", c, "wlan0", "8812eu").ok);
    CHECK(readAllText(rec).find("iw wlan0 set channel 100 10MHz") != std::string::npos);

    c.link.width = 40;
    fs::remove(rec);
    REQUIRE(fpvd::tuneRadio("scripts/radio-tune.sh", "channel", c, "wlan0", "8812eu").ok);
    CHECK(readAllText(rec).find("iw wlan0 set channel 100 HT40+") != std::string::npos);

    c.link.width = 20;
    fs::remove(rec);
    REQUIRE(fpvd::tuneRadio("scripts/radio-tune.sh", "channel", c, "wlan0", "8812eu").ok);
    CHECK(readAllText(rec).find("iw wlan0 set channel 100 HT20") != std::string::npos);

    fs::remove_all(tmp);
}

TEST_CASE("radio-tune.sh: mtu via ip link") {
    auto tmp = fs::temp_directory_path() / "fpvd-rt-mtu";
    fs::remove_all(tmp);
    auto rec = setupStubs(tmp);

    fpvd::Config c{};
    c.link.mtu = 1400;
    REQUIRE(fpvd::tuneRadio("scripts/radio-tune.sh", "mtu", c, "wlan0", "8812eu").ok);
    CHECK(readAllText(rec).find("ip link set wlan0 mtu 1400") != std::string::npos);

    fs::remove_all(tmp);
}
