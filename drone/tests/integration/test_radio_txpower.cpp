#include "doctest.h"
#include "dynlink/radio_txpower.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {
// Temp bin dir with a stub `iw` that records argv; prepended to PATH.
fs::path setupIwStub(const fs::path& tmp) {
    fs::create_directories(tmp);
    auto rec = tmp / "cmds.txt";
    fs::remove(rec);
    auto p = tmp / "iw";
    std::ofstream s(p);
    s << "#!/bin/sh\n"
      << "echo \"iw $*\" >> \"" << rec.string() << "\"\n"
      << "exit 0\n";
    s.close();
    fs::permissions(p, fs::perms::owner_all | fs::perms::group_read | fs::perms::group_exec |
                           fs::perms::others_read | fs::perms::others_exec);
    std::string path = tmp.string() + ":" + (std::getenv("PATH") ? std::getenv("PATH") : "");
    ::setenv("PATH", path.c_str(), 1);
    return rec;
}
std::string readAll(const fs::path& p) {
    std::ifstream f(p);
    std::string out((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    return out;
}
size_t countOccurrences(const std::string& hay, const std::string& needle) {
    size_t n = 0, pos = 0;
    while ((pos = hay.find(needle, pos)) != std::string::npos) {
        ++n;
        pos += needle.size();
    }
    return n;
}
} // namespace

TEST_CASE("RadioTxpower: applyAuto runs iw auto once, diff-suppressed") {
    auto tmp = fs::temp_directory_path() / "fpvd-radiotx-auto";
    fs::remove_all(tmp);
    auto rec = setupIwStub(tmp);

    fpvd::dynlink::RadioTxpower r("wlan0");
    CHECK(r.applyAuto() == 0);
    CHECK(r.applyAuto() == 0); // suppressed
    CHECK(countOccurrences(readAll(rec), "iw dev wlan0 set txpower auto") == 1);

    // A fixed apply re-issues fixed and clears auto state.
    CHECK(r.apply(20) == 0);
    CHECK(readAll(rec).find("iw dev wlan0 set txpower fixed 2000") != std::string::npos);

    // applyAuto again re-issues auto (state was cleared by apply).
    CHECK(r.applyAuto() == 0);
    CHECK(countOccurrences(readAll(rec), "iw dev wlan0 set txpower auto") == 2);

    fs::remove_all(tmp);
}
