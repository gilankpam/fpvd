/* test_dl_radio_txpower.cpp — unit tests for RadioTxpower.
 *
 * Stubs `iw` on PATH (a fake executable that records argv to a temp file),
 * following the pattern from tests/integration/test_radio_tune_script.cpp.
 */
#include "doctest.h"
#include "dynlink/radio_txpower.hpp"

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <string>

namespace fs = std::filesystem;

namespace {

// Create a temp bin dir with a stub `iw` that records its args to a file.
// The stub writes one line per invocation: "iw <args>".
// Returns the record-file path.
fs::path setupIwStub(const fs::path& tmp, int exitCode = 0) {
    fs::create_directories(tmp);
    auto rec = tmp / "cmds.txt";
    fs::remove(rec);
    auto p = tmp / "iw";
    std::ofstream s(p);
    s << "#!/bin/sh\n"
      << "echo \"iw $*\" >> \"" << rec.string() << "\"\n"
      << "exit " << exitCode << "\n";
    s.close();
    fs::permissions(p, fs::perms::owner_all | fs::perms::group_read |
                          fs::perms::group_exec | fs::perms::others_read |
                          fs::perms::others_exec);
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

// Count lines in a file (number of iw invocations recorded).
int countLines(const fs::path& p) {
    std::ifstream f(p);
    if (!f) return 0;
    int n = 0;
    std::string line;
    while (std::getline(f, line)) if (!line.empty()) ++n;
    return n;
}

} // namespace

TEST_CASE("RadioTxpower: apply runs iw with mBm (dBm*100)") {
    auto tmp = fs::temp_directory_path() / "fpvd-txpower-basic";
    fs::remove_all(tmp);
    auto rec = setupIwStub(tmp, 0);

    fpvd::dynlink::RadioTxpower r("wlan0");

    // First apply: should run iw with 20*100=2000
    CHECK(r.apply(20) == 0);
    auto out = readAllText(rec);
    CHECK(out.find("iw dev wlan0 set txpower fixed 2000") != std::string::npos);
    CHECK(countLines(rec) == 1);

    // Second apply with same value: diff suppressed, no new iw invocation
    CHECK(r.apply(20) == 0);
    CHECK(countLines(rec) == 1);

    // Third apply with new value: should run iw with 22*100=2200
    CHECK(r.apply(22) == 0);
    auto out2 = readAllText(rec);
    CHECK(out2.find("iw dev wlan0 set txpower fixed 2200") != std::string::npos);
    CHECK(countLines(rec) == 2);

    fs::remove_all(tmp);
}

TEST_CASE("RadioTxpower: applySafe runs unconditionally") {
    auto tmp = fs::temp_directory_path() / "fpvd-txpower-safe";
    fs::remove_all(tmp);
    auto rec = setupIwStub(tmp, 0);

    fpvd::dynlink::RadioTxpower r("wlan0");

    // applySafe always runs iw, even if same dBm
    CHECK(r.applySafe(20) == 0);
    CHECK(countLines(rec) == 1);

    CHECK(r.applySafe(20) == 0);
    CHECK(countLines(rec) == 2);

    auto out = readAllText(rec);
    int count = 0;
    std::string::size_type pos = 0;
    while ((pos = out.find("iw dev wlan0 set txpower fixed 2000", pos)) != std::string::npos) {
        ++count;
        pos += 1;
    }
    CHECK(count == 2);

    fs::remove_all(tmp);
}

TEST_CASE("RadioTxpower: non-zero iw exit -> apply returns -1") {
    auto tmp = fs::temp_directory_path() / "fpvd-txpower-fail";
    fs::remove_all(tmp);
    setupIwStub(tmp, 1); // exit 1

    fpvd::dynlink::RadioTxpower r("wlan0");
    CHECK(r.apply(20) == -1);

    fs::remove_all(tmp);
}

TEST_CASE("RadioTxpower: failed apply does not cache value (retry runs iw again)") {
    auto tmp = fs::temp_directory_path() / "fpvd-txpower-retry";
    fs::remove_all(tmp);
    auto rec = setupIwStub(tmp, 1); // exit 1

    fpvd::dynlink::RadioTxpower r("wlan0");

    // Two failed applies at same dBm -> iw should be called both times (no caching on failure)
    CHECK(r.apply(20) == -1);
    CHECK(r.apply(20) == -1);
    CHECK(countLines(rec) == 2);

    fs::remove_all(tmp);
}

TEST_CASE("RadioTxpower: negative dBm produces negative mBm") {
    auto tmp = fs::temp_directory_path() / "fpvd-txpower-neg";
    fs::remove_all(tmp);
    auto rec = setupIwStub(tmp, 0);

    fpvd::dynlink::RadioTxpower r("wlan0");

    // -5 dBm -> mBm = -5 * 100 = -500
    CHECK(r.apply(-5) == 0);
    auto out = readAllText(rec);
    CHECK(out.find("txpower fixed -500") != std::string::npos);

    fs::remove_all(tmp);
}

TEST_CASE("RadioTxpower: setIface resets diff state") {
    auto tmp = fs::temp_directory_path() / "fpvd-txpower-setiface";
    fs::remove_all(tmp);
    auto rec = setupIwStub(tmp, 0);

    fpvd::dynlink::RadioTxpower r("wlan0");

    CHECK(r.apply(20) == 0);
    CHECK(countLines(rec) == 1);

    // Same dBm -> no-op
    CHECK(r.apply(20) == 0);
    CHECK(countLines(rec) == 1);

    // After setIface, current_ resets: same dBm should now re-run iw
    r.setIface("wlan1");
    CHECK(r.apply(20) == 0);
    auto out = readAllText(rec);
    CHECK(out.find("iw dev wlan1 set txpower fixed 2000") != std::string::npos);
    CHECK(countLines(rec) == 2);

    fs::remove_all(tmp);
}
