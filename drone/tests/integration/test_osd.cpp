#include "doctest.h"
#include "osd/writer.hpp"
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;
using namespace fpvd::osd;
using fpvd::dynlink::Decision;

static std::string slurp(const fs::path& p) {
    std::ifstream f(p); std::stringstream ss; ss << f.rdbuf(); return ss.str();
}

TEST_CASE("osd: writeStatus renders the BF token by code") {
    auto msg = fs::temp_directory_path() / "fpvd-osd-bf.msg";
    fs::remove(msg);
    Decision d{}; d.mcs = 4; d.bitrateKbps = 9000; d.k = 8; d.n = 12;
    d.txPowerDbm = 22;

    OsdWriter off(msg.string(), /*enabled=*/true);
    off.writeStatus(d, 0, /*bfCode=*/0, /*idrCount=*/0);
    CHECK(slurp(msg).find(" B") == std::string::npos);   // no token when off

    OsdWriter armed(msg.string(), true);
    armed.writeStatus(d, 0, /*bfCode=*/1, /*idrCount=*/0);
    CHECK(slurp(msg).find(" B-") != std::string::npos);  // armed, no report

    OsdWriter working(msg.string(), true);
    working.writeStatus(d, 0, /*bfCode=*/2, /*idrCount=*/0);
    CHECK(slurp(msg).find(" B+") != std::string::npos);  // working
    fs::remove(msg);
}

TEST_CASE("osd: provider code reaches the rendered line") {
    auto msg = fs::temp_directory_path() / "fpvd-osd-prov.msg";
    fs::remove(msg);
    Decision d{}; d.mcs = 3;
    OsdWriter w(msg.string(), true);
    int code = 2;                                  // stand-in for bfCodeProvider_()
    w.writeStatus(d, 0, code, /*idrCount=*/0);
    CHECK(slurp(msg).find(" B+") != std::string::npos);
    fs::remove(msg);
}
