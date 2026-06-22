#include "doctest.h"
#include "probe/probe_constants.hpp"
#include "probe/probe_specs.hpp"
#include <string>

using namespace fpvd;

static std::string joined(const std::vector<std::string>& v) {
    std::string s;
    for (auto& a : v) {
        s += a;
        s += ' ';
    }
    return s;
}

TEST_CASE("probe specs: one fec-off wfb_tx with control port + one feeder") {
    Config c{};
    c.link.linkId = 7669206;
    c.link.width = 20;
    c.link.stbc = true;
    c.link.ldpc = true;
    auto specs =
        buildProbeSpecs(c, "wlan0", "/etc/drone.key", "/usr/libexec/fpvd/probe-feeder", /*mcs=*/4);
    REQUIRE(specs.size() == 2);

    // wfb_tx
    const auto& tx = specs[0];
    CHECK(tx.name == "probe-tx");
    auto j = joined(tx.argv);
    CHECK(j.find("/usr/bin/wfb_tx ") != std::string::npos);
    CHECK(j.find(" -M 4 ") != std::string::npos);  // initial mcs
    CHECK(j.find(" -B 20 ") != std::string::npos); // modulationWidth(20)
    CHECK(j.find(" -S 1 ") != std::string::npos);  // stbc
    CHECK(j.find(" -L 1 ") != std::string::npos);  // ldpc
    CHECK(j.find(" -k 1 ") != std::string::npos);  // FEC off
    CHECK(j.find(" -n 1 ") != std::string::npos);
    CHECK(j.find(" -C 8001 ") != std::string::npos); // control port (retune)
    CHECK(j.find(" -i 7669206 ") != std::string::npos);
    CHECK(j.find(" -p 50 ") != std::string::npos);   // fixed radio_port
    CHECK(j.find(" -u 6700 ") != std::string::npos); // feed port
    CHECK(j.find(" wlan0 ") != std::string::npos);

    // feeder
    const auto& fd = specs[1];
    CHECK(fd.name == "probe-feed");
    CHECK(fd.argv ==
          std::vector<std::string>{"/usr/libexec/fpvd/probe-feeder", "6700", "25", "1400"});
    CHECK(fd.startAfter == std::vector<std::string>{"probe-tx"});
}

TEST_CASE("probe specs: mcs is clamped to the ceiling") {
    Config c{};
    auto specs = buildProbeSpecs(c, "wlan0", "/etc/drone.key", "/feeder", /*mcs=*/9);
    REQUIRE(specs.size() == 2);
    CHECK(joined(specs[0].argv).find(" -M 7 ") != std::string::npos); // clamped to kProbeMcsCeiling
}
