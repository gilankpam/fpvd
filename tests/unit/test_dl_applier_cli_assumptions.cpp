#include "doctest.h"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static std::string readFile(const std::string& path) {
    std::ifstream f(path);
    std::stringstream b;
    b << f.rdbuf();
    return b.str();
}

TEST_CASE("dl-applier --help references every flag the translator emits") {
    auto help = readFile("tests/fixtures/dl_applier_help.txt");
    REQUIRE_FALSE(help.empty()); // fixture must exist

    const std::vector<std::string> required = {
        // listen + ctrl + encoder + idr + mavlink + osd hard-coded constants
        "--listen-addr", "--listen-port",
        "--wfb-tx-ctrl-addr", "--wfb-tx-ctrl-port",
        "--encoder-kind", "--encoder-host", "--encoder-port",
        "--idr-listen-addr", "--idr-listen-port",
        "--mavlink-addr", "--mavlink-port",
        "--osd-msg-path", "--osd-update-interval-ms",
        "--osd-enable", "--osd-debug-latency",
        // schema-driven scalars and toggles
        "--health-timeout-ms",
        "--interleaving-supported",
        "--min-idr-interval-ms",
        "--apply-stagger-ms", "--apply-sub-pace-ms",
        "--roi-qp-threshold-kbps", "--roi-qp-low-anchor-kbps",
        "--roi-qp-floor", "--roi-qp-step",
        "--safe-mcs", "--safe-k", "--safe-n", "--safe-depth",
        "--safe-bandwidth", "--safe-tx-power-dBm",
        "--safe-bitrate-kbps",
        // derived
        "--hello-mtu-bytes", "--hello-fps",
        "--wlan-dev",
    };
    for (auto& f : required) {
        INFO("flag not found in dl-applier --help: " << f);
        CHECK(help.find(f) != std::string::npos);
    }
}
