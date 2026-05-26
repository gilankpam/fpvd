#pragma once
#include "config/schema.hpp"
#include <optional>
#include <string>

namespace fpvd {

struct RadioResult {
    bool ok{false};
    int exitCode{0};
    std::string driver;
    std::string iface;
    std::optional<std::string> adapterId;
    std::string stderrText;
};

// Execute `scriptPath`, passing the relevant link.* fields as env vars
// (FPVD_CHANNEL, FPVD_WIDTH, FPVD_TXPOWER, FPVD_MTU). Capture stdout
// (expected key=value lines) and stderr. Returns within ~30 seconds.
RadioResult bringUpRadio(const std::string& scriptPath, const Config& c);

} // namespace fpvd
