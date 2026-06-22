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
// (FPVD_CHANNEL, FPVD_WIDTH, FPVD_TXPOWER_DBM, FPVD_MTU). Capture stdout
// (expected key=value lines) and stderr. Returns within ~30 seconds.
RadioResult bringUpRadio(const std::string& scriptPath, const Config& c);

// Apply a single live radio change via `scriptPath <action>` (channel,
// txpower, or mtu). Passes the relevant link.* fields plus the already-known
// iface/driver as env vars. Captures stderr; RadioResult.driver/iface unused.
RadioResult tuneRadio(const std::string& scriptPath, const std::string& action, const Config& c,
                      const std::string& iface, const std::string& driver);

} // namespace fpvd
