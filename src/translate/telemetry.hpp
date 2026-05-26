#pragma once
#include "config/schema.hpp"
#include <string>
#include <vector>

namespace fpvd {

// Returns the argv (with argv[0] = binary path) for the configured router.
// Returns an empty vector if telemetry.router == "none".
std::vector<std::string> telemetryArgs(const Config& c);

} // namespace fpvd
