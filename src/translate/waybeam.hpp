#pragma once
#include "config/schema.hpp"
#include <nlohmann/json.hpp>

namespace fpvd {

// Produce the JSON document that gets written to /etc/waybeam.json,
// derived from the user-modeled subset (Config) plus hardcoded defaults
// for waybeam keys we don't expose (isp/sensor/audio/imu/etc.).
nlohmann::json toWaybeamJson(const Config& c);

} // namespace fpvd
