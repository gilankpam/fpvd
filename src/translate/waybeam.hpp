#pragma once
#include "config/schema.hpp"
#include <nlohmann/json.hpp>
#include <map>
#include <string>

namespace fpvd {

// Produce the JSON document that gets written to /etc/waybeam.json,
// derived from the user-modeled subset (Config) plus hardcoded defaults
// for waybeam keys we don't expose (isp/sensor/audio/imu/etc.).
nlohmann::json toWaybeamJson(const Config& c);

// Changed waybeam fields between two configs, bucketed by mutability.
//   live    — push via GET /api/v1/set (applied instantly, no restart)
//   restart — require a waybeam process restart to take effect
// Values are pre-formatted waybeam field values (snake_case field names).
struct WaybeamFieldDiff {
    std::map<std::string, std::string> live;
    std::map<std::string, std::string> restart;
};

// Diff the waybeam-relevant fields of `oldc` vs `newc`. `video.codec` is never
// emitted (retired in waybeam — hardcoded H.265). When `dlEnabled`, the
// dynamic-link-owned fields (video.bitrate, video.qpDelta, video.roi.*,
// video.fps) are omitted: the dynamic-link controller is their sole writer.
WaybeamFieldDiff waybeamConfigDiff(const Config& oldc, const Config& newc,
                                   bool dlEnabled);

} // namespace fpvd
