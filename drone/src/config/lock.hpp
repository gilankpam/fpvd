#pragma once
#include "config/schema.hpp"
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace fpvd {

// When dynamicLink.enabled is true in the merged-pending config, the
// PATCH body may not write to any of these paths. The result lists the
// exact dotted paths the body tried to touch.
//
// Evaluation rule: the PATCH body itself is walked; the body's deep
// structure is what's checked (so writing `link.fec` wholesale counts
// as writing the subtree). The "would pending have enabled==true?"
// question is answered by the caller passing the merged pending Config.
struct LockResult {
    bool ok{true};
    std::vector<std::string> lockedPaths{};
};

LockResult checkDynamicLinkLock(const nlohmann::json& patchBody, const Config& mergedPending);

} // namespace fpvd
