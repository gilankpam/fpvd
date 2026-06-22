#include "config/lock.hpp"

namespace fpvd {

// Locked subtree: writes anywhere inside count. The strings here are the
// path *prefixes* the body cannot touch when dynamicLink is enabled.
static const std::vector<std::vector<std::string>> kLockedPaths = {
    {"link", "mcs"},
    // Only rs block geometry is DL-derived (computeK/computeN track the MCS), so
    // only k/n are locked. link.fec.mode / overheadPct / deadlineMs are static
    // swfec params the controller preserves — unlocked, like link.stbc/link.ldpc.
    // A wholesale link.fec overwrite still trips via the ancestor check below.
    {"link", "fec", "k"},
    {"link", "fec", "n"},
    {"link", "txPowerDbm"},
    // NOTE: link.stbc / link.ldpc are deliberately NOT locked. They are static
    // link parameters the controller preserves on every CMD_SET_RADIO from the
    // config snapshot — the GS decision never carries them — so an operator may
    // retune them while DL is enabled. link.txPowerDbm IS locked: the per-MCS power
    // curve drives tx power per decision, so a manual value would be overridden.
    // link.width is operator-owned: the controller derives the rate row + radiotap
    // from the selected width on each decision, so it is NOT locked.
    {"video", "bitrate"},
    {"video", "qpDelta"},
    {"video", "roi"},
};

// Walk the patch body collecting every dotted path it writes. Object
// children recurse; leaf values (numbers, strings, bools, null, arrays)
// terminate the path. An empty-object child still counts: the path leads
// to that key, even if it's wiping the subtree.
static void collectWrittenPaths(const nlohmann::json& body, std::vector<std::string>& prefix,
                                std::vector<std::vector<std::string>>& out) {
    if (!body.is_object()) {
        out.push_back(prefix);
        return;
    }
    if (body.empty()) {
        out.push_back(prefix);
        return;
    }
    for (auto it = body.begin(); it != body.end(); ++it) {
        prefix.push_back(it.key());
        collectWrittenPaths(it.value(), prefix, out);
        prefix.pop_back();
    }
}

static std::string joinDotted(const std::vector<std::string>& p) {
    std::string out;
    for (size_t i = 0; i < p.size(); ++i) {
        if (i)
            out.push_back('.');
        out += p[i];
    }
    return out;
}

// True iff `path` starts with `prefix` (component-wise).
static bool isUnderPrefix(const std::vector<std::string>& path,
                          const std::vector<std::string>& prefix) {
    if (path.size() < prefix.size())
        return false;
    for (size_t i = 0; i < prefix.size(); ++i) {
        if (path[i] != prefix[i])
            return false;
    }
    return true;
}

// True iff `path` is a strict ancestor of `prefix` (so a wholesale write
// at or above `link.fec` still trips the `link.fec` lock).
static bool isAncestorOf(const std::vector<std::string>& path,
                         const std::vector<std::string>& prefix) {
    return isUnderPrefix(prefix, path);
}

LockResult checkDynamicLinkLock(const nlohmann::json& patchBody, const Config& mergedPending) {
    if (!mergedPending.dynamicLink.enabled)
        return {true, {}};
    if (!patchBody.is_object())
        return {true, {}};
    if (patchBody.empty())
        return {true, {}}; // empty object touches no paths

    std::vector<std::vector<std::string>> written;
    std::vector<std::string> prefix;
    collectWrittenPaths(patchBody, prefix, written);

    LockResult r;
    for (auto& w : written) {
        for (auto& lk : kLockedPaths) {
            if (isUnderPrefix(w, lk) || isAncestorOf(w, lk)) {
                r.lockedPaths.push_back(joinDotted(w));
                break;
            }
        }
    }
    r.ok = r.lockedPaths.empty();
    return r;
}

} // namespace fpvd
