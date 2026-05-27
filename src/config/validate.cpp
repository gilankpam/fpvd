#include "config/validate.hpp"
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <functional>

namespace fpvd {

static bool parseResolution(const std::string& s, int& w, int& h) {
    auto x = s.find('x');
    if (x == std::string::npos) return false;
    try {
        w = std::stoi(s.substr(0, x));
        h = std::stoi(s.substr(x + 1));
        return w > 0 && h > 0;
    } catch (...) { return false; }
}

static bool hasCycle(const std::map<std::string, Service>& svcs) {
    std::unordered_map<std::string, int> color;  // 0 unvisited, 1 in-progress, 2 done
    std::function<bool(const std::string&)> dfs = [&](const std::string& n) {
        if (color[n] == 1) return true;
        if (color[n] == 2) return false;
        auto it = svcs.find(n);
        if (it == svcs.end()) return false;
        color[n] = 1;
        for (auto& dep : it->second.startAfter) {
            if (svcs.count(dep) && dfs(dep)) return true;
        }
        color[n] = 2;
        return false;
    };
    for (auto& kv : svcs) {
        if (color[kv.first] == 0 && dfs(kv.first)) return true;
    }
    return false;
}

std::vector<ValidationError> validate(const Config& c) {
    std::vector<ValidationError> errs;

    // link
    if (c.link.width != 20 && c.link.width != 40)
        errs.push_back({"link.width", "must be 20 or 40"});
    if (c.link.mcs < 0 || c.link.mcs > 7)
        errs.push_back({"link.mcs", "must be 0..7"});
    if (c.link.txpower < 1 || c.link.txpower > 63)
        errs.push_back({"link.txpower", "must be 1..63"});
    if (c.link.fec.k < 1 || c.link.fec.k > 32 ||
        c.link.fec.n < 1 || c.link.fec.n > 32 ||
        c.link.fec.k >= c.link.fec.n)
        errs.push_back({"link.fec", "require 1<=k<n<=32"});
    if (c.link.channel < 1 || c.link.channel > 200)
        errs.push_back({"link.channel", "out of range"});

    // video
    if (c.video.codec != "h264" && c.video.codec != "h265")
        errs.push_back({"video.codec", "must be h264 or h265"});
    int w=0, h=0;
    if (!parseResolution(c.video.resolution, w, h))
        errs.push_back({"video.resolution", "must be WxH"});
    if (c.video.fps <= 0 || c.video.fps > 120)
        errs.push_back({"video.fps", "must be in (0,120]"});
    if (c.video.bitrate <= 0)
        errs.push_back({"video.bitrate", "must be > 0"});
    if (c.video.rcMode != "cbr" && c.video.rcMode != "vbr")
        errs.push_back({"video.rcMode", "must be cbr or vbr"});

    // image
    static const std::set<int> rots{0, 90, 180, 270};
    if (!rots.count(c.image.rotate))
        errs.push_back({"image.rotate", "must be 0/90/180/270"});

    // telemetry
    if (c.telemetry.router != "msposd" &&
        c.telemetry.router != "mavfwd" &&
        c.telemetry.router != "none")
        errs.push_back({"telemetry.router", "must be msposd|mavfwd|none"});

    // services
    static const std::set<std::string> restartModes{"always","on-failure","never"};
    for (auto& [name, s] : c.services) {
        if (!restartModes.count(s.restart))
            errs.push_back({"services." + name + ".restart",
                            "must be always|on-failure|never"});
        if (s.exec.empty())
            errs.push_back({"services." + name + ".exec", "must not be empty"});
    }
    if (hasCycle(c.services))
        errs.push_back({"services", "startAfter has a cycle"});

    // dynamicLink
    {
        const auto& dl = c.dynamicLink;
        if (dl.safe.mcs < 0 || dl.safe.mcs > 7)
            errs.push_back({"dynamicLink.safe.mcs", "must be 0..7"});
        if (dl.safe.k < 1 || dl.safe.k > 32 ||
            dl.safe.n < 1 || dl.safe.n > 32 ||
            dl.safe.k >= dl.safe.n)
            errs.push_back({"dynamicLink.safe.fec", "require 1<=k<n<=32"});
        if (dl.safe.depth < 1 || dl.safe.depth > 8)
            errs.push_back({"dynamicLink.safe.depth", "must be 1..8"});
        if (dl.safe.bandwidth != 20 && dl.safe.bandwidth != 40)
            errs.push_back({"dynamicLink.safe.bandwidth", "must be 20 or 40"});
        if (dl.safe.txPowerDbm < 0 || dl.safe.txPowerDbm > 30)
            errs.push_back({"dynamicLink.safe.txPowerDbm", "must be 0..30"});
        if (dl.safe.bitrateKbps <= 0)
            errs.push_back({"dynamicLink.safe.bitrateKbps", "must be > 0"});

        if (dl.healthTimeoutMs < 1000)
            errs.push_back({"dynamicLink.healthTimeoutMs", "must be >= 1000"});
        if (dl.minIdrIntervalMs < 16)
            errs.push_back({"dynamicLink.minIdrIntervalMs", "must be >= 16"});
        if (dl.applyStaggerMs < 0 || dl.applyStaggerMs > 1000)
            errs.push_back({"dynamicLink.applyStaggerMs", "must be 0..1000"});
        if (dl.applySubPaceMs < 0 || dl.applySubPaceMs > 50)
            errs.push_back({"dynamicLink.applySubPaceMs", "must be 0..50"});

        if (dl.roiQp.thresholdKbps <= 0 ||
            dl.roiQp.lowAnchorKbps <= 0 ||
            dl.roiQp.thresholdKbps <= dl.roiQp.lowAnchorKbps)
            errs.push_back({"dynamicLink.roiQp",
                            "require thresholdKbps > lowAnchorKbps > 0"});
        if (dl.roiQp.floor > 0)
            errs.push_back({"dynamicLink.roiQp.floor", "must be <= 0"});
        if (dl.roiQp.step < 1)
            errs.push_back({"dynamicLink.roiQp.step", "must be >= 1"});
    }

    return errs;
}

} // namespace fpvd
