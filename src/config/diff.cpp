#include "config/diff.hpp"
#include <nlohmann/json.hpp>

namespace fpvd {

SubsystemDiff diffSubsystems(const Config& a, const Config& b) {
    using nlohmann::json;
    SubsystemDiff d;
    json ja = a, jb = b;
    if (ja["link"] != jb["link"]) d.radio = true;
    if (ja["video"] != jb["video"] || ja["image"] != jb["image"] ||
        ja["recording"] != jb["recording"] || ja["snapshot"] != jb["snapshot"])
        d.encoder = true;
    if (ja["telemetry"] != jb["telemetry"]) d.telemetry = true;

    // dynamicLink fires when its own subtree changes, OR when a derived
    // input feeding the translator (link.mtu, video.fps) moves.
    if (ja["dynamicLink"] != jb["dynamicLink"]) d.dynamicLink = true;
    if (ja["link"]["mtu"] != jb["link"]["mtu"]) d.dynamicLink = true;
    if (ja["video"]["fps"] != jb["video"]["fps"]) d.dynamicLink = true;

    // services
    for (auto& [name, sa] : a.services) {
        auto it = b.services.find(name);
        if (it == b.services.end()) { d.servicesAffected.insert(name); continue; }
        json jsa = sa, jsb = it->second;
        if (jsa != jsb) d.servicesAffected.insert(name);
    }
    for (auto& [name, sb] : b.services) {
        (void)sb;
        if (!a.services.count(name)) d.servicesAffected.insert(name);
    }
    return d;
}

} // namespace fpvd
