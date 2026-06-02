#include "config/diff.hpp"
#include <nlohmann/json.hpp>

namespace fpvd {

SubsystemDiff diffSubsystems(const Config& a, const Config& b) {
    using nlohmann::json;
    SubsystemDiff d;
    json ja = a, jb = b;
    if (ja["link"] != jb["link"]) d.radio = true;
    // NOTE: encoder is no longer a rebuild trigger — Daemon::apply() applies
    // encoder changes via the waybeam API (waybeamConfigDiff), not a rebuild.
    // Retained for subsystem-classification reporting/tests.
    if (ja["video"] != jb["video"] || ja["image"] != jb["image"] ||
        ja["recording"] != jb["recording"])
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

LinkChange classifyLinkChange(const Config& a, const Config& b) {
    const auto& la = a.link;
    const auto& lb = b.link;
    LinkChange c;
    const bool channel = la.channel != lb.channel;
    const bool width   = la.width   != lb.width;
    c.nicChannel    = channel || width;
    c.nicWidth      = width;
    c.nicTxpower    = la.txpower != lb.txpower;
    c.nicMtu        = la.mtu != lb.mtu;
    c.videoRadiotap = (la.mcs != lb.mcs) || (la.stbc != lb.stbc) ||
                      (la.ldpc != lb.ldpc) || width;
    c.videoFec      = (la.fec.k != lb.fec.k) || (la.fec.n != lb.fec.n);
    c.fullRestart   = (la.linkId != lb.linkId) ||
                      (la.wlanAdapter != lb.wlanAdapter);
    // link.beamforming is intentionally not routed here — it is reconciled
    // separately in Daemon::apply() (bfChanged), not via this hot-apply diff.
    return c;
}

} // namespace fpvd
