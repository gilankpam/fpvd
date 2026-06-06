#pragma once
#include "config/schema.hpp"
#include <set>

namespace fpvd {

struct SubsystemDiff {
    bool radio{false};
    bool encoder{false};
    bool telemetry{false};
    bool dynamicLink{false};
    bool probeChanged{false};
    std::set<std::string> servicesAffected{};
};

SubsystemDiff diffSubsystems(const Config& a, const Config& b);

// Per-field routing for a link change. nicChannel/nicWidth drive iw retune
// (channel and width both reconfigure the NIC; width also bumps the video
// radiotap bandwidth). videoRadiotap/videoFec drive wfb_tx control commands.
// fullRestart fields (linkId/wlanAdapter) cannot be hot-applied.
struct LinkChange {
    bool nicChannel{false};    // channel || width  (NIC retune; drops air link)
    bool nicWidth{false};      // width specifically (radiotap bandwidth follows)
    bool nicTxpower{false};    // txpower
    bool nicMtu{false};        // mtu
    bool videoRadiotap{false}; // mcs || stbc || ldpc || width
    bool videoFec{false};      // fec.k || fec.n
    bool fullRestart{false};   // linkId || wlanAdapter
};

LinkChange classifyLinkChange(const Config& a, const Config& b);

} // namespace fpvd
