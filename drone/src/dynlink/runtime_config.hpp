#pragma once
#include "dynlink/encoder_client.hpp" // RoiCurve
#include <cstdint>
#include <string>

namespace fpvd {
struct Config;
}

namespace fpvd::dynlink {

struct BitrateEngineConfig {
    double baseRedundancyRatio{0.5};
    double blocksPerFrame{2.0};
    int kMin{2};
    int kMax{50};
    int minBitrateKbps{1000};
    int maxBitrateKbps{24000};
    int mtuBytes{1500}; // from link.mtu
    int fps{60};        // from video.fps
};

struct DlRuntimeConfig {
    uint32_t healthTimeoutMs;
    uint32_t applyStaggerMs;
    uint32_t applySubPaceMs;
    bool swfec{false}; // link.fec.mode == "swfec"
    uint8_t swfecOverheadPct{50};
    uint8_t swfecDeadlineMs{30};
    bool debug;
    RoiCurve roiQp;
    BitrateEngineConfig bitrate;
    // Static link radiotap flags the controller PRESERVES (never decides). The
    // GS never sends stbc/ldpc, so every CMD_SET_RADIO the loop emits carries
    // these config values through unchanged rather than the old hardcoded 0/false.
    bool stbc;
    bool ldpc;
    // Canonical CHANNEL width (link.width: 10/20/40) — feeds the OpenIPC rate
    // table. The radiotap MODULATION width is modulationWidth() of this, derived
    // at the controller (d.bandwidth); they differ only at 10 MHz.
    uint8_t linkWidthMhz{20};
    bool probeEnabled{false}; // dynamicLink.probe.enabled; gates children + airtime
    uint16_t probeCtlPort{0}; // probe wfb_tx -C port; 0 when probe disabled
    int probeMcsCeiling{7};
    bool txPowerControl{true}; // dynamicLink.txPowerControl; false => controller runs iw auto
    std::string iface;
};

enum class HelloPub { Disabled, Announcing, Keepalive };

struct DlStatus { // published by the loop, read by HTTP thread
    bool running{false};
    bool watchdogTripped{false};
    long lastDecisionAgeMs{-1};         // -1 => none yet
    HelloPub hello{HelloPub::Disabled}; // always Disabled post-3b; kept because status.cpp reads it
                                        // for /status
};

// Pinned production endpoints; overridable in tests.
struct Endpoints {
    // listenPort is 9999 (not the dynamic-link drone.conf sample's 5800):
    // wfb_tun's default listen_port is 5800, so binding 5800 here collides
    // with the wfb tunnel — both bind 0.0.0.0:5800, the kernel splits the
    // tunnel frames, and wfb_tun aborts on a corrupted packet. The standalone
    // dl-applier used 9999 for exactly this reason.
    std::string listenAddr{"0.0.0.0"};
    uint16_t listenPort{9999};
    std::string wfbCtlAddr{"127.0.0.1"};
    uint16_t wfbCtlPort{8000};
    std::string encHost{"127.0.0.1"};
    uint16_t encPort{80};
    std::string gsTunnelAddr{"10.5.0.1"};
    uint16_t gsTunnelPort{5801};
    // OSD message-file path moved to the daemon-owned osd writer (osd/). This
    // is the control-loop tick cadence (named for its original OSD-refresh use).
    uint32_t osdUpdateIntervalMs{1000};
};

DlRuntimeConfig buildDlSnapshot(const Config& c, const std::string& iface);

} // namespace fpvd::dynlink
