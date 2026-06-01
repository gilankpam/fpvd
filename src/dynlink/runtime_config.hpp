#pragma once
#include "dynlink/encoder_client.hpp"   // RoiCurve
#include <cstdint>
#include <string>

namespace fpvd { struct Config; }

namespace fpvd::dynlink {

struct SafeDefaults {
    uint8_t  mcs;
    uint8_t  k;
    uint8_t  n;
    uint8_t  depth;
    uint8_t  bandwidth;
    int8_t   txPowerDbm;
    uint16_t bitrateKbps;
};

struct DlRuntimeConfig {
    uint32_t healthTimeoutMs;
    uint32_t minIdrIntervalMs;
    uint32_t applyStaggerMs;
    uint32_t applySubPaceMs;
    bool     interleavingSupported;
    bool     osdEnabled;
    bool     osdDebugLatency;
    bool     debug;
    RoiCurve     roiQp;
    SafeDefaults safe;
    uint16_t helloMtuBytes;
    uint16_t helloFps;
    std::string  iface;
};

enum class HelloPub { Disabled, Announcing, Keepalive };

struct DlStatus {                         // published by the loop, read by HTTP thread
    bool     running{false};
    bool     watchdogTripped{false};
    long     lastDecisionAgeMs{-1};       // -1 => none yet
    HelloPub hello{HelloPub::Disabled};
};

// Pinned production endpoints; overridable in tests.
struct Endpoints {
    std::string  listenAddr{"0.0.0.0"};    uint16_t listenPort{5800};
    std::string  wfbCtlAddr{"127.0.0.1"};  uint16_t wfbCtlPort{8000};
    std::string  encHost{"127.0.0.1"};     uint16_t encPort{80};
    std::string  idrAddr{"0.0.0.0"};       uint16_t idrPort{11223};
    std::string  gsTunnelAddr{"10.5.0.1"}; uint16_t gsTunnelPort{5801};
    std::string  osdMsgPath{"/tmp/MSPOSD.msg"};
    uint32_t     osdUpdateIntervalMs{1000};
};

DlRuntimeConfig buildDlSnapshot(const Config& c, const std::string& iface);

} // namespace fpvd::dynlink
