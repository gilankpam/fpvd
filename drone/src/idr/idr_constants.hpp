#pragma once
#include <cstdint>

namespace fpvd::idr {

// IDR keyframe-request relay — fixed, not operator-exposed.
//
// PixelPilot on the GS forwards a short UDP datagram to the drone whenever the
// decoder hits an RTP gap or stall; the relay turns that into a throttled
// `GET /request/idr` against the waybeam encoder. These are link-internal
// transport constants (mirror of the GS idrForward side), so they live here as
// compile-time values rather than in the user config tree.
constexpr uint16_t kIdrPort = 11223;            // UDP listen port (GS tunnel -> drone)
constexpr uint32_t kIdrMinIntervalMs = 500;     // throttle window between encoder IDR requests
constexpr const char* kIdrBindAddr = "0.0.0.0"; // bind all interfaces

} // namespace fpvd::idr
