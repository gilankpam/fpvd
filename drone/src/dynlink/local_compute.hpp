/* local_compute.hpp — Phase 3a drone-local decision compute. Overwrites the
 * GS-sent bitrate/k/n/depth/fps on a decoded Decision with values the drone
 * derives from {mcs, bandwidth} via the OpenIPC calculator + block-fill FEC. */
#pragma once
#include "dynlink/wire.hpp"            // Decision
#include "dynlink/runtime_config.hpp"  // DlRuntimeConfig
#include <cstdint>

namespace fpvd::dynlink {

// Constant interleave depth (no config field, by design — see the Phase-3a
// spec). Applied via the existing per-decision depth diff in dispatchTxApply.
inline constexpr uint8_t kInterleaveDepth = 1;

// Overwrites d.bitrateKbps / d.k / d.n / d.depth / d.fps in place from the
// drone-local engine. Leaves d.mcs, d.bandwidth, d.txPowerDbm untouched.
void applyLocalCompute(const DlRuntimeConfig& cfg, Decision& d);

} // namespace fpvd::dynlink
