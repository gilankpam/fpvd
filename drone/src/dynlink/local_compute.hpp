/* local_compute.hpp — Phase 3a drone-local decision compute. Overwrites the
 * GS-sent bitrate/k/n/fps on a decoded Decision with values the drone
 * derives from {mcs, bandwidth} via the OpenIPC calculator + block-fill FEC. */
#pragma once
#include "dynlink/wire.hpp"            // Decision
#include "dynlink/runtime_config.hpp"  // DlRuntimeConfig
#include <cstdint>

namespace fpvd::dynlink {

// Overwrites d.bitrateKbps / d.k / d.n / d.fps / d.txPowerDbm in place
// from the drone-local engine (txPowerDbm via the per-MCS curve). Leaves d.mcs,
// d.bandwidth untouched.
void applyLocalCompute(const DlRuntimeConfig& cfg, Decision& d);

} // namespace fpvd::dynlink
