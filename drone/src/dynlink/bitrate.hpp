/* bitrate.hpp — OpenIPC WFB-calculator effective-rate table + the
 * deterministic video-bitrate formula (Phase 3a). Pure functions. */
#pragma once
#include <cstdint>

namespace fpvd::dynlink {

// OpenIPC effective-rate table (kbps, long GI), MCS 0-7, already de-rated to
// real WFB throughput. bandwidthMhz is 20 or 40 (the radiotap value). Returns
// 0 for an unknown bandwidth or out-of-range mcs.
uint32_t openIpcBaseRateKbps(int bandwidthMhz, int mcs);

// wire_target_kbps = baseRate[bw][mcs] * (2/3 - probe_util), where
// probe_util = probeKbps / baseRate[bw][min(mcs+1, probeCeiling)].
// Clamps (2/3 - probe_util) to >= 0. Returns 0 if baseRate[bw][mcs] is 0.
double computeWireTargetKbps(int bandwidthMhz, int mcs, int probeCeiling,
                             double probeKbps);

// bitrate = trunc(wire_target * k / n), clamped to [minKbps, maxKbps].
// Truncation (not round) keeps the on-air wire rate <= wire_target.
uint16_t computeBitrateKbps(double wireTargetKbps, int k, int n,
                            int minKbps, int maxKbps);

} // namespace fpvd::dynlink
