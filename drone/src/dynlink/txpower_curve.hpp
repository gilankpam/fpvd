#pragma once
#include <cstdint>
#include <nlohmann/json.hpp>

namespace fpvd::dynlink {

// Per-MCS TX power (dBm) for the BL-M8812EU2 — the level-4 column of the OpenIPC
// adaptive-link wlan_adapters.yaml table, rounded to whole dBm (ref mBm
// 2900/2750/2500/2250/1900..., i.e. 29/27.5/25/22.5/19...; 27.5->28, 22.5->23).
// Full power at low MCS for range; backed off on the high-PAPR 64-QAM rungs
// (MCS4-7) so the PA stays linear. Indexed by MCS 0..7.
inline constexpr int8_t kTxPowerDbmByMcs[8] = {29, 28, 25, 23, 19, 19, 19, 19};

// dBm for the given MCS, clamping mcs to [0,7].
int8_t txpowerDbmForMcs(int mcs);

// The full per-MCS curve as a JSON array (MCS 0..7) for /status — the GS
// consumes this as the single source of truth for RSSI normalization.
nlohmann::json txPowerCurveJson();

} // namespace fpvd::dynlink
