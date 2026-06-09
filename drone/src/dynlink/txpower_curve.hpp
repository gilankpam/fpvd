#pragma once
#include <array>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fpvd::dynlink {

using TxPowerCurve = std::array<int8_t, 8>;   // dBm per MCS 0..7

// bl-m8812eu2 default (OpenIPC adaptive-link level-4 column). Full power at low
// MCS for range; backed off on the high-PAPR 64-QAM rungs so the PA stays linear.
inline constexpr TxPowerCurve kCurveM8812eu2 = { 29, 28, 25, 23, 19, 19, 19, 19 };

// Conservative default for radios we have not characterized: modest, flat-ish,
// backed off at the top so an unknown PA is never overdriven.
inline constexpr TxPowerCurve kCurveFallback = { 22, 22, 22, 20, 19, 19, 19, 19 };

struct ResolvedCurve {
    TxPowerCurve curve;
    std::string  source;   // "override" | "<radio>" | "fallback"
};

// Per-radio default, keyed by adapterId then driver; kCurveFallback otherwise.
ResolvedCurve defaultTxpowerCurve(const std::optional<std::string>& adapterId,
                                  const std::string& driver);

// Effective curve: a present, 8-long override wins (source "override"); else the
// per-radio default. A malformed override (wrong length) is ignored (default used).
ResolvedCurve resolveTxpowerCurve(const std::optional<std::vector<int>>& override_,
                                  const std::optional<std::string>& adapterId,
                                  const std::string& driver);

// dBm for the given MCS, clamping mcs to [0,7].
int8_t txpowerDbmForMcs(const TxPowerCurve& curve, int mcs);

} // namespace fpvd::dynlink
