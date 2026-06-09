/* txpower_curve.cpp — per-MCS tx power: per-radio defaults + override resolver. */
#include "dynlink/txpower_curve.hpp"

namespace fpvd::dynlink {

ResolvedCurve defaultTxpowerCurve(const std::optional<std::string>& adapterId,
                                  const std::string& driver) {
    if (adapterId && *adapterId == "bl-m8812eu2")
        return {kCurveM8812eu2, "bl-m8812eu2"};
    if (driver == "8812eu")
        return {kCurveM8812eu2, "bl-m8812eu2"};
    return {kCurveFallback, "fallback"};
}

ResolvedCurve resolveTxpowerCurve(const std::optional<std::vector<int>>& override_,
                                  const std::optional<std::string>& adapterId,
                                  const std::string& driver) {
    if (override_ && override_->size() == 8) {
        TxPowerCurve cv{};
        for (size_t i = 0; i < 8; ++i)
            cv[i] = static_cast<int8_t>((*override_)[i]);
        return {cv, "override"};
    }
    return defaultTxpowerCurve(adapterId, driver);
}

int8_t txpowerDbmForMcs(const TxPowerCurve& curve, int mcs) {
    if (mcs < 0) mcs = 0;
    if (mcs > 7) mcs = 7;
    return curve[static_cast<size_t>(mcs)];
}

} // namespace fpvd::dynlink
