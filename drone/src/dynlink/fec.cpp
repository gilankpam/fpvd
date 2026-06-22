/* fec.cpp — block-fill compute_k + fixed-ratio compute_n (Phase 3a). */
#include "dynlink/fec.hpp"
#include <cmath>

namespace fpvd::dynlink {

int computeK(double wireTargetKbps, int mtuBytes, int fps, double baseRedundancyRatio,
             double blocksPerFrame, int kMin, int kMax) {
    if (wireTargetKbps <= 0.0 || mtuBytes <= 0 || fps <= 0 || blocksPerFrame <= 0.0 ||
        baseRedundancyRatio <= -1.0)
        return kMin;
    double anchorKbps = wireTargetKbps / (1.0 + baseRedundancyRatio);
    double packetsPerFrame = anchorKbps * 1000.0 / (static_cast<double>(fps) * mtuBytes * 8.0);
    int k = static_cast<int>(packetsPerFrame / blocksPerFrame); // trunc, matches GS int()
    if (k < kMin)
        k = kMin;
    if (k > kMax)
        k = kMax;
    return k;
}

int computeN(int k, double baseRedundancyRatio) {
    return static_cast<int>(std::ceil(static_cast<double>(k) * (1.0 + baseRedundancyRatio)));
}

} // namespace fpvd::dynlink
