/* bitrate.cpp — OpenIPC effective-rate table + bitrate formula (Phase 3a). */
#include "dynlink/bitrate.hpp"

namespace fpvd::dynlink {

uint32_t openIpcBaseRateKbps(int bandwidthMhz, int mcs) {
    // Source: OpenIPC WFB calculator (src/components/wfb-calculator.astro),
    // long-GI rows. 40/short/MCS6 upstream typo (980000) is irrelevant — GI is
    // long. Index = MCS 0-7.
    static const uint32_t kRate20[8] =
        {6500, 12000, 15500, 20000, 25000, 42000, 47500, 55000};
    static const uint32_t kRate40[8] =
        {9800, 18600, 30400, 40200, 55800, 80400, 90200, 97000};
    if (mcs < 0 || mcs > 7) return 0;
    if (bandwidthMhz == 20) return kRate20[mcs];
    if (bandwidthMhz == 40) return kRate40[mcs];
    return 0;
}

double computeWireTargetKbps(int bandwidthMhz, int mcs, int probeCeiling,
                             double probeKbps) {
    uint32_t base = openIpcBaseRateKbps(bandwidthMhz, mcs);
    if (base == 0) return 0.0;
    int probeRung = mcs + 1;
    if (probeRung > probeCeiling) probeRung = probeCeiling;
    uint32_t probeBase = openIpcBaseRateKbps(bandwidthMhz, probeRung);
    double probeUtil = (probeBase > 0) ? (probeKbps / static_cast<double>(probeBase)) : 0.0;
    double util = (2.0 / 3.0) - probeUtil;
    if (util < 0.0) util = 0.0;
    return static_cast<double>(base) * util;
}

uint16_t computeBitrateKbps(double wireTargetKbps, int k, int n,
                            int minKbps, int maxKbps) {
    auto sat16 = [](long x) -> uint16_t {
        if (x < 0) x = 0;
        if (x > 65535) x = 65535;
        return static_cast<uint16_t>(x);
    };
    if (k <= 0 || n <= 0)
        return sat16(minKbps);
    double raw = wireTargetKbps * static_cast<double>(k) / static_cast<double>(n);
    long v = static_cast<long>(raw);          // truncate toward zero (wire rounds DOWN)
    if (v < minKbps) v = minKbps;
    if (v > maxKbps) v = maxKbps;
    return sat16(v);
}

} // namespace fpvd::dynlink
