/* local_compute.cpp — Phase 3a drone-local decision compute. */
#include "dynlink/local_compute.hpp"
#include "dynlink/bitrate.hpp"
#include "dynlink/fec.hpp"
#include "dynlink/txpower_curve.hpp"
#include "probe/probe_constants.hpp"

namespace fpvd::dynlink {

void applyLocalCompute(const DlRuntimeConfig& cfg, Decision& d) {
    const BitrateEngineConfig& b = cfg.bitrate;
    // The probe runs FEC-off at a fixed rate; its true on-air kbps.
    double probeKbps =
        static_cast<double>(fpvd::kProbePps) * fpvd::kProbePacketBytes * 8.0 / 1000.0;
    double wireTarget =
        computeWireTargetKbps(d.bandwidth, d.mcs, cfg.probeMcsCeiling, probeKbps);
    int k = computeK(wireTarget, b.mtuBytes, b.fps,
                     b.baseRedundancyRatio, b.blocksPerFrame, b.kMin, b.kMax);
    int n = computeN(k, b.baseRedundancyRatio);
    auto sat8 = [](int x) -> uint8_t {
        if (x < 0) x = 0;
        if (x > 255) x = 255;
        return static_cast<uint8_t>(x);
    };
    d.k           = sat8(k);
    d.n           = sat8(n);
    d.bitrateKbps = computeBitrateKbps(wireTarget, k, n,
                                       b.minBitrateKbps, b.maxBitrateKbps);
    d.depth       = kInterleaveDepth;
    d.fps         = sat8(b.fps);
    d.txPowerDbm  = txpowerDbmForMcs(cfg.txPowerCurve, d.mcs);   // per-MCS PA-linearity backoff curve
}

} // namespace fpvd::dynlink
