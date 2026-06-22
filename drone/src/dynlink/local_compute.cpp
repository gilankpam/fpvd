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
    // Channel width (10/20/40) drives the rate table — NOT d.bandwidth, which is
    // the modulation width (20 for a 10 MHz link) and would mis-bill 10 MHz as 20.
    double wireTarget =
        computeWireTargetKbps(cfg.linkWidthMhz, d.mcs, cfg.probeMcsCeiling, probeKbps);
    auto sat8 = [](int x) -> uint8_t {
        if (x < 0)
            x = 0;
        if (x > 255)
            x = 255;
        return static_cast<uint8_t>(x);
    };
    if (cfg.swfec) {
        // swfec: FEC is static config — the k/n Decision slots carry
        // overhead_pct/deadline_ms (pushed via CMD_SET_FEC on change only).
        d.k = cfg.swfecOverheadPct;
        d.n = cfg.swfecDeadlineMs;
        d.bitrateKbps = computeBitrateKbpsSwfec(wireTarget, cfg.swfecOverheadPct, b.minBitrateKbps,
                                                b.maxBitrateKbps);
    } else {
        int k = computeK(wireTarget, b.mtuBytes, b.fps, b.baseRedundancyRatio, b.blocksPerFrame,
                         b.kMin, b.kMax);
        int n = computeN(k, b.baseRedundancyRatio);
        d.k = sat8(k);
        d.n = sat8(n);
        d.bitrateKbps = computeBitrateKbps(wireTarget, k, n, b.minBitrateKbps, b.maxBitrateKbps);
    }
    d.fps = sat8(b.fps);
    d.txPowerDbm = txpowerDbmForMcs(d.mcs); // per-MCS PA-linearity backoff curve
}

} // namespace fpvd::dynlink
