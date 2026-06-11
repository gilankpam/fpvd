/* test_dl_local_compute.cpp — Phase 3a decision compose/override. */
#include "doctest.h"
#include "dynlink/local_compute.hpp"
#include "dynlink/bitrate.hpp"
#include "dynlink/fec.hpp"
#include "probe/probe_constants.hpp"
using namespace fpvd::dynlink;

static DlRuntimeConfig cfgWithBitrate() {
    DlRuntimeConfig c{};
    c.probeMcsCeiling = 7;
    c.bitrate = BitrateEngineConfig{0.5, 2.0, 2, 50, 1000, 24000, 1500, 60};
    return c;
}

TEST_CASE("applyLocalCompute overrides bitrate/k/n/depth/fps, keeps mcs/bw/txpower") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    Decision d{};
    d.mcs = 5; d.bandwidth = 20; d.txPowerDbm = 27;
    // GS-sent values that MUST be overridden:
    d.bitrateKbps = 9999; d.k = 99; d.n = 99; d.depth = 9; d.fps = 30;

    applyLocalCompute(cfg, d);

    double probeKbps = static_cast<double>(fpvd::kProbePps) * fpvd::kProbePacketBytes * 8.0 / 1000.0;
    double wt = computeWireTargetKbps(20, 5, 7, probeKbps);
    int    k  = computeK(wt, 1500, 60, 0.5, 2.0, 2, 50);
    int    n  = computeN(k, 0.5);

    CHECK(d.k == static_cast<uint8_t>(k));
    CHECK(d.n == static_cast<uint8_t>(n));
    CHECK(d.bitrateKbps == computeBitrateKbps(wt, k, n, 1000, 24000));
    CHECK(d.depth == kInterleaveDepth);     // constant 1
    CHECK(d.fps == 60);                     // drone video.fps, not the wire 30
    // untouched:
    CHECK(d.mcs == 5);
    CHECK(d.bandwidth == 20);
    CHECK(d.txPowerDbm == 19);   // now SET from the per-MCS curve (mcs5 -> 19 dBm)
}

TEST_CASE("applyLocalCompute is monotonic in mcs (higher rung -> higher bitrate)") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    Decision lo{}; lo.mcs = 2; lo.bandwidth = 20;
    Decision hi{}; hi.mcs = 5; hi.bandwidth = 20;
    applyLocalCompute(cfg, lo);
    applyLocalCompute(cfg, hi);
    CHECK(hi.bitrateKbps > lo.bitrateKbps);
}

TEST_CASE("applyLocalCompute floors k at kMin on the lowest rung") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    Decision d{}; d.mcs = 0; d.bandwidth = 20;
    applyLocalCompute(cfg, d);
    CHECK(d.k == 2);                 // computeK clamps to kMin at mcs0
    CHECK(d.n == 3);                 // ceil(2 * 1.5)
    CHECK(d.bitrateKbps > 1000);
}

TEST_CASE("applyLocalCompute at the probe ceiling (mcs == ceiling)") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    Decision d{}; d.mcs = 7; d.bandwidth = 20;
    applyLocalCompute(cfg, d);
    double pk = static_cast<double>(fpvd::kProbePps) * fpvd::kProbePacketBytes * 8.0 / 1000.0;
    double wt = computeWireTargetKbps(20, 7, 7, pk);   // probe rung clamps to 7
    int k = computeK(wt, 1500, 60, 0.5, 2.0, 2, 50);
    CHECK(d.k == static_cast<uint8_t>(k));
    CHECK(d.bitrateKbps == computeBitrateKbps(wt, k, computeN(k, 0.5), 1000, 24000));
}

TEST_CASE("applyLocalCompute sets txPowerDbm from the per-MCS curve") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    Decision d{};
    d.bandwidth = 20;

    d.mcs = 0; applyLocalCompute(cfg, d); CHECK(d.txPowerDbm == 29);
    d.mcs = 3; applyLocalCompute(cfg, d); CHECK(d.txPowerDbm == 23);
    d.mcs = 4; applyLocalCompute(cfg, d); CHECK(d.txPowerDbm == 19);
    d.mcs = 7; applyLocalCompute(cfg, d); CHECK(d.txPowerDbm == 19);
}

TEST_CASE("applyLocalCompute swfec: k/n slots carry overhead/deadline, bitrate de-rated") {
    DlRuntimeConfig cfg = cfgWithBitrate();
    cfg.swfec = true;
    cfg.swfecOverheadPct = 50;
    cfg.swfecDeadlineMs = 30;
    Decision d{};
    d.mcs = 2; d.bandwidth = 20;
    applyLocalCompute(cfg, d);
    CHECK(d.k == 50);   // overhead_pct
    CHECK(d.n == 30);   // deadline_ms
    // Cross-check the bitrate against the swfec formula at the same wire target.
    double probeKbps = fpvd::kProbePps * fpvd::kProbePacketBytes * 8.0 / 1000.0;
    double wt = computeWireTargetKbps(20, 2, cfg.probeMcsCeiling, probeKbps);
    CHECK(d.bitrateKbps == computeBitrateKbpsSwfec(wt, 50,
            cfg.bitrate.minBitrateKbps, cfg.bitrate.maxBitrateKbps));
}
