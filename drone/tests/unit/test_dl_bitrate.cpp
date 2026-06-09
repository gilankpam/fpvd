/* test_dl_bitrate.cpp — OpenIPC table + bitrate formula (Phase 3a). */
#include "doctest.h"
#include "dynlink/bitrate.hpp"
using namespace fpvd::dynlink;

TEST_CASE("openIpc base rate table endpoints") {
    CHECK(openIpcBaseRateKbps(20, 0) == 6500u);
    CHECK(openIpcBaseRateKbps(20, 7) == 55000u);
    CHECK(openIpcBaseRateKbps(40, 0) == 9800u);
    CHECK(openIpcBaseRateKbps(40, 7) == 97000u);
    CHECK(openIpcBaseRateKbps(20, 5) == 42000u);
    // unknown bandwidth / out-of-range mcs -> 0
    CHECK(openIpcBaseRateKbps(10, 0) == 0u);
    CHECK(openIpcBaseRateKbps(20, 8) == 0u);
    CHECK(openIpcBaseRateKbps(20, -1) == 0u);
}

TEST_CASE("wire target with no probe reserve matches OpenIPC 2/3") {
    // MCS0/20, probeKbps=0 -> 6500 * 2/3 = 4333.33 (parent design worked example)
    CHECK(computeWireTargetKbps(20, 0, 7, 0.0) == doctest::Approx(4333.333).epsilon(0.001));
}

TEST_CASE("wire target subtracts probe airtime at the probe rung") {
    // MCS0/20, probe at min(0+1,7)=1 -> baseRate=12000; probe_util=280/12000.
    double pu = 280.0 / 12000.0;
    double expect = 6500.0 * (2.0 / 3.0 - pu);
    CHECK(computeWireTargetKbps(20, 0, 7, 280.0) == doctest::Approx(expect).epsilon(0.001));
    CHECK(computeWireTargetKbps(20, 0, 7, 280.0) < computeWireTargetKbps(20, 0, 7, 0.0));
}

TEST_CASE("wire target clamps the utilization floor at zero") {
    // A huge probeKbps would drive (2/3 - probe_util) negative; clamp to 0.
    CHECK(computeWireTargetKbps(20, 0, 7, 1.0e9) == doctest::Approx(0.0));
}

TEST_CASE("bitrate truncates wire_target*k/n and clamps") {
    // 4333.33 * 8/12 = 2888.88 -> trunc 2888 (parent: 6500*4/9 = 2888).
    CHECK(computeBitrateKbps(4333.333, 8, 12, 1000, 24000) == 2888);
    // below the floor clamps up
    CHECK(computeBitrateKbps(500.0, 8, 12, 1000, 24000) == 1000);
    // above the ceiling clamps down
    CHECK(computeBitrateKbps(50000.0, 8, 12, 1000, 24000) == 24000);
}

TEST_CASE("wire target works at 40 MHz") {
    // 40MHz/MCS4 base = 55800; no probe -> 55800 * 2/3 = 37200.
    CHECK(computeWireTargetKbps(40, 4, 7, 0.0) == doctest::Approx(37200.0).epsilon(0.001));
}

TEST_CASE("bitrate floors to min when k or n is non-positive") {
    CHECK(computeBitrateKbps(5000.0, 0, 12, 1000, 24000) == 1000);
    CHECK(computeBitrateKbps(5000.0, 8, 0, 1000, 24000) == 1000);
}

TEST_CASE("wire target with probe rung saturated at the ceiling") {
    // mcs=7, ceiling=7 -> probe rung clamps to 7 (same as payload rung).
    // baseRate[20][7]=55000; probe_util = 280/55000.
    double expect = 55000.0 * (2.0 / 3.0 - 280.0 / 55000.0);
    CHECK(computeWireTargetKbps(20, 7, 7, 280.0) == doctest::Approx(expect).epsilon(0.001));
}
