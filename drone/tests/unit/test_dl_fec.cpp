/* test_dl_fec.cpp — block-fill compute_k + fixed-ratio compute_n (Phase 3a). */
#include "doctest.h"
#include "dynlink/fec.hpp"
using namespace fpvd::dynlink;

TEST_CASE("computeK sizes for block-fill at a typical wire target") {
    // wireTarget=28000 (MCS5/20 * 2/3), mtu=1500, fps=60, ratio=0.5, bpf=2.0
    // anchor = 28000/1.5 = 18666.67; ppf = 18666.67*1000/(60*1500*8) = 25.93;
    // k = trunc(25.93/2.0) = 12.
    CHECK(computeK(28000.0, 1500, 60, 0.5, 2.0, 2, 50) == 12);
}

TEST_CASE("computeK clamps to [kMin, kMax]") {
    CHECK(computeK(100.0, 1500, 60, 0.5, 2.0, 2, 50) == 2);       // tiny -> kMin
    CHECK(computeK(5.0e6, 1500, 60, 0.5, 2.0, 2, 50) == 50);      // huge -> kMax
    CHECK(computeK(0.0,  1500, 60, 0.5, 2.0, 2, 50) == 2);        // non-positive -> kMin
    CHECK(computeK(28000.0, 0, 60, 0.5, 2.0, 2, 50) == 2);        // bad mtu -> kMin
}

TEST_CASE("computeN is ceil(k * (1 + ratio))") {
    CHECK(computeN(12, 0.5) == 18);   // ceil(18.0) = 18
    CHECK(computeN(8, 0.5) == 12);    // ceil(12.0) = 12
    CHECK(computeN(7, 0.5) == 11);    // ceil(10.5) = 11
    CHECK(computeN(2, 0.5) == 3);     // ceil(3.0) = 3
}

TEST_CASE("computeK guards fps, blocksPerFrame, and ratio<=-1") {
    CHECK(computeK(28000.0, 1500, 0,   0.5,  2.0, 2, 50) == 2);   // fps=0 -> kMin
    CHECK(computeK(28000.0, 1500, 60,  0.5,  0.0, 2, 50) == 2);   // bpf=0 -> kMin
    CHECK(computeK(28000.0, 1500, 60, -1.0,  2.0, 2, 50) == 2);   // ratio=-1 (div0) -> kMin
}

TEST_CASE("computeN works at a second ratio") {
    CHECK(computeN(8, 0.25) == 10);   // ceil(10.0) = 10
    CHECK(computeN(0, 0.5)  == 0);    // k=0 contract -> 0
}
