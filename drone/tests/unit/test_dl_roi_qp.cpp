/* test_dl_roi_qp.cpp — port of test_roi_qp.c vectors verbatim.
 * Defaults: threshold=6000, anchor=2000, floor=-24, step=3. */
#include "doctest.h"
#include "dynlink/roi_qp.hpp"
using namespace fpvd::dynlink;

TEST_CASE("roi qp above threshold is zero") {
    CHECK(computeRoiQp(6000, 6000, 2000, -24, 3) == 0);
    CHECK(computeRoiQp(10000, 6000, 2000, -24, 3) == 0);
}

TEST_CASE("roi qp at low anchor is floor") { CHECK(computeRoiQp(2000, 6000, 2000, -24, 3) == -24); }

TEST_CASE("roi qp below low anchor clamps at floor") {
    CHECK(computeRoiQp(1500, 6000, 2000, -24, 3) == -24);
    CHECK(computeRoiQp(500, 6000, 2000, -24, 3) == -24);
    CHECK(computeRoiQp(0, 6000, 2000, -24, 3) == -24);
}

TEST_CASE("roi qp midpoint ramps linearly") {
    // Defaults: threshold=6000, anchor=2000, floor=-24, step=3.
    // At bitrate=4000: span=4000, delta=2000, raw=-24*2000/4000=-12.
    // -12 is a multiple of 3, so quantized=-12.
    CHECK(computeRoiQp(4000, 6000, 2000, -24, 3) == -12);
    // At 5000: span=4000, delta=3000, raw=-24*1000/4000=-6.
    CHECK(computeRoiQp(5000, 6000, 2000, -24, 3) == -6);
    // At 3000: span=4000, delta=1000, raw=-24*3000/4000=-18.
    CHECK(computeRoiQp(3000, 6000, 2000, -24, 3) == -18);
}

TEST_CASE("roi qp quantization lands on step multiples") {
    // Sweep 2000..6000 in 50-kbps increments; every result must be a
    // multiple of step (3) and must be in [floor, 0].
    for (int br = 2000; br <= 6000; br += 50) {
        int q = computeRoiQp(static_cast<uint16_t>(br), 6000, 2000, -24, 3);
        CHECK(q <= 0);
        CHECK(q >= -24);
        CHECK(q % 3 == 0);
    }
}

TEST_CASE("roi qp custom config") {
    // threshold=8000, anchor=3000, floor=-18, step=2.
    // Endpoints
    CHECK(computeRoiQp(8000, 8000, 3000, -18, 2) == 0);
    CHECK(computeRoiQp(3000, 8000, 3000, -18, 2) == -18);
    // Midpoint: span=5000, delta=2500, raw=-18*2500/5000=-9.
    // Quantize step=2: -9/2=-4 (C truncates toward zero), *2=-8.
    CHECK(computeRoiQp(5500, 8000, 3000, -18, 2) == -8);
}
