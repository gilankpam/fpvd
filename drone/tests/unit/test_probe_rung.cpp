#include "doctest.h"
#include "dynlink/controller.hpp"
using fpvd::dynlink::DynamicLinkController;

TEST_CASE("probe rung tracks current+1, clamped to ceiling") {
    CHECK(DynamicLinkController::probeRungFor(2, 7) == 3);
    CHECK(DynamicLinkController::probeRungFor(5, 7) == 6);
    CHECK(DynamicLinkController::probeRungFor(6, 7) == 7);
    CHECK(DynamicLinkController::probeRungFor(7, 7) == 7); // ceiling
    CHECK(DynamicLinkController::probeRungFor(9, 7) == 7); // above ceiling
}

TEST_CASE("osd write throttle gates to osdUpdateIntervalMs") {
    using DLC = DynamicLinkController;
    // Never written yet (lastMs==0): always due, regardless of clock.
    CHECK(DLC::osdWriteDue(/*now=*/0, /*last=*/0, /*interval=*/1000));
    CHECK(DLC::osdWriteDue(/*now=*/50000, /*last=*/0, /*interval=*/1000));

    // Decisions arrive ~10 Hz (100 ms apart): all but the boundary are throttled.
    CHECK_FALSE(DLC::osdWriteDue(10100, 10000, 1000)); // 100 ms elapsed
    CHECK_FALSE(DLC::osdWriteDue(10999, 10000, 1000)); // 999 ms — still throttled
    CHECK(DLC::osdWriteDue(11000, 10000, 1000));       // exactly 1 s — due
    CHECK(DLC::osdWriteDue(12500, 10000, 1000));       // well past — due
}
