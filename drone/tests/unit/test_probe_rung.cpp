#include "doctest.h"
#include "dynlink/controller.hpp"
using fpvd::dynlink::DynamicLinkController;

TEST_CASE("probe rung tracks current+1, clamped to ceiling") {
    CHECK(DynamicLinkController::probeRungFor(2, 7) == 3);
    CHECK(DynamicLinkController::probeRungFor(5, 7) == 6);
    CHECK(DynamicLinkController::probeRungFor(6, 7) == 7);
    CHECK(DynamicLinkController::probeRungFor(7, 7) == 7);   // ceiling
    CHECK(DynamicLinkController::probeRungFor(9, 7) == 7);   // above ceiling
}
