/* test_dl_txpower_curve.cpp — bl-m8812eu2 level-4 default curve constant. */
#include "doctest.h"
#include "dynlink/txpower_curve.hpp"
using namespace fpvd::dynlink;

TEST_CASE("kCurveM8812eu2 is the bl-m8812eu2 level-4 curve") {
    CHECK(kCurveM8812eu2 == TxPowerCurve{29,28,25,23,19,19,19,19});
    CHECK(txpowerDbmForMcs(kCurveM8812eu2, 0) == 29);
    CHECK(txpowerDbmForMcs(kCurveM8812eu2, 4) == 19);
}
