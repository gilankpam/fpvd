/* test_dl_txpower_curve.cpp — per-MCS tx power table (bl-m8812eu2 level 4). */
#include "doctest.h"
#include "dynlink/txpower_curve.hpp"
using namespace fpvd::dynlink;

TEST_CASE("txpowerDbmForMcs returns the bl-m8812eu2 level-4 curve") {
    CHECK(txpowerDbmForMcs(0) == 29);
    CHECK(txpowerDbmForMcs(1) == 28);
    CHECK(txpowerDbmForMcs(2) == 25);
    CHECK(txpowerDbmForMcs(3) == 23);
    CHECK(txpowerDbmForMcs(4) == 19);
    CHECK(txpowerDbmForMcs(5) == 19);
    CHECK(txpowerDbmForMcs(6) == 19);
    CHECK(txpowerDbmForMcs(7) == 19);
}

TEST_CASE("txpowerDbmForMcs clamps out-of-range mcs") {
    CHECK(txpowerDbmForMcs(-1) == 29);  // clamps to MCS0
    CHECK(txpowerDbmForMcs(8)  == 19);  // clamps to MCS7
    CHECK(txpowerDbmForMcs(99) == 19);
}
