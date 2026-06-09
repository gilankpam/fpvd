/* test_dl_txpower_resolve.cpp — per-radio default registry + override resolver. */
#include "doctest.h"
#include "dynlink/txpower_curve.hpp"
using namespace fpvd::dynlink;

TEST_CASE("resolveTxpowerCurve: bl-m8812eu2 default when no override") {
    auto r = resolveTxpowerCurve(std::nullopt, std::string("bl-m8812eu2"), "8812eu");
    CHECK(r.source == "bl-m8812eu2");
    CHECK(r.curve == TxPowerCurve{29,28,25,23,19,19,19,19});
}

TEST_CASE("resolveTxpowerCurve: explicit override wins and reports source override") {
    std::vector<int> ov{10,10,10,10,10,10,10,10};
    auto r = resolveTxpowerCurve(ov, std::string("bl-m8812eu2"), "8812eu");
    CHECK(r.source == "override");
    CHECK(r.curve == TxPowerCurve{10,10,10,10,10,10,10,10});
}

TEST_CASE("resolveTxpowerCurve: unknown radio falls back") {
    auto r = resolveTxpowerCurve(std::nullopt, std::nullopt, "8733bu");
    CHECK(r.source == "fallback");
    CHECK(r.curve[7] <= 20);   // conservative tail
}

TEST_CASE("txpowerDbmForMcs clamps mcs into the curve") {
    TxPowerCurve cv{29,28,25,23,19,19,19,19};
    CHECK(txpowerDbmForMcs(cv, -1) == 29);
    CHECK(txpowerDbmForMcs(cv, 3)  == 23);
    CHECK(txpowerDbmForMcs(cv, 99) == 19);
}
