/* test_dl_apply_direction.cpp — port of test_apply_stagger.c direction cases. */
#include "doctest.h"
#include "dynlink/apply_direction.hpp"
using namespace fpvd::dynlink;

TEST_CASE("apply direction first decision is equal") {
    // First decision (no prior state) skips staggering even if the
    // incoming bitrate differs from the zero-init baseline.
    CHECK(applyDirection(0,     12000, true) == ApplyDir::Equal);
    CHECK(applyDirection(8000,  0,     true) == ApplyDir::Equal);
}

TEST_CASE("apply direction equal bitrate is equal") {
    CHECK(applyDirection(8000, 8000, false) == ApplyDir::Equal);
    CHECK(applyDirection(0,    0,    false) == ApplyDir::Equal);
}

TEST_CASE("apply direction higher bitrate is up") {
    CHECK(applyDirection(8000, 12000, false) == ApplyDir::Up);
    CHECK(applyDirection(0,    1,     false) == ApplyDir::Up);
}

TEST_CASE("apply direction lower bitrate is down") {
    CHECK(applyDirection(12000, 8000, false) == ApplyDir::Down);
    CHECK(applyDirection(1,     0,    false) == ApplyDir::Down);
}

TEST_CASE("apply direction extremes") {
    // uint16_t boundaries — make sure no signed-comparison surprise.
    CHECK(applyDirection(65535, 0,     false) == ApplyDir::Down);
    CHECK(applyDirection(0,     65535, false) == ApplyDir::Up);
}
