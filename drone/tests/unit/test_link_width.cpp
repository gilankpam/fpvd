#include "doctest.h"
#include "link_width.hpp"

TEST_CASE("link_width: modulationWidth maps 10 to 20, others unchanged") {
    CHECK(fpvd::modulationWidth(10) == 20);
    CHECK(fpvd::modulationWidth(20) == 20);
    CHECK(fpvd::modulationWidth(40) == 40);
}
