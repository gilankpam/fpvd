/* test_dl_dedup.cpp — sequence dedup + reset semantics (ported from test_dedup.c). */
#include "doctest.h"
#include "dynlink/dedup.hpp"
using namespace fpvd::dynlink;

TEST_CASE("dedup accepts fresh, drops stale/equal, reset reseeds") {
    Dedup d;
    CHECK(d.check(10) == false); // first accept
    CHECK(d.check(10) == true);  // equal -> drop
    CHECK(d.check(9) == true);   // older -> drop
    CHECK(d.check(11) == false); // newer -> accept
    d.reset();
    CHECK(d.check(5) == false); // post-reset accepts unconditionally
}

TEST_CASE("dedup: first packet always accepted, exact dup rejected") {
    Dedup d;
    CHECK(d.check(42) == false); // fresh seed
    CHECK(d.check(42) == true);  // exact dup
}

TEST_CASE("dedup: monotonic sequence accepted") {
    Dedup d;
    d.check(10);
    CHECK(d.check(11) == false);
    CHECK(d.check(12) == false);
    CHECK(d.check(100) == false);
}

TEST_CASE("dedup: older seq rejected") {
    Dedup d;
    d.check(100);
    CHECK(d.check(99) == true);
    CHECK(d.check(50) == true);
    CHECK(d.check(1) == true);
}

TEST_CASE("dedup: reset accepts lower seq (real-world GS restart recovery)") {
    /* Real-world trigger: drone learns seq=100200 from an injected
     * test, then GS restarts and emits seq=1884. Without reset, the
     * applier never recovers. */
    Dedup d;
    d.check(100200);
    CHECK(d.check(1884) == true); // would-be stale: rejected
    d.reset();
    CHECK(d.check(1884) == false); // post-reset: fresh seed
    CHECK(d.check(1885) == false);
    CHECK(d.check(1884) == true); // and dup logic resumes
}

TEST_CASE("dedup: uint32 wrap handled via signed delta") {
    /* uint32_t wrap: seq 2^32-5 → seq 10 should be treated as forward
     * progress (signed delta ≈ +15). */
    Dedup d;
    d.check(0xFFFFFFFBu);                // near-max
    CHECK(d.check(10) == false);         // delta = +15, accept
    CHECK(d.check(0xFFFFFFFBu) == true); // now genuinely stale
}
