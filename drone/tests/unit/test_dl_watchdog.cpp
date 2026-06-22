/* test_dl_watchdog.cpp — GS-link health watchdog (ported from test_watchdog.c). */
#include "doctest.h"
#include "dynlink/watchdog.hpp"
using namespace fpvd::dynlink;

TEST_CASE("watchdog trips once per silent window") {
    Watchdog w(1000); // 1000 ms timeout
    w.notifyDecision(0);
    CHECK(w.tick(500) == false);  // within window
    CHECK(w.tick(1500) == true);  // first stale tick -> trip (one-shot)
    CHECK(w.tick(2000) == false); // still silent -> no re-trip
    CHECK(w.isTripped() == true);
    w.notifyDecision(2500); // fresh decision clears latch
    CHECK(w.isTripped() == false);
}

TEST_CASE("watchdog: quiet before first decision") {
    Watchdog w(1000);
    /* No decision yet — ticks never trip. */
    CHECK(w.tick(2000) == false);
    CHECK(w.tick(5000) == false);
    CHECK(w.isTripped() == false);
}

TEST_CASE("watchdog: fires after timeout") {
    Watchdog w(1000);
    w.notifyDecision(10000);
    CHECK(w.tick(10500) == false); /* 500 < 1000 */
    CHECK(w.tick(10999) == false); /* still inside */
    CHECK(w.tick(11000) == true);  /* exactly at timeout */
    CHECK(w.isTripped() == true);
}

TEST_CASE("watchdog: fires exactly once while silent") {
    Watchdog w(500);
    w.notifyDecision(0);
    CHECK(w.tick(500) == true);
    /* No more pushes while still silent. */
    CHECK(w.tick(1000) == false);
    CHECK(w.tick(2000) == false);
    CHECK(w.tick(60000) == false);
}

TEST_CASE("watchdog: reset clears latch and re-arms") {
    Watchdog w(500);
    w.notifyDecision(0);
    CHECK(w.tick(500) == true);
    /* Fresh decision clears tripped flag. */
    w.notifyDecision(600);
    CHECK(w.isTripped() == false);
    /* And the next silent window will fire again. */
    CHECK(w.tick(1200) == true);
}

TEST_CASE("watchdog: setTimeout updates timeout without clearing latch") {
    Watchdog w(1000);
    w.notifyDecision(0);
    CHECK(w.tick(500) == false); /* within the original 1000 ms window */

    /* Shorten the timeout — 300 ms > 200 ms, so now stale. */
    w.setTimeout(200);
    CHECK(w.tick(300) == true); /* first stale tick under new timeout -> trip */

    /* setTimeout must NOT clear an existing latch. */
    w.setTimeout(5000);
    CHECK(w.isTripped() == true); /* still tripped */
    CHECK(w.tick(400) == false);  /* one-shot already fired, stays silent */
}
