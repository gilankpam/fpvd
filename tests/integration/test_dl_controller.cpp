#include "doctest.h"
#include "dynlink/controller.hpp"
#include "dynlink/runtime_config.hpp"
#include <thread>
#include <chrono>
using namespace fpvd::dynlink;

static Endpoints ephemeral() {
    Endpoints e; e.listenPort = 45800; e.idrPort = 0;   // fixed test port; idr disabled
    return e;
}

TEST_CASE("controller starts and stops cleanly") {
    DlRuntimeConfig snap{};                 // zero-ish; fields not exercised here
    snap.healthTimeoutMs = 10000; snap.iface = "wlan0";
    DynamicLinkController c(ephemeral());
    c.start(snap, /*generationId=*/0x1234);
    CHECK(c.status().running == true);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    c.stop();
    CHECK(c.status().running == false);
    c.start(snap, 0x1234);                  // restartable
    CHECK(c.status().running == true);
    c.stop();
}
