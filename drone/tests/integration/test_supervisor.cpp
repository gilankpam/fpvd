#include "doctest.h"
#include "supervise/supervisor.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

TEST_CASE("supervisor: restarts a fast-crashing child") {
    fpvd::SupervisedSpec spec{
        "crashy",
        {"/bin/sh", "-c", "exit 1"},
        {},
        fpvd::RestartPolicy::Always,
        {}  // startAfter
    };
    fpvd::Supervisor sup(spec, /*backoffStartMs=*/50,
                          /*failureCap=*/3,
                          /*failureWindow=*/std::chrono::seconds(5));
    sup.start();
    // Wait for the failure cap to trip.
    std::this_thread::sleep_for(2s);
    CHECK(sup.state() == fpvd::ProcState::Failed);
    CHECK(sup.restartCount() >= 3);
    sup.shutdown();
}

TEST_CASE("supervisor: keeps a long-running child running across crash") {
    fpvd::SupervisedSpec spec{
        "blip",
        {"/bin/sh", "-c", "sleep 0.1; exit 0"},
        {}, fpvd::RestartPolicy::Always, {}
    };
    fpvd::Supervisor sup(spec, 50, 100, 60s);
    sup.start();
    // After 400ms, the child should have crashed and restarted multiple
    // times, but not be in Failed state (failure cap not hit).
    std::this_thread::sleep_for(400ms);
    CHECK(sup.state() != fpvd::ProcState::Failed);
    CHECK(sup.restartCount() > 0);
    sup.shutdown();
}

TEST_CASE("supervisor: shutdown stops the child") {
    fpvd::SupervisedSpec spec{
        "sleeper", {"/bin/sh", "-c", "sleep 30"}, {},
        fpvd::RestartPolicy::Always, {}
    };
    fpvd::Supervisor sup(spec, 50, 5, 60s);
    sup.start();
    std::this_thread::sleep_for(100ms);
    CHECK(sup.state() == fpvd::ProcState::Running);
    sup.shutdown();
    CHECK(sup.state() == fpvd::ProcState::Stopped);
}
