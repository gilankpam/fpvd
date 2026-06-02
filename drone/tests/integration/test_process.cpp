#include "doctest.h"
#include "supervise/process.hpp"
#include <thread>
#include <chrono>

using namespace std::chrono_literals;

TEST_CASE("process: start a child that exits 0, observe clean exit") {
    fpvd::Process p({"/bin/sh", "-c", "exit 0"});
    p.start();
    auto ok = p.waitFor(2s);
    REQUIRE(ok);
    CHECK(p.state() == fpvd::ProcState::Exited);
    CHECK(p.lastExitCode() == 0);
}

TEST_CASE("process: start a child that exits 1, observe nonzero exit") {
    fpvd::Process p({"/bin/sh", "-c", "exit 1"});
    p.start();
    REQUIRE(p.waitFor(2s));
    CHECK(p.lastExitCode() == 1);
}

TEST_CASE("process: SIGTERM stops a sleeper") {
    fpvd::Process p({"/bin/sh", "-c", "sleep 30"});
    p.start();
    std::this_thread::sleep_for(100ms);
    CHECK(p.state() == fpvd::ProcState::Running);
    p.stop(2s);  // SIGTERM then SIGKILL after 2s
    REQUIRE(p.waitFor(3s));
    CHECK(p.state() == fpvd::ProcState::Exited);
}

TEST_CASE("process: SIGKILL fallback when child ignores SIGTERM") {
    fpvd::Process p({"/bin/sh", "-c", "trap '' TERM; sleep 30"});
    p.start();
    std::this_thread::sleep_for(100ms);
    p.stop(500ms);  // expect SIGKILL after 500ms
    REQUIRE(p.waitFor(2s));
    CHECK(p.state() == fpvd::ProcState::Exited);
}
