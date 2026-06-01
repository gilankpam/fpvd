#include "doctest.h"
#include "supervise/orchestrator.hpp"
#include <thread>

using namespace std::chrono_literals;

TEST_CASE("orchestrator: starts processes in topological order") {
    fpvd::Orchestrator orch;
    orch.add({"a", {"/bin/sh", "-c", "sleep 5"}, {}, fpvd::RestartPolicy::Never, {}});
    orch.add({"b", {"/bin/sh", "-c", "sleep 5"}, {}, fpvd::RestartPolicy::Never, {"a"}});
    orch.add({"c", {"/bin/sh", "-c", "sleep 5"}, {}, fpvd::RestartPolicy::Never, {"b"}});

    auto order = orch.startOrder();
    REQUIRE(order.size() == 3);
    CHECK(order[0] == "a");
    CHECK(order[1] == "b");
    CHECK(order[2] == "c");

    orch.startAll();
    std::this_thread::sleep_for(100ms);
    CHECK(orch.get("a")->state() == fpvd::ProcState::Running);
    CHECK(orch.get("b")->state() == fpvd::ProcState::Running);
    CHECK(orch.get("c")->state() == fpvd::ProcState::Running);

    orch.stopAll();
}

TEST_CASE("orchestrator: stop order is reverse of start") {
    fpvd::Orchestrator orch;
    orch.add({"a", {"/bin/sh","-c","sleep 5"}, {}, fpvd::RestartPolicy::Never, {}});
    orch.add({"b", {"/bin/sh","-c","sleep 5"}, {}, fpvd::RestartPolicy::Never, {"a"}});
    auto stop = orch.stopOrder();
    REQUIRE(stop.size() == 2);
    CHECK(stop[0] == "b");
    CHECK(stop[1] == "a");
}

TEST_CASE("orchestrator: rejects cycles") {
    fpvd::Orchestrator orch;
    orch.add({"a", {"/bin/sh", "-c", "exit 0"}, {}, fpvd::RestartPolicy::Never, {"b"}});
    orch.add({"b", {"/bin/sh", "-c", "exit 0"}, {}, fpvd::RestartPolicy::Never, {"a"}});
    CHECK_THROWS_AS(orch.startOrder(), fpvd::OrchestrationError);
}

TEST_CASE("orchestrator: restart bounces one process, leaves others running") {
    fpvd::Orchestrator orch;
    orch.add({"a", {"/bin/sh", "-c", "sleep 30"}, {}, fpvd::RestartPolicy::Always, {}});
    orch.add({"b", {"/bin/sh", "-c", "sleep 30"}, {}, fpvd::RestartPolicy::Always, {}});
    orch.startAll();
    std::this_thread::sleep_for(100ms);

    pid_t aBefore = orch.get("a")->pid();
    pid_t bBefore = orch.get("b")->pid();
    REQUIRE(aBefore > 0);
    REQUIRE(bBefore > 0);

    orch.restart("a");
    std::this_thread::sleep_for(100ms);

    CHECK(orch.get("a")->state() == fpvd::ProcState::Running);
    CHECK(orch.get("a")->pid() != aBefore);     // new process
    CHECK(orch.get("b")->pid() == bBefore);     // untouched

    orch.restart("does-not-exist");             // no-op, must not throw

    orch.stopAll();
}

TEST_CASE("orchestrator: restart honors the settle delay and still bounces") {
    fpvd::Orchestrator orch;
    orch.add({"a", {"/bin/sh", "-c", "sleep 30"}, {}, fpvd::RestartPolicy::Always, {}});
    orch.startAll();
    std::this_thread::sleep_for(100ms);

    pid_t before = orch.get("a")->pid();
    REQUIRE(before > 0);

    auto t0 = std::chrono::steady_clock::now();
    orch.restart("a", std::chrono::milliseconds{300});   // settle 300ms
    auto elapsed = std::chrono::steady_clock::now() - t0;
    std::this_thread::sleep_for(100ms);

    CHECK(orch.get("a")->state() == fpvd::ProcState::Running);
    CHECK(orch.get("a")->pid() != before);               // still bounced
    CHECK(elapsed >= std::chrono::milliseconds{250});     // settle was applied

    orch.stopAll();
}
