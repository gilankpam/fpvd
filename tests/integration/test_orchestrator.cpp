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
