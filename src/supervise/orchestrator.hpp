#pragma once
#include "supervise/supervisor.hpp"
#include <chrono>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace fpvd {

struct OrchestrationError : std::runtime_error {
    using std::runtime_error::runtime_error;
};

class Orchestrator {
public:
    void add(SupervisedSpec spec);
    void remove(const std::string& name);  // shuts down if running
    // Bounce one process (shutdown + start); no-op if absent. Blocks the caller
    // for up to the Supervisor SIGTERM grace (~5s) if the child ignores SIGTERM.
    // `settle` is an extra pause inserted AFTER the old process is reaped and
    // BEFORE the new one is forked, so kernel/driver state from the old process
    // can drain first. Required on Star6E for a video0.size change: the
    // SigmaStar VENC/VPE driver releases pipeline state asynchronously after the
    // process exits, and a fresh waybeam that re-inits a different-geometry
    // pipeline too early fails to create the VENC channel (frozen video).
    // waybeam's own respawn waits 500 ms here for the same reason.
    void restart(const std::string& name,
                 std::chrono::milliseconds settle = std::chrono::milliseconds{0});
    Supervisor* get(const std::string& name);

    // Topologically ordered names. Throws OrchestrationError on cycle.
    std::vector<std::string> startOrder() const;
    std::vector<std::string> stopOrder() const;

    // Start every supervised process in order.
    void startAll();
    // Stop every supervised process in reverse order.
    void stopAll();

    // Convenience: list every name.
    std::vector<std::string> names() const;

private:
    std::map<std::string, std::unique_ptr<Supervisor>> sups_;
    std::map<std::string, SupervisedSpec> specs_;
};

} // namespace fpvd
