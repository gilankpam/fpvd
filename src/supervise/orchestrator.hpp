#pragma once
#include "supervise/supervisor.hpp"
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
    void restart(const std::string& name);
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
