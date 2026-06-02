#pragma once
#include "supervise/process.hpp"
#include <atomic>
#include <chrono>
#include <map>
#include <memory>
#include <string>
#include <thread>
#include <vector>

namespace fpvd {

enum class RestartPolicy { Always, OnFailure, Never };

struct SupervisedSpec {
    std::string name;
    std::vector<std::string> argv;
    std::map<std::string, std::string> env;
    RestartPolicy restart{RestartPolicy::Always};
    std::vector<std::string> startAfter{};
};

class Supervisor {
public:
    Supervisor(SupervisedSpec spec,
                int backoffStartMs = 1000,
                int failureCap = 5,
                std::chrono::seconds failureWindow = std::chrono::seconds(60));
    ~Supervisor();

    void start();             // Begin supervision loop in background thread.
    void shutdown();          // Stop child and join thread.

    const std::string& name() const { return spec_.name; }
    ProcState state() const { return state_.load(); }
    int restartCount() const { return restarts_.load(); }
    std::optional<int> lastExitCode() const;
    pid_t pid() const;

private:
    void loop();

    SupervisedSpec spec_;
    int backoffStartMs_;
    int failureCap_;
    std::chrono::seconds failureWindow_;

    std::unique_ptr<Process> proc_;
    std::thread thr_;
    std::atomic<bool> stopFlag_{false};
    std::atomic<ProcState> state_{ProcState::Stopped};
    std::atomic<int> restarts_{0};
};

} // namespace fpvd
