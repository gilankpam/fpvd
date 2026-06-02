#pragma once
#include <chrono>
#include <map>
#include <optional>
#include <string>
#include <sys/types.h>
#include <vector>

namespace fpvd {

enum class ProcState { Stopped, Starting, Running, Exited, Failed };

class Process {
public:
    explicit Process(std::vector<std::string> argv,
                     std::map<std::string, std::string> env = {});

    // Spawn the child. Sets state to Running on success.
    void start();

    // Send SIGTERM, then SIGKILL after `gracePeriod` if still alive.
    // Returns once kill has been signaled (does not wait for reap).
    void stop(std::chrono::milliseconds gracePeriod);

    // Block until the child exits or `timeout` elapses. Returns true if
    // the child reaped, false on timeout.
    bool waitFor(std::chrono::milliseconds timeout);

    // Drain SIGCHLD: returns true if child has exited and updates
    // lastExitCode().
    bool reapIfReady();

    pid_t pid() const { return pid_; }
    ProcState state() const { return state_; }
    std::optional<int> lastExitCode() const { return lastExitCode_; }
    const std::vector<std::string>& argv() const { return argv_; }

private:
    std::vector<std::string> argv_;
    std::map<std::string, std::string> env_;
    pid_t pid_{-1};
    ProcState state_{ProcState::Stopped};
    std::optional<int> lastExitCode_{};
};

} // namespace fpvd
