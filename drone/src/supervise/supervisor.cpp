#include "supervise/supervisor.hpp"
#include <algorithm>
#include <deque>

namespace fpvd {

Supervisor::Supervisor(SupervisedSpec spec, int backoffStartMs,
                        int failureCap, std::chrono::seconds failureWindow)
    : spec_(std::move(spec)), backoffStartMs_(backoffStartMs),
      failureCap_(failureCap), failureWindow_(failureWindow) {}

Supervisor::~Supervisor() { shutdown(); }

void Supervisor::start() {
    stopFlag_.store(false);
    thr_ = std::thread([this]{ loop(); });
}

void Supervisor::shutdown() {
    if (!thr_.joinable()) return;
    stopFlag_.store(true);
    if (proc_) proc_->stop(std::chrono::seconds(5));
    thr_.join();
    state_.store(ProcState::Stopped);
}

pid_t Supervisor::pid() const {
    return proc_ ? proc_->pid() : -1;
}
std::optional<int> Supervisor::lastExitCode() const {
    return proc_ ? proc_->lastExitCode() : std::nullopt;
}

void Supervisor::loop() {
    int backoffMs = backoffStartMs_;
    std::deque<std::chrono::steady_clock::time_point> recentFailures;
    while (!stopFlag_.load()) {
        proc_ = std::make_unique<Process>(spec_.argv, spec_.env);
        proc_->start();
        if (proc_->state() != ProcState::Running) {
            state_.store(ProcState::Failed);
            return;
        }
        state_.store(ProcState::Running);
        auto startedAt = std::chrono::steady_clock::now();
        // Poll until exit or shutdown.
        while (!stopFlag_.load()) {
            if (proc_->reapIfReady()) break;
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (stopFlag_.load()) return;
        auto exitedAt = std::chrono::steady_clock::now();
        auto uptime = std::chrono::duration_cast<std::chrono::seconds>(
            exitedAt - startedAt);

        if (spec_.restart == RestartPolicy::Never) {
            state_.store(ProcState::Exited);
            return;
        }
        if (spec_.restart == RestartPolicy::OnFailure
            && proc_->lastExitCode().value_or(1) == 0) {
            state_.store(ProcState::Exited);
            return;
        }

        // Record failure; trim outside window.
        recentFailures.push_back(exitedAt);
        while (!recentFailures.empty() &&
               (exitedAt - recentFailures.front()) > failureWindow_) {
            recentFailures.pop_front();
        }
        restarts_.fetch_add(1);
        if ((int)recentFailures.size() >= failureCap_) {
            state_.store(ProcState::Failed);
            return;
        }

        // Reset backoff after sustained uptime.
        if (uptime >= std::chrono::seconds(60)) {
            backoffMs = backoffStartMs_;
        } else {
            // Double, cap at 30s.
            backoffMs = std::min(backoffMs * 2, 30000);
        }
        // Sleep in small chunks so shutdown is responsive.
        auto wakeAt = std::chrono::steady_clock::now()
                      + std::chrono::milliseconds(backoffMs);
        while (!stopFlag_.load()
               && std::chrono::steady_clock::now() < wakeAt) {
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }
}

} // namespace fpvd
