#pragma once
#include "dynlink/runtime_config.hpp"
#include <atomic>
#include <memory>
#include <mutex>
#include <thread>

namespace fpvd::dynlink {

class DynamicLinkController {
public:
    explicit DynamicLinkController(Endpoints ep = {});
    ~DynamicLinkController();
    DynamicLinkController(const DynamicLinkController&) = delete;
    DynamicLinkController& operator=(const DynamicLinkController&) = delete;

    void start(const DlRuntimeConfig& snap, uint32_t generationId);
    void stop();                                  // idempotent; joins the thread
    bool running() const { return running_.load(); }
    void setConfig(const DlRuntimeConfig& snap);  // hot reload (stub for now; Task 17 fills it)
    DlStatus status() const;                       // snapshot of published status

private:
    void run();                                    // the poll(2) loop (Tasks 14-17)
    void publishStatus(const DlStatus&);

    Endpoints ep_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopFlag_{false};
    int eventFd_{-1};                              // reload/stop wake
    uint32_t generationId_{0};
    mutable std::mutex cfgMu_;
    std::shared_ptr<const DlRuntimeConfig> cfg_;   // guarded by cfgMu_
    mutable std::mutex statusMu_;
    DlStatus status_{};
};

} // namespace fpvd::dynlink
