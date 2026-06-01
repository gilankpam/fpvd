#pragma once
#include <cstdint>

namespace fpvd::dynlink {

class Watchdog {
public:
    explicit Watchdog(uint32_t timeoutMs);
    void setTimeout(uint32_t timeoutMs);          // for hot reconcile (new)
    void notifyDecision(uint64_t nowMs);          // port dl_watchdog_notify_decision
    bool tick(uint64_t nowMs);                    // port dl_watchdog_tick (one-shot)
    bool isTripped() const { return tripped_; }
private:
    uint64_t lastDecisionMs_{0};
    uint32_t timeoutMs_{0};
    bool everSeen_{false};
    bool tripped_{false};
};

} // namespace fpvd::dynlink
