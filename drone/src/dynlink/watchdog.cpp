/* watchdog.cpp — port of dl_watchdog.c */
#include "dynlink/watchdog.hpp"

namespace fpvd::dynlink {

Watchdog::Watchdog(uint32_t timeoutMs)
    : timeoutMs_(timeoutMs) {}

void Watchdog::setTimeout(uint32_t timeoutMs) {
    timeoutMs_ = timeoutMs;
}

void Watchdog::notifyDecision(uint64_t nowMs) {
    lastDecisionMs_ = nowMs;
    everSeen_ = true;
    tripped_ = false;
}

bool Watchdog::tick(uint64_t nowMs) {
    if (!everSeen_) return false;  /* no reference point yet */
    if (tripped_) return false;
    if (nowMs >= lastDecisionMs_ &&
        (nowMs - lastDecisionMs_) >= timeoutMs_) {
        tripped_ = true;
        return true;
    }
    return false;
}

} // namespace fpvd::dynlink
