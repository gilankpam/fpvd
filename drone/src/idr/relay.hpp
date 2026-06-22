#pragma once
#include "idr/idr_listen.hpp"
#include "waybeam/client.hpp"
#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

namespace fpvd::idr {

/* Always-on IDR keyframe-request relay.
 *
 * Owns a UDP IdrListener and turns each received burst into a throttled
 * `GET /request/idr` against the waybeam encoder — independent of the dynamic
 * link controller, so keyframe requests work whether dynamicLink is enabled or
 * not. The waybeam transport is injected (non-owning; it must outlive the
 * relay; WaybeamClient is documented thread-safe). Runs its own poll thread.
 *
 * port == 0 disables the relay entirely (no socket, no thread).
 */
class IdrRelay {
  public:
    IdrRelay(WaybeamClient& enc, std::string bindAddr, uint16_t port, uint32_t minIntervalMs);
    ~IdrRelay();
    IdrRelay(const IdrRelay&) = delete;
    IdrRelay& operator=(const IdrRelay&) = delete;

    // Spawn the poll thread. No-op if the listener is disabled (port 0) or the
    // thread is already running.
    void start();
    // Signal + join the poll thread. Idempotent.
    void stop();

    // Count of received IDR bursts forwarded toward the encoder (one per
    // drain-with-data). This is what the OSD "I" counter renders; it counts
    // received requests, not post-throttle sends.
    uint64_t count() const { return count_.load(std::memory_order_relaxed); }

    // Send one throttled IDR request. Public so the throttle is directly unit
    // testable. Returns 0 sent, 1 throttled, -1 transport failure. The throttle
    // arms on any attempt (including failure). nowMs is a monotonic clock.
    int requestIdr(uint64_t nowMs);

  private:
    void run();

    IdrListener listener_;
    WaybeamClient* enc_;
    uint32_t minIntervalMs_;

    std::atomic<uint64_t> count_{0};
    std::atomic<bool> stop_{false};
    int wakeFd_{-1}; // eventfd: interrupts poll() on stop()
    std::thread thread_;

    bool everSent_{false};
    uint64_t lastIdrMs_{0};
};

} // namespace fpvd::idr
