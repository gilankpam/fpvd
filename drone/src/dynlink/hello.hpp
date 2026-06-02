#pragma once
/* hello.hpp — drone-side HELLO state machine (C++ port of dl_hello.h/c).
 *
 * KEY DIFFERENCE from the C version: MTU, FPS, and generationId are passed in
 * directly by the caller (fpvd is authoritative for these values).  No file
 * reads happen here.
 */
#include "dynlink/wire.hpp"
#include <cstddef>
#include <cstdint>

namespace fpvd::dynlink {

enum class HelloState { Init, Announcing, Keepalive, Disabled };

struct HelloCadence {
    uint32_t announceInitialMs{500};
    uint32_t announceSteadyMs{5000};
    uint32_t keepaliveMs{10000};
    uint32_t announceInitialCount{60};
};

class HelloSm {
public:
    // Constructor: starts in ANNOUNCING if mtu > 0 && fps > 0, else DISABLED.
    HelloSm(uint32_t generationId, uint16_t mtuBytes, uint16_t fps, HelloCadence cad);

    HelloState state() const { return state_; }

    // Build and encode a Hello announcement frame.  Returns bytes written (==
    // kHelloOnWire) or 0 if DISABLED or buflen too small.
    // Increments announceCount_ after a successful encode (mirrors C behaviour).
    size_t buildAnnounce(uint8_t* buf, size_t buflen);

    // Next timer delay in ms.  Returns 0 when:
    //   - state is DISABLED
    //   - state is ANNOUNCING and announceCount_ == 0  (fire immediately)
    uint32_t nextDelayMs() const;

    // Process an incoming HelloAck.  Returns true iff the ack matched our
    // generationId and the state was (or is now) KEEPALIVE.
    bool onAck(const HelloAck& ack);

    // Called each keepalive interval while in KEEPALIVE.  Increments the miss
    // counter; drops back to ANNOUNCING and returns true on the 3rd miss.
    bool onKeepaliveTick();

    // Hot-reconcile helpers (fpvd-specific, absent from the C version):
    // Update mtu/fps without changing generationId.  The caller is responsible
    // for deciding when to send the next announcement.
    void setMtuFps(uint16_t mtu, uint16_t fps);

    // Update the vanilla-wfb-ng capability flag.  If the flag bit changes, the
    // state machine drops back to ANNOUNCING so the GS relearns the capability.
    void setVanilla(bool vanilla);

private:
    HelloState state_{HelloState::Announcing};
    uint32_t   generationId_;
    uint16_t   mtu_;
    uint16_t   fps_;
    uint32_t   announceCount_{0};
    uint32_t   keepalivesWithoutAck_{0};
    uint8_t    flags_{0};
    HelloCadence cad_;
};

} // namespace fpvd::dynlink
