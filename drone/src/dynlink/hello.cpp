/* hello.cpp — drone-side HELLO state machine (C++ port of dl_hello.c).
 *
 * Ports the state-machine logic faithfully.  The only intentional deviation:
 * MTU, FPS, and generationId are injected by the constructor rather than
 * read from /etc/wfb.yaml and /etc/majestic.yaml — fpvd is authoritative
 * for these values.
 */
#include "dynlink/hello.hpp"

namespace fpvd::dynlink {

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------

HelloSm::HelloSm(uint32_t generationId, uint16_t mtuBytes, uint16_t fps,
                 HelloCadence cad)
    : generationId_(generationId)
    , mtu_(mtuBytes)
    , fps_(fps)
    , cad_(cad)
{
    // Mirror dl_hello_init: if mtu or fps are invalid (0), stay DISABLED.
    if (mtuBytes == 0 || fps == 0) {
        state_ = HelloState::Disabled;
    } else {
        state_ = HelloState::Announcing;
    }
}

// ---------------------------------------------------------------------------
// buildAnnounce — mirrors dl_hello_build_announce
// ---------------------------------------------------------------------------

size_t HelloSm::buildAnnounce(uint8_t* buf, size_t buflen) {
    if (state_ == HelloState::Disabled) return 0;

    Hello pkt{};
    pkt.version        = kWireVersion;
    pkt.flags          = flags_;
    pkt.generationId   = generationId_;
    pkt.mtuBytes       = mtu_;
    pkt.fps            = fps_;
    pkt.applierBuildSha = 0;

    size_t n = encodeHello(pkt, buf, buflen);
    if (n == 0) return 0;

    announceCount_++;
    return n;
}

// ---------------------------------------------------------------------------
// nextDelayMs — mirrors dl_hello_next_delay_ms
// ---------------------------------------------------------------------------

uint32_t HelloSm::nextDelayMs() const {
    if (state_ == HelloState::Disabled) return 0;
    if (state_ == HelloState::Keepalive) return cad_.keepaliveMs;

    // ANNOUNCING:
    if (announceCount_ == 0) return 0;  // first fire is immediate
    if (announceCount_ < cad_.announceInitialCount) return cad_.announceInitialMs;
    return cad_.announceSteadyMs;
}

// ---------------------------------------------------------------------------
// onAck — mirrors dl_hello_on_ack
// ---------------------------------------------------------------------------

bool HelloSm::onAck(const HelloAck& ack) {
    if (state_ == HelloState::Disabled) return false;
    if (ack.generationIdEcho != generationId_) return false;

    state_                = HelloState::Keepalive;
    keepalivesWithoutAck_ = 0;
    announceCount_        = 0;
    return true;
}

// ---------------------------------------------------------------------------
// onKeepaliveTick — mirrors dl_hello_on_keepalive_tick
// ---------------------------------------------------------------------------

bool HelloSm::onKeepaliveTick() {
    if (state_ != HelloState::Keepalive) return false;

    keepalivesWithoutAck_++;
    if (keepalivesWithoutAck_ >= 3) {
        state_                = HelloState::Announcing;
        announceCount_        = 0;
        keepalivesWithoutAck_ = 0;
        return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// setMtuFps — hot-reconcile (fpvd-specific)
// ---------------------------------------------------------------------------

void HelloSm::setMtuFps(uint16_t mtu, uint16_t fps) {
    mtu_ = mtu;
    fps_ = fps;
}

// ---------------------------------------------------------------------------
// setVanilla — hot-reconcile (fpvd-specific)
// ---------------------------------------------------------------------------

void HelloSm::setVanilla(bool vanilla) {
    uint8_t newFlags = flags_;
    if (vanilla) {
        newFlags |= kHelloFlagVanillaWfbNg;
    } else {
        newFlags &= static_cast<uint8_t>(~kHelloFlagVanillaWfbNg);
    }

    if (newFlags != flags_) {
        flags_ = newFlags;
        // Flag changed: GS needs to relearn the capability, so drop back to
        // ANNOUNCING (reset announce_count so the fast cadence fires first).
        if (state_ == HelloState::Keepalive || state_ == HelloState::Announcing) {
            state_         = HelloState::Announcing;
            announceCount_ = 0;
        }
    } else {
        flags_ = newFlags;
    }
}

} // namespace fpvd::dynlink
