/* test_dl_hello.cpp — C++ port of test_dl_hello.c.
 * State-machine tests adapted for injected mtu/fps (no file reads).
 * All cases mirror the C test assertions exactly.
 */
#include "doctest.h"
#include "dynlink/hello.hpp"

using namespace fpvd::dynlink;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

// Cadence matching the C test setup:
//   hello_announce_initial_ms  = 500
//   hello_announce_steady_ms   = 5000
//   hello_keepalive_ms         = 10000
//   hello_announce_initial_count = 3
static HelloCadence testCadence() {
    HelloCadence c;
    c.announceInitialMs    = 500;
    c.announceSteadyMs     = 5000;
    c.keepaliveMs          = 10000;
    c.announceInitialCount = 3;
    return c;
}

// ---------------------------------------------------------------------------
// hello_announcing_first_delay_is_immediate
// ---------------------------------------------------------------------------

TEST_CASE("hello: first delay is immediate (announce_count == 0)") {
    // Mirror: hello_announcing_first_delay_is_immediate
    HelloSm h(/*generationId=*/0xABCD1234u, /*mtu=*/3994, /*fps=*/60, testCadence());
    CHECK(h.state() == HelloState::Announcing);
    // Before any build, announce_count == 0 → nextDelayMs returns 0
    CHECK(h.nextDelayMs() == 0u);
}

// ---------------------------------------------------------------------------
// hello_announcing_uses_initial_ms_for_first_retries
// ---------------------------------------------------------------------------

TEST_CASE("hello: announcing uses initial_ms for first retries, then steady_ms") {
    // Mirror: hello_announcing_uses_initial_ms_for_first_retries
    // announceInitialCount = 3:
    //   after 1 build → count=1 < 3 → 500 ms
    //   after 2 builds → count=2 < 3 → 500 ms
    //   after 3 builds → count=3 (not < 3) → 5000 ms
    HelloSm h(0xABCD1234u, 3994, 60, testCadence());
    uint8_t buf[kHelloOnWire];

    h.buildAnnounce(buf, sizeof(buf));
    CHECK(h.nextDelayMs() == 500u);

    h.buildAnnounce(buf, sizeof(buf));
    h.buildAnnounce(buf, sizeof(buf));
    CHECK(h.nextDelayMs() == 5000u);
}

// ---------------------------------------------------------------------------
// hello_ack_matching_genid_transitions_to_keepalive
// ---------------------------------------------------------------------------

TEST_CASE("hello: matching ACK transitions to KEEPALIVE") {
    // Mirror: hello_ack_matching_genid_transitions_to_keepalive
    HelloSm h(0xABCD1234u, 3994, 60, testCadence());
    HelloAck ack{};
    ack.magic           = kHelloAckMagic;
    ack.generationIdEcho = 0xABCD1234u;
    bool matched = h.onAck(ack);
    CHECK(matched == true);
    CHECK(h.state() == HelloState::Keepalive);
    CHECK(h.nextDelayMs() == 10000u);
}

// ---------------------------------------------------------------------------
// hello_ack_mismatching_genid_ignored
// ---------------------------------------------------------------------------

TEST_CASE("hello: mismatched generation ACK is ignored") {
    // Mirror: hello_ack_mismatching_genid_ignored
    HelloSm h(0xABCD1234u, 3994, 60, testCadence());
    HelloState origState = h.state();
    HelloAck bad{};
    bad.magic            = kHelloAckMagic;
    bad.generationIdEcho = 0xABCD1234u ^ 0xFFFFFFFFu;
    bool matched = h.onAck(bad);
    CHECK(matched == false);
    CHECK(h.state() == origState);
}

// ---------------------------------------------------------------------------
// hello_keepalive_without_ack_drops_back_to_announcing
// ---------------------------------------------------------------------------

TEST_CASE("hello: 3 keepalive ticks without ACK drops back to ANNOUNCING") {
    // Mirror: hello_keepalive_without_ack_drops_back_to_announcing
    HelloSm h(0xABCD1234u, 3994, 60, testCadence());
    HelloAck ack{};
    ack.magic            = kHelloAckMagic;
    ack.generationIdEcho = 0xABCD1234u;
    h.onAck(ack);
    CHECK(h.state() == HelloState::Keepalive);

    // First two ticks → still KEEPALIVE (return false)
    CHECK(h.onKeepaliveTick() == false);
    CHECK(h.onKeepaliveTick() == false);
    // Third tick → drops to ANNOUNCING (return true)
    CHECK(h.onKeepaliveTick() == true);
    CHECK(h.state() == HelloState::Announcing);
}

// ---------------------------------------------------------------------------
// hello_build_announce_sets_packet_fields_from_init
// ---------------------------------------------------------------------------

TEST_CASE("hello: buildAnnounce encodes correct mtu/fps/generationId") {
    // Mirror: hello_build_announce_sets_packet_fields_from_init
    HelloSm h(0xCAFEBABEu, 3994, 60, testCadence());
    uint8_t buf[kHelloOnWire];
    size_t n = h.buildAnnounce(buf, sizeof(buf));
    CHECK(n == kHelloOnWire);

    Hello decoded{};
    CHECK(decodeHello(buf, n, decoded) == DecodeResult::Ok);
    CHECK(decoded.mtuBytes    == 3994u);
    CHECK(decoded.fps         == 60u);
    CHECK(decoded.generationId == 0xCAFEBABEu);
}

// ---------------------------------------------------------------------------
// hello_announce_flags_zero_when_interleaving_supported (vanilla=false)
// ---------------------------------------------------------------------------

TEST_CASE("hello: flags byte is zero when vanilla=false (interleaving supported)") {
    // Mirror: hello_announce_flags_zero_when_interleaving_supported
    HelloSm h(0xABCD1234u, 3994, 60, testCadence());
    h.setVanilla(false);  // interleaving supported → vanilla bit clear
    uint8_t buf[kHelloOnWire];
    size_t n = h.buildAnnounce(buf, sizeof(buf));
    CHECK(n == (size_t)kHelloOnWire);
    // Byte 5 is the flags field (offset 5 in Hello layout)
    CHECK(buf[5] == 0x00u);
}

// ---------------------------------------------------------------------------
// hello_announce_flags_sets_vanilla_bit_when_unsupported (vanilla=true)
// ---------------------------------------------------------------------------

TEST_CASE("hello: vanilla flag bit set when vanilla=true (interleaving NOT supported)") {
    // Mirror: hello_announce_flags_sets_vanilla_bit_when_unsupported
    HelloSm h(0xABCD1234u, 3994, 60, testCadence());
    h.setVanilla(true);  // vanilla wfb-ng → set bit
    uint8_t buf[kHelloOnWire];
    size_t n = h.buildAnnounce(buf, sizeof(buf));
    CHECK(n == (size_t)kHelloOnWire);
    CHECK((buf[5] & kHelloFlagVanillaWfbNg) == kHelloFlagVanillaWfbNg);
}

// ---------------------------------------------------------------------------
// Disabled state: mtu=0 or fps=0 → DISABLED
// ---------------------------------------------------------------------------

TEST_CASE("hello: mtu=0 starts in DISABLED state") {
    HelloSm h(0xABCD1234u, /*mtu=*/0, /*fps=*/60, testCadence());
    CHECK(h.state() == HelloState::Disabled);
    CHECK(h.nextDelayMs() == 0u);
    uint8_t buf[kHelloOnWire];
    CHECK(h.buildAnnounce(buf, sizeof(buf)) == 0u);
}

TEST_CASE("hello: fps=0 starts in DISABLED state") {
    HelloSm h(0xABCD1234u, /*mtu=*/1400, /*fps=*/0, testCadence());
    CHECK(h.state() == HelloState::Disabled);
    CHECK(h.nextDelayMs() == 0u);
    uint8_t buf[kHelloOnWire];
    CHECK(h.buildAnnounce(buf, sizeof(buf)) == 0u);
}

// ---------------------------------------------------------------------------
// keepalive tick in non-KEEPALIVE state is a no-op
// ---------------------------------------------------------------------------

TEST_CASE("hello: keepalive tick in ANNOUNCING state is no-op") {
    HelloSm h(0xABCD1234u, 1400, 60, testCadence());
    CHECK(h.state() == HelloState::Announcing);
    CHECK(h.onKeepaliveTick() == false);
    CHECK(h.state() == HelloState::Announcing);
}

// ---------------------------------------------------------------------------
// onAck resets keepalivesWithoutAck when already in KEEPALIVE
// ---------------------------------------------------------------------------

TEST_CASE("hello: ACK in KEEPALIVE resets the miss counter") {
    HelloSm h(0xABCD1234u, 1400, 60, testCadence());
    HelloAck ack{};
    ack.magic            = kHelloAckMagic;
    ack.generationIdEcho = 0xABCD1234u;

    h.onAck(ack);
    CHECK(h.state() == HelloState::Keepalive);

    // Tick twice (miss count = 2)
    h.onKeepaliveTick();
    h.onKeepaliveTick();

    // ACK resets miss count; next 3 ticks should be needed to drop
    h.onAck(ack);
    CHECK(h.state() == HelloState::Keepalive);

    CHECK(h.onKeepaliveTick() == false);
    CHECK(h.onKeepaliveTick() == false);
    CHECK(h.onKeepaliveTick() == true);
    CHECK(h.state() == HelloState::Announcing);
}

// ---------------------------------------------------------------------------
// setMtuFps hot-reconcile
// ---------------------------------------------------------------------------

TEST_CASE("hello: setMtuFps updates fields and re-encodes correctly") {
    HelloSm h(0xABCD1234u, 1400, 30, testCadence());

    h.setMtuFps(3994, 60);

    uint8_t buf[kHelloOnWire];
    h.buildAnnounce(buf, sizeof(buf));
    Hello decoded{};
    CHECK(decodeHello(buf, sizeof(buf), decoded) == DecodeResult::Ok);
    CHECK(decoded.mtuBytes == 3994u);
    CHECK(decoded.fps      == 60u);
}

TEST_CASE("hello: setVanilla(true) while in KEEPALIVE resets to ANNOUNCING") {
    HelloSm h(0xABCD1234u, 1400, 60, testCadence());
    HelloAck ack{};
    ack.magic            = kHelloAckMagic;
    ack.generationIdEcho = 0xABCD1234u;
    h.onAck(ack);
    CHECK(h.state() == HelloState::Keepalive);

    // Changing vanilla flag forces re-announce so GS relearns capability
    h.setVanilla(true);
    CHECK(h.state() == HelloState::Announcing);
}
