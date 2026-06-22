/* test_dl_wire.cpp — C++ port of test_wire.c.
 * All golden vectors are transcribed verbatim from
 * dynamic-link/tests/drone/test_wire.c — no values invented.
 */
#include "doctest.h"
#include "dynlink/wire.hpp"

using namespace fpvd::dynlink;

// ---------------------------------------------------------------------------
// Decision encode/decode (v3: {mcs}-only, 15 bytes)
// ---------------------------------------------------------------------------

TEST_CASE("v3 Decision encodes to 15 bytes and round-trips mcs/sequence/flags") {
    Decision d{};
    d.flags = 0x01;
    d.sequence = 0xAABBCCDD;
    d.mcs = 5;
    uint8_t buf[64];
    size_t n = encodeDecision(d, buf, sizeof(buf));
    CHECK(n == 15);
    CHECK(buf[4] == 3); // version
    Decision out{};
    CHECK(decodeDecision(buf, n, out) == DecodeResult::Ok);
    CHECK(out.version == 3);
    CHECK(out.sequence == 0xAABBCCDD);
    CHECK(out.mcs == 5);
    CHECK(out.flags == 0x01);
}

TEST_CASE("v3 decode rejects a v2-sized/old-version buffer") {
    // 31-byte buffer carrying valid DLK1 magic and version=2.
    // version 2 carries valid DLK1 magic → must be rejected at the version check,
    // before CRC is evaluated. The ordering is the wire contract.
    uint8_t buf[31] = {0};
    // write the DLK1 magic big-endian
    buf[0] = 0x44;
    buf[1] = 0x4C;
    buf[2] = 0x4B;
    buf[3] = 0x31;
    buf[4] = 2; // version 2
    Decision out{};
    DecodeResult r = decodeDecision(buf, sizeof(buf), out);
    CHECK(r == DecodeResult::BadVersion);
}

TEST_CASE("wire: v3 protocol constants") {
    CHECK(kWireVersion == 3);
    CHECK(kWirePayloadSize == 11u);
    CHECK(kWireOnWire == 15u);
}

TEST_CASE("wire: v3 decision big-endian byte order") {
    Decision d{};
    d.sequence = 0x01020304u;
    d.mcs = 0xAB;

    uint8_t buf[kWireOnWire];
    encodeDecision(d, buf, sizeof(buf));

    // Magic 0x444C4B31 at [0..3] big-endian
    CHECK(buf[0] == 0x44);
    CHECK(buf[1] == 0x4C);
    CHECK(buf[2] == 0x4B);
    CHECK(buf[3] == 0x31);
    // version = 3 at [4]
    CHECK(buf[4] == 3);
    // sequence at [6..9] big-endian
    CHECK(buf[6] == 0x01);
    CHECK(buf[7] == 0x02);
    CHECK(buf[8] == 0x03);
    CHECK(buf[9] == 0x04);
    // mcs at [10]
    CHECK(buf[10] == 0xAB);
}

TEST_CASE("wire: decision rejects short buffer") {
    // Mirrors test_wire_rejects_short
    Decision r{};
    CHECK(decodeDecision(reinterpret_cast<const uint8_t*>("short"), 5, r) == DecodeResult::Short);
}

TEST_CASE("wire: decision rejects bad magic") {
    // Mirrors test_wire_rejects_bad_magic
    Decision d{};
    uint8_t buf[kWireOnWire];
    encodeDecision(d, buf, sizeof(buf));
    buf[0] = 0xFF;
    Decision r{};
    CHECK(decodeDecision(buf, sizeof(buf), r) == DecodeResult::BadMagic);
}

TEST_CASE("wire: decision rejects bad version") {
    // Mirrors test_wire_rejects_bad_version
    Decision d{};
    uint8_t buf[kWireOnWire];
    encodeDecision(d, buf, sizeof(buf));
    buf[4] = 99; // version byte
    Decision r{};
    CHECK(decodeDecision(buf, sizeof(buf), r) == DecodeResult::BadVersion);
}

TEST_CASE("wire: decision rejects bad crc") {
    // Mirrors test_wire_rejects_bad_crc
    Decision d{};
    d.mcs = 5;
    uint8_t buf[kWireOnWire];
    encodeDecision(d, buf, sizeof(buf));
    buf[10] ^= 0xFF; // corrupt mcs payload byte; CRC no longer matches
    Decision r{};
    CHECK(decodeDecision(buf, sizeof(buf), r) == DecodeResult::BadCrc);
}

TEST_CASE("wire: crc32 of empty string is 0") {
    // Mirrors test_wire_crc32_empty
    CHECK(crc32(reinterpret_cast<const uint8_t*>(""), 0) == 0u);
}

TEST_CASE("wire: crc32 known IEEE 802.3 vector") {
    // Mirrors test_wire_crc32_known
    // "123456789" -> 0xCBF43926 (standard IEEE 802.3 test vector)
    uint32_t c = crc32(reinterpret_cast<const uint8_t*>("123456789"), 9);
    CHECK(c == 0xCBF43926u);
}

// ---------------------------------------------------------------------------
// peekKind dispatch
// ---------------------------------------------------------------------------

TEST_CASE("wire: peekKind dispatches correctly") {
    // Mirrors test_wire_peek_kind_dispatches
    {
        Decision d{};
        uint8_t dbuf[kWireOnWire];
        encodeDecision(d, dbuf, sizeof(dbuf));
        CHECK(peekKind(dbuf, sizeof(dbuf)) == PacketKind::Decision);
    }
    {
        uint8_t junk[8] = {0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0};
        CHECK(peekKind(junk, sizeof(junk)) == PacketKind::Unknown);
        CHECK(peekKind(junk, 2) == PacketKind::Unknown);
    }
}
