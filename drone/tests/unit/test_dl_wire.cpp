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
    d.flags    = 0x01;
    d.sequence = 0xAABBCCDD;
    d.mcs      = 5;
    uint8_t buf[64];
    size_t n = encodeDecision(d, buf, sizeof(buf));
    CHECK(n == 15);
    CHECK(buf[4] == 3);            // version
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
    buf[0] = 0x44; buf[1] = 0x4C; buf[2] = 0x4B; buf[3] = 0x31;
    buf[4] = 2;  // version 2
    Decision out{};
    DecodeResult r = decodeDecision(buf, sizeof(buf), out);
    CHECK(r == DecodeResult::BadVersion);
}

TEST_CASE("wire: v3 protocol constants") {
    CHECK(kWireVersion     == 3);
    CHECK(kWirePayloadSize == 11u);
    CHECK(kWireOnWire      == 15u);
}

TEST_CASE("wire: v3 decision big-endian byte order") {
    Decision d{};
    d.sequence = 0x01020304u;
    d.mcs      = 0xAB;

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
    CHECK(decodeDecision(reinterpret_cast<const uint8_t*>("short"), 5, r)
          == DecodeResult::Short);
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
    buf[4] = 99;  // version byte
    Decision r{};
    CHECK(decodeDecision(buf, sizeof(buf), r) == DecodeResult::BadVersion);
}

TEST_CASE("wire: decision rejects bad crc") {
    // Mirrors test_wire_rejects_bad_crc
    Decision d{};
    d.mcs = 5;
    uint8_t buf[kWireOnWire];
    encodeDecision(d, buf, sizeof(buf));
    buf[10] ^= 0xFF;  // corrupt mcs payload byte; CRC no longer matches
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
// Ping
// ---------------------------------------------------------------------------

TEST_CASE("wire: ping round trip") {
    // Mirrors test_wire_ping_round_trip
    Ping p{};
    p.flags     = 0;
    p.gsSeq     = 0xCAFEBABEu;
    p.gsMonoUs  = 0x0102030405060708ull;

    uint8_t buf[kPingOnWire];
    CHECK(encodePing(p, buf, sizeof(buf)) == kPingOnWire);

    // Magic at [0..3] = 'DLPG' = 0x44 0x4C 0x50 0x47
    CHECK(buf[0] == 0x44);
    CHECK(buf[1] == 0x4C);
    CHECK(buf[2] == 0x50);
    CHECK(buf[3] == 0x47);

    // gs_mono_us at [12..19] big-endian
    CHECK(buf[12] == 0x01);
    CHECK(buf[19] == 0x08);

    Ping r{};
    CHECK(decodePing(buf, sizeof(buf), r) == DecodeResult::Ok);
    CHECK(r.gsSeq    == p.gsSeq);
    CHECK(r.gsMonoUs == p.gsMonoUs);
}

TEST_CASE("wire: ping rejects bad crc") {
    // Mirrors test_wire_ping_rejects_bad_crc
    Ping p{};
    p.gsSeq    = 1;
    p.gsMonoUs = 1;
    uint8_t buf[kPingOnWire];
    encodePing(p, buf, sizeof(buf));
    buf[10] ^= 0xFF;
    Ping r{};
    CHECK(decodePing(buf, sizeof(buf), r) == DecodeResult::BadCrc);
}

TEST_CASE("wire: ping rejects short buffer") {
    Ping p{};
    p.gsSeq    = 0xCAFEBABEu;
    p.gsMonoUs = 0x0102030405060708ull;
    uint8_t buf[kPingOnWire];
    encodePing(p, buf, sizeof(buf));
    Ping r{};
    CHECK(decodePing(buf, kPingOnWire - 1, r) == DecodeResult::Short);
}

TEST_CASE("wire: ping rejects bad magic") {
    Ping p{};
    p.gsSeq    = 0xCAFEBABEu;
    p.gsMonoUs = 0x0102030405060708ull;
    uint8_t buf[kPingOnWire];
    encodePing(p, buf, sizeof(buf));
    buf[0] ^= 0xFF;
    Ping r{};
    CHECK(decodePing(buf, sizeof(buf), r) == DecodeResult::BadMagic);
}

TEST_CASE("wire: ping rejects bad version") {
    Ping p{};
    p.gsSeq    = 0xCAFEBABEu;
    p.gsMonoUs = 0x0102030405060708ull;
    uint8_t buf[kPingOnWire];
    encodePing(p, buf, sizeof(buf));
    buf[4] = kWireVersion + 1;
    // Recompute CRC so we test bad-version, not bad-crc
    uint32_t new_crc = crc32(buf, kPingPayloadSize);
    buf[kPingPayloadSize]     = static_cast<uint8_t>((new_crc >> 24) & 0xFF);
    buf[kPingPayloadSize + 1] = static_cast<uint8_t>((new_crc >> 16) & 0xFF);
    buf[kPingPayloadSize + 2] = static_cast<uint8_t>((new_crc >>  8) & 0xFF);
    buf[kPingPayloadSize + 3] = static_cast<uint8_t>(new_crc & 0xFF);
    Ping r{};
    CHECK(decodePing(buf, sizeof(buf), r) == DecodeResult::BadVersion);
}

// ---------------------------------------------------------------------------
// Pong
// ---------------------------------------------------------------------------

TEST_CASE("wire: pong round trip") {
    // Mirrors test_wire_pong_round_trip
    Pong p{};
    p.flags            = 0;
    p.gsSeq            = 7;
    p.gsMonoUsEcho     = 1000000ull;
    p.droneMonoRecvUs  = 2000000ull;
    p.droneMonoSendUs  = 2000050ull;

    uint8_t buf[kPongOnWire];
    CHECK(encodePong(p, buf, sizeof(buf)) == kPongOnWire);

    // Magic 'DLPN'
    CHECK(buf[0] == 0x44);
    CHECK(buf[1] == 0x4C);
    CHECK(buf[2] == 0x50);
    CHECK(buf[3] == 0x4E);

    Pong r{};
    CHECK(decodePong(buf, sizeof(buf), r) == DecodeResult::Ok);
    CHECK(r.gsSeq           == p.gsSeq);
    CHECK(r.gsMonoUsEcho    == p.gsMonoUsEcho);
    CHECK(r.droneMonoRecvUs == p.droneMonoRecvUs);
    CHECK(r.droneMonoSendUs == p.droneMonoSendUs);
}

TEST_CASE("wire: pong rejects short buffer") {
    Pong p{};
    p.gsSeq            = 7;
    p.gsMonoUsEcho     = 1000000ull;
    p.droneMonoRecvUs  = 2000000ull;
    p.droneMonoSendUs  = 2000050ull;
    uint8_t buf[kPongOnWire];
    encodePong(p, buf, sizeof(buf));
    Pong r{};
    CHECK(decodePong(buf, kPongOnWire - 1, r) == DecodeResult::Short);
}

TEST_CASE("wire: pong rejects bad magic") {
    Pong p{};
    p.gsSeq            = 7;
    p.gsMonoUsEcho     = 1000000ull;
    p.droneMonoRecvUs  = 2000000ull;
    p.droneMonoSendUs  = 2000050ull;
    uint8_t buf[kPongOnWire];
    encodePong(p, buf, sizeof(buf));
    buf[0] ^= 0xFF;
    Pong r{};
    CHECK(decodePong(buf, sizeof(buf), r) == DecodeResult::BadMagic);
}

TEST_CASE("wire: pong rejects bad version") {
    Pong p{};
    p.gsSeq            = 7;
    p.gsMonoUsEcho     = 1000000ull;
    p.droneMonoRecvUs  = 2000000ull;
    p.droneMonoSendUs  = 2000050ull;
    uint8_t buf[kPongOnWire];
    encodePong(p, buf, sizeof(buf));
    buf[4] = kWireVersion + 1;
    // Recompute CRC so we test bad-version, not bad-crc
    uint32_t new_crc = crc32(buf, kPongPayloadSize);
    buf[kPongPayloadSize]     = static_cast<uint8_t>((new_crc >> 24) & 0xFF);
    buf[kPongPayloadSize + 1] = static_cast<uint8_t>((new_crc >> 16) & 0xFF);
    buf[kPongPayloadSize + 2] = static_cast<uint8_t>((new_crc >>  8) & 0xFF);
    buf[kPongPayloadSize + 3] = static_cast<uint8_t>(new_crc & 0xFF);
    Pong r{};
    CHECK(decodePong(buf, sizeof(buf), r) == DecodeResult::BadVersion);
}

TEST_CASE("wire: pong rejects bad crc") {
    Pong p{};
    p.gsSeq            = 7;
    p.gsMonoUsEcho     = 1000000ull;
    p.droneMonoRecvUs  = 2000000ull;
    p.droneMonoSendUs  = 2000050ull;
    uint8_t buf[kPongOnWire];
    encodePong(p, buf, sizeof(buf));
    buf[kPongOnWire - 1] ^= 0x01;  // corrupt last CRC byte
    Pong r{};
    CHECK(decodePong(buf, sizeof(buf), r) == DecodeResult::BadCrc);
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
        Ping p{};
        uint8_t pbuf[kPingOnWire];
        encodePing(p, pbuf, sizeof(pbuf));
        CHECK(peekKind(pbuf, sizeof(pbuf)) == PacketKind::Ping);
    }
    {
        Pong pong{};
        uint8_t obuf[kPongOnWire];
        encodePong(pong, obuf, sizeof(obuf));
        CHECK(peekKind(obuf, sizeof(obuf)) == PacketKind::Pong);
    }
    {
        uint8_t junk[8] = { 0xDE, 0xAD, 0xBE, 0xEF, 0, 0, 0, 0 };
        CHECK(peekKind(junk, sizeof(junk)) == PacketKind::Unknown);
        CHECK(peekKind(junk, 2)            == PacketKind::Unknown);
    }
}

// ---------------------------------------------------------------------------
// Hello
// ---------------------------------------------------------------------------

TEST_CASE("wire: hello encode/decode round trip") {
    // Mirrors wire_hello_encode_decode_roundtrip
    Hello in{};
    in.version         = kWireVersion;
    in.flags           = 0;
    in.generationId    = 0xCAFEBABEu;
    in.mtuBytes        = 3994;
    in.fps             = 60;
    in.applierBuildSha = 0xDEADBEEFu;

    uint8_t buf[kHelloOnWire];
    size_t n = encodeHello(in, buf, sizeof(buf));
    CHECK(n == kHelloOnWire);

    Hello out{};
    CHECK(decodeHello(buf, n, out) == DecodeResult::Ok);
    CHECK(out.generationId    == in.generationId);
    CHECK(out.mtuBytes        == in.mtuBytes);
    CHECK(out.fps             == in.fps);
    CHECK(out.applierBuildSha == in.applierBuildSha);
}

TEST_CASE("wire: hello rejects bad crc") {
    // Mirrors wire_hello_bad_crc_rejected
    Hello in{};
    in.version      = kWireVersion;
    in.generationId = 1;
    in.mtuBytes     = 1400;
    in.fps          = 30;
    uint8_t buf[kHelloOnWire];
    encodeHello(in, buf, sizeof(buf));
    buf[kHelloOnWire - 1] ^= 0x01;  // corrupt CRC
    Hello out{};
    CHECK(decodeHello(buf, sizeof(buf), out) == DecodeResult::BadCrc);
}

TEST_CASE("wire: hello rejects bad magic") {
    // Mirrors wire_hello_bad_magic_rejected
    uint8_t buf[kHelloOnWire] = {};  // all-zero magic
    Hello out{};
    CHECK(decodeHello(buf, sizeof(buf), out) == DecodeResult::BadMagic);
}

TEST_CASE("wire: hello rejects short buffer") {
    // Mirrors wire_hello_short_buffer_rejected
    Hello in{};
    in.version      = kWireVersion;
    in.generationId = 1;
    in.mtuBytes     = 1400;
    in.fps          = 30;
    uint8_t buf[kHelloOnWire];
    encodeHello(in, buf, sizeof(buf));
    Hello out{};
    CHECK(decodeHello(buf, kHelloOnWire - 1, out) == DecodeResult::Short);
}

TEST_CASE("wire: hello rejects bad version") {
    // Mirrors wire_hello_bad_version_rejected
    Hello in{};
    in.version      = kWireVersion;
    in.generationId = 1;
    in.mtuBytes     = 1400;
    in.fps          = 30;
    uint8_t buf[kHelloOnWire];
    encodeHello(in, buf, sizeof(buf));
    buf[4] = static_cast<uint8_t>(kWireVersion + 1);
    // Recompute CRC so we test bad-version, not bad-crc
    uint32_t new_crc = crc32(buf, kHelloPayloadSize);
    buf[kHelloPayloadSize]     = static_cast<uint8_t>((new_crc >> 24) & 0xFF);
    buf[kHelloPayloadSize + 1] = static_cast<uint8_t>((new_crc >> 16) & 0xFF);
    buf[kHelloPayloadSize + 2] = static_cast<uint8_t>((new_crc >>  8) & 0xFF);
    buf[kHelloPayloadSize + 3] = static_cast<uint8_t>(new_crc & 0xFF);
    Hello out{};
    CHECK(decodeHello(buf, sizeof(buf), out) == DecodeResult::BadVersion);
}

// ---------------------------------------------------------------------------
// HelloAck
// ---------------------------------------------------------------------------

TEST_CASE("wire: hello_ack encode/decode round trip") {
    // Mirrors wire_hello_ack_encode_decode_roundtrip
    HelloAck in{};
    in.version          = kWireVersion;
    in.generationIdEcho = 0x12345678u;

    uint8_t buf[kHelloAckOnWire];
    size_t n = encodeHelloAck(in, buf, sizeof(buf));
    CHECK(n == kHelloAckOnWire);

    HelloAck out{};
    CHECK(decodeHelloAck(buf, n, out) == DecodeResult::Ok);
    CHECK(out.generationIdEcho == in.generationIdEcho);
}

TEST_CASE("wire: hello_ack rejects bad crc") {
    // Mirrors wire_hello_ack_bad_crc_rejected
    HelloAck in{};
    in.generationIdEcho = 7;
    uint8_t buf[kHelloAckOnWire];
    encodeHelloAck(in, buf, sizeof(buf));
    buf[kHelloAckOnWire - 1] ^= 0x01;
    HelloAck out{};
    CHECK(decodeHelloAck(buf, sizeof(buf), out) == DecodeResult::BadCrc);
}

TEST_CASE("wire: hello_ack rejects bad magic") {
    // Mirrors wire_hello_ack_bad_magic_rejected
    uint8_t buf[kHelloAckOnWire] = {};
    HelloAck out{};
    CHECK(decodeHelloAck(buf, sizeof(buf), out) == DecodeResult::BadMagic);
}

TEST_CASE("wire: hello_ack rejects short buffer") {
    // Mirrors wire_hello_ack_short_buffer_rejected
    HelloAck in{};
    in.generationIdEcho = 1;
    uint8_t buf[kHelloAckOnWire];
    encodeHelloAck(in, buf, sizeof(buf));
    HelloAck out{};
    CHECK(decodeHelloAck(buf, kHelloAckOnWire - 1, out) == DecodeResult::Short);
}

TEST_CASE("wire: hello_ack rejects bad version") {
    // Mirrors wire_hello_ack_bad_version_rejected
    HelloAck in{};
    in.generationIdEcho = 1;
    uint8_t buf[kHelloAckOnWire];
    encodeHelloAck(in, buf, sizeof(buf));
    buf[4] = static_cast<uint8_t>(kWireVersion + 1);
    uint32_t new_crc = crc32(buf, kHelloAckPayloadSize);
    buf[kHelloAckPayloadSize]     = static_cast<uint8_t>((new_crc >> 24) & 0xFF);
    buf[kHelloAckPayloadSize + 1] = static_cast<uint8_t>((new_crc >> 16) & 0xFF);
    buf[kHelloAckPayloadSize + 2] = static_cast<uint8_t>((new_crc >>  8) & 0xFF);
    buf[kHelloAckPayloadSize + 3] = static_cast<uint8_t>(new_crc & 0xFF);
    HelloAck out{};
    CHECK(decodeHelloAck(buf, sizeof(buf), out) == DecodeResult::BadVersion);
}

// ---------------------------------------------------------------------------
// peekKind for Hello / HelloAck
// ---------------------------------------------------------------------------

TEST_CASE("wire: peekKind identifies hello_ack") {
    // Mirrors wire_peek_kind_hello_ack
    HelloAck in{};
    in.version          = kWireVersion;
    in.generationIdEcho = 7;
    uint8_t buf[kHelloAckOnWire];
    encodeHelloAck(in, buf, sizeof(buf));
    CHECK(peekKind(buf, sizeof(buf)) == PacketKind::HelloAck);
}

TEST_CASE("wire: peekKind identifies hello") {
    // Mirrors wire_peek_kind_hello
    Hello in{};
    in.version      = kWireVersion;
    in.generationId = 1;
    in.mtuBytes     = 1400;
    in.fps          = 30;
    uint8_t buf[kHelloOnWire];
    encodeHello(in, buf, sizeof(buf));
    CHECK(peekKind(buf, sizeof(buf)) == PacketKind::Hello);
}

// ---------------------------------------------------------------------------
// hello flag macro value
// ---------------------------------------------------------------------------

TEST_CASE("wire: kHelloFlagVanillaWfbNg is bit 0") {
    // Mirrors hello_flag_vanilla_macro_value
    CHECK(kHelloFlagVanillaWfbNg == 0x01u);
}
