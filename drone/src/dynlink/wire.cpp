/* wire.cpp — C++ port of dl_wire.c.
 *
 * Byte layout is IDENTICAL to the C implementation so that GS-produced
 * frames can be decoded here without modification.
 *
 * Decision layout (big-endian, 31 bytes = 27 payload + 4 CRC):
 *   off  size  field
 *    0    4    magic       = kWireMagic (0x444C4B31)
 *    4    1    version     = kWireVersion (2)
 *    5    1    flags
 *    6    2    _pad (0)
 *    8    4    sequence
 *   12    4    timestampMs
 *   16    1    mcs
 *   17    1    bandwidth
 *   18    1    txPowerDbm  (signed)
 *   19    1    k
 *   20    1    n
 *   21    1    depth
 *   22    2    bitrateKbps
 *   24    1    fps
 *   25    2    _pad2 (0)
 *   27    4    crc32(bytes[0..26])
 *
 * Ping layout (big-endian, 24 bytes = 20 payload + 4 CRC):
 *    0  4  magic = kPingMagic
 *    4  1  version
 *    5  1  flags
 *    6  2  _pad
 *    8  4  gsSeq
 *   12  8  gsMonoUs
 *   20  4  crc32(bytes[0..19])
 *
 * Pong layout (big-endian, 40 bytes = 36 payload + 4 CRC):
 *    0  4  magic = kPongMagic
 *    4  1  version
 *    5  1  flags
 *    6  2  _pad
 *    8  4  gsSeq
 *   12  8  gsMonoUsEcho
 *   20  8  droneMonoRecvUs
 *   28  8  droneMonoSendUs
 *   36  4  crc32(bytes[0..35])
 *
 * Hello layout (big-endian, 32 bytes = 28 payload + 4 CRC):
 *    0  4  magic = kHelloMagic
 *    4  1  version
 *    5  1  flags
 *    6  2  _pad
 *    8  4  generationId
 *   12  2  mtuBytes
 *   14  2  fps
 *   16  4  applierBuildSha
 *   20  8  reserved (zero)
 *   28  4  crc32(bytes[0..27])
 *
 * HelloAck layout (big-endian, 32 bytes = 28 payload + 4 CRC):
 *    0  4  magic = kHelloAckMagic
 *    4  1  version
 *    5  3  _pad
 *    8  4  generationIdEcho
 *   12  16 reserved (zero)
 *   28  4  crc32(bytes[0..27])
 */
#include "dynlink/wire.hpp"

#include <cstring>

namespace fpvd::dynlink {

// ---------------------------------------------------------------------------
// Big-endian helpers — mirrors the static functions in dl_wire.c exactly.
// ---------------------------------------------------------------------------

static void put_u16(uint8_t* p, uint16_t v) {
    p[0] = static_cast<uint8_t>(v >> 8);
    p[1] = static_cast<uint8_t>(v & 0xFF);
}

static void put_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[2] = static_cast<uint8_t>((v >>  8) & 0xFF);
    p[3] = static_cast<uint8_t>(v & 0xFF);
}

static uint16_t get_u16(const uint8_t* p) {
    return static_cast<uint16_t>((p[0] << 8) | p[1]);
}

static uint32_t get_u32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24)
         | (static_cast<uint32_t>(p[1]) << 16)
         | (static_cast<uint32_t>(p[2]) <<  8)
         |  static_cast<uint32_t>(p[3]);
}

static void put_u64(uint8_t* p, uint64_t v) {
    p[0] = static_cast<uint8_t>(v >> 56);
    p[1] = static_cast<uint8_t>(v >> 48);
    p[2] = static_cast<uint8_t>(v >> 40);
    p[3] = static_cast<uint8_t>(v >> 32);
    p[4] = static_cast<uint8_t>(v >> 24);
    p[5] = static_cast<uint8_t>(v >> 16);
    p[6] = static_cast<uint8_t>(v >>  8);
    p[7] = static_cast<uint8_t>(v & 0xFF);
}

static uint64_t get_u64(const uint8_t* p) {
    return (static_cast<uint64_t>(p[0]) << 56)
         | (static_cast<uint64_t>(p[1]) << 48)
         | (static_cast<uint64_t>(p[2]) << 40)
         | (static_cast<uint64_t>(p[3]) << 32)
         | (static_cast<uint64_t>(p[4]) << 24)
         | (static_cast<uint64_t>(p[5]) << 16)
         | (static_cast<uint64_t>(p[6]) <<  8)
         |  static_cast<uint64_t>(p[7]);
}

// ---------------------------------------------------------------------------
// CRC-32 (IEEE 802.3, poly 0xEDB88320, reflected, init/final 0xFFFFFFFF).
// Table-free — matches dl_wire_crc32 bit-for-bit.
// ---------------------------------------------------------------------------

uint32_t crc32(const uint8_t* buf, size_t len) {
    uint32_t crc = 0xFFFFFFFFu;
    for (size_t i = 0; i < len; ++i) {
        crc ^= buf[i];
        for (int b = 0; b < 8; ++b) {
            uint32_t mask = -static_cast<int32_t>(crc & 1u);
            crc = (crc >> 1) ^ (0xEDB88320u & mask);
        }
    }
    return crc ^ 0xFFFFFFFFu;
}

// ---------------------------------------------------------------------------
// peekKind
// ---------------------------------------------------------------------------

PacketKind peekKind(const uint8_t* buf, size_t len) {
    if (len < 4) return PacketKind::Unknown;
    uint32_t m = get_u32(buf);
    if (m == kWireMagic)     return PacketKind::Decision;
    if (m == kPingMagic)     return PacketKind::Ping;
    if (m == kPongMagic)     return PacketKind::Pong;
    if (m == kHelloMagic)    return PacketKind::Hello;
    if (m == kHelloAckMagic) return PacketKind::HelloAck;
    return PacketKind::Unknown;
}

// ---------------------------------------------------------------------------
// Decision
// ---------------------------------------------------------------------------

size_t encodeDecision(const Decision& d, uint8_t* buf, size_t buflen) {
    if (buflen < kWireOnWire) return 0;

    std::memset(buf, 0, kWireOnWire);
    put_u32(&buf[0],  kWireMagic);
    buf[4] = kWireVersion;
    buf[5] = d.flags;
    /* buf[6..7] = _pad */
    put_u32(&buf[8],  d.sequence);
    put_u32(&buf[12], d.timestampMs);
    buf[16] = d.mcs;
    buf[17] = d.bandwidth;
    buf[18] = static_cast<uint8_t>(d.txPowerDbm);  // signed→unsigned via cast
    buf[19] = d.k;
    buf[20] = d.n;
    buf[21] = d.depth;
    put_u16(&buf[22], d.bitrateKbps);
    buf[24] = d.fps;
    /* buf[25..26] = _pad2 */
    uint32_t c = crc32(buf, kWirePayloadSize);
    put_u32(&buf[kWirePayloadSize], c);
    return kWireOnWire;
}

DecodeResult decodeDecision(const uint8_t* buf, size_t len, Decision& d) {
    if (len < kWireOnWire) return DecodeResult::Short;

    uint32_t magic = get_u32(&buf[0]);
    if (magic != kWireMagic) return DecodeResult::BadMagic;

    uint8_t version = buf[4];
    if (version != kWireVersion) return DecodeResult::BadVersion;

    uint32_t crc_wire = get_u32(&buf[kWirePayloadSize]);
    uint32_t crc_calc = crc32(buf, kWirePayloadSize);
    if (crc_wire != crc_calc) return DecodeResult::BadCrc;

    d = {};
    d.magic       = magic;
    d.version     = version;
    d.flags       = buf[5];
    d.sequence    = get_u32(&buf[8]);
    d.timestampMs = get_u32(&buf[12]);
    d.mcs         = buf[16];
    d.bandwidth   = buf[17];
    d.txPowerDbm  = static_cast<int8_t>(buf[18]);
    d.k           = buf[19];
    d.n           = buf[20];
    d.depth       = buf[21];
    d.bitrateKbps = get_u16(&buf[22]);
    d.fps         = buf[24];
    return DecodeResult::Ok;
}

// ---------------------------------------------------------------------------
// Ping
// ---------------------------------------------------------------------------

size_t encodePing(const Ping& p, uint8_t* buf, size_t buflen) {
    if (buflen < kPingOnWire) return 0;
    std::memset(buf, 0, kPingOnWire);
    put_u32(&buf[0],  kPingMagic);
    buf[4] = kWireVersion;
    buf[5] = p.flags;
    /* buf[6..7] = _pad */
    put_u32(&buf[8],  p.gsSeq);
    put_u64(&buf[12], p.gsMonoUs);
    uint32_t c = crc32(buf, kPingPayloadSize);
    put_u32(&buf[kPingPayloadSize], c);
    return kPingOnWire;
}

DecodeResult decodePing(const uint8_t* buf, size_t len, Ping& p) {
    if (len < kPingOnWire) return DecodeResult::Short;
    if (get_u32(&buf[0]) != kPingMagic) return DecodeResult::BadMagic;
    if (buf[4] != kWireVersion) return DecodeResult::BadVersion;
    uint32_t crc_wire = get_u32(&buf[kPingPayloadSize]);
    uint32_t crc_calc = crc32(buf, kPingPayloadSize);
    if (crc_wire != crc_calc) return DecodeResult::BadCrc;

    p = {};
    p.magic    = kPingMagic;
    p.version  = buf[4];
    p.flags    = buf[5];
    p.gsSeq    = get_u32(&buf[8]);
    p.gsMonoUs = get_u64(&buf[12]);
    return DecodeResult::Ok;
}

// ---------------------------------------------------------------------------
// Pong
// ---------------------------------------------------------------------------

size_t encodePong(const Pong& p, uint8_t* buf, size_t buflen) {
    if (buflen < kPongOnWire) return 0;
    std::memset(buf, 0, kPongOnWire);
    put_u32(&buf[0],  kPongMagic);
    buf[4] = kWireVersion;
    buf[5] = p.flags;
    /* buf[6..7] = _pad */
    put_u32(&buf[8],  p.gsSeq);
    put_u64(&buf[12], p.gsMonoUsEcho);
    put_u64(&buf[20], p.droneMonoRecvUs);
    put_u64(&buf[28], p.droneMonoSendUs);
    uint32_t c = crc32(buf, kPongPayloadSize);
    put_u32(&buf[kPongPayloadSize], c);
    return kPongOnWire;
}

DecodeResult decodePong(const uint8_t* buf, size_t len, Pong& p) {
    if (len < kPongOnWire) return DecodeResult::Short;
    if (get_u32(&buf[0]) != kPongMagic) return DecodeResult::BadMagic;
    if (buf[4] != kWireVersion) return DecodeResult::BadVersion;
    uint32_t crc_wire = get_u32(&buf[kPongPayloadSize]);
    uint32_t crc_calc = crc32(buf, kPongPayloadSize);
    if (crc_wire != crc_calc) return DecodeResult::BadCrc;

    p = {};
    p.magic            = kPongMagic;
    p.version          = buf[4];
    p.flags            = buf[5];
    p.gsSeq            = get_u32(&buf[8]);
    p.gsMonoUsEcho     = get_u64(&buf[12]);
    p.droneMonoRecvUs  = get_u64(&buf[20]);
    p.droneMonoSendUs  = get_u64(&buf[28]);
    return DecodeResult::Ok;
}

// ---------------------------------------------------------------------------
// Hello
// ---------------------------------------------------------------------------

size_t encodeHello(const Hello& h, uint8_t* buf, size_t buflen) {
    if (buflen < kHelloOnWire) return 0;
    std::memset(buf, 0, kHelloOnWire);
    put_u32(&buf[0],  kHelloMagic);
    /* Always emit current version; caller-supplied .version is ignored. */
    buf[4] = kWireVersion;
    buf[5] = h.flags;
    /* buf[6..7] = _pad */
    put_u32(&buf[8],  h.generationId);
    put_u16(&buf[12], h.mtuBytes);
    put_u16(&buf[14], h.fps);
    put_u32(&buf[16], h.applierBuildSha);
    /* buf[20..27] = reserved (zero) */
    uint32_t c = crc32(buf, kHelloPayloadSize);
    put_u32(&buf[kHelloPayloadSize], c);
    return kHelloOnWire;
}

DecodeResult decodeHello(const uint8_t* buf, size_t len, Hello& h) {
    if (len < kHelloOnWire) return DecodeResult::Short;
    uint32_t magic = get_u32(&buf[0]);
    if (magic != kHelloMagic) return DecodeResult::BadMagic;
    if (buf[4] != kWireVersion) return DecodeResult::BadVersion;
    uint32_t crc_wire = get_u32(&buf[kHelloPayloadSize]);
    uint32_t crc_calc = crc32(buf, kHelloPayloadSize);
    if (crc_wire != crc_calc) return DecodeResult::BadCrc;

    h = {};
    h.magic          = magic;
    h.version        = buf[4];
    h.flags          = buf[5];
    h.generationId   = get_u32(&buf[8]);
    h.mtuBytes       = get_u16(&buf[12]);
    h.fps            = get_u16(&buf[14]);
    h.applierBuildSha = get_u32(&buf[16]);
    return DecodeResult::Ok;
}

// ---------------------------------------------------------------------------
// HelloAck
// ---------------------------------------------------------------------------

size_t encodeHelloAck(const HelloAck& h, uint8_t* buf, size_t buflen) {
    if (buflen < kHelloAckOnWire) return 0;
    std::memset(buf, 0, kHelloAckOnWire);
    put_u32(&buf[0], kHelloAckMagic);
    /* Always emit current version; caller-supplied .version is ignored. */
    buf[4] = kWireVersion;
    /* buf[5..7] = _pad */
    put_u32(&buf[8], h.generationIdEcho);
    /* buf[12..27] = reserved (zero) */
    uint32_t c = crc32(buf, kHelloAckPayloadSize);
    put_u32(&buf[kHelloAckPayloadSize], c);
    return kHelloAckOnWire;
}

DecodeResult decodeHelloAck(const uint8_t* buf, size_t len, HelloAck& h) {
    if (len < kHelloAckOnWire) return DecodeResult::Short;
    uint32_t magic = get_u32(&buf[0]);
    if (magic != kHelloAckMagic) return DecodeResult::BadMagic;
    if (buf[4] != kWireVersion) return DecodeResult::BadVersion;
    uint32_t crc_wire = get_u32(&buf[kHelloAckPayloadSize]);
    uint32_t crc_calc = crc32(buf, kHelloAckPayloadSize);
    if (crc_wire != crc_calc) return DecodeResult::BadCrc;

    h = {};
    h.magic            = magic;
    h.version          = buf[4];
    h.generationIdEcho = get_u32(&buf[8]);
    return DecodeResult::Ok;
}

} // namespace fpvd::dynlink
