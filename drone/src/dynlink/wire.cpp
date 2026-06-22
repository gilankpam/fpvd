/* wire.cpp — C++ port of dl_wire.c.
 *
 * Byte layout is IDENTICAL to the C implementation so that GS-produced
 * frames can be decoded here without modification.
 *
 * Decision layout (big-endian, 15 bytes = 11 payload + 4 CRC):
 *   off  size  field
 *    0    4    magic       = kWireMagic (0x444C4B31)
 *    4    1    version     = kWireVersion (3)
 *    5    1    flags
 *    6    4    sequence
 *   10    1    mcs
 *   11    4    crc32(bytes[0..10])
 */
#include "dynlink/wire.hpp"

#include <cstring>

namespace fpvd::dynlink {

// ---------------------------------------------------------------------------
// Big-endian helpers — mirrors the static functions in dl_wire.c exactly.
// ---------------------------------------------------------------------------

static void put_u32(uint8_t* p, uint32_t v) {
    p[0] = static_cast<uint8_t>(v >> 24);
    p[1] = static_cast<uint8_t>((v >> 16) & 0xFF);
    p[2] = static_cast<uint8_t>((v >> 8) & 0xFF);
    p[3] = static_cast<uint8_t>(v & 0xFF);
}

static uint32_t get_u32(const uint8_t* p) {
    return (static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
           (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]);
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
    if (len < 4)
        return PacketKind::Unknown;
    return get_u32(buf) == kWireMagic ? PacketKind::Decision : PacketKind::Unknown;
}

// ---------------------------------------------------------------------------
// Decision
// ---------------------------------------------------------------------------

size_t encodeDecision(const Decision& d, uint8_t* buf, size_t buflen) {
    if (buflen < kWireOnWire)
        return 0;
    std::memset(buf, 0, kWireOnWire);
    put_u32(&buf[0], kWireMagic); // [0..3]  magic
    buf[4] = kWireVersion;        // [4]     version = 3
    buf[5] = d.flags;             // [5]     flags
    put_u32(&buf[6], d.sequence); // [6..9]  sequence
    buf[10] = d.mcs;              // [10]    mcs
    uint32_t c = crc32(buf, kWirePayloadSize);
    put_u32(&buf[kWirePayloadSize], c); // [11..14] crc32
    return kWireOnWire;
}

DecodeResult decodeDecision(const uint8_t* buf, size_t len, Decision& d) {
    if (len < kWireOnWire)
        return DecodeResult::Short;
    uint32_t magic = get_u32(&buf[0]);
    if (magic != kWireMagic)
        return DecodeResult::BadMagic;
    uint8_t version = buf[4];
    if (version != kWireVersion)
        return DecodeResult::BadVersion;
    uint32_t crc_wire = get_u32(&buf[kWirePayloadSize]);
    uint32_t crc_calc = crc32(buf, kWirePayloadSize);
    if (crc_wire != crc_calc)
        return DecodeResult::BadCrc;
    d = {};
    d.magic = magic;
    d.version = version;
    d.flags = buf[5];
    d.sequence = get_u32(&buf[6]);
    d.mcs = buf[10];
    return DecodeResult::Ok; // bandwidth/k/n/bitrate/fps left default; filled by config +
                             // applyLocalCompute
}

} // namespace fpvd::dynlink
