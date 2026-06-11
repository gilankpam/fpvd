#pragma once
#include <cstddef>
#include <cstdint>

namespace fpvd::dynlink {

inline constexpr uint32_t kWireMagic        = 0x444C4B31u;  // "DLK1"
inline constexpr uint8_t  kWireVersion      = 3;
inline constexpr size_t   kWirePayloadSize  = 11;           // magic+ver+flags+seq+mcs
inline constexpr size_t   kWireOnWire       = 15;           // 11 payload + 4 CRC

struct Decision {
    uint32_t magic{};
    uint8_t  version{};
    uint8_t  flags{};
    uint32_t sequence{};
    uint32_t timestampMs{};
    uint8_t  mcs{};
    uint8_t  bandwidth{};
    int8_t   txPowerDbm{};
    uint8_t  k{};
    uint8_t  n{};
    uint16_t bitrateKbps{};
    uint8_t  fps{};
};

enum class DecodeResult { Ok, Short, BadMagic, BadVersion, BadCrc };
enum class PacketKind { Unknown, Decision };

uint32_t   crc32(const uint8_t* buf, size_t len);
PacketKind peekKind(const uint8_t* buf, size_t len);

size_t       encodeDecision(const Decision& d, uint8_t* buf, size_t buflen);
DecodeResult decodeDecision(const uint8_t* buf, size_t len, Decision& d);

} // namespace fpvd::dynlink
