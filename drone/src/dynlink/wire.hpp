#pragma once
#include <cstddef>
#include <cstdint>

namespace fpvd::dynlink {

inline constexpr uint32_t kWireMagic        = 0x444C4B31u;  // "DLK1"
inline constexpr uint8_t  kWireVersion      = 3;
inline constexpr size_t   kWirePayloadSize  = 11;           // magic+ver+flags+seq+mcs
inline constexpr size_t   kWireOnWire       = 15;           // 11 payload + 4 CRC

inline constexpr uint32_t kPingMagic        = 0x444C5047u;  // "DLPG"
inline constexpr size_t   kPingPayloadSize  = 20;
inline constexpr size_t   kPingOnWire       = 24;           // payload + 4 CRC

inline constexpr uint32_t kPongMagic        = 0x444C504Eu;  // "DLPN"
inline constexpr size_t   kPongPayloadSize  = 36;
inline constexpr size_t   kPongOnWire       = 40;           // payload + 4 CRC

inline constexpr uint32_t kHelloMagic       = 0x444C4845u;  // "DLHE"
inline constexpr size_t   kHelloPayloadSize = 28;
inline constexpr size_t   kHelloOnWire      = 32;           // payload + 4 CRC

inline constexpr uint32_t kHelloAckMagic    = 0x444C4841u;  // "DLHA"
inline constexpr size_t   kHelloAckPayloadSize = 28;
inline constexpr size_t   kHelloAckOnWire   = 32;           // payload + 4 CRC

inline constexpr uint8_t  kHelloFlagVanillaWfbNg = 0x01u;

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
    uint8_t  depth{};
    uint16_t bitrateKbps{};
    uint8_t  fps{};
};

struct Ping {
    uint32_t magic{};
    uint8_t  version{};
    uint8_t  flags{};
    uint32_t gsSeq{};
    uint64_t gsMonoUs{};
};

struct Pong {
    uint32_t magic{};
    uint8_t  version{};
    uint8_t  flags{};
    uint32_t gsSeq{};
    uint64_t gsMonoUsEcho{};
    uint64_t droneMonoRecvUs{};
    uint64_t droneMonoSendUs{};
};

struct Hello {
    uint32_t magic{};
    uint8_t  version{};
    uint8_t  flags{};
    uint32_t generationId{};
    uint16_t mtuBytes{};
    uint16_t fps{};
    uint32_t applierBuildSha{};
};

struct HelloAck {
    uint32_t magic{};
    uint8_t  version{};
    uint32_t generationIdEcho{};
};

enum class DecodeResult { Ok, Short, BadMagic, BadVersion, BadCrc };
enum class PacketKind { Unknown, Decision, Ping, Pong, Hello, HelloAck };

uint32_t   crc32(const uint8_t* buf, size_t len);
PacketKind peekKind(const uint8_t* buf, size_t len);

size_t       encodeDecision(const Decision& d, uint8_t* buf, size_t buflen);
DecodeResult decodeDecision(const uint8_t* buf, size_t len, Decision& d);

size_t       encodePing(const Ping& p, uint8_t* buf, size_t buflen);
DecodeResult decodePing(const uint8_t* buf, size_t len, Ping& p);

size_t       encodePong(const Pong& p, uint8_t* buf, size_t buflen);
DecodeResult decodePong(const uint8_t* buf, size_t len, Pong& p);

size_t       encodeHello(const Hello& h, uint8_t* buf, size_t buflen);
DecodeResult decodeHello(const uint8_t* buf, size_t len, Hello& h);

size_t       encodeHelloAck(const HelloAck& h, uint8_t* buf, size_t buflen);
DecodeResult decodeHelloAck(const uint8_t* buf, size_t len, HelloAck& h);

} // namespace fpvd::dynlink
