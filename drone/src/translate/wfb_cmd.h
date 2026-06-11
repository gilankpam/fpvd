#pragma once
#include <cstddef>
#include <cstdint>

namespace fpvd {

constexpr uint8_t kWfbCmdSetFec              = 1;
constexpr uint8_t kWfbCmdSetRadio            = 2;

#pragma pack(push, 1)
struct WfbCmdReq {
    uint32_t req_id;   // network byte order
    uint8_t  cmd_id;
    union {
        struct { uint8_t k; uint8_t n; } set_fec;
        struct {
            uint8_t stbc;
            bool    ldpc;
            bool    short_gi;
            uint8_t bandwidth;
            uint8_t mcs_index;
            bool    vht_mode;
            uint8_t vht_nss;
        } set_radio;
    } u;
};

struct WfbCmdResp {
    uint32_t req_id;   // network byte order
    uint32_t rc;       // network byte order
    union {
        struct { uint8_t k; uint8_t n; } get_fec;
        struct {
            uint8_t stbc;
            bool    ldpc;
            bool    short_gi;
            uint8_t bandwidth;
            uint8_t mcs_index;
            bool    vht_mode;
            uint8_t vht_nss;
        } get_radio;
    } u;
};
#pragma pack(pop)

} // namespace fpvd
