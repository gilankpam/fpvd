#pragma once
#include <cstdint>
#include <string>

namespace fpvd {

struct WfbCtlResult {
    bool ok{false};
    std::string error;
};

// Minimal UDP client for a wfb_tx control socket (-C port, bound to
// 127.0.0.1). Connected UDP; req_id htonl + match-on-recv; 500 ms recv
// timeout; drains stale replies before each send.
class WfbControlClient {
public:
    WfbControlClient(const std::string& addr, uint16_t port);
    ~WfbControlClient();

    WfbControlClient(const WfbControlClient&) = delete;
    WfbControlClient& operator=(const WfbControlClient&) = delete;

    WfbCtlResult setRadio(uint8_t stbc, bool ldpc, bool shortGi,
                          uint8_t bandwidth, uint8_t mcs,
                          bool vhtMode, uint8_t vhtNss);
    WfbCtlResult setFec(uint8_t k, uint8_t n);

private:
    WfbCtlResult sendAndRecv(const void* req, size_t reqLen,
                             uint32_t reqId, const char* label);
    int fd_{-1};
    uint32_t reqId_{1};
    std::string openError_;
};

} // namespace fpvd
