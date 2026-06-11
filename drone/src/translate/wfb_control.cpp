#include "translate/wfb_control.hpp"
#include "translate/wfb_cmd.h"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
#include <cerrno>
#include <cstddef>
#include <cstring>

namespace fpvd {

WfbControlClient::WfbControlClient(const std::string& addr, uint16_t port) {
    fd_ = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (fd_ < 0) {
        openError_ = std::string("socket: ") + std::strerror(errno);
        return;
    }
    sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port = htons(port);
    if (::inet_pton(AF_INET, addr.c_str(), &dst.sin_addr) != 1) {
        openError_ = "bad address: " + addr;
        ::close(fd_);
        fd_ = -1;
        return;
    }
    if (::connect(fd_, reinterpret_cast<sockaddr*>(&dst), sizeof(dst)) < 0) {
        openError_ = std::string("connect: ") + std::strerror(errno);
        ::close(fd_);
        fd_ = -1;
        return;
    }
    timeval tv{};
    tv.tv_sec = 0;
    tv.tv_usec = 500000;   // 500 ms
    ::setsockopt(fd_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
}

WfbControlClient::~WfbControlClient() {
    if (fd_ >= 0) ::close(fd_);
}

WfbCtlResult WfbControlClient::sendAndRecv(const void* req, size_t reqLen,
                                           uint32_t reqId, const char* label) {
    if (fd_ < 0) return {false, std::string(label) + ": " + openError_};

    // Drain stale replies left over from a prior timed-out request.
    WfbCmdResp scratch;
    while (::recv(fd_, &scratch, sizeof(scratch), MSG_DONTWAIT) > 0) {}

    ssize_t nsent = ::send(fd_, req, reqLen, 0);
    if (nsent < 0 || static_cast<size_t>(nsent) != reqLen)
        return {false, std::string(label) + ": send failed"};

    for (;;) {
        WfbCmdResp resp;
        ssize_t nrecv = ::recv(fd_, &resp, sizeof(resp), 0);
        if (nrecv < 0)
            return {false, std::string(label) + ": timeout"};
        if (static_cast<size_t>(nrecv) < offsetof(WfbCmdResp, u))
            return {false, std::string(label) + ": short reply"};
        if (ntohl(resp.req_id) != reqId) continue;   // stale; keep waiting
        uint32_t rc = ntohl(resp.rc);
        if (rc != 0)
            return {false, std::string(label) + ": rc=" + std::to_string(rc)};
        return {true, {}};
    }
}

WfbCtlResult WfbControlClient::setRadio(uint8_t stbc, bool ldpc, bool shortGi,
                                        uint8_t bandwidth, uint8_t mcs,
                                        bool vhtMode, uint8_t vhtNss) {
    uint32_t id = reqId_++;
    WfbCmdReq req{};
    req.req_id = htonl(id);
    req.cmd_id = kWfbCmdSetRadio;
    req.u.set_radio.stbc = stbc;
    req.u.set_radio.ldpc = ldpc;
    req.u.set_radio.short_gi = shortGi;
    req.u.set_radio.bandwidth = bandwidth;
    req.u.set_radio.mcs_index = mcs;
    req.u.set_radio.vht_mode = vhtMode;
    req.u.set_radio.vht_nss = vhtNss;
    return sendAndRecv(&req, offsetof(WfbCmdReq, u) + sizeof(req.u.set_radio),
                       id, "set_radio");
}

WfbCtlResult WfbControlClient::setFec(uint8_t k, uint8_t n) {
    uint32_t id = reqId_++;
    WfbCmdReq req{};
    req.req_id = htonl(id);
    req.cmd_id = kWfbCmdSetFec;
    req.u.set_fec.k = k;
    req.u.set_fec.n = n;
    return sendAndRecv(&req, offsetof(WfbCmdReq, u) + sizeof(req.u.set_fec),
                       id, "set_fec");
}

} // namespace fpvd
