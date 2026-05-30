#include "doctest.h"
#include "translate/wfb_control.hpp"
#include "translate/wfb_cmd.h"
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstddef>
#include <cstring>
#include <thread>

namespace {
// Bind a UDP server on 127.0.0.1:<ephemeral>; return fd and learned port.
int bindServer(uint16_t& port) {
    int fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(fd >= 0);
    sockaddr_in a{};
    a.sin_family = AF_INET;
    a.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    a.sin_port = 0;
    REQUIRE(::bind(fd, reinterpret_cast<sockaddr*>(&a), sizeof(a)) == 0);
    socklen_t len = sizeof(a);
    REQUIRE(::getsockname(fd, reinterpret_cast<sockaddr*>(&a), &len) == 0);
    port = ntohs(a.sin_port);
    return fd;
}
} // namespace

TEST_CASE("wfb_control: setRadio sends correct wire bytes, parses ok reply") {
    uint16_t port = 0;
    int srv = bindServer(port);

    fpvd::WfbCtlResult res;
    std::thread client([&] {
        fpvd::WfbControlClient c("127.0.0.1", port);
        res = c.setRadio(/*stbc=*/0, /*ldpc=*/false, /*shortGi=*/false,
                         /*bandwidth=*/40, /*mcs=*/5, /*vhtMode=*/false,
                         /*vhtNss=*/1);
    });

    fpvd::WfbCmdReq req{};
    sockaddr_in from{};
    socklen_t flen = sizeof(from);
    ssize_t n = ::recvfrom(srv, &req, sizeof(req), 0,
                           reinterpret_cast<sockaddr*>(&from), &flen);
    REQUIRE(n == static_cast<ssize_t>(offsetof(fpvd::WfbCmdReq, u) +
                                      sizeof(req.u.set_radio)));
    CHECK(req.cmd_id == fpvd::kWfbCmdSetRadio);
    CHECK(req.u.set_radio.bandwidth == 40);
    CHECK(req.u.set_radio.mcs_index == 5);
    CHECK(req.u.set_radio.vht_nss == 1);

    fpvd::WfbCmdResp resp{};
    resp.req_id = req.req_id;   // echo as-is (already network order)
    resp.rc = htonl(0);
    ::sendto(srv, &resp, offsetof(fpvd::WfbCmdResp, u), 0,
             reinterpret_cast<sockaddr*>(&from), flen);

    client.join();
    CHECK(res.ok);
    ::close(srv);
}

TEST_CASE("wfb_control: setFec sends k/n and non-zero rc surfaces as error") {
    uint16_t port = 0;
    int srv = bindServer(port);

    fpvd::WfbCtlResult res;
    std::thread client([&] {
        fpvd::WfbControlClient c("127.0.0.1", port);
        res = c.setFec(3, 5);
    });

    fpvd::WfbCmdReq req{};
    sockaddr_in from{};
    socklen_t flen = sizeof(from);
    ::recvfrom(srv, &req, sizeof(req), 0,
               reinterpret_cast<sockaddr*>(&from), &flen);
    CHECK(req.cmd_id == fpvd::kWfbCmdSetFec);
    CHECK(req.u.set_fec.k == 3);
    CHECK(req.u.set_fec.n == 5);

    fpvd::WfbCmdResp resp{};
    resp.req_id = req.req_id;
    resp.rc = htonl(22);   // EINVAL
    ::sendto(srv, &resp, offsetof(fpvd::WfbCmdResp, u), 0,
             reinterpret_cast<sockaddr*>(&from), flen);

    client.join();
    CHECK_FALSE(res.ok);
    CHECK(res.error.find("rc=22") != std::string::npos);
    ::close(srv);
}

TEST_CASE("wfb_control: silence yields timeout error") {
    uint16_t port = 0;
    int srv = bindServer(port);
    fpvd::WfbControlClient c("127.0.0.1", port);
    auto res = c.setFec(3, 5);   // server never replies (~500 ms)
    CHECK_FALSE(res.ok);
    CHECK(res.error.find("timeout") != std::string::npos);
    ::close(srv);
}
