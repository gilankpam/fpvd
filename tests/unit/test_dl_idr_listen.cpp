/* test_dl_idr_listen.cpp — unit tests for IdrListener (ported from test_idr_listen.c). */
#include "doctest.h"
#include "dynlink/idr_listen.hpp"

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

using namespace fpvd::dynlink;

TEST_CASE("idr listener: port 0 disables") {
    IdrListener l("127.0.0.1", 0);
    CHECK(l.fd() == -1);
    CHECK(l.drain() == 0);
}

TEST_CASE("idr listener: bind, drain, three datagrams") {
    /* Fixed high port; single-threaded test harness so collisions are
     * vanishingly rare. */
    const uint16_t PORT = 51123;
    IdrListener l("127.0.0.1", PORT);
    REQUIRE(l.fd() >= 0);

    /* No data yet — drain returns 0. */
    CHECK(l.drain() == 0);

    /* Send 3 datagrams from a client socket. */
    int s = socket(AF_INET, SOCK_DGRAM, 0);
    REQUIRE(s >= 0);

    struct sockaddr_in dst{};
    dst.sin_family = AF_INET;
    dst.sin_port   = htons(PORT);
    inet_pton(AF_INET, "127.0.0.1", &dst.sin_addr);

    const char msg[] = "abc\n";
    for (int i = 0; i < 3; i++) {
        ssize_t r = sendto(s, msg, sizeof(msg) - 1, 0,
                           reinterpret_cast<struct sockaddr *>(&dst), sizeof(dst));
        CHECK(r == static_cast<ssize_t>(sizeof(msg) - 1));
    }

    /* Wait for data to arrive. */
    struct pollfd pf{ l.fd(), POLLIN, 0 };
    CHECK(poll(&pf, 1, 500) > 0);

    /* Drain should consume all 3. */
    CHECK(l.drain() == 3);

    /* Second drain on empty socket returns 0. */
    CHECK(l.drain() == 0);

    close(s);
}
