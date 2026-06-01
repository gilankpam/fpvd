/* idr_listen.cpp — UDP listener for PixelPilot IDR-token bursts. */
#include "dynlink/idr_listen.hpp"

#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cstdio>   // fprintf / stderr for error logging

namespace fpvd::dynlink {

IdrListener::IdrListener(const std::string& bindAddr, uint16_t port) {
    if (port == 0) return;  // disabled — fd_ stays -1

    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK, 0);
    if (fd < 0) {
        fprintf(stderr, "idr_listen: socket: %s\n", strerror(errno));
        return;
    }

    int one = 1;
    (void)setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port   = htons(port);
    if (bindAddr.empty()) {
        sa.sin_addr.s_addr = htonl(INADDR_ANY);
    } else if (inet_pton(AF_INET, bindAddr.c_str(), &sa.sin_addr) != 1) {
        fprintf(stderr, "idr_listen: bad bind_addr '%s'\n", bindAddr.c_str());
        close(fd);
        return;
    }

    if (bind(fd, reinterpret_cast<struct sockaddr *>(&sa), sizeof(sa)) < 0) {
        fprintf(stderr, "idr_listen: bind %s:%u: %s\n",
                bindAddr.empty() ? "0.0.0.0" : bindAddr.c_str(),
                static_cast<unsigned>(port), strerror(errno));
        close(fd);
        return;
    }

    fd_ = fd;
    fprintf(stderr, "idr_listen: bound %s:%u\n",
            bindAddr.empty() ? "0.0.0.0" : bindAddr.c_str(),
            static_cast<unsigned>(port));
}

IdrListener::~IdrListener() {
    if (fd_ >= 0) close(fd_);
}

size_t IdrListener::drain() {
    if (fd_ < 0) return 0;

    size_t count = 0;
    uint8_t buf[64];
    while (true) {
        ssize_t n = recvfrom(fd_, buf, sizeof(buf), 0, nullptr, nullptr);
        if (n < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK) break;
            if (errno == EINTR) continue;
            fprintf(stderr, "idr_listen: recvfrom: %s\n", strerror(errno));
            break;
        }
        ++count;
    }
    return count;
}

} // namespace fpvd::dynlink
