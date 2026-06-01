#include "dynlink/controller.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace fpvd::dynlink {

// ---- helpers ----------------------------------------------------------------

static int openListenSocket(const std::string& addr, uint16_t port) {
    int fd = socket(AF_INET, SOCK_DGRAM | SOCK_NONBLOCK | SOCK_CLOEXEC, 0);
    if (fd < 0) return -1;

    int one = 1;
    setsockopt(fd, SOL_SOCKET, SO_REUSEADDR, &one, sizeof(one));

    struct sockaddr_in sa{};
    sa.sin_family = AF_INET;
    sa.sin_port = htons(port);
    if (inet_pton(AF_INET, addr.c_str(), &sa.sin_addr) != 1) {
        close(fd);
        return -1;
    }
    if (bind(fd, reinterpret_cast<struct sockaddr*>(&sa), sizeof(sa)) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

// ---- lifecycle --------------------------------------------------------------

DynamicLinkController::DynamicLinkController(Endpoints ep)
    : ep_(std::move(ep)) {}

DynamicLinkController::~DynamicLinkController() {
    stop();
}

void DynamicLinkController::start(const DlRuntimeConfig& snap, uint32_t generationId) {
    std::lock_guard<std::mutex> lk(lifetimeMu_);
    if (running_.load()) stopLocked();  // NOT stop() — avoid re-locking lifetimeMu_

    { std::lock_guard<std::mutex> cg(cfgMu_); cfg_ = std::make_shared<const DlRuntimeConfig>(snap); }
    generationId_.store(generationId);
    stopFlag_.store(false);
    int efd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    eventFd_.store(efd);                // -1 tolerated (run() handles it)
    { std::lock_guard<std::mutex> sg(statusMu_); status_.running = true; }
    running_.store(true);
    thread_ = std::thread([this, efd]{ run(efd); });
}

void DynamicLinkController::stop() {
    std::lock_guard<std::mutex> lk(lifetimeMu_);
    stopLocked();
}

void DynamicLinkController::stopLocked() {
    if (!running_.load()) return;
    stopFlag_.store(true);
    int fd = eventFd_.exchange(-1);  // take ownership of the fd atomically
    if (fd >= 0) { uint64_t one = 1; ssize_t w = write(fd, &one, sizeof(one)); (void)w; }
    if (thread_.joinable()) thread_.join();
    if (fd >= 0) ::close(fd);        // stop() owns the close, AFTER join
    running_.store(false);
    { std::lock_guard<std::mutex> sg(statusMu_); status_.running = false; }
}

// ---- poll loop --------------------------------------------------------------

void DynamicLinkController::run(int evfd) {
    // evfd passed by value from start() — captured before thread launch,
    // so it is immune to the eventFd_.exchange(-1) in stopLocked().

    // Open listen socket; tolerate failure — loop still runs for clean stop.
    int listenFd = -1;
    if (ep_.listenPort != 0) {
        listenFd = openListenSocket(ep_.listenAddr, ep_.listenPort);
    }

    // Build pollfd array: [listenFd (optional), evfd (optional)].
    struct pollfd pfds[2];
    int nfds = 0;
    int listenIdx = -1;
    int eventIdx  = -1;

    if (listenFd >= 0) {
        pfds[nfds].fd = listenFd;
        pfds[nfds].events = POLLIN;
        listenIdx = nfds++;
    }
    if (evfd >= 0) {
        pfds[nfds].fd = evfd;
        pfds[nfds].events = POLLIN;
        eventIdx = nfds++;
    }

    while (true) {
        if (nfds == 0) {
            // No fds to poll — spin-check stopFlag with a short sleep.
            if (stopFlag_.load()) break;
            struct timespec ts{0, 10 * 1000 * 1000}; // 10 ms
            nanosleep(&ts, nullptr);
            continue;
        }

        int n = poll(pfds, static_cast<nfds_t>(nfds), -1);
        if (n < 0) {
            if (errno == EINTR) continue;
            break;
        }

        // Check eventfd first (stop/reload signal).
        if (eventIdx >= 0 && (pfds[eventIdx].revents & POLLIN)) {
            uint64_t val;
            ssize_t r = read(evfd, &val, sizeof(val));
            (void)r; // drain; ignore EAGAIN
            if (stopFlag_.load()) break;
        }

        // listenFd readable: drain and discard (Task 14 will decode).
        if (listenIdx >= 0 && (pfds[listenIdx].revents & POLLIN)) {
            uint8_t buf[256];
            struct sockaddr_in src{};
            socklen_t slen = sizeof(src);
            ssize_t got = recvfrom(listenFd, buf, sizeof(buf), 0,
                                   reinterpret_cast<struct sockaddr*>(&src), &slen);
            (void)got; // discard — Task 14 adds decode
        }
    }

    if (listenFd >= 0) {
        ::close(listenFd);  // close ONLY listenFd here
    }
    // do NOT close evfd/eventFd_ here — stop() owns the close after join()
}

// ---- status / config --------------------------------------------------------

DlStatus DynamicLinkController::status() const {
    std::lock_guard<std::mutex> lk(statusMu_);
    DlStatus s = status_;
    s.running = running_.load();
    return s;
}

void DynamicLinkController::publishStatus(const DlStatus& s) {
    std::lock_guard<std::mutex> lk(statusMu_);
    status_ = s;
}

void DynamicLinkController::setConfig(const DlRuntimeConfig& snap) {
    std::lock_guard<std::mutex> lk(lifetimeMu_);  // excludes a concurrent stop closing the fd
    { std::lock_guard<std::mutex> cg(cfgMu_); cfg_ = std::make_shared<const DlRuntimeConfig>(snap); }
    int fd = eventFd_.load();
    if (fd >= 0) { uint64_t one = 1; ssize_t w = write(fd, &one, sizeof(one)); (void)w; }
}

} // namespace fpvd::dynlink
