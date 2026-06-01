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
    // If already running, stop first.
    if (running_.load()) {
        stop();
    }

    // Store config under mutex.
    {
        std::lock_guard<std::mutex> lk(cfgMu_);
        cfg_ = std::make_shared<const DlRuntimeConfig>(snap);
    }
    generationId_ = generationId;

    // Create eventfd for stop/reload signalling.
    eventFd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    if (eventFd_ < 0) {
        // Non-fatal: the loop can still run (poll will just block on listenFd
        // only), but stop() will not be able to wake it.  Record failure in
        // status but mark running anyway so the destructor can join safely.
    }

    stopFlag_.store(false);
    running_.store(true);

    {
        std::lock_guard<std::mutex> lk(statusMu_);
        status_.running = true;
    }

    thread_ = std::thread([this] { run(); });
}

void DynamicLinkController::stop() {
    if (!running_.load()) return;

    stopFlag_.store(true);

    // Wake the poll loop via eventfd.
    if (eventFd_ >= 0) {
        uint64_t one = 1;
        ssize_t r = write(eventFd_, &one, sizeof(one));
        (void)r;
    }

    if (thread_.joinable()) {
        thread_.join();
    }

    // eventFd_ is closed by run() on exit; just clear the member.
    eventFd_ = -1;

    running_.store(false);
    {
        std::lock_guard<std::mutex> lk(statusMu_);
        status_.running = false;
    }
}

// ---- poll loop --------------------------------------------------------------

void DynamicLinkController::run() {
    // Open listen socket; tolerate failure — loop still runs for clean stop.
    int listenFd = -1;
    if (ep_.listenPort != 0) {
        listenFd = openListenSocket(ep_.listenAddr, ep_.listenPort);
    }

    // Build pollfd array: [listenFd (optional), eventFd_ (optional)].
    struct pollfd pfds[2];
    int nfds = 0;
    int listenIdx = -1;
    int eventIdx  = -1;

    if (listenFd >= 0) {
        pfds[nfds].fd = listenFd;
        pfds[nfds].events = POLLIN;
        listenIdx = nfds++;
    }
    if (eventFd_ >= 0) {
        pfds[nfds].fd = eventFd_;
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
            ssize_t r = read(eventFd_, &val, sizeof(val));
            (void)r; // drain
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
        close(listenFd);
    }
    if (eventFd_ >= 0) {
        close(eventFd_);
        // NOTE: stop() zeroes eventFd_ after join(); we close the fd here
        // so the fd is closed before join() returns.  stop() will set -1.
    }
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
    {
        std::lock_guard<std::mutex> lk(cfgMu_);
        cfg_ = std::make_shared<const DlRuntimeConfig>(snap);
    }
    // Wake the loop so it can act on the new config (Task 17 will use it).
    if (eventFd_ >= 0) {
        uint64_t one = 1;
        ssize_t r = write(eventFd_, &one, sizeof(one));
        (void)r;
    }
}

} // namespace fpvd::dynlink
