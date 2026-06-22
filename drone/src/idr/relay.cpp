/* relay.cpp — always-on IDR keyframe-request relay. */
#include "idr/relay.hpp"

#include <cerrno>
#include <ctime>
#include <poll.h>
#include <sys/eventfd.h>
#include <unistd.h>

namespace fpvd::idr {

static uint64_t nowMonotonicMs() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000 + static_cast<uint64_t>(ts.tv_nsec) / 1000000;
}

IdrRelay::IdrRelay(WaybeamClient& enc, std::string bindAddr, uint16_t port, uint32_t minIntervalMs)
    : listener_(bindAddr, port), enc_(&enc), minIntervalMs_(minIntervalMs) {}

IdrRelay::~IdrRelay() { stop(); }

int IdrRelay::requestIdr(uint64_t nowMs) {
    if (everSent_ && (nowMs - lastIdrMs_) < static_cast<uint64_t>(minIntervalMs_)) {
        return 1; // throttled
    }
    count_.fetch_add(1, std::memory_order_relaxed); // one logical request sent to encoder
    bool ok = enc_->get("/request/idr");
    lastIdrMs_ = nowMs; // arm throttle on ANY attempt (even failure)
    everSent_ = true;
    return ok ? 0 : -1;
}

void IdrRelay::start() {
    if (listener_.fd() < 0)
        return; // disabled — nothing to listen to
    if (thread_.joinable())
        return; // already running
    stop_.store(false);
    wakeFd_ = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    thread_ = std::thread([this] { run(); });
}

void IdrRelay::stop() {
    if (!thread_.joinable()) {
        if (wakeFd_ >= 0) {
            close(wakeFd_);
            wakeFd_ = -1;
        }
        return;
    }
    stop_.store(true);
    if (wakeFd_ >= 0) {
        uint64_t one = 1;
        ssize_t w = write(wakeFd_, &one, sizeof(one));
        (void)w;
    }
    thread_.join();
    if (wakeFd_ >= 0) {
        close(wakeFd_);
        wakeFd_ = -1;
    }
}

void IdrRelay::run() {
    struct pollfd pfds[2];
    pfds[0].fd = listener_.fd();
    pfds[0].events = POLLIN;
    pfds[1].fd = wakeFd_;
    pfds[1].events = POLLIN;

    while (!stop_.load()) {
        int n = poll(pfds, 2, -1);
        if (n < 0) {
            if (errno == EINTR)
                continue;
            break;
        }
        if (pfds[1].revents & POLLIN) {
            uint64_t val;
            ssize_t r = read(wakeFd_, &val, sizeof(val));
            (void)r; // drain the wake; loop re-checks stop_
            if (stop_.load())
                break;
        }
        if (pfds[0].revents & POLLIN) {
            size_t got = listener_.drain();
            if (got > 0) {
                requestIdr(nowMonotonicMs()); // bumps count_ only when not throttled
            }
        }
    }
}

} // namespace fpvd::idr
