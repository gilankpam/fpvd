#include "dynlink/controller.hpp"

#include "dynlink/apply_direction.hpp"

#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <sys/eventfd.h>
#include <sys/socket.h>
#include <sys/timerfd.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

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

static uint64_t nowMonotonicMs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000ull +
           static_cast<uint64_t>(ts.tv_nsec) / 1000000ull;
}

// Phase state for the staggered apply (port of apply_state_t in dl_applier.c).
enum class ApplyState {
    Idle = 0,
    UpGap,    // phase 1 = tx+radio applied; encoder pending
    DownGap,  // phase 1 = encoder applied; tx+radio pending
};

static int openTickTimer(uint32_t intervalMs) {
    int fd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);
    if (fd < 0) return -1;
    struct itimerspec ts{};
    ts.it_value.tv_sec = intervalMs / 1000;
    ts.it_value.tv_nsec = static_cast<long>(intervalMs % 1000) * 1000000L;
    ts.it_interval = ts.it_value;
    if (timerfd_settime(fd, 0, &ts, nullptr) < 0) {
        close(fd);
        return -1;
    }
    return fd;
}

static int armGap(int gapFd, uint32_t ms) {
    struct itimerspec ts{};
    ts.it_value.tv_sec = ms / 1000;
    ts.it_value.tv_nsec = static_cast<long>(ms % 1000) * 1000000L;
    // it_interval stays zero -> single-shot.
    return timerfd_settime(gapFd, 0, &ts, nullptr);
}

static int disarmGap(int gapFd) {
    struct itimerspec ts{};
    return timerfd_settime(gapFd, 0, &ts, nullptr);
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

    // Construct backend clients fresh from the snapshot + endpoints. They are
    // used only from the run() thread, so no locking is needed on them.
    wfb_      = std::make_unique<WfbControlClient>(ep_.wfbCtlAddr, ep_.wfbCtlPort);
    enc_.emplace(ep_.encHost, ep_.encPort, snap.minIdrIntervalMs, snap.roiQp);
    radio_.emplace(snap.iface);
    osd_.emplace(ep_.osdMsgPath, snap.osdEnabled, ep_.osdUpdateIntervalMs, snap.osdDebugLatency);
    watchdog_.emplace(snap.healthTimeoutMs);
    dedup_.reset();

    // Reset diff baselines to "first/invalid" so the first decision re-emits all.
    lastTx_ = Decision{};
    lastRadio_ = Decision{};
    lastEnc_ = Decision{};
    lastApplied_ = Decision{};
    lastDecisionMs_ = 0;

    int efd = eventfd(0, EFD_NONBLOCK | EFD_CLOEXEC);
    eventFd_.store(efd);                // -1 tolerated (run() handles it)
    {
        std::lock_guard<std::mutex> sg(statusMu_);
        status_.running = true;
        status_.watchdogTripped = false;
        status_.lastDecisionAgeMs = -1;
    }
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

// ---- dispatch helpers -------------------------------------------------------

// Port of dl_backend_tx_apply: FEC (if k/n differ) -> DEPTH (if interleaving
// supported && depth differ) -> RADIO (if mcs/bandwidth differ), with
// sub-pacing (usleep applySubPaceMs) between sub-commands. Updates lastTx_.
// d.bandwidth is already the 20/40 radiotap value — passed directly.
void DynamicLinkController::dispatchTxApply(const DlRuntimeConfig& cfg, const Decision& d) {
    bool first = (lastTx_.magic != kWireMagic);
    bool emitted = false;
    useconds_t paceUs = static_cast<useconds_t>(cfg.applySubPaceMs) * 1000u;

    if (first || lastTx_.k != d.k || lastTx_.n != d.n) {
        wfb_->setFec(d.k, d.n);
        emitted = true;
    }
    if (cfg.interleavingSupported && (first || lastTx_.depth != d.depth)) {
        if (emitted && paceUs > 0) usleep(paceUs);
        wfb_->setInterleaveDepth(d.depth);
        emitted = true;
    }
    if (first || lastTx_.mcs != d.mcs || lastTx_.bandwidth != d.bandwidth) {
        if (emitted && paceUs > 0) usleep(paceUs);
        wfb_->setRadio(/*stbc=*/0, /*ldpc=*/false, /*shortGi=*/false,
                       /*bandwidth=*/d.bandwidth, /*mcs=*/d.mcs,
                       /*vhtMode=*/false, /*vhtNss=*/1);
    }
    lastTx_ = d;
}

// Port of dl_backend_tx_apply_safe: emit FEC + (DEPTH if interleaving) + RADIO
// unconditionally, with sub-pacing. safe.bandwidth is the 20/40 radiotap value.
void DynamicLinkController::dispatchTxSafe(const DlRuntimeConfig& cfg) {
    useconds_t paceUs = static_cast<useconds_t>(cfg.applySubPaceMs) * 1000u;
    wfb_->setFec(cfg.safe.k, cfg.safe.n);
    if (cfg.interleavingSupported) {
        if (paceUs > 0) usleep(paceUs);
        wfb_->setInterleaveDepth(cfg.safe.depth);
    }
    if (paceUs > 0) usleep(paceUs);
    wfb_->setRadio(/*stbc=*/0, /*ldpc=*/false, /*shortGi=*/false,
                   /*bandwidth=*/cfg.safe.bandwidth, /*mcs=*/cfg.safe.mcs,
                   /*vhtMode=*/false, /*vhtNss=*/1);
}

// ---- poll loop --------------------------------------------------------------

void DynamicLinkController::run(int evfd) {
    // evfd passed by value from start() — captured before thread launch,
    // so it is immune to the eventFd_.exchange(-1) in stopLocked().

    // Snapshot the config once for the lifetime of this run() (Task 17 will
    // add hot-reconcile on the eventfd wake). cfg_ is stable across start().
    std::shared_ptr<const DlRuntimeConfig> cfgPtr;
    { std::lock_guard<std::mutex> cg(cfgMu_); cfgPtr = cfg_; }
    const DlRuntimeConfig cfg = cfgPtr ? *cfgPtr : DlRuntimeConfig{};

    // Open listen socket; tolerate failure — loop still runs for clean stop.
    int listenFd = -1;
    if (ep_.listenPort != 0) {
        listenFd = openListenSocket(ep_.listenAddr, ep_.listenPort);
    }

    // Tick timer: interval = min(osdUpdateIntervalMs, healthTimeoutMs/2),
    // floored at 100 ms (same math as dl_applier.c).
    uint32_t tickMs = ep_.osdUpdateIntervalMs;
    if (cfg.healthTimeoutMs / 2 < tickMs) tickMs = cfg.healthTimeoutMs / 2;
    if (tickMs < 100) tickMs = 100;
    int tickFd = openTickTimer(tickMs);

    // Gap timer (single-shot, armed on demand for staggered apply).
    int gapFd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);

    // Build pollfd array: [listenFd?, tickFd?, gapFd?, evfd?].
    struct pollfd pfds[4];
    int nfds = 0;
    int listenIdx = -1, tickIdx = -1, gapIdx = -1, eventIdx = -1;

    if (listenFd >= 0) { pfds[nfds].fd = listenFd; pfds[nfds].events = POLLIN; listenIdx = nfds++; }
    if (tickFd   >= 0) { pfds[nfds].fd = tickFd;   pfds[nfds].events = POLLIN; tickIdx   = nfds++; }
    if (gapFd    >= 0) { pfds[nfds].fd = gapFd;    pfds[nfds].events = POLLIN; gapIdx    = nfds++; }
    if (evfd     >= 0) { pfds[nfds].fd = evfd;     pfds[nfds].events = POLLIN; eventIdx  = nfds++; }

    ApplyState applyState = ApplyState::Idle;
    Decision   applyPending{};

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
            // (Task 17 will reconcile cfg_ here on a reload wake.)
        }

        // ---- listenFd readable: decode + dispatch a decision ----------------
        if (listenIdx >= 0 && (pfds[listenIdx].revents & POLLIN)) {
            uint8_t buf[256];
            struct sockaddr_in src{};
            socklen_t slen = sizeof(src);
            ssize_t got = recvfrom(listenFd, buf, sizeof(buf), 0,
                                   reinterpret_cast<struct sockaddr*>(&src), &slen);
            if (got < 0) {
                // EAGAIN/EINTR: nothing to do.
            } else if (peekKind(buf, static_cast<size_t>(got)) == PacketKind::Decision) {
                Decision d{};
                DecodeResult dr = decodeDecision(buf, static_cast<size_t>(got), d);
                if (dr != DecodeResult::Ok) {
                    // bad decode — ignore (matches dl_applier.c logging branch).
                } else if (dedup_.check(d.sequence)) {
                    // duplicate / stale seq — drop.
                } else {
                    // New decision supersedes any in-flight phase 2; the
                    // per-backend diff below reapplies anything that differs.
                    if (applyState != ApplyState::Idle) {
                        disarmGap(gapFd);
                        applyState = ApplyState::Idle;
                    }

                    uint64_t now = nowMonotonicMs();
                    bool first = (lastEnc_.magic != kWireMagic);
                    ApplyDir dir = applyDirection(lastEnc_.bitrateKbps,
                                                  d.bitrateKbps, first);
                    useconds_t subPaceUs =
                        static_cast<useconds_t>(cfg.applySubPaceMs) * 1000u;

                    if (cfg.applyStaggerMs == 0 || dir == ApplyDir::Equal) {
                        // Single shot: all backends fire now.
                        dispatchTxApply(cfg, d);
                        radio_->apply(d.txPowerDbm);
                        enc_->apply(d.bitrateKbps, d.fps);
                        lastRadio_ = d;
                        lastEnc_ = d;
                    } else if (dir == ApplyDir::Up) {
                        // Power up BEFORE MCS up. radio -> (sub-pace) -> tx;
                        // encoder bitrate expands after the outer gap.
                        radio_->apply(d.txPowerDbm);
                        lastRadio_ = d;
                        if (subPaceUs > 0) usleep(subPaceUs);
                        dispatchTxApply(cfg, d);
                        applyPending = d;
                        applyState = ApplyState::UpGap;
                        armGap(gapFd, cfg.applyStaggerMs);
                    } else {  // ApplyDir::Down
                        // Throttle producer first, then narrow capacity after gap.
                        enc_->apply(d.bitrateKbps, d.fps);
                        lastEnc_ = d;
                        applyPending = d;
                        applyState = ApplyState::DownGap;
                        armGap(gapFd, cfg.applyStaggerMs);
                    }

                    lastApplied_ = d;
                    osd_->writeStatus(lastApplied_, 0);
                    watchdog_->notifyDecision(now);
                    lastDecisionMs_ = now;
                    {
                        std::lock_guard<std::mutex> sg(statusMu_);
                        status_.watchdogTripped = false;
                        status_.lastDecisionAgeMs = 0;
                    }
                }
            }
        }

        // ---- tick timer: watchdog + OSD periodic refresh --------------------
        if (tickIdx >= 0 && (pfds[tickIdx].revents & POLLIN)) {
            uint64_t expirations;
            ssize_t r = read(tickFd, &expirations, sizeof(expirations));
            (void)r;  // drained; count ignored
            uint64_t now = nowMonotonicMs();
            if (watchdog_->tick(now)) {
                // Drop any queued phase 2 — safe values supersede.
                if (applyState != ApplyState::Idle) {
                    disarmGap(gapFd);
                    applyState = ApplyState::Idle;
                }
                dispatchTxSafe(cfg);
                radio_->applySafe(cfg.safe.txPowerDbm);
                enc_->applySafe(cfg.safe.bitrateKbps);
                osd_->eventWatchdog();
                // Invalidate last-states so the next fresh decision emits
                // everything; reset dedup so a restarted GS recovers.
                // (Port of dl_applier.c's memset(&last_tx/radio/enc, 0).)
                // RadioTxpower keeps its own internal diff baseline, so also
                // invalidate it here — setIface(cfg.iface) resets current_ to
                // nullopt, forcing the next decision to re-run iw even if its
                // txpower equals the safe value just pushed.
                lastTx_ = Decision{};
                lastRadio_ = Decision{};
                lastEnc_ = Decision{};
                radio_->setIface(cfg.iface);
                dedup_.reset();
                {
                    std::lock_guard<std::mutex> sg(statusMu_);
                    status_.watchdogTripped = true;
                }
            }
            // NB: dl_applier.c does NOT refresh the OSD status line on the
            // tick — writeStatus() clears the event line, which would wipe a
            // just-emitted WATCHDOG toast. OSD status is written only in the
            // decision branch; the tick only emits the watchdog event.
            if (lastDecisionMs_ != 0) {
                std::lock_guard<std::mutex> sg(statusMu_);
                status_.lastDecisionAgeMs =
                    static_cast<long>(now - lastDecisionMs_);
            }
        }

        // ---- gap timer: phase-2 apply ---------------------------------------
        if (gapIdx >= 0 && (pfds[gapIdx].revents & POLLIN)) {
            uint64_t expirations;
            ssize_t r = read(gapFd, &expirations, sizeof(expirations));
            (void)r;  // always drain to clear POLLIN
            if (applyState == ApplyState::UpGap) {
                enc_->apply(applyPending.bitrateKbps, applyPending.fps);
                lastEnc_ = applyPending;
            } else if (applyState == ApplyState::DownGap) {
                dispatchTxApply(cfg, applyPending);
                if (cfg.applySubPaceMs > 0) {
                    usleep(static_cast<useconds_t>(cfg.applySubPaceMs) * 1000u);
                }
                radio_->apply(applyPending.txPowerDbm);
                lastRadio_ = applyPending;
            }
            // Idle here means a stale expiration the kernel queued before
            // disarm landed — drained, ignore.
            applyState = ApplyState::Idle;
        }
    }

    if (listenFd >= 0) ::close(listenFd);
    if (tickFd   >= 0) ::close(tickFd);
    if (gapFd    >= 0) ::close(gapFd);
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
