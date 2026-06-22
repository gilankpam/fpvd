#include "dynlink/controller.hpp"

#include "dynlink/apply_direction.hpp"
#include "dynlink/local_compute.hpp"
#include "dynlink/txpower_curve.hpp"

#include <cassert>
#include <cstring>
#include <arpa/inet.h>
#include <errno.h>
#include <netinet/in.h>
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
    : ep_(std::move(ep)), wb_(ep_.encHost, ep_.encPort) {}

DynamicLinkController::~DynamicLinkController() {
    stop();
}

void DynamicLinkController::start(const DlRuntimeConfig& snap) {
    std::lock_guard<std::mutex> lk(lifetimeMu_);
    if (running_.load()) stopLocked();  // NOT stop() — avoid re-locking lifetimeMu_

    { std::lock_guard<std::mutex> cg(cfgMu_); cfg_ = std::make_shared<const DlRuntimeConfig>(snap); }
    stopFlag_.store(false);

    // Construct backend clients fresh from the snapshot + endpoints. They are
    // used only from the run() thread, so no locking is needed on them.
    wfb_      = std::make_unique<WfbControlClient>(ep_.wfbCtlAddr, ep_.wfbCtlPort);
    // Probe retune client: built only when a control port is configured; a fresh
    // unique_ptr per start() (reset first) so a stop()+start() cycle rebuilds it,
    // mirroring wfb_. Held nullptr when the probe is disabled.
    probeWfb_.reset();
    if (snap.probeCtlPort != 0)
        probeWfb_ = std::make_unique<WfbControlClient>("127.0.0.1", snap.probeCtlPort);
    enc_.emplace(wb_, snap.roiQp);
    radio_.emplace(snap.iface);
    // osd_ is the daemon-owned always-on writer, injected via setOsdWriter().
    watchdog_.emplace(snap.healthTimeoutMs);
    // IDR keyframe requests are handled by the always-on idr::IdrRelay
    // (daemon-supervised), not this controller — see src/idr/relay.hpp.
    dedup_.reset();

    // Reset diff baselines to "first/invalid" so the first decision re-emits all.
    lastTx_ = Decision{};
    lastRadio_ = Decision{};   // vestigial — radio_/lastRadio_ removed in Phase 3b
    lastEnc_ = Decision{};
    lastApplied_ = Decision{};
    lastProbeMcs_ = -1;            // force the probe to re-tune on the first decision
    lastDecisionMs_ = 0;
    lastOsdWriteMs_ = 0;           // first decision writes the OSD immediately

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

// Port of dl_backend_tx_apply: FEC (if k/n differ) -> RADIO (if mcs/bandwidth
// differ), with sub-pacing (usleep applySubPaceMs) between sub-commands.
// Updates lastTx_.
// d.bandwidth is already the 20/40 radiotap value — passed directly.
void DynamicLinkController::dispatchTxApply(const DlRuntimeConfig& cfg, const Decision& d) {
    bool first = (lastTx_.magic != kWireMagic);
    bool emitted = false;
    useconds_t paceUs = static_cast<useconds_t>(cfg.applySubPaceMs) * 1000u;

    if (first || lastTx_.k != d.k || lastTx_.n != d.n) {
        wfb_->setFec(d.k, d.n);
        emitted = true;
    }
    if (first || lastTx_.mcs != d.mcs || lastTx_.bandwidth != d.bandwidth) {
        if (emitted && paceUs > 0) usleep(paceUs);
        // stbc/ldpc are preserved from config, not decided by the loop (the GS
        // decision carries neither). CMD_SET_RADIO is atomic over the whole
        // radiotap word, so we must restate the configured flags on every push.
        wfb_->setRadio(/*stbc=*/static_cast<uint8_t>(cfg.stbc ? 1 : 0),
                       /*ldpc=*/cfg.ldpc, /*shortGi=*/false,
                       /*bandwidth=*/d.bandwidth, /*mcs=*/d.mcs,
                       /*vhtMode=*/false, /*vhtNss=*/1);
        // Retune the observe-only probe to current+1 (clamped), mirroring the
        // video PHY flags. Best-effort: a soft failure (probe not yet up) is
        // retried on the next decision. The probe rides its own radio_port, so
        // this never touches the video stream.
        if (probeWfb_) {
            int rung = probeRungFor(d.mcs, cfg.probeMcsCeiling);
            if (rung != lastProbeMcs_) {
                probeWfb_->setRadio(static_cast<uint8_t>(cfg.stbc ? 1 : 0),
                                    cfg.ldpc, /*shortGi=*/false,
                                    /*bandwidth=*/d.bandwidth,
                                    /*mcs=*/static_cast<uint8_t>(rung),
                                    /*vhtMode=*/false, /*vhtNss=*/1);
                lastProbeMcs_ = rung;
            }
        }
        // Per-MCS tx power (operating-rung coupling): back off on the high-PAPR
        // 64-QAM rungs to keep the PA linear, full power at low MCS for range.
        // RadioTxpower::apply is diff-based, so iw only runs when the value changes.
        if (radio_) radio_->apply(d.txPowerDbm);
    }
    lastTx_ = d;
}

// Derives a Decision at the robust MCS-0 floor (kDlFailsafeMcs) through the
// same applyLocalCompute path as a normal decision (GS-decides-MCS,
// drone-derives-the-rest). Bandwidth is pinned to the operating width — never
// drop bandwidth on a watchdog trip.
Decision DynamicLinkController::dispatchTxSafe(const DlRuntimeConfig& cfg) {
    useconds_t paceUs = static_cast<useconds_t>(cfg.applySubPaceMs) * 1000u;
    Decision d{};
    d.mcs       = kDlFailsafeMcs;
    d.bandwidth = cfg.linkBandwidth;
    applyLocalCompute(cfg, d);   // fills k, n, bitrateKbps, fps, txPowerDbm
    wfb_->setFec(d.k, d.n);
    if (paceUs > 0) usleep(paceUs);
    // Preserve the configured stbc/ldpc (robustness coding helps during recovery).
    wfb_->setRadio(/*stbc=*/static_cast<uint8_t>(cfg.stbc ? 1 : 0),
                   /*ldpc=*/cfg.ldpc, /*shortGi=*/false,
                   /*bandwidth=*/d.bandwidth, /*mcs=*/d.mcs,
                   /*vhtMode=*/false, /*vhtNss=*/1);
    // Move the probe down with the video so it never sits above the safe rung.
    if (probeWfb_) {
        int rung = probeRungFor(d.mcs, cfg.probeMcsCeiling);
        probeWfb_->setRadio(static_cast<uint8_t>(cfg.stbc ? 1 : 0), cfg.ldpc, false,
                            d.bandwidth, static_cast<uint8_t>(rung), false, 1);
        lastProbeMcs_ = rung;
    }
    // Low MCS -> high power -> robust recovery (txPowerDbm == curve[0] from derive).
    if (radio_) radio_->applySafe(d.txPowerDbm);
    return d;
}

// ---- poll loop --------------------------------------------------------------

void DynamicLinkController::run(int evfd) {
    // evfd passed by value from start() — captured before thread launch,
    // so it is immune to the eventFd_.exchange(-1) in stopLocked().

    // Snapshot the config once at start; hot-reconcile on eventfd wakes updates
    // the local `cfg` in-place (Task 17). cfg_ is always set in start() before
    // this thread is launched, so cfgPtr must never be null here.
    std::shared_ptr<const DlRuntimeConfig> cfgPtr;
    { std::lock_guard<std::mutex> cg(cfgMu_); cfgPtr = cfg_; }
    assert(cfgPtr && "controller started without config");
    DlRuntimeConfig cfg = *cfgPtr;  // mutable: hot-reconcile updates this in-place

    // Open listen socket; tolerate failure — loop still runs for clean stop.
    int listenFd = -1;
    if (ep_.listenPort != 0) {
        listenFd = openListenSocket(ep_.listenAddr, ep_.listenPort);
    }

    // Tick timer: interval = min(osdUpdateIntervalMs, healthTimeoutMs/2),
    // floored at 100 ms (same math as dl_applier.c).
    // tickMs is kept as a local so hot-reconcile can detect changes and re-arm.
    auto computeTickMs = [&](const DlRuntimeConfig& c) -> uint32_t {
        uint32_t t = ep_.osdUpdateIntervalMs;
        if (c.healthTimeoutMs / 2 < t) t = c.healthTimeoutMs / 2;
        if (t < 100) t = 100;
        return t;
    };
    uint32_t tickMs = computeTickMs(cfg);
    int tickFd = openTickTimer(tickMs);

    // Gap timer (single-shot, armed on demand for staggered apply).
    int gapFd = timerfd_create(CLOCK_MONOTONIC, TFD_NONBLOCK | TFD_CLOEXEC);

    // Build pollfd array: [listenFd?, tickFd?, gapFd?, evfd?].
    struct pollfd pfds[4];
    int nfds = 0;
    int listenIdx = -1, tickIdx = -1, gapIdx = -1, eventIdx = -1;

    if (listenFd    >= 0) { pfds[nfds].fd = listenFd;    pfds[nfds].events = POLLIN; listenIdx = nfds++; }
    if (tickFd      >= 0) { pfds[nfds].fd = tickFd;      pfds[nfds].events = POLLIN; tickIdx   = nfds++; }
    if (gapFd       >= 0) { pfds[nfds].fd = gapFd;       pfds[nfds].events = POLLIN; gapIdx    = nfds++; }
    if (evfd        >= 0) { pfds[nfds].fd = evfd;        pfds[nfds].events = POLLIN; eventIdx  = nfds++; }

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

            // Hot-reload reconcile: load the latest cfg_ snapshot and apply
            // structural changes that the components hold their own copies of.
            // The bulk of cfg knobs (safe.*, applyStaggerMs, applySubPaceMs)
            // are picked up automatically on the next iteration via the
            // now-mutable local `cfg`.
            std::shared_ptr<const DlRuntimeConfig> newCfgPtr;
            { std::lock_guard<std::mutex> cg(cfgMu_); newCfgPtr = cfg_; }
            if (newCfgPtr) {
                const DlRuntimeConfig& newCfg = *newCfgPtr;

                // Watchdog: update its internal timeout copy.
                if (watchdog_) watchdog_->setTimeout(newCfg.healthTimeoutMs);

                // Tick timer: re-arm if the interval changed.
                uint32_t newTickMs = computeTickMs(newCfg);
                if (newTickMs != tickMs && tickFd >= 0) {
                    struct itimerspec ts{};
                    ts.it_value.tv_sec  = newTickMs / 1000;
                    ts.it_value.tv_nsec = static_cast<long>(newTickMs % 1000) * 1000000L;
                    ts.it_interval = ts.it_value;
                    timerfd_settime(tickFd, 0, &ts, nullptr);
                    tickMs = newTickMs;
                }

                // Encoder: update ROI curve.
                if (enc_) {
                    enc_->setRoiCurve(newCfg.roiQp);
                }

                // OSD enabled is owned by the daemon (top-level osd.enabled),
                // not the controller — nothing to reconcile here.

                // stbc/ldpc are preserved, not decided — but a hot change to
                // them still has to reach the radio. dispatchTxApply only
                // re-emits CMD_SET_RADIO on an mcs/bandwidth change, so when
                // ONLY the radiotap flags changed we restate the radio here.
                const bool radiotapFlagsChanged =
                    (newCfg.stbc != cfg.stbc) || (newCfg.ldpc != cfg.ldpc);

                // Update the local cfg snapshot. safe.*, applyStaggerMs,
                // applySubPaceMs, and other loop-read knobs are now live on
                // the next iteration.
                cfg = newCfg;

                // Restate the radiotap with the controller's CURRENT mcs/bw
                // (never the config mcs — that would clobber the adaptive
                // selection). Gated on a decision baseline + idle apply state so
                // we never race the staggered-apply machine; if non-idle, the
                // next decision's dispatch carries the new flags.
                if (radiotapFlagsChanged && lastTx_.magic == kWireMagic &&
                    applyState == ApplyState::Idle && wfb_) {
                    wfb_->setRadio(static_cast<uint8_t>(cfg.stbc ? 1 : 0),
                                   cfg.ldpc, /*shortGi=*/false,
                                   lastTx_.bandwidth, lastTx_.mcs,
                                   /*vhtMode=*/false, /*vhtNss=*/1);
                }
            }
        }

        // ---- listenFd readable: decode + dispatch a decision/ack ----------------
        if (listenIdx >= 0 && (pfds[listenIdx].revents & POLLIN)) {
            uint8_t buf[256];
            struct sockaddr_in src{};
            socklen_t slen = sizeof(src);
            ssize_t got = recvfrom(listenFd, buf, sizeof(buf), 0,
                                   reinterpret_cast<struct sockaddr*>(&src), &slen);
            if (got < 0) {
                // EAGAIN/EINTR: nothing to do.
            } else {
                PacketKind kind = peekKind(buf, static_cast<size_t>(got));
                if (kind == PacketKind::Decision) {
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
                            if (gapFd >= 0) disarmGap(gapFd);
                            applyState = ApplyState::Idle;
                        }

                        // v3 wire carries only {mcs}; bandwidth is static config.
                        d.bandwidth = cfg.linkBandwidth;

                        // Phase 3a: the drone computes its own bitrate/k/n
                        // (and a constant fps) from {mcs,bandwidth};
                        // the GS-sent values on the wire are ignored.
                        applyLocalCompute(cfg, d);

                        uint64_t now = nowMonotonicMs();
                        bool first = (lastEnc_.magic != kWireMagic);
                        ApplyDir dir = applyDirection(lastEnc_.bitrateKbps,
                                                      d.bitrateKbps, first);

                        // canStagger: staggered dispatch requires both a non-zero
                        // gap interval AND a live gap timerfd. If the fd failed to
                        // open (gapFd < 0) we must fall back to single-shot so that
                        // the deferred phase-2 is not silently lost.
                        const bool canStagger = (cfg.applyStaggerMs != 0) && (gapFd >= 0);

                        if (!canStagger || dir == ApplyDir::Equal) {
                            // Single shot: all backends fire now.
                            dispatchTxApply(cfg, d);
                            enc_->apply(d.bitrateKbps, d.fps);
                            lastEnc_ = d;
                        } else if (dir == ApplyDir::Up) {
                            // Raise capacity (mcs) now; the encoder bitrate
                            // expands after the outer gap. dispatchTxApply also
                            // steps tx power with the mcs (per-MCS curve).
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
                        // Throttle OSD status writes to osdUpdateIntervalMs
                        // (default 1 Hz). Decisions arrive ~10 Hz; the OSD only
                        // needs ~1 Hz, so most writes are coalesced — but mcs/
                        // bitrate still display within one interval. The watchdog
                        // event line is untouched (it fires on the tick).
                        if (osd_ && osdWriteDue(now, lastOsdWriteMs_,
                                                ep_.osdUpdateIntervalMs)) {
                            osd_->writeStatus(lastApplied_,
                                              bfCodeProvider_ ? bfCodeProvider_() : 0,
                                              idrCountProvider_ ? idrCountProvider_() : 0);
                            lastOsdWriteMs_ = now;
                        }
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
                    if (gapFd >= 0) disarmGap(gapFd);
                    applyState = ApplyState::Idle;
                }
                Decision sf = dispatchTxSafe(cfg);
                enc_->applySafe(sf.bitrateKbps);
                if (osd_) osd_->eventWatchdog();
                // Invalidate last-states so the next fresh decision emits
                // everything; reset dedup so a restarted GS recovers.
                // (Port of dl_applier.c's memset(&last_tx/radio/enc, 0).)
                // NOTE: lastRadio_ is reset here for C-reference parity only —
                // it is NOT the txpower diff baseline (see hpp comment). The
                // real diff lives in RadioTxpower::current_; setIface() below
                // resets that to nullopt, forcing the next decision to re-run
                // iw even if its txpower equals the safe value just pushed.
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
                // Phase 2 of a Down: narrow tx capacity (mcs/fec)
                // now that the encoder already throttled in phase 1.
                // No lastEnc_ update here — it was set in phase 1
                // (the decision branch). dispatchTxApply also steps
                // tx power with the mcs (per-MCS curve).
                dispatchTxApply(cfg, applyPending);
            }
            // Idle here means a stale expiration the kernel queued before
            // disarm landed — drained, ignore.
            applyState = ApplyState::Idle;
        }

    }

    if (listenFd    >= 0) ::close(listenFd);
    if (tickFd      >= 0) ::close(tickFd);
    if (gapFd       >= 0) ::close(gapFd);
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
