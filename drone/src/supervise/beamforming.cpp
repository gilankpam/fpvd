#include "supervise/beamforming.hpp"
#include "link_width.hpp"
#include <cctype>
#include <chrono>
#include <fstream>
#include <sstream>
#include <sys/stat.h>

namespace fpvd {

static std::string extractMac(const std::string& text) {
    // Find the first aa:bb:cc:dd:ee:ff token.
    for (size_t i = 0; i + 17 <= text.size(); ++i) {
        bool ok = true;
        for (size_t j = 0; j < 17; ++j) {
            char ch = text[i + j];
            if (j % 3 == 2) { if (ch != ':') { ok = false; break; } }
            else if (!std::isxdigit(static_cast<unsigned char>(ch))) { ok = false; break; }
        }
        if (ok) return text.substr(i, 17);
    }
    return "";
}

std::string resolveLocalMac(const std::string& procBase,
                            const std::string& sysBase,
                            const std::string& iface) {
    {
        std::ifstream f(procBase + "/" + iface + "/mac_addr");
        if (f) {
            std::stringstream ss; ss << f.rdbuf();
            auto mac = extractMac(ss.str());
            if (!mac.empty()) return mac;
        }
    }
    {
        std::ifstream f(sysBase + "/" + iface + "/address");
        if (f) {
            std::string line; std::getline(f, line);
            auto mac = extractMac(line);
            if (!mac.empty()) return mac;
        }
    }
    return "";
}

int parseCbrRssi(const std::string& rfinfo) {
    int field = 0;
    size_t start = 0;
    while (field < 3) {                       // skip token, ndp0, ndp1
        size_t colon = rfinfo.find(':', start);
        if (colon == std::string::npos) return 0;
        start = colon + 1;
        ++field;
    }
    size_t end = rfinfo.find(':', start);
    std::string tok = rfinfo.substr(start, end == std::string::npos
                                            ? std::string::npos : end - start);
    try { return std::stoi(tok); } catch (...) { return 0; }
}

BeamformingController::BeamformingController(std::string procBase,
                                             std::string sysBase)
    : procBase_(std::move(procBase)), sysBase_(std::move(sysBase)) {}

BeamformingController::~BeamformingController() { stop(); }

bool BeamformingController::supported(const std::string& iface) const {
    struct stat st{};
    std::string p = procBase_ + "/" + iface + "/bf_monitor_conf";
    return ::stat(p.c_str(), &st) == 0;
}

bool BeamformingController::writeNode(const std::string& iface,
                                      const std::string& node,
                                      const std::string& content) {
    std::ofstream f(procBase_ + "/" + iface + "/" + node);
    if (!f) return false;
    f << content;
    f.flush();
    return static_cast<bool>(f);
}

std::string BeamformingController::readNode(const std::string& iface,
                                            const std::string& node) const {
    std::ifstream f(procBase_ + "/" + iface + "/" + node);
    if (!f) return "";
    std::stringstream ss; ss << f.rdbuf();
    return ss.str();
}

void BeamformingController::reconcile(bool enabled, const BfParams& p, bool force) {
    if (!enabled) {
        stop();
        std::lock_guard<std::mutex> g(mu_);
        status_ = BfStatus{};   // clean Disabled, requested=false
        running_ = false;
        return;
    }

    // Already running with identical params => no-op, UNLESS force (e.g. a radio
    // reset wiped the registers and we must re-write the conf node).
    if (!force) {
        std::lock_guard<std::mutex> g(mu_);
        if (running_ && params_ == p && status_.state == BfState::Active)
            return;
    }
    // Any change (or forced re-arm) while running => restart cleanly.
    stop();

    BfStatus s;
    s.requested = true;
    s.remoteMac = p.remoteMac;
    s.bw = modulationWidth(p.width);
    s.localMac = resolveLocalMac(procBase_, sysBase_, p.iface);

    if (!supported(p.iface)) {
        s.state = BfState::Unsupported;
        s.reason = "no bf_monitor proc node on " + p.iface +
                   " (driver " + p.driver + ")";
        std::lock_guard<std::mutex> g(mu_);
        status_ = s; params_ = p; running_ = false;
        return;
    }

    // Init sequence (synchronous so errors surface immediately).
    bool ok = writeNode(p.iface, "bf_monitor_conf",
                         "1 " + p.remoteMac + " 0 0");
    ok = writeNode(p.iface, "ack_timeout", std::to_string(p.ackTimeout)) && ok;
    if (!ok) {
        s.state = BfState::Error;
        s.reason = "failed to write bf_monitor init nodes";
        std::lock_guard<std::mutex> g(mu_);
        status_ = s; params_ = p; running_ = false;
        return;
    }

    s.state = BfState::Active;
    {
        std::lock_guard<std::mutex> g(mu_);
        status_ = s; params_ = p; running_ = true;
    }
    startLoop();
}

void BeamformingController::startLoop() {
    stopFlag_.store(false);
    thr_ = std::thread([this] { loop(); });
}

void BeamformingController::loop() {
    int token = 0;
    BfParams p;
    { std::lock_guard<std::mutex> g(mu_); p = params_; }
    const int bw = modulationWidth(p.width);

    while (!stopFlag_.load()) {
        std::string trig = resolveLocalMac(procBase_, sysBase_, p.iface);
        trig += " " + p.remoteMac + " 0 0 " + std::to_string(token) +
                " " + std::to_string(bw);
        bool ok = writeNode(p.iface, "bf_monitor_trig", trig);
        if (!ok) {
            // Transient write failure: record it but KEEP the loop alive so it
            // self-heals on the next tick. A single failure must not kill BF.
            std::lock_guard<std::mutex> g(mu_);
            status_.state = BfState::Error;
            status_.reason = "bf_monitor_trig write failed";
        } else {
            token = (token + 1) % 64;
            std::string cbr = readNode(p.iface, "bf_monitor_trig");
            int cbrRssi = parseCbrRssi(readNode(p.iface, "bf_monitor_rfinfo"));
            std::lock_guard<std::mutex> g(mu_);
            status_.soundingCount++;
            status_.cbrRssi = cbrRssi;
            status_.lastCbr = cbr.empty() ? std::nullopt
                                          : std::optional<std::string>(cbr);
            if (status_.state == BfState::Error) {   // recovered from a transient failure
                status_.state = BfState::Active;
                status_.reason.clear();
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(p.intervalMs));
        if (stopFlag_.load()) break;
        if (ok) writeNode(p.iface, "bf_monitor_en", "1");
    }
}

void BeamformingController::stopLoopThread() {
    stopFlag_.store(true);
    if (thr_.joinable()) thr_.join();
}

void BeamformingController::stop() {
    std::string iface;
    bool wasRunning;
    {
        std::lock_guard<std::mutex> g(mu_);
        wasRunning = running_;
        iface = params_.iface;
    }
    stopLoopThread();
    if (wasRunning && !iface.empty()) {
        writeNode(iface, "bf_monitor_conf", "0 00:00:00:00:00:00 0 0");
        writeNode(iface, "ack_timeout", "33");
    }
    std::lock_guard<std::mutex> g(mu_);
    running_ = false;
    if (status_.state == BfState::Active) {
        status_.state = BfState::Disabled;
        status_.requested = false;
    }
}

BfStatus BeamformingController::status() const {
    std::lock_guard<std::mutex> g(mu_);
    return status_;
}

} // namespace fpvd
