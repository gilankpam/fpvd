#pragma once
#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace fpvd {

struct BfParams {
    std::string iface;      // e.g. "wlan0"
    std::string driver;     // RadioResult.driver, for the unsupported reason text
    std::string remoteMac;  // ground-station MAC
    int width{20};          // link.width; controller derives modulation bw
    int ackTimeout{255};
    int intervalMs{100};

    bool operator==(const BfParams& o) const {
        return iface == o.iface && driver == o.driver &&
               remoteMac == o.remoteMac && width == o.width &&
               ackTimeout == o.ackTimeout && intervalMs == o.intervalMs;
    }
    bool operator!=(const BfParams& o) const { return !(*this == o); }
};

enum class BfState { Disabled, Unsupported, Active, Error };

struct BfStatus {
    bool requested{false};
    BfState state{BfState::Disabled};
    std::string reason;
    std::string localMac;   // resolved drone reference for the GS
    std::string remoteMac;
    int bw{0};
    long soundingCount{0};
    std::optional<std::string> lastCbr;
};

// Resolve the chip's MAC: parse the first MAC-like token from
// <procBase>/<iface>/mac_addr; fall back to <sysBase>/<iface>/address.
// Returns "" if neither is readable.
std::string resolveLocalMac(const std::string& procBase,
                            const std::string& sysBase,
                            const std::string& iface);

class BeamformingController {
public:
    explicit BeamformingController(
        std::string procBase = "/proc/net/rtl88x2eu",
        std::string sysBase  = "/sys/class/net");
    ~BeamformingController();

    // Idempotent: starts, stops, or restarts the sounding loop to match the
    // desired (enabled, params) state.
    void reconcile(bool enabled, const BfParams& p, bool force = false);
    void stop();                  // stop loop + reset driver state
    BfStatus status() const;

private:
    bool supported(const std::string& iface) const;  // bf_monitor_conf exists?
    bool writeNode(const std::string& iface, const std::string& node,
                   const std::string& content);       // returns false on error
    std::string readNode(const std::string& iface, const std::string& node) const;
    void startLoop();
    void loop();
    void stopLoopThread();        // join without driver reset

    std::string procBase_;
    std::string sysBase_;

    mutable std::mutex mu_;       // guards status_ + params_
    BfStatus status_;
    BfParams params_;
    bool running_{false};

    std::thread thr_;
    std::atomic<bool> stopFlag_{false};
};

} // namespace fpvd
