#pragma once
#include <atomic>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace fpvd {

struct BfParams {
    std::string iface;     // e.g. "wlan0"
    std::string driver;    // RadioResult.driver, for the unsupported reason text
    std::string remoteMac; // ground-station MAC
    int width{20};         // link.width; controller derives modulation bw
    int ackTimeout{255};
    int intervalMs{100};

    bool operator==(const BfParams& o) const {
        return iface == o.iface && driver == o.driver && remoteMac == o.remoteMac &&
               width == o.width && ackTimeout == o.ackTimeout && intervalMs == o.intervalMs;
    }
    bool operator!=(const BfParams& o) const { return !(*this == o); }
};

enum class BfState { Disabled, Unsupported, Active, Error };

struct BfStatus {
    bool requested{false};
    BfState state{BfState::Disabled};
    std::string reason;
    std::string localMac; // resolved drone reference for the GS
    std::string remoteMac;
    int bw{0};
    long soundingCount{0};
    int cbrRssi{0};       // cbr_rssi0 from bf_monitor_rfinfo; 0 = no report
    bool cbrFresh{false}; // rfinfo token advancing = receiving fresh reports
    std::optional<std::string> lastCbr;
};

// Resolve the chip's MAC: parse the first MAC-like token from
// <procBase>/<iface>/mac_addr; fall back to <sysBase>/<iface>/address.
// Returns "" if neither is readable.
std::string resolveLocalMac(const std::string& procBase, const std::string& sysBase,
                            const std::string& iface);

// Parse the 4th colon-field (cbr_rssi0, dBm) of a bf_monitor_rfinfo line.
// Returns 0 on empty/malformed input (0 == no report received).
int parseCbrRssi(const std::string& rfinfo);

// Parse the 1st colon-field (sounding token) of a bf_monitor_rfinfo line.
// Returns -1 on empty/malformed input. The token advances only on a new CBR.
int parseCbrToken(const std::string& rfinfo);

class BeamformingController {
  public:
    explicit BeamformingController(std::string procBase = "/proc/net/rtl88x2eu",
                                   std::string sysBase = "/sys/class/net");
    ~BeamformingController();

    // Idempotent: starts, stops, or restarts the sounding loop to match the
    // desired (enabled, params) state.
    void reconcile(bool enabled, const BfParams& p, bool force = false);
    void stop(); // stop loop + reset driver state
    BfStatus status() const;

    // status(), but with localMac resolved from `iface` when the beamformer
    // isn't armed (status_.localMac is only set on reconcile). The GS armer
    // reads the drone's localMac to arm its beamformee, so it must be reportable
    // while BF is disabled. Never overrides an already-resolved armed MAC.
    BfStatus statusWithPrimary(const std::string& iface) const;

  private:
    bool supported(const std::string& iface) const; // bf_monitor_conf exists?
    bool writeNode(const std::string& iface, const std::string& node,
                   const std::string& content); // returns false on error
    std::string readNode(const std::string& iface, const std::string& node) const;
    void startLoop();
    void loop();
    void stopLoopThread(); // join without driver reset

    std::string procBase_;
    std::string sysBase_;

    mutable std::mutex mu_; // guards status_ + params_
    BfStatus status_;
    BfParams params_;
    bool running_{false};

    std::thread thr_;
    std::atomic<bool> stopFlag_{false};
};

} // namespace fpvd
