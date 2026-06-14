#pragma once
#include "config/diff.hpp"
#include "config/schema.hpp"
#include "config/validate.hpp"
#include "dynlink/controller.hpp"
#include "dynlink/runtime_config.hpp"
#include "idr/idr_constants.hpp"
#include "idr/relay.hpp"
#include "osd/osd_constants.hpp"
#include "osd/writer.hpp"
#include "waybeam/client.hpp"
#include "supervise/orchestrator.hpp"
#include "supervise/beamforming.hpp"
#include <atomic>
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

namespace fpvd {

struct DaemonPaths {
    std::string configPath;      // /etc/fpvd/config.json (the full config)
    std::string radioUpScript;   // /usr/libexec/fpvd/radio-up.sh
    std::string waybeamJsonPath; // /etc/waybeam.json
    std::string radioTuneScript{}; // /usr/libexec/fpvd/radio-tune.sh (optional)
    dynlink::Endpoints dlEndpoints{};  // defaults to production endpoints; overridable in tests
    // UDP port for the always-on IDR relay (GS tunnel -> drone). Not operator
    // config — a fixed transport constant; this field exists only so tests can
    // pick an ephemeral port or disable it (0). Production uses idr::kIdrPort.
    int idrPort{idr::kIdrPort};
    // OSD message-file path. Not operator config; this field exists only so
    // tests can redirect it to a temp file. Production uses osd::kOsdMsgPath.
    std::string osdMsgPath{osd::kOsdMsgPath};
    // Settle delay for the waybeam-only restart (see Orchestrator::restart):
    // gives the SigmaStar driver time to drain the old pipeline before the fresh
    // waybeam re-inits, so a video0.size change doesn't wedge the VENC channel.
    // 700 ms > waybeam's own 500 ms margin. Tests set this to 0 to stay fast.
    int waybeamRestartSettleMs{700};
};

struct PatchResult {
    bool ok{true};
    std::vector<ValidationError> errors;
    std::vector<std::string> lockedPaths;  // non-empty => 400 dynamic_link_locked
};

struct ApplyResult {
    bool ok{true};
    std::vector<ValidationError> errors;
    std::vector<std::string> restarted;
    std::optional<std::string> radioError;
    int version{0};
};

struct LastApply {
    std::string at;                  // ISO8601 timestamp
    bool ok{false};
    std::vector<std::string> restarted;
    std::optional<std::string> error;
};

struct RadioInfo {
    std::string driver;
    std::string iface;
    std::optional<std::string> adapterId;
};

class Daemon {
public:
    explicit Daemon(DaemonPaths paths);
    ~Daemon();

    Daemon(const Daemon&) = delete;
    Daemon& operator=(const Daemon&) = delete;

    // Load defaults and overlay, write initial /etc/waybeam.json,
    // configure orchestrator, optionally start processes.
    void bootstrap(bool startProcesses);

    const Config& effective() const { return effective_; }
    const Config& pending() const { return pending_; }
    int version() const { return version_; }
    const LastApply& lastApply() const { return lastApply_; }
    const RadioInfo& radio() const { return radio_; }
    BfStatus beamformingStatus() const {
        return bf_.statusWithPrimary(radio_.iface.empty() ? "wlan0" : radio_.iface);
    }
    dynlink::DlStatus dynamicLinkStatus() const { return dl_.status(); }
    std::chrono::steady_clock::time_point startedAt() const { return startedAt_; }

    PatchResult patchPending(const nlohmann::json& patch);
    ApplyResult apply(bool reallyRestart);
    void reset();

    Orchestrator& orchestrator() { return orch_; }
    nlohmann::json defaultsJson();   // returns the code-default config (Config{})

private:
    void seedOrchestrator();
    // Targeted add/remove of the observe-only probe pair (probe-tx + probe-feed)
    // on the live dynamicLink on<->off transition — never bounces wfb/video.
    // add() registers the spec; restart() starts the not-yet-running process.
    void addProbeStream();
    void removeProbeStream();
    void reconcileBeamforming(bool force = false);
    // 0 = BF off/disabled, 1 = armed but no report, 2 = working (cbr_rssi != 0).
    int bfOsdCode() const;
    void rewriteWaybeamJson();
    void startController();
    // Rebuild the msposd OSD spec from effective_ and (re)start it, so it picks
    // up the new canvas size (-z resolution) after a waybeam restart. A plain
    // orch_.restart would relaunch msposd with the stale spec argv. No-op unless
    // msposd is the supervised telemetry router.
    void restartOsd();
    // Push the static configured radio (mcs/fec/txpower/bandwidth) + encoder
    // (bitrate/roiQp) values back after dynamic-link is disabled, so the link
    // reverts to its pre-DL state instead of staying at the controller's last
    // adaptive values. Best-effort.
    void restateStaticLink();
    // Write the system-stats OSD line to msposd's message file when dynamic-link
    // isn't feeding the OSD (DL off). msposd holds + re-renders it with live
    // placeholder values. No-op unless the telemetry router is msposd.
    void writeOsdBaseLine();
    // Background loop that re-writes the base OSD line every ~1s while DL is off,
    // so it survives the window where a resolution change restarts waybeam +
    // msposd (the one-shot write is consumed before the fresh msposd can render).
    void osdHeartbeat();

    DaemonPaths paths_;
    Config effective_;
    Config pending_;
    int version_{0};
    LastApply lastApply_;
    RadioInfo radio_;
    WaybeamClient waybeam_;   // declared before dl_/orch_/idrRelay_ for init order
    // Always-on IDR keyframe relay: shares waybeam_ (thread-safe), runs whether
    // dynamicLink is enabled or not. Declared after waybeam_ so it outlives it.
    idr::IdrRelay idrRelay_;
    // Always-on OSD writer: the single owner of the msposd message file. Both
    // the controller (status line, injected via dl_.setOsdWriter) and the daemon
    // (base line) write through it. Declared before dl_ so it outlives the
    // controller's pointer to it.
    osd::OsdWriter osd_;
    Orchestrator orch_;
    BeamformingController bf_;
    dynlink::DynamicLinkController dl_;
    std::chrono::steady_clock::time_point startedAt_;
    std::mutex mu_;
    std::thread osdThread_;
    std::atomic<bool> osdStop_{false};
};

} // namespace fpvd
