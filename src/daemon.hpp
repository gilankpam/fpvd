#pragma once
#include "config/diff.hpp"
#include "config/schema.hpp"
#include "config/validate.hpp"
#include "dynlink/controller.hpp"
#include "dynlink/runtime_config.hpp"
#include "waybeam/client.hpp"
#include "supervise/orchestrator.hpp"
#include "supervise/beamforming.hpp"
#include <chrono>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace fpvd {

struct DaemonPaths {
    std::string defaultsPath;    // /rom/etc/fpvd/defaults.json
    std::string overlayPath;     // /etc/fpvd/config.json
    std::string radioUpScript;   // /usr/libexec/fpvd/radio-up.sh
    std::string waybeamJsonPath; // /etc/waybeam.json
    std::string radioTuneScript{}; // /usr/libexec/fpvd/radio-tune.sh (optional)
    dynlink::Endpoints dlEndpoints{};  // defaults to production endpoints; overridable in tests
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
    BfStatus beamformingStatus() const { return bf_.status(); }
    dynlink::DlStatus dynamicLinkStatus() const { return dl_.status(); }
    std::chrono::steady_clock::time_point startedAt() const { return startedAt_; }

    PatchResult patchPending(const nlohmann::json& patch);
    ApplyResult apply(bool reallyRestart);
    void reset();

    Orchestrator& orchestrator() { return orch_; }
    nlohmann::json defaultsJson();   // returns parsed defaults file

private:
    void seedOrchestrator();
    void reconcileBeamforming();
    void rewriteWaybeamJson();
    void startController();
    // Rebuild the msposd OSD spec from effective_ and (re)start it, so it picks
    // up the new canvas size (-z resolution) after a waybeam restart. A plain
    // orch_.restart would relaunch msposd with the stale spec argv. No-op unless
    // msposd is the supervised telemetry router.
    void restartOsd();

    DaemonPaths paths_;
    Config effective_;
    Config pending_;
    int version_{0};
    LastApply lastApply_;
    RadioInfo radio_;
    WaybeamClient waybeam_;   // declared before dl_/orch_ for init order
    Orchestrator orch_;
    BeamformingController bf_;
    dynlink::DynamicLinkController dl_;
    uint32_t dlGenerationId_;
    std::chrono::steady_clock::time_point startedAt_;
    std::mutex mu_;
};

} // namespace fpvd
