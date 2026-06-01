#pragma once
#include "config/diff.hpp"
#include "config/schema.hpp"
#include "config/validate.hpp"
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

    // Load defaults and overlay, write initial /etc/waybeam.json,
    // configure orchestrator, optionally start processes.
    void bootstrap(bool startProcesses);

    const Config& effective() const { return effective_; }
    const Config& pending() const { return pending_; }
    int version() const { return version_; }
    const LastApply& lastApply() const { return lastApply_; }
    const RadioInfo& radio() const { return radio_; }
    BfStatus beamformingStatus() const { return bf_.status(); }
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

    DaemonPaths paths_;
    Config effective_;
    Config pending_;
    int version_{0};
    LastApply lastApply_;
    RadioInfo radio_;
    Orchestrator orch_;
    BeamformingController bf_;
    std::chrono::steady_clock::time_point startedAt_;
    std::mutex mu_;
};

} // namespace fpvd
