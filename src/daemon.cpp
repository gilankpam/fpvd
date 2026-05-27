#include "daemon.hpp"
#include "config/lock.hpp"
#include "config/store.hpp"
#include "config/validate.hpp"
#include "supervise/radio.hpp"
#include "translate/telemetry.hpp"
#include "translate/waybeam.hpp"
#include "translate/wfb.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <filesystem>

namespace fpvd {

static std::string nowIso() {
    auto t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
    return buf;
}

Daemon::Daemon(DaemonPaths paths) : paths_(std::move(paths)) {
    startedAt_ = std::chrono::steady_clock::now();
}

nlohmann::json Daemon::defaultsJson() {
    std::ifstream f(paths_.defaultsPath);
    return nlohmann::json::parse(f);
}

void Daemon::bootstrap(bool startProcesses) {
    effective_ = loadEffective(paths_.defaultsPath, paths_.overlayPath);
    pending_ = effective_;
    auto errs = validate(effective_);
    if (!errs.empty()) {
        throw StoreError("invalid configuration on bootstrap");
    }
    rewriteWaybeamJson();
    if (startProcesses) {
        auto rr = bringUpRadio(paths_.radioUpScript, effective_);
        if (!rr.ok) {
            lastApply_ = {nowIso(), false, {}, std::string{"radio: "} + rr.stderrText};
            return;
        }
        radio_ = {rr.driver, rr.iface, rr.adapterId};
        seedOrchestrator();
        orch_.startAll();
    }
}

static SupervisedSpec wfbSpec(const std::string& name, std::vector<std::string> argv,
                              std::vector<std::string> after) {
    SupervisedSpec s{};
    s.name = name; s.argv = std::move(argv);
    s.restart = RestartPolicy::Always; s.startAfter = std::move(after);
    return s;
}

void Daemon::seedOrchestrator() {
    const std::string iface = radio_.iface.empty() ? "wlan0" : radio_.iface;
    const std::string key = "/etc/drone.key";
    orch_.add(wfbSpec("wfb_video_tx",
              wfbArgs(effective_, WfbRole::VideoTx, iface, key), {}));
    orch_.add(wfbSpec("wfb_tun_rx",
              wfbArgs(effective_, WfbRole::TunRx, iface, key), {}));
    orch_.add(wfbSpec("wfb_tun_tx",
              wfbArgs(effective_, WfbRole::TunTx, iface, key), {}));
    orch_.add(wfbSpec("wfb_tun", wfbTunArgs(), {"wfb_tun_rx", "wfb_tun_tx"}));
    orch_.add(wfbSpec("wfb_tlm_rx",
              wfbArgs(effective_, WfbRole::TlmRx, iface, key), {}));
    orch_.add(wfbSpec("wfb_tlm_tx",
              wfbArgs(effective_, WfbRole::TlmTx, iface, key), {}));
    orch_.add(wfbSpec("waybeam", {"/usr/bin/waybeam"}, {"wfb_video_tx"}));
    auto tArgs = telemetryArgs(effective_);
    if (!tArgs.empty()) {
        std::string name = (effective_.telemetry.router == "mavfwd")
                              ? "mavfwd" : "msposd";
        orch_.add(wfbSpec(name, std::move(tArgs),
                          {"wfb_tlm_rx", "wfb_tlm_tx"}));
    }
    for (auto& [n, s] : effective_.services) {
        if (!s.enabled) continue;
        SupervisedSpec spec{};
        spec.name = n;
        spec.argv = {s.exec};
        for (auto& a : s.args) spec.argv.push_back(a);
        spec.env = s.env;
        spec.startAfter = s.startAfter;
        spec.restart = (s.restart == "always") ? RestartPolicy::Always
                     : (s.restart == "on-failure") ? RestartPolicy::OnFailure
                     : RestartPolicy::Never;
        orch_.add(std::move(spec));
    }
}

void Daemon::rewriteWaybeamJson() {
    atomicWriteJson(paths_.waybeamJsonPath, toWaybeamJson(effective_));
}

PatchResult Daemon::patchPending(const nlohmann::json& patch) {
    std::lock_guard<std::mutex> g(mu_);
    nlohmann::json next = deepMergeJson(nlohmann::json(pending_), patch);
    Config candidate;
    try { candidate = next.get<Config>(); }
    catch (const nlohmann::json::exception& e) {
        return {false, {{"<root>", e.what()}}, {}};
    }
    auto lockR = checkDynamicLinkLock(patch, candidate);
    if (!lockR.ok) {
        return {false, {}, std::move(lockR.lockedPaths)};
    }
    auto errs = validate(candidate);
    if (!errs.empty()) return {false, std::move(errs), {}};
    pending_ = candidate;
    return {true, {}, {}};
}

ApplyResult Daemon::apply(bool reallyRestart) {
    std::lock_guard<std::mutex> g(mu_);
    auto errs = validate(pending_);
    if (!errs.empty()) return {false, std::move(errs), {}, std::nullopt, version_};

    // Compute diff before overwriting effective.
    auto subs = diffSubsystems(effective_, pending_);

    // Persist overlay (sparse diff vs defaults).
    auto defaultsJ = defaultsJson();
    auto pendingJ = nlohmann::json(pending_);
    auto overlay = computeOverlay(defaultsJ, pendingJ);
    atomicWriteJson(paths_.overlayPath, overlay);

    effective_ = pending_;
    rewriteWaybeamJson();

    std::vector<std::string> restarted;
    if (reallyRestart) {
        // Subsystem-level restart: rebuild orchestrator (simple v1).
        orch_.stopAll();
        orch_ = Orchestrator{};
        // Re-run radio bring-up if link changed.
        if (subs.radio) {
            auto rr = bringUpRadio(paths_.radioUpScript, effective_);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, {},
                              std::string("radio: ") + rr.stderrText};
                return {false, {}, {}, rr.stderrText, version_};
            }
            radio_ = {rr.driver, rr.iface, rr.adapterId};
        }
        seedOrchestrator();
        orch_.startAll();
        if (subs.radio) restarted.push_back("radio");
        if (subs.encoder) restarted.push_back("encoder");
        if (subs.telemetry) restarted.push_back("telemetry");
        for (auto& n : subs.servicesAffected) restarted.push_back(n);
    }
    version_++;
    lastApply_ = {nowIso(), true, restarted, std::nullopt};
    return {true, {}, restarted, std::nullopt, version_};
}

void Daemon::reset() {
    std::lock_guard<std::mutex> g(mu_);
    std::error_code ec;
    std::filesystem::remove(paths_.overlayPath, ec);
    pending_ = loadEffective(paths_.defaultsPath, "/no/such/path");
}

} // namespace fpvd
