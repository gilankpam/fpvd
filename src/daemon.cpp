#include "daemon.hpp"
#include "config/lock.hpp"
#include "config/store.hpp"
#include "config/validate.hpp"
#include "supervise/radio.hpp"
#include "translate/dynamic_link.hpp"
#include "translate/telemetry.hpp"
#include "translate/waybeam.hpp"
#include "translate/wfb.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <thread>

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

    // Telemetry router process name (empty when router is "none" or unknown).
    std::string telemetryName;
    if (effective_.telemetry.router == "mavfwd") telemetryName = "mavfwd";
    else if (effective_.telemetry.router == "msposd") telemetryName = "msposd";

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
    if (!tArgs.empty() && !telemetryName.empty()) {
        orch_.add(wfbSpec(telemetryName, std::move(tArgs),
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
    if (effective_.dynamicLink.enabled) {
        SupervisedSpec dl{};
        dl.name = "dl_applier";
        dl.argv = dynamicLinkArgs(effective_, iface);
        dl.restart = RestartPolicy::Always;
        dl.startAfter = {"wfb_video_tx", "wfb_tun", "waybeam"};
        if (!telemetryName.empty()) dl.startAfter.push_back(telemetryName);
        orch_.add(std::move(dl));
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
    const bool wasDlEnabled = effective_.dynamicLink.enabled;
    // Channel/bandwidth retune drops the over-the-air link (and the
    // wfb_tun tunnel carrying this HTTP session) until the client also
    // retunes — defer the restart so the response can flush first.
    // Other link changes (mcs/txpower/fec/...) don't drop the air link.
    const bool deferRadioRetune =
        effective_.link.channel != pending_.link.channel ||
        effective_.link.width   != pending_.link.width;

    // Persist overlay (sparse diff vs defaults).
    auto defaultsJ = defaultsJson();
    auto pendingJ = nlohmann::json(pending_);
    auto overlay = computeOverlay(defaultsJ, pendingJ);
    atomicWriteJson(paths_.overlayPath, overlay);

    effective_ = pending_;
    rewriteWaybeamJson();

    std::vector<std::string> restarted;
    if (subs.radio) restarted.push_back("radio");
    if (subs.encoder) restarted.push_back("encoder");
    if (subs.telemetry) restarted.push_back("telemetry");
    // Only report dl_applier as restarted when the process existed before
    // the apply or will exist after; pure baseline edits while DL is off
    // don't restart anything.
    if (subs.dynamicLink && (wasDlEnabled || effective_.dynamicLink.enabled)) {
        restarted.push_back("dl_applier");
    }
    for (auto& n : subs.servicesAffected) restarted.push_back(n);

    if (reallyRestart && deferRadioRetune) {
        // Defer the restart so the HTTP response can flush before the
        // channel/bandwidth change drops the client's wfb_tun session.
        // lastApply_.ok is set to true optimistically; the deferred
        // worker flips it to false (with .error) if radio bring-up fails.
        version_++;
        lastApply_ = {nowIso(), true, restarted, std::nullopt};
        std::thread([this]{
            std::this_thread::sleep_for(std::chrono::milliseconds(200));
            std::lock_guard<std::mutex> g2(mu_);
            orch_.stopAll();
            orch_ = Orchestrator{};
            auto rr = bringUpRadio(paths_.radioUpScript, effective_);
            if (!rr.ok) {
                lastApply_.ok = false;
                lastApply_.error = std::string("radio: ") + rr.stderrText;
                return;
            }
            radio_ = {rr.driver, rr.iface, rr.adapterId};
            seedOrchestrator();
            orch_.startAll();
        }).detach();
        return {true, {}, restarted, std::nullopt, version_};
    }

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
    } else {
        // Re-seed the orchestrator's specs so introspection
        // (Orchestrator::names()) reflects the new config. Note: assigning
        // a fresh Orchestrator here destroys any existing Supervisor
        // unique_ptrs, which would also shutdown() their children — this
        // is safe today only because every caller pairs bootstrap(false)
        // with apply(false), so no live children exist.
        orch_ = Orchestrator{};
        seedOrchestrator();
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
