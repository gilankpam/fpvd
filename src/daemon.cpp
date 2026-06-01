#include "daemon.hpp"
#include "config/lock.hpp"
#include "config/store.hpp"
#include "config/validate.hpp"
#include "supervise/radio.hpp"
#include "translate/telemetry.hpp"
#include "translate/waybeam.hpp"
#include "translate/wfb.hpp"
#include "translate/wfb_control.hpp"
#include "link_width.hpp"
#include <chrono>
#include <ctime>
#include <fstream>
#include <filesystem>
#include <random>
#include <thread>

namespace fpvd {

static std::string nowIso() {
    auto t = std::time(nullptr);
    char buf[32];
    std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", std::gmtime(&t));
    return buf;
}

Daemon::Daemon(DaemonPaths paths)
    : paths_(std::move(paths)),
      dl_(paths_.dlEndpoints),
      dlGenerationId_(std::random_device{}()),
      startedAt_(std::chrono::steady_clock::now()) {
}

Daemon::~Daemon() {
    // Stop the in-process control loop (joins its thread) before any member is destroyed.
    dl_.stop();
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
        reconcileBeamforming();
        if (effective_.dynamicLink.enabled) {
            dl_.start(dynlink::buildDlSnapshot(effective_, radio_.iface), dlGenerationId_);
        }
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
}


void Daemon::reconcileBeamforming() {
    const auto& bfc = effective_.link.beamforming;
    BfParams p;
    p.iface      = radio_.iface.empty() ? "wlan0" : radio_.iface;
    p.driver     = radio_.driver;
    p.remoteMac  = bfc.remoteMac;
    p.width      = effective_.link.width;
    p.ackTimeout = bfc.ackTimeout;
    p.intervalMs = bfc.intervalMs;
    bf_.reconcile(bfc.enabled, p);
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

    auto subs = diffSubsystems(effective_, pending_);
    auto link = classifyLinkChange(effective_, pending_);
    const bool enabledOld = effective_.dynamicLink.enabled;

    // Beamforming is reconciled (not exec-supervised); report it as restarted
    // when its own block or the derived modulation width changed.
    const bool bfChanged =
        nlohmann::json(effective_.link.beamforming) !=
            nlohmann::json(pending_.link.beamforming) ||
        effective_.link.width != pending_.link.width;

    // Persist overlay (sparse diff vs defaults).
    auto defaultsJ = defaultsJson();
    auto pendingJ = nlohmann::json(pending_);
    auto overlay = computeOverlay(defaultsJ, pendingJ);
    atomicWriteJson(paths_.overlayPath, overlay);

    effective_ = pending_;
    rewriteWaybeamJson();
    const bool enabledNew = effective_.dynamicLink.enabled;

    std::vector<std::string> restarted;
    if (subs.radio) restarted.push_back("radio");
    if (subs.encoder) restarted.push_back("encoder");
    if (subs.telemetry) restarted.push_back("telemetry");
    // The in-process DynamicLinkController is hot-reloaded (start/stop/setConfig)
    // — never bounced with wfb. Report it as "dynamicLink" when its config moves
    // while it is (or becomes) active, or when it is being toggled on/off.
    const bool dlAffects =
        subs.dynamicLink && (enabledOld || enabledNew);
    if (dlAffects) restarted.push_back("dynamicLink");
    for (auto& n : subs.servicesAffected) restarted.push_back(n);
    if (bfChanged) restarted.push_back("beamforming");

    // A rebuild bounces the whole orchestrator (including wfb). It is needed
    // only when a non-link subsystem changes, or when a link change cannot be
    // hot-applied (linkId / wlanAdapter). A dynamicLink-only change (or an
    // mtu-only change consumed by the controller) is NOT a rebuild: it hot-
    // reloads the controller. A video.fps change still rebuilds via subs.encoder
    // (the restart-around re-snapshots the controller after startAll).
    const bool needsRebuild = subs.encoder || subs.telemetry ||
        !subs.servicesAffected.empty() || link.fullRestart;

    if (reallyRestart && needsRebuild) {
        // Full-restart path: rebuild orchestrator + radio bring-up. The in-process
        // controller is stopped first (clean stop while wfb is still up) and
        // restarted after startAll() — a "restart-around" so a rebuild for a
        // non-DL reason (e.g. encoder/fps) does not leave the controller dead.
        if (enabledOld) dl_.stop();
        orch_.stopAll();
        orch_ = Orchestrator{};
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
        reconcileBeamforming();
        // Start AFTER radio_ is refreshed so the snapshot's iface is fresh.
        if (enabledNew)
            dl_.start(dynlink::buildDlSnapshot(effective_, radio_.iface),
                      dlGenerationId_);
        version_++;
        lastApply_ = {nowIso(), true, restarted, std::nullopt};
        return {true, {}, restarted, std::nullopt, version_};
    }

    if (reallyRestart) {
        // Hot path: no wfb restart. Route the in-process controller FIRST, so it
        // runs regardless of which hot return is taken below (the deferred
        // nicChannel return detaches a worker and returns early). start() binds
        // sockets + launches the thread; setConfig() hot-reloads; stop() joins.
        if (!enabledOld && enabledNew)
            dl_.start(dynlink::buildDlSnapshot(effective_, radio_.iface),
                      dlGenerationId_);
        else if (enabledOld && !enabledNew)
            dl_.stop();
        else if (enabledOld && enabledNew && subs.dynamicLink)
            dl_.setConfig(dynlink::buildDlSnapshot(effective_, radio_.iface));

        // A purely hot-applicable link change — no wfb restart.
        // (A) Immediate, non-link-dropping changes.
        if (link.nicTxpower) {
            auto rr = tuneRadio(paths_.radioTuneScript, "txpower", effective_,
                                radio_.iface, radio_.driver);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("txpower: ") + rr.stderrText};
                return {false, {}, restarted, rr.stderrText, version_};
            }
        }
        if (link.nicMtu) {
            auto rr = tuneRadio(paths_.radioTuneScript, "mtu", effective_,
                                radio_.iface, radio_.driver);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("mtu: ") + rr.stderrText};
                return {false, {}, restarted, rr.stderrText, version_};
            }
        }
        if (link.videoFec) {
            WfbControlClient cli("127.0.0.1", kVideoControlPort);
            auto rr = cli.setFec(static_cast<uint8_t>(effective_.link.fec.k),
                                 static_cast<uint8_t>(effective_.link.fec.n));
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("fec: ") + rr.error};
                return {false, {}, restarted, rr.error, version_};
            }
        }
        if (link.videoRadiotap && !link.nicWidth) {
            // mcs/stbc/ldpc with no width change — push now (no link drop).
            WfbControlClient cli("127.0.0.1", kVideoControlPort);
            auto rr = cli.setRadio(
                static_cast<uint8_t>(effective_.link.stbc ? 1 : 0),
                effective_.link.ldpc, false,
                static_cast<uint8_t>(modulationWidth(effective_.link.width)),
                static_cast<uint8_t>(effective_.link.mcs), false, 1);
            if (!rr.ok) {
                lastApply_ = {nowIso(), false, restarted,
                              std::string("radio: ") + rr.error};
                return {false, {}, restarted, rr.error, version_};
            }
        }

        // (B) Link-dropping change (channel and/or width) — defer ~200ms so the
        // HTTP response flushes before the air link (and wfb_tun session) drops.
        if (link.nicChannel) {
            version_++;
            lastApply_ = {nowIso(), true, restarted, std::nullopt};
            const bool pushWidth = link.nicWidth;
            // restarted is already recorded in lastApply_ above; the worker
            // only flips ok/error, so it need not capture it.
            std::thread([this, pushWidth] {
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                std::lock_guard<std::mutex> g2(mu_);
                auto rr = tuneRadio(paths_.radioTuneScript, "channel", effective_,
                                    radio_.iface, radio_.driver);
                if (!rr.ok) {
                    lastApply_.ok = false;
                    lastApply_.error = std::string("channel: ") + rr.stderrText;
                    return;
                }
                if (pushWidth) {
                    // NIC retuned first; now bump the video radiotap bandwidth.
                    WfbControlClient cli("127.0.0.1", kVideoControlPort);
                    auto cr = cli.setRadio(
                        static_cast<uint8_t>(effective_.link.stbc ? 1 : 0),
                        effective_.link.ldpc, false,
                        static_cast<uint8_t>(modulationWidth(effective_.link.width)),
                        static_cast<uint8_t>(effective_.link.mcs), false, 1);
                    if (!cr.ok) {
                        lastApply_.ok = false;
                        lastApply_.error = std::string("radio: ") + cr.error;
                    }
                }
            }).detach();
            return {true, {}, restarted, std::nullopt, version_};
        }

        version_++;
        lastApply_ = {nowIso(), true, restarted, std::nullopt};
        return {true, {}, restarted, std::nullopt, version_};
    }

    // reallyRestart == false: re-seed orchestrator specs only (dry config load).
    orch_ = Orchestrator{};
    seedOrchestrator();
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
