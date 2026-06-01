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
      waybeam_(paths_.dlEndpoints.encHost, paths_.dlEndpoints.encPort),
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
            startController();
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

void Daemon::startController() {
    // Builds a fresh snapshot from the current effective_ config + detected iface,
    // and starts the in-process control loop.
    dl_.start(dynlink::buildDlSnapshot(effective_, radio_.iface), dlGenerationId_);
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
    // enabledOld from effective_ (pre-commit), enabledNew from pending_ (the about-to-be-committed config).
    const bool enabledOld = effective_.dynamicLink.enabled;
    const bool enabledNew = pending_.dynamicLink.enabled;

    // Beamforming is reconciled (not exec-supervised); report it as restarted
    // when its own block or the derived modulation width changed.
    const bool bfChanged =
        nlohmann::json(effective_.link.beamforming) !=
            nlohmann::json(pending_.link.beamforming) ||
        effective_.link.width != pending_.link.width;

    // Encoder reconcile (computed from the pre-commit diff). codec is excluded;
    // dynamic-link-owned fields are excluded while DL is enabled.
    auto wbDiff = waybeamConfigDiff(effective_, pending_, enabledNew);
    const bool encRestart = !wbDiff.restart.empty();   // any restart field => restart
    const bool encLive    = !encRestart && !wbDiff.live.empty();
    const bool encChanged = encRestart || encLive;

    // A full orchestrator rebuild is needed only for non-encoder subsystems.
    const bool needsRebuild = subs.telemetry ||
        !subs.servicesAffected.empty() || link.fullRestart;

    // Transactional LIVE push: apply before committing so a failed push fails the
    // apply with nothing changed and the radio link untouched. Skipped under a
    // full rebuild (it restarts waybeam + reloads the file) and on the dry path.
    if (reallyRestart && !needsRebuild && encLive) {
        if (!waybeam_.setFields(wbDiff.live)) {
            lastApply_ = {nowIso(), false, {},
                          std::string("waybeam: /api/v1/set failed")};
            return {false, {}, {}, std::string("waybeam: /api/v1/set failed"),
                    version_};
        }
    }

    // Persist overlay (sparse diff vs defaults).
    auto defaultsJ = defaultsJson();
    auto pendingJ = nlohmann::json(pending_);
    auto overlay = computeOverlay(defaultsJ, pendingJ);
    atomicWriteJson(paths_.overlayPath, overlay);

    effective_ = pending_;
    rewriteWaybeamJson();

    std::vector<std::string> restarted;
    if (subs.radio) restarted.push_back("radio");
    if (encChanged) restarted.push_back("encoder");
    if (subs.telemetry) restarted.push_back("telemetry");
    // The in-process DynamicLinkController is hot-reloaded (start/stop/setConfig)
    // — never bounced with wfb. Report it as "dynamicLink" when its config moves
    // while it is (or becomes) active, or when it is being toggled on/off.
    const bool dlAffects =
        subs.dynamicLink && (enabledOld || enabledNew);
    if (dlAffects) restarted.push_back("dynamicLink");
    for (auto& n : subs.servicesAffected) restarted.push_back(n);
    if (bfChanged) restarted.push_back("beamforming");

    // A rebuild bounces the whole orchestrator (including wfb). It is needed only
    // when a non-link subsystem changes (telemetry/services), or when a link
    // change cannot be hot-applied (linkId / wlanAdapter). A dynamicLink-only
    // change (or an mtu-only change consumed by the controller) is NOT a rebuild:
    // it hot-reloads the controller. Encoder changes are NOT rebuilds either —
    // LIVE fields are pushed to waybeam (/api/v1/set) and restart-class fields
    // bounce only waybeam (see encRestart). A video.fps change is a LIVE /set
    // when DL is off, and is routed through dl_.setConfig() when DL is on (fps is
    // excluded from waybeamConfigDiff there).

    if (reallyRestart && needsRebuild) {
        // Full-restart path: rebuild orchestrator + radio bring-up. The in-process
        // controller is stopped first (clean stop while wfb is still up) and
        // restarted after startAll() — a "restart-around" so a rebuild for a
        // non-DL reason (e.g. encoder/fps) does not leave the controller dead.
        // On radio bring-up failure below, the controller stays stopped (safe state:
        // no wfb, no decisions); the operator retries POST /apply.
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
            startController();
        version_++;
        lastApply_ = {nowIso(), true, restarted, std::nullopt};
        return {true, {}, restarted, std::nullopt, version_};
    }

    if (reallyRestart) {
        // Encoder restart-class change: waybeam.json was rewritten above; bounce
        // ONLY waybeam (wfb stays up, radio link preserved). On Star6E a /set-
        // driven reinit would self-respawn and race our supervisor, so fpvd owns
        // the restart. The settle delay lets the SigmaStar driver drain the old
        // pipeline before the fresh waybeam re-inits (a video0.size change wedges
        // the VENC channel otherwise). No-op if waybeam is not currently supervised.
        if (encRestart)
            orch_.restart("waybeam",
                          std::chrono::milliseconds{paths_.waybeamRestartSettleMs});
        // Hot path: no wfb restart. Route the in-process controller before the
        // link hot-apply blocks below, so it
        // runs regardless of which hot return is taken below (the deferred
        // nicChannel return detaches a worker and returns early). start() binds
        // sockets + launches the thread; setConfig() hot-reloads; stop() joins.
        if (!enabledOld && enabledNew)
            startController();
        else if (enabledOld && !enabledNew)
            dl_.stop();
        else if (enabledOld && enabledNew && (subs.dynamicLink || link.videoRadiotap))
            // A stbc/ldpc retune is the only videoRadiotap change reachable under
            // DL (mcs/width/fec are locked), so refresh the controller snapshot;
            // the loop restates the radio with its current mcs (see reconcile).
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
        if (link.videoRadiotap && !link.nicWidth && !enabledNew) {
            // mcs/stbc/ldpc with no width change — push now (no link drop).
            // Skipped when DL is enabled: the controller is the sole writer of
            // the video radiotap there. Pushing the *config* mcs from here would
            // clobber the loop's adaptive MCS, so a stbc/ldpc retune is instead
            // routed through the controller (setConfig above), which restates
            // the radio preserving its current mcs/bw.
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
