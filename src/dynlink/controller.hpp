#pragma once
#include "dynlink/dedup.hpp"
#include "dynlink/encoder_client.hpp"
#include "dynlink/hello.hpp"
#include "dynlink/idr_listen.hpp"
#include "dynlink/osd.hpp"
#include "dynlink/radio_txpower.hpp"
#include "dynlink/runtime_config.hpp"
#include "dynlink/watchdog.hpp"
#include "dynlink/wire.hpp"
#include "translate/wfb_control.hpp"
#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <thread>

namespace fpvd::dynlink {

class DynamicLinkController {
public:
    explicit DynamicLinkController(Endpoints ep = {});
    ~DynamicLinkController();
    DynamicLinkController(const DynamicLinkController&) = delete;
    DynamicLinkController& operator=(const DynamicLinkController&) = delete;

    void start(const DlRuntimeConfig& snap, uint32_t generationId);
    void stop();                                  // idempotent; joins the thread
    bool running() const { return running_.load(); }
    void setConfig(const DlRuntimeConfig& snap);  // hot reload (stub for now; Task 17 fills it)
    DlStatus status() const;                       // snapshot of published status

private:
    void run(int evfd);                            // the poll(2) loop (Tasks 14-17); evfd passed from start()
    void stopLocked();                             // assumes lifetimeMu_ is already held
    void publishStatus(const DlStatus&);

    // Decision dispatch helpers (run on the control thread only — no locking).
    void dispatchTxApply(const DlRuntimeConfig& cfg, const Decision& d);
    void dispatchTxSafe(const DlRuntimeConfig& cfg);

    Endpoints ep_;
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopFlag_{false};
    std::atomic<int> eventFd_{-1};                 // reload/stop wake
    std::atomic<uint32_t> generationId_{0};
    std::mutex lifetimeMu_;                          // serializes start/stop/setConfig lifecycle transitions
    mutable std::mutex cfgMu_;
    std::shared_ptr<const DlRuntimeConfig> cfg_;   // guarded by cfgMu_
    mutable std::mutex statusMu_;
    DlStatus status_{};

    // Owned backend clients + control-loop state. Constructed in start()
    // from the config snapshot + endpoints; used ONLY from run() (the
    // control thread), so they need no locking. Held by unique_ptr/optional
    // because their ctors take args and they are (re)constructed per start().
    std::unique_ptr<WfbControlClient> wfb_;
    std::optional<EncoderClient>      enc_;
    std::optional<RadioTxpower>       radio_;
    std::optional<OsdWriter>          osd_;
    std::optional<Watchdog>           watchdog_;
    std::optional<HelloSm>            hello_;      // constructed in start(); used only from run()
    std::optional<IdrListener>        idr_;        // constructed in start(); fd owned by IdrListener
    Dedup                             dedup_;

    // Per-backend prev-state (diff baselines). lastTx_ is diffed against new
    // decisions inside dispatchTxApply. lastEnc_ tracks bitrate for direction
    // only (the encoder client owns its own internal diff). A "first/invalid"
    // baseline is signalled by magic != kWireMagic (port of dl_applier.c).
    //
    // lastRadio_ is maintained for structural parity with the C reference's
    // last_radio but is NOT the diff baseline for txpower decisions — the real
    // txpower diff lives inside RadioTxpower::current_, which setIface() resets
    // to nullopt on a watchdog trip, forcing the next decision to re-run iw
    // even if its txpower equals the safe value just pushed. Do NOT use
    // lastRadio_ for diffing; do NOT remove it (keep C-reference parity).
    Decision lastTx_{};
    Decision lastRadio_{};
    Decision lastEnc_{};
    Decision lastApplied_{};   // for OSD display only
    uint64_t lastDecisionMs_{0};
};

} // namespace fpvd::dynlink
