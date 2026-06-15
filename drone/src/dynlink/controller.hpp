#pragma once
#include "dynlink/dedup.hpp"
#include "dynlink/encoder_client.hpp"
#include "dynlink/radio_txpower.hpp"
#include "osd/writer.hpp"
#include "dynlink/runtime_config.hpp"
#include "dynlink/watchdog.hpp"
#include "dynlink/wire.hpp"
#include "translate/wfb_control.hpp"
#include <atomic>
#include <functional>
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

    void start(const DlRuntimeConfig& snap);
    void stop();                                  // idempotent; joins the thread
    bool running() const { return running_.load(); }
    void setConfig(const DlRuntimeConfig& snap);  // hot reload (stub for now; Task 17 fills it)
    DlStatus status() const;                       // snapshot of published status

    // Set once before start(): supplies the BF OSD code (0/1/2) for the status
    // line. Invoked on the control thread; must be set while stopped.
    void setBfCodeProvider(std::function<int()> f) { bfCodeProvider_ = std::move(f); }

    // Set once before start(): supplies the running IDR-request count for the
    // status line. The relay that owns this count lives outside the controller
    // (always-on, daemon-supervised), so the OSD reads it through this provider.
    void setIdrCountProvider(std::function<uint64_t()> f) { idrCountProvider_ = std::move(f); }

    // Set once before start(): the daemon-owned, always-on OSD writer the loop
    // pushes its status/event lines to (non-owning; must outlive the controller).
    // nullptr disables OSD writes from the loop.
    void setOsdWriter(osd::OsdWriter* w) { osd_ = w; }

    // Probe rung selector: the observe-only probe rides one rung above the video
    // MCS, clamped to the hardware ceiling. Static + header-inline so it is unit
    // testable without constructing the controller (which binds sockets/threads).
    static int probeRungFor(int mcs, int ceiling) {
        return mcs + 1 < ceiling ? mcs + 1 : ceiling;
    }

    // OSD status-write throttle gate. The GS sends decisions ~10 Hz, but the OSD
    // is a human-readable display that only needs refreshing at
    // osdUpdateIntervalMs (default 1 Hz); rewriting the msg file on every
    // decision is wasted I/O + msposd churn. lastMs==0 means "never written" ->
    // always due. Static + header-inline so the gate is unit-testable without
    // constructing the controller.
    static bool osdWriteDue(uint64_t nowMs, uint64_t lastMs, uint32_t intervalMs) {
        return lastMs == 0 || nowMs - lastMs >= intervalMs;
    }

private:
    void run(int evfd);                            // the poll(2) loop (Tasks 14-17); evfd passed from start()
    void stopLocked();                             // assumes lifetimeMu_ is already held
    void publishStatus(const DlStatus&);

    // Decision dispatch helpers (run on the control thread only — no locking).
    void dispatchTxApply(const DlRuntimeConfig& cfg, const Decision& d);
    void dispatchTxSafe(const DlRuntimeConfig& cfg);

    Endpoints ep_;
    WaybeamClient wb_;            // transport for enc_; built from ep_ in the ctor
    std::thread thread_;
    std::atomic<bool> running_{false};
    std::atomic<bool> stopFlag_{false};
    std::atomic<int> eventFd_{-1};                 // reload/stop wake
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
    std::unique_ptr<WfbControlClient> probeWfb_;   // probe tx retune (nullptr if disabled)
    int lastProbeMcs_{-1};                          // last rung pushed to the probe
    std::optional<EncoderClient>      enc_;
    std::optional<RadioTxpower>       radio_;
    osd::OsdWriter*                   osd_{nullptr};   // non-owning; set via setOsdWriter()
    std::optional<Watchdog>           watchdog_;
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
    uint64_t lastOsdWriteMs_{0};   // throttle baseline for osdWriteDue (0 = never written)
    std::function<int()> bfCodeProvider_;   // 0 if unset
    std::function<uint64_t()> idrCountProvider_;   // 0 if unset
};

} // namespace fpvd::dynlink
