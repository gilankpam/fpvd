#pragma once
/* osd.hpp — msposd status-line writer (ported from dl_osd.h). */
#include "dynlink/wire.hpp"
#include <cstdint>
#include <string>

namespace fpvd::dynlink {

class OsdWriter {
public:
    OsdWriter(std::string msgPath, bool enabled, uint32_t updateIntervalMs, bool debugLatency);

    /* Write the steady-state line reflecting the last applied decision.
     * rssiDbm is a hint; pass 0 if unknown. Safe to call on every tick
     * — file is rewritten atomically (tmpfile+rename). */
    void writeStatus(const Decision& d, int rssiDbm, int bfCode = 0);

    /* Write a transient event line on top of the status line. msposd
     * renders both if present. */
    void writeEvent(const std::string& text);

    /* Increment the running counter of received IDR requests. Cheap;
     * safe to call from the main poll loop on every drain wake. */
    void bumpIdr();

    /* Convenience event: WATCHDOG safe_defaults. */
    void eventWatchdog();

    /* Hot-reconcile: update enabled flag at runtime. */
    void setEnabled(bool e) { enabled_ = e; }

private:
    std::string msgPath_;
    bool        enabled_;
    uint32_t    updateIntervalMs_;
    bool        debugLatency_;   /* stored but not rendered (dl_latency out of scope) */
    uint64_t    idrCount_{0};
    char        statusLine_[128]{};
    char        eventLine_[128]{};

    /* Write eventLine_ (if set) + statusLine_ atomically to msgPath_. */
    void flush();
};

} // namespace fpvd::dynlink
