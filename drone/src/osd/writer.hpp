#pragma once
/* writer.hpp — msposd status-line writer (the drone's /tmp/MSPOSD.msg source).
 * Moved out of dynlink: the OSD message file is a drone-wide artifact rendered
 * whether or not the dynamic link is enabled, so the daemon owns one always-on
 * writer that both the controller (rich status line) and the daemon (system-
 * stats base line) write through. */
#include "dynlink/wire.hpp"   // fpvd::dynlink::Decision
#include <cstdint>
#include <mutex>
#include <string>

namespace fpvd::osd {

class OsdWriter {
public:
    OsdWriter(std::string msgPath, bool enabled);

    /* Rich status line for the last applied dynamic-link decision. rssiDbm is a
     * hint (0 if unknown); idrCount is the always-on relay's received-burst
     * count; bfCode is 0/1/2. */
    void writeStatus(const dynlink::Decision& d, int rssiDbm, int bfCode,
                     uint64_t idrCount);

    /* System-stats base line (placeholders only) shown when the dynamic link
     * isn't feeding the OSD. bfCode is 0/1/2. */
    void writeBaseLine(int bfCode);

    /* Transient event line rendered above the status line. msposd renders both
     * if present. */
    void writeEvent(const std::string& text);

    /* Convenience event: WATCHDOG safe_defaults. */
    void eventWatchdog();

    /* Enable/disable all OSD writes at runtime (gates the status AND base line).
     * The daemon owns this flag from the top-level `osd.enabled` config. */
    void setEnabled(bool e);

private:
    /* Write eventLine_ (if set) + statusLine_ atomically to msgPath_. Assumes
     * mu_ is held. With both lines unset this truncates the file to empty,
     * which clears the msposd overlay (see setEnabled's on->off path). */
    void flushLocked();

    std::mutex  mu_;
    std::string msgPath_;
    bool        enabled_;
    char        statusLine_[128]{};
    char        eventLine_[128]{};
};

} // namespace fpvd::osd
