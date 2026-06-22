#pragma once
/* writer.hpp — msposd status-line writer (the drone's /tmp/MSPOSD.msg source).
 * The daemon owns one always-on writer; both the controller (rich status column)
 * and the daemon (system-stats base column) write through it. msposd renders the
 * lines onto the video, coloring per line and substituting &-placeholders. */
#include "dynlink/wire.hpp" // fpvd::dynlink::Decision
#include <cstdint>
#include <mutex>
#include <string>

namespace fpvd::osd {

class OsdWriter {
  public:
    OsdWriter(std::string msgPath, bool enabled);

    /* Rich glyph column for the last applied dynamic-link decision. idrCount is
     * the always-on relay's received count; bfCode is 0 off / 1 armed / 2 active. */
    void writeStatus(const dynlink::Decision& d, int bfCode, uint64_t idrCount);

    /* System-stats subset (video/temps/cpu + BF) shown when the dynamic link
     * isn't feeding the OSD. bfCode is 0/1/2. */
    void writeBaseLine(int bfCode);

    /* Transient red event line rendered above the column. */
    void writeEvent(const std::string& text);

    /* Convenience event: WATCHDOG safe_defaults. */
    void eventWatchdog();

    /* Enable/disable all OSD writes at runtime; on on->off this clears the file. */
    void setEnabled(bool e);

  private:
    /* Write eventLine_ (if set) + statusLines_ atomically to msgPath_. Assumes
     * mu_ is held. Both empty -> truncates the file, clearing the overlay. */
    void flushLocked();

    std::mutex mu_;
    std::string msgPath_;
    bool enabled_;
    std::string statusLines_; // '\n'-joined column, no leading/trailing newline
    std::string eventLine_;   // optional red toast, rendered above the column
};

} // namespace fpvd::osd
