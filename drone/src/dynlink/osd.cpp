/* osd.cpp — msposd status-line writer (ported from dl_osd.c). */
#include "dynlink/osd.hpp"

#include <cerrno>
#include <cstdio>
#include <cstring>

namespace fpvd::dynlink {

/* msposd directive prefix: `&L50` is color 5 (yellow) + zone 0 (TopLeft);
 * `&F30` sets font size 30.  Same prefix used by the reference alink_drone.
 * Without these directives msposd falls back to its boot-default &F38 &L43
 * (huge font, "TopMoving" marquee-scroll). */
static constexpr const char* kOsdPrefix = "&L50&F30 ";

OsdWriter::OsdWriter(std::string msgPath, bool enabled,
                     uint32_t updateIntervalMs, bool debugLatency)
    : msgPath_(std::move(msgPath))
    , enabled_(enabled)
    , updateIntervalMs_(updateIntervalMs)
    , debugLatency_(debugLatency)   // stored; latency rendering deferred (dl_latency out of scope)
{}

void OsdWriter::bumpIdr() {
    ++idrCount_;
}

void OsdWriter::flush() {
    if (!enabled_) return;

    /* Write atomically: path.tmp -> rename(path).  Avoids msposd reading
     * a half-written buffer. */
    std::string tmpPath = msgPath_ + ".tmp";
    FILE* fd = std::fopen(tmpPath.c_str(), "w");
    if (!fd) {
        return;
    }
    if (eventLine_[0]) std::fprintf(fd, "%s\n", eventLine_);
    if (statusLine_[0]) std::fprintf(fd, "%s\n", statusLine_);
    std::fflush(fd);
    std::fclose(fd);
    if (std::rename(tmpPath.c_str(), msgPath_.c_str()) < 0) {
        std::remove(tmpPath.c_str());
    }
}

void OsdWriter::writeStatus(const Decision& d, int rssiDbm) {
    if (!enabled_) return;

    /* &T/&W/&B/&C are msposd placeholders (board temp, wifi-module temp,
     * video bitrate+fps, cpu%); msposd substitutes at render time. */
    std::snprintf(statusLine_, sizeof(statusLine_),
                  "%sMCS%u %uM (%u,%u)d%u TX%d R%d I%u | &B T&T W&W CPU&C",
                  kOsdPrefix,
                  static_cast<unsigned>(d.mcs),
                  static_cast<unsigned>((d.bitrateKbps + 500) / 1000),
                  static_cast<unsigned>(d.k),
                  static_cast<unsigned>(d.n),
                  static_cast<unsigned>(d.depth),
                  static_cast<int>(d.txPowerDbm),
                  rssiDbm,
                  static_cast<unsigned>(idrCount_));

    /* Fresh status = the link recovered (or never tripped).  Clear any
     * stale event line so a past WATCHDOG/REJECT toast doesn't sit on the
     * OSD forever — msposd will keep rendering the last bytes we wrote, so
     * we have to actively unset. */
    eventLine_[0] = '\0';

    /* debugLatency_ rendering is deferred (dl_latency module is out of scope).
     * The flag is stored but no debug_block is appended here. */

    flush();
}

void OsdWriter::writeEvent(const std::string& text) {
    if (!enabled_) return;
    std::snprintf(eventLine_, sizeof(eventLine_),
                  "%s%s", kOsdPrefix, text.c_str());
    flush();
}

void OsdWriter::eventWatchdog() {
    writeEvent("WATCHDOG safe_defaults");
}

} // namespace fpvd::dynlink
