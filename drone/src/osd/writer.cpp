/* writer.cpp — msposd status-line writer (ported from dl_osd.c, lifted out of
 * dynlink and made thread-safe for the shared daemon-owned instance). */
#include "osd/writer.hpp"

#include <cstdio>

namespace fpvd::osd {

/* msposd directive prefix: `&L50` is color 5 (yellow) + zone 0 (TopLeft);
 * `&F30` sets font size 30. Without these directives msposd falls back to its
 * boot-default &F38 &L43 (huge font, "TopMoving" marquee-scroll). */
static constexpr const char* kOsdPrefix = "&L50&F30 ";

/* BF OSD token: 0 off (nothing), 1 armed-no-report, 2 working. ASCII so the
 * msposd font always renders it. */
static const char* bfToken(int bfCode) {
    return bfCode == 2 ? " B+" : bfCode == 1 ? " B-" : "";
}

OsdWriter::OsdWriter(std::string msgPath, bool enabled)
    : msgPath_(std::move(msgPath)), enabled_(enabled) {}

void OsdWriter::setEnabled(bool e) {
    std::lock_guard<std::mutex> lk(mu_);
    enabled_ = e;
}

void OsdWriter::flushLocked() {
    /* Write atomically: path.tmp -> rename(path). Avoids msposd reading a
     * half-written buffer. */
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

void OsdWriter::writeStatus(const dynlink::Decision& d, int rssiDbm, int bfCode,
                            uint64_t idrCount) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!enabled_) return;

    /* &T/&W/&B/&C are msposd placeholders (board temp, wifi-module temp,
     * video bitrate+fps, cpu%); msposd substitutes at render time. */
    std::snprintf(statusLine_, sizeof(statusLine_),
                  "%sMCS%u %uM (%u,%u) TX%d R%d I%u%s | &B T&T W&W CPU&C",
                  kOsdPrefix,
                  static_cast<unsigned>(d.mcs),
                  static_cast<unsigned>((d.bitrateKbps + 500) / 1000),
                  static_cast<unsigned>(d.k),
                  static_cast<unsigned>(d.n),
                  static_cast<int>(d.txPowerDbm),
                  rssiDbm,
                  static_cast<unsigned>(idrCount),
                  bfToken(bfCode));

    /* Fresh status = the link recovered (or never tripped). Clear any stale
     * event line so a past WATCHDOG/REJECT toast doesn't sit on the OSD forever
     * — msposd keeps rendering the last bytes we wrote, so we actively unset. */
    eventLine_[0] = '\0';
    flushLocked();
}

void OsdWriter::writeBaseLine(int bfCode) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!enabled_) return;

    /* Placeholders-only line for when the dynamic link isn't feeding the OSD.
     * msposd holds + re-renders it, substituting the &-placeholders live. */
    std::snprintf(statusLine_, sizeof(statusLine_),
                  "%s&B  T&T  W&W  CPU&C%s", kOsdPrefix, bfToken(bfCode));
    eventLine_[0] = '\0';
    flushLocked();
}

void OsdWriter::writeEvent(const std::string& text) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!enabled_) return;
    std::snprintf(eventLine_, sizeof(eventLine_),
                  "%s%s", kOsdPrefix, text.c_str());
    flushLocked();
}

void OsdWriter::eventWatchdog() {
    writeEvent("WATCHDOG safe_defaults");
}

} // namespace fpvd::osd
