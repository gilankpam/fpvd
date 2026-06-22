/* writer.cpp — msposd glyph-column writer. Composes a multi-line message; each
 * line carries its own &L color directive + an icon glyph + value. */
#include "osd/writer.hpp"

#include "osd/osd_constants.hpp"

#include <cstdio>
#include <string>
#include <vector>

namespace fpvd::osd {

namespace {

/* "&L{color}{zone}&F{size} {glyph} {value}" — one OSD line. One space between the
 * glyph and its value so the numbers sit off the icon (and, with the monospaced
 * Nerd Font, align in a column). Skipped when either side is empty: the BF line is
 * glyph-only, the event line is glyph-less. */
std::string osdLine(int color, const char* glyph, const std::string& value) {
    char head[32];
    std::snprintf(head, sizeof(head), "&L%d%d&F%d ", color, kOsdZone, kOsdFontSize);
    std::string line(head);
    line += glyph;
    if (glyph[0] != '\0' && !value.empty())
        line += ' ';
    line += value;
    return line;
}

int colorForMcs(unsigned mcs) {
    if (mcs == 0)
        return kColRed; // failsafe rung
    if (mcs >= kMcsGood)
        return kColGreen;
    return kColYellow;
}

const char* signalGlyph(unsigned mcs) {
    if (mcs >= kMcsGood)
        return kGlyphSignal3;
    if (mcs >= 2)
        return kGlyphSignal2;
    return kGlyphSignal1;
}

/* Beamforming line (or empty string to omit). bfCode: 0 off, 1 armed, 2 active. */
std::string bfLine(int bfCode) {
    if (bfCode == 2)
        return osdLine(kColCyan, kGlyphAntenna, ""); // active + fresh CBR report
    if (bfCode == 1)
        return osdLine(kColYellow, kGlyphAntenna, ""); // armed but stale / no report
    return {};
}

std::string join(const std::vector<std::string>& lines) {
    std::string out;
    for (const auto& l : lines) {
        if (l.empty())
            continue; // skip omitted lines (e.g. BF off)
        if (!out.empty())
            out += '\n';
        out += l;
    }
    return out;
}

} // namespace

OsdWriter::OsdWriter(std::string msgPath, bool enabled)
    : msgPath_(std::move(msgPath)), enabled_(enabled) {}

void OsdWriter::setEnabled(bool e) {
    std::lock_guard<std::mutex> lk(mu_);
    const bool wasEnabled = enabled_;
    enabled_ = e;
    /* On on->off, actively clear: msposd holds + re-renders the last bytes, so
     * flipping the flag alone leaves a stale overlay forever. */
    if (wasEnabled && !e) {
        statusLines_.clear();
        eventLine_.clear();
        flushLocked();
    }
}

void OsdWriter::flushLocked() {
    /* Atomic write: path.tmp -> rename(path), so msposd never reads a partial file. */
    std::string tmpPath = msgPath_ + ".tmp";
    FILE* fd = std::fopen(tmpPath.c_str(), "w");
    if (!fd)
        return;
    if (!eventLine_.empty())
        std::fprintf(fd, "%s\n", eventLine_.c_str());
    if (!statusLines_.empty())
        std::fprintf(fd, "%s\n", statusLines_.c_str());
    std::fflush(fd);
    std::fclose(fd);
    if (std::rename(tmpPath.c_str(), msgPath_.c_str()) < 0)
        std::remove(tmpPath.c_str());
}

void OsdWriter::writeStatus(const dynlink::Decision& d, int bfCode, uint64_t idrCount) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!enabled_)
        return;

    const unsigned mcs = d.mcs;
    const int mcsColor = colorForMcs(mcs);
    const unsigned mbps = (static_cast<unsigned>(d.bitrateKbps) + 500u) / 1000u;

    std::vector<std::string> lines;
    lines.push_back(osdLine(mcsColor, signalGlyph(mcs), "MCS" + std::to_string(mcs)));
    lines.push_back(osdLine(mcsColor, kGlyphSpeed, std::to_string(mbps) + "Mbps"));
    lines.push_back(osdLine(kColWhite, kGlyphShield,
                            std::to_string(static_cast<unsigned>(d.k)) + "/" +
                                std::to_string(static_cast<unsigned>(d.n))));
    lines.push_back(
        osdLine(kColWhite, kGlyphFlash, std::to_string(static_cast<int>(d.txPowerDbm)) + "dBm"));
    lines.push_back(bfLine(bfCode));
    lines.push_back(osdLine(kColWhite, kGlyphRefresh, std::to_string(idrCount)));
    /* msposd substitutes &B (bitrate+fps), &T/&W (temps), &C (cpu%) at render. */
    lines.push_back(osdLine(kColWhite, kGlyphFilm, "&B"));
    lines.push_back(osdLine(kColWhite, kGlyphThermo, std::string("&T") + kUnitDegC));
    lines.push_back(osdLine(kColWhite, kGlyphWifi, std::string("&W") + kUnitDegC));
    lines.push_back(osdLine(kColWhite, kGlyphCpu, "&C"));

    statusLines_ = join(lines);
    eventLine_.clear(); // fresh status = recovered; drop any stale toast
    flushLocked();
}

void OsdWriter::writeBaseLine(int bfCode) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!enabled_)
        return;

    std::vector<std::string> lines;
    lines.push_back(bfLine(bfCode));
    lines.push_back(osdLine(kColWhite, kGlyphFilm, "&B"));
    lines.push_back(osdLine(kColWhite, kGlyphThermo, std::string("&T") + kUnitDegC));
    lines.push_back(osdLine(kColWhite, kGlyphWifi, std::string("&W") + kUnitDegC));
    lines.push_back(osdLine(kColWhite, kGlyphCpu, "&C"));

    statusLines_ = join(lines);
    // Safe to clear unconditionally: the heartbeat only writes the base line
    // while DL is stopped, and events are emitted only while DL runs.
    eventLine_.clear();
    flushLocked();
}

void OsdWriter::writeEvent(const std::string& text) {
    std::lock_guard<std::mutex> lk(mu_);
    if (!enabled_)
        return;
    eventLine_ = osdLine(kColRed, "", text); // red toast, glyph-less
    flushLocked();
}

void OsdWriter::eventWatchdog() { writeEvent("WATCHDOG safe_defaults"); }

} // namespace fpvd::osd
