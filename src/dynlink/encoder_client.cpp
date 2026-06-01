/* encoder_client.cpp — waybeam HTTP backend (port of dl_backend_enc.c).
 *
 * API:
 *   GET /api/v1/set?video0.bitrate=<kbps>&fpv.roiQp=<delta>[&video0.fps=<fps>]
 *   GET /request/idr
 */
#include "dynlink/encoder_client.hpp"
#include "dynlink/roi_qp.hpp"

#include <httplib.h>

#include <cstdio>
#include <string>

namespace fpvd::dynlink {

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
EncoderClient::EncoderClient(std::string host, uint16_t port,
                             uint32_t minIdrIntervalMs, RoiCurve roi)
    : host_(std::move(host))
    , port_(port)
    , minIdrIntervalMs_(minIdrIntervalMs)
    , roi_(roi)
{}

// ---------------------------------------------------------------------------
// Private HTTP helper
// ---------------------------------------------------------------------------
int EncoderClient::httpGet(const std::string& path) {
    httplib::Client cli(host_, static_cast<int>(port_));
    // Short timeouts so a dead encoder doesn't block the caller.
    cli.set_connection_timeout(0, 300'000);  // 300 ms
    cli.set_read_timeout(0, 500'000);        // 500 ms
    auto res = cli.Get(path.c_str());
    if (res && res->status / 100 == 2) return 0;
    return -1;
}

// ---------------------------------------------------------------------------
// apply  — port of apply_set + dl_backend_enc_apply
// ---------------------------------------------------------------------------
int EncoderClient::apply(uint16_t bitrateKbps, uint8_t fps) {
    if (bitrateKbps == 0) return 0;  // sentinel: don't push

    auto roiQp = static_cast<int8_t>(
        computeRoiQp(bitrateKbps,
                     roi_.thresholdKbps,
                     roi_.lowAnchorKbps,
                     roi_.floor,
                     roi_.step));

    if (lastValid_ &&
        lastBitrate_ == bitrateKbps &&
        lastRoiQp_   == roiQp &&
        lastFps_     == fps) {
        return 0;  // no-op: nothing changed
    }

    // Build query string.
    // Always emit fpv.roiQp (signed) — even when 0, to clear ROI on waybeam.
    char path[256];
    int  n = std::snprintf(path, sizeof(path),
                           "/api/v1/set?video0.bitrate=%u&fpv.roiQp=%d",
                           static_cast<unsigned>(bitrateKbps),
                           static_cast<int>(roiQp));
    if (n < 0 || static_cast<size_t>(n) >= sizeof(path)) return -1;

    if (fps != 0) {
        int m = std::snprintf(path + n, sizeof(path) - static_cast<size_t>(n),
                              "&video0.fps=%u",
                              static_cast<unsigned>(fps));
        if (m < 0 || static_cast<size_t>(m) >= sizeof(path) - static_cast<size_t>(n))
            return -1;
    }

    int rc = httpGet(path);
    if (rc == 0) {
        lastBitrate_ = bitrateKbps;
        lastRoiQp_   = roiQp;
        lastFps_     = fps;
        lastValid_   = true;
    }
    return rc;
}

// ---------------------------------------------------------------------------
// requestIdr — port of dl_backend_enc_request_idr
// ---------------------------------------------------------------------------
int EncoderClient::requestIdr(uint64_t nowMs) {
    if (idrEverSent_ && (nowMs - lastIdrMs_) < static_cast<uint64_t>(minIdrIntervalMs_)) {
        return 1;  // throttled
    }

    int rc = httpGet("/request/idr");
    // Arm throttle on ANY attempt (even on failure) — spam prevention.
    lastIdrMs_   = nowMs;
    idrEverSent_ = true;
    return rc;
}

// ---------------------------------------------------------------------------
// applySafe — port of dl_backend_enc_apply_safe
// ---------------------------------------------------------------------------
int EncoderClient::applySafe(uint16_t safeBitrateKbps) {
    auto roiQp = static_cast<int8_t>(
        computeRoiQp(safeBitrateKbps,
                     roi_.thresholdKbps,
                     roi_.lowAnchorKbps,
                     roi_.floor,
                     roi_.step));

    // fps=0 → no video0.fps in query (matches apply_set behaviour)
    char path[256];
    int  n = std::snprintf(path, sizeof(path),
                           "/api/v1/set?video0.bitrate=%u&fpv.roiQp=%d",
                           static_cast<unsigned>(safeBitrateKbps),
                           static_cast<int>(roiQp));
    if (n < 0 || static_cast<size_t>(n) >= sizeof(path)) return -1;

    int rc = httpGet(path);
    if (rc == 0) {
        lastBitrate_ = safeBitrateKbps;
        lastRoiQp_   = roiQp;
        lastFps_     = 0;
        lastValid_   = true;
    }
    return rc;
}

} // namespace fpvd::dynlink
