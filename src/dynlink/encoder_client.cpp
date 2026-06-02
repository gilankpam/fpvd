/* encoder_client.cpp — dynamic-link encoder policy over the shared
 * WaybeamClient transport. Query strings and dedup are unchanged from the
 * original dl_backend_enc port. */
#include "dynlink/encoder_client.hpp"
#include "dynlink/roi_qp.hpp"

#include <map>
#include <string>

namespace fpvd::dynlink {

EncoderClient::EncoderClient(WaybeamClient& client, uint32_t minIdrIntervalMs,
                             RoiCurve roi)
    : client_(&client), minIdrIntervalMs_(minIdrIntervalMs), roi_(roi) {}

int EncoderClient::apply(uint16_t bitrateKbps, uint8_t fps) {
    if (bitrateKbps == 0) return 0;  // sentinel: don't push

    auto roiQp = static_cast<int8_t>(
        computeRoiQp(bitrateKbps, roi_.thresholdKbps, roi_.lowAnchorKbps,
                     roi_.floor, roi_.step));

    if (lastValid_ && lastBitrate_ == bitrateKbps &&
        lastRoiQp_ == roiQp && lastFps_ == fps) {
        return 0;  // no-op: nothing changed
    }

    std::map<std::string, std::string> fields{
        {"video0.bitrate", std::to_string(static_cast<unsigned>(bitrateKbps))},
        {"fpv.roiQp",      std::to_string(static_cast<int>(roiQp))},
    };
    if (fps != 0)
        fields["video0.fps"] = std::to_string(static_cast<unsigned>(fps));

    bool ok = client_->setFields(fields);
    if (ok) {
        lastBitrate_ = bitrateKbps;
        lastRoiQp_   = roiQp;
        lastFps_     = fps;
        lastValid_   = true;
    }
    return ok ? 0 : -1;
}

int EncoderClient::requestIdr(uint64_t nowMs) {
    if (idrEverSent_ &&
        (nowMs - lastIdrMs_) < static_cast<uint64_t>(minIdrIntervalMs_)) {
        return 1;  // throttled
    }
    bool ok = client_->get("/request/idr");
    lastIdrMs_   = nowMs;       // arm throttle on ANY attempt (even failure)
    idrEverSent_ = true;
    return ok ? 0 : -1;
}

int EncoderClient::applySafe(uint16_t safeBitrateKbps) {
    auto roiQp = static_cast<int8_t>(
        computeRoiQp(safeBitrateKbps, roi_.thresholdKbps, roi_.lowAnchorKbps,
                     roi_.floor, roi_.step));

    std::map<std::string, std::string> fields{
        {"video0.bitrate", std::to_string(static_cast<unsigned>(safeBitrateKbps))},
        {"fpv.roiQp",      std::to_string(static_cast<int>(roiQp))},
    };  // fps omitted (matches original apply_safe behaviour)

    bool ok = client_->setFields(fields);
    if (ok) {
        lastBitrate_ = safeBitrateKbps;
        lastRoiQp_   = roiQp;
        lastFps_     = 0;
        lastValid_   = true;
    }
    return ok ? 0 : -1;
}

} // namespace fpvd::dynlink
