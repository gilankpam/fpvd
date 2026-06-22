#pragma once
#include "waybeam/client.hpp"
#include <cstdint>

namespace fpvd::dynlink {

struct RoiCurve {
    uint16_t thresholdKbps;
    uint16_t lowAnchorKbps;
    int8_t floor;
    uint8_t step;
};

class EncoderClient {
  public:
    // Transport is injected (non-owning); the referenced WaybeamClient must
    // outlive this EncoderClient.
    EncoderClient(WaybeamClient& client, RoiCurve roi);

    // GET /api/v1/set?video0.bitrate=&fpv.roiQp=&[video0.fps=]. Diff-based.
    // bitrate==0 is a no-op sentinel. Returns 0 ok/no-op, -1 HTTP fail.
    int apply(uint16_t bitrateKbps, uint8_t fps);

    // Push safe bitrate (roiQp recomputed, fps unchanged). Returns 0/-1.
    int applySafe(uint16_t safeBitrateKbps);

    void setRoiCurve(RoiCurve roi) { roi_ = roi; }

  private:
    WaybeamClient* client_;
    RoiCurve roi_;

    bool lastValid_{false};
    uint16_t lastBitrate_{0};
    int8_t lastRoiQp_{0};
    uint8_t lastFps_{0};
};

} // namespace fpvd::dynlink
