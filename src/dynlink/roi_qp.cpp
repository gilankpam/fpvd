/* roi_qp.cpp — port of dl_compute_roi_qp_raw from dl_backend_enc.c.
 * Arithmetic is transcribed exactly: integer division truncates toward zero,
 * result clamped to [floor, 0]. Division-by-zero is not guarded (caller
 * must ensure thresholdKbps > lowAnchorKbps, matching the C contract). */
#include "roi_qp.hpp"

namespace fpvd::dynlink {

int computeRoiQp(uint16_t bitrateKbps,
                 uint16_t thresholdKbps,
                 uint16_t lowAnchorKbps,
                 int8_t   floor,
                 uint8_t  step) {
    if (bitrateKbps >= thresholdKbps) return 0;
    int span  = static_cast<int>(thresholdKbps) - static_cast<int>(lowAnchorKbps);
    int delta = static_cast<int>(bitrateKbps)   - static_cast<int>(lowAnchorKbps);
    if (delta < 0) delta = 0;
    int raw = (static_cast<int>(floor) * (span - delta)) / span;  // negative
    int q   = (raw / static_cast<int>(step)) * static_cast<int>(step);  // truncate toward zero
    if (q < static_cast<int>(floor)) q = static_cast<int>(floor);
    if (q > 0) q = 0;
    return q;
}

} // namespace fpvd::dynlink
