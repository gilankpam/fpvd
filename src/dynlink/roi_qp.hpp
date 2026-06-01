/* roi_qp.hpp — port of dl_compute_roi_qp_raw from dl_backend_enc.c.
 * Returns a ROI-QP delta in [floor, 0]. */
#pragma once
#include <cstdint>

namespace fpvd::dynlink {

// Port of dl_compute_roi_qp_raw. Returns a delta in [floor, 0].
// Precondition: thresholdKbps > lowAnchorKbps (span > 0).
int computeRoiQp(uint16_t bitrateKbps,
                 uint16_t thresholdKbps,
                 uint16_t lowAnchorKbps,
                 int8_t   floor,
                 uint8_t  step);

} // namespace fpvd::dynlink
