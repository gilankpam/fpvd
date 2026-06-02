/* apply_direction.hpp — port of dl_apply.h: direction helper for staggered
 * profile apply.
 *
 * Header-only: the function is pure and trivial enough that a separate
 * translation unit would just be ceremony, and keeping it inline lets tests
 * link without pulling in unrelated translation units.
 */
#pragma once
#include <cstdint>

namespace fpvd::dynlink {

enum class ApplyDir {
    Equal =  0,  ///< single-shot, no stagger
    Up    =  1,  ///< tx+radio first, encoder after gap
    Down  = -1,  ///< encoder first, tx+radio after gap
};

inline ApplyDir applyDirection(uint16_t prevBitrateKbps,
                                uint16_t newBitrateKbps,
                                bool     firstDecision) {
    if (firstDecision)                          return ApplyDir::Equal;
    if (newBitrateKbps > prevBitrateKbps)       return ApplyDir::Up;
    if (newBitrateKbps < prevBitrateKbps)       return ApplyDir::Down;
    return ApplyDir::Equal;
}

} // namespace fpvd::dynlink
