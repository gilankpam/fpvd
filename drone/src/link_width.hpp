#pragma once

namespace fpvd {

// This driver realizes a 5 or 10 MHz channel by underclocking the baseband
// (4x / 2x) while keeping 20 MHz modulation. So the modulation width used by
// wfb_tx (-B) and the BF sounding frame is 20 for a 5/10 MHz link; 20/40 pass
// through unchanged.
inline int modulationWidth(int linkWidth) {
    return (linkWidth == 5 || linkWidth == 10) ? 20 : linkWidth;
}

} // namespace fpvd
