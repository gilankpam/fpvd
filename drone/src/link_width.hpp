#pragma once

namespace fpvd {

// This driver realizes a 10MHz channel by underclocking the baseband while
// keeping 20MHz modulation. So the modulation width used by wfb_tx (-B) and
// the BF sounding frame is 20 for a 10MHz link; 20/40 pass through unchanged.
inline int modulationWidth(int linkWidth) { return linkWidth == 10 ? 20 : linkWidth; }

} // namespace fpvd
