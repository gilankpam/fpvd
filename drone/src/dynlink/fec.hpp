/* fec.hpp — latency-sized k (block-fill) + fixed-ratio n (Phase 3a).
 * Port of gs fpvdgs/dynlink/dynamic_fec.compute_k / compute_n. Pure. */
#pragma once

namespace fpvd::dynlink {

// Block size, sized so block_fill stays inside one frame period. Anchored on
// the encoder bitrate at n_base (= wireTarget / (1 + baseRedundancyRatio)).
// Returns kMin for any non-positive input. Result clamped to [kMin, kMax].
int computeK(double wireTargetKbps, int mtuBytes, int fps, double baseRedundancyRatio,
             double blocksPerFrame, int kMin, int kMax);

// n = ceil(k * (1 + baseRedundancyRatio)). Fixed ratio; no escalation.
int computeN(int k, double baseRedundancyRatio);

} // namespace fpvd::dynlink
