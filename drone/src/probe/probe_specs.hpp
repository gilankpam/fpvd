#pragma once
#include "config/schema.hpp"
#include "supervise/supervisor.hpp"
#include <string>
#include <vector>

namespace fpvd {

// Observe-only probe link: ONE FEC-off wfb_tx (with a -C control port for live
// MCS retune) + one feeder, mirroring the video PHY (width/stbc/ldpc, long GI),
// FEC off (k=1 n=1). `mcs` is the initial rung; it is clamped to kProbeMcsCeiling.
// Lifecycle is owned by the caller (seed when dynamicLink is enabled).
std::vector<SupervisedSpec> buildProbeSpecs(const Config& c,
                                            const std::string& iface,
                                            const std::string& key,
                                            const std::string& feederPath,
                                            int mcs);

} // namespace fpvd
