#pragma once
#include "config/schema.hpp"
#include "supervise/supervisor.hpp"
#include <string>
#include <vector>

namespace fpvd {

// Observe-only probe link: one FEC-off wfb_tx + one feeder per probe MCS,
// mirroring the video PHY (width/stbc/ldpc, long GI). Returns empty when the
// probe is disabled. index i -> radio_port basePort+i, feed port baseFeedPort+i.
std::vector<SupervisedSpec> buildProbeSpecs(const Config& c,
                                            const std::string& iface,
                                            const std::string& key,
                                            const std::string& feederPath);

} // namespace fpvd
