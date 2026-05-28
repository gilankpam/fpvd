#pragma once
#include "config/schema.hpp"
#include <string>
#include <vector>

namespace fpvd {

// Build the argv (including argv[0] = /usr/bin/dl-applier) for the
// drone-side dl-applier. `iface` is the wlan device picked by
// radio-up.sh.
std::vector<std::string> dynamicLinkArgs(const Config& c,
                                          const std::string& iface);

} // namespace fpvd
