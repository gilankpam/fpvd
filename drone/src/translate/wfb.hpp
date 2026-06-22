#pragma once
#include "config/schema.hpp"
#include <string>
#include <vector>

namespace fpvd {

// wfb_tx control socket port for the video instance (-C). Bound to 127.0.0.1.
constexpr int kVideoControlPort = 8000;

enum class WfbRole { VideoTx, TunRx, TunTx, TlmRx, TlmTx };

// Build the argv (including argv[0] = binary path) for a wfb-ng role.
std::vector<std::string> wfbArgs(const Config& c, WfbRole role, const std::string& iface,
                                 const std::string& keyPath);

// wfb_tun argv (does not need config — purely topology).
std::vector<std::string> wfbTunArgs();

} // namespace fpvd
