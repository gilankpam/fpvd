#pragma once
#include "daemon.hpp"
#include <nlohmann/json.hpp>

namespace fpvd {

nlohmann::json buildStatus(Daemon& d);

} // namespace fpvd
