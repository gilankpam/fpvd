#pragma once
#include "config/schema.hpp"
#include <set>

namespace fpvd {

struct SubsystemDiff {
    bool radio{false};
    bool encoder{false};
    bool telemetry{false};
    std::set<std::string> servicesAffected{};
};

SubsystemDiff diffSubsystems(const Config& a, const Config& b);

} // namespace fpvd
