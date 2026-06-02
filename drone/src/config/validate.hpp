#pragma once
#include "config/schema.hpp"
#include <string>
#include <vector>

namespace fpvd {

struct ValidationError {
    std::string path;
    std::string message;
};

std::vector<ValidationError> validate(const Config& c);

} // namespace fpvd
