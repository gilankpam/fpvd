#pragma once
#include "config/schema.hpp"
#include <stdexcept>
#include <string>

namespace fpvd {

struct StoreError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Load the baseline configuration from a JSON file. Throws StoreError
// on read failure or invalid JSON.
Config loadDefaults(const std::string& path);

} // namespace fpvd
