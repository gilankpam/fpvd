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

// Deep-merge two JSON values. Objects are merged key-by-key recursively;
// arrays are replaced wholesale; scalars in `overlay` win.
nlohmann::json deepMergeJson(const nlohmann::json& base,
                              const nlohmann::json& overlay);

// Load defaults from `defaultsPath`, optionally overlay `overlayPath` if
// it exists, and return the effective configuration. Throws StoreError
// on parse errors in either file.
Config loadEffective(const std::string& defaultsPath,
                     const std::string& overlayPath);

// Return the JSON diff: every key/path in `effective` that differs from
// the same path in `defaults`. Arrays are diffed as wholesale.
nlohmann::json computeOverlay(const nlohmann::json& defaults,
                               const nlohmann::json& effective);

// Atomically write `j` to `path` (write to .tmp, fsync, rename).
// Throws StoreError on I/O error.
void atomicWriteJson(const std::string& path, const nlohmann::json& j);

} // namespace fpvd
