#pragma once
#include "config/schema.hpp"
#include <stdexcept>
#include <string>
#include <vector>

namespace fpvd {

struct StoreError : public std::runtime_error {
    using std::runtime_error::runtime_error;
};

// Deep-merge two JSON values. Objects are merged key-by-key recursively;
// arrays are replaced wholesale; scalars in `overlay` win.
nlohmann::json deepMergeJson(const nlohmann::json& base, const nlohmann::json& overlay);

// Load the effective configuration by merging the on-disk full config at
// `configPath` onto the code defaults (nlohmann::json(Config{})): a present
// key wins, a missing key falls back to the code default. Unknown/deprecated
// keys are warned about (stderr) and ignored, which keeps config upgrades
// safe. A wrong-typed or out-of-range value of a KNOWN key is still a hard
// error (StoreError). A missing config file yields pure code defaults.
Config loadEffective(const std::string& configPath);

// Return the dotted paths of keys present in `cfg` that do not exist in the
// code-default config (nlohmann::json(Config{})). Used to warn on
// deprecated/renamed/stray keys. The free-form `services` map is skipped:
// its child keys are arbitrary user-named processes, not a fixed schema.
std::vector<std::string> unknownConfigKeys(const nlohmann::json& cfg);

// Atomically write `j` to `path` (write to .tmp, fsync, rename).
// Throws StoreError on I/O error.
void atomicWriteJson(const std::string& path, const nlohmann::json& j);

} // namespace fpvd
