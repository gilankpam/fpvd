#include "config/store.hpp"
#include <fstream>
#include <sstream>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>
#include <vector>

namespace fpvd {

nlohmann::json deepMergeJson(const nlohmann::json& base,
                              const nlohmann::json& overlay) {
    if (!base.is_object() || !overlay.is_object()) {
        return overlay;
    }
    nlohmann::json out = base;
    for (auto it = overlay.begin(); it != overlay.end(); ++it) {
        if (out.contains(it.key()) && out[it.key()].is_object()
            && it.value().is_object()) {
            out[it.key()] = deepMergeJson(out[it.key()], it.value());
        } else {
            out[it.key()] = it.value();
        }
    }
    return out;
}

// Recursively collect dotted paths of keys in `cfg` absent from the
// reference (code-default) object `ref`. Recurses only where both sides are
// objects; the top-level `services` map is skipped (free-form user processes).
static void collectUnknown(const nlohmann::json& cfg, const nlohmann::json& ref,
                           const std::string& prefix,
                           std::vector<std::string>& out) {
    if (!cfg.is_object() || !ref.is_object()) return;
    for (auto it = cfg.begin(); it != cfg.end(); ++it) {
        std::string path = prefix.empty() ? it.key() : prefix + "." + it.key();
        if (prefix.empty() && it.key() == "services") continue;  // free-form map
        if (!ref.contains(it.key())) { out.push_back(path); continue; }
        if (it.value().is_object() && ref[it.key()].is_object())
            collectUnknown(it.value(), ref[it.key()], path, out);
    }
}

std::vector<std::string> unknownConfigKeys(const nlohmann::json& cfg) {
    std::vector<std::string> out;
    collectUnknown(cfg, nlohmann::json(Config{}), "", out);
    return out;
}

Config loadEffective(const std::string& configPath) {
    nlohmann::json base = Config{};
    std::ifstream f(configPath);
    if (!f) return Config{};
    std::stringstream buf;
    buf << f.rdbuf();
    nlohmann::json fileJ;
    try { fileJ = nlohmann::json::parse(buf.str()); }
    catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("config parse error: ") + e.what());
    }
    for (auto& k : unknownConfigKeys(fileJ))
        std::fprintf(stderr, "fpvd: warning: unknown config key '%s' (ignored)\n",
                     k.c_str());
    auto merged = deepMergeJson(base, fileJ);
    try { return merged.get<Config>(); }
    catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("config schema: ") + e.what());
    }
}

void atomicWriteJson(const std::string& path, const nlohmann::json& j) {
    std::string tmp = path + ".tmp";
    std::filesystem::create_directories(
        std::filesystem::path(path).parent_path());
    int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) throw StoreError("open .tmp: " + std::string(strerror(errno)));
    std::string s = j.dump(2);
    s.push_back('\n');
    if (::write(fd, s.data(), s.size()) != (ssize_t)s.size()) {
        ::close(fd);
        throw StoreError("write: " + std::string(strerror(errno)));
    }
    if (::fsync(fd) != 0) {
        ::close(fd);
        throw StoreError("fsync: " + std::string(strerror(errno)));
    }
    ::close(fd);
    if (::rename(tmp.c_str(), path.c_str()) != 0) {
        throw StoreError("rename: " + std::string(strerror(errno)));
    }
}

} // namespace fpvd
