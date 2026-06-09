#include "config/store.hpp"
#include <fstream>
#include <sstream>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <filesystem>

namespace fpvd {

static nlohmann::json readJsonFile(const std::string& path,
                                    const std::string& what) {
    std::ifstream f(path);
    if (!f) throw StoreError("failed to open " + what + ": " + path);
    std::stringstream buf;
    buf << f.rdbuf();
    try {
        return nlohmann::json::parse(buf.str());
    } catch (const nlohmann::json::exception& e) {
        throw StoreError(what + " parse error: " + e.what());
    }
}

Config loadDefaults(const std::string& path) {
    auto j = readJsonFile(path, "defaults");
    try {
        return j.get<Config>();
    } catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("defaults schema: ") + e.what());
    }
}

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

Config loadEffective(const std::string& defaultsPath,
                     const std::string& overlayPath) {
    auto baseJ = readJsonFile(defaultsPath, "defaults");
    std::ifstream f(overlayPath);
    if (!f) {
        try { return baseJ.get<Config>(); }
        catch (const nlohmann::json::exception& e) {
            throw StoreError(std::string("defaults schema: ") + e.what());
        }
    }
    std::stringstream buf;
    buf << f.rdbuf();
    nlohmann::json overlayJ;
    try { overlayJ = nlohmann::json::parse(buf.str()); }
    catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("overlay parse error: ") + e.what());
    }
    // Back-compat: the adaptive-link failsafe key was renamed safe -> failsafe.
    // Migrate a legacy overlay so a deployed drone's failsafe is preserved.
    if (overlayJ.is_object() && overlayJ.contains("dynamicLink") &&
        overlayJ["dynamicLink"].is_object()) {
        auto& dlj = overlayJ["dynamicLink"];
        if (dlj.contains("safe") && !dlj.contains("failsafe")) {
            dlj["failsafe"] = dlj["safe"];
            dlj.erase("safe");
        }
    }
    auto merged = deepMergeJson(baseJ, overlayJ);
    try { return merged.get<Config>(); }
    catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("merged schema: ") + e.what());
    }
}

nlohmann::json computeOverlay(const nlohmann::json& defaults,
                               const nlohmann::json& effective) {
    if (!defaults.is_object() || !effective.is_object()) {
        return (defaults == effective) ? nlohmann::json::object() : effective;
    }
    nlohmann::json diff = nlohmann::json::object();
    for (auto it = effective.begin(); it != effective.end(); ++it) {
        if (!defaults.contains(it.key())) {
            diff[it.key()] = it.value();
        } else if (defaults[it.key()] != it.value()) {
            if (defaults[it.key()].is_object() && it.value().is_object()) {
                auto sub = computeOverlay(defaults[it.key()], it.value());
                if (!sub.empty()) diff[it.key()] = sub;
            } else {
                diff[it.key()] = it.value();
            }
        }
    }
    return diff;
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
