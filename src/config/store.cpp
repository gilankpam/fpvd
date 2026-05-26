#include "config/store.hpp"
#include <fstream>
#include <sstream>

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
    auto merged = deepMergeJson(baseJ, overlayJ);
    try { return merged.get<Config>(); }
    catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("merged schema: ") + e.what());
    }
}

} // namespace fpvd
