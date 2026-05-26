#include "config/store.hpp"
#include <fstream>
#include <sstream>

namespace fpvd {

Config loadDefaults(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        throw StoreError("failed to open defaults file: " + path);
    }
    std::stringstream buf;
    buf << f.rdbuf();
    try {
        return nlohmann::json::parse(buf.str()).get<Config>();
    } catch (const nlohmann::json::exception& e) {
        throw StoreError(std::string("defaults parse error: ") + e.what());
    }
}

} // namespace fpvd
