#include "waybeam/client.hpp"
#include <cctype>
#include <httplib.h>

namespace fpvd {

static std::string urlEncode(const std::string& s) {
    static const char* hex = "0123456789ABCDEF";
    std::string out;
    out.reserve(s.size());
    for (unsigned char c : s) {
        if (std::isalnum(c) || c == '-' || c == '_' || c == '.' || c == '~') {
            out.push_back(static_cast<char>(c));
        } else {
            out.push_back('%');
            out.push_back(hex[c >> 4]);
            out.push_back(hex[c & 0x0F]);
        }
    }
    return out;
}

WaybeamClient::WaybeamClient(std::string host, uint16_t port, int connectTimeoutMs,
                             int readTimeoutMs)
    : host_(std::move(host)), port_(port), connectTimeoutMs_(connectTimeoutMs),
      readTimeoutMs_(readTimeoutMs) {}

bool WaybeamClient::get(const std::string& path) {
    httplib::Client cli(host_, static_cast<int>(port_));
    cli.set_connection_timeout(0, connectTimeoutMs_ * 1000); // µs
    cli.set_read_timeout(0, readTimeoutMs_ * 1000);
    auto res = cli.Get(path.c_str());
    return res && res->status / 100 == 2;
}

bool WaybeamClient::setFields(const std::map<std::string, std::string>& fields) {
    if (fields.empty())
        return true;
    std::string path = "/api/v1/set?";
    bool first = true;
    for (const auto& [k, v] : fields) {
        if (!first)
            path.push_back('&');
        first = false;
        path += urlEncode(k);
        path.push_back('=');
        path += urlEncode(v);
    }
    return get(path);
}

} // namespace fpvd
