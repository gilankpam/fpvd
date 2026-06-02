#pragma once
#include <cstdint>
#include <map>
#include <string>

namespace fpvd {

// Thin transport for waybeam's HTTP API (cpp-httplib). Stateless: a fresh
// httplib::Client is created per call, so a single instance is safe to share
// across threads. Shared by the dynamic-link EncoderClient and the daemon's
// config-apply path.
class WaybeamClient {
public:
    WaybeamClient(std::string host, uint16_t port,
                  int connectTimeoutMs = 300, int readTimeoutMs = 500);

    // GET /api/v1/set?k1=v1&k2=v2 … (keys/values percent-encoded). true on 2xx.
    // An empty map is a no-op that returns true.
    bool setFields(const std::map<std::string, std::string>& fields);

    // GET an already-formed path (e.g. "/request/idr"). true on 2xx.
    bool get(const std::string& path);

private:
    std::string host_;
    uint16_t    port_;
    int         connectTimeoutMs_;
    int         readTimeoutMs_;
};

} // namespace fpvd
