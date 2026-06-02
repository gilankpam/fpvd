#pragma once
#include <cstdint>
#include <optional>
#include <string>

namespace fpvd::dynlink {

// Sets NIC txpower via `iw dev <iface> set txpower fixed <dBm*100>` (posix_spawnp).
// Diff-based: only runs iw when the dBm value changes. Port of dl_backend_radio.
class RadioTxpower {
public:
    explicit RadioTxpower(std::string iface) : iface_(std::move(iface)) {}
    void setIface(std::string iface) { iface_ = std::move(iface); current_.reset(); }
    int apply(int8_t dBm);      // 0 ok/no-op, -1 iw failure
    int applySafe(int8_t dBm);  // unconditional run (watchdog fallback)
private:
    int runIw(int8_t dBm);      // port run_iw
    std::string iface_;
    std::optional<int8_t> current_{};
};

} // namespace fpvd::dynlink
