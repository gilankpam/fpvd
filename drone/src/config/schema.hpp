#pragma once
#include <nlohmann/json.hpp>
#include <map>
#include <optional>
#include <string>
#include <vector>

// nlohmann/json (v3.11) does not provide a built-in adl_serializer for
// std::optional<T>. Add one in the nlohmann namespace so JSON `null`
// round-trips with std::nullopt, and a present value round-trips through
// T's own serializer. Without this, NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE
// fails to compile for any struct field of type std::optional<T>.
namespace nlohmann {
template <typename T>
struct adl_serializer<std::optional<T>> {
    static void to_json(json& j, const std::optional<T>& opt) {
        if (opt.has_value()) {
            j = *opt;
        } else {
            j = nullptr;
        }
    }
    static void from_json(const json& j, std::optional<T>& opt) {
        if (j.is_null()) {
            opt = std::nullopt;
        } else {
            opt = j.get<T>();
        }
    }
};
} // namespace nlohmann

namespace fpvd {

struct Fec {
    std::string mode{"swfec"};   // "rs" | "swfec" — mode flip restarts wfb_tx (-z is constructor-time)
    int k{8};                 // rs-mode block geometry: data fragments per block
    int n{12};                //   ...and total fragments (k data + n-k parity)
    int overheadPct{50};      // swfec-mode repair budget, 0..255 (uint8 on the control wire)
    int deadlineMs{30};       // swfec-mode recovery window, 1..255
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Fec, mode, k, n, overheadPct, deadlineMs)

struct Beamforming {
    bool enabled{false};
    std::string remoteMac{};   // ground-station eFuse MAC, required when enabled
    int ackTimeout{255};       // 33..255 us
    int intervalMs{100};       // sounding cadence
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Beamforming, enabled, remoteMac,
                                                ackTimeout, intervalMs)

struct Link {
    int channel{132};
    int width{20};
    int txPowerDbm{20};
    int mcs{2};
    Fec fec{};
    bool stbc{true};
    bool ldpc{true};
    long linkId{7669206};
    int mtu{1500};
    std::optional<std::string> wlanAdapter{};
    Beamforming beamforming{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Link, channel, width, txPowerDbm,
                                                mcs, fec, stbc, ldpc, linkId,
                                                mtu, wlanAdapter, beamforming)

struct Roi {
    bool enabled{true};
    int qp{0};
    double center{0.4};
    int steps{2};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Roi, enabled, qp, center, steps)

struct Video {
    std::string codec{"h265"};
    std::string resolution{"1920x1080"};
    int fps{60};
    int bitrate{8192};
    std::string rcMode{"cbr"};
    double gopSize{1.0};
    // waybeam error-resilience preset. Derives intra-refresh, the SVC-T
    // reference pyramid, and GOP length inside waybeam; gopSize is honored
    // only when resilience == "off". Validated against a fixed preset set in
    // config/validate.cpp.
    std::string resilience{"off"};
    int qpDelta{-4};
    std::string sensorBin{""};   // sensor binning mode (empty = sensor default)
    Roi roi{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Video, codec, resolution, fps, bitrate,
                                   rcMode, gopSize, resilience, qpDelta,
                                   sensorBin, roi)

struct Image { bool mirror{false}; bool flip{false}; int rotate{0}; };
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Image, mirror, flip, rotate)

struct Telemetry {
    std::string router{"msposd"};
    std::string serial{"ttyS2"};
    int osdFps{20};
    int baud{115200};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Telemetry, router, serial, osdFps, baud)

struct Recording {
    bool enabled{false};
    std::string format{"ts"};
    std::string mode{"mirror"};
    int maxSeconds{300};
    int maxMB{500};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Recording, enabled, format, mode,
                                   maxSeconds, maxMB)

struct DynamicLinkSafe {
    int mcs{1};
    int k{8};
    int n{12};
    int overheadPct{100};   // swfec-mode safe recovery: more repair at the low rung (0..255, uint8 wire)
    int deadlineMs{30};     // 1..255 (uint8 wire)
    int bitrateKbps{2000};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkSafe, mcs, k, n,
                                               overheadPct, deadlineMs,
                                               bitrateKbps)

struct DynamicLinkRoiQp {
    int thresholdKbps{6000};
    int lowAnchorKbps{2000};
    int floor{-24};
    int step{3};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkRoiQp,
                                               thresholdKbps, lowAnchorKbps,
                                               floor, step)

struct DynamicLinkCompute {
    int    minBitrateKbps{1000};
    int    maxBitrateKbps{24000};
    double baseRedundancyRatio{0.5};   // n/k = 1 + ratio (= 8/12 data fraction)
    double blocksPerFrame{2.0};
    int    kMin{2};
    int    kMax{50};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkCompute,
                                                minBitrateKbps, maxBitrateKbps,
                                                baseRedundancyRatio, blocksPerFrame,
                                                kMin, kMax)

struct DynamicLink {
    bool enabled{false};
    int healthTimeoutMs{10000};
    int applyStaggerMs{50};
    int applySubPaceMs{5};
    DynamicLinkRoiQp  roiQp{};
    DynamicLinkSafe   safe{};
    DynamicLinkCompute compute{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLink, enabled,
                                               healthTimeoutMs, applyStaggerMs,
                                               applySubPaceMs,
                                               roiQp, safe, compute)

// OSD overlay (msposd message file). Top-level: the OSD is rendered whether or
// not the dynamic link is enabled, so its enable flag lives outside dynamicLink.
struct Osd {
    bool enabled{true};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Osd, enabled)

struct Service {
    bool enabled{true};
    std::string exec{};
    std::vector<std::string> args{};
    std::map<std::string, std::string> env{};
    std::vector<std::string> startAfter{};
    std::string restart{"always"};  // always | on-failure | never
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Service, enabled, exec, args, env,
                                   startAfter, restart)

struct Config {
    Link link{};
    Video video{};
    Image image{};
    Telemetry telemetry{};
    Recording recording{};
    Osd osd{};
    DynamicLink dynamicLink{};
    std::map<std::string, Service> services{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Config, link, video, image,
                                               telemetry, recording, osd,
                                               dynamicLink, services)

} // namespace fpvd
