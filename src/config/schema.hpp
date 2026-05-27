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

struct Fec { int k{8}; int n{12}; };
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Fec, k, n)

struct Link {
    int channel{161};
    int width{20};
    int txpower{1};
    int mcs{2};
    Fec fec{};
    bool stbc{false};
    bool ldpc{false};
    long linkId{7669206};
    int mtu{1500};
    std::optional<std::string> wlanAdapter{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Link, channel, width, txpower, mcs, fec,
                                   stbc, ldpc, linkId, mtu, wlanAdapter)

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
    int qpDelta{-4};
    Roi roi{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Video, codec, resolution, fps, bitrate,
                                   rcMode, gopSize, qpDelta, roi)

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
    std::string dir{"/mnt/mmcblk0p1"};
    std::string format{"ts"};
    std::string mode{"mirror"};
    int maxSeconds{300};
    int maxMB{500};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Recording, enabled, dir, format, mode,
                                   maxSeconds, maxMB)

struct Snapshot { bool enabled{true}; int quality{80}; };
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE(Snapshot, enabled, quality)

struct DynamicLinkSafe {
    int mcs{1};
    int k{8};
    int n{12};
    int depth{1};
    int bandwidth{20};
    int txPowerDbm{20};
    int bitrateKbps{2000};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkSafe, mcs, k, n,
                                               depth, bandwidth, txPowerDbm,
                                               bitrateKbps)

struct DynamicLinkOsd {
    bool enabled{true};
    bool debugLatency{false};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkOsd, enabled,
                                               debugLatency)

struct DynamicLinkRoiQp {
    int thresholdKbps{6000};
    int lowAnchorKbps{2000};
    int floor{-24};
    int step{3};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLinkRoiQp,
                                               thresholdKbps, lowAnchorKbps,
                                               floor, step)

struct DynamicLink {
    bool enabled{false};
    int healthTimeoutMs{10000};
    bool interleavingSupported{true};
    int minIdrIntervalMs{500};
    int applyStaggerMs{50};
    int applySubPaceMs{5};
    bool mavlinkEnable{true};
    DynamicLinkOsd osd{};
    DynamicLinkRoiQp roiQp{};
    DynamicLinkSafe safe{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(DynamicLink, enabled,
                                               healthTimeoutMs,
                                               interleavingSupported,
                                               minIdrIntervalMs, applyStaggerMs,
                                               applySubPaceMs, mavlinkEnable,
                                               osd, roiQp, safe)

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
    Snapshot snapshot{};
    DynamicLink dynamicLink{};
    std::map<std::string, Service> services{};
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Config, link, video, image,
                                               telemetry, recording, snapshot,
                                               dynamicLink, services)

} // namespace fpvd
