#include "translate/dynamic_link.hpp"
#include <string>

namespace fpvd {

static const char* b01(bool v) { return v ? "1" : "0"; }

std::vector<std::string> dynamicLinkArgs(const Config& c,
                                          const std::string& iface) {
    using std::to_string;
    const auto& dl = c.dynamicLink;
    std::vector<std::string> a = {
        "/usr/bin/dl-applier",

        // listen endpoint for GS decision packets (over wfb-ng tunnel)
        "--listen-addr", "0.0.0.0",
        "--listen-port", "5800",

        // wfb_tx control socket — pinned to match wfb_video_tx's -C 8000
        "--wfb-tx-ctrl-addr", "127.0.0.1",
        "--wfb-tx-ctrl-port", "8000",

        // encoder
        "--encoder-kind", "waybeam",
        "--encoder-host", "127.0.0.1",
        "--encoder-port", "80",

        // IDR-token listener (PixelPilot_rk)
        "--idr-listen-addr", "0.0.0.0",
        "--idr-listen-port", "11223",

        // MAVLink — port pinned to wfb_tlm_tx's listen socket (14551)
        "--mavlink-addr", "127.0.0.1",
        "--mavlink-port", "14551",

        // OSD output path
        "--osd-msg-path", "/tmp/MSPOSD.msg",
        "--osd-update-interval-ms", "1000",

        // schema-driven scalars and toggles
        "--health-timeout-ms", to_string(dl.healthTimeoutMs),
        "--interleaving-supported", b01(dl.interleavingSupported),
        "--debug-enable", b01(dl.debug),
        "--min-idr-interval-ms", to_string(dl.minIdrIntervalMs),
        "--apply-stagger-ms", to_string(dl.applyStaggerMs),
        "--apply-sub-pace-ms", to_string(dl.applySubPaceMs),
        "--mavlink-enable", b01(dl.mavlinkEnable),
        "--osd-enable", b01(dl.osd.enabled),
        "--osd-debug-latency", b01(dl.osd.debugLatency),

        // ROI-QP curve
        "--roi-qp-threshold-kbps", to_string(dl.roiQp.thresholdKbps),
        "--roi-qp-low-anchor-kbps", to_string(dl.roiQp.lowAnchorKbps),
        "--roi-qp-floor", to_string(dl.roiQp.floor),
        "--roi-qp-step", to_string(dl.roiQp.step),

        // per-airframe safe defaults (failsafe-1 fallback)
        "--safe-mcs", to_string(dl.safe.mcs),
        "--safe-k", to_string(dl.safe.k),
        "--safe-n", to_string(dl.safe.n),
        "--safe-depth", to_string(dl.safe.depth),
        "--safe-bandwidth", to_string(dl.safe.bandwidth),
        "--safe-tx-power-dBm", to_string(dl.safe.txPowerDbm),
        "--safe-bitrate-kbps", to_string(dl.safe.bitrateKbps),

        // derived from existing fpvd schema
        "--hello-mtu-bytes", to_string(c.link.mtu),
        "--hello-fps", to_string(c.video.fps),

        // radio device picked by radio-up.sh
        "--wlan-dev", iface,
    };
    return a;
}

} // namespace fpvd
