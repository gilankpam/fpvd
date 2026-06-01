#include "dynlink/runtime_config.hpp"
#include "config/schema.hpp"

namespace fpvd::dynlink {

DlRuntimeConfig buildDlSnapshot(const Config& c, const std::string& iface)
{
    const auto& dl = c.dynamicLink;

    DlRuntimeConfig s{};

    s.healthTimeoutMs      = static_cast<uint32_t>(dl.healthTimeoutMs);
    s.minIdrIntervalMs     = static_cast<uint32_t>(dl.minIdrIntervalMs);
    s.applyStaggerMs       = static_cast<uint32_t>(dl.applyStaggerMs);
    s.applySubPaceMs       = static_cast<uint32_t>(dl.applySubPaceMs);
    s.interleavingSupported = dl.interleavingSupported;
    s.osdEnabled           = dl.osd.enabled;
    s.osdDebugLatency      = dl.osd.debugLatency;
    s.debug                = false;   // no debug field in schema DynamicLink yet

    s.roiQp = RoiCurve{
        static_cast<uint16_t>(dl.roiQp.thresholdKbps),
        static_cast<uint16_t>(dl.roiQp.lowAnchorKbps),
        static_cast<int8_t> (dl.roiQp.floor),
        static_cast<uint8_t> (dl.roiQp.step),
    };

    s.safe = SafeDefaults{
        static_cast<uint8_t> (dl.safe.mcs),
        static_cast<uint8_t> (dl.safe.k),
        static_cast<uint8_t> (dl.safe.n),
        static_cast<uint8_t> (dl.safe.depth),
        static_cast<uint8_t> (dl.safe.bandwidth),
        static_cast<int8_t>  (dl.safe.txPowerDbm),
        static_cast<uint16_t>(dl.safe.bitrateKbps),
    };

    s.helloMtuBytes = static_cast<uint16_t>(c.link.mtu);
    s.helloFps      = static_cast<uint16_t>(c.video.fps);
    s.iface         = iface;

    return s;
}

} // namespace fpvd::dynlink
