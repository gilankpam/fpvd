#include "dynlink/runtime_config.hpp"
#include "config/schema.hpp"
#include "probe/probe_constants.hpp"
#include "link_width.hpp"

namespace fpvd::dynlink {

DlRuntimeConfig buildDlSnapshot(const Config& c, const std::string& iface)
{
    const auto& dl = c.dynamicLink;

    DlRuntimeConfig s{};

    s.healthTimeoutMs      = static_cast<uint32_t>(dl.healthTimeoutMs);
    s.applyStaggerMs       = static_cast<uint32_t>(dl.applyStaggerMs);
    s.applySubPaceMs       = static_cast<uint32_t>(dl.applySubPaceMs);
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
        static_cast<uint16_t>(dl.safe.bitrateKbps),
    };
    s.safe.overheadPct = static_cast<uint8_t>(dl.safe.overheadPct);
    s.safe.deadlineMs  = static_cast<uint8_t>(dl.safe.deadlineMs);

    s.swfec            = (c.link.fec.mode == "swfec");
    s.swfecOverheadPct = static_cast<uint8_t>(c.link.fec.overheadPct);
    s.swfecDeadlineMs  = static_cast<uint8_t>(c.link.fec.deadlineMs);

    s.bitrate = BitrateEngineConfig{
        dl.compute.baseRedundancyRatio,
        dl.compute.blocksPerFrame,
        dl.compute.kMin,
        dl.compute.kMax,
        dl.compute.minBitrateKbps,
        dl.compute.maxBitrateKbps,
        c.link.mtu,
        c.video.fps,
    };

    s.stbc          = c.link.stbc;
    s.ldpc          = c.link.ldpc;
    s.linkBandwidth = static_cast<uint8_t>(modulationWidth(c.link.width));
    s.probeCtlPort  = static_cast<uint16_t>(kProbeControlPort);
    s.probeMcsCeiling = kProbeMcsCeiling;
    s.iface         = iface;

    return s;
}

} // namespace fpvd::dynlink
