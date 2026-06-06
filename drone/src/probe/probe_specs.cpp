#include "probe/probe_specs.hpp"
#include "probe/probe_constants.hpp"
#include "link_width.hpp"
#include <algorithm>

namespace fpvd {

std::vector<SupervisedSpec> buildProbeSpecs(const Config& c,
                                            const std::string& iface,
                                            const std::string& key,
                                            const std::string& feederPath,
                                            int mcs) {
    std::vector<SupervisedSpec> out;
    const int rung = std::clamp(mcs, 0, kProbeMcsCeiling);

    SupervisedSpec tx{};
    tx.name = "probe-tx";
    tx.argv = {
        "/usr/bin/wfb_tx", "-K", key,
        "-M", std::to_string(rung),
        "-B", std::to_string(modulationWidth(c.link.width)),
        "-S", c.link.stbc ? "1" : "0",
        "-L", c.link.ldpc ? "1" : "0",
        "-k", "1", "-n", "1",
        "-C", std::to_string(kProbeControlPort),
        "-i", std::to_string(c.link.linkId),
        "-p", std::to_string(kProbeRadioPort),
        "-u", std::to_string(kProbeFeedPort),
        iface,
    };
    tx.restart = RestartPolicy::Always;
    out.push_back(std::move(tx));

    SupervisedSpec fd{};
    fd.name = "probe-feed";
    fd.argv = {feederPath, std::to_string(kProbeFeedPort),
               std::to_string(kProbePps), std::to_string(kProbePacketBytes)};
    fd.restart = RestartPolicy::Always;
    fd.startAfter = {"probe-tx"};
    out.push_back(std::move(fd));
    return out;
}

} // namespace fpvd
