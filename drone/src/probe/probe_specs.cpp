#include "probe/probe_specs.hpp"
#include "link_width.hpp"

namespace fpvd {

std::vector<SupervisedSpec> buildProbeSpecs(const Config& c,
                                            const std::string& iface,
                                            const std::string& key,
                                            const std::string& feederPath) {
    std::vector<SupervisedSpec> out;
    if (!c.probe.enabled) return out;
    const auto& p = c.probe;
    for (size_t i = 0; i < p.mcsList.size(); ++i) {
        const int mcs  = p.mcsList[i];
        const int port = p.basePort + static_cast<int>(i);
        const int feed = p.baseFeedPort + static_cast<int>(i);
        const std::string txName = "probe-tx-mcs" + std::to_string(mcs);
        const std::string fdName = "probe-feed-mcs" + std::to_string(mcs);

        SupervisedSpec tx{};
        tx.name = txName;
        tx.argv = {
            "/usr/bin/wfb_tx", "-K", key,
            "-M", std::to_string(mcs),
            "-B", std::to_string(modulationWidth(c.link.width)),
            "-S", c.link.stbc ? "1" : "0",
            "-L", c.link.ldpc ? "1" : "0",
            "-k", "1", "-n", "1",
            "-i", std::to_string(c.link.linkId),
            "-p", std::to_string(port),
            "-u", std::to_string(feed),
            iface,
        };
        tx.restart = RestartPolicy::Always;
        out.push_back(std::move(tx));

        SupervisedSpec fd{};
        fd.name = fdName;
        fd.argv = {feederPath, std::to_string(feed),
                   std::to_string(p.pps), std::to_string(p.packetBytes)};
        fd.restart = RestartPolicy::Always;
        fd.startAfter = {txName};
        out.push_back(std::move(fd));
    }
    return out;
}

} // namespace fpvd
