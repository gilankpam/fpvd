#include "translate/wfb.hpp"
#include "link_width.hpp"

namespace fpvd {

static std::vector<std::string> commonTx(const Config& c, int mcs,
                                          const std::string& /*iface*/,
                                          const std::string& key) {
    return {
        "/usr/bin/wfb_tx",
        "-K", key,
        "-M", std::to_string(mcs),
        "-B", std::to_string(modulationWidth(c.link.width)),
        "-k", std::to_string(c.link.fec.k),
        "-n", std::to_string(c.link.fec.n),
        "-S", c.link.stbc ? "1" : "0",
        "-L", c.link.ldpc ? "1" : "0",
        "-i", std::to_string(c.link.linkId)
    };
}

// tun/tlm are boot-once processes with fixed, robust radiotap/FEC, decoupled
// from c.link.* (except the shared linkId). See
// docs/superpowers/specs/2026-05-30-link-hot-apply-design.md.
static std::vector<std::string> tunTlmTx(const Config& c, const std::string& key) {
    return {
        "/usr/bin/wfb_tx",
        "-K", key,
        "-M", "0",
        "-B", "20",
        "-k", "3",
        "-n", "5",
        "-S", "0",
        "-L", "0",
        "-i", std::to_string(c.link.linkId)
    };
}

std::vector<std::string> wfbArgs(const Config& c, WfbRole role,
                                  const std::string& iface,
                                  const std::string& key) {
    switch (role) {
        case WfbRole::VideoTx: {
            auto a = commonTx(c, c.link.mcs, iface, key);
            a.push_back("-U"); a.push_back("venc_wfb");
            a.push_back("-C"); a.push_back(std::to_string(kVideoControlPort));
            a.push_back("-J"); a.push_back("10");
            a.push_back("-E"); a.push_back("5000");
            a.push_back(iface);
            return a;
        }
        case WfbRole::TunRx:
            return {"/usr/bin/wfb_rx", "-K", key,
                    "-i", std::to_string(c.link.linkId),
                    "-p", "160", "-u", "5800", iface};
        case WfbRole::TunTx: {
            auto a = tunTlmTx(c, key);
            a.push_back("-p"); a.push_back("32");
            a.push_back("-u"); a.push_back("5801");
            a.push_back(iface);
            return a;
        }
        case WfbRole::TlmRx:
            return {"/usr/bin/wfb_rx", "-K", key,
                    "-i", std::to_string(c.link.linkId),
                    "-p", "144", "-u", "14550", iface};
        case WfbRole::TlmTx: {
            auto a = tunTlmTx(c, key);
            a.push_back("-p"); a.push_back("16");
            a.push_back("-u"); a.push_back("14551");
            a.push_back(iface);
            return a;
        }
    }
    return {};
}

std::vector<std::string> wfbTunArgs() {
    return {"/usr/bin/wfb_tun", "-a", "10.5.0.10/24"};
}

} // namespace fpvd
