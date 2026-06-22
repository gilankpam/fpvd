#pragma once
namespace fpvd {
// Fixed, observe-only probe link (one FEC-off wfb_tx tracking current+1).
// Both GS and drone must agree on kProbeRadioPort.
constexpr int kProbeRadioPort = 50;     // wfb radio_port (matches GS)
constexpr int kProbeControlPort = 8001; // wfb_tx -C control port (video uses 8000)
constexpr int kProbeFeedPort = 6700;    // wfb_tx -u feed port (feeder -> tx)
constexpr int kProbePps = 25;           // feeder packets/sec
constexpr int kProbePacketBytes = 1400; // feeder datagram size (mirror video MTU)
constexpr int kProbeMcsCeiling = 7;     // hardware MCS ceiling; clamp current+1 here
} // namespace fpvd
