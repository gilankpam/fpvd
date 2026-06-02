#include "translate/telemetry.hpp"

namespace fpvd {

std::vector<std::string> telemetryArgs(const Config& c) {
    if (c.telemetry.router == "none") return {};
    const std::string serialDev = "/dev/" + c.telemetry.serial;
    const std::string baud = std::to_string(c.telemetry.baud);
    if (c.telemetry.router == "msposd") {
        return {
            "/usr/bin/msposd",
            "-b", baud,
            "-c", "8",
            "-r", std::to_string(c.telemetry.osdFps),
            "-m", serialDev,
            "-o", "127.0.0.1:14551",
            "-z", c.video.resolution
        };
    }
    // mavfwd
    return {
        "/usr/bin/mavfwd",
        "-b", baud,
        "-c", "8",
        "-p", "100",
        "-a", "15",
        "-t",
        "-m", serialDev,
        "-o", "127.0.0.1:14551",
        "-i", "127.0.0.1:14550"
    };
}

} // namespace fpvd
