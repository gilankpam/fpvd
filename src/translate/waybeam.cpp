#include "translate/waybeam.hpp"

namespace fpvd {

nlohmann::json toWaybeamJson(const Config& c) {
    return {
        {"system", {{"webPort", 80}, {"overclockLevel", 1}, {"verbose", false}}},
        {"sensor", {
            {"index", -1}, {"mode", -1},
            {"unlockEnabled", true}, {"unlockCmd", 35},
            {"unlockReg", 12298}, {"unlockValue", 128}, {"unlockDir", 0}
        }},
        {"isp", {
            {"sensorBin", ""}, {"legacyAe", true}, {"aeMode", "native"},
            {"aeFps", 15}, {"gainMax", 0}, {"awbMode", "auto"},
            {"awbCt", 5500}, {"keepAspect", true}
        }},
        {"image", {
            {"mirror", c.image.mirror},
            {"flip", c.image.flip},
            {"rotate", c.image.rotate}
        }},
        {"video0", {
            {"codec", c.video.codec},
            {"rcMode", c.video.rcMode},
            {"fps", c.video.fps},
            {"size", c.video.resolution},
            {"bitrate", c.video.bitrate},
            {"gopSize", c.video.gopSize},
            {"qpDelta", c.video.qpDelta},
            {"frameLost", true},
            {"sceneThreshold", 0},
            {"sceneHoldoff", 2},
            {"intraRefreshMode", "off"},
            {"intraRefreshLines", 0},
            {"intraRefreshQp", 0},
            {"zoomPct", 0.0},
            {"zoomX", 0.5},
            {"zoomY", 0.5}
        }},
        {"outgoing", {
            {"enabled", true},
            {"server", "unix://venc_wfb"},
            {"streamMode", "rtp"},
            {"maxPayloadSize", 1400},
            {"connectedUdp", true},
            {"audioPort", 5601},
            {"sidecarPort", 5602}
        }},
        {"fpv", {
            {"roiEnabled", c.video.roi.enabled},
            {"roiQp", c.video.roi.qp},
            {"roiSteps", c.video.roi.steps},
            {"roiCenter", c.video.roi.center},
            {"noiseLevel", 0}
        }},
        {"audio", {
            {"enabled", false}, {"sampleRate", 48000}, {"channels", 1},
            {"codec", "opus"}, {"volume", 80}, {"mute", false}
        }},
        {"imu", {
            {"enabled", false}, {"i2cDevice", "/dev/i2c-1"},
            {"i2cAddr", "0x68"}, {"sampleRateHz", 200},
            {"gyroRangeDps", 1000}, {"calFile", "/etc/imu.cal"},
            {"calSamples", 400}
        }},
        {"record", {
            {"enabled", c.recording.enabled},
            {"format", c.recording.format},
            {"mode", c.recording.mode},
            {"maxSeconds", c.recording.maxSeconds},
            {"maxMB", c.recording.maxMB},
            {"bitrate", 0}, {"fps", 0}, {"gopSize", 0.0}, {"server", ""}
        }},
        {"debug", {{"showOsd", false}}}
    };
}

} // namespace fpvd
