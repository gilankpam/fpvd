#include "translate/waybeam.hpp"
#include <cstdio>
#include <string>

namespace fpvd {

nlohmann::json toWaybeamJson(const Config& c) {
    // Keep the fpvd-managed field set in sync with waybeamConfigDiff().
    return {
        {"system", {{"webPort", 80}, {"overclockLevel", 1}, {"verbose", false}}},
        {"sensor",
         {{"index", -1},
          {"mode", -1},
          {"unlockEnabled", true},
          {"unlockCmd", 35},
          {"unlockReg", 12298},
          {"unlockValue", 128},
          {"unlockDir", 0}}},
        {"isp",
         {{"sensorBin", c.video.sensorBin},
          {"legacyAe", true},
          {"aeMode", "native"},
          {"aeFps", 15},
          {"gainMax", 0},
          {"awbMode", "auto"},
          {"awbCt", 5500},
          {"keepAspect", true}}},
        {"image", {{"mirror", c.image.mirror}, {"flip", c.image.flip}, {"rotate", c.image.rotate}}},
        {"video0",
         {{"rcMode", c.video.rcMode},
          {"fps", c.video.fps},
          {"size", c.video.resolution},
          {"bitrate", c.video.bitrate},
          {"gopSize", c.video.gopSize},
          // waybeam ignores gopSize when resilience != "off" (the preset owns
          // intra-refresh + GOP). See waybeamConfigDiff() — resilience is a
          // RESTART-class field.
          {"resilience", c.video.resilience},
          {"qpDelta", c.video.qpDelta},
          {"frameLost", true},
          {"sceneThreshold", 0},
          {"sceneHoldoff", 2},
          {"intraRefreshMode", "off"},
          {"intraRefreshLines", 0},
          {"intraRefreshQp", 0},
          {"zoomPct", 0.0},
          {"zoomX", 0.5},
          {"zoomY", 0.5}}},
        {"outgoing",
         {{"enabled", true},
          {"server", "unix://venc_wfb"},
          {"streamMode", "rtp"},
          // 1400 + 12B RTP must stay <= swfec's SWFEC_MAX_INPUT (static_assert
          // in the wfb-ng fork's tx.hpp) — oversize datagrams are silently
          // dropped in swfec mode.
          {"maxPayloadSize", 1400},
          {"connectedUdp", true},
          {"audioPort", 5601},
          {"sidecarPort", 5602}}},
        {"fpv",
         {{"roiEnabled", c.video.roi.enabled},
          {"roiQp", c.video.roi.qp},
          {"roiSteps", c.video.roi.steps},
          {"roiCenter", c.video.roi.center},
          {"noiseLevel", 0}}},
        {"audio",
         {{"enabled", false},
          {"sampleRate", 48000},
          {"channels", 1},
          {"codec", "opus"},
          {"volume", 80},
          {"mute", false}}},
        {"imu",
         {{"enabled", false},
          {"i2cDevice", "/dev/i2c-1"},
          {"i2cAddr", "0x68"},
          {"sampleRateHz", 200},
          {"gyroRangeDps", 1000},
          {"calFile", "/etc/imu.cal"},
          {"calSamples", 400}}},
        {"record",
         {{"enabled", c.recording.enabled},
          {"format", c.recording.format},
          {"mode", c.recording.mode},
          {"maxSeconds", c.recording.maxSeconds},
          {"maxMB", c.recording.maxMB},
          {"bitrate", 0},
          {"fps", 0},
          {"gopSize", 0.0},
          {"server", ""}}},
        {"debug", {{"showOsd", false}}}};
}

static std::string fmtBool(bool b) { return b ? "true" : "false"; }

// Formats a double for a waybeam query value. Assumes the C locale (fpvd never
// calls setlocale) so the separator is '.'; waybeam parses these fields as
// floats, so an integral value renders without a decimal (e.g. 2.0 -> "2").
static std::string fmtDouble(double d) {
    char buf[32];
    std::snprintf(buf, sizeof(buf), "%g", d);
    return buf;
}

WaybeamFieldDiff waybeamConfigDiff(const Config& a, const Config& b, bool dlEnabled) {
    // Keep the fpvd-managed field set in sync with toWaybeamJson().
    WaybeamFieldDiff d;
    const auto& va = a.video;
    const auto& vb = b.video;

    // LIVE — dynamic-link-owned fields are skipped while DL is enabled.
    if (!dlEnabled) {
        if (va.bitrate != vb.bitrate)
            d.live["video0.bitrate"] = std::to_string(vb.bitrate);
        if (va.fps != vb.fps)
            d.live["video0.fps"] = std::to_string(vb.fps);
        if (va.qpDelta != vb.qpDelta)
            d.live["video0.qp_delta"] = std::to_string(vb.qpDelta);
        if (va.roi.enabled != vb.roi.enabled)
            d.live["fpv.roi_enabled"] = fmtBool(vb.roi.enabled);
        if (va.roi.qp != vb.roi.qp)
            d.live["fpv.roi_qp"] = std::to_string(vb.roi.qp);
        if (va.roi.steps != vb.roi.steps)
            d.live["fpv.roi_steps"] = std::to_string(vb.roi.steps);
        if (va.roi.center != vb.roi.center)
            d.live["fpv.roi_center"] = fmtDouble(vb.roi.center);
    }
    // gopSize is LIVE but NOT dynamic-link-owned (the controller never writes
    // gop), so it is pushed regardless of DL state.
    if (va.gopSize != vb.gopSize)
        d.live["video0.gop_size"] = fmtDouble(vb.gopSize);

    // RESTART
    if (va.resolution != vb.resolution)
        d.restart["video0.size"] = vb.resolution;
    if (va.rcMode != vb.rcMode)
        d.restart["video0.rc_mode"] = vb.rcMode;
    // sensorBin reconfigures the sensor readout — needs a pipeline reinit, so it
    // is restart-class. Not dynamic-link-owned (the controller never writes it).
    if (va.sensorBin != vb.sensorBin)
        d.restart["isp.sensor_bin"] = vb.sensorBin;
    // resilience is a named encoder preset (intra-refresh + GOP). waybeam
    // documents it as reboot-required; we apply it restart-class by bouncing
    // the waybeam process — whether that suffices vs a full reboot is an open
    // bench-verification risk (see design spec). Not dynamic-link-owned — the
    // controller never writes it.
    if (va.resilience != vb.resilience)
        d.restart["video0.resilience"] = vb.resilience;

    const auto& ia = a.image;
    const auto& ib = b.image;
    if (ia.mirror != ib.mirror)
        d.restart["image.mirror"] = fmtBool(ib.mirror);
    if (ia.flip != ib.flip)
        d.restart["image.flip"] = fmtBool(ib.flip);
    if (ia.rotate != ib.rotate)
        d.restart["image.rotate"] = std::to_string(ib.rotate);

    const auto& ra = a.recording;
    const auto& rb = b.recording;
    if (ra.enabled != rb.enabled)
        d.restart["record.enabled"] = fmtBool(rb.enabled);
    if (ra.format != rb.format)
        d.restart["record.format"] = rb.format;
    if (ra.mode != rb.mode)
        d.restart["record.mode"] = rb.mode;
    if (ra.maxSeconds != rb.maxSeconds)
        d.restart["record.max_seconds"] = std::to_string(rb.maxSeconds);
    if (ra.maxMB != rb.maxMB)
        d.restart["record.max_mb"] = std::to_string(rb.maxMB);

    // video.codec intentionally not mapped (retired; pinned to h265).
    return d;
}

} // namespace fpvd
