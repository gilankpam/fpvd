#pragma once

namespace fpvd::osd {

// The msposd overlay message file the drone writes. msposd (the telemetry
// router child) reads it, overlays it on the video, and substitutes its
// &-placeholders (&B bitrate+fps, &T/&W temps, &C cpu%) at render time. A fixed
// path, not operator config — the daemon owns the single writer.
constexpr const char* kOsdMsgPath = "/tmp/MSPOSD.msg";

} // namespace fpvd::osd
