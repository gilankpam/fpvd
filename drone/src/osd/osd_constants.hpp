#pragma once

namespace fpvd::osd {

// The msposd overlay message file the drone writes. msposd reads it, overlays
// it on the video, and substitutes its &-placeholders (&B bitrate+fps, &T/&W
// temps, &C cpu%) at render time. Fixed path; the daemon owns the single writer.
inline constexpr const char* kOsdMsgPath = "/tmp/MSPOSD.msg";

// ---- Per-line msposd directives --------------------------------------------
// Every rendered line is prefixed "&L{color}{zone}&F{size} ". msposd colors and
// sizes per line; there is no per-glyph color, so we render one indicator per
// line. Color digit = first digit of &L; zone = second digit (0 = TopLeft).
inline constexpr int kOsdFontSize = 30; // &F size (matches the prior single line)
inline constexpr int kOsdZone = 0;      // TopLeft

// msposd palette digits (see ../msposd/osd.c I4ColorIndex + dev notes).
inline constexpr int kColWhite = 0;
inline constexpr int kColRed = 2;
inline constexpr int kColGreen = 3;
inline constexpr int kColYellow = 5;
inline constexpr int kColCyan = 7;

// MCS color tiers (drone only knows MCS): mcs 0 = failsafe rung -> red,
// mcs >= kMcsGood -> green, otherwise yellow.
inline constexpr unsigned kMcsGood = 4u;

// ---- Glyphs (Nerd Fonts Material Design Icons, plane-15 PUA) ----------------
// Emitted as UTF-8 via u8 universal-character-names; the C++17 compiler encodes
// the codepoint. drone/tools/osd_glyphs_verify.py asserts each resolves in the
// shipped font (keep the two lists in sync). msposd decodes UTF-8 (incl. 4-byte)
// and rasterizes whatever glyph the font holds.
inline constexpr const char* kGlyphSignal1 = u8"\U000F091F"; // signal-cellular-1
inline constexpr const char* kGlyphSignal2 = u8"\U000F0920"; // signal-cellular-2
inline constexpr const char* kGlyphSignal3 = u8"\U000F0921"; // signal-cellular-3
inline constexpr const char* kGlyphSpeed = u8"\U000F04C5";   // speedometer (cap)
inline constexpr const char* kGlyphShield = u8"\U000F0498";  // shield (fec)
inline constexpr const char* kGlyphFlash = u8"\U000F0241";   // flash (tx power)
inline constexpr const char* kGlyphAntenna = u8"\U000F0E11"; // antenna (beamforming)
inline constexpr const char* kGlyphRefresh = u8"\U000F0450"; // refresh (idr)
inline constexpr const char* kGlyphFilm = u8"\U000F024A";    // filmstrip (video)
inline constexpr const char* kGlyphThermo = u8"\U000F050F";  // thermometer (board temp)
inline constexpr const char* kGlyphWifi = u8"\U000F05A9";    // wifi (wifi temp)
inline constexpr const char* kGlyphCpu = u8"\U000F035B";     // memory (cpu)

// Unit suffix: degree sign + C (U+00B0 is in UbuntuMono's Latin-1 set).
inline constexpr const char* kUnitDegC = u8"°C";

} // namespace fpvd::osd
