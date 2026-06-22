#!/usr/bin/env python3
"""Verify every OSD glyph codepoint resolves in the shipped Nerd Font.

Usage:
  python3 drone/tools/osd_glyphs_verify.py deploy/drone/assets/UbuntuMono-Regular.ttf

Exits non-zero and lists misses if any codepoint maps to .notdef / is absent.
Keep CODEPOINTS in sync with drone/src/osd/osd_constants.hpp.
Requires: pip install fonttools
"""

import sys
from fontTools.ttLib import TTFont

CODEPOINTS = {
    "kGlyphSignal1": 0xF091F,
    "kGlyphSignal2": 0xF0920,
    "kGlyphSignal3": 0xF0921,
    "kGlyphSpeed": 0xF04C5,
    "kGlyphShield": 0xF0498,
    "kGlyphFlash": 0xF0241,
    "kGlyphAntenna": 0xF0E11,
    "kGlyphRefresh": 0xF0450,
    "kGlyphFilm": 0xF024A,
    "kGlyphThermo": 0xF050F,
    "kGlyphWifi": 0xF05A9,
    "kGlyphCpu": 0xF035B,
    "degree(U+00B0)": 0x00B0,
}


def main():
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    cmap = TTFont(sys.argv[1]).getBestCmap()
    misses = []
    for name, cp in CODEPOINTS.items():
        g = cmap.get(cp)
        if not g or g == ".notdef":
            misses.append((name, cp))
    if misses:
        print(
            "MISSING glyphs (fix codepoint via https://www.nerdfonts.com/cheat-sheet,"
        )
        print("update BOTH this file and drone/src/osd/osd_constants.hpp):")
        for name, cp in misses:
            print(f"  {name}: U+{cp:05X}")
        return 1
    print(f"OK: all {len(CODEPOINTS)} codepoints present in {sys.argv[1]}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
