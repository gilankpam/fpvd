#!/usr/bin/env python3
"""Verify the OSD glyph codepoints resolve to the INTENDED glyphs in the shipped font.

Usage:
  python3 drone/tools/osd_glyphs_verify.py [path/to/font.ttf]
  (defaults to deploy/drone/assets/UbuntuMono-Regular.ttf)

This is stronger than a presence check: a codepoint can resolve to *some* glyph
(so it looks "present") while being the wrong picture (e.g. a handbag instead of
an antenna). We parse each kGlyph* codepoint straight out of osd_constants.hpp
and assert the font maps it to the Material-Design-Icon name written in that
line's trailing comment. The comment name IS the contract.

Requires: pip install fonttools
"""

import re
import sys
from pathlib import Path

from fontTools.ttLib import TTFont

REPO = Path(__file__).resolve().parents[2]
HEADER = REPO / "drone/src/osd/osd_constants.hpp"
DEFAULT_FONT = REPO / "deploy/drone/assets/UbuntuMono-Regular.ttf"

# Matches: kGlyphX = u8"\U000Fxxxx"; // <expected-glyph-name> ...
LINE_RE = re.compile(
    r'(kGlyph\w+)\s*=\s*u8"\\U([0-9A-Fa-f]{8})";\s*//\s*([A-Za-z0-9_-]+)'
)


def parse_header():
    """Return [(const, codepoint, expected_glyph_name), ...] from osd_constants.hpp."""
    out = []
    for line in HEADER.read_text(encoding="utf-8").splitlines():
        m = LINE_RE.search(line)
        if m:
            out.append((m.group(1), int(m.group(2), 16), m.group(3)))
    return out


def main():
    font_path = Path(sys.argv[1]) if len(sys.argv) > 1 else DEFAULT_FONT
    entries = parse_header()
    if not entries:
        print(f"ERROR: no kGlyph* constants parsed from {HEADER}")
        return 2
    cmap = TTFont(str(font_path)).getBestCmap()

    bad = [
        (const, cp, expected, cmap.get(cp) or "(absent)")
        for const, cp, expected in entries
        if cmap.get(cp) != expected
    ]

    if bad:
        print(f"MISMATCH against {font_path}:")
        for const, cp, expected, actual in bad:
            print(
                f"  {const}: U+{cp:05X} expected '{expected}' but font has '{actual}'"
            )
        byname = {n: c for c, n in cmap.items()}
        print("\nCorrect codepoints for the expected names present in this font:")
        for const, cp, expected, _actual in bad:
            fix = byname.get(expected)
            print(
                f"  {const} -> {expected}: "
                + (f"U+{fix:05X}" if fix else "(name absent)")
            )
        return 1

    print(
        f"OK: all {len(entries)} glyph codepoints resolve to their intended names in {font_path}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
