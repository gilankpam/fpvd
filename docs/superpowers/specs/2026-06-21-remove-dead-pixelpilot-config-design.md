# Remove dead pixelpilot config knobs

**Date:** 2026-06-21
**Scope:** GS (`fpvdgs`) only
**Status:** Design approved

## Problem

The GS `pixelpilot` config block exposes nine knobs whose corresponding
pixelpilot CLI arguments no longer exist in the current pixelpilot binary.
`render_pixelpilot_argv` still emits these flags, so launching the new binary
would pass arguments it rejects. The knobs are dead config surface and must be
removed.

## Fields to remove

| Config key | Dead CLI arg |
|---|---|
| `pixelpilot.videoScale` | `--video-scale` |
| `pixelpilot.rtpJitterMs` | `--rtp-jitter-ms` |
| `pixelpilot.dvr.framerate` | `--dvr-framerate` |
| `pixelpilot.dvr.mode` | `--dvr-mode` |
| `pixelpilot.dvr.maxSizeMb` | `--dvr-max-size` |
| `pixelpilot.dvr.reencCodec` | `--dvr-reenc-codec` |
| `pixelpilot.dvr.reencBitrate` | `--dvr-reenc-bitrate` |
| `pixelpilot.dvr.reencFps` | `--dvr-reenc-fps` |
| `pixelpilot.dvr.reencResolution` | `--dvr-reenc-resolution` |

Removing `dvr.mode` retires reencode mode entirely, so all four `reenc*` params
(codec, bitrate, fps, resolution) are orphaned and removed together.

## Fields retained

CLI args still valid, so these stay:

- `pixelpilot`: `bin`, `env`, `configPath`, `osdConfigPath`, `screenMode`,
  `codec`, `rtpPort`, `extraArgs`
- `pixelpilot.dvr`: `dir`, `template`, `fmp4`, `sequencedFiles`, `osd`

## Changes by file

1. **`gs/fpvdgs/config_defaults.py`** — delete the nine keys from the
   `pixelpilot` and `pixelpilot.dvr` default blocks.

2. **`gs/fpvdgs/pixelpilot.py`** (`render_pixelpilot_argv`) — delete the
   `--video-scale`, `--dvr-framerate`, and `--rtp-jitter-ms` emissions, and the
   `--dvr-mode`/`--dvr-max-size`/`--dvr-reenc-codec`/`--dvr-reenc-bitrate`/
   `--dvr-reenc-fps`/`--dvr-reenc-resolution` argv block. That block collapses
   to just `--dvr-template`.

3. **`gs/fpvdgs/schema.py`** (`_validate_pixelpilot`) — delete the `videoScale`
   and `rtpJitterMs` validators; remove `framerate`, `maxSizeMb`, `reencBitrate`,
   `reencFps` from the positive-int loop (the loop then has no keys and is
   removed); remove `mode`, `reencCodec`, `reencResolution` from the string loop
   (leaving `dir`, `template`).

4. **Tests**
   - `gs/tests/unit/test_pixelpilot_render.py` — update `DEFAULTS`, the expected
     argv in `test_defaults_render_full_argset`, and `test_knobs_reflected`
     (drop the removed-field assertions).
   - `gs/tests/unit/test_schema.py` — update the valid pixelpilot block and drop
     the now-meaningless invalid-case rows for the removed fields.
   - `gs/tests/unit/test_api.py` — update the `videoScale`/`dvrFramerate`
     fixtures and the `videoScale` PATCH round-trip test.

5. **Docs** — update live references in `docs/api.md` and `gs/README.md`.
   Historical specs/plans under `docs/superpowers/` are point-in-time and left
   untouched per project convention.

## Migration and safety

No migration code. A deployed `config.json` still carrying the old keys is
handled by the existing tolerant loader, which warns and strips unknown keys, so
there is no boot-brick risk.

## Validation

The full GS pytest suite (`cd gs && .venv/bin/python -m pytest tests/ -q`) is the
acceptance bar and must stay green as a whole.

## Out of scope

- No drone-side changes (these knobs are GS-only).
- No changes to the retained DVR flags or any other pixelpilot config.
