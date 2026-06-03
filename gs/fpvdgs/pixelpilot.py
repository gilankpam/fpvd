"""Render the pixelpilot child argv from the effective config.

Reproduces the stock systemd ExecStart byte-for-byte at defaults:
  pixelpilot --osd --osd-custom-message --osd-config OSD --screen-mode SM
             --video-scale VS --dvr-framerate FPS --dvr-fmp4 --dvr-sequenced-files
             --dvr-template DIR/TMPL --config CONFIG [EXTRA...]
The always-on flags are baked in here; the four operator knobs and the
structural paths come from the `pixelpilot` config block.
"""

import os


def render_pixelpilot_argv(effective: dict) -> list[str]:
    pp = effective.get("pixelpilot", {})
    dvr_dir = pp.get("dvrDir", "/var/dvr")
    dvr_template = pp.get("dvrTemplate", "record_%Y-%m-%d_%H-%M-%S.mp4")
    return [
        pp.get("bin", "/usr/bin/pixelpilot"),
        "--osd", "--osd-custom-message",
        "--osd-config", pp.get("osdConfigPath", "/etc/pixelpilot/config_osd.json"),
        "--screen-mode", pp.get("screenMode", "1920x1080@60"),
        "--video-scale", str(pp.get("videoScale", 1.0)),
        "--dvr-framerate", str(pp.get("dvrFramerate", 60)),
        "--dvr-fmp4", "--dvr-sequenced-files",
        "--dvr-template", os.path.join(dvr_dir, dvr_template),
        "--config", pp.get("configPath", "/etc/pixelpilot/pixelpilot.yaml"),
        *pp.get("extraArgs", []),
    ]
