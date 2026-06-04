"""Render the pixelpilot child process argv + env from the effective config.

Targets the PixelPilot FPV Decoder for Rockchip (>=1.3) CLI. The block models
the launch knobs the ground-station UI drives; flag order is irrelevant to the
getopt-style parser, so the renderer emits a stable canonical order. A separate
env map carries process environment (e.g. LD_LIBRARY_PATH for a perf-build lib
dir) that the supervisor merges over os.environ.
"""

import os


def render_pixelpilot_argv(effective: dict) -> list[str]:
    pp = effective.get("pixelpilot", {})
    dvr = pp.get("dvr", {})
    dvr_dir = dvr.get("dir", "/media/dvr")
    dvr_template = dvr.get("template", "record_%Y-%m-%d_%H-%M-%S.mp4")
    argv = [
        pp.get("bin", "/usr/bin/pixelpilot"),
        "--config", pp.get("configPath", "/etc/pixelpilot.yaml"),
        "--osd", "--osd-custom-message",
        "--osd-config", pp.get("osdConfigPath", "/etc/pixelpilot/osd.json"),
        "--codec", pp.get("codec", "h265"),
        "--screen-mode", pp.get("screenMode", "1920x1080@60"),
        "--video-scale", str(pp.get("videoScale", 1.0)),
        "--dvr-framerate", str(dvr.get("framerate", 60)),
    ]
    if dvr.get("fmp4", True):
        argv.append("--dvr-fmp4")
    if dvr.get("sequencedFiles", True):
        argv.append("--dvr-sequenced-files")
    argv += [
        "--dvr-template", os.path.join(dvr_dir, dvr_template),
        "--dvr-mode", dvr.get("mode", "raw"),
        "--dvr-max-size", str(dvr.get("maxSizeMb", 4000)),
        "--dvr-reenc-codec", dvr.get("reencCodec", "h264"),
        "--dvr-reenc-bitrate", str(dvr.get("reencBitrate", 8000)),
        "--dvr-reenc-fps", str(dvr.get("reencFps", 30)),
        "--dvr-reenc-resolution", dvr.get("reencResolution", "1080p"),
    ]
    if dvr.get("osd", False):
        argv.append("--dvr-osd")
    argv += [
        "-p", str(pp.get("rtpPort", 5600)),
        "--rtp-jitter-ms", str(pp.get("rtpJitterMs", 1)),
    ]
    argv += pp.get("extraArgs", [])
    return argv


def render_pixelpilot_env(effective: dict) -> dict:
    """Extra environment for the pixelpilot child, merged over os.environ by the
    supervisor (e.g. LD_LIBRARY_PATH for a perf-build lib dir)."""
    env = effective.get("pixelpilot", {}).get("env", {})
    return {str(k): str(v) for k, v in env.items()}
