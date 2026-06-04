from fpvdgs.pixelpilot import render_pixelpilot_argv, render_pixelpilot_env

DEFAULTS = {"pixelpilot": {
    "bin": "/usr/bin/pixelpilot",
    "configPath": "/etc/pixelpilot.yaml",
    "osdConfigPath": "/etc/pixelpilot/osd.json",
    "screenMode": "1920x1080@60",
    "videoScale": 1.0,
    "codec": "h265",
    "rtpPort": 5600,
    "rtpJitterMs": 1,
    "dvr": {"framerate": 60, "dir": "/media/dvr",
            "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
            "fmp4": True, "sequencedFiles": True, "osd": False,
            "mode": "raw", "maxSizeMb": 4000, "reencCodec": "h264",
            "reencBitrate": 8000, "reencFps": 30, "reencResolution": "1080p"},
    "extraArgs": [],
}}


def test_defaults_render_full_argset():
    assert render_pixelpilot_argv(DEFAULTS) == [
        "/usr/bin/pixelpilot",
        "--config", "/etc/pixelpilot.yaml",
        "--osd", "--osd-custom-message",
        "--osd-config", "/etc/pixelpilot/osd.json",
        "--codec", "h265",
        "--screen-mode", "1920x1080@60",
        "--video-scale", "1.0",
        "--dvr-framerate", "60",
        "--dvr-fmp4", "--dvr-sequenced-files",
        "--dvr-template", "/media/dvr/record_%Y-%m-%d_%H-%M-%S.mp4",
        "--dvr-mode", "raw",
        "--dvr-max-size", "4000",
        "--dvr-reenc-codec", "h264",
        "--dvr-reenc-bitrate", "8000",
        "--dvr-reenc-fps", "30",
        "--dvr-reenc-resolution", "1080p",
        "-p", "5600",
        "--rtp-jitter-ms", "1",
    ]


def test_dvr_osd_flag_toggles():
    cfg = {"pixelpilot": {"dvr": {"osd": True}}}
    assert "--dvr-osd" in render_pixelpilot_argv(cfg)
    cfg2 = {"pixelpilot": {"dvr": {"osd": False}}}
    assert "--dvr-osd" not in render_pixelpilot_argv(cfg2)


def test_fmp4_and_sequenced_default_on_can_disable():
    argv = render_pixelpilot_argv({"pixelpilot": {"dvr": {"fmp4": False, "sequencedFiles": False}}})
    assert "--dvr-fmp4" not in argv and "--dvr-sequenced-files" not in argv


def test_knobs_reflected():
    cfg = {"pixelpilot": {"rtpPort": 5602, "rtpJitterMs": 5, "codec": "h264",
                          "screenMode": "1280x720@60", "videoScale": 0.75,
                          "dvr": {"mode": "reencode", "reencBitrate": 50000,
                                  "reencFps": 60, "reencResolution": "720p"}}}
    argv = render_pixelpilot_argv(cfg)
    assert argv[argv.index("-p") + 1] == "5602"
    assert argv[argv.index("--rtp-jitter-ms") + 1] == "5"
    assert argv[argv.index("--codec") + 1] == "h264"
    assert argv[argv.index("--screen-mode") + 1] == "1280x720@60"
    assert argv[argv.index("--video-scale") + 1] == "0.75"
    assert argv[argv.index("--dvr-mode") + 1] == "reencode"
    assert argv[argv.index("--dvr-reenc-bitrate") + 1] == "50000"


def test_extra_args_appended():
    argv = render_pixelpilot_argv({"pixelpilot": {"extraArgs": ["--disable-vsync"]}})
    assert argv[-1] == "--disable-vsync"


def test_missing_block_uses_defaults():
    argv = render_pixelpilot_argv({})
    assert argv[0] == "/usr/bin/pixelpilot"
    assert "-p" in argv and "--config" in argv


def test_render_env_stringifies():
    cfg = {"pixelpilot": {"env": {"LD_LIBRARY_PATH": "/usr/lib/pixelpilot95",
                                  "PP_NO_PANEL_FX": 1}}}
    assert render_pixelpilot_env(cfg) == {"LD_LIBRARY_PATH": "/usr/lib/pixelpilot95",
                                          "PP_NO_PANEL_FX": "1"}


def test_render_env_empty_default():
    assert render_pixelpilot_env({}) == {}
