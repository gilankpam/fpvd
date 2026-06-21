from fpvdgs.pixelpilot import render_pixelpilot_argv, render_pixelpilot_env

DEFAULTS = {"pixelpilot": {
    "bin": "/usr/bin/pixelpilot",
    "configPath": "/etc/pixelpilot.yaml",
    "osdConfigPath": "/etc/pixelpilot/osd.json",
    "screenMode": "1920x1080@60",
    "codec": "h265",
    "rtpPort": 5600,
    "dvr": {"dir": "/media/dvr",
            "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
            "fmp4": True, "sequencedFiles": True, "osd": False},
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
        "--dvr-fmp4", "--dvr-sequenced-files",
        "--dvr-template", "/media/dvr/record_%Y-%m-%d_%H-%M-%S.mp4",
        "-p", "5600",
    ]


def test_dvr_osd_flag_toggles():
    cfg = {"pixelpilot": {"dvr": {"osd": True}}}
    assert "--dvr-osd" in render_pixelpilot_argv(cfg)
    cfg2 = {"pixelpilot": {"dvr": {"osd": False}}}
    assert "--dvr-osd" not in render_pixelpilot_argv(cfg2)


def test_dead_flags_not_emitted():
    argv = render_pixelpilot_argv(DEFAULTS)
    for dead in ("--video-scale", "--rtp-jitter-ms", "--dvr-framerate",
                 "--dvr-mode", "--dvr-max-size", "--dvr-reenc-codec",
                 "--dvr-reenc-bitrate", "--dvr-reenc-fps",
                 "--dvr-reenc-resolution"):
        assert dead not in argv


def test_fmp4_and_sequenced_default_on_can_disable():
    argv = render_pixelpilot_argv({"pixelpilot": {"dvr": {"fmp4": False, "sequencedFiles": False}}})
    assert "--dvr-fmp4" not in argv and "--dvr-sequenced-files" not in argv


def test_knobs_reflected():
    cfg = {"pixelpilot": {"rtpPort": 5602, "codec": "h264",
                          "screenMode": "1280x720@60"}}
    argv = render_pixelpilot_argv(cfg)
    assert argv[argv.index("-p") + 1] == "5602"
    assert argv[argv.index("--codec") + 1] == "h264"
    assert argv[argv.index("--screen-mode") + 1] == "1280x720@60"


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
