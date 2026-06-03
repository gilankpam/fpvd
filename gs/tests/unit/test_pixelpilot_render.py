from fpvdgs.pixelpilot import render_pixelpilot_argv

DEFAULTS = {
    "pixelpilot": {
        "enabled": True,
        "bin": "/usr/bin/pixelpilot",
        "configPath": "/etc/pixelpilot/pixelpilot.yaml",
        "screenMode": "1920x1080@60",
        "videoScale": 1.0,
        "osdConfigPath": "/etc/pixelpilot/config_osd.json",
        "dvrFramerate": 60,
        "dvrDir": "/var/dvr",
        "dvrTemplate": "record_%Y-%m-%d_%H-%M-%S.mp4",
        "extraArgs": [],
    }
}


def test_defaults_reproduce_execstart():
    assert render_pixelpilot_argv(DEFAULTS) == [
        "/usr/bin/pixelpilot",
        "--osd", "--osd-custom-message",
        "--osd-config", "/etc/pixelpilot/config_osd.json",
        "--screen-mode", "1920x1080@60",
        "--video-scale", "1.0",
        "--dvr-framerate", "60",
        "--dvr-fmp4", "--dvr-sequenced-files",
        "--dvr-template", "/var/dvr/record_%Y-%m-%d_%H-%M-%S.mp4",
        "--config", "/etc/pixelpilot/pixelpilot.yaml",
    ]


def test_knobs_are_reflected():
    cfg = {"pixelpilot": dict(DEFAULTS["pixelpilot"],
                              screenMode="1280x720@60", videoScale=1.5,
                              dvrFramerate=30,
                              osdConfigPath="/tmp/osd.json")}
    argv = render_pixelpilot_argv(cfg)
    assert argv[argv.index("--screen-mode") + 1] == "1280x720@60"
    assert argv[argv.index("--video-scale") + 1] == "1.5"
    assert argv[argv.index("--dvr-framerate") + 1] == "30"
    assert argv[argv.index("--osd-config") + 1] == "/tmp/osd.json"


def test_extra_args_appended_verbatim():
    cfg = {"pixelpilot": dict(DEFAULTS["pixelpilot"],
                              extraArgs=["--no-vsync", "--foo", "bar"])}
    assert render_pixelpilot_argv(cfg)[-3:] == ["--no-vsync", "--foo", "bar"]


def test_missing_block_uses_builtin_defaults():
    # An empty config still renders a valid argv (defaults baked into the renderer).
    argv = render_pixelpilot_argv({})
    assert argv[0] == "/usr/bin/pixelpilot"
    assert "--config" in argv


def test_custom_dvr_dir_and_template_compose():
    cfg = {"pixelpilot": dict(DEFAULTS["pixelpilot"],
                              dvrDir="/mnt/usb", dvrTemplate="rec_%Y.mp4")}
    argv = render_pixelpilot_argv(cfg)
    assert argv[argv.index("--dvr-template") + 1] == "/mnt/usb/rec_%Y.mp4"
