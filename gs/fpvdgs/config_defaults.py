"""Single source of GS config defaults (the former etc/defaults.json, in code).

Mirrors the drone: code holds every default; config.json is the full effective
config, merged onto these defaults. `--dump-config` materializes this tree."""
from __future__ import annotations


def default_config() -> dict:
    return {
        "link": {
            "channel": 132, "width": 20, "txPowerDbm": None, "region": "US",
            "linkId": 7669206, "beamforming": {"enabled": False}, "wlans": "auto",
        },
        "wfb": {
            "profile": "gs",
            "mavlink": {"peer": "connect://127.0.0.1:14550"},
            "raw": {},
        },
        "drone": {"endpoint": "http://10.5.0.10:8080"},
        "dynamicLink": {
            "enabled": False, "maxMcs": 5, "radioProfile": "m8812eu2",
            "droneAddr": None, "dronePort": 9999, "videoStreamId": "video",
            "tuning": {},
        },
        "idrForward": {"enabled": True, "port": 11223},
        "pixelpilot": {
            "enabled": True, "bin": "/usr/bin/pixelpilot", "env": {},
            "configPath": "/etc/pixelpilot.yaml",
            "osdConfigPath": "/etc/pixelpilot/osd.json",
            "screenMode": "1920x1080@60", "videoScale": 1.0, "codec": "h265",
            "rtpPort": 5600, "rtpJitterMs": 0,
            "dvr": {
                "framerate": 60, "dir": "/media/dvr",
                "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
                "fmp4": True, "sequencedFiles": True, "osd": False,
                "mode": "raw", "maxSizeMb": 4000, "reencCodec": "h264",
                "reencBitrate": 8000, "reencFps": 30, "reencResolution": "1080p",
            },
            "extraArgs": [],
        },
    }
