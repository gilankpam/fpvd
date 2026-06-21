"""Single source of GS config defaults (the former etc/defaults.json, in code).

Mirrors the drone: code holds every default; config.json is the full effective
config, merged onto these defaults. `--dump-config` materializes this tree."""
from __future__ import annotations

from .connection_monitor import ConnectionMonitorConfig
from .dynlink.learned_prior import LearnedPriorConfig
from .dynlink.policy import SelectorConfig
from .dynlink.signals import SignalAggregator


def _dynamic_link_defaults() -> dict:
    sel = SelectorConfig()
    agg = SignalAggregator()
    lp = LearnedPriorConfig()
    return {
        "enabled": False,
        # maxMcs 5 = operator runtime cap (distinct from SelectorConfig.max_mcs=7 HW ceiling fallback)
        "maxMcs": 5,
        "radioProfile": "m8812eu2",
        "dronePort": 9999,
        "selector": {
            "probeViableThreshold": sel.probe_viable_threshold,
            "probeFreshnessMs": sel.probe_freshness_ms,
            "promoteDebounceWindows": sel.promote_debounce_windows,
            "videoDemotePer": sel.video_demote_per,
            "emergencyFecPressure": sel.emergency_fec_pressure,
            "holdModesDownMs": sel.hold_modes_down_ms,
            "minBetweenChangesMs": sel.min_between_changes_ms,
            "starvationWindows": sel.starvation_windows,
            "lossWindows": sel.loss_windows,
        },
        "smoothing": {
            "ewmaAlphaRssi": agg.ewma_alpha_rssi,
            "ewmaAlphaFec": agg.ewma_alpha_fec,
            "ewmaAlphaBurst": agg.ewma_alpha_burst,
            "starvationThresholdPps": agg.starvation_threshold_pps,
        },
        "learnedPrior": {
            "settleTicks": lp.settle_ticks,
            "viableLoss": lp.viable_loss,
            "alphaTighten": lp.alpha_tighten,
            "alphaRelax": lp.alpha_relax,
            "minSamples": lp.min_samples,
            "recencyDecay": lp.recency_decay,
        },
        "flightlog": {"enabled": True},
        "rssiNorm": {"enabled": True},
    }


def _connection_monitor_defaults() -> dict:
    cm = ConnectionMonitorConfig()
    return {
        "enabled": cm.enabled,
        "tunnelStaleS": cm.tunnel_stale_s,
        "httpPollS": cm.http_poll_s,
        "httpTimeoutS": cm.http_timeout_s,
        "httpFailCount": cm.http_fail_count,
        "evalIntervalS": cm.eval_interval_s,
    }


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
        "drone": {"host": "10.5.0.10", "apiPort": 8080},
        "dynamicLink": _dynamic_link_defaults(),
        "idrForward": {"enabled": True, "port": 11223},
        "connectionMonitor": _connection_monitor_defaults(),
        "pixelpilot": {
            "enabled": True, "bin": "/usr/bin/pixelpilot", "env": {},
            "configPath": "/etc/pixelpilot.yaml",
            "osdConfigPath": "/etc/pixelpilot/osd.json",
            "screenMode": "1920x1080@60", "codec": "h265",
            "rtpPort": 5600,
            "dvr": {
                "dir": "/media/dvr",
                "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
                "fmp4": True, "sequencedFiles": True, "osd": False,
            },
            "extraArgs": [],
        },
    }
