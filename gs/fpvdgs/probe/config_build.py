"""Build the self-contained snapshot the ProbeController consumes. One fixed
probe wfb_rx on kProbePort (matching the drone's probe radio_port). No probe
config — the probe lifecycle follows dynamicLink."""
from __future__ import annotations

from ..runner_supervisor import resolve_wlans

GS_KEY = "/etc/gs.key"
PROBE_PORT = 50    # wfb radio_port; MUST match the drone's kProbeRadioPort
PROBE_RX_L = 50    # wfb_rx -l log interval (ms)
PROBE_EWMA_ALPHA = 0.25         # per-MCS PER EWMA smoothing
PROBE_BLACKOUT_WINDOWS = 10     # consecutive empty windows before per=1.0


def make_probe_snapshot(effective: dict) -> dict:
    """The snapshot for the single probe wfb_rx: fixed port + key/linkId/wlans,
    plus the per-window measurement tuning (`probe` config block): `rxL` (wfb_rx
    -l window ms), `ewmaAlpha`, and `blackoutWindows`. These trade airtime vs
    measurement smoothness — a wider window aggregates more packets per sample."""
    probe = effective.get("probe", {}) or {}
    return {
        "port": PROBE_PORT,
        "rxL": int(probe.get("rxL", PROBE_RX_L)),
        "ewmaAlpha": float(probe.get("ewmaAlpha", PROBE_EWMA_ALPHA)),
        "blackoutWindows": int(probe.get("blackoutWindows", PROBE_BLACKOUT_WINDOWS)),
        "key": GS_KEY,
        "linkId": effective.get("link", {}).get("linkId"),
        "wlans": resolve_wlans(effective),
    }
