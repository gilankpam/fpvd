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
    """Snapshot for the single probe wfb_rx: fixed port + key/linkId/wlans.
    The per-window measurement knobs (rxL, ewmaAlpha, blackoutWindows) are
    frozen calibration constants — there is no config path (rxL=50 is
    consistent with selector.probeFreshnessMs=500, so a probed rung never
    reads stale between wfb_rx stats batches)."""
    return {
        "port": PROBE_PORT,
        "rxL": PROBE_RX_L,
        "ewmaAlpha": PROBE_EWMA_ALPHA,
        "blackoutWindows": PROBE_BLACKOUT_WINDOWS,
        "key": GS_KEY,
        "linkId": effective.get("link", {}).get("linkId"),
        "wlans": resolve_wlans(effective),
    }
