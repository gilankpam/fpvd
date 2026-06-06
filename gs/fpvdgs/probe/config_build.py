"""Build the self-contained snapshot the ProbeController consumes. One fixed
probe wfb_rx on kProbePort (matching the drone's probe radio_port). No probe
config — the probe lifecycle follows dynamicLink."""
from __future__ import annotations

from ..runner_supervisor import resolve_wlans

GS_KEY = "/etc/gs.key"
PROBE_PORT = 50    # wfb radio_port; MUST match the drone's kProbeRadioPort
PROBE_RX_L = 50    # wfb_rx -l log interval (ms)


def make_probe_snapshot(effective: dict) -> dict:
    """The snapshot for the single probe wfb_rx: fixed port + key/linkId/wlans."""
    return {
        "port": PROBE_PORT,
        "rxL": PROBE_RX_L,
        "key": GS_KEY,
        "linkId": effective.get("link", {}).get("linkId"),
        "wlans": resolve_wlans(effective),
    }
