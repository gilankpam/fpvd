"""Build the self-contained snapshot the ProbeController consumes, from the
effective/pending config. Mirrors dynlink.config_build.make_dl_snapshot."""
from __future__ import annotations

from ..runner_supervisor import resolve_wlans

GS_KEY = "/etc/gs.key"


def make_probe_snapshot(effective: dict) -> dict:
    """Augment the `probe` config block with the wfb keypair path, linkId, and
    resolved wlans that the probe wfb_rx command needs."""
    p = dict(effective.get("probe", {}))
    p["key"] = GS_KEY
    p["linkId"] = effective.get("link", {}).get("linkId")
    p["wlans"] = resolve_wlans(effective)
    return p
