"""Compose the GS-local config and the drone's config into ONE unified
Option-C tree (GET /config), and route unified patches back to each side.

Mapping (single source of truth):
  link.{channel,width,linkId,beamforming}  SHARED  (GS holds the live copy)
  link.gs.{region,rxpower,wlans}            GS      (GS link minus shared)
  link.drone.*                             DRONE   (drone link minus shared)
  dynamicLink.enabled                      SHARED/BOTH (hard-gated)
  dynamicLink.controller.*                 GS
  dynamicLink.applier.*                    DRONE   (drone dynamicLink minus enabled)
  video/image/telemetry/recording/services DRONE   (passthrough)
  wfb/pixelpilot/droneLink                 GS      (passthrough)
"""
from __future__ import annotations

SHARED_LINK_KEYS = ("channel", "width", "linkId", "beamforming")
GS_LINK_KEYS = ("region", "rxpower", "wlans")
DRONE_SECTIONS = ("video", "image", "telemetry", "recording", "services")
GS_SECTIONS = ("wfb", "pixelpilot", "droneLink")


class FacadeError(ValueError):
    """A unified PATCH touched an unknown or read-only path."""


def build_config_tree(gs_eff: dict, drone_cfg: dict | None, meta: dict) -> dict:
    """Merge the GS effective config and the drone config (live or last-seen,
    or None if never seen) into the unified Option-C tree. `meta` is the
    caller-built `_meta` block (reachability/staleness)."""
    gs_link = gs_eff.get("link", {})
    drone_link = (drone_cfg or {}).get("link", {})
    link = {k: gs_link[k] for k in SHARED_LINK_KEYS if k in gs_link}
    link["gs"] = {k: gs_link[k] for k in GS_LINK_KEYS if k in gs_link}
    link["drone"] = {k: v for k, v in drone_link.items() if k not in SHARED_LINK_KEYS}

    gs_dl = gs_eff.get("dynamicLink", {})
    drone_dl = (drone_cfg or {}).get("dynamicLink", {})
    dynamic_link = {
        "enabled": bool(gs_dl.get("enabled", False)),
        "controller": gs_dl.get("controller", {}),
        "applier": {k: v for k, v in drone_dl.items() if k != "enabled"},
    }

    out = {"_meta": meta, "link": link, "dynamicLink": dynamic_link}
    for s in DRONE_SECTIONS:
        out[s] = (drone_cfg or {}).get(s, {})
    for s in GS_SECTIONS:
        if s in gs_eff:
            out[s] = gs_eff[s]
    return out
