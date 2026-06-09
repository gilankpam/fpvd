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


def split_patch(patch: dict) -> tuple[dict, dict, bool]:
    """Route a unified sparse PATCH into (gs_sparse, drone_sparse, touches_shared_link).
    Shared link keys go to the GS pending only (the coordinator pushes them to the
    drone at apply). dynamicLink.enabled goes to BOTH. Raises FacadeError on a
    read-only (_meta) or unknown section."""
    gs: dict = {}
    drone: dict = {}
    for top, val in patch.items():
        if top == "_meta":
            raise FacadeError("_meta is read-only")
        elif top == "link":
            _split_link(val or {}, gs, drone)
        elif top == "dynamicLink":
            _split_dynamic_link(val or {}, gs, drone)
        elif top in DRONE_SECTIONS:
            drone[top] = val
        elif top in GS_SECTIONS:
            gs[top] = val
        else:
            raise FacadeError(f"unknown config section: {top!r}")
    touches_shared = bool(set((patch.get("link") or {})) & set(SHARED_LINK_KEYS))
    return gs, drone, touches_shared


def _split_link(link: dict, gs: dict, drone: dict) -> None:
    gs_link, drone_link = {}, {}
    for k, v in link.items():
        if k in SHARED_LINK_KEYS:
            gs_link[k] = v
        elif k == "gs":
            gs_link.update(v or {})
        elif k == "drone":
            drone_link.update(v or {})
        else:
            raise FacadeError(f"unknown link key: {k!r}")
    if gs_link:
        gs["link"] = gs_link
    if drone_link:
        drone["link"] = drone_link


def _split_dynamic_link(dl: dict, gs: dict, drone: dict) -> None:
    gs_dl, drone_dl = {}, {}
    for k, v in dl.items():
        if k == "enabled":
            gs_dl["enabled"] = v
            drone_dl["enabled"] = v
        elif k == "controller":
            gs_dl["controller"] = v
        elif k == "applier":
            drone_dl.update(v or {})
        else:
            raise FacadeError(f"unknown dynamicLink key: {k!r}")
    if gs_dl:
        gs["dynamicLink"] = gs_dl
    if drone_dl:
        drone["dynamicLink"] = drone_dl
