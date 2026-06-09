"""GS-local-first link coordinator.

A link change ALWAYS applies on the GS (it is how a link is established).
The drone push is best-effort, only for apply_to == "both" and only when the
drone is reachable — never a precondition.
"""

from .drone_client import DroneUnreachable, DroneRejected
from .schema import SchemaError

# Only the truly-shared radio params go to the drone. GS-only keys
# (region, wlans, txpower) are per-side and never pushed. `beamforming` is
# pushed separately (with the MAC transformed) by apply_link.
DRONE_PUSH_KEYS = ("channel", "width", "linkId")


def _bw_class(width):
    """Radiotap bandwidth class: 10 and 20 MHz are wire-identical (BW_20);
    only 40 differs (BW_40). See wfb-ng src/tx.cpp."""
    return 40 if width == 40 else 20


class LinkCoordinator:
    def __init__(self, store, renderer_write, runner, drone, validate=None,
                 retune=None, beamforming=None, wlans_resolver=None):
        # renderer_write(effective_cfg: dict) -> None  renders + writes the cfg file
        # validate(effective_cfg: dict) -> None         raises on invalid values (optional)
        # retune(link: dict) -> bool                     live iw retune (optional; None = always bounce)
        # beamforming                                    GS beamformee controller (optional)
        # wlans_resolver(cfg: dict) -> list[str]         resolves the card list; [0] is the BF peer
        self.store = store
        self.renderer_write = renderer_write
        self.runner = runner
        self.drone = drone
        self.validate = validate
        self.retune = retune
        self.beamforming = beamforming
        self.wlans_resolver = wlans_resolver
        self._last_sync = None

    def in_sync(self):
        return self._last_sync

    def _can_retune_live(self, old, new):
        """A live iw retune is safe only when the change is limited to fields
        that `iw` can apply on a running monitor card (channel/width/txpower/
        region) AND the radiotap BW class is unchanged. `beamforming` is
        reconciled separately, so it is excluded here. Anything else (wlans,
        linkId, …) or a 40 MHz crossing falls back to a full runner bounce."""
        if self.retune is None:
            return False
        changed = {k for k in set(old) | set(new)
                   if k != "beamforming" and old.get(k) != new.get(k)}
        if not changed <= {"channel", "width", "txpower", "region"}:
            return False
        return _bw_class(old.get("width")) == _bw_class(new.get("width"))

    def _primary_iface(self, cfg):
        if self.beamforming is None or self.wlans_resolver is None:
            return None
        wlans = self.wlans_resolver(cfg)
        return wlans[0] if wlans else None

    def _reconcile_beamforming(self, enabled, primary, drone_mac):
        """Arm/disarm the GS beamformee. Orthogonal to retune/bounce. Returns
        the status block for the apply result, or None when BF is not wired."""
        if self.beamforming is None or primary is None:
            return None
        if enabled and not drone_mac:
            # Can't arm without the drone's MAC (drone unreachable / no status).
            st = dict(self.beamforming.status())
            st["state"] = "pending"
            st["reason"] = "drone unreachable; peer MAC unknown"
            return st
        return self.beamforming.reconcile(enabled, primary,
                                          drone_mac if enabled else "")

    def apply_link(self, apply_to: str = "both") -> dict:
        pending_cfg = self.store.pending()
        if self.validate is not None:
            self.validate(pending_cfg)   # raises (e.g. SchemaError) on bad values
        link = pending_cfg.get("link", {})
        last_good = self.store.effective()
        old_link = last_good.get("link", {})

        primary = self._primary_iface(pending_cfg)
        bf_new = link.get("beamforming") or {}
        bf_enabled = bool(bf_new.get("enabled"))
        bf_changed = bf_new != (old_link.get("beamforming") or {})

        # Capability hard-reject: enabling BF requires the bf_monitor_conf node
        # to exist on the primary card RIGHT NOW. Aborts before any commit/push.
        if bf_enabled and self.beamforming is not None:
            if primary is None or not self.beamforming.supported(primary):
                raise SchemaError(
                    f"beamforming unavailable on {primary}: no bf_monitor_conf "
                    f"node (GS driver lacks CONFIG_BEAMFORMING_MONITOR)")

        drone_applied = False
        drone_reachable = False
        drone_mac = ""
        drone_error = None
        if apply_to == "both":
            drone_reachable = self.drone.healthz()
            if drone_reachable:
                # Push only the shared keys that actually CHANGED (see the
                # dynamic-link-locked-width regression). beamforming is pushed
                # separately, with the GS MAC as the drone's remoteMac.
                push = {k: link[k] for k in DRONE_PUSH_KEYS
                        if k in link and link[k] != old_link.get(k)}
                if bf_changed and self.beamforming is not None:
                    gs_mac = self.beamforming.local_mac(primary) if primary else ""
                    push["beamforming"] = {"enabled": bf_enabled,
                                           "remoteMac": gs_mac}
                    # STBC and TX beamforming are mutually exclusive on the drone
                    # (it rejects beamforming while stbc=true). Flip stbc to match:
                    # false to enable BF, true to restore on disable.
                    push["stbc"] = not bf_enabled
                try:
                    if push:
                        self.drone.patch_config({"link": push})
                        self.drone.apply()
                    # Read the drone's card MAC AFTER the enable push: the drone
                    # only populates beamforming.localMac once its own BF is
                    # enabled (it resolves the MAC in reconcile). Reading before
                    # the push would see "" on the first apply, so the GS could
                    # never arm without a second apply.
                    if bf_enabled and self.beamforming is not None:
                        drone_mac = (self.drone.get_status()
                                     .get("beamforming", {}).get("localMac", ""))
                    drone_applied = True   # empty push => already in sync
                except DroneRejected as e:
                    # Validation rejection — a real error, NOT a connectivity
                    # failure. Keep drone_reachable True; surface the error.
                    drone_error = {"code": e.code, "message": e.message,
                                   "details": e.body.get("details")
                                              if isinstance(e.body, dict) else None}
                except DroneUnreachable:
                    drone_reachable = False

        # GS-side beamforming reconcile — orthogonal to the RF path below.
        bf_result = self._reconcile_beamforming(bf_enabled, primary, drone_mac)

        # The retune/bounce decision uses only the NON-beamforming link delta,
        # so toggling BF never retunes or bounces the running video pipeline.
        non_bf_changed = any(k != "beamforming" and old_link.get(k) != link.get(k)
                             for k in set(old_link) | set(link))

        self.renderer_write(pending_cfg)
        if not non_bf_changed:
            gs_applied = True
            mode = "none"
        else:
            live = self._can_retune_live(old_link, link)
            if live:
                gs_applied = self.retune(link)
                mode = "live"
                if not gs_applied:           # live retune failed → bounce
                    gs_applied = self.runner.restart()
                    mode = "bounce"
            else:
                gs_applied = self.runner.restart()
                mode = "bounce"

        if gs_applied:
            self.store.commit()
            self._last_sync = (apply_to == "both") and drone_applied
        else:
            self.renderer_write(last_good)
            self.runner.restart()
            # Reconcile BF back to last-good so the beamformee HW doesn't stay in
            # the new state while the committed config rolled back.
            old_bf_enabled = bool((old_link.get("beamforming") or {}).get("enabled"))
            self._reconcile_beamforming(old_bf_enabled, primary, drone_mac)

        res = {
            "gsApplied": bool(gs_applied),
            "droneApplied": drone_applied,
            "droneReachable": drone_reachable,
            "inSync": bool(gs_applied) and drone_applied,
            "mode": mode,
        }
        if bf_result is not None:
            res["beamforming"] = bf_result
        if drone_error is not None:
            res["droneError"] = drone_error
        return res
