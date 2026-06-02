"""GS-local-first link coordinator.

A link change ALWAYS applies on the GS (it is how a link is established).
The drone push is best-effort, only for apply_to == "both" and only when the
drone is reachable — never a precondition.
"""

from .drone_client import DroneUnreachable

# Only the truly-shared radio params go to the drone. GS-only keys
# (region, wlans, txpower, beamforming) are per-side and never pushed.
DRONE_PUSH_KEYS = ("channel", "width", "linkId")


def _bw_class(width):
    """Radiotap bandwidth class: 10 and 20 MHz are wire-identical (BW_20);
    only 40 differs (BW_40). See wfb-ng src/tx.cpp."""
    return 40 if width == 40 else 20


class LinkCoordinator:
    def __init__(self, store, renderer_write, runner, drone, validate=None, retune=None):
        # renderer_write(effective_cfg: dict) -> None  renders + writes the cfg file
        # validate(effective_cfg: dict) -> None         raises on invalid values (optional)
        # retune(channel: int, width: int) -> bool      live iw retune (optional; None = always bounce)
        self.store = store
        self.renderer_write = renderer_write
        self.runner = runner
        self.drone = drone
        self.validate = validate
        self.retune = retune
        self._last_sync = None

    def in_sync(self):
        return self._last_sync

    def _can_retune_live(self, old, new):
        """A live iw retune is safe only when the change is limited to
        channel/width AND the radiotap BW class is unchanged (so the running
        wfb_tx's -B need not change). Otherwise a full bounce re-inits -B."""
        if self.retune is None:
            return False
        changed = {k for k in set(old) | set(new) if old.get(k) != new.get(k)}
        if not changed <= {"channel", "width"}:
            return False
        return _bw_class(old.get("width")) == _bw_class(new.get("width"))

    def apply_link(self, apply_to: str = "both") -> dict:
        pending_cfg = self.store.pending()
        if self.validate is not None:
            self.validate(pending_cfg)   # raises (e.g. SchemaError) on bad values
        link = pending_cfg.get("link", {})

        drone_applied = False
        drone_reachable = False
        if apply_to == "both":
            drone_reachable = self.drone.healthz()
            if drone_reachable:
                push = {k: link[k] for k in DRONE_PUSH_KEYS if k in link}
                try:
                    self.drone.patch_config({"link": push})
                    self.drone.apply()
                    drone_applied = True
                except DroneUnreachable:
                    drone_reachable = False

        # Apply the GS side. Persist (render) the pending cfg, then either:
        #  - live iw retune the cards (no process restart), or
        #  - bounce the runner.
        # Commit only on success; otherwise roll the cfg back to last-good.
        last_good = self.store.effective()
        old_link = last_good.get("link", {})
        live = self._can_retune_live(old_link, link)

        self.renderer_write(pending_cfg)
        if live:
            gs_applied = self.retune(link.get("channel"), link.get("width"))
            mode = "live"
            if not gs_applied:           # live retune failed → fall back to a bounce
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

        return {
            "gsApplied": bool(gs_applied),
            "droneApplied": drone_applied,
            "droneReachable": drone_reachable,
            "inSync": bool(gs_applied) and drone_applied,
            "mode": mode,
        }
