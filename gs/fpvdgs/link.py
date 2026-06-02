"""GS-local-first link coordinator.

A link change ALWAYS applies on the GS (it is how a link is established).
The drone push is best-effort, only for apply_to == "both" and only when the
drone is reachable — never a precondition.
"""

from .drone_client import DroneUnreachable

# Only the truly-shared radio params go to the drone. GS-only keys
# (region, wlans, txpower, beamforming) are per-side and never pushed.
DRONE_PUSH_KEYS = ("channel", "width", "linkId")


class LinkCoordinator:
    def __init__(self, store, renderer_write, runner, drone, validate=None):
        # renderer_write(effective_cfg: dict) -> None  renders + writes the cfg file
        # validate(effective_cfg: dict) -> None         raises on invalid values (optional)
        self.store = store
        self.renderer_write = renderer_write
        self.runner = runner
        self.drone = drone
        self.validate = validate
        self._last_sync = None

    def in_sync(self):
        return self._last_sync

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

        # Apply the GS side: render pending, restart; commit only on success,
        # otherwise roll the cfg back to last-good and bring the runner back.
        last_good = self.store.effective()
        self.renderer_write(pending_cfg)
        gs_applied = self.runner.restart()
        if gs_applied:
            self.store.commit()
        else:
            self.renderer_write(last_good)
            self.runner.restart()

        if gs_applied:
            self._last_sync = (apply_to == "both") and drone_applied

        return {
            "gsApplied": bool(gs_applied),
            "droneApplied": drone_applied,
            "droneReachable": drone_reachable,
            "inSync": bool(gs_applied) and drone_applied,
        }
