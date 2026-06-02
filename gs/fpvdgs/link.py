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
    def __init__(self, store, renderer_write, runner, drone):
        # renderer_write(effective_cfg: dict) -> None  renders + writes the cfg file
        self.store = store
        self.renderer_write = renderer_write
        self.runner = runner
        self.drone = drone

    def apply_link(self, apply_to: str = "both") -> dict:
        pending = self.store.pending()
        link = pending.get("link", {})

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

        # Apply the GS side unconditionally.
        self.store.commit()
        self.renderer_write(self.store.effective())
        gs_applied = self.runner.restart()

        return {
            "gsApplied": bool(gs_applied),
            "droneApplied": drone_applied,
            "droneReachable": drone_reachable,
            "inSync": drone_applied,
        }
