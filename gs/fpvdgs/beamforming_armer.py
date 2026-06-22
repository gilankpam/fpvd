"""GS beamformee reconcile loop.

The BeamformingController only changes state on an explicit reconcile. After a
GS restart/reboot (which clears the driver's TXBF registers),
link.beamforming.enabled stays true but the beamformee is never re-armed — BF
silently stays off. This background loop is a full reconcile toward config: it
arms the beamformee when config wants BF and it's not active (once the drone is
reachable, retrying until then), and disarms it when config doesn't want BF but
it's still active. It reads the drone's MAC read-only and never pushes to the
drone (the client owns the drone-side handshake). Idempotent and a no-op when
already in the desired state; complements a nudge from /gs/apply.
"""

import threading

from .drone_client import DroneRejected, DroneUnreachable


class BeamformingArmer:
    def __init__(self, beamforming, drone, wlans_resolver, config_provider, interval: float = 5.0):
        # beamforming: BeamformingController; drone: DroneClient
        # wlans_resolver(cfg) -> list[str]; config_provider() -> effective cfg
        self._bf = beamforming
        self._drone = drone
        self._wlans = wlans_resolver
        self._cfg = config_provider
        self._interval = interval
        self._stop = threading.Event()
        self._thr = None

    def start(self):
        if self._thr is not None:
            return
        self._stop.clear()
        self._thr = threading.Thread(target=self._loop, daemon=True)
        self._thr.start()

    def stop(self):
        self._stop.set()
        if self._thr is not None:
            self._thr.join(timeout=self._interval + 1)
            self._thr = None

    def _loop(self):
        while not self._stop.is_set():
            try:
                self.tick()
            except Exception:
                pass  # the reconcile loop must never die
            self._stop.wait(self._interval)

    def tick(self):
        """Reconcile the beamformee to config: arm when enabled+inactive,
        disarm when disabled+active. Reads the drone MAC read-only; never
        pushes to the drone (the client owns the drone-side handshake)."""
        cfg = self._cfg()
        bf = (cfg.get("link", {}) or {}).get("beamforming", {}) or {}
        want = bool(bf.get("enabled"))
        active = self._bf.status().get("state") == "active"

        if not want:
            if active:
                wlans = self._wlans(cfg) or []
                primary = wlans[0] if wlans else None
                if primary:
                    self._bf.reconcile(False, primary, "")
            return

        if active:
            return
        wlans = self._wlans(cfg) or []
        primary = wlans[0] if wlans else None
        if not primary or not self._bf.supported(primary):
            return
        if not self._drone.healthz():
            return
        try:
            mac = self._drone.get_status().get("beamforming", {}).get("localMac", "")
        except (DroneUnreachable, DroneRejected):
            return
        if mac:
            self._bf.reconcile(True, primary, mac)
