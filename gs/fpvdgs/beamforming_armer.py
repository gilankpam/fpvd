"""GS beamformee boot re-arm loop.

The BeamformingController only arms on a /link/apply. After a GS restart/reboot
(which clears the driver's TXBF registers), link.beamforming.enabled stays true
but the beamformee is never re-armed — BF silently stays off. This background
loop arms it once the drone is reachable, retrying until then. Idempotent with
/link/apply (a no-op when already active or disabled); never disarms.
"""

import threading

from .drone_client import DroneUnreachable, DroneRejected


class BeamformingArmer:
    def __init__(self, beamforming, drone, wlans_resolver, config_provider,
                 interval: float = 5.0):
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
                self._tick()
            except Exception:
                pass   # the re-arm loop must never die
            self._stop.wait(self._interval)

    def _tick(self):
        """Arm the beamformee iff config wants it but it isn't active yet."""
        cfg = self._cfg()
        bf = (cfg.get("link", {}) or {}).get("beamforming", {}) or {}
        if not bf.get("enabled"):
            return
        if self._bf.status().get("state") == "active":
            return
        wlans = self._wlans(cfg) or []
        primary = wlans[0] if wlans else None
        if not primary or not self._bf.supported(primary):
            return
        if not self._drone.healthz():
            return
        try:
            mac = (self._drone.get_status()
                   .get("beamforming", {}).get("localMac", ""))
        except (DroneUnreachable, DroneRejected):
            return
        if mac:
            self._bf.reconcile(True, primary, mac)
