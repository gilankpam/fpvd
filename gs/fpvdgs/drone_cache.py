"""Thin last-seen cache of the drone's /config, so GET /config can render the
drone subtree (grayed via _meta.droneStale) when the drone is unreachable.
The drone stays authoritative; this is a read-only render aid only."""
from __future__ import annotations

import copy
import datetime

from .drone_client import DroneUnreachable


def _utc_now_iso() -> str:
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


class DroneConfigCache:
    def __init__(self, drone, *, clock=_utc_now_iso):
        self._drone = drone
        self._clock = clock
        self._last_cfg = None
        self._last_seen = None

    def read(self):
        """Return (drone_cfg_or_None, meta). Refreshes the snapshot on success;
        serves the last-seen snapshot with droneStale on failure."""
        try:
            cfg = self._drone.get_config()
        except DroneUnreachable:
            return (copy.deepcopy(self._last_cfg),
                    {"droneReachable": False, "droneLastSeen": self._last_seen,
                     "droneStale": True})
        self._last_cfg = copy.deepcopy(cfg)
        self._last_seen = self._clock()
        return (copy.deepcopy(cfg),
                {"droneReachable": True, "droneLastSeen": self._last_seen,
                 "droneStale": False})
