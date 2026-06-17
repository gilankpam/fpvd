"""Generic in-process pub/sub event bus for cross-subsystem GS events.

Thread-safe, synchronous, exception-isolated dispatch. Publishers and
subscribers may live on different threads; each callback runs on the
PUBLISHER's thread, so a callback must be quick, non-blocking, and thread-safe
(marshal real work onto its own loop). The bus caches the latest payload per
state key (see _STATE_KEY) so a late subscriber can read current state via
state()."""
from __future__ import annotations

import logging
import threading
from typing import Callable

log = logging.getLogger("fpvdgs.events")

DRONE_CONNECTED = "drone.connected"
DRONE_DISCONNECTED = "drone.disconnected"

# event -> the state() cache key it updates
_STATE_KEY = {
    DRONE_CONNECTED: "drone",
    DRONE_DISCONNECTED: "drone",
}


class EventBus:
    def __init__(self) -> None:
        self._lock = threading.RLock()
        self._subs: dict[str, list[Callable[[dict], None]]] = {}
        self._state: dict[str, dict] = {}

    def subscribe(self, event: str, cb: Callable[[dict], None]) -> None:
        with self._lock:
            self._subs.setdefault(event, []).append(cb)

    def unsubscribe(self, event: str, cb: Callable[[dict], None]) -> None:
        with self._lock:
            subs = self._subs.get(event)
            if subs and cb in subs:
                subs.remove(cb)

    def publish(self, event: str, payload: dict | None = None) -> None:
        payload = payload if payload is not None else {}
        # Snapshot subscribers + update the state cache under the lock, then
        # dispatch OUTSIDE it so a callback can safely re-enter the bus.
        with self._lock:
            subs = list(self._subs.get(event, ()))
            key = _STATE_KEY.get(event)
            if key is not None:
                # Store a copy so a callback mutating its payload can't corrupt
                # the cached state read later via state().
                self._state[key] = dict(payload)
        for cb in subs:
            try:
                cb(payload)
            except Exception:
                log.exception("event subscriber for %s raised", event)

    def state(self, key: str, default=None):
        with self._lock:
            v = self._state.get(key, default)
            return dict(v) if isinstance(v, dict) else v
