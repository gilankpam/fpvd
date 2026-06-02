"""Config store: defaults baked-in, sparse user overlay, pending edits."""

import copy
import json
import os
import threading


def deep_merge(base: dict, overlay: dict) -> dict:
    """Return a new dict: overlay deep-merged onto base. Inputs untouched."""
    out = copy.deepcopy(base)
    for key, value in overlay.items():
        if isinstance(value, dict) and isinstance(out.get(key), dict):
            out[key] = deep_merge(out[key], value)
        else:
            out[key] = copy.deepcopy(value)
    return out


class ConfigStore:
    def __init__(self, defaults: dict, overlay: dict | None = None,
                 overlay_path: str | None = None):
        self._defaults = copy.deepcopy(defaults)
        self._overlay = copy.deepcopy(overlay) if overlay else {}
        self._pending = copy.deepcopy(self._overlay)
        self._overlay_path = overlay_path
        self._lock = threading.RLock()

    @classmethod
    def load(cls, defaults_path: str, overlay_path: str) -> "ConfigStore":
        with open(defaults_path) as f:
            defaults = json.load(f)
        overlay = {}
        if overlay_path and os.path.exists(overlay_path):
            with open(overlay_path) as f:
                overlay = json.load(f)
        return cls(defaults, overlay, overlay_path)

    def defaults(self) -> dict:
        return copy.deepcopy(self._defaults)

    def effective(self) -> dict:
        with self._lock:
            return deep_merge(self._defaults, self._overlay)

    def pending(self) -> dict:
        with self._lock:
            return deep_merge(self._defaults, self._pending)

    def patch(self, sparse: dict) -> None:
        with self._lock:
            self._pending = deep_merge(self._pending, sparse)

    def commit(self) -> None:
        with self._lock:
            self._overlay = copy.deepcopy(self._pending)
            self._persist()

    def reset(self) -> None:
        with self._lock:
            self._overlay = {}
            self._pending = {}
            self._persist()

    def _persist(self) -> None:
        if not self._overlay_path:
            return
        tmp = self._overlay_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._overlay, f, indent=2)
        os.replace(tmp, self._overlay_path)
