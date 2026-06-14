"""Config store: code defaults + a single full config.json, deep-merged.

Mirrors the drone model — code (config_defaults.default_config) is the single
source of defaults; config.json holds the full effective config and is merged
onto the defaults so a missing key takes its default. Persistence rewrites the
full effective config (no sparse overlay)."""

import copy
import json
import os
import threading

from .config_defaults import default_config


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
    def __init__(self, defaults: dict, loaded: dict | None = None,
                 config_path: str | None = None):
        self._defaults = copy.deepcopy(defaults)
        self._config = deep_merge(self._defaults, loaded or {})
        self._pending = copy.deepcopy(self._config)
        self._config_path = config_path
        self._lock = threading.RLock()

    @classmethod
    def load(cls, config_path: str) -> "ConfigStore":
        defaults = default_config()
        loaded = {}
        if config_path and os.path.exists(config_path):
            with open(config_path) as f:
                loaded = json.load(f)
        return cls(defaults, loaded, config_path)

    def defaults(self) -> dict:
        return copy.deepcopy(self._defaults)

    def effective(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._config)

    def pending(self) -> dict:
        with self._lock:
            return copy.deepcopy(self._pending)

    def patch(self, sparse: dict) -> None:
        with self._lock:
            self._pending = deep_merge(self._pending, sparse)

    def commit(self) -> None:
        with self._lock:
            self._config = copy.deepcopy(self._pending)
            self._persist()

    def reset(self) -> None:
        with self._lock:
            self._config = copy.deepcopy(self._defaults)
            self._pending = copy.deepcopy(self._defaults)
            self._persist()

    def _persist(self) -> None:
        if not self._config_path:
            return
        tmp = self._config_path + ".tmp"
        with open(tmp, "w") as f:
            json.dump(self._config, f, indent=2)
        os.replace(tmp, self._config_path)
