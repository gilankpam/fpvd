"""Config store: code defaults + a single full config.json, deep-merged.

Mirrors the drone model — code (config_defaults.default_config) is the single
source of defaults; config.json holds the full effective config and is merged
onto the defaults so a missing key takes its default. Persistence rewrites the
full effective config (no sparse overlay)."""

import copy
import json
import logging
import os
import threading

from .config_defaults import default_config
from .schema import (
    DRONE_KEYS,
    DYNAMIC_LINK_KEYS,
    LEARNED_PRIOR_KEYS,
    SELECTOR_KEYS,
    SMOOTHING_KEYS,
    TAP_KEYS,
    TX_SELECTOR_KEYS,
)

log = logging.getLogger("fpvdgs.config")


def _warn_unknown(loaded: dict, defaults: dict) -> dict:
    """Warn on AND strip keys absent from the code defaults — scoped to the
    top level, the dynamicLink / drone subtrees, and wfb.txSelector. Returns a
    pruned copy so stale / unknown keys never reach the effective config: this
    keeps an old config.json from bricking boot (validate_effective is strict
    on those keys) and matches the drone's drop-unknowns load. Other blocks
    (pixelpilot/wfb's own top-level keys/link) hold open maps and are left
    untouched."""
    pruned = copy.deepcopy(loaded)
    for key in sorted(set(pruned) - set(defaults)):
        log.warning("ignoring unknown config key: %s", key)
        del pruned[key]
    for block, known in (("dynamicLink", DYNAMIC_LINK_KEYS), ("drone", DRONE_KEYS)):
        sub = pruned.get(block)
        if isinstance(sub, dict):
            for key in sorted(set(sub) - known):
                log.warning("ignoring unknown %s key: %s", block, key)
                del sub[key]
    # dynamicLink's nested selector/smoothing blocks are ALSO strict in
    # validate_effective, so strip their unknown keys too — otherwise a removed
    # knob (e.g. a dropped emergencyLossRate) left in a stale config.json reaches
    # validate_effective and bricks boot.
    dl = pruned.get("dynamicLink")
    if isinstance(dl, dict):
        for block, known in (
            ("selector", SELECTOR_KEYS),
            ("smoothing", SMOOTHING_KEYS),
            ("learnedPrior", LEARNED_PRIOR_KEYS),
            ("tap", TAP_KEYS),
        ):
            sub = dl.get(block)
            if isinstance(sub, dict):
                for key in sorted(set(sub) - known):
                    log.warning("ignoring unknown dynamicLink.%s key: %s", block, key)
                    del sub[key]
    # wfb itself stays an open map (profile/mavlink/raw), but txSelector is
    # strict in validate_effective, so it needs the same nested pruning.
    wfb = pruned.get("wfb")
    if isinstance(wfb, dict):
        txsel = wfb.get("txSelector")
        if isinstance(txsel, dict):
            for key in sorted(set(txsel) - TX_SELECTOR_KEYS):
                log.warning("ignoring unknown wfb.txSelector key: %s", key)
                del txsel[key]
    return pruned


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
    def __init__(self, defaults: dict, loaded: dict | None = None, config_path: str | None = None):
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
            loaded = _warn_unknown(loaded, defaults)
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
