"""Per-flight structured JSONL logger (Phase 4, spec §8).

One file per dynamicLink session, one JSON record per selector tick.
GS-side, dependency-free. Size-capped + rotated. Pulled off-device for
analysis by gs/tools/flightlog_analyze.py."""
from __future__ import annotations

import json
import logging
import os
from dataclasses import dataclass

log = logging.getLogger("fpvdgs.dynlink")


@dataclass
class FlightLogConfig:
    enabled: bool = True
    dir: str = "/media/dvr/log/dynamic-link/"
    max_files: int = 8
    max_mb: float = 4.0
    flight_gap_s: float = 15.0   # link gone > this (s) => next healthy tick = new flight file


class FlightLog:
    def __init__(self, cfg: FlightLogConfig, *, start_ms: int) -> None:
        self.cfg = cfg
        self._fh = None
        self._bytes = 0
        self._max_bytes = int(cfg.max_mb * 1024 * 1024)
        if not cfg.enabled:
            return
        try:
            os.makedirs(cfg.dir, exist_ok=True)
            self._path = os.path.join(cfg.dir, f"{start_ms}.jsonl")
            self._fh = open(self._path, "w")
        except OSError as e:
            log.warning("flightlog: open failed in %s: %s", cfg.dir, e)
            self._fh = None

    def write(self, record: dict) -> None:
        if self._fh is None:
            return
        if self._bytes >= self._max_bytes:
            return  # this session hit its size cap; stop appending
        try:
            line = json.dumps(record, separators=(",", ":")) + "\n"
            self._fh.write(line)
            self._bytes += len(line)
        except (OSError, TypeError) as e:
            log.warning("flightlog: write failed: %s", e)

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError:
                pass
            self._fh = None
        self._prune()

    def _prune(self) -> None:
        try:
            files = sorted(
                (os.path.join(self.cfg.dir, f) for f in os.listdir(self.cfg.dir)
                 if f.endswith(".jsonl")),
                key=lambda p: (os.path.getmtime(p), os.path.basename(p)),
            )
        except OSError:
            return
        for stale in files[:-self.cfg.max_files] if self.cfg.max_files > 0 else []:
            try:
                os.remove(stale)
            except OSError:
                pass
