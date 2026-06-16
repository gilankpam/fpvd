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
    # Records between fsyncs (10 Hz → 50 = 5 s). The GS hard-reboots on video
    # loss; without fsync the unsynced tail (and on a vfat card, sometimes the
    # whole file) is lost on reboot. flush()+fsync() forces it to the card.
    sync_interval: int = 50


class FlightLog:
    def __init__(self, cfg: FlightLogConfig) -> None:
        self.cfg = cfg
        self._fh = None
        self._bytes = 0
        self._since_sync = 0
        self._reopen_pending = 0
        self._max_bytes = int(cfg.max_mb * 1024 * 1024)
        self._open()

    def _sync(self) -> None:
        """Force the buffered records out to the card. Survives the GS's
        reboot-on-video-loss, which a plain buffered write does not."""
        if self._fh is None:
            return
        try:
            self._fh.flush()
            os.fsync(self._fh.fileno())
        except OSError as e:
            log.warning("flightlog: fsync failed: %s", e)

    def _next_seq(self) -> int:
        """Next flight number = (highest numeric .jsonl stem on disk) + 1.

        Derived from the directory, not a clock, so it stays incremental
        across a GS restart (which would reset the monotonic clock). Resets
        to 1 only when the directory holds no flight files."""
        hi = 0
        try:
            for f in os.listdir(self.cfg.dir):
                if not f.endswith(".jsonl"):
                    continue
                try:
                    hi = max(hi, int(f[:-len(".jsonl")]))
                except ValueError:
                    continue
        except OSError:
            pass
        return hi + 1

    def _open(self) -> None:
        if not self.cfg.enabled:
            return
        try:
            os.makedirs(self.cfg.dir, exist_ok=True)
            self._path = os.path.join(self.cfg.dir, f"{self._next_seq():06d}.jsonl")
            self._fh = open(self._path, "w")
            self._bytes = 0
            self._since_sync = 0
        except OSError as e:
            log.warning("flightlog: open failed in %s: %s", self.cfg.dir, e)
            self._fh = None

    def write(self, record: dict) -> None:
        if self._fh is None:
            # _open() may have raced the DVR autofs automount at startup (mounts
            # on access, with latency) and failed -> writes were no-ops. Lazily
            # retry, throttled, so logging starts once the mount is up instead of
            # staying dead the whole flight (how cold-boot flights were lost).
            self._reopen_pending += 1
            if self._reopen_pending >= self.cfg.sync_interval:
                self._reopen_pending = 0
                self._open()
            if self._fh is None:
                return
        if self._bytes >= self._max_bytes:
            return  # this session hit its size cap; stop appending
        try:
            line = json.dumps(record, separators=(",", ":")) + "\n"
            self._fh.write(line)
            self._bytes += len(line)
            self._since_sync += 1
            if self._since_sync >= self.cfg.sync_interval:
                self._sync()
                self._since_sync = 0
        except (OSError, TypeError) as e:
            log.warning("flightlog: write failed: %s", e)

    def close(self) -> None:
        if self._fh is not None:
            self._sync()
            try:
                self._fh.close()
            except OSError:
                pass
            self._fh = None
        self._prune()

    def sync(self) -> None:
        """Flush + fsync the open flight file now — durability on demand, e.g.
        at a link-loss edge. No-op if no file is open."""
        self._sync()

    def begin_flight(self) -> None:
        """Ensure a fresh file is open for a new flight: roll to a new file if
        the current one already holds records, keep an already-open empty file,
        or (re)open one if none is open. No-op if disabled. Driven by the
        drone-connected event. When it rolls, the outgoing file is fsynced
        before close (via roll())."""
        if not self.cfg.enabled:
            return
        if self._fh is not None and self._bytes == 0:
            return                      # already on a fresh, empty flight file
        self.roll()

    def roll(self) -> None:
        """End the current flight file and begin a new one (a new flight).
        No-op if disabled. Re-attempts the open even if the previous one
        failed (e.g. the DVR mount came back)."""
        if not self.cfg.enabled:
            return
        if self._fh is not None:
            self._sync()
            try:
                self._fh.close()
            except OSError:
                pass
            self._fh = None
        self._prune()
        self._open()

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
