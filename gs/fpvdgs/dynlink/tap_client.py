"""asyncio listener for the wfb_rx dynlink tap (localhost UDP, tap wire v1).

The controller owns mode selection (tap-alive vs :8103 fallback); this
module only decodes datagrams and dispatches records."""

from __future__ import annotations

import asyncio
import logging

from .tap_wire import TapDecodeError, TapLoss, TapMicro, decode

log = logging.getLogger("fpvdgs.dynlink")


class TapProtocol(asyncio.DatagramProtocol):
    """Decode + dispatch. Malformed / unknown-version datagrams are counted
    and ignored (one WARN, then silent) — an unknown version also means a
    partial deploy, and silence here lets the staleness fallback take over."""

    def __init__(self, on_micro, on_loss) -> None:
        self._on_micro = on_micro
        self._on_loss = on_loss
        self.malformed = 0
        self.ignored = 0
        self._warned = False

    def datagram_received(self, data: bytes, addr) -> None:
        try:
            rec = decode(data)
        except TapDecodeError as e:
            self.malformed += 1
            if not self._warned:
                log.warning("tap: malformed datagram (%s) — counting silently from now on", e)
                self._warned = True
            return
        if rec is None:
            self.ignored += 1
            if not self._warned:
                log.warning("tap: unknown record type/version — wfb binary mismatch? ignoring")
                self._warned = True
            return
        if isinstance(rec, TapMicro):
            self._on_micro(rec)
        elif isinstance(rec, TapLoss):
            self._on_loss(rec)


class TapCapture:
    """Debug JSONL dump of decoded tap records (dynamicLink.tap.captureRaw).

    One fixed file, truncated on controller start, size-capped — a bench
    debugging aid, not a flight log (the flightlog carries the per-tick
    story; this carries the raw tap input for TapReplayClient)."""

    MAX_BYTES = 32 * 1024 * 1024

    def __init__(self, path: str = "/media/dvr/log/dynamic-link/tap_capture.jsonl") -> None:
        self._fh = None
        self._bytes = 0
        try:
            self._fh = open(path, "w")
        except OSError as e:
            log.warning("tap capture: open %s failed: %s", path, e)

    def write(self, kind: str, rec) -> None:
        if self._fh is None or self._bytes >= self.MAX_BYTES:
            return
        import dataclasses
        import json

        line = json.dumps({"type": kind, "rec": dataclasses.asdict(rec)}) + "\n"
        try:
            self._fh.write(line)
            self._bytes += len(line)
        except OSError:
            self._fh = None

    def close(self) -> None:
        if self._fh is not None:
            try:
                self._fh.close()
            except OSError:
                pass
            self._fh = None


class TapReplayClient:
    """Replay a TapCapture JSONL file through the tap callbacks (offline
    selector validation, sibling of stats_client.ReplayClient)."""

    def __init__(self, path: str, on_micro, on_loss) -> None:
        self._path = path
        self._on_micro = on_micro
        self._on_loss = on_loss

    def run(self) -> None:
        import json

        from .stats_client import RxAnt

        with open(self._path) as fd:
            for line in fd:
                line = line.strip()
                if not line:
                    continue
                try:
                    obj = json.loads(line)
                except ValueError:
                    continue
                rec = obj.get("rec") or {}
                if obj.get("type") == "micro":
                    ants = [RxAnt(**a) for a in rec.pop("rx_ant_stats", [])]
                    self._on_micro(TapMicro(rx_ant_stats=ants, **rec))
                elif obj.get("type") == "loss":
                    self._on_loss(TapLoss(**rec))
