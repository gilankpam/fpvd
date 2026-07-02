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
