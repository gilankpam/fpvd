"""Size and timeout-based message aggregation queue for wfb data plane."""

import asyncio
import logging

logger = logging.getLogger(__name__)


class AggQueue:
    """Batches small messages up to a size limit and time limit.

    Semantics ported from wfb-ng's ProxyProtocol.messageReceived/flush_queue.
    """

    def __init__(self, max_size, timeout_s, send):
        """Initialize the aggregation queue.

        Args:
            max_size: Maximum bytes per batch. If None, passthrough mode (no aggregation).
            timeout_s: Timeout in seconds. If 0 or None, passthrough mode.
            send: Callable(bytes) to send the batch.
        """
        self.max_size = max_size
        self.timeout_s = timeout_s
        self.send = send
        self.queue = []
        self.queue_size = 0
        self.timer = None

    def put(self, data: bytes) -> None:
        """Queue a message for aggregation or send immediately.

        Args:
            data: Message bytes to send or queue.
        """
        # Passthrough mode: no aggregation
        if self.max_size is None or not self.timeout_s:
            self.send(data)
            return

        # Oversize message: drop with warning
        if len(data) > self.max_size:
            logger.warning("Message too big: %d > %d", len(data), self.max_size)
            return

        # Message doesn't fit in current queue: flush first
        if self.queue_size + len(data) > self.max_size:
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            if self.queue:
                self.send(b"".join(self.queue))
                self.queue = []
                self.queue_size = 0

        # Queue the message
        self.queue.append(data)
        self.queue_size += len(data)

        # Arm timer if not already armed
        if self.timeout_s and self.timer is None:
            loop = asyncio.get_running_loop()
            self.timer = loop.call_later(self.timeout_s, self.flush)

    def flush(self) -> None:
        """Send the aggregated queue immediately."""
        if self.queue_size > 0:
            if self.timer is not None:
                self.timer.cancel()
                self.timer = None
            self.send(b"".join(self.queue))
            self.queue = []
            self.queue_size = 0

    def close(self) -> None:
        """Cancel any pending timer."""
        if self.timer is not None:
            self.timer.cancel()
            self.timer = None
