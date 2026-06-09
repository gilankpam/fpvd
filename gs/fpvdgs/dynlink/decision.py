"""Decision record emitted by the policy engine on every tick."""
from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import Any


@dataclass
class Decision:
    """What the controller *would* apply if the wire were connected.

    Phase 3b: the wire carries `{mcs}` only — the drone computes its own
    bitrate / FEC / depth / tx_power locally. `reason` and
    `signals_snapshot` are kept for telemetry / status, not the wire.
    """
    timestamp: float
    mcs: int
    reason: str = ""
    signals_snapshot: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return asdict(self)
