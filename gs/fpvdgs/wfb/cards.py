"""The card model: `link.cards` is a flat list of wifi cards, each either
locally attached to this GS host (`host is None`) or reached over SSH on a
remote node (Phase 2). `link.wlans` is the pre-migration legacy overlay key
— a plain list of local iface names (or "auto") — kept alive only so old
overlays/tests keep working; `parse_cards` still consumes it.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Card:
    host: str | None
    iface: str
    ssh_user: str = "root"
    ssh_port: int = 22
    ssh_key: str | None = None
    txpower_dbm: float | str | None = None  # "off" => RX-only, no TX child

    @property
    def is_local(self) -> bool:
        return self.host is None

    @property
    def is_rx_only(self) -> bool:
        return self.txpower_dbm == "off"


def _card_from_dict(d: dict) -> Card:
    if "iface" not in d:
        raise ValueError(f"card object requires 'iface': {d!r}")
    return Card(
        host=d.get("host"),
        iface=d["iface"],
        ssh_user=d.get("sshUser", "root"),
        ssh_port=int(d.get("sshPort", 22)),
        ssh_key=d.get("sshKey"),
        txpower_dbm=d.get("txPowerDbm"),
    )


def _parse_entries(entries: list) -> list[Card]:
    cards = []
    for entry in entries:
        if isinstance(entry, str):
            cards.append(Card(host=None, iface=entry))
        elif isinstance(entry, dict):
            cards.append(_card_from_dict(entry))
        else:
            raise ValueError(f"invalid card entry: {entry!r}")
    return cards


def parse_cards(link: dict) -> list[Card] | str:
    """Parse `link.cards` (preferred), falling back to the legacy `link.wlans`
    overlay key, into a list of Card, or "auto" (expansion is resolve_cards'
    job).

    `cards` wins over `wlans` whenever `cards` is a concrete list. Otherwise
    (cards absent, None, or the "auto" placeholder — which is what a merged
    effective config always carries by default) a concrete `wlans` list is
    treated as a legacy local-card override. If neither key yields a concrete
    list, the result is "auto". Since "auto" only ever auto-detects LOCAL
    NICs, an "auto" resolution that still finds a remote card lurking in the
    legacy `wlans` list is a contradiction and raises.
    """
    cards_raw = link.get("cards")
    if isinstance(cards_raw, list):
        return _parse_entries(cards_raw)
    wlans_raw = link.get("wlans")
    if isinstance(wlans_raw, list):
        cards = _parse_entries(wlans_raw)
        if any(not c.is_local for c in cards):
            raise ValueError("link.wlans (legacy) cannot contain a remote card entry")
        return cards
    return "auto"


def resolve_cards(effective: dict, nic_detector=None) -> list[Card]:
    """Expand `link.cards`/`link.wlans` to a concrete list of Card, expanding
    "auto" via `nic_detector` (default: runner_supervisor's `_wfb_nics`)."""
    parsed = parse_cards(effective.get("link", {}))
    if parsed != "auto":
        return parsed
    if nic_detector is None:
        from ..runner_supervisor import _wfb_nics as nic_detector
    return [Card(host=None, iface=iface) for iface in nic_detector()]


def local_ifaces(cards: list[Card]) -> list[str]:
    return [c.iface for c in cards if c.is_local]


def remote_cards(cards: list[Card]) -> list[Card]:
    return [c for c in cards if not c.is_local]


def has_remote(cards: list[Card]) -> bool:
    return any(not c.is_local for c in cards)
