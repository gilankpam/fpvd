"""Per-card radio params via `iw dev <iface> info` -- local cards run it
locally, remote cards over SSH. On-demand + slow (SSH); never on the hot
/gs/status path. Cards are queried concurrently so latency ~ one round-trip.
"""

from __future__ import annotations

import subprocess
from concurrent.futures import ThreadPoolExecutor

from .status import parse_iw_info
from .wfb.cards import Card, parse_cards, resolve_cards

SSH_OPTS = [
    "-o",
    "BatchMode=yes",
    "-o",
    "StrictHostKeyChecking=accept-new",
    "-o",
    "ConnectTimeout=4",
]


def _local_iw_argv(iface: str) -> list[str]:
    return ["iw", "dev", iface, "info"]


def _remote_iw_argv(card: Card) -> list[str]:
    argv = ["ssh", *SSH_OPTS, "-p", str(card.ssh_port)]
    if card.ssh_key:
        argv += ["-i", card.ssh_key]
    argv += [f"{card.ssh_user}@{card.host}", f"iw dev {card.iface} info"]
    return argv


def card_radio(card: Card, *, runner=subprocess.run, timeout: float = 6.0) -> dict:
    """One card's radio entry. `runner` is injected for tests (defaults to
    subprocess.run). Never raises: an ssh/iw failure or timeout -> reachable
    False + no radio."""
    argv = _local_iw_argv(card.iface) if card.is_local else _remote_iw_argv(card)
    entry = {
        "host": card.host,  # None for local
        "iface": card.iface,
        "local": card.is_local,
    }
    try:
        proc = runner(argv, capture_output=True, text=True, timeout=timeout)
    except (OSError, subprocess.SubprocessError):
        entry["reachable"] = False
        return entry
    if proc.returncode != 0:
        entry["reachable"] = False
        return entry
    entry["reachable"] = True
    entry["radio"] = parse_iw_info(proc.stdout)
    return entry


def query_cards(cards: list[Card], *, runner=subprocess.run, timeout: float = 6.0) -> list[dict]:
    """All cards' radio entries, queried concurrently (list order preserved)."""
    if not cards:
        return []
    with ThreadPoolExecutor(max_workers=min(8, len(cards))) as ex:
        return list(ex.map(lambda c: card_radio(c, runner=runner, timeout=timeout), cards))


def nodes_status(
    effective: dict, nic_detector=None, *, runner=subprocess.run, timeout: float = 6.0
) -> dict:
    """Build the /gs/nodes payload from the effective config.

    A literal "auto" (all-local auto-detect) with no `nic_detector` yields no
    nodes rather than reaching for the real local-NIC detector -- an on-demand
    caller with no detector to hand (or a test) gets an empty list, not a
    surprise subprocess call. When a detector IS supplied, it's threaded
    through to `resolve_cards` so "auto" expands to the same local cards the
    rest of the GS would use.
    """
    parsed = parse_cards(effective.get("link", {}))
    if parsed == "auto":
        if nic_detector is None:
            return {"nodes": []}
        cards = resolve_cards(effective, nic_detector=nic_detector)
    else:
        cards = parsed
    return {"nodes": query_cards(list(cards), runner=runner, timeout=timeout)}
