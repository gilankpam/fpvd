# gs/fpvdgs/dynlink/config_build.py
"""Translate fpvd's `dynamicLink` config block into the policy/aggregator
objects the lifted control core expects, and build the controller snapshot.

The lifted `_build_policy_config(raw)` / `_build_aggregator(raw)` consume a
dict shaped like the old gs.yaml. We construct that `raw` from the opaque
`tuning` passthrough, then overlay the curated top-level keys so they always
win."""
from __future__ import annotations

import copy
import logging
from pathlib import Path
from urllib.parse import urlparse

from .policy import (
    GateConfig, LeadingLoopConfig,
    PolicyConfig, ProfileSelectionConfig,
)
from .profile import RadioProfile, load_profile
from .signals import SignalAggregator

log = logging.getLogger("fpvdgs.dynlink")

PROFILES_DIR = Path(__file__).resolve().parent / "profiles"


def _raw_from_block(block: dict) -> dict:
    """Build a gs.yaml-shaped `raw` dict: tuning is the base, curated keys
    are overlaid so they always win over any tuning attempt."""
    raw = copy.deepcopy(block.get("tuning") or {})
    leading = raw.setdefault("leading_loop", {})
    gate = raw.setdefault("gate", {})
    if "bandwidth" in block:
        leading["bandwidth"] = int(block["bandwidth"])
    tx = block.get("txpower") or {}
    if "min" in tx:
        leading["tx_power_min_dBm"] = float(tx["min"])
    if "max" in tx:
        leading["tx_power_max_dBm"] = float(tx["max"])
    if "maxMcs" in block:
        gate["max_mcs"] = int(block["maxMcs"])
    return raw


def build_policy_config(block: dict) -> PolicyConfig:
    return _build_policy_config(_raw_from_block(block))


def build_aggregator(block: dict) -> SignalAggregator:
    return _build_aggregator(_raw_from_block(block))


def resolve_profile(block: dict) -> RadioProfile:
    name = block.get("radioProfile", "m8812eu2")
    return load_profile(name, [PROFILES_DIR])


def make_dl_snapshot(effective: dict) -> dict:
    """Self-contained snapshot the controller consumes. Resolves the drone
    UDP target: explicit dynamicLink.droneAddr wins, else the host from
    drone.endpoint; port defaults to 9999 (the fpvd drone's listener)."""
    block = dict(effective.get("dynamicLink", {}))
    endpoint = effective.get("drone", {}).get("endpoint", "http://10.5.0.10:8080")
    host = urlparse(endpoint).hostname or "10.5.0.10"
    block["droneAddr"] = block.get("droneAddr") or host
    block["dronePort"] = int(block.get("dronePort") or 9999)
    return block


_DEPRECATED_LEADING_KEYS = {
    # Old hysteresis / inhibit knobs — superseded by gate / profile_selection.
    "snr_margin_db", "snr_up_guard_db", "snr_up_hold_ms", "snr_down_hold_ms",
    "loss_margin_weight", "fec_margin_weight", "forced_drop_inhibit_ms",
    "mcs_max",  # moved to gate.max_mcs
    # Old RSSI closed loop / closed-loop power knobs — fully retired.
    "rssi_margin_db", "rssi_up_guard_db", "rssi_up_hold_ms",
    "rssi_down_hold_ms", "rssi_target_dBm", "rssi_deadband_db",
    "tx_power_cooldown_ms", "tx_power_freeze_after_mcs_ms",
    "tx_power_step_max_db", "tx_power_gain_up_db", "tx_power_gain_down_db",
}

_DEPRECATED_GATE_KEYS = {
    "snr_ema_alpha", "snr_slope_alpha", "snr_predict_horizon_ticks",
    "snr_safety_margin", "loss_margin_weight", "fec_margin_weight",
    "hysteresis_up_db", "hysteresis_down_db",
}

_DEPRECATED_PHASE3A_KEYS = {
    # bitrate/FEC/predictor knobs moved to the drone in Phase 3a.
    "utilization_factor", "min_bitrate_kbps", "max_bitrate_kbps",
    "base_redundancy_ratio", "max_redundancy_ratio", "blocks_per_frame",
    "depth_max", "n_loss_threshold", "n_loss_windows", "n_loss_step",
    "n_recover_windows", "n_recover_step", "max_n_escalation",
    "per_packet_airtime_us", "max_latency_ms",
}


def _build_policy_config(raw: dict) -> PolicyConfig:
    leading_raw = raw.get("leading_loop", {})
    gate_raw = raw.get("gate", {})
    selection_raw = raw.get("profile_selection", {})

    bitrate_raw = raw.get("policy", {}).get("bitrate", {})
    fec_raw = raw.get("fec", {})
    video_raw = raw.get("video", {})
    retired_present = sorted(
        {k for raw_sub in (bitrate_raw, fec_raw, video_raw)
         for k in _DEPRECATED_PHASE3A_KEYS
         if k in (raw_sub or {})}
    )
    if retired_present:
        log.warning(
            "bitrate/FEC/predictor knobs are now drone-local (Phase 3a) and "
            "ignored on the GS: %s", ", ".join(retired_present)
        )

    deprecated_present = sorted(
        k for k in _DEPRECATED_LEADING_KEYS if k in leading_raw
    )
    if deprecated_present:
        log.warning(
            "leading_loop has deprecated keys (ignored): %s. "
            "Migrate to the `gate:` / `profile_selection:` sections.",
            ", ".join(deprecated_present),
        )

    leading = LeadingLoopConfig(
        bandwidth=int(leading_raw.get("bandwidth", 20)),
        tx_power_min_dBm=float(leading_raw.get("tx_power_min_dBm", 5.0)),
        tx_power_max_dBm=float(leading_raw.get("tx_power_max_dBm", 23.0)),
        # Deprecated keys: parse-and-store so they round-trip but the
        # selector ignores them. Defaults match the old values so any
        # in-tree consumer relying on them still gets sensible numbers.
        mcs_max=int(leading_raw.get("mcs_max", 7)),
        snr_margin_db=float(leading_raw.get("snr_margin_db", 3.0)),
        snr_up_guard_db=float(leading_raw.get("snr_up_guard_db", 2.0)),
        snr_up_hold_ms=float(leading_raw.get("snr_up_hold_ms", 2000.0)),
        snr_down_hold_ms=float(leading_raw.get("snr_down_hold_ms", 500.0)),
        loss_margin_weight=float(leading_raw.get("loss_margin_weight", 20.0)),
        fec_margin_weight=float(leading_raw.get("fec_margin_weight", 20.0)),
        forced_drop_inhibit_ms=float(
            leading_raw.get("forced_drop_inhibit_ms", 5000.0)
        ),
        rssi_up_guard_db=float(leading_raw.get("rssi_up_guard_db", 3.0)),
        rssi_up_hold_ms=float(leading_raw.get("rssi_up_hold_ms", 2000.0)),
        rssi_down_hold_ms=float(leading_raw.get("rssi_down_hold_ms", 500.0)),
        rssi_target_dBm=float(leading_raw.get("rssi_target_dBm", -60.0)),
        rssi_deadband_db=float(leading_raw.get("rssi_deadband_db", 3.0)),
        tx_power_cooldown_ms=float(
            leading_raw.get("tx_power_cooldown_ms", 1000.0)
        ),
        tx_power_freeze_after_mcs_ms=float(
            leading_raw.get("tx_power_freeze_after_mcs_ms", 2000.0)
        ),
        tx_power_step_max_db=float(
            leading_raw.get("tx_power_step_max_db", 3.0)
        ),
        tx_power_gain_up_db=float(leading_raw.get("tx_power_gain_up_db", 1.0)),
        tx_power_gain_down_db=float(
            leading_raw.get("tx_power_gain_down_db", 1.0)
        ),
    )

    dep_gate = sorted(k for k in _DEPRECATED_GATE_KEYS if k in gate_raw)
    if dep_gate:
        log.warning(
            "gate has deprecated SNR knobs (ignored): %s. "
            "MCS is now probe-driven.", ", ".join(dep_gate)
        )

    gate = GateConfig(
        probe_viable_threshold=float(gate_raw.get("probe_viable_threshold", 0.99)),
        probe_freshness_ms=float(gate_raw.get("probe_freshness_ms", 500.0)),
        promote_debounce_windows=int(gate_raw.get("promote_debounce_windows", 3)),
        video_demote_per=float(gate_raw.get("video_demote_per", 0.05)),
        emergency_loss_rate=float(gate_raw.get("emergency_loss_rate", 0.05)),
        emergency_fec_pressure=float(
            gate_raw.get("emergency_fec_pressure", 0.80)
        ),
        max_mcs=int(gate_raw.get("max_mcs", 7)),
        max_mcs_step_up=int(gate_raw.get("max_mcs_step_up", 1)),
    )

    if "hold_fallback_mode_ms" in selection_raw:
        log.warning(
            "profile_selection.hold_fallback_mode_ms is deprecated and no "
            "longer affects controller behavior — MCS=0 → 1 climbs now use "
            "the unified confidence-loop gate. Remove the key from gs.yaml."
        )

    selection = ProfileSelectionConfig(
        hold_fallback_mode_ms=int(
            selection_raw.get("hold_fallback_mode_ms", 1000)
        ),
        hold_modes_down_ms=int(selection_raw.get("hold_modes_down_ms", 2000)),
        min_between_changes_ms=int(
            selection_raw.get("min_between_changes_ms", 200)
        ),
        fast_downgrade=bool(selection_raw.get("fast_downgrade", True)),
        upward_confidence_loops=int(
            selection_raw.get("upward_confidence_loops", 4)
        ),
    )

    policy_raw = raw.get("policy", {})
    return PolicyConfig(
        leading=leading,
        gate=gate,
        selection=selection,
        starvation_windows=int(policy_raw.get("starvation_windows", 5)),
    )


def _build_aggregator(raw: dict) -> SignalAggregator:
    s = raw.get("smoothing", {})
    starv = s.get("starvation_threshold_pps", 50.0)
    return SignalAggregator(
        ewma_alpha_rssi=float(s.get("ewma_alpha_rssi", 0.2)),
        ewma_alpha_fec=float(s.get("ewma_alpha_fec", 0.2)),
        ewma_alpha_burst=float(s.get("ewma_alpha_burst", 0.1)),
        starvation_threshold_pps=float(starv),
    )
