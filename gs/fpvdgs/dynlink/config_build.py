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
    CooldownConfig, FECBounds, GateConfig, LeadingLoopConfig,
    PolicyConfig, ProfileSelectionConfig, SafeDefaults,
)
from .bitrate import BitrateConfig
from .dynamic_fec import DynamicFecConfig
from .predictor import PredictorConfig
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


def _build_policy_config(raw: dict) -> PolicyConfig:
    leading_raw = raw.get("leading_loop", {})
    gate_raw = raw.get("gate", {})
    selection_raw = raw.get("profile_selection", {})
    cooldown_raw = raw.get("cooldown", {})
    fec_raw = raw.get("fec", {})
    safe_raw = raw.get("safe_defaults", {})
    video_raw = raw.get("video", {})

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

    gate = GateConfig(
        snr_ema_alpha=float(gate_raw.get("snr_ema_alpha", 0.3)),
        snr_slope_alpha=float(gate_raw.get("snr_slope_alpha", 0.3)),
        snr_predict_horizon_ticks=float(
            gate_raw.get("snr_predict_horizon_ticks", 3.0)
        ),
        snr_safety_margin=float(gate_raw.get("snr_safety_margin", 3.0)),
        loss_margin_weight=float(gate_raw.get("loss_margin_weight", 20.0)),
        fec_margin_weight=float(gate_raw.get("fec_margin_weight", 5.0)),
        hysteresis_up_db=float(gate_raw.get("hysteresis_up_db", 2.5)),
        hysteresis_down_db=float(gate_raw.get("hysteresis_down_db", 1.0)),
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
    cooldown = CooldownConfig(
        min_change_interval_ms_fec=float(
            cooldown_raw.get("min_change_interval_ms_fec", 200.0)
        ),
        min_change_interval_ms_depth=float(
            cooldown_raw.get("min_change_interval_ms_depth", 200.0)
        ),
        min_change_interval_ms_radio=float(
            cooldown_raw.get("min_change_interval_ms_radio", 500.0)
        ),
        min_change_interval_ms_cross=float(
            cooldown_raw.get("min_change_interval_ms_cross", 50.0)
        ),
    )
    fec = FECBounds(
        depth_max=int(fec_raw.get("depth_max", 3)),
    )

    fec_kbounds_raw = fec_raw.get("k_bounds", {})
    max_red = float(fec_raw.get("max_redundancy_ratio", 1.0))
    hard_bpf = 1.0 + max_red
    bpf = float(fec_raw.get("blocks_per_frame", hard_bpf))
    if bpf < hard_bpf:
        log.warning(
            "config: fec.blocks_per_frame=%.2f is below "
            "1 + max_redundancy_ratio (%.2f) — block_fill will exceed "
            "one frame period under sustained loss. Set blocks_per_frame "
            ">= %.2f for the hard latency bound.",
            bpf, hard_bpf, hard_bpf,
        )
    dynamic_fec = DynamicFecConfig(
        k_min=int(fec_kbounds_raw.get("min", 4)),
        k_max=int(fec_kbounds_raw.get("max", 16)),
        base_redundancy_ratio=float(fec_raw.get("base_redundancy_ratio", 0.5)),
        max_redundancy_ratio=max_red,
        blocks_per_frame=bpf,
        n_loss_threshold=float(fec_raw.get("n_loss_threshold", 0.02)),
        n_loss_windows=int(fec_raw.get("n_loss_windows", 3)),
        n_loss_step=int(fec_raw.get("n_loss_step", 1)),
        n_recover_windows=int(fec_raw.get("n_recover_windows", 10)),
        n_recover_step=int(fec_raw.get("n_recover_step", 1)),
        max_n_escalation=int(fec_raw.get("max_n_escalation", 4)),
    )

    # Legacy fec.* keys: present in old gs.yaml configs but no longer
    # wired. Log a warning so the operator cleans them up.
    _legacy_fec_keys = (
        "mtu_bytes",                    # now drone-reported via DLHE (P4a)
        "fec_block_fill_ms_target",     # removed during the static-table era
        "n_min", "n_preempt_step",      # removed during the static-table era
    )
    _legacy_fec_reasons = {
        "mtu_bytes": "MTU is now reported by the drone at runtime",
        "fec_block_fill_ms_target": "block-fill is now bounded by k_bounds.max",
        "n_min": "absorbed into k_bounds.min",
        "n_preempt_step": "preemptive escalation removed",
    }
    for k in _legacy_fec_keys:
        if k in fec_raw:
            log.warning(
                "config: ignoring legacy fec.%s — %s", k,
                _legacy_fec_reasons.get(k, "deprecated"),
            )

    # Legacy encoder keys: encoder.fps moved drone-side.
    encoder_raw = raw.get("encoder", {})
    if "fps" in encoder_raw:
        log.warning(
            "config: ignoring legacy encoder.fps — FPS is now reported "
            "by the drone via DLHE (P4a)"
        )
    safe_video = safe_raw.get("video", {})
    safe = SafeDefaults(
        k=int(safe_video.get("k", 8)),
        n=int(safe_video.get("n", 12)),
        depth=int(safe_raw.get("depth", 1)),
        mcs=int(safe_raw.get("mcs", 1)),
    )
    predictor = PredictorConfig(
        per_packet_airtime_us=float(video_raw.get("per_packet_airtime_us", 80.0)),
    )
    policy_raw = raw.get("policy", {})
    bitrate_raw = policy_raw.get("bitrate", {})
    if "base_redundancy_ratio" in bitrate_raw:
        log.warning(
            "policy.bitrate.base_redundancy_ratio is deprecated and "
            "ignored; fec.base_redundancy_ratio is now authoritative "
            "(bitrate is derived from live (k, n) per the bitrate-aware "
            "FEC design)."
        )
    try:
        bitrate = BitrateConfig(
            utilization_factor=float(bitrate_raw.get("utilization_factor", 0.8)),
            min_bitrate_kbps=int(bitrate_raw.get("min_bitrate_kbps", 1000)),
            max_bitrate_kbps=int(bitrate_raw.get("max_bitrate_kbps", 24000)),
        )
    except ValueError as e:
        raise ValueError(f"policy.bitrate.{e}") from e
    return PolicyConfig(
        leading=leading,
        gate=gate,
        selection=selection,
        cooldown=cooldown,
        fec=fec,
        safe=safe,
        bitrate=bitrate,
        dynamic_fec=dynamic_fec,
        predictor=predictor,
        max_latency_ms=float(video_raw.get("max_latency_ms", 50.0)),
        starvation_windows=int(policy_raw.get("starvation_windows", 5)),
    )


def _build_aggregator(raw: dict) -> SignalAggregator:
    s = raw.get("smoothing", {})
    gate = raw.get("gate", {})
    starv = s.get("starvation_threshold_pps", 50.0)
    # snr_slope alpha lives under [gate] so operators can tune it
    # alongside the other gate knobs; aggregator just consumes it.
    return SignalAggregator(
        ewma_alpha_rssi=float(s.get("ewma_alpha_rssi", 0.2)),
        ewma_alpha_fec=float(s.get("ewma_alpha_fec", 0.2)),
        ewma_alpha_burst=float(s.get("ewma_alpha_burst", 0.1)),
        ewma_alpha_snr_slope=float(gate.get("snr_slope_alpha", 0.3)),
        starvation_threshold_pps=float(starv),
    )
