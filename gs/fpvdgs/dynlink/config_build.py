# gs/fpvdgs/dynlink/config_build.py
"""Map fpvd's `dynamicLink` config block onto the policy/aggregator objects
the controller consumes, and build the controller snapshot.

The block is explicit (no opaque `tuning` passthrough): `selector` and
`smoothing` carry the tunable knobs; `flightlog`/`rssiNorm` expose only an
`enabled` toggle (their internals are frozen code constants); learned-prior
internals are frozen entirely. camelCase JSON maps to the dataclasses'
snake_case fields."""
from __future__ import annotations

from .flightlog import FlightLogConfig
from .learned_prior import LearnedPriorConfig
from .policy import PolicyConfig, SelectorConfig
from .signals import RssiNormConfig, SignalAggregator


def build_policy_config(block: dict) -> PolicyConfig:
    sel = block.get("selector", {}) or {}
    d = SelectorConfig()
    selector = SelectorConfig(
        probe_viable_threshold=float(sel.get("probeViableThreshold", d.probe_viable_threshold)),
        probe_freshness_ms=float(sel.get("probeFreshnessMs", d.probe_freshness_ms)),
        promote_debounce_windows=int(sel.get("promoteDebounceWindows", d.promote_debounce_windows)),
        video_demote_per=float(sel.get("videoDemotePer", d.video_demote_per)),
        emergency_fec_pressure=float(sel.get("emergencyFecPressure", d.emergency_fec_pressure)),
        max_mcs=int(block.get("maxMcs", d.max_mcs)),
        hold_modes_down_ms=int(sel.get("holdModesDownMs", d.hold_modes_down_ms)),
        min_between_changes_ms=int(sel.get("minBetweenChangesMs", d.min_between_changes_ms)),
        starvation_windows=int(sel.get("starvationWindows", d.starvation_windows)),
    )
    fl = block.get("flightlog", {}) or {}
    # flightlog internals are frozen — read only `enabled`.
    flightlog = FlightLogConfig(enabled=bool(fl.get("enabled", True)))
    return PolicyConfig(
        selector=selector,
        learned_prior=LearnedPriorConfig(),   # frozen: always-on, internal defaults
        flightlog=flightlog,
    )


def build_aggregator(block: dict) -> SignalAggregator:
    s = block.get("smoothing", {}) or {}
    rn = block.get("rssiNorm", {}) or {}
    d = SignalAggregator()
    # rssiNorm curve is frozen — read only `enabled` (the rollback toggle).
    rssi_norm = RssiNormConfig(enabled=bool(rn.get("enabled", True)))
    return SignalAggregator(
        ewma_alpha_rssi=float(s.get("ewmaAlphaRssi", d.ewma_alpha_rssi)),
        ewma_alpha_fec=float(s.get("ewmaAlphaFec", d.ewma_alpha_fec)),
        ewma_alpha_burst=float(s.get("ewmaAlphaBurst", d.ewma_alpha_burst)),
        starvation_threshold_pps=float(
            s.get("starvationThresholdPps", d.starvation_threshold_pps)),
        rssi_norm=rssi_norm,
    )


def make_dl_snapshot(effective: dict) -> dict:
    """Self-contained snapshot the controller consumes. The drone decision
    UDP target is drone.host : dynamicLink.dronePort (9999 default)."""
    block = dict(effective.get("dynamicLink", {}))
    block["droneAddr"] = effective.get("drone", {}).get("host", "10.5.0.10")
    block["dronePort"] = int(block.get("dronePort") or 9999)
    return block
