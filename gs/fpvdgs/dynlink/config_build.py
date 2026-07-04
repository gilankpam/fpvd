# gs/fpvdgs/dynlink/config_build.py
"""Map fpvd's `dynamicLink` config block onto the policy/aggregator objects
the controller consumes, and build the controller snapshot.

The block is explicit (no opaque `tuning` passthrough): `selector` and
`smoothing` carry the tunable knobs; `flightlog` exposes only an `enabled`
toggle (its internals are frozen code constants);
learned-prior exposes its learning knobs (settleTicks/alphaTighten/alphaRelax/minSamples/recencyDecay);
predictive + persistence internals stay frozen. camelCase JSON maps to the
dataclasses' snake_case fields."""

from __future__ import annotations

from .flightlog import FlightLogConfig
from .learned_prior import LearnedPriorConfig
from .policy import PolicyConfig, SelectorConfig
from .signals import SignalAggregator


def build_policy_config(block: dict) -> PolicyConfig:
    sel = block.get("selector", {}) or {}
    d = SelectorConfig()
    selector = SelectorConfig(
        video_demote_per=float(sel.get("videoDemotePer", d.video_demote_per)),
        emergency_fec_pressure=float(sel.get("emergencyFecPressure", d.emergency_fec_pressure)),
        max_mcs=int(block.get("maxMcs", d.max_mcs)),
        hold_modes_down_ms=int(sel.get("holdModesDownMs", d.hold_modes_down_ms)),
        min_between_changes_ms=int(sel.get("minBetweenChangesMs", d.min_between_changes_ms)),
        starvation_windows=int(sel.get("starvationWindows", d.starvation_windows)),
        snr_promote_margin_db=float(sel.get("snrPromoteMarginDb", d.snr_promote_margin_db)),
        snr_demote_margin_db=float(sel.get("snrDemoteMarginDb", d.snr_demote_margin_db)),
        demote_cooldown_windows=int(sel.get("demoteCooldownWindows", d.demote_cooldown_windows)),
        flap_base_backoff_ms=int(sel.get("flapBaseBackoffMs", d.flap_base_backoff_ms)),
        flap_backoff_mult=float(sel.get("flapBackoffMult", d.flap_backoff_mult)),
        flap_backoff_cap_ms=int(sel.get("flapBackoffCapMs", d.flap_backoff_cap_ms)),
        flap_reset_clean_dwell_ticks=int(
            sel.get("flapResetCleanDwellTicks", d.flap_reset_clean_dwell_ticks)
        ),
        trial_window_ms=int(sel.get("trialWindowMs", d.trial_window_ms)),
        promote_dwell_ticks=int(sel.get("promoteDwellTicks", d.promote_dwell_ticks)),
        promote_slope_min=float(sel.get("promoteSlopeMin", d.promote_slope_min)),
        collapse_delta_db=float(sel.get("collapseDeltaDb", d.collapse_delta_db)),
        snapback_recover_margin_db=float(
            sel.get("snapbackRecoverMarginDb", d.snapback_recover_margin_db)
        ),
        confirm_ttl_ms=int(sel.get("confirmTtlMs", d.confirm_ttl_ms)),
        flap_snr_release_db=float(sel.get("flapSnrReleaseDb", d.flap_snr_release_db)),
        flap_decay_ms=int(sel.get("flapDecayMs", d.flap_decay_ms)),
    )
    fl = block.get("flightlog", {}) or {}
    # flightlog internals are frozen — read only `enabled`.
    flightlog = FlightLogConfig(enabled=bool(fl.get("enabled", True)))
    # learned-prior (knee model): expose the learning knobs for in-flight
    # tuning; the predictive-machinery + persist internals stay at defaults.
    lp = block.get("learnedPrior", {}) or {}
    dlp = LearnedPriorConfig()
    learned_prior = LearnedPriorConfig(
        settle_ticks=int(lp.get("settleTicks", dlp.settle_ticks)),
        viable_loss=float(lp.get("viableLoss", dlp.viable_loss)),
        alpha_tighten=float(lp.get("alphaTighten", dlp.alpha_tighten)),
        alpha_relax=float(lp.get("alphaRelax", dlp.alpha_relax)),
        min_samples=float(lp.get("minSamples", dlp.min_samples)),
        recency_decay=float(lp.get("recencyDecay", dlp.recency_decay)),
    )
    return PolicyConfig(
        selector=selector,
        learned_prior=learned_prior,
        flightlog=flightlog,
        link_width=int(block.get("linkWidth", 20)),
    )


def build_aggregator(block: dict) -> SignalAggregator:
    s = block.get("smoothing", {}) or {}
    d = SignalAggregator()
    return SignalAggregator(
        ewma_alpha_rssi=float(s.get("ewmaAlphaRssi", d.ewma_alpha_rssi)),
        ewma_alpha_fec=float(s.get("ewmaAlphaFec", d.ewma_alpha_fec)),
        ewma_alpha_burst=float(s.get("ewmaAlphaBurst", d.ewma_alpha_burst)),
        starvation_threshold_pps=float(s.get("starvationThresholdPps", d.starvation_threshold_pps)),
    )


def make_dl_snapshot(effective: dict) -> dict:
    """Self-contained snapshot the controller consumes. The drone decision
    UDP target is drone.host : dynamicLink.dronePort (9999 default)."""
    block = dict(effective.get("dynamicLink", {}))
    block["droneAddr"] = effective.get("drone", {}).get("host", "10.5.0.10")
    block["dronePort"] = int(block.get("dronePort") or 9999)
    # Channel width (link.width) — keys the per-width learned prior and is logged
    # per tick. The dynamicLink block itself stays width-agnostic.
    block["linkWidth"] = int(effective.get("link", {}).get("width", 20))
    return block
