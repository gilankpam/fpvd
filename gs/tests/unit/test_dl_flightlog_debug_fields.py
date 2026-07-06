"""Flight-log debug fields: failure classifier, selector state (clean_dwell,
trial, snapback_tgt), slope/knees, and predictive internals — the data
needed to replay a promote→demote oscillation offline."""

from __future__ import annotations

import json

from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.policy import Policy, PolicyConfig
from fpvdgs.dynlink.signals import Signals


def _profile():
    return "m8812eu2"  # Policy takes the adapter id string (learned-prior key)


def _cfg(tmp_path, **lp):
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path), **lp),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def _sig(rssi, ts=1.0):
    return Signals(rssi=rssi, residual_loss_w=0.0, fec_work=0.0, link_starved_w=False, timestamp=ts)


def _sig_snr(snr, ts=1.0):
    return Signals(snr=snr, residual_loss_w=0.0, fec_work=0.0, link_starved_w=False, timestamp=ts)


def _records(tmp_path):
    files = sorted((tmp_path / "fl").glob("*.jsonl"))
    assert files, "expected a flight-log file"
    with open(files[-1]) as f:
        return [json.loads(line) for line in f if line.strip()]


def test_record_carries_new_selector_state_fields(tmp_path):
    """The record carries the new probe-less selector observability fields:
    fail_class, trial, snapback_tgt, clean_dwell (replacing probe/promote_clean)."""
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig(-50.0))
    p.close()
    last = _records(tmp_path)[-1]
    assert "fail_class" in last  # None when no loss-demote this tick
    assert "trial" in last  # int|None: current trial rung
    assert "snapback_tgt" in last  # int|None: snap-back target this tick
    assert "clean_dwell" in last  # int: clean consecutive ticks at current rung
    assert "probe" not in last  # removed: probe is no longer consulted for promotes
    assert "promote_clean" not in last  # removed: replaced by clean_dwell


def test_record_carries_slope_and_knees(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3), prof)
    for _ in range(5):
        p.learned_prior.ingest(snr=30.0, operating_mcs=5, operating_clean=True, settled=True)
    p.tick(_sig_snr(30.0, ts=1.0))
    p.tick(_sig_snr(28.0, ts=1.1))
    p.close()
    recs = _records(tmp_path)
    assert recs[0]["slope"] == 0.0
    assert recs[1]["slope"] == -2.0
    assert isinstance(recs[1]["snr_knees"], list) and len(recs[1]["snr_knees"]) == 8
    assert recs[1]["prior_learn"] is False


def test_record_slope_none_when_no_snr(tmp_path):
    # cold SNR prior → no demote intent; no SNR → slope is None.
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig_snr(30.0, ts=1.0))
    p.tick(_sig_snr(None, ts=1.1))
    p.close()
    recs = _records(tmp_path)
    assert recs[0]["slope"] == 0.0  # one sample -> flat
    assert recs[1]["slope"] is None  # no SNR this tick


def test_record_carries_clean_dwell_counter(tmp_path):
    # clean_dwell counts consecutive clean ticks at the current rung (boot MCS 1).
    # It starts at 0 and increments each clean tick; the record shows the post-tick value.
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig(-50.0, ts=1.0))
    p.tick(_sig(-50.0, ts=1.1))
    p.close()
    recs = _records(tmp_path)
    assert recs[0]["clean_dwell"] == 1
    assert recs[1]["clean_dwell"] == 2


def test_logged_slope_is_least_squares_not_single_tick(tmp_path):
    """A lone SNR spike barely moves the logged slope (least-squares over a
    window) — the old single-tick delta would log the full +5 dB jump."""
    p = Policy(_cfg(tmp_path), _profile())
    for snr, ts in [(30.0, 1.0), (30.0, 1.1), (30.0, 1.2), (30.0, 1.3), (35.0, 1.4)]:
        p.tick(_sig_snr(snr, ts=ts))
    p.close()
    last = _records(tmp_path)[-1]
    # lsq over [30,30,30,30,35] = +1.0  (single-tick delta would be +5.0)
    assert abs(last["slope"] - 1.0) < 1e-6


def test_logged_slope_uses_only_the_rolling_window(tmp_path):
    """Samples older than the default 10-tick window must not affect the slope:
    5 flat ticks then a 10-tick -1/tick ramp → slope -1.0 (the flat prefix has
    rolled out). If the prefix leaked in, the slope would be shallower."""
    p = Policy(_cfg(tmp_path), _profile())
    ts = 1.0
    for _ in range(5):  # flat prefix — rolls out of the window
        p.tick(_sig_snr(30.0, ts=ts))
        ts += 0.1
    for i in range(10):  # -1 dB/tick ramp fills the window
        p.tick(_sig_snr(30.0 - i, ts=ts))
        ts += 0.1
    p.close()
    last = _records(tmp_path)[-1]
    assert abs(last["slope"] - (-1.0)) < 1e-6


def test_record_carries_predict_gated_flag(tmp_path):
    """predict_gated is True when current rung confidently unviable but the
    slope-direction gate blocks the demote (flat SNR = no real fade); the reason
    carries no predict_demote."""
    prof = _profile()
    p = Policy(
        _cfg(tmp_path, min_samples=3, predictive_horizon_ticks=3, predictive_debounce_windows=2),
        prof,
    )
    p.learned_prior._snr_model._knee[5] = 38.0
    p.learned_prior._snr_model._count[5] = 12.0
    p.leading.state.current_mcs = 5
    p.tick(_sig_snr(34.0, ts=1.0))
    p.tick(_sig_snr(34.0, ts=1.1))
    p.close()
    last = _records(tmp_path)[-1]
    assert last["predict_gated"] is True
    assert "predict_demote" not in last["reason"]


def test_record_predict_gated_false_when_no_demote_intent(tmp_path):
    # no SNR input (_sig sets snr=None) → predict block skipped → predict_gated False.
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig(-50.0))
    p.close()
    assert _records(tmp_path)[-1]["predict_gated"] is False


def test_record_carries_snr_and_evm(tmp_path):
    from fpvdgs.dynlink.signals import Signals

    p = Policy(_cfg(tmp_path), _profile())
    sig = Signals(
        rssi=-55.0,
        residual_loss_w=0.0,
        fec_work=0.0,
        link_starved_w=False,
        timestamp=1.0,
        snr_w=27.0,
        evm_w=89.0,
        evm_lo_w=80.0,
        evm_min_w=75.0,
    )
    p.tick(sig)
    p.close()
    rec = _records(tmp_path)[-1]
    assert rec["snr"] == 27.0
    assert rec["evm"] == 89.0 and rec["evm_lo"] == 80.0 and rec["evm_min"] == 75.0


def test_record_snr_evm_none_when_absent(tmp_path):
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig(-55.0))  # no snr/evm on the signal -> None
    p.close()
    rec = _records(tmp_path)[-1]
    assert rec["snr"] is None and rec["evm"] is None


def test_record_carries_link_width(tmp_path):
    cfg = _cfg(tmp_path)
    cfg.link_width = 10
    p = Policy(cfg, _profile())
    p.tick(_sig(-50.0))
    p.close()
    assert _records(tmp_path)[-1]["width"] == 10


def test_record_link_width_defaults_to_20(tmp_path):
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig(-50.0))
    p.close()
    assert _records(tmp_path)[-1]["width"] == 20


def test_record_carries_snr_ewma_knees(tmp_path):
    from fpvdgs.dynlink.signals import Signals

    p = Policy(_cfg(tmp_path, min_samples=3), _profile())
    sig = Signals(
        rssi=-50.0, residual_loss_w=0.0, fec_work=0.0, link_starved_w=False, timestamp=1.0, snr=35.0
    )
    p.tick(sig)
    p.close()
    rec = _records(tmp_path)[-1]
    assert rec["snr_ewma"] == 35.0
    assert "snr_ceiling" not in rec  # removed: cross-rung ceiling field is gone
    assert isinstance(rec["snr_knees"], list) and len(rec["snr_knees"]) == 8


def test_reactive_demote_steps_one_rung(tmp_path):
    """Reactive loss demotes exactly one rung (one step at a time), not a
    multi-rung jump: a single breaching window steps 5→4, no further."""
    from fpvdgs.dynlink.signals import Signals

    p = Policy(_cfg(tmp_path, min_samples=3), _profile())
    # Direct-set: SNR knees planted at rungs 1 and 4 (prior state; demote is reactive)
    p.learned_prior._snr_model._knee[1] = 10.0
    p.learned_prior._snr_model._count[1] = 12.0
    p.learned_prior._snr_model._knee[4] = 30.0
    p.learned_prior._snr_model._count[4] = 12.0
    p.leading.state.current_mcs = 5

    def sig(ts):
        return Signals(
            rssi=-60.0,
            residual_loss_w=0.30,
            fec_work=0.0,
            link_starved_w=False,
            timestamp=ts,
            snr=12.0,
        )

    dec = p.tick(sig(1.0))  # single breaching window -> one-step demote
    assert dec.mcs == 4  # 5 -> 4 (one step)
    p.close()


def test_proactive_snr_demote_before_loss(tmp_path):
    from fpvdgs.dynlink.signals import Signals

    p = Policy(_cfg(tmp_path, min_samples=3), _profile())
    # Direct-set: SNR knee: rung3 viable at snr>=15, rung4 at >=30. Current snr 20 -> rung4 unviable.
    p.learned_prior._snr_model._knee[3] = 15.0
    p.learned_prior._snr_model._count[3] = 12.0
    p.learned_prior._snr_model._knee[4] = 30.0
    p.learned_prior._snr_model._count[4] = 12.0
    p.leading.state.current_mcs = 4

    def sig(ts):  # NO loss, NO rssi (isolates the SNR proactive path)
        return Signals(
            rssi=None,
            residual_loss_w=0.0,
            fec_work=0.0,
            link_starved_w=False,
            timestamp=ts,
            snr=20.0,
        )

    decs = [p.tick(sig(1.0 + 0.1 * k)) for k in range(4)]  # debounce then demote
    assert decs[-1].mcs == 3  # demoted 4->3 with zero loss
    assert any("snr_demote mcs4->3" in d.reason for d in decs)
    p.close()


def test_promote_explores_cold_frontier_rung(tmp_path):
    # DEADLOCK regression (the live "maxMcs=5 never reaches 5"): the SNR prior is
    # confident for rung4 ONLY (the link only ever settled there). Healthy SNR +
    # cold rung5 (explore route). rung5 is UNKNOWN (cold), not unviable, so the
    # selector must be allowed to explore it AND the proactive SNR demote must NOT
    # yank it back before its knee can warm.
    from fpvdgs.dynlink.signals import Signals

    cfg = _cfg(tmp_path, min_samples=3)
    cfg.selector.max_mcs = 5
    cfg.selector.promote_dwell_ticks = 1  # fast promote for this regression test
    cfg.selector.hold_modes_down_ms = 0
    cfg.selector.min_between_changes_ms = 0
    cfg.selector.snr_demote_debounce = 2
    p = Policy(cfg, _profile())
    for _ in range(12):  # rung4 confident, viable at snr >= ~27
        p.learned_prior.ingest(snr=27.0, operating_mcs=4, operating_clean=True, settled=True)
    p.leading.state.current_mcs = 4

    def sig(ts):  # healthy snr (clears rung4), no loss
        return Signals(
            rssi=None,
            residual_loss_w=0.0,
            fec_work=0.0,
            link_starved_w=False,
            timestamp=ts,
            snr=39.0,
        )

    decs = [p.tick(sig(1.0 + 0.1 * k)) for k in range(10)]
    p.close()
    assert [d.mcs for d in decs[-4:]] == [5, 5, 5, 5]  # reached 5 AND holds (no yo-yo)


def test_record_carries_damper_release(tmp_path):
    """2026-07-06 spec: per-tick damper-release source (timer | raw_min |
    None) — the flight gate splits releases by channel."""
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig_snr(30.0, ts=1.0))
    p.close()
    last = _records(tmp_path)[-1]
    assert "damper_release" in last
    assert last["damper_release"] is None  # no damper activity this tick
