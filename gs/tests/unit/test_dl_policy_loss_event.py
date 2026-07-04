"""Fast-path reactive demote (policy.loss_event) — same rule as the tick's
single-window loss demote, evaluated the moment loss is observed."""

from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.policy import Policy, PolicyConfig
from fpvdgs.dynlink.signals import Signals


def _policy():
    return Policy(PolicyConfig(flightlog=FlightLogConfig(enabled=False)))


def _signals(loss=0.10, snr=30.0, snr_w=29.0, ts=100.0):
    return Signals(residual_loss_w=loss, snr=snr, snr_w=snr_w, timestamp=ts, tap_active=True)


def _climb(p, rungs=2):
    """Drive the selector up via ticks so there is room to demote."""
    ts = 1.0
    while p.leading.state.current_mcs < rungs:
        p.tick(_signals(loss=0.0, snr=35.0, snr_w=35.0, ts=ts))
        ts += 0.1
    return ts


def test_loss_event_fires_one_step_demote_with_fast_reason():
    p = _policy()
    ts = _climb(p, rungs=2)
    start = p.leading.state.current_mcs
    d = p.loss_event(_signals(loss=0.10, ts=ts), latency_ms=1.5)
    assert d is not None
    assert d.mcs == start - 1
    assert "video_per_demote" in d.reason and "_fast" in d.reason


def test_loss_event_below_threshold_is_none_and_no_dwell_side_effect():
    p = _policy()
    ts = _climb(p, rungs=2)
    dwell = p.leading._clean_dwell
    assert p.loss_event(_signals(loss=0.01, ts=ts)) is None
    assert p.leading._clean_dwell == dwell  # not a tick: no clean-dwell credit


def test_loss_event_respects_shared_cooldown():
    p = _policy()
    ts = _climb(p, rungs=3)
    assert p.loss_event(_signals(loss=0.10, ts=ts)) is not None
    # cooldown (windows_since_demote=0 < demote_cooldown_windows) blocks a repeat
    assert p.loss_event(_signals(loss=0.10, ts=ts + 0.01)) is None


def test_loss_event_teaches_fade_knee():
    p = _policy()
    ts = _climb(p, rungs=2)
    start = p.leading.state.current_mcs
    # fade signature: raw snr_w collapsed >= collapse_delta_db below the EWMA
    d = p.loss_event(_signals(loss=0.10, snr=30.0, snr_w=20.0, ts=ts))
    assert d is not None
    knees = (
        p.learned_prior.snr_knees_snapshot()
    )  # list indexed by rung (see test_dl_learned_prior.py)
    assert knees[start] is not None


def test_tick_flightlog_carries_tap_active(tmp_path):
    import json

    from fpvdgs.dynlink.flightlog import FlightLogConfig as FLC

    p = Policy(PolicyConfig(flightlog=FLC(enabled=True, dir=str(tmp_path))))
    p.tick(_signals(loss=0.0, ts=1.0))
    p.flightlog.close()
    recs = [json.loads(line) for f in tmp_path.glob("*.jsonl") for line in open(f)]
    assert recs and recs[-1]["tap_active"] is True
