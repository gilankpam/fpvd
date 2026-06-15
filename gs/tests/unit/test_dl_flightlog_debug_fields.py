"""Flight-log debug fields: per-rung probe snapshot, predictive internals
(pc/slope), and the promote debounce counter — the data needed to replay a
promote→demote oscillation offline."""
from __future__ import annotations

import json

from fpvdgs.dynlink.policy import Policy, PolicyConfig
from fpvdgs.dynlink.learned_prior import LearnedPriorConfig
from fpvdgs.dynlink.flightlog import FlightLogConfig
from fpvdgs.dynlink.signals import Signals


def _profile():
    return "m8812eu2"   # Policy takes the radioProfile string (learned-prior key)


def _cfg(tmp_path, **lp):
    return PolicyConfig(
        learned_prior=LearnedPriorConfig(persist_dir=str(tmp_path), **lp),
        flightlog=FlightLogConfig(dir=str(tmp_path / "fl")),
    )


def _sig(rssi, ts=1.0):
    return Signals(rssi=rssi, residual_loss_w=0.0, fec_work=0.0,
                   link_starved_w=False, timestamp=ts)


def _records(tmp_path):
    files = sorted((tmp_path / "fl").glob("*.jsonl"))
    assert files, "expected a flight-log file"
    with open(files[-1]) as f:
        return [json.loads(line) for line in f if line.strip()]


def test_record_carries_per_rung_probe_per_and_age(tmp_path):
    probe_snap = {"running": True, "streams": 1,
                  "mcs": {"2": {"per": 0.0125, "ageMs": 120.0,
                                "rssi": -50.0, "snr": 25.0}}}
    p = Policy(_cfg(tmp_path), _profile(), probe_status=lambda: probe_snap)
    p.tick(_sig(-50.0))
    p.close()
    last = _records(tmp_path)[-1]
    # Compact per-rung view: per + ageMs only (rssi/snr not logged).
    assert last["probe"] == {"2": {"per": 0.0125, "ageMs": 120.0}}


def test_record_probe_none_without_probe_status(tmp_path):
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig(-50.0))
    p.close()
    assert _records(tmp_path)[-1]["probe"] is None


def test_record_carries_pc_and_slope(tmp_path):
    prof = _profile()
    p = Policy(_cfg(tmp_path, ewma_alpha=1.0, min_samples_warmstart=3,
                    min_samples_predictive=3), prof)
    for _ in range(5):
        p.learned_prior.ingest(rssi=-50.0, probed_rung=5, probe_clean=True,
                               operating_mcs=5, operating_clean=True)
    p.tick(_sig(-50.0, ts=1.0))
    p.tick(_sig(-52.0, ts=1.1))
    p.close()
    recs = _records(tmp_path)
    assert recs[0]["slope"] == 0.0          # no previous RSSI yet
    assert recs[0]["pc"] == 5
    assert recs[1]["slope"] == -2.0
    # projected -52 + (-2 * 3 ticks) = -58 -> below the only confident bin,
    # ladder extrapolates from the lowest anchor (5)
    assert recs[1]["pc"] == 5


def test_record_pc_and_slope_none_when_prior_cold_or_no_rssi(tmp_path):
    # Cold prior (empty persist_dir, no learned data) → pc is None even with
    # RSSI present; no RSSI → pc is always None.
    p = Policy(_cfg(tmp_path), _profile())
    p.tick(_sig(-50.0))
    p.tick(_sig(None, ts=1.1))
    p.close()
    recs = _records(tmp_path)
    assert recs[0]["pc"] is None
    assert recs[0]["slope"] == 0.0          # slope is prior-independent
    assert recs[1]["pc"] is None
    assert recs[1]["slope"] is None         # no RSSI this tick


def test_record_carries_promote_clean_counter(tmp_path):
    # Clean + fresh current+1 rung (boot MCS 1 -> target 2): the debounce
    # counter accumulates 1, 2, ... across ticks (default debounce 3, so no
    # commit yet); the record shows the post-tick value.
    probe_snap = {"mcs": {"2": {"per": 0.0, "ageMs": 100.0}}}
    p = Policy(_cfg(tmp_path), _profile(), probe_status=lambda: probe_snap)
    p.tick(_sig(-50.0, ts=1.0))
    p.tick(_sig(-50.0, ts=1.1))
    p.close()
    recs = _records(tmp_path)
    assert recs[0]["promote_clean"] == 1
    assert recs[1]["promote_clean"] == 2


def test_logged_slope_is_least_squares_not_single_tick(tmp_path):
    """A lone RSSI spike barely moves the logged slope (least-squares over a
    window) — the old single-tick delta would log the full +5 dB jump."""
    p = Policy(_cfg(tmp_path), _profile())
    for rssi, ts in [(-50.0, 1.0), (-50.0, 1.1), (-50.0, 1.2),
                     (-50.0, 1.3), (-45.0, 1.4)]:
        p.tick(_sig(rssi, ts=ts))
    p.close()
    last = _records(tmp_path)[-1]
    # lsq over [-50,-50,-50,-50,-45] = +1.0  (single-tick delta would be +5.0)
    assert abs(last["slope"] - 1.0) < 1e-6


def test_logged_slope_uses_only_the_rolling_window(tmp_path):
    """Samples older than the default 10-tick window must not affect the slope:
    5 flat ticks then a 10-tick -1/tick ramp → slope -1.0 (the flat prefix has
    rolled out). If the prefix leaked in, the slope would be shallower."""
    p = Policy(_cfg(tmp_path), _profile())
    ts = 1.0
    for _ in range(5):                        # flat prefix — rolls out of the window
        p.tick(_sig(-50.0, ts=ts)); ts += 0.1
    for i in range(10):                       # -1 dB/tick ramp fills the window
        p.tick(_sig(-50.0 - i, ts=ts)); ts += 0.1
    p.close()
    last = _records(tmp_path)[-1]
    assert abs(last["slope"] - (-1.0)) < 1e-6
