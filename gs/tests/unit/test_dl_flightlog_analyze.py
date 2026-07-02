import importlib.util
import json
from pathlib import Path

import pytest

TOOL = Path(__file__).resolve().parents[2] / "tools" / "flightlog_analyze.py"


def _load_tool():
    spec = importlib.util.spec_from_file_location("flightlog_analyze", TOOL)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def test_summarize_counts_mcs_and_demotes(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    with open(log, "w") as f:
        f.write(json.dumps({"ts": 1.0, "rssi": -50, "mcs": 5, "reason": ""}) + "\n")
        f.write(
            json.dumps({"ts": 1.1, "rssi": -55, "mcs": 4, "reason": "predict_demote mcs5->4"})
            + "\n"
        )
        f.write(
            json.dumps({"ts": 1.2, "rssi": -60, "mcs": 3, "reason": "video_per_demote loss=0.060"})
            + "\n"
        )
    s = mod.summarize(str(log))
    assert s["records"] == 3
    assert s["time_at_mcs"][5] >= 1
    assert s["predictive_demotes"] == 1
    assert s["reactive_demotes"] == 1


def test_probe_target_per_reads_current_plus_one_rung():
    mod = _load_tool()
    rec = {
        "mcs": 3,
        "probe": {"4": {"per": 0.02, "ageMs": 150.0}, "5": {"per": 0.9, "ageMs": 9000.0}},
    }
    assert mod.probe_target_per(rec) == 0.02


def test_probe_target_per_none_when_absent():
    mod = _load_tool()
    assert mod.probe_target_per({"mcs": 3}) is None  # pre-field log
    assert mod.probe_target_per({"mcs": 3, "probe": None}) is None  # no probe_status
    assert mod.probe_target_per({"mcs": 3, "probe": {}}) is None  # rung not heard
    assert mod.probe_target_per({"probe": {"4": {"per": 0.1}}}) is None  # no mcs


def test_summarize_counts_gated_demotes(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    with open(log, "w") as f:
        f.write(json.dumps({"ts": 1.0, "mcs": 5, "reason": "", "predict_gated": True}) + "\n")
        f.write(json.dumps({"ts": 1.1, "mcs": 5, "reason": "", "predict_gated": False}) + "\n")
        f.write(json.dumps({"ts": 1.2, "mcs": 5, "reason": ""}) + "\n")  # pre-field log
    s = mod.summarize(str(log))
    assert s["gated_demotes"] == 1


def test_summarize_counts_prior_learn_and_last_knees(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    with open(log, "w") as f:
        f.write(
            json.dumps(
                {
                    "ts": 1.0,
                    "mcs": 4,
                    "reason": "",
                    "prior_learn": True,
                    "snr_knees": [None, -80, None, None, -60, None, None, None],
                }
            )
            + "\n"
        )
        f.write(
            json.dumps(
                {
                    "ts": 1.1,
                    "mcs": 4,
                    "reason": "",
                    "prior_learn": False,
                    "snr_knees": [None, -80, None, None, -60, None, None, None],
                }
            )
            + "\n"
        )
        f.write(json.dumps({"ts": 1.2, "mcs": 4, "reason": ""}) + "\n")  # pre-field
    s = mod.summarize(str(log))
    assert s["prior_learn_ticks"] == 1
    assert s["last_knees"] == [None, -80, None, None, -60, None, None, None]


def test_summarize_snr_evm(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    with open(log, "w") as f:
        f.write(
            json.dumps({"ts": 1.0, "mcs": 4, "reason": "", "snr": 28, "evm": 89, "evm_min": 81})
            + "\n"
        )
        f.write(
            json.dumps({"ts": 1.1, "mcs": 4, "reason": "", "snr": 24, "evm": 80, "evm_min": 70})
            + "\n"
        )
        f.write(json.dumps({"ts": 1.2, "mcs": 4, "reason": ""}) + "\n")  # pre-field
    s = mod.summarize(str(log))
    assert s["mean_snr"] == 26.0  # (28+24)/2
    assert s["mean_evm"] == 84.5  # (89+80)/2
    assert s["min_evm"] == 70  # worst evm_min seen


# ---- new-log fields: fail_class, promote_route, fade_recovery ----------


def test_summarize_fail_class_and_promote_routes(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    recs = [
        {"ts": 1.0, "mcs": 5, "reason": ""},
        {"ts": 1.1, "mcs": 4, "reason": "demote class=fade", "fail_class": "fade"},
        {"ts": 1.2, "mcs": 4, "reason": "demote class=flap", "fail_class": "flap"},
        {"ts": 1.3, "mcs": 5, "reason": "knee_promote mcs4->5"},
        {"ts": 1.4, "mcs": 5, "reason": "snapback_promote tgt=4"},
        {"ts": 1.5, "mcs": 5, "reason": "explore_promote mcs5->6"},
    ]
    with open(log, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")
    s = mod.summarize(str(log))
    assert s["fail_class_counts"] == {"fade": 1, "flap": 1}
    assert s["promote_route_counts"] == {
        "knee_promote": 1,
        "snapback_promote": 1,
        "explore_promote": 1,
    }


def test_summarize_fade_recovery(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    # MCS5 tick -> fade demote to MCS3 -> MCS3 -> MCS4 -> MCS5 (recovery at pre-fade level)
    recs = [
        {"ts": 1.0, "mcs": 5, "reason": ""},
        {"ts": 2.0, "mcs": 3, "reason": "demote class=fade", "fail_class": "fade"},
        {"ts": 3.0, "mcs": 3, "reason": ""},
        {"ts": 4.0, "mcs": 4, "reason": "knee_promote"},
        {"ts": 5.0, "mcs": 5, "reason": "knee_promote"},
    ]
    with open(log, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")
    s = mod.summarize(str(log))
    # fade_from at ts=2.0 with pre-fade mcs=5; first tick with mcs>=5 is ts=5.0
    assert s["fade_recovery_p50_s"] == pytest.approx(3.0)
    assert s["fade_recovery_max_s"] == pytest.approx(3.0)


def test_summarize_fade_recovery_multiple(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    # Two fade events: recovery times 2.0s and 4.0s → p50=3.0, max=4.0
    recs = [
        {"ts": 0.0, "mcs": 4, "reason": ""},
        {"ts": 1.0, "mcs": 2, "reason": "demote class=fade", "fail_class": "fade"},
        {"ts": 3.0, "mcs": 4, "reason": "knee_promote"},  # recovery: 3.0-1.0=2.0s
        {"ts": 4.0, "mcs": 2, "reason": "demote class=fade", "fail_class": "fade"},
        {"ts": 8.0, "mcs": 4, "reason": "knee_promote"},  # recovery: 8.0-4.0=4.0s
    ]
    with open(log, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")
    s = mod.summarize(str(log))
    assert s["fade_recovery_p50_s"] == pytest.approx(3.0)  # median of [2.0, 4.0]
    assert s["fade_recovery_max_s"] == pytest.approx(4.0)


def test_summarize_no_fades(tmp_path):
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    recs = [
        {"ts": 1.0, "mcs": 5, "reason": ""},
        {"ts": 1.1, "mcs": 4, "reason": "demote class=flap", "fail_class": "flap"},
    ]
    with open(log, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")
    s = mod.summarize(str(log))
    assert s["fade_recovery_p50_s"] is None
    assert s["fade_recovery_max_s"] is None


def test_summarize_old_log_with_probe_no_crash(tmp_path):
    """Old-shape records (with probe, no fail_class/trial) must not crash."""
    mod = _load_tool()
    log = tmp_path / "f.jsonl"
    recs = [
        {
            "ts": 1.0,
            "mcs": 5,
            "reason": "",
            "probe": {"6": {"per": 0.02, "ageMs": 100.0}},
        },
        {
            "ts": 1.1,
            "mcs": 4,
            "reason": "predict_demote mcs5->4",
            "probe": {"5": {"per": 0.5, "ageMs": 200.0}},
        },
    ]
    with open(log, "w") as f:
        for r in recs:
            f.write(json.dumps(r) + "\n")
    s = mod.summarize(str(log))
    assert s["records"] == 2
    assert s["predictive_demotes"] == 1
    assert s["fail_class_counts"] == {}
    assert s["promote_route_counts"] == {}
    assert s["fade_recovery_p50_s"] is None
    assert s["fade_recovery_max_s"] is None
