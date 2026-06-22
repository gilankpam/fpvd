import importlib.util
import json
from pathlib import Path

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
                    "knees": [None, -80, None, None, -60, None, None, None],
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
                    "knees": [None, -80, None, None, -60, None, None, None],
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
