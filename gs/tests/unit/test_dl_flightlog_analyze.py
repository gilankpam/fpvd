import json
import importlib.util
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
        f.write(json.dumps({"ts": 1.1, "rssi": -55, "mcs": 4,
                            "reason": "predict_demote mcs5->4"}) + "\n")
        f.write(json.dumps({"ts": 1.2, "rssi": -60, "mcs": 3,
                            "reason": "video_per_demote loss=0.060"}) + "\n")
    s = mod.summarize(str(log))
    assert s["records"] == 3
    assert s["time_at_mcs"][5] >= 1
    assert s["predictive_demotes"] == 1
    assert s["reactive_demotes"] == 1
