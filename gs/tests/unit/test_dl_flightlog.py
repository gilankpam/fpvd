import json
from fpvdgs.dynlink.flightlog import FlightLog, FlightLogConfig


def test_writes_jsonl_records(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)), start_ms=1000)
    fl.write({"ts": 1.0, "mcs": 5})
    fl.write({"ts": 1.1, "mcs": 4})
    fl.close()
    files = list(tmp_path.glob("*.jsonl"))
    assert len(files) == 1
    lines = files[0].read_text().splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["mcs"] == 5


def test_disabled_is_noop(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), enabled=False), start_ms=1)
    fl.write({"ts": 1.0})
    fl.close()
    assert list(tmp_path.iterdir()) == []


def test_rotation_keeps_max_files(tmp_path):
    # create 5 sessions with max_files=3 → oldest pruned on close
    for i in range(5):
        fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=3),
                       start_ms=1000 + i)
        fl.write({"ts": float(i)})
        fl.close()
    files = sorted(tmp_path.glob("*.jsonl"))
    assert len(files) == 3       # only the 3 newest survive


def test_write_after_close_is_safe(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)), start_ms=1)
    fl.close()
    fl.write({"ts": 1.0})        # no crash
