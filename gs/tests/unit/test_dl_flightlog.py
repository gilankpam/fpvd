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


def test_config_defaults_dvr_dir_and_gap():
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    c = FlightLogConfig()
    assert c.dir == "/media/dvr/log/dynamic-link/"
    assert c.flight_gap_s == 15.0


def test_roll_starts_new_file_and_both_persist(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=8), start_ms=1000)
    fl.write({"ts": 1.0, "mcs": 5})
    fl.roll()
    fl.write({"ts": 9.0, "mcs": 2})
    fl.close()
    files = sorted(tmp_path.glob("*.jsonl"))
    assert len(files) == 2                      # rolled into a second file
    newest = max(files, key=lambda p: p.stat().st_mtime)
    assert '"mcs":2' in newest.read_text()


def test_roll_prunes_to_max_files(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=2), start_ms=1000)
    fl.write({"ts": 1.0})
    fl.roll(); fl.write({"ts": 2.0})
    fl.roll(); fl.write({"ts": 3.0})            # 3 flights, cap 2
    fl.close()
    assert len(list(tmp_path.glob("*.jsonl"))) == 2


def test_roll_is_noop_when_disabled(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), enabled=False), start_ms=1)
    fl.roll()
    fl.write({"ts": 1.0})
    fl.close()
    assert list(tmp_path.iterdir()) == []
