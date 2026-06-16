import json
from fpvdgs.dynlink.flightlog import FlightLog, FlightLogConfig


def test_writes_jsonl_records(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    fl.write({"ts": 1.0, "mcs": 5})
    fl.write({"ts": 1.1, "mcs": 4})
    fl.close()
    files = list(tmp_path.glob("*.jsonl"))
    assert len(files) == 1
    lines = files[0].read_text().splitlines()
    assert len(lines) == 2
    assert json.loads(lines[0])["mcs"] == 5


def test_disabled_is_noop(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), enabled=False))
    fl.write({"ts": 1.0})
    fl.close()
    assert list(tmp_path.iterdir()) == []


def test_write_recovers_when_open_failed_at_startup(tmp_path):
    # The DVR is an autofs automount; if _open() races the mount at startup it
    # fails (_fh=None) and writes silently no-op for the whole flight. Once the
    # mount is up, write() must lazily re-open and resume logging.
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), sync_interval=2))
    fl._fh.close()                       # simulate "open failed at startup"
    fl._fh = None
    fl.write({"mcs": 1})                 # retry throttle not reached -> still down
    assert fl._fh is None
    fl.write({"mcs": 2})                 # throttle reached -> re-open + write
    assert fl._fh is not None
    fl.close()
    recs = [json.loads(line) for f in sorted(tmp_path.glob("*.jsonl"))
            for line in f.read_text().splitlines() if line.strip()]
    assert {"mcs": 2} in recs            # logging resumed once the mount was up


def test_rotation_keeps_max_files(tmp_path):
    # create 5 sessions with max_files=3 → oldest pruned on close
    for i in range(5):
        fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=3))
        fl.write({"ts": float(i)})
        fl.close()
    files = sorted(tmp_path.glob("*.jsonl"))
    assert len(files) == 3       # only the 3 newest survive


def test_write_after_close_is_safe(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    fl.close()
    fl.write({"ts": 1.0})        # no crash


def test_config_defaults_dvr_dir_and_gap():
    from fpvdgs.dynlink.flightlog import FlightLogConfig
    c = FlightLogConfig()
    assert c.dir == "/media/dvr/log/dynamic-link/"
    assert c.flight_gap_s == 15.0


def test_roll_starts_new_file_and_both_persist(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=8))
    fl.write({"ts": 1.0, "mcs": 5})
    fl.roll()
    fl.write({"ts": 9.0, "mcs": 2})
    fl.close()
    files = sorted(tmp_path.glob("*.jsonl"))
    assert len(files) == 2                      # rolled into a second file
    newest = max(files, key=lambda p: p.stat().st_mtime)
    assert '"mcs":2' in newest.read_text()


def test_roll_prunes_to_max_files(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=2))
    fl.write({"ts": 1.0})
    fl.roll(); fl.write({"ts": 2.0})
    fl.roll(); fl.write({"ts": 3.0})            # 3 flights, cap 2
    fl.close()
    assert len(list(tmp_path.glob("*.jsonl"))) == 2


def test_roll_is_noop_when_disabled(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), enabled=False))
    fl.roll()
    fl.write({"ts": 1.0})
    fl.close()
    assert list(tmp_path.iterdir()) == []


def test_filename_increments_across_restart(tmp_path):
    # First flight, then a "GS restart": a fresh instance must continue the
    # sequence from disk (the monotonic clock has reset to ~0) and must NOT
    # overwrite the earlier flight.
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=8))
    fl.write({"ts": 1.0, "mcs": 5})
    fl.close()
    fl2 = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=8))
    fl2.write({"ts": 2.0, "mcs": 2})
    fl2.close()
    files = sorted(tmp_path.glob("*.jsonl"))
    assert len(files) == 2                      # the restart did not clobber flight 1
    seqs = sorted(int(p.stem) for p in files)
    assert seqs[1] == seqs[0] + 1               # strictly incremental by name


def test_roll_increments_by_name(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=8))
    fl.write({"ts": 1.0})
    fl.roll()
    fl.write({"ts": 2.0})
    fl.close()
    seqs = sorted(int(p.stem) for p in tmp_path.glob("*.jsonl"))
    assert seqs == [seqs[0], seqs[0] + 1]


def test_first_flight_starts_at_one(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    fl.write({"ts": 1.0})
    fl.close()
    files = list(tmp_path.glob("*.jsonl"))
    assert int(files[0].stem) == 1


def test_syncs_periodically_to_disk_before_close(tmp_path):
    # Records must reach the card mid-flight, so a hard reboot doesn't lose
    # the whole log. With sync_interval=10, after 25 writes >= 20 are on disk
    # WITHOUT close (the old buffered code left 0 on disk until close).
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), sync_interval=10))
    for i in range(25):
        fl.write({"ts": float(i)})
    f = sorted(tmp_path.glob("*.jsonl"))[-1]
    assert f.read_text().count("\n") >= 20
    fl.close()


def test_fsyncs_on_roll(tmp_path, monkeypatch):
    # roll() ends a flight; it must fsync the completed file to the card
    # BEFORE closing, so the just-finished flight survives a reboot.
    import fpvdgs.dynlink.flightlog as mod
    calls = []
    monkeypatch.setattr(mod.os, "fsync", lambda fd: calls.append(fd))
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    fl.write({"ts": 1.0})
    fl.roll()
    assert calls, "roll() must fsync before closing the flight file"
    fl.close()


def test_begin_flight_keeps_fresh_empty_file(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    path1 = fl._path
    fl.begin_flight()                  # nothing written yet -> keep the same file
    assert fl._path == path1
    fl.close()
    assert len(list(tmp_path.glob("*.jsonl"))) == 1


def test_begin_flight_rolls_when_file_has_records(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), max_files=8))
    fl.write({"ts": 1.0})
    fl.begin_flight()                  # has records -> start a new flight file
    fl.write({"ts": 2.0})
    fl.close()
    assert len(list(tmp_path.glob("*.jsonl"))) == 2


def test_begin_flight_noop_when_disabled(tmp_path):
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path), enabled=False))
    fl.begin_flight()
    fl.close()
    assert list(tmp_path.iterdir()) == []


def test_begin_flight_reopens_when_fh_is_none(tmp_path):
    # The DVR autofs race can leave _fh=None (open failed at startup). A connect
    # event then calls begin_flight(), which must (re)open via roll().
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    fl._fh.close()
    fl._fh = None                        # simulate "open failed at startup"
    fl.begin_flight()
    assert fl._fh is not None            # reopened
    fl.write({"ts": 1.0})
    fl.close()
    recs = [json.loads(line) for f in sorted(tmp_path.glob("*.jsonl"))
            for line in f.read_text().splitlines() if line.strip()]
    assert {"ts": 1.0} in recs           # logging resumed after the reopen


def test_sync_fsyncs_open_file(tmp_path, monkeypatch):
    import fpvdgs.dynlink.flightlog as mod
    calls = []
    monkeypatch.setattr(mod.os, "fsync", lambda fd: calls.append(fd))
    fl = FlightLog(FlightLogConfig(dir=str(tmp_path)))
    fl.write({"ts": 1.0})
    fl.sync()
    assert calls, "sync() must fsync the open file"
    fl.close()
