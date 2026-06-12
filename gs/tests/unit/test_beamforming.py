from fpvdgs.beamforming import BeamformingController, read_mac


def _node(tmp_path, iface):
    """Create a fake bf_monitor_conf proc node; return (proc_base, conf_path)."""
    proc = tmp_path / "proc"
    (proc / iface).mkdir(parents=True)
    conf = proc / iface / "bf_monitor_conf"
    conf.write_text("")
    return str(proc), conf


def _sys(tmp_path, iface, mac):
    sysd = tmp_path / "sys"
    (sysd / iface).mkdir(parents=True)
    (sysd / iface / "address").write_text(mac + "\n")
    return str(sysd)


def test_supported_true_when_node_present(tmp_path):
    proc, _ = _node(tmp_path, "wlan0")
    bf = BeamformingController(proc_base=proc)
    assert bf.supported("wlan0") is True


def test_supported_false_when_node_absent(tmp_path):
    bf = BeamformingController(proc_base=str(tmp_path / "proc"))
    assert bf.supported("wlan0") is False


def test_read_mac(tmp_path):
    sysd = _sys(tmp_path, "wlan0", "84:fc:14:6c:36:e6")
    assert read_mac("wlan0", sys_base=sysd) == "84:fc:14:6c:36:e6"


def test_enable_writes_conf_and_reports_active(tmp_path):
    proc, conf = _node(tmp_path, "wlan0")
    sysd = _sys(tmp_path, "wlan0", "84:fc:14:6c:36:e6")
    bf = BeamformingController(proc_base=proc, sys_base=sysd)
    st = bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    assert conf.read_text() == "1 00:c0:ca:dd:ee:ff 0 0"
    assert st["state"] == "active"
    assert st["requested"] is True
    assert st["peerMac"] == "00:c0:ca:dd:ee:ff"
    assert st["localMac"] == "84:fc:14:6c:36:e6"


def test_disable_writes_reset_and_reports_disabled(tmp_path):
    proc, conf = _node(tmp_path, "wlan0")
    bf = BeamformingController(proc_base=proc)
    bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    st = bf.reconcile(False, "wlan0", "")
    assert conf.read_text() == "0 00:00:00:00:00:00 0 0"
    assert st["state"] == "disabled"
    assert st["requested"] is False


def test_unsupported_when_no_node(tmp_path):
    bf = BeamformingController(proc_base=str(tmp_path / "proc"))
    st = bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    assert st["state"] == "unsupported"
    assert "bf_monitor_conf" in st["reason"]


def test_idempotent_no_rewrite(tmp_path):
    proc, conf = _node(tmp_path, "wlan0")
    bf = BeamformingController(proc_base=proc)
    bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    conf.write_text("SENTINEL")          # prove a second reconcile does NOT rewrite
    bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    assert conf.read_text() == "SENTINEL"


def test_write_failure_reports_error(tmp_path):
    # Node directory exists for supported(), but the conf path is a directory so
    # open(..., "w") raises -> state=error.
    proc = tmp_path / "proc"
    (proc / "wlan0" / "bf_monitor_conf").mkdir(parents=True)
    bf = BeamformingController(proc_base=str(proc))
    st = bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    assert st["state"] == "error"


def test_status_with_primary_reports_mac_when_disarmed(tmp_path):
    # BF never armed (state=disabled, _iface=""), but a client must still be able
    # to read the GS card MAC to set the drone's remoteMac before enabling BF.
    sysd = _sys(tmp_path, "wlan0", "84:fc:14:6c:36:e6")
    bf = BeamformingController(proc_base=str(tmp_path / "proc"), sys_base=sysd)
    st = bf.status_with_primary("wlan0")
    assert st["localMac"] == "84:fc:14:6c:36:e6"
    assert st["iface"] == "wlan0"
    assert st["state"] == "disabled"


def test_status_with_primary_preserves_armed_iface(tmp_path):
    # When already armed, the armed iface/MAC win over the primary arg.
    proc, _ = _node(tmp_path, "wlan0")
    sysd = _sys(tmp_path, "wlan0", "84:fc:14:6c:36:e6")
    bf = BeamformingController(proc_base=proc, sys_base=sysd)
    bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")
    st = bf.status_with_primary("wlan1")
    assert st["iface"] == "wlan0"
    assert st["localMac"] == "84:fc:14:6c:36:e6"


def test_status_with_primary_none_is_plain_status(tmp_path):
    bf = BeamformingController(proc_base=str(tmp_path / "proc"),
                              sys_base=str(tmp_path / "sys"))
    st = bf.status_with_primary(None)
    assert st["localMac"] == ""
    assert st["iface"] == ""


def test_disable_write_failure_reports_error(tmp_path, monkeypatch):
    proc, conf = _node(tmp_path, "wlan0")
    bf = BeamformingController(proc_base=proc)
    bf.reconcile(True, "wlan0", "00:c0:ca:dd:ee:ff")   # arm OK
    # Make the next write fail without disturbing supported()/the path's existence.
    import fpvdgs.beamforming as mod
    orig_open = open
    def boom(path, *a, **k):
        if str(path).endswith("bf_monitor_conf") and (a[:1] == ("w",) or k.get("mode") == "w"):
            raise OSError("read-only")
        return orig_open(path, *a, **k)
    monkeypatch.setattr("builtins.open", boom)
    st = bf.reconcile(False, "wlan0", "")
    assert st["state"] == "error"
    assert "reset" in st["reason"].lower()
