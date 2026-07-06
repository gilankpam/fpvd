import os
import shutil
import subprocess
import time

from fpvdgs.wfb.cards import Card
from fpvdgs.wfb.cluster import plan_cluster, render_node_script

LOCAL = [Card(host=None, iface="wlan1"), Card(host=None, iface="wlan2")]
REMOTE_HOST = "192.168.1.10"
REMOTE = [Card(host=REMOTE_HOST, iface="wlan0", txpower_dbm="off")]

LINK = {"channel": 132, "width": 20, "region": "US"}
LINK_ID = 7669206
SERVER_ADDRESS = "10.18.0.1"


def _remote_script(**overrides):
    plan = plan_cluster(LOCAL + REMOTE)
    link = dict(LINK, **overrides)
    return render_node_script(
        REMOTE_HOST,
        REMOTE,
        plan,
        link=link,
        link_id=LINK_ID,
        server_address=SERVER_ADDRESS,
    )


def test_golden_forwarder_and_injector_lines():
    script = _remote_script()
    assert "wfb_rx -f -c 10.18.0.1 -u 10000 -p 0 -i 7669206 -R 2097152 wlan0 &" in script
    assert "wfb_rx -f -c 10.18.0.1 -u 10001 -p 16 -i 7669206 -R 2097152 wlan0 &" in script
    assert "wfb_tx -I 11001 -R 2097152 wlan0 &" in script  # mavlink injector
    assert "iw dev wlan0 set channel 132 HT20" in script
    assert "iw dev wlan0 set txpower" not in script  # "off" card: no txpower line
    assert "(sleep 1; exec cat <&3 > /dev/null) &" in script  # ssh watchdog


def test_no_bashisms():
    script = _remote_script()
    assert script.startswith("#!/bin/sh\n")
    assert "wait -n" not in script
    assert "set -b" not in script
    assert "set -emb" not in script


def test_watchdog_reads_saved_stdin_fd_not_fd0():
    # On a non-tty BusyBox/ash node, `set -m` job control is off and POSIX
    # redirects a background job's fd 0 to /dev/null — so a watchdog reading
    # fd 0 would EOF instantly and tear the node down ~1s after start
    # (bench-observed 2026-07-04). The script must save the SSH stdin on fd 3
    # (`exec 3<&0`) BEFORE spawning any background job, and the watchdog must
    # read fd 3, so it stays blocked until the SSH channel actually closes.
    script = _remote_script()
    lines = script.splitlines()
    save_idx = next(i for i, ln in enumerate(lines) if ln.strip() == "exec 3<&0")
    watchdog_idx = next(i for i, ln in enumerate(lines) if "exec cat <&3" in ln)
    assert save_idx < watchdog_idx  # fd saved before any background job reads it
    assert "exec cat > /dev/null" not in script  # the broken fd-0 form is gone


def test_cleanup_trap_catches_abrupt_signals():
    # An abrupt SSH drop delivers SIGHUP; the EXIT trap alone doesn't fire on an
    # unhandled signal, so the children would leak. Trap the signals too.
    script = _remote_script()
    assert "trap _cleanup EXIT HUP INT TERM" in script


def test_reaps_stale_wfb_before_spawning():
    # Each (re)connect must reap any wfb_rx/wfb_tx a prior session orphaned
    # (abrupt drops / overlapping reconnect), else duplicate forwarders/injectors
    # pile up on the card and destabilise the link. The reap must run BEFORE this
    # session spawns its own forwarders/injectors.
    script = _remote_script()
    lines = script.splitlines()
    reap_idx = next(
        i for i, ln in enumerate(lines) if "grep -E 'wfb_(rx|tx)'" in ln and "kill" not in ln
    )
    first_spawn_idx = next(i for i, ln in enumerate(lines) if ln.startswith("wfb_rx -f"))
    assert reap_idx < first_spawn_idx  # reap stale orphans before spawning ours
    assert "kill -9 $_p 2>/dev/null || true" in script  # tolerant of already-dead pids


def test_sh_syntax_check(tmp_path):
    script = _remote_script()
    path = tmp_path / "node.sh"
    path.write_text(script)

    result = subprocess.run(["sh", "-n", str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr

    dash = shutil.which("dash")
    if dash:
        result = subprocess.run([dash, "-n", str(path)], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr


def test_init_script_emitted_before_iw_reg_set_and_iface_down():
    cards = [
        Card(
            host=REMOTE_HOST,
            iface="wlan0",
            init_script="iw phy phy0 interface add wlan0 type monitor || true",
        )
    ]
    plan = plan_cluster(cards)
    script = render_node_script(
        REMOTE_HOST,
        cards,
        plan,
        link=LINK,
        link_id=LINK_ID,
        server_address=SERVER_ADDRESS,
    )
    lines = script.splitlines()
    assert "iw phy phy0 interface add wlan0 type monitor || true" in lines
    init_idx = lines.index("iw phy phy0 interface add wlan0 type monitor || true")
    reg_idx = next(i for i, ln in enumerate(lines) if ln.startswith("iw reg set"))
    down_idx = next(i for i, ln in enumerate(lines) if ln.startswith("ip link set"))
    assert init_idx < reg_idx
    assert init_idx < down_idx


def test_no_init_script_back_compat_no_marker():
    # No card carries initScript -> no marker/behavior change (back-compat with
    # the existing golden/order tests, which build cards without initScript).
    script = _remote_script()
    assert "# custom init" not in script


def test_init_script_deduped_across_cards_on_same_node():
    cards = [
        Card(
            host=REMOTE_HOST,
            iface="wlan0",
            init_script="iw phy phy0 interface add wlan0 type monitor || true",
        ),
        Card(
            host=REMOTE_HOST,
            iface="wlan1",
            init_script="iw phy phy0 interface add wlan0 type monitor || true",
        ),
    ]
    plan = plan_cluster(cards)
    script = render_node_script(
        REMOTE_HOST,
        cards,
        plan,
        link=LINK,
        link_id=LINK_ID,
        server_address=SERVER_ADDRESS,
    )
    assert script.count("iw phy phy0 interface add wlan0 type monitor || true") == 1


def test_init_script_sh_syntax_check(tmp_path):
    cards = [
        Card(
            host=REMOTE_HOST,
            iface="wlan0",
            init_script="iw phy phy0 interface add wlan0 type monitor || true",
        )
    ]
    plan = plan_cluster(cards)
    script = render_node_script(
        REMOTE_HOST,
        cards,
        plan,
        link=LINK,
        link_id=LINK_ID,
        server_address=SERVER_ADDRESS,
    )
    path = tmp_path / "node.sh"
    path.write_text(script)

    result = subprocess.run(["sh", "-n", str(path)], capture_output=True, text=True)
    assert result.returncode == 0, result.stderr

    dash = shutil.which("dash")
    if dash:
        result = subprocess.run([dash, "-n", str(path)], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr


def test_txpower_line_in_mbm():
    cards = [Card(host=REMOTE_HOST, iface="wlan0", txpower_dbm=20)]
    plan = plan_cluster(cards)
    script = render_node_script(
        REMOTE_HOST,
        cards,
        plan,
        link=LINK,
        link_id=LINK_ID,
        server_address=SERVER_ADDRESS,
    )
    assert "iw dev wlan0 set txpower fixed 2000" in script


def test_node_script_renders_probe_forwarder_when_streams_include_it():
    from fpvdgs.wfb.cluster import streams_for

    cards = [Card(host="192.168.1.10", iface="wlan0")]
    plan = plan_cluster(cards, with_probe=True)
    node = next(n for n in plan.nodes if n != "127.0.0.1")
    script = render_node_script(
        node,
        plan.nodes[node],
        plan,
        link={"channel": 132, "width": 20},
        link_id=123,
        server_address="10.0.0.1",
        streams=streams_for(True),
    )
    assert f"-u {plan.server_port['probe']} -p 50" in script
    # default streams: no probe line
    script_off = render_node_script(
        node,
        plan.nodes[node],
        plan,
        link={"channel": 132, "width": 20},
        link_id=123,
        server_address="10.0.0.1",
    )
    assert "-p 50" not in script_off


_STUB_SLEEP = """#!/bin/sh
echo "$0 $*" >> "{log}"
sleep 5
"""

_STUB_ONESHOT = """#!/bin/sh
echo "$0 $*" >> "{log}"
"""

_STUB_SHORTLIVED_RX = """#!/bin/sh
echo "$0 $*" >> "{log}"
sleep 0.2
"""


def _write_stub(path, content, log):
    path.write_text(content.format(log=log))
    path.chmod(0o755)


def test_path_stub_execution_runs_children_in_order(tmp_path):
    script = _remote_script()
    script_path = tmp_path / "node.sh"
    script_path.write_text(script)
    script_path.chmod(0o755)

    log = tmp_path / "log.txt"
    log.write_text("")

    for name in ("iw", "ip", "nmcli"):
        _write_stub(tmp_path / name, _STUB_ONESHOT, log)
    for name in ("wfb_rx", "wfb_tx"):
        _write_stub(tmp_path / name, _STUB_SLEEP, log)

    env = {"PATH": f"{tmp_path}:{shutil.os.environ['PATH']}"}
    proc = subprocess.Popen(
        ["sh", str(script_path)],
        cwd=tmp_path,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    time.sleep(1.5)
    proc.terminate()
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)

    lines = log.read_text().splitlines()
    assert any("/nmcli" in line for line in lines)
    assert any("/ip " in line or line.endswith("/ip") for line in lines)
    assert any("/iw " in line for line in lines)
    assert any("/wfb_rx" in line for line in lines)
    assert any("/wfb_tx" in line for line in lines)

    # init commands happen before the service commands
    nmcli_idx = next(i for i, line in enumerate(lines) if "/nmcli" in line)
    wfb_rx_idx = next(i for i, line in enumerate(lines) if "/wfb_rx" in line)
    assert nmcli_idx < wfb_rx_idx


def test_path_stub_fail_fast_on_child_death(tmp_path):
    script = _remote_script()
    script_path = tmp_path / "node.sh"
    script_path.write_text(script)
    script_path.chmod(0o755)

    log = tmp_path / "log.txt"
    log.write_text("")

    for name in ("iw", "ip", "nmcli"):
        _write_stub(tmp_path / name, _STUB_ONESHOT, log)
    _write_stub(tmp_path / "wfb_rx", _STUB_SHORTLIVED_RX, log)
    _write_stub(tmp_path / "wfb_tx", _STUB_SLEEP, log)

    env = {"PATH": f"{tmp_path}:{shutil.os.environ['PATH']}"}
    start = time.monotonic()
    proc = subprocess.Popen(
        ["sh", str(script_path)],
        cwd=tmp_path,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        rc = proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
        raise AssertionError("script did not exit after a child died (wait -n not replaced?)")
    elapsed = time.monotonic() - start

    assert rc != 0
    assert elapsed < 3.0


_STUB_PID_SHORT = """#!/bin/sh
echo $$ >> "{log}"
sleep 0.2
"""

_STUB_PID_LONG = """#!/bin/sh
echo $$ >> "{log}"
exec sleep 5
"""


def test_path_stub_fail_fast_kills_sibling_children(tmp_path):
    """Critical regression guard: the cleanup trap must fire on the
    script's own fail-fast `exit 1` (i.e. `trap _cleanup EXIT`), not only
    on an externally-delivered SIGTERM (`trap _cleanup TERM`). Otherwise,
    when one child (wfb_rx) dies and the poll loop calls its own `exit 1`,
    the trap never runs and the sibling (wfb_tx) is orphaned, still
    holding the wlan.
    """
    script = _remote_script()
    script_path = tmp_path / "node.sh"
    script_path.write_text(script)
    script_path.chmod(0o755)

    log = tmp_path / "log.txt"
    log.write_text("")
    rx_pidfile = tmp_path / "rx_pids.txt"
    rx_pidfile.write_text("")
    tx_pidfile = tmp_path / "tx_pids.txt"
    tx_pidfile.write_text("")

    for name in ("iw", "ip", "nmcli"):
        _write_stub(tmp_path / name, _STUB_ONESHOT, log)
    _write_stub(tmp_path / "wfb_rx", _STUB_PID_SHORT, rx_pidfile)
    _write_stub(tmp_path / "wfb_tx", _STUB_PID_LONG, tx_pidfile)

    env = {"PATH": f"{tmp_path}:{shutil.os.environ['PATH']}"}
    proc = subprocess.Popen(
        ["sh", str(script_path)],
        cwd=tmp_path,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        proc.wait(timeout=5)
    except subprocess.TimeoutExpired:
        proc.kill()
        proc.wait(timeout=5)
        raise AssertionError("script did not exit after a child died (fail-fast broken?)")

    tx_pids = [int(p) for p in tx_pidfile.read_text().split()]
    assert tx_pids, "sibling wfb_tx stub never started"

    # Allow a short grace period for SIGTERM delivery/reaping.
    deadline = time.monotonic() + 2.0
    alive = set(tx_pids)
    while alive and time.monotonic() < deadline:
        for pid in list(alive):
            try:
                os.kill(pid, 0)
            except ProcessLookupError:
                alive.discard(pid)
        if alive:
            time.sleep(0.1)

    assert not alive, (
        f"sibling wfb_tx pid(s) {alive} still alive after the script exited "
        "(cleanup trap did not fire on the internal fail-fast exit -- "
        "must be `trap _cleanup EXIT`, not `trap _cleanup TERM`)"
    )
