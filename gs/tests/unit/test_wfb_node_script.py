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
    assert "(sleep 1; exec cat > /dev/null) &" in script  # ssh watchdog


def test_no_bashisms():
    script = _remote_script()
    assert script.startswith("#!/bin/sh\n")
    assert "wait -n" not in script
    assert "set -b" not in script
    assert "set -emb" not in script


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
