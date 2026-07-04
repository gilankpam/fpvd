import json
import socket
import threading

from fpvdgs.dynlink.stats_client import RxAnt, RxEvent, SessionInfo, SettingsEvent
from fpvdgs.wfb.cli import main, render_frame
from fpvdgs.wfb.cluster import cluster_wlan_id

SES = SessionInfo(
    fec_type="swfec", fec_k=60, fec_n=30, epoch=1, interleave_depth=1, contract_version=3
)


def _rx_ant(ant, pkt_recv, rssi, snr, evm=(-1, -1, -1)):
    return RxAnt(
        ant=ant,
        freq=5660,
        mcs=5,
        bw=20,
        pkt_recv=pkt_recv,
        rssi_min=rssi[0],
        rssi_avg=rssi[1],
        rssi_max=rssi[2],
        snr_min=snr[0],
        snr_avg=snr[1],
        snr_max=snr[2],
        evm_min=evm[0],
        evm_avg=evm[1],
        evm_max=evm[2],
    )


def test_render_frame_marks_tx_selected_card_and_shows_counters():
    settings = SettingsEvent(
        profile="gs", is_cluster=False, wlans=["wlan0", "wlan1"], settings={}, timestamp=1.0
    )
    video = RxEvent(
        timestamp=1.0,
        id="video rx",
        packets_window={"fec_rec": 2, "lost": 1, "bad": 0},
        rx_ant_stats=[
            _rx_ant(0x000, 185, (-52, -48, -45), (26, 28, 30), (22, 24, 26)),
            _rx_ant(0x100, 180, (-60, -58, -55), (18, 20, 22)),
        ],
        session=SES,
        tx_wlan=1,
    )
    mavlink = RxEvent(
        timestamp=1.0,
        id="mavlink rx",
        packets_window={"fec_rec": 0, "lost": 0, "bad": 0},
        rx_ant_stats=[
            _rx_ant(0x000, 10, (-50, -49, -48), (30, 31, 32)),
        ],
        session=None,
        tx_wlan=None,
    )

    out = render_frame([settings, video, mavlink])
    lines = out.splitlines()

    assert any("profile=gs" in ln for ln in lines)
    assert any("video rx" in ln for ln in lines)
    assert any("mavlink rx" in ln for ln in lines)

    video_card0 = next(ln for ln in lines if ln.startswith("card 0") and "pkt 185" in ln)
    assert "*" not in video_card0
    video_card1 = next(ln for ln in lines if ln.startswith("card 1") and "pkt 180" in ln)
    assert "*" in video_card1
    assert "rssi -60/-58/-55" in video_card1
    assert "snr 18/20/22" in video_card1

    mav_card0 = next(ln for ln in lines if ln.startswith("card 0") and "pkt 10" in ln)
    assert "*" not in mav_card0

    assert any("fec_rec=2" in ln and "lost=1" in ln and "bad=0" in ln for ln in lines)
    assert any("swfec" in ln and "k=60" in ln and "n=30" in ln and "epoch=1" in ln for ln in lines)


def test_render_frame_shows_node_for_cluster_encoded_ant():
    """A cluster-encoded wlan id (node ipv4 packed into the high bits, per
    cluster.cluster_wlan_id) renders `node <ip> card <n>`, not a raw huge
    integer `card <n>`."""
    wlan_id = cluster_wlan_id("192.168.1.10", 0)
    video = RxEvent(
        timestamp=1.0,
        id="video rx",
        packets_window={"fec_rec": 0, "lost": 0, "bad": 0},
        rx_ant_stats=[
            _rx_ant(wlan_id << 8, 100, (-52, -48, -45), (26, 28, 30)),
        ],
        session=None,
        tx_wlan=None,
    )

    out = render_frame([video])

    assert "node 192.168.1.10 card 0" in out
    assert "ant 0" in out


def test_render_frame_local_ant_stays_plain_card():
    """A plain local ant (small wlan id, no ipv4 in the high bits) keeps the
    Phase 1 `card <n>` format, no `node` prefix."""
    video = RxEvent(
        timestamp=1.0,
        id="video rx",
        packets_window={"fec_rec": 0, "lost": 0, "bad": 0},
        rx_ant_stats=[
            _rx_ant(0x000, 100, (-52, -48, -45), (26, 28, 30)),
        ],
        session=None,
        tx_wlan=None,
    )

    out = render_frame([video])

    assert "node" not in out
    assert any(ln.startswith("card 0") for ln in out.splitlines())


def test_render_frame_marks_tx_selected_cluster_card():
    """The tx-selected marker still matches on the full (cluster-encoded)
    wlan id, unaffected by the node/card label rendering."""
    wlan_id = cluster_wlan_id("192.168.1.10", 1)
    video = RxEvent(
        timestamp=1.0,
        id="video rx",
        packets_window={"fec_rec": 0, "lost": 0, "bad": 0},
        rx_ant_stats=[
            _rx_ant(wlan_id << 8, 100, (-52, -48, -45), (26, 28, 30)),
        ],
        session=None,
        tx_wlan=wlan_id,
    )

    out = render_frame([video])
    line = next(ln for ln in out.splitlines() if "node 192.168.1.10 card 1" in ln)
    assert "*" in line


SETTINGS_LINE = (
    json.dumps(
        {
            "type": "settings",
            "profile": "gs",
            "is_cluster": False,
            "wlans": ["wlan0"],
            "settings": {"common": {"log_interval": 100}},
        }
    )
    + "\n"
)

RX_LINE = (
    json.dumps(
        {
            "type": "rx",
            "timestamp": 123.0,
            "id": "video rx",
            "tx_wlan": 0,
            "packets": {"fec_rec": [2, 2], "lost": [1, 1], "bad": [0, 0]},
            "rx_ant_stats": [
                {
                    "ant": 0,
                    "freq": 5660,
                    "mcs": 5,
                    "bw": 20,
                    "pkt_recv": 185,
                    "rssi_min": -52,
                    "rssi_avg": -48,
                    "rssi_max": -45,
                    "snr_min": 26,
                    "snr_avg": 28,
                    "snr_max": 30,
                }
            ],
            "session": {
                "fec_type": "swfec",
                "fec_k": 60,
                "fec_n": 30,
                "epoch": 1,
                "contract_version": 3,
            },
        }
    )
    + "\n"
)


def _start_fake_server():
    """A one-shot TCP server: accepts one client, writes a settings line
    and one rx line, then closes. Returns (endpoint, thread)."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def _serve():
        conn, _ = srv.accept()
        try:
            conn.sendall(SETTINGS_LINE.encode())
            conn.sendall(RX_LINE.encode())
        finally:
            conn.close()
            srv.close()

    t = threading.Thread(target=_serve, daemon=True)
    t.start()
    return f"tcp://127.0.0.1:{port}", t


def test_main_json_once_prints_raw_lines(capsys):
    endpoint, t = _start_fake_server()
    main(["--endpoint", endpoint, "--json", "--once"])
    t.join(timeout=2)

    out = capsys.readouterr().out
    assert SETTINGS_LINE.strip() in out
    assert RX_LINE.strip() in out


def test_main_once_renders_a_window(capsys):
    endpoint, t = _start_fake_server()
    main(["--endpoint", endpoint, "--once"])
    t.join(timeout=2)

    out = capsys.readouterr().out
    assert "video rx" in out
    assert any(line.startswith("card 0") and "pkt 185" in line for line in out.splitlines())
    assert "profile=gs" in out


def _start_fake_server_with_bad_contract_version():
    """Server sends a record with unsupported contract_version, then a valid rx record."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def _serve():
        conn, _ = srv.accept()
        try:
            conn.sendall(SETTINGS_LINE.encode())
            # Send rx with unsupported contract_version=4
            bad_rx = json.dumps(
                {
                    "type": "rx",
                    "timestamp": 123.0,
                    "id": "video rx",
                    "tx_wlan": 0,
                    "packets": {"fec_rec": [2, 2], "lost": [1, 1], "bad": [0, 0]},
                    "rx_ant_stats": [
                        {
                            "ant": 0,
                            "freq": 5660,
                            "mcs": 5,
                            "bw": 20,
                            "pkt_recv": 185,
                            "rssi_min": -52,
                            "rssi_avg": -48,
                            "rssi_max": -45,
                            "snr_min": 26,
                            "snr_avg": 28,
                            "snr_max": 30,
                        }
                    ],
                    "session": {
                        "fec_type": "swfec",
                        "fec_k": 60,
                        "fec_n": 30,
                        "epoch": 1,
                        "contract_version": 4,  # unsupported!
                    },
                }
            )
            conn.sendall((bad_rx + "\n").encode())
            # Send valid rx record
            conn.sendall(RX_LINE.encode())
        finally:
            conn.close()
            srv.close()

    t = threading.Thread(target=_serve, daemon=True)
    t.start()
    return f"tcp://127.0.0.1:{port}", t


def test_main_once_skips_bad_contract_version_record(capsys):
    """Bad contract_version is skipped with a clean stderr message; valid record renders."""
    endpoint, t = _start_fake_server_with_bad_contract_version()
    main(["--endpoint", endpoint, "--once"])
    t.join(timeout=2)

    captured = capsys.readouterr()
    out, err = captured.out, captured.err
    # Valid rx record should render
    assert "video rx" in out
    assert any(line.startswith("card 0") and "pkt 185" in line for line in out.splitlines())
    # Clean message on stderr (not a traceback)
    assert "unsupported feed contract_version" in err
    assert "Traceback" not in err


def _start_fake_server_with_malformed_record():
    """Server sends a record missing required fields (rssi_min), then a valid rx record."""
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", 0))
    srv.listen(1)
    port = srv.getsockname()[1]

    def _serve():
        conn, _ = srv.accept()
        try:
            conn.sendall(SETTINGS_LINE.encode())
            # Send rx missing rssi_min
            malformed_rx = json.dumps(
                {
                    "type": "rx",
                    "timestamp": 123.0,
                    "id": "video rx",
                    "tx_wlan": 0,
                    "packets": {"fec_rec": [2, 2], "lost": [1, 1], "bad": [0, 0]},
                    "rx_ant_stats": [
                        {
                            "ant": 0,
                            "freq": 5660,
                            "mcs": 5,
                            "bw": 20,
                            "pkt_recv": 185,
                            # Missing rssi_min!
                            "rssi_avg": -48,
                            "rssi_max": -45,
                            "snr_min": 26,
                            "snr_avg": 28,
                            "snr_max": 30,
                        }
                    ],
                    "session": {
                        "fec_type": "swfec",
                        "fec_k": 60,
                        "fec_n": 30,
                        "epoch": 1,
                        "contract_version": 3,
                    },
                }
            )
            conn.sendall((malformed_rx + "\n").encode())
            # Send valid rx record
            conn.sendall(RX_LINE.encode())
        finally:
            conn.close()
            srv.close()

    t = threading.Thread(target=_serve, daemon=True)
    t.start()
    return f"tcp://127.0.0.1:{port}", t


def test_main_once_skips_malformed_record(capsys):
    """Malformed record is skipped with a clean stderr message; valid record renders."""
    endpoint, t = _start_fake_server_with_malformed_record()
    main(["--endpoint", endpoint, "--once"])
    t.join(timeout=2)

    captured = capsys.readouterr()
    out, err = captured.out, captured.err
    # Valid rx record should render
    assert "video rx" in out
    assert any(line.startswith("card 0") and "pkt 185" in line for line in out.splitlines())
    # Clean message on stderr (not a traceback)
    assert "skipping malformed record" in err
    assert "Traceback" not in err
