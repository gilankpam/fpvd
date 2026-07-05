"""Golden tests for the wfb_rx/wfb_tx stdout line protocol parser."""

from fpvdgs.wfb.lineproto import RxLineParser, TxLineParser

# Captured live from forked wfb_rx (contract v3, EVM triple appended).
RX_LINES = [
    "1719400000100\tSESSION\t42:2:60:30:3",
    "1719400000100\tRX_ANT\t5660:5:20\t0\t95:-52:-48:-45:26:28:30:22:24:26",
    "1719400000100\tRX_ANT\t5660:5:20\t1\t90:-60:-55:-50:20:22:24:18:20:22",
    "1719400000100\tPKT\t185:214785:0:1:184:180:4:2:0:180:208913",
]


def _collect():
    windows, sessions = [], []
    p = RxLineParser(
        "video rx",
        on_window=lambda cid, pk, ant, ses: windows.append((cid, pk, ant, ses)),
        on_session=lambda cid, ses: sessions.append((cid, ses)),
    )
    return p, windows, sessions


def test_rx_session_parsed_and_dedup():
    p, windows, sessions = _collect()
    for line in RX_LINES:
        p.feed_line(line)
    # SESSION fired once (change), carries decoded fec_type name
    assert sessions == [
        (
            "video rx",
            {
                "fec_type": "swfec",
                "fec_k": 60,
                "fec_n": 30,
                "epoch": 42,
                "contract_version": 3,
            },
        )
    ]
    # re-feeding the identical SESSION line (per-window re-emit) is silent
    p.feed_line(RX_LINES[0])
    assert len(sessions) == 1


def test_rx_window_ant_and_packets():
    p, windows, _ = _collect()
    for line in RX_LINES:
        p.feed_line(line)
    assert len(windows) == 1
    cid, packets, ant, session = windows[0]
    assert cid == "video rx"
    # ((freq, mcs, bw), ant_id) -> 10-tuple incl. EVM
    assert ant[((5660, 5, 20), 0)] == (95, -52, -48, -45, 26, 28, 30, 22, 24, 26)
    assert ant[((5660, 5, 20), 1)][0] == 90
    # (window, cumulative) pairs; first window: both equal
    assert packets["all"] == (185, 185)
    assert packets["lost"] == (2, 2)
    assert session["fec_type"] == "swfec"
    # window flushes the ant dict
    p.feed_line("1719400000200\tPKT\t10:1000:0:0:10:10:0:0:0:10:900")
    assert windows[1][2] == {}
    # cumulative accumulates
    assert windows[1][1]["all"] == (10, 195)


def test_rx_ant_without_evm_pads_negative_one():
    p, windows, _ = _collect()
    p.feed_line("100\tRX_ANT\t5660:1:20\t0\t50:-70:-65:-60:10:12:14")
    p.feed_line("100\tPKT\t50:1000:0:0:50:50:0:0:0:50:900")
    assert windows[0][2][((5660, 1, 20), 0)] == (50, -70, -65, -60, 10, 12, 14, -1, -1, -1)


def test_rx_malformed_lines_skipped():
    p, windows, sessions = _collect()
    p.feed_line("garbage")
    p.feed_line("100\tRX_ANT\tbad")
    p.feed_line("100\tPKT\t1:2:3")  # too few counters
    assert windows == [] and sessions == []


def test_rx_ant_short_key_skipped_not_indexerror():
    # A 2-part "freq:mcs" key (missing bw) must be rejected by the parser
    # itself, not silently accepted and left to IndexError downstream in
    # aggregator._to_rx_event (which unpacks key as (freq, mcs, bw)).
    p, windows, _ = _collect()
    p.feed_line("100\tRX_ANT\t5660:5\t0\t95:-52:-48:-45:26:28:30:22:24:26")
    p.feed_line("100\tPKT\t50:1000:0:0:50:50:0:0:0:50:900")
    assert windows[0][2] == {}  # malformed ant entry skipped, window still flushes


def test_tx_handshake_unix_sockets_and_control_port():
    done = []
    p = TxLineParser("mavlink tx", on_window=lambda *a: None)
    p.on_handshake = lambda: done.append(True)
    p.feed_line("100\tLISTEN_UNIX\tmavlink-tx-a1b2:0")
    p.feed_line("100\tLISTEN_UNIX\tmavlink-tx-c3d4:1")
    p.feed_line("100\tLISTEN_UNIX_END")
    p.feed_line("100\tLISTEN_UDP_CONTROL\t14100")
    assert p.unix_sockets == {0: "mavlink-tx-a1b2", 1: "mavlink-tx-c3d4"}
    assert p.control_port == 14100
    assert p.handshake_done and done == [True]


def test_tx_window_counters_and_latency():
    wins = []
    p = TxLineParser("tun tx", on_window=lambda cid, pk, lat: wins.append((cid, pk, lat)))
    p.feed_line("100\tTX_ANT\t1\t120:0:900:1100:1500")
    p.feed_line("100\tPKT\t0:120:14000:120:15000:0:0")
    cid, packets, lat = wins[0]
    assert packets["injected"] == (120, 120)
    assert lat[1] == (120, 0, 900, 1100, 1500)
