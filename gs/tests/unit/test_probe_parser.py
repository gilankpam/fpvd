from fpvdgs.probe import parser

def test_parse_rx_ant():
    ev = parser.parse_line("5952334\tRX_ANT\t5805:3:20\t0\t2:-58:-57:-57:25:25:26")
    assert ev == ("RX_ANT", {"mcs": 3, "rssi": -57, "snr": 25})

def test_parse_pkt():
    ev = parser.parse_line("99\tPKT\t6:8568:0:0:6:3:0:0:0:3:4200:0:0:0")
    assert ev == ("PKT", {"data": 6, "fec_rec": 0, "lost": 0})

def test_parse_pkt_11field_legacy():
    ev = parser.parse_line("99\tPKT\t10:5000:0:0:8:8:1:1:0:8:4000")
    assert ev == ("PKT", {"data": 8, "fec_rec": 1, "lost": 1})

def test_parse_ignores_session_and_garbage():
    assert parser.parse_line("9\tSESSION\t0:1:1:1") is None
    assert parser.parse_line("not a stats line") is None

def test_aggregator_ewma_and_per():
    agg = parser.McsAggregator(alpha=0.5)
    agg.on_rx_ant(mcs=5, rssi=-60, snr=20)
    agg.on_pkt(mcs=5, data=90, lost=10)   # window PER = 10/100 = 0.10
    agg.on_pkt(mcs=5, data=98, lost=2)    # window PER = 2/100 = 0.02; EWMA -> 0.06
    snap = agg.snapshot()
    assert snap[5]["rssi"] == -60
    assert snap[5]["snr"] == 20
    assert abs(snap[5]["per"] - 0.06) < 1e-9
    assert snap[5]["windows"] == 2

def test_aggregator_blackout_window_is_full_loss():
    agg = parser.McsAggregator(alpha=1.0)
    agg.on_pkt(mcs=7, data=0, lost=0)     # nothing decoded this window
    assert agg.snapshot()[7]["per"] == 1.0
