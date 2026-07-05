from fpvdgs.wfb.txsel import TxSelector, TxSelectorConfig


def agg(**wlans):
    """wlans: w0=(pkts, rssi), ... -> stats_agg keyed ant_id = wlan<<8."""
    out = {}
    for name, (pkts, rssi) in wlans.items():
        wlan = int(name[1:])
        out[wlan << 8] = (pkts, rssi - 5, rssi, rssi + 5, 10, 12, 14)
    return out


def test_cold_start_picks_best():
    s = TxSelector(TxSelectorConfig())
    assert s.select(agg(w0=(100, -60), w1=(100, -50))) == 1
    assert s.current == 1


def test_hysteresis_blocks_small_rssi_gain():
    s = TxSelector(TxSelectorConfig())
    s.select(agg(w0=(100, -50), w1=(100, -60)))
    assert s.current == 0
    # w1 is 2 dB better: below rssi_delta_db=3 -> hold
    assert s.select(agg(w0=(100, -52), w1=(100, -50))) is None
    assert s.current == 0
    # 4 dB better -> switch
    assert s.select(agg(w0=(100, -54), w1=(100, -50))) == 1


def test_packet_count_gate_beats_rssi():
    s = TxSelector(TxSelectorConfig())
    s.select(agg(w0=(100, -50), w1=(100, -60)))
    # w1 has huge RSSI but is missing most packets -> not eligible
    assert s.select(agg(w0=(100, -55), w1=(10, -30))) is None
    assert s.current == 0


def test_rx_only_wlan_never_selected():
    s = TxSelector(TxSelectorConfig(), rx_only_wlan_ids=frozenset({1}))
    assert s.select(agg(w0=(50, -70), w1=(100, -40))) == 0


def test_current_dies_forces_switch():
    s = TxSelector(TxSelectorConfig())
    s.select(agg(w0=(100, -50), w1=(100, -60)))
    # w0 stops receiving entirely -> w1 wins even with worse RSSI
    assert s.select(agg(w0=(0, -50), w1=(100, -60))) == 1


def test_empty_agg_is_noop():
    s = TxSelector(TxSelectorConfig())
    assert s.select({}) is None
    assert s.current is None
