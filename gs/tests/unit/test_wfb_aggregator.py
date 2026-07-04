import asyncio

from fpvdgs.dynlink.stats_client import RxEvent, SessionEvent, TxEvent
from fpvdgs.wfb.aggregator import StatsHub, WFBFlags
from fpvdgs.wfb.txsel import TxSelector, TxSelectorConfig

ANT = {((5660, 5, 20), 0): (95, -52, -48, -45, 26, 28, 30, 22, 24, 26)}
PKTS = {
    k: (0, 0)
    for k in (
        "all",
        "all_bytes",
        "dec_err",
        "session",
        "data",
        "uniq",
        "fec_rec",
        "lost",
        "bad",
        "out",
        "out_bytes",
    )
}
SES = {"fec_type": "swfec", "fec_k": 60, "fec_n": 30, "epoch": 1, "contract_version": 3}


def hub():
    return StatsHub(TxSelector(TxSelectorConfig()), time_fn=lambda: 123.0)


def test_rx_event_delivered_to_subscriber_loop():
    h = hub()
    got = []
    loop = asyncio.new_event_loop()
    sub = h.subscribe(loop, got.append)
    h.update_rx_stats("video rx", dict(PKTS, all=(185, 185)), ANT, SES)
    loop.run_until_complete(asyncio.sleep(0))
    loop.close()
    (ev,) = got
    assert isinstance(ev, RxEvent) and ev.id == "video rx"
    assert ev.packets_window["all"] == 185
    a = ev.rx_ant_stats[0]
    assert (a.ant, a.freq, a.mcs, a.bw, a.pkt_recv) == (0, 5660, 5, 20, 95)
    assert (a.rssi_avg, a.snr_avg, a.evm_avg) == (-48, 28, 24)
    assert ev.session.fec_type == "swfec" and ev.timestamp == 123.0
    sub.close()


# (b) process_new_session emits a SessionEvent to subscribers.
def test_process_new_session_emits_session_event():
    h = hub()
    got = []
    loop = asyncio.new_event_loop()
    sub = h.subscribe(loop, got.append)
    h.process_new_session("video rx", SES)
    loop.run_until_complete(asyncio.sleep(0))
    loop.close()
    (ev,) = got
    assert isinstance(ev, SessionEvent)
    assert ev.id == "video rx"
    assert ev.session.fec_type == "swfec"
    assert ev.session.fec_k == 60
    assert ev.timestamp == 123.0
    sub.close()


# (c) aggregate_window pkt-weighted merge across two children matches
# hand-computed values (weights = each child's pkt_recv for that ant_id).
def test_aggregate_window_merges_two_children_pkt_weighted():
    h = hub()
    ant_a = {((5660, 5, 20), 0): (100, -60, -50, -40, 20, 25, 30, -1, -1, -1)}
    ant_b = {((5745, 5, 20), 0): (50, -55, -45, -35, 22, 27, 32, -1, -1, -1)}
    h.update_rx_stats("video rx", dict(PKTS), ant_a, None)
    h.update_rx_stats("mavlink rx", dict(PKTS), ant_b, None)

    out = h.aggregate_window()

    # pkt_s=150; rssi_min=min(-60,-55); rssi_avg=(-50*100 + -45*50)//150;
    # rssi_max=max(-40,-35); snr_min=min(20,22); snr_avg=(25*100+27*50)//150;
    # snr_max=max(30,32) — integer floor division per the wfb-ng port.
    assert out["stats_agg"] == {0: (150, -60, -49, -35, 20, 25, 32)}
    assert out["tx_sel"] == 0  # single wlan (ant_id 0 >> 8), switches from None
    # single-card mavlink tuple: (rssi_avg, rssi_avg - snr_avg) = (-49, -74)
    assert out["rssi"][:2] == (-49, -74)
    assert out["rssi"][4] == 0  # no bad packets -> no flags


# (d) rssi_cb gets (-128, -128, ..., LINK_LOST) on an empty window, and
# LINK_JAMMED when bad > 0.
def test_rssi_cb_link_lost_on_empty_window():
    h = hub()
    got = []
    h.add_rssi_cb(lambda *args: got.append(args))  # rssi_cb is 5 positional args
    out = h.aggregate_window()
    assert out["rssi"] == (-128, -128, 0, 0, WFBFlags.LINK_LOST)
    assert got == [(-128, -128, 0, 0, WFBFlags.LINK_LOST)]


def test_rssi_cb_link_jammed_on_bad_packets():
    h = hub()
    got = []
    h.add_rssi_cb(lambda *args: got.append(args))
    ant = {((5660, 5, 20), 0): (10, -60, -55, -50, 20, 22, 25, -1, -1, -1)}
    pkts = dict(PKTS, bad=(1, 1))
    h.update_rx_stats("video rx", pkts, ant, None)

    out = h.aggregate_window()

    assert out["rssi"][4] == WFBFlags.LINK_JAMMED
    assert out["rssi"][2] == 1  # rx_errors = dec_err[0]+bad[0]+lost[0] = 0+1+0
    assert got[-1][4] == WFBFlags.LINK_JAMMED


# (e) ant_sel_cb fires at registration with the current value, and again on
# an actual switch.
def test_ant_sel_cb_fires_at_registration_and_on_switch():
    h = hub()
    got = []
    h.add_ant_sel_cb(got.append)
    assert got == [None]  # wfb-ng contract: fire once at registration

    ant = {((5660, 5, 20), 0): (10, -60, -55, -50, 20, 22, 25, -1, -1, -1)}
    h.update_rx_stats("video rx", dict(PKTS), ant, None)
    h.aggregate_window()

    assert got == [None, 0]  # switched onto wlan 0


# (f) subscriber queue drops oldest beyond 256 without blocking update_rx_stats.
def test_subscriber_queue_drops_oldest_beyond_256():
    h = hub()
    loop = asyncio.new_event_loop()  # never driven -> simulates a stalled consumer
    sub = h.subscribe(loop, lambda ev: None)
    try:
        for i in range(300):
            h.update_rx_stats(f"child{i}", dict(PKTS), {}, None)
        assert len(sub.queue) == 256
        assert sub.queue[0].id == "child44"  # oldest 44 dropped (300-256)
        assert sub.queue[-1].id == "child299"
    finally:
        sub.close()
        loop.close()


# (g) NativeStatsSource (via client_factory()) delivers events through
# asyncio.run() with the StatsClient ctor shape: (endpoint, on_event).
def test_native_stats_source_delivers_events_and_replays_session():
    h = hub()
    h.process_new_session("video rx", SES)  # known before the consumer connects
    got = []
    Cls = h.client_factory()

    async def _drive():
        client = Cls("tcp://ignored:0", got.append)  # StatsClient-compatible ctor
        run_task = asyncio.ensure_future(client.run())
        await asyncio.sleep(0)  # let run() subscribe + replay
        h.update_rx_stats("video rx", dict(PKTS, all=(1, 1)), ANT, SES)
        h.update_tx_stats("video tx", dict.fromkeys(("fec_timeouts",), (0, 0)), {})
        await asyncio.sleep(0)
        client.stop()
        await run_task

    asyncio.run(_drive())

    assert any(isinstance(ev, SessionEvent) for ev in got)  # replay on connect
    assert any(isinstance(ev, RxEvent) for ev in got)
    assert any(isinstance(ev, TxEvent) for ev in got)
