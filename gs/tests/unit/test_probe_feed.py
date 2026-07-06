"""ProbeFeed — in-process probe measurement (2026-07-06 spec Part B)."""

from fpvdgs.probe.feed import PROBE_FRESH_S, ProbeFeed


class FakeClock:
    def __init__(self):
        self.t = 100.0

    def __call__(self):
        return self.t


def _ant(mcs, snr=25):
    return f"1000\tRX_ANT\t5660:{mcs}:20\t0\t10:-70:-65:-60:20:{snr}:28"


def _pkt(data, lost):
    # 11 cumulative counters; parser reads data=idx4, fec_rec=idx6, lost=idx7
    return f"1000\tPKT\t{data + lost}:0:0:0:{data}:0:0:{lost}:0:{data}:0"


def test_routes_pkt_to_latest_rx_ant_mcs_and_reports_fresh():
    clk = FakeClock()
    f = ProbeFeed(time_fn=clk)
    f.feed_line(_ant(5))
    f.feed_line(_pkt(data=10, lost=0))
    snap = f.snapshot_fresh()
    assert snap[5]["per"] == 0.0
    assert snap[5]["fresh"] is True
    assert snap[5]["snr"] == 25


def test_stale_after_fresh_window():
    clk = FakeClock()
    f = ProbeFeed(time_fn=clk)
    f.feed_line(_ant(5))
    f.feed_line(_pkt(data=10, lost=1))
    clk.t += PROBE_FRESH_S + 0.1
    assert f.snapshot_fresh()[5]["fresh"] is False


def test_empty_windows_do_not_refresh_freshness():
    """Spec deviation 1: only windows that carried packets stamp freshness,
    so a blackout decays to neutral instead of vetoing forever."""
    clk = FakeClock()
    f = ProbeFeed(time_fn=clk)
    f.feed_line(_ant(5))
    f.feed_line(_pkt(data=10, lost=0))
    clk.t += 0.3
    for _ in range(12):  # > PROBE_BLACKOUT_WINDOWS empties -> per pinned 1.0
        f.feed_line(_pkt(data=0, lost=0))
    snap = f.snapshot_fresh()
    assert snap[5]["per"] == 1.0  # blackout pin (aggregator contract)
    assert snap[5]["fresh"] is True  # still within 0.5s of the last real packet
    clk.t += PROBE_FRESH_S
    assert f.snapshot_fresh()[5]["fresh"] is False  # decays to neutral


def test_pkt_before_any_rx_ant_is_dropped():
    f = ProbeFeed(time_fn=FakeClock())
    f.feed_line(_pkt(data=5, lost=5))  # no MCS label yet -> unroutable
    assert f.snapshot_fresh() == {}


def test_garbage_lines_ignored():
    f = ProbeFeed(time_fn=FakeClock())
    f.feed_line("not a stats line")
    f.feed_line("1000\tRX_ANT\tmangled")
    assert f.snapshot_fresh() == {}
