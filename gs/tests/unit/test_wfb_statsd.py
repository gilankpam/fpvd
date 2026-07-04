import asyncio
import time

from fpvdgs.dynlink.stats_client import (
    RxEvent,
    SessionEvent,
    SettingsEvent,
    iter_events_from_reader,
)
from fpvdgs.wfb.aggregator import StatsHub
from fpvdgs.wfb.statsd import StatsServer
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
    return StatsHub(TxSelector(TxSelectorConfig()))


async def _wait_until(cond, timeout=2.0):
    start = time.monotonic()
    while not cond():
        if time.monotonic() - start > timeout:
            raise AssertionError("timed out waiting for condition")
        await asyncio.sleep(0.01)


def test_round_trip_through_real_tcp_stats_feed():
    # No StatsClient anymore (native mode uses an in-process factory) — drive
    # the wire contract directly with a raw TCP connection + the shared
    # newline-JSON decoder, which is still the real e2e path an external
    # consumer (fpvd-stats CLI, `ss -tln` health check) would use.
    h = hub()
    settings_fn = lambda: {"common": {"log_interval": 100}}  # noqa: E731
    server = StatsServer(h, settings_fn, host="127.0.0.1", port=0)
    got: list = []

    async def _drive():
        loop = asyncio.get_running_loop()
        await server.start(loop)
        try:
            reader, writer = await asyncio.open_connection("127.0.0.1", server.port)

            async def _consume():
                async for ev in iter_events_from_reader(reader):
                    got.append(ev)

            consume_task = asyncio.ensure_future(_consume())
            try:
                await _wait_until(lambda: any(isinstance(e, SettingsEvent) for e in got))

                h.process_new_session("video rx", SES)
                h.update_rx_stats("video rx", dict(PKTS, all=(185, 185)), ANT, SES)

                await _wait_until(lambda: any(isinstance(e, RxEvent) for e in got))
            finally:
                writer.close()
                consume_task.cancel()
                try:
                    await consume_task
                except (asyncio.CancelledError, Exception):
                    pass
        finally:
            await server.stop()

    asyncio.run(_drive())

    settings_ev = next(e for e in got if isinstance(e, SettingsEvent))
    assert settings_ev.profile == "gs"
    assert settings_ev.is_cluster is False
    assert settings_ev.settings == {"common": {"log_interval": 100}}

    session_ev = next(e for e in got if isinstance(e, SessionEvent))
    assert session_ev.id == "video rx"
    assert session_ev.session.fec_type == "swfec"
    assert session_ev.session.fec_k == 60
    assert session_ev.session.fec_n == 30
    assert session_ev.session.epoch == 1
    assert session_ev.session.contract_version == 3

    rx_ev = next(e for e in got if isinstance(e, RxEvent))
    assert rx_ev.id == "video rx"
    assert rx_ev.packets_window["all"] == 185
    (ant,) = rx_ev.rx_ant_stats
    assert (ant.ant, ant.freq, ant.mcs, ant.bw, ant.pkt_recv) == (0, 5660, 5, 20, 95)
    assert (ant.rssi_min, ant.rssi_avg, ant.rssi_max) == (-52, -48, -45)
    assert (ant.snr_min, ant.snr_avg, ant.snr_max) == (26, 28, 30)
    assert (ant.evm_min, ant.evm_avg, ant.evm_max) == (22, 24, 26)
    assert rx_ev.session is not None and rx_ev.session.fec_type == "swfec"


def test_slow_client_is_disconnected_when_write_buffer_exceeds_cap():
    h = hub()
    settings_fn = lambda: {"common": {}}  # noqa: E731
    server = StatsServer(h, settings_fn, host="127.0.0.1", port=0, max_write_buffer=4096)

    async def _drive():
        loop = asyncio.get_running_loop()
        await server.start(loop)
        try:
            reader, writer = await asyncio.open_connection("127.0.0.1", server.port)
            try:
                await _wait_until(lambda: len(server._writers) >= 1)

                # Never read from `reader` -- flood the hub so the server's
                # write buffer for this client backs up past the (small,
                # test-only) cap.
                big_packets = dict(PKTS)
                for i in range(20000):
                    h.update_rx_stats(f"video rx {i}", big_packets, ANT, None)
                    if i % 500 == 0:
                        await asyncio.sleep(0)

                await _wait_until(lambda: len(server._writers) == 0, timeout=5.0)
            finally:
                writer.close()
        finally:
            await server.stop()

    asyncio.run(_drive())
