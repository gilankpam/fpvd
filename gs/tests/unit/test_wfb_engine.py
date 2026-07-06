"""Tests for WfbEngine — the loop-in-a-thread orchestrator tying every wfb/
module together into one ProcessSupervisor-compatible object.

Everything is faked: `child_cls` is a stub recording start/stop order (and
optionally failing by name), `radio_init` is a recorder, `graph_builder`
returns a tiny fake graph, and `mav_service_cls`/`tunnel_service_cls` are
stubs recording their wiring calls. No real subprocesses, sockets, or tun
devices are touched.
"""

from __future__ import annotations

import contextlib
import time
from types import SimpleNamespace

from fpvdgs.wfb.engine import WfbEngine

WLANS = ["wlan0", "wlan1"]

RX_NAMES = ("video_rx", "mavlink_rx", "tunnel_rx")
TX_NAMES = ("mavlink_tx", "tunnel_tx")


def make_config(**overrides):
    cfg = {
        "link": {"wlans": "auto", "width": 20},
        "wfb": {"txSelector": {"rssiDeltaDb": 3, "counterRelDelta": 0.1, "counterAbsDelta": 3}},
    }
    cfg.update(overrides)
    return cfg


def make_spec(name, kind):
    return SimpleNamespace(name=name, kind=kind, argv=[f"/bin/{name}"], parser=kind, unix_path=None)


def make_graph_builder(argv_tag="v1"):
    def _build(effective, wlans, *, rand_suffix):
        specs = {name: make_spec(name, "rx") for name in RX_NAMES}
        specs.update({name: make_spec(name, "tx") for name in TX_NAMES})
        for name, spec in specs.items():
            spec.argv = [f"/bin/{name}", f"--tag={argv_tag}", *wlans]
        return SimpleNamespace(
            video_rx=specs["video_rx"],
            mavlink_rx=specs["mavlink_rx"],
            mavlink_tx=specs["mavlink_tx"],
            tunnel_rx=specs["tunnel_rx"],
            tunnel_tx=specs["tunnel_tx"],
            mav_rx_sock="mav-rx-sock",
            mav_peer=SimpleNamespace(peer="connect://127.0.0.1:14550"),
            tun_rx_sock="tun-rx-sock",
            tun_cfg=SimpleNamespace(ifname="gs-wfb"),
        )

    return _build


def make_probe_graph_builder(argv_tag="v1"):
    """Mirrors real `graph.py`'s `dl.enabled AND dl.probe.enabled` gate: the
    returned graph carries a `probe_rx` ServiceSpec-alike (parser="probe")
    exactly when the effective config it is handed has both flags set --
    same shape as `make_graph_builder`, plus that one extra field."""

    def _build(effective, wlans, *, rand_suffix):
        graph = make_graph_builder(argv_tag)(effective, wlans, rand_suffix=rand_suffix)
        dl = effective.get("dynamicLink") or {}
        probe_on = bool(dl.get("enabled", False)) and bool(
            (dl.get("probe") or {}).get("enabled", False)
        )
        graph.probe_rx = make_spec("probe_rx", "rx") if probe_on else None
        return graph

    return _build


class FakeTxParser:
    def __init__(self, unix_sockets=None):
        self.unix_sockets = dict(unix_sockets or {})


def make_child_cls(order, fail_names=frozenset()):
    """Factory returning a WfbChild-compatible stub class. `order` is a
    shared list every instance appends ("start", name)/("stop", name) to,
    so tests can assert both membership and ordering."""

    class _FakeChild:
        _next_pid = [1000]

        def __init__(self, spec, hub, *, on_fault=None, **kw):
            self.spec = spec
            self.hub = hub
            self._on_fault = on_fault
            self.kwargs = kw  # records e.g. sink=... passed for the probe leg
            self.running = False
            self._pid = _FakeChild._next_pid[0]
            _FakeChild._next_pid[0] += 1
            self.tx_parser = FakeTxParser({0: f"{spec.name}-sock-0"}) if spec.kind == "tx" else None
            self.autoRestarts = 0
            self.lastExit = None
            self.fault = False

        async def start(self):
            order.append(("start", self.spec.name))
            if self.spec.name in fail_names:
                self.running = False
                return False
            self.running = True
            return True

        async def stop(self):
            order.append(("stop", self.spec.name))
            self.running = False

        def state(self):
            return {
                "running": self.running,
                "pid": self._pid if self.running else None,
                "restarts": 0,
                "autoRestarts": self.autoRestarts,
                "lastExit": self.lastExit,
                "fault": self.fault,
            }

        def trigger_fault(self):
            self.fault = True
            if self._on_fault is not None:
                self._on_fault(self)

    return _FakeChild


def make_service_cls(started, stopped, set_tx_calls, set_all_calls=None):
    """Factory for a fake MavlinkService/TunnelService. `started`/`stopped`
    are shared lists recording lifecycle calls; `set_tx_calls` records every
    `set_tx_socket` argument; `set_all_calls` (tunnel only) records every
    `set_all_tx_sockets` argument."""

    class _FakeService:
        def __init__(self, cfg):
            self.cfg = cfg

        async def start(self, loop, rx_unix_path, *args, **kwargs):
            started.append(rx_unix_path)

        async def stop(self):
            stopped.append(True)

        def set_tx_socket(self, name):
            set_tx_calls.append(name)

        def set_all_tx_sockets(self, names):
            if set_all_calls is not None:
                set_all_calls.append(list(names))

        def rssi_cb(self, *args):
            pass

    return _FakeService


def make_engine(
    order=None,
    fail_names=frozenset(),
    config=None,
    radio_calls=None,
    radio_ok=True,
    argv_tag="v1",
    mav_set_tx=None,
    tun_set_tx=None,
    tun_set_all=None,
    graph_builder=None,
):
    order = order if order is not None else []
    radio_calls = radio_calls if radio_calls is not None else []
    cfg = config if config is not None else make_config()

    def radio_init(wlans, link):
        radio_calls.append((tuple(wlans), dict(link)))
        return radio_ok

    mav_started, mav_stopped = [], []
    tun_started, tun_stopped = [], []
    mav_set_tx = mav_set_tx if mav_set_tx is not None else []
    tun_set_tx = tun_set_tx if tun_set_tx is not None else []
    tun_set_all = tun_set_all if tun_set_all is not None else []

    engine = WfbEngine(
        config_provider=lambda: cfg,
        wlans_resolver=lambda c: list(WLANS),
        graph_builder=graph_builder if graph_builder is not None else make_graph_builder(argv_tag),
        child_cls=make_child_cls(order, fail_names),
        radio_init=radio_init,
        stats_port=0,
        mav_service_cls=make_service_cls(mav_started, mav_stopped, mav_set_tx),
        tunnel_service_cls=make_service_cls(tun_started, tun_stopped, tun_set_tx, tun_set_all),
    )
    return engine, {
        "order": order,
        "radio_calls": radio_calls,
        "mav_started": mav_started,
        "mav_stopped": mav_stopped,
        "tun_started": tun_started,
        "tun_stopped": tun_stopped,
        "mav_set_tx": mav_set_tx,
        "tun_set_tx": tun_set_tx,
        "tun_set_all": tun_set_all,
    }


def _wait_until(predicate, timeout=5.0, interval=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


# -- (a) start() True, radio_init before children, rx before tx -------------


def test_start_returns_true_radio_before_children_rx_before_tx():
    engine, rec = make_engine()
    try:
        assert engine.start() is True
        assert len(rec["radio_calls"]) == 1
        assert rec["radio_calls"][0][0] == tuple(WLANS)

        starts = [name for kind, name in rec["order"] if kind == "start"]
        assert starts.index("video_rx") < starts.index("mavlink_tx")
        assert starts.index("mavlink_rx") < starts.index("mavlink_tx")
        assert starts.index("tunnel_rx") < starts.index("tunnel_tx")
        assert set(starts) == set(RX_NAMES) | set(TX_NAMES)

        st = engine.state()
        assert st["running"] is True
        assert st["fault"] is False
    finally:
        engine.shutdown()


# -- probe leg (2026-07-06 spec Part B): spawned with an in-process
# ProbeFeed sink when dynamicLink.probe.enabled, absent + no feed otherwise.


def test_engine_spawns_probe_leg_and_exposes_feed():
    cfg = make_config(dynamicLink={"enabled": True, "probe": {"enabled": True}})
    engine, rec = make_engine(config=cfg, graph_builder=make_probe_graph_builder())
    try:
        assert engine.start() is True

        starts = [name for kind, name in rec["order"] if kind == "start"]
        assert "probe_rx" in starts

        assert engine.probe_feed is not None
        probe_child = engine._children["probe_rx"]
        assert probe_child.kwargs.get("sink") is engine.probe_feed
    finally:
        engine.shutdown()


def test_engine_probe_disabled_no_leg_no_feed():
    engine, rec = make_engine()
    try:
        assert engine.start() is True

        starts = [name for kind, name in rec["order"] if kind == "start"]
        assert "probe_rx" not in starts
        assert engine.probe_feed is None
    finally:
        engine.shutdown()


# -- (b) tx child start failure -> start() False, everything torn down ------


def test_tx_child_failure_tears_down_everything_started():
    engine, rec = make_engine(fail_names={"mavlink_tx"})
    try:
        assert engine.start() is False

        starts = [name for kind, name in rec["order"] if kind == "start"]
        stops = [name for kind, name in rec["order"] if kind == "stop"]

        # Everything that was started before the failure must be stopped,
        # including the failed child itself.
        assert set(stops) == set(starts)
        # tunnel_tx never got a chance to start (mavlink_tx failed first).
        assert "tunnel_tx" not in starts

        st = engine.state()
        assert st["running"] is False
    finally:
        engine.shutdown()


# -- (c) restart() bumps restarts + rebuilds from changed config -------------


def test_restart_bumps_restarts_and_rebuilds_from_changed_config():
    cfg = make_config()
    order = []
    radio_calls = []

    def radio_init(wlans, link):
        radio_calls.append(1)
        return True

    seen_argvs = []

    def child_cls_factory():
        base = make_child_cls(order)

        class _RecordingChild(base):
            async def start(self):
                seen_argvs.append(tuple(self.spec.argv))
                return await super().start()

        return _RecordingChild

    tag_holder = {"tag": "v1"}

    def graph_builder(effective, wlans, *, rand_suffix):
        return make_graph_builder(tag_holder["tag"])(effective, wlans, rand_suffix=rand_suffix)

    engine = WfbEngine(
        config_provider=lambda: cfg,
        wlans_resolver=lambda c: list(WLANS),
        graph_builder=graph_builder,
        child_cls=child_cls_factory(),
        radio_init=radio_init,
        stats_port=0,
        mav_service_cls=make_service_cls([], [], []),
        tunnel_service_cls=make_service_cls([], [], [], []),
    )
    try:
        assert engine.start() is True
        assert engine.state()["restarts"] == 0
        first_video_argv = next(a for a in seen_argvs if a[0] == "/bin/video_rx")
        assert "--tag=v1" in first_video_argv

        tag_holder["tag"] = "v2"
        assert engine.restart() is True
        assert engine.state()["restarts"] == 1

        second_video_argv = [a for a in seen_argvs if a[0] == "/bin/video_rx"][-1]
        assert "--tag=v2" in second_video_argv
    finally:
        engine.shutdown()


def test_reap_stale_wfb_kills_all_but_self_tolerating_dead():
    import signal as _sig

    from fpvdgs.wfb.engine import reap_stale_wfb

    killed = []

    def fake_kill(pid, sig):
        if pid == 999:
            raise ProcessLookupError  # already gone
        killed.append((pid, sig))

    reaped = reap_stale_wfb(pids_fn=lambda: [10, 20, 42, 999], kill=fake_kill, self_pid=42)
    assert reaped == [10, 20]  # 42 skipped (self), 999 tolerated (dead)
    assert killed == [(10, _sig.SIGKILL), (20, _sig.SIGKILL)]


def test_engine_reaps_stale_wfb_before_starting_children():
    # A prior incarnation's orphaned wfb children hold our ports; the engine must
    # reap them BEFORE spawning its own, or the new children fail to bind (bench-
    # caught 2026-07-04 as a cascading apply 500).
    cfg = make_config()
    order = []
    engine = WfbEngine(
        config_provider=lambda: cfg,
        wlans_resolver=lambda c: list(WLANS),
        graph_builder=make_graph_builder(),
        child_cls=make_child_cls(order),
        radio_init=lambda wlans, link: order.append(("radio", None)) or True,
        stats_port=0,
        mav_service_cls=make_service_cls([], [], []),
        tunnel_service_cls=make_service_cls([], [], [], []),
        reap_fn=lambda: order.append(("reap", None)) or [],
    )
    try:
        assert engine.start() is True
        assert ("reap", None) in order
        reap_i = order.index(("reap", None))
        first_start_i = next(i for i, e in enumerate(order) if e[0] == "start")
        assert reap_i < first_start_i  # reaped before any child started
    finally:
        engine.shutdown()


def test_restart_with_config_builds_from_it_not_provider_and_consumes_once():
    # restart(config) must build from `config` — the pending config the api hands
    # in before store.commit() — NOT config_provider(), which still returns the
    # OLD committed store during an apply (bench-caught 2026-07-04). And a plain
    # restart() (reset/boot/crash) falls back to the provider: staged is one-shot.
    provider_cfg = make_config(_tag="PROVIDER")
    staged_cfg = make_config(_tag="STAGED")
    seen = []

    def graph_builder(effective, wlans, *, rand_suffix):
        seen.append(effective.get("_tag"))
        return make_graph_builder("x")(effective, wlans, rand_suffix=rand_suffix)

    engine = WfbEngine(
        config_provider=lambda: provider_cfg,
        wlans_resolver=lambda c: list(WLANS),
        graph_builder=graph_builder,
        child_cls=make_child_cls([]),
        radio_init=lambda wlans, link: True,
        stats_port=0,
        mav_service_cls=make_service_cls([], [], []),
        tunnel_service_cls=make_service_cls([], [], [], []),
    )
    try:
        assert engine.start() is True
        assert seen[-1] == "PROVIDER"  # boot: no staged config -> provider
        assert engine.restart(staged_cfg) is True
        assert seen[-1] == "STAGED"  # restart(config) built from the staged config
        assert engine.restart() is True
        assert seen[-1] == "PROVIDER"  # staged consumed once -> falls back to provider
    finally:
        engine.shutdown()


# -- (d) state() aggregation incl. fault propagation -------------------------


def test_state_aggregates_children_and_propagates_fault():
    engine, rec = make_engine()
    try:
        assert engine.start() is True
        st = engine.state()
        assert st["running"] is True
        assert st["pid"] is not None
        assert st["fault"] is False

        # Reach into the running engine's children to simulate a crash-loop
        # fault on one child (this is what WfbChild's own on_fault callback
        # would trigger in production).
        child = engine._children["tunnel_rx"]
        child.trigger_fault()

        assert _wait_until(lambda: engine.state()["fault"] is True)
    finally:
        engine.shutdown()


# -- (e) stop() from a non-engine thread joins cleanly -----------------------


def test_stop_joins_cleanly_from_calling_thread():
    engine, rec = make_engine()
    assert engine.start() is True

    engine.stop()

    assert engine.state()["running"] is False
    stops = [name for kind, name in rec["order"] if kind == "stop"]
    assert set(stops) == set(RX_NAMES) | set(TX_NAMES)
    # tx children torn down before rx children.
    assert max(stops.index(n) for n in TX_NAMES) < min(stops.index(n) for n in RX_NAMES)


# -- (f) ant-sel fire re-targets services after a simulated tx respawn -------


def test_ant_sel_retargets_services_after_simulated_tx_respawn():
    mav_set_tx, tun_set_tx, tun_set_all = [], [], []
    engine, rec = make_engine(mav_set_tx=mav_set_tx, tun_set_tx=tun_set_tx, tun_set_all=tun_set_all)
    try:
        assert engine.start() is True

        # Initial wiring: ant_sel_cb fires at registration (current=None),
        # so both services should already have been targeted at wlan 0's
        # socket (the only wlan advertised by the fake children).
        assert mav_set_tx[-1] == "mavlink_tx-sock-0"
        assert tun_set_tx[-1] == "tunnel_tx-sock-0"
        assert tun_set_all[-1] == ["tunnel_tx-sock-0"]

        # Simulate a tx respawn: wfb_tx re-advertises brand-new sockets
        # under a *different* parser object (per children.py's TX RESPAWN
        # NOTE, a respawn always builds a fresh TxLineParser).
        mav_tx_child = engine._children["mavlink_tx"]
        tun_tx_child = engine._children["tunnel_tx"]
        mav_tx_child.tx_parser = FakeTxParser({0: "mavlink_tx-sock-RESPAWNED"})
        tun_tx_child.tx_parser = FakeTxParser({0: "tunnel_tx-sock-RESPAWNED"})

        assert _wait_until(lambda: mav_set_tx[-1] == "mavlink_tx-sock-RESPAWNED", timeout=3.0)
        assert _wait_until(lambda: tun_set_tx[-1] == "tunnel_tx-sock-RESPAWNED", timeout=3.0)
        assert tun_set_all[-1] == ["tunnel_tx-sock-RESPAWNED"]
    finally:
        engine.shutdown()


# -- (f2) same-name tx respawn: the realistic case where a respawned
# wfb_tx reuses the SAME argv, so the fresh TxLineParser re-advertises the
# IDENTICAL socket name as the dead incarnation. The engine must still
# re-invoke set_tx_socket/set_all_tx_sockets on respawn detection (parser
# object identity changed) rather than skipping the re-wire because the
# name value didn't change -- that identity-based re-invocation is what
# lets TunnelService actually reconnect on a same-name respawn (see
# test_wfb_tunnel.py's dedicated reconnect test). -----------------------


def test_ant_sel_retargets_services_after_same_name_tx_respawn():
    mav_set_tx, tun_set_tx, tun_set_all = [], [], []
    engine, rec = make_engine(mav_set_tx=mav_set_tx, tun_set_tx=tun_set_tx, tun_set_all=tun_set_all)
    try:
        assert engine.start() is True

        assert mav_set_tx[-1] == "mavlink_tx-sock-0"
        assert tun_set_tx[-1] == "tunnel_tx-sock-0"
        assert tun_set_all[-1] == ["tunnel_tx-sock-0"]
        mav_calls_before = len(mav_set_tx)
        tun_calls_before = len(tun_set_tx)
        tun_all_calls_before = len(tun_set_all)

        # Simulate a same-argv tx respawn: a fresh TxLineParser (new object
        # identity -> respawn detected) but the IDENTICAL socket name (the
        # realistic case, since a respawn reuses the same argv).
        mav_tx_child = engine._children["mavlink_tx"]
        tun_tx_child = engine._children["tunnel_tx"]
        mav_tx_child.tx_parser = FakeTxParser({0: "mavlink_tx-sock-0"})
        tun_tx_child.tx_parser = FakeTxParser({0: "tunnel_tx-sock-0"})

        assert _wait_until(lambda: len(mav_set_tx) > mav_calls_before, timeout=3.0)
        assert _wait_until(lambda: len(tun_set_tx) > tun_calls_before, timeout=3.0)
        assert _wait_until(lambda: len(tun_set_all) > tun_all_calls_before, timeout=3.0)

        # Values are unchanged (same name) but the calls were genuinely
        # RE-INVOKED, not skipped as a no-op re-target.
        assert mav_set_tx[-1] == "mavlink_tx-sock-0"
        assert tun_set_tx[-1] == "tunnel_tx-sock-0"
        assert tun_set_all[-1] == ["tunnel_tx-sock-0"]
    finally:
        engine.shutdown()


# -- tx respawn mid-handshake: an empty-sockets parser must not be "wired
# up" as if it were the final state; the engine must keep retrying until
# the new parser's handshake actually completes. --------------------------


def test_tx_respawn_mid_handshake_does_not_wire_up_empty_sockets():
    mav_set_tx = []
    engine, rec = make_engine(mav_set_tx=mav_set_tx)
    try:
        assert engine.start() is True
        assert mav_set_tx[-1] == "mavlink_tx-sock-0"

        mav_tx_child = engine._children["mavlink_tx"]
        # Respawn observed mid-handshake: fresh parser object, still empty.
        mav_tx_child.tx_parser = FakeTxParser({})
        time.sleep(0.3)
        # Nothing to point at yet -> must not have re-fired set_tx_socket.
        assert mav_set_tx[-1] == "mavlink_tx-sock-0"

        # Handshake completes: same parser OBJECT gains its sockets in
        # place (this is how a real TxLineParser fills in over time).
        mav_tx_child.tx_parser.unix_sockets[0] = "mavlink_tx-sock-RESPAWNED"

        assert _wait_until(lambda: mav_set_tx[-1] == "mavlink_tx-sock-RESPAWNED", timeout=3.0)
    finally:
        engine.shutdown()


# -- client_factory() resolves the live hub, not a frozen one ----------------


def test_client_factory_resolves_current_hub_after_restart():
    import asyncio

    engine, rec = make_engine()
    try:
        assert engine.start() is True
        # Mimic DynamicLinkController: call client_factory() ONCE and hold
        # the returned class long-term, across engine restarts.
        factory_cls = engine.client_factory()
        hub1 = engine.hub
        assert hub1 is not None

        assert engine.restart() is True
        hub2 = engine.hub
        assert hub2 is not None
        assert hub2 is not hub1

        # A fresh instance constructed from the SAME held class, after the
        # restart, must bind to the NEW hub (not the one live when
        # client_factory() was first called).
        got = []

        async def _drive():
            client = factory_cls("tcp://ignored:0", got.append)
            run_task = asyncio.ensure_future(client.run())
            await asyncio.sleep(0)  # let run() subscribe to the live hub
            hub2.process_new_session(
                "video rx",
                {"fec_type": "swfec", "fec_k": 1, "fec_n": 2, "epoch": 1, "contract_version": 3},
            )
            await asyncio.sleep(0)
            client.stop()
            await run_task

        asyncio.run(_drive())
        assert len(got) >= 1
    finally:
        engine.shutdown()


# -- radio_init failure -> start() False, no children started ---------------


def test_radio_init_failure_stops_before_any_child_starts():
    engine, rec = make_engine(radio_ok=False)
    try:
        assert engine.start() is False
        assert rec["order"] == []
    finally:
        engine.shutdown()


# -- I1: engine restart must not permanently freeze a stats subscriber -----
#
# Reproduces DynamicLinkController._stats_loop's reconnect contract (a
# consumer-owned loop/thread holding one client_factory()-built instance
# long-term) against a real WfbEngine.restart(). Before the fix, the old
# hub is simply dropped by `_teardown()` with no wakeup, so the
# NativeStatsSource subscribed to it blocks forever on `self._stop.wait()`
# -- and since the consumer's reconnect loop only re-invokes the factory
# once `run()` RETURNS, events stop flowing forever even though a brand
# new hub (and new children) is up and running post-restart.


def test_engine_restart_rebinds_stats_subscriber_without_freezing():
    import asyncio
    import threading

    from fpvdgs.dynlink.stats_client import SessionEvent

    engine, rec = make_engine()
    try:
        assert engine.start() is True
        # Mimic DynamicLinkController: call client_factory() ONCE and hold
        # the returned class long-term, across restart().
        factory_cls = engine.client_factory()

        events = []
        state = {}

        def consumer_thread_main():
            async def stats_loop():
                stop_event = asyncio.Event()
                state["stop_event"] = stop_event
                state["loop"] = asyncio.get_running_loop()
                # Mirrors DynamicLinkController._stats_loop exactly: a
                # fresh client per reconnect attempt, retried until
                # stop_event fires, with a short backoff between attempts.
                while not stop_event.is_set():
                    client = factory_cls("tcp://ignored:0", events.append)
                    run_task = asyncio.ensure_future(client.run())
                    stop_task = asyncio.ensure_future(stop_event.wait())
                    try:
                        await asyncio.wait(
                            {run_task, stop_task}, return_when=asyncio.FIRST_COMPLETED
                        )
                    finally:
                        client.stop()
                        for t in (run_task, stop_task):
                            t.cancel()
                    if stop_event.is_set():
                        break
                    try:
                        await asyncio.wait_for(stop_event.wait(), timeout=0.05)
                    except asyncio.TimeoutError:
                        pass

            # asyncio.run() (rather than a bare new_event_loop()) drains and
            # cancels every leftover task -- including the nested ones
            # spawned inside client.run() -- before closing the loop, so a
            # forceful test-side stop doesn't leak "Task was destroyed but
            # it is pending" warnings from tasks that never got a final
            # turn to process their own cancellation.
            asyncio.run(stats_loop())

        thread = threading.Thread(target=consumer_thread_main, daemon=True)
        thread.start()

        def make_session(epoch):
            return {
                "fec_type": "swfec",
                "fec_k": 1,
                "fec_n": 2,
                "epoch": epoch,
                "contract_version": 3,
            }

        def seen(epoch):
            return any(isinstance(ev, SessionEvent) and ev.session.epoch == epoch for ev in events)

        def post_and_check(hub, epoch):
            # Re-posts on every poll (harmless: each phase uses its OWN
            # unique epoch marker, so a late-arriving duplicate from an
            # earlier poll can never satisfy the OTHER phase's check).
            hub.process_new_session("video_rx", make_session(epoch))
            return seen(epoch)

        hub1 = engine.hub
        assert hub1 is not None
        assert _wait_until(lambda: engine.hub is hub1)
        assert _wait_until(lambda: post_and_check(hub1, 101), timeout=5.0)

        events.clear()
        assert engine.restart() is True
        hub2 = engine.hub
        assert hub2 is not None
        assert hub2 is not hub1

        # The SAME long-held client instance must rebind to the new hub and
        # keep delivering events -- within a bounded timeout, not forever.
        assert _wait_until(
            lambda: post_and_check(hub2, 202), timeout=5.0
        ), "stats subscriber never rebound to the post-restart hub"
    finally:
        loop = state.get("loop")
        stop_event = state.get("stop_event")
        if loop is not None and stop_event is not None:
            with contextlib.suppress(RuntimeError):
                loop.call_soon_threadsafe(stop_event.set)
        thread.join(timeout=5.0)
        engine.shutdown()


# -- _teardown clears self.hub, and a held client_factory() class must NOT
# treat that as terminal: `run()` polls quietly for a hub to come back
# instead of returning, and only unblocks on an explicit stop(). (Before
# the I1 fix, run() returned immediately on `hub is None` -- fine for
# DynamicLinkController's own reconnect loop, but ConnectionMonitor has no
# such loop, so a returned run() there permanently kills reachability
# tracking. The fix moves the retry INSIDE run() so both callers are safe
# without either of them changing.) ------------------------------------


def test_teardown_clears_hub_but_run_polls_quietly_until_stop():
    import asyncio

    engine, rec = make_engine()
    try:
        assert engine.start() is True
        assert engine.hub is not None
        # Mimic DynamicLinkController/ConnectionMonitor: call client_factory()
        # ONCE and hold the returned class long-term, across stop/restart.
        factory_cls = engine.client_factory()

        engine.stop()
        assert engine.hub is None

        got = []

        async def _drive():
            client = factory_cls("tcp://ignored:0", got.append)
            run_task = asyncio.ensure_future(client.run())
            # No hub will ever come back in this test -- run() must NOT
            # return on its own (a self-returned run() reads as terminal
            # to ConnectionMonitor, which never re-invokes the factory).
            await asyncio.sleep(1.0)
            assert not run_task.done(), "run() returned on its own with no explicit stop()"
            client.stop()
            await asyncio.wait_for(run_task, timeout=2.0)

        asyncio.run(_drive())
        assert got == []
    finally:
        engine.shutdown()


# -- N1: `_teardown` must clear `self.hub` in the SAME breath as `hub.close()`
# -- not after sequentially stopping statsd/tunnel/mav/children (hundreds of
# ms to ~2s on real hardware). Before the fix, a held `_EngineStatsSource`
# (see `client_factory()`) that re-reads `engine.hub` during that window gets
# the already-closed-but-still-referenced hub back: `hub.subscribe()` on a
# closed hub fires `closed_event` immediately (see `StatsHub.subscribe`), so
# the inner `NativeStatsSource.run()` replays every session in `hub._sessions`
# (the "replay current sessions so late subscribers see FEC params" step)
# and returns right away -- and the outer `_EngineStatsSource.run()` loops
# straight back and does it again, spinning at loop-turn frequency and
# re-delivering the OLD hub's stale session as a duplicate SessionEvent on
# every turn, for as long as teardown takes. This test stalls one teardown
# step on a test-controlled gate and single-steps the loop with bare
# `asyncio.sleep(0)` turns (no wall-clock races) to pin that window.


def test_teardown_drops_hub_atomically_with_close_no_stale_session_spin():
    import asyncio

    from fpvdgs.dynlink.stats_client import SessionEvent

    engine, rec = make_engine()

    async def scenario():
        assert await engine._setup() is True
        hub1 = engine.hub
        assert hub1 is not None
        hub1.process_new_session(
            "video_rx",
            {"fec_type": "swfec", "fec_k": 1, "fec_n": 2, "epoch": 7, "contract_version": 3},
        )

        # Mimic DynamicLinkController/ConnectionMonitor: call client_factory()
        # ONCE and hold a consumer subscribed across the upcoming teardown.
        factory_cls = engine.client_factory()
        events = []
        client = factory_cls("tcp://ignored:0", events.append)
        consumer_task = asyncio.ensure_future(client.run())
        await asyncio.sleep(0)  # let run() subscribe to hub1 + replay the seeded session
        assert any(isinstance(e, SessionEvent) and e.session.epoch == 7 for e in events)

        # Gate `mav_service.stop()` -- a step several awaits AFTER the point
        # where `self.hub` must already be cleared -- so we can hold
        # `_teardown` mid-flight and observe/exercise that window
        # deterministically instead of racing wall-clock thread scheduling.
        gate = asyncio.Event()
        real_stop = engine._mav_service.stop

        async def _gated_stop():
            await gate.wait()
            await real_stop()

        engine._mav_service.stop = _gated_stop

        teardown_task = asyncio.ensure_future(engine._teardown())

        # Give the held consumer's `_EngineStatsSource` loop many turns while
        # teardown sits stuck on the gate. Pre-fix, `self.hub` is still hub1
        # (closed, non-None) throughout this window, so every turn
        # resubscribes and replays the seeded session again.
        for _ in range(50):
            await asyncio.sleep(0)

        assert (
            engine.hub is None
        ), "self.hub must already be cleared while teardown is still in flight"
        dup_epoch7 = sum(1 for e in events if isinstance(e, SessionEvent) and e.session.epoch == 7)
        assert (
            dup_epoch7 <= 1
        ), f"stale closed hub replayed its session {dup_epoch7} times during the teardown window"

        gate.set()
        await teardown_task
        client.stop()
        with contextlib.suppress(asyncio.CancelledError):
            await asyncio.wait_for(consumer_task, timeout=2.0)

    asyncio.run(scenario())
