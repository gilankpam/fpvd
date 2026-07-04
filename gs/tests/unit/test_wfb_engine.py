"""Tests for WfbEngine — the loop-in-a-thread orchestrator tying every wfb/
module together into one RunnerSupervisor-compatible object.

Everything is faked: `child_cls` is a stub recording start/stop order (and
optionally failing by name), `radio_init` is a recorder, `graph_builder`
returns a tiny fake graph, and `mav_service_cls`/`tunnel_service_cls` are
stubs recording their wiring calls. No real subprocesses, sockets, or tun
devices are touched.
"""

from __future__ import annotations

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
        graph_builder=make_graph_builder(argv_tag),
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
