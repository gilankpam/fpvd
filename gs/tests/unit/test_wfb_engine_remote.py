"""Tests for WfbEngine's Phase 2 remote-card wiring: node sessions, local
forwarders/injectors, degrade-not-fail on an unreachable node, and the
teardown-order guarantee (remote node sessions die before local children).

Everything is faked (child_cls, node_session_cls, radio_init); `graph_builder`/
`graph_builder_remote` are call-recording spies -- the remote spy still
delegates to the REAL `build_graph_remote`/`plan_cluster`/`derive_server_address`
(all pure, and `derive_server_address` never opens a socket here because the
test config always supplies an explicit `link.serverAddress` override), so
this also exercises the real cluster-wiring/argv-rendering path end to end.
No real subprocesses or ssh connections are ever touched.

The all-local path (`not has_remote(cards)`) must stay byte-for-byte Phase 1:
`build_graph` is used, `build_graph_remote` is never even called, and
`state()["nodes"]` stays `{}`.
"""

from __future__ import annotations

import time
from types import SimpleNamespace

import fpvdgs.wfb.engine as engine_module
from fpvdgs.wfb.cards import resolve_cards
from fpvdgs.wfb.cluster import plan_cluster
from fpvdgs.wfb.engine import WfbEngine
from fpvdgs.wfb.graph import build_graph_remote as real_build_graph_remote

LOCAL_WLANS = ["wlan0", "wlan1"]

RX_NAMES = ("video_rx", "mavlink_rx", "tunnel_rx")
TX_NAMES = ("mavlink_tx", "tunnel_tx")


def make_all_local_config():
    return {
        "link": {"cards": [{"iface": "wlan0"}, {"iface": "wlan1"}], "width": 20},
        "wfb": {
            "mavlink": {"peer": "connect://127.0.0.1:14550"},
            "txSelector": {"rssiDeltaDb": 3, "counterRelDelta": 0.1, "counterAbsDelta": 3},
        },
    }


def make_remote_config(cards=None, server_address="10.0.0.1"):
    cards = (
        cards
        if cards is not None
        else [
            {"iface": "wlan0"},
            {"iface": "wlan1", "txPowerDbm": "off"},  # rx-only local card
            {"host": "node1", "iface": "wlan0"},
        ]
    )
    return {
        "link": {"cards": cards, "width": 20, "serverAddress": server_address},
        "wfb": {
            "mavlink": {"peer": "connect://127.0.0.1:14550"},
            "txSelector": {"rssiDeltaDb": 3, "counterRelDelta": 0.1, "counterAbsDelta": 3},
        },
    }


def make_spec(name, kind):
    return SimpleNamespace(name=name, kind=kind, argv=[f"/bin/{name}"], parser=kind, unix_path=None)


def make_graph_builder():
    """All-local fake graph builder, Phase 1 style (no remote fields)."""

    def _build(effective, wlans, *, rand_suffix):
        specs = {name: make_spec(name, "rx") for name in RX_NAMES}
        specs.update({name: make_spec(name, "tx") for name in TX_NAMES})
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

    return _FakeChild


def make_node_session_cls(order, fail_nodes=frozenset()):
    """Fake NodeSession: records ("node_start"|"node_stop", node) into the
    SAME shared `order` list used by the fake children, so tests can assert
    relative ordering directly by index. `start()` RAISES for a node in
    `fail_nodes` (simulating an unreachable/failed ssh spawn) -- the engine
    must swallow this (degrade, never fail engine start)."""

    class _FakeNodeSession:
        def __init__(self, node, script, *, argv_builder=None, on_state=None, **kw):
            self.node = node
            self.script = script
            self.argv_builder = argv_builder
            self._on_state = on_state
            self.alive = False
            self._restarts = 0

        async def start(self):
            order.append(("node_start", self.node))
            if self.node in fail_nodes:
                raise RuntimeError(f"unreachable node: {self.node}")
            self.alive = True
            return True

        async def stop(self):
            order.append(("node_stop", self.node))
            self.alive = False

        def state(self):
            return {"alive": self.alive, "restarts": self._restarts}

    return _FakeNodeSession


def make_service_cls(started, stopped, set_tx_calls, set_all_calls=None):
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
    fail_nodes=frozenset(),
    config=None,
    radio_calls=None,
    radio_ok=True,
    graph_builder=None,
    graph_builder_remote=None,
    node_session_cls=None,
):
    order = order if order is not None else []
    radio_calls = radio_calls if radio_calls is not None else []
    cfg = config if config is not None else make_all_local_config()

    def radio_init(wlans, link):
        radio_calls.append((tuple(wlans), dict(link)))
        return radio_ok

    engine = WfbEngine(
        config_provider=lambda: cfg,
        wlans_resolver=lambda c: list(LOCAL_WLANS),
        graph_builder=graph_builder if graph_builder is not None else make_graph_builder(),
        graph_builder_remote=graph_builder_remote
        if graph_builder_remote is not None
        else _spy_remote_graph_builder([]),
        child_cls=make_child_cls(order, fail_names),
        node_session_cls=node_session_cls
        if node_session_cls is not None
        else make_node_session_cls(order, fail_nodes),
        radio_init=radio_init,
        stats_port=0,
        mav_service_cls=make_service_cls([], [], []),
        tunnel_service_cls=make_service_cls([], [], [], []),
    )
    return engine, {"order": order, "radio_calls": radio_calls}


def _spy_remote_graph_builder(calls):
    def _build(effective, cards, plan, server_address, *, rand_suffix):
        calls.append((cards, plan, server_address))
        return real_build_graph_remote(
            effective, cards, plan, server_address, rand_suffix=rand_suffix
        )

    return _build


def _wait_until(predicate, timeout=5.0, interval=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


# -- (a) all-local: build_graph used, build_graph_remote NEVER called,
# zero NodeSessions, state()["nodes"] == {} -----------------------------


def test_all_local_stays_phase1_no_remote_wiring():
    order = []
    remote_calls = []

    def graph_builder_remote_spy(*a, **kw):
        remote_calls.append((a, kw))
        raise AssertionError("build_graph_remote must not be called in the all-local path")

    def node_session_ctor_spy(*a, **kw):
        raise AssertionError("no NodeSession should be constructed in the all-local path")

    engine, rec = make_engine(
        order=order,
        config=make_all_local_config(),
        graph_builder_remote=graph_builder_remote_spy,
        node_session_cls=node_session_ctor_spy,
    )
    try:
        assert engine.start() is True
        assert remote_calls == []

        starts = [name for kind, name in rec["order"] if kind == "start"]
        assert set(starts) == set(RX_NAMES) | set(TX_NAMES)
        assert not any(kind == "node_start" for kind, _ in rec["order"])

        assert engine.state()["nodes"] == {}
    finally:
        engine.shutdown()


# -- (b) remote: radio_init gets LOCAL ifaces only; aggregator/distributor +
# forwarder/injector specs all present; one NodeSession per remote node ------


def test_remote_radio_init_local_only_and_all_leg_kinds_present():
    order = []
    radio_calls = []
    remote_calls = []
    cfg = make_remote_config()

    engine, rec = make_engine(
        order=order,
        radio_calls=radio_calls,
        config=cfg,
        graph_builder_remote=_spy_remote_graph_builder(remote_calls),
    )
    try:
        assert engine.start() is True

        assert len(remote_calls) == 1
        # radio_init must see LOCAL ifaces only (wlan0, wlan1) -- never
        # node1's remote wlan0 (that init happens inside the node script).
        assert len(radio_calls) == 1
        assert radio_calls[0][0] == ("wlan0", "wlan1")

        starts = [name for kind, name in rec["order"] if kind == "start"]
        assert set(RX_NAMES) | set(TX_NAMES) <= set(starts)
        assert "video fwd" in starts
        assert "mavlink fwd" in starts and "mavlink inj" in starts
        assert "tunnel fwd" in starts and "tunnel inj" in starts
        assert "video inj" not in starts  # video is rx-only, no tx leg

        node_starts = [node for kind, node in rec["order"] if kind == "node_start"]
        assert node_starts == ["node1"]

        st = engine.state()
        assert st["nodes"] == {"node1": {"alive": True, "restarts": 0}}
    finally:
        engine.shutdown()


# -- (c) a node that fails to start degrades, never fails engine start ------


def test_node_session_start_failure_degrades_not_fails():
    order = []
    cfg = make_remote_config()

    engine, rec = make_engine(
        order=order,
        config=cfg,
        fail_nodes={"node1"},
    )
    try:
        assert engine.start() is True
        st = engine.state()
        assert st["nodes"]["node1"]["alive"] is False
    finally:
        engine.shutdown()


# -- (d) teardown stops node sessions BEFORE local children ------------------


def test_teardown_stops_node_sessions_before_children():
    order = []
    engine, rec = make_engine(order=order, config=make_remote_config())
    try:
        assert engine.start() is True
    finally:
        engine.shutdown()

    node_stop_idx = min(i for i, (kind, _) in enumerate(order) if kind == "node_stop")
    child_stop_idx = min(i for i, (kind, _) in enumerate(order) if kind == "stop")
    assert node_stop_idx < child_stop_idx


# -- (e) rx_only wlan ids computed by plan_cluster reach TxSelector ----------


def test_rx_only_wlan_ids_reach_tx_selector(monkeypatch):
    cfg = make_remote_config()
    cards = resolve_cards(cfg)
    expected_plan = plan_cluster(cards)
    assert len(expected_plan.rx_only_wlan_ids) == 1  # sanity: the "off" wlan1 card

    captured = {}

    class FakeTxSelector:
        def __init__(self, txsel_cfg, rx_only_wlan_ids=frozenset()):
            captured["rx_only_wlan_ids"] = rx_only_wlan_ids
            self.current = None

        def select(self, stats_agg):
            return None

    monkeypatch.setattr(engine_module, "TxSelector", FakeTxSelector)

    engine, rec = make_engine(config=cfg)
    try:
        assert engine.start() is True
        assert captured["rx_only_wlan_ids"] == expected_plan.rx_only_wlan_ids
    finally:
        engine.shutdown()


# -- (f) remote + probe: plan_cluster(with_probe)/build_graph_remote pairing -
# starts AND stops the probe leg (mirrors test_engine_teardown_stops_probe_leg
# in test_wfb_engine.py, but for the remote path, which lays out the probe
# leg via plan_cluster(with_probe=...) + build_graph_remote instead of the
# all-local graph builder) --------------------------------------------------


def test_remote_teardown_stops_probe_leg():
    order = []
    cfg = make_remote_config()
    cfg["dynamicLink"] = {"enabled": True, "probe": {"enabled": True}}

    # Default graph_builder_remote (a spy delegating to the real
    # `build_graph_remote`) so this exercises the real
    # `plan_cluster(with_probe=...)` <-> `build_graph_remote` pairing, not a
    # fake graph's hand-rolled `probe_rx` field.
    engine, rec = make_engine(order=order, config=cfg)
    try:
        assert engine.start() is True
        starts = [name for kind, name in rec["order"] if kind == "start"]
        assert "probe_rx" in starts
    finally:
        engine.shutdown()

    stops = [name for kind, name in rec["order"] if kind == "stop"]
    assert "probe_rx" in stops
