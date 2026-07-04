"""WfbEngine — loop-in-a-thread orchestrator tying together every native wfb
data-plane module (`graph`/`children`/`aggregator`/`txsel`/`mavproxy`/
`tunnel`/`statsd`) into one `RunnerSupervisor`-compatible object.

Lifecycle pattern is copied from `dynlink.controller.DynamicLinkController`:
a daemon thread owns a fresh `asyncio` event loop per `start()`, a
`threading.Event` (`_started`) unblocks the thread-safe `start()` call once
setup has finished (success or failure), and `stop()` requests shutdown via
`loop.call_soon_threadsafe`. `config_provider()` is re-read on every `start()`
— including a `restart()` — so a config change takes effect by restarting the
engine; there is no in-place reconfiguration.

TX socket wiring — the "TX RESPAWN NOTE" in `children.py` warns that a
respawned `wfb_tx` child gets a brand-new `TxLineParser` (a different Python
object) whose `unix_sockets` starts empty until the new handshake completes.
This module never caches a `unix_sockets` dict: every retarget re-reads
`child.tx_parser.unix_sockets` live, and the 100 ms ticker additionally
watches `tx_parser` *object identity* for the two tx children so a respawn
(detected by that identity changing) re-fires the current TX selection and
re-broadcasts `set_all_tx_sockets`, even though no antenna switch actually
happened.

Phase 2 remote cards — whenever `has_remote(cards)` (any `link.cards` entry
names a `host`), `_setup` takes the remote branch: `radio_init` only ever
sees LOCAL ifaces (a remote card's radio is tuned by its own node script,
not by this process); `build_graph_remote` replaces `build_graph`, adding
per-service local forwarder/injector children (`parser=None` — see
`children.py`'s `spec.parser is None` branch) alongside the usual
aggregator/distributor legs; and one `NodeSession` per remote node is
started, over `functools.partial(_ssh_argv_for_card, card)` bound to that
node's representative `Card`. A node session is *degrade, never fail*: it
never blocks or fails engine `start()`, it just reports itself down via
`state()["nodes"][node]["alive"]` until it (re)connects. Node sessions are
torn down FIRST in `_teardown`, ahead of every local service, so a remote
node's own children die before the GS-side aggregators/distributors they
were feeding disappear. The all-local path is untouched by any of this —
same `build_graph`, same leg list, `state()["nodes"] == {}`.
"""

from __future__ import annotations

import asyncio
import contextlib
import functools
import logging
import threading
import time
import uuid

from .. import radio
from .aggregator import StatsHub
from .cards import Card, has_remote, local_ifaces, parse_cards, remote_cards
from .children import WfbChild
from .cluster import NodeSession, derive_server_address, plan_cluster, ssh_argv
from .graph import build_graph, build_graph_remote
from .mavproxy import MavlinkService
from .statsd import StatsServer
from .tunnel import TunnelService
from .txsel import TxSelector, TxSelectorConfig

log = logging.getLogger("fpvdgs.wfb")

AGGREGATE_INTERVAL_S = 0.1
START_JOIN_TIMEOUT_S = 30.0
STOP_JOIN_TIMEOUT_S = 30.0
_HUB_POLL_INTERVAL_S = 0.5  # client_factory()'s retry cadence while engine.hub is None

RX_LEG_ORDER = ("video_rx", "mavlink_rx", "tunnel_rx")
TX_LEG_ORDER = ("mavlink_tx", "tunnel_tx")


def _rand_suffix() -> str:
    return uuid.uuid4().hex[:8]


def _ssh_argv_for_card(card: Card, _node) -> list[str]:
    """`NodeSession.argv_builder` shape is `(node) -> argv`, but `node` here
    is the plain host string (see `_start_node_sessions`), not a `Card` --
    `ssh_argv` needs the full `Card` (user/port/key). `functools.partial`
    binds `card` as the first positional arg; the `node` NodeSession passes
    at call time lands in `_node` and is intentionally ignored."""
    return ssh_argv(card)


class WfbEngine:
    def __init__(
        self,
        config_provider,
        wlans_resolver,
        *,
        graph_builder=build_graph,
        graph_builder_remote=build_graph_remote,
        child_cls=WfbChild,
        node_session_cls=NodeSession,
        tun_factory=None,
        radio_init=radio.init_cards,
        stats_port=8103,
        mav_service_cls=MavlinkService,
        tunnel_service_cls=TunnelService,
    ):
        self._config_provider = config_provider
        self._wlans_resolver = wlans_resolver
        self._graph_builder = graph_builder
        self._graph_builder_remote = graph_builder_remote
        self._child_cls = child_cls
        self._node_session_cls = node_session_cls
        self._tun_factory = tun_factory
        self._radio_init = radio_init
        self._stats_port = stats_port
        self._mav_service_cls = mav_service_cls
        self._tunnel_service_cls = tunnel_service_cls

        self.hub: StatsHub | None = None  # current incarnation; read by statsd/CLI/tests

        self._lifecycle = threading.RLock()
        self._lock = threading.RLock()
        self._thread: threading.Thread | None = None
        self._loop: asyncio.AbstractEventLoop | None = None
        self._stop_event: asyncio.Event | None = None
        self._started = threading.Event()
        self._start_ok = False

        self._children: dict[str, object] = {}
        self._extra_rx_names: list[str] = []  # remote-mode local forwarders
        self._extra_tx_names: list[str] = []  # remote-mode local injectors
        self._mav_service = None
        self._tunnel_service = None
        self._stats_server: StatsServer | None = None
        self._tick_task: asyncio.Task | None = None
        self._tx_selector: TxSelector | None = None
        self._tx_parser_ids: dict[str, object] = {}

        # Remote-mode (Phase 2) node sessions, keyed by node (host string).
        # Always empty in all-local mode -- state()["nodes"] stays {}.
        self._node_sessions: dict[str, object] = {}
        self._nodes_state: dict[str, dict] = {}

        self._restarts = 0
        self._engine_fault = False

    # ---- RunnerSupervisor-compatible surface (thread-safe) ---------------
    def start(self) -> bool:
        with self._lifecycle:
            with self._lock:
                if self._thread and self._thread.is_alive():
                    return self._start_ok
                self._started.clear()
                self._start_ok = False
                self._thread = threading.Thread(
                    target=self._thread_main, name="wfb-engine", daemon=True
                )
                self._thread.start()
            self._started.wait(timeout=START_JOIN_TIMEOUT_S)
            with self._lock:
                return self._start_ok

    def stop(self) -> None:
        with self._lifecycle:
            with self._lock:
                loop, stop_event, thread = self._loop, self._stop_event, self._thread
            if loop is not None and stop_event is not None:
                try:
                    loop.call_soon_threadsafe(stop_event.set)
                except RuntimeError:
                    pass  # loop already closed/closing — teardown in progress
            if thread is not None and thread is not threading.current_thread():
                thread.join(timeout=STOP_JOIN_TIMEOUT_S)
            with self._lock:
                self._thread = None

    def restart(self) -> bool:
        with self._lifecycle:
            self.stop()
            with self._lock:
                self._restarts += 1
            return self.start()

    def shutdown(self) -> None:
        self.stop()

    def set_argv(self, argv) -> None:
        log.debug("wfb engine: set_argv(%r) ignored — config re-read on next start", argv)

    def set_env(self, env) -> None:
        log.debug("wfb engine: set_env(%r) ignored — config re-read on next start", env)

    def state(self) -> dict:
        with self._lock:
            children = dict(self._children)
            restarts = self._restarts
            engine_fault = self._engine_fault
            nodes = dict(self._nodes_state)

        if not children:
            return {
                "running": False,
                "pid": None,
                "restarts": restarts,
                "autoRestarts": 0,
                "lastExit": None,
                "fault": engine_fault,
                "nodes": nodes,
            }

        child_states = {name: c.state() for name, c in children.items()}
        running = all(s["running"] for s in child_states.values())
        video = child_states.get("video_rx")
        pid = video["pid"] if video is not None else None
        auto_restarts = sum(s["autoRestarts"] for s in child_states.values())
        last_exit = None
        for name in RX_LEG_ORDER + TX_LEG_ORDER:
            s = child_states.get(name)
            if s is not None and s["lastExit"] is not None:
                last_exit = s["lastExit"]
                break
        fault = engine_fault or any(s["fault"] for s in child_states.values())

        return {
            "running": running,
            "pid": pid,
            "restarts": restarts,
            "autoRestarts": auto_restarts,
            "lastExit": last_exit,
            "fault": fault,
            "nodes": nodes,
        }

    def client_factory(self):
        """Returns a StatsClient-compatible class, stable across restarts.

        `DynamicLinkController`/`ConnectionMonitor` call this ONCE and keep
        the returned class long-term (`stats_client_factory=...`), then
        construct a fresh instance per reconnect attempt. Each instance
        resolves `self.hub` at `run()` time rather than baking in a
        particular hub, so a reconnect attempt made *after* an engine
        restart binds to the new hub.

        `run()` does NOT return just because the CURRENT hub went away
        (`StatsHub.close()`, fired by `_teardown` before a `restart()`/
        `stop()` drops `engine.hub`) — it loops internally, polling for the
        next hub and resubscribing to it. This matters because the two
        callers' reconnect contracts differ: `DynamicLinkController.
        _stats_loop` reconnects indefinitely across a returned/raised
        `run()`, but `ConnectionMonitor._run()` has no such outer loop — a
        returned `run()` there is terminal for the whole monitor thread.
        Looping in here (rather than relying on the caller to re-invoke the
        factory) is what makes rebinding actually happen for BOTH, without
        touching either caller.
        """
        engine = self

        class _EngineStatsSource:
            def __init__(self, endpoint, on_event, **kw):
                self._endpoint = endpoint
                self._on_event = on_event
                self._kw = kw
                self._inner = None
                self._stop: asyncio.Event | None = None

            async def run(self):
                self._stop = asyncio.Event()
                while not self._stop.is_set():
                    with engine._lock:
                        hub = engine.hub
                    if hub is None:
                        # Engine not up yet, or between this restart's
                        # teardown and its next setup -- wait briefly and
                        # retry rather than returning (a returned run() is
                        # terminal for ConnectionMonitor; see docstring).
                        with contextlib.suppress(asyncio.TimeoutError):
                            await asyncio.wait_for(self._stop.wait(), timeout=_HUB_POLL_INTERVAL_S)
                        continue
                    inner_cls = hub.client_factory()
                    self._inner = inner_cls(self._endpoint, self._on_event, **self._kw)
                    try:
                        # Awaited directly (not wrapped in its own Task): a
                        # `stop()` call reaches this via `self._inner.stop()`
                        # below, and a hub `close()` reaches it via the
                        # inner `NativeStatsSource`'s own closed_event race
                        # -- either way `run()` returns on its own, so there
                        # is nothing left to separately race here. Direct
                        # awaiting also means an external cancellation of
                        # THIS task propagates straight into the inner
                        # coroutine's own suspension point instead of
                        # leaving an orphaned sibling Task behind.
                        await self._inner.run()
                    finally:
                        self._inner = None
                    # `run()` returning means the inner NativeStatsSource's
                    # hub closed (restart/stop) or stop() fired -- loop back
                    # to recheck `self._stop` and re-resolve engine.hub.

            def stop(self):
                if self._stop is not None:
                    self._stop.set()
                if self._inner is not None:
                    self._inner.stop()

        return _EngineStatsSource

    # ---- thread + loop lifecycle ------------------------------------------
    def _thread_main(self) -> None:
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        with self._lock:
            self._loop = loop
        try:
            loop.run_until_complete(self._run())
        except Exception:
            log.exception("wfb engine: loop crashed")
        finally:
            try:
                loop.close()
            finally:
                with self._lock:
                    self._loop = None
                self._started.set()  # unblock start() even on an early crash

    async def _run(self) -> None:
        self._stop_event = asyncio.Event()
        ok = await self._setup()
        with self._lock:
            self._start_ok = ok
        self._started.set()
        if not ok:
            return
        try:
            await self._stop_event.wait()
        finally:
            await self._teardown()

    # ---- setup --------------------------------------------------------------
    async def _setup(self) -> bool:
        loop = asyncio.get_running_loop()
        with self._lock:
            self._engine_fault = False

        try:
            cfg = self._config_provider()
            link = cfg.get("link", {}) or {}
            # `parse_cards` is pure (no I/O): a concrete `link.cards`/`wlans`
            # list is returned as-is, and only the "auto" placeholder would
            # ever need hardware nic-detection -- but "auto" can only ever
            # resolve to LOCAL cards (see cards.py's module docstring), so
            # treating it as an empty (definitely-not-remote) list here is
            # exact, not an approximation, and keeps this branch-decision
            # free of any subprocess/hardware dependency.
            parsed_cards = parse_cards(link)
            cards: list[Card] = [] if parsed_cards == "auto" else parsed_cards
            remote = has_remote(cards)
        except Exception:
            log.exception("wfb engine: failed to resolve config/cards")
            return False

        if not remote:
            # ---- EXACTLY Phase 1: local ifaces come from wlans_resolver ----
            try:
                wlans = self._wlans_resolver(cfg)
            except Exception:
                log.exception("wfb engine: failed to resolve config/wlans")
                return False
        else:
            # Remote mode: `cards` is already fully resolved (concrete, by
            # definition of not being "auto"), so local ifaces come straight
            # from it -- no need to consult wlans_resolver at all. Remote
            # card init lives in the node's own bootstrap script, not here.
            wlans = local_ifaces(cards)

        try:
            radio_ok = await loop.run_in_executor(None, self._radio_init, wlans, link)
        except Exception:
            log.exception("wfb engine: radio_init raised")
            return False
        if not radio_ok:
            log.error("wfb engine: radio_init failed for wlans=%s", wlans)
            return False

        plan = None
        server_address = None
        if remote:
            try:
                plan = plan_cluster(cards)
                first_remote = remote_cards(cards)[0]
                server_address = derive_server_address(first_remote.host, link.get("serverAddress"))
            except Exception:
                log.exception("wfb engine: cluster planning failed")
                return False

        try:
            if remote:
                graph = self._graph_builder_remote(
                    cfg, cards, plan, server_address, rand_suffix=_rand_suffix
                )
            else:
                graph = self._graph_builder(cfg, wlans, rand_suffix=_rand_suffix)
        except Exception:
            log.exception("wfb engine: graph_builder raised")
            return False

        tx_selector = TxSelector(
            _tx_selector_config(cfg),
            rx_only_wlan_ids=plan.rx_only_wlan_ids if plan is not None else frozenset(),
        )
        hub = StatsHub(tx_selector, time_fn=time.time)

        specs_by_name = {
            "video_rx": graph.video_rx,
            "mavlink_rx": graph.mavlink_rx,
            "tunnel_rx": graph.tunnel_rx,
            "mavlink_tx": graph.mavlink_tx,
            "tunnel_tx": graph.tunnel_tx,
        }

        # Base legs (aggregator rx / distributor tx in remote mode, plain rx/tx
        # in all-local mode -- same names either way) plus, in remote mode
        # only, the local forwarders (rx, parser=None) and injectors (tx,
        # parser=None): all-local mode never appends anything here, so this
        # loop's *behavior* for that path is unchanged from Phase 1.
        leg_specs: list[tuple[str, object]] = [
            (name, specs_by_name[name]) for name in RX_LEG_ORDER + TX_LEG_ORDER
        ]
        extra_rx_names: list[str] = []
        extra_tx_names: list[str] = []
        if remote:
            for spec in graph.local_forwarders:
                leg_specs.append((spec.name, spec))
                extra_rx_names.append(spec.name)
            for spec in graph.local_injectors:
                leg_specs.append((spec.name, spec))
                extra_tx_names.append(spec.name)

        children: dict[str, object] = {}
        started_order: list[str] = []
        ok_all = True
        for name, spec in leg_specs:
            try:
                child = self._child_cls(spec, hub, on_fault=self._on_child_fault)
                ok = await child.start()
            except Exception:
                log.exception("wfb engine: child %s failed to construct/start", name)
                child, ok = None, False
            if child is not None:
                children[name] = child
                started_order.append(name)
            if not ok:
                ok_all = False
                break

        if not ok_all:
            await self._stop_children(children, reversed(started_order))
            return False

        mav_child = children["mavlink_tx"]
        tun_child = children["tunnel_tx"]

        mav_service = self._mav_service_cls(graph.mav_peer)
        tunnel_service = self._tunnel_service_cls(graph.tun_cfg)
        stats_server = StatsServer(hub, self._settings_fn, port=self._stats_port)

        try:
            await mav_service.start(loop, graph.mav_rx_sock, hub)
            tun_kwargs = {} if self._tun_factory is None else {"tun_factory": self._tun_factory}
            await tunnel_service.start(loop, graph.tun_rx_sock, **tun_kwargs)

            if tun_child.tx_parser is not None:
                tunnel_service.set_all_tx_sockets(list(tun_child.tx_parser.unix_sockets.values()))

            with self._lock:
                self._children = children
                self._mav_service = mav_service
                self._tunnel_service = tunnel_service
                self._tx_selector = tx_selector
                self.hub = hub
            self._tx_parser_ids = {
                "mavlink_tx": mav_child.tx_parser,
                "tunnel_tx": tun_child.tx_parser,
            }
            self._extra_rx_names = extra_rx_names
            self._extra_tx_names = extra_tx_names

            hub.add_ant_sel_cb(self._apply_tx_selection)
            # NOTE: `mav_service.start()` above already registers its own
            # `rssi_cb` with the hub (when `cfg.inject_rssi`, the default) —
            # do not also register it here, or RADIO_STATUS frames double up.

            await stats_server.start(loop)
            with self._lock:
                self._stats_server = stats_server
        except Exception:
            log.exception("wfb engine: service wiring failed")
            # Close BEFORE dropping the reference: any stats subscriber that
            # already resolved `self.hub` (or races `subscribe()` against
            # this close) must be woken rather than left hanging forever.
            hub.close()
            with contextlib.suppress(Exception):
                await stats_server.stop()
            with contextlib.suppress(Exception):
                await tunnel_service.stop()
            with contextlib.suppress(Exception):
                await mav_service.stop()
            await self._stop_children(children, reversed(started_order))
            with self._lock:
                self._children = {}
                self._mav_service = None
                self._tunnel_service = None
                self._tx_selector = None
                self.hub = None
            return False

        # ---- remote node sessions (started AFTER local children; a node
        # that fails to come up degrades, never fails engine start) --------
        if remote:
            await self._start_node_sessions(graph, plan)

        self._tick_task = asyncio.ensure_future(self._ticker())
        return True

    async def _start_node_sessions(self, graph, plan) -> None:
        for node in sorted(graph.node_scripts):
            script = graph.node_scripts[node]
            rep_card = plan.nodes[node][0]
            session = self._node_session_cls(
                node,
                script,
                argv_builder=functools.partial(_ssh_argv_for_card, rep_card),
                on_state=self._make_node_state_cb(node),
            )
            self._node_sessions[node] = session
            try:
                ok = await session.start()
            except Exception:
                log.exception("wfb engine: node %s session failed to start (degrading)", node)
                ok = False
            with self._lock:
                self._nodes_state[node] = {"alive": bool(ok), "restarts": 0}
            if not ok:
                log.warning("wfb engine: node %s failed to start", node)

    def _make_node_state_cb(self, node):
        def _on_node_state(_node, up: bool) -> None:
            if up:
                log.info("wfb engine: node %s up", node)
            else:
                log.warning("wfb engine: node %s down", node)
            session = self._node_sessions.get(node)
            restarts = 0
            if session is not None:
                with contextlib.suppress(Exception):
                    restarts = session.state()["restarts"]
            with self._lock:
                self._nodes_state[node] = {"alive": up, "restarts": restarts}

        return _on_node_state

    @staticmethod
    async def _stop_children(children: dict, order) -> None:
        for name in order:
            child = children.get(name)
            if child is None:
                continue
            with contextlib.suppress(Exception):
                await child.stop()

    def _settings_fn(self) -> dict:
        try:
            cfg = self._config_provider()
        except Exception:
            return {}
        return dict(cfg.get("wfb", {}) or {})

    # ---- teardown -----------------------------------------------------------
    async def _teardown(self) -> None:
        # Close AND drop the reference together, before anything else: a
        # closed-but-still-`self.hub`-visible window lets `_EngineStatsSource.
        # run()` (see `client_factory`'s docstring) re-read `engine.hub`, get
        # the already-closed hub, resubscribe (which fires its closed_event
        # immediately and replays the hub's `_sessions` as duplicate
        # SessionEvents), and loop right back -- spinning at loop-turn
        # frequency and re-delivering stale session state for the whole
        # teardown. Dropping the reference in the same breath as `close()`
        # closes that window: the next `run()` iteration sees `hub is None`
        # and falls into the poll-and-retry wait instead of re-subscribing to
        # a dead hub. Nothing else in this function reads `self.hub` --
        # `_ticker` (the only other reader, `self.hub.aggregate_window()`) is
        # cancelled below and is always suspended at its own `asyncio.sleep`
        # when `_teardown` runs (single-threaded event loop), so it cannot
        # observe this hub again before that cancellation lands.
        # Node sessions FIRST, ahead of everything else: a remote node's own
        # script tears down its wfb_rx/wfb_tx children the moment its ssh
        # session dies (trap on stdin close -- see cluster.py's NodeSession
        # docstring), so stopping the session before the local
        # aggregators/distributors vanish avoids a brief window where a
        # still-alive remote injector/forwarder is talking to a GS side that
        # has already torn down.
        node_sessions = self._node_sessions
        self._node_sessions = {}
        for session in node_sessions.values():
            with contextlib.suppress(Exception):
                await session.stop()
        with self._lock:
            self._nodes_state = {}

        with self._lock:
            hub = self.hub
            self.hub = None
        if hub is not None:
            hub.close()

        if self._tick_task is not None:
            self._tick_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await self._tick_task
            self._tick_task = None

        if self._stats_server is not None:
            with contextlib.suppress(Exception):
                await self._stats_server.stop()
            self._stats_server = None

        if self._tunnel_service is not None:
            with contextlib.suppress(Exception):
                await self._tunnel_service.stop()
            self._tunnel_service = None

        if self._mav_service is not None:
            with contextlib.suppress(Exception):
                await self._mav_service.stop()
            self._mav_service = None

        children = self._children
        await self._stop_children(children, TX_LEG_ORDER + tuple(self._extra_tx_names))
        await self._stop_children(children, RX_LEG_ORDER + tuple(self._extra_rx_names))

        with self._lock:
            self._children = {}
            self._tx_selector = None
        self._tx_parser_ids = {}
        self._extra_rx_names = []
        self._extra_tx_names = []

    # ---- TX antenna wiring (engine loop thread only) -------------------------
    def _on_child_fault(self, child) -> None:
        name = getattr(getattr(child, "spec", None), "name", "?")
        log.error("wfb engine: child %s faulted (crash-loop budget exceeded)", name)
        with self._lock:
            self._engine_fault = True

    def _apply_tx_selection(self, wlan_id) -> None:
        if wlan_id is not None:
            log.info("wfb: tx card -> wlan %d", wlan_id)
        mav_child = self._children.get("mavlink_tx")
        self._mav_service.set_tx_socket(self._tx_socket_for(mav_child, wlan_id))
        tun_child = self._children.get("tunnel_tx")
        self._tunnel_service.set_tx_socket(self._tx_socket_for(tun_child, wlan_id))

    @staticmethod
    def _tx_socket_for(child, wlan_id) -> str | None:
        if child is None or child.tx_parser is None:
            return None
        sockets = child.tx_parser.unix_sockets
        if not sockets:
            return None
        if wlan_id is not None and wlan_id in sockets:
            return sockets[wlan_id]
        return next(iter(sockets.values()))  # None (no selection yet) -> first available

    def _check_tx_respawns(self) -> None:
        """Detect a tx child respawn by `tx_parser` object identity (a fresh
        parser is built on every `wfb_tx` (re)spawn — see children.py's TX
        RESPAWN NOTE) and re-wire both uplink services from scratch so
        neither holds a stale socket name from the dead incarnation.

        A freshly-built `TxLineParser` starts with an empty `unix_sockets`
        dict that only fills in once the new handshake completes (a few ms
        after respawn, but not necessarily within this tick). Identity is
        only "committed" to `_tx_parser_ids` once `unix_sockets` is
        non-empty, so a respawn observed mid-handshake is retried on the
        next tick instead of being wired up once with nothing to point at."""
        changed = False
        for name in TX_LEG_ORDER:
            child = self._children.get(name)
            if child is None:
                continue
            parser = child.tx_parser
            if parser is None or not parser.unix_sockets:
                continue  # no parser yet, or handshake still in flight
            if self._tx_parser_ids.get(name) is parser:
                continue  # already wired up for this parser incarnation
            self._tx_parser_ids[name] = parser
            changed = True
            log.info("wfb: %s tx_parser respawned — re-targeting tx sockets", name)
        if not changed:
            return
        self._apply_tx_selection(self._tx_selector.current if self._tx_selector else None)
        tun_child = self._children.get("tunnel_tx")
        if tun_child is not None and tun_child.tx_parser is not None:
            self._tunnel_service.set_all_tx_sockets(list(tun_child.tx_parser.unix_sockets.values()))

    async def _ticker(self) -> None:
        try:
            while True:
                await asyncio.sleep(AGGREGATE_INTERVAL_S)
                self._check_tx_respawns()
                self.hub.aggregate_window()
        except asyncio.CancelledError:
            raise


def _tx_selector_config(cfg: dict) -> TxSelectorConfig:
    txsel = ((cfg.get("wfb", {}) or {}).get("txSelector", {})) or {}
    return TxSelectorConfig(
        rssi_delta_db=txsel.get("rssiDeltaDb", 3),
        counter_rel_delta=txsel.get("counterRelDelta", 0.1),
        counter_abs_delta=txsel.get("counterAbsDelta", 3),
    )
