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
"""

from __future__ import annotations

import asyncio
import contextlib
import logging
import threading
import time
import uuid

from .. import radio
from .aggregator import StatsHub
from .children import WfbChild
from .graph import build_graph
from .mavproxy import MavlinkService
from .statsd import StatsServer
from .tunnel import TunnelService
from .txsel import TxSelector, TxSelectorConfig

log = logging.getLogger("fpvdgs.wfb")

AGGREGATE_INTERVAL_S = 0.1
START_JOIN_TIMEOUT_S = 30.0
STOP_JOIN_TIMEOUT_S = 30.0

RX_LEG_ORDER = ("video_rx", "mavlink_rx", "tunnel_rx")
TX_LEG_ORDER = ("mavlink_tx", "tunnel_tx")


def _rand_suffix() -> str:
    return uuid.uuid4().hex[:8]


class WfbEngine:
    def __init__(
        self,
        config_provider,
        wlans_resolver,
        *,
        graph_builder=build_graph,
        child_cls=WfbChild,
        tun_factory=None,
        radio_init=radio.init_cards,
        stats_port=8103,
        mav_service_cls=MavlinkService,
        tunnel_service_cls=TunnelService,
    ):
        self._config_provider = config_provider
        self._wlans_resolver = wlans_resolver
        self._graph_builder = graph_builder
        self._child_cls = child_cls
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
        self._mav_service = None
        self._tunnel_service = None
        self._stats_server: StatsServer | None = None
        self._tick_task: asyncio.Task | None = None
        self._tx_selector: TxSelector | None = None
        self._tx_parser_ids: dict[str, object] = {}

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

        if not children:
            return {
                "running": False,
                "pid": None,
                "restarts": restarts,
                "autoRestarts": 0,
                "lastExit": None,
                "fault": engine_fault,
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
        }

    def client_factory(self):
        """Returns a StatsClient-compatible class, stable across restarts.

        `DynamicLinkController`/`ConnectionMonitor` call this ONCE and keep
        the returned class long-term (`stats_client_factory=...`), then
        construct a fresh instance per reconnect attempt. Each instance
        resolves `self.hub` at `run()` time rather than baking in a
        particular hub, so a reconnect attempt made *after* an engine
        restart binds to the new hub. A connection already in progress when
        a restart happens keeps talking to the old (now-idle) hub until
        that `run()` call ends on its own — full mid-flight rebinding is out
        of scope here.
        """
        engine = self

        class _EngineStatsSource:
            def __init__(self, endpoint, on_event, **kw):
                self._endpoint = endpoint
                self._on_event = on_event
                self._kw = kw
                self._inner = None

            async def run(self):
                with engine._lock:
                    hub = engine.hub
                if hub is None:
                    return  # engine not up yet; caller's reconnect loop retries
                inner_cls = hub.client_factory()
                self._inner = inner_cls(self._endpoint, self._on_event, **self._kw)
                await self._inner.run()

            def stop(self):
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
            wlans = self._wlans_resolver(cfg)
        except Exception:
            log.exception("wfb engine: failed to resolve config/wlans")
            return False

        try:
            radio_ok = await loop.run_in_executor(None, self._radio_init, wlans, link)
        except Exception:
            log.exception("wfb engine: radio_init raised")
            return False
        if not radio_ok:
            log.error("wfb engine: radio_init failed for wlans=%s", wlans)
            return False

        try:
            graph = self._graph_builder(cfg, wlans, rand_suffix=_rand_suffix)
        except Exception:
            log.exception("wfb engine: graph_builder raised")
            return False

        tx_selector = TxSelector(_tx_selector_config(cfg))
        hub = StatsHub(tx_selector, time_fn=time.time)

        specs_by_name = {
            "video_rx": graph.video_rx,
            "mavlink_rx": graph.mavlink_rx,
            "tunnel_rx": graph.tunnel_rx,
            "mavlink_tx": graph.mavlink_tx,
            "tunnel_tx": graph.tunnel_tx,
        }

        children: dict[str, object] = {}
        started_order: list[str] = []
        ok_all = True
        for name in RX_LEG_ORDER + TX_LEG_ORDER:
            spec = specs_by_name[name]
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

            hub.add_ant_sel_cb(self._apply_tx_selection)
            # NOTE: `mav_service.start()` above already registers its own
            # `rssi_cb` with the hub (when `cfg.inject_rssi`, the default) —
            # do not also register it here, or RADIO_STATUS frames double up.

            await stats_server.start(loop)
            with self._lock:
                self._stats_server = stats_server
        except Exception:
            log.exception("wfb engine: service wiring failed")
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

        self._tick_task = asyncio.ensure_future(self._ticker())
        return True

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
        await self._stop_children(children, TX_LEG_ORDER)
        await self._stop_children(children, RX_LEG_ORDER)

        with self._lock:
            self._children = {}
            self._tx_selector = None
        self._tx_parser_ids = {}

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
