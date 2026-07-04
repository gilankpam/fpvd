"""Task 13: supervisor integration + consumer swap.

Two things are load-bearing here:

1. Native wiring end-to-end at the unit level: a REAL `WfbEngine` (real
   `WfbChild` processes running Task 8's fake wfb_rx/wfb_tx scripts, real
   `StatsHub`/`StatsServer`/`TxSelector`) feeding a REAL
   `DynamicLinkController` through `engine.client_factory()` — no TCP
   :8103 hop, no mocks on the dynlink side. Only `mav_service_cls`/
   `tunnel_service_cls` are faked (the mavlink/tunnel legs are irrelevant
   to the video-stats path this test exercises, and the real ones open
   real unix sockets / tun devices).
2. `build_app` always constructs a `WfbEngine` (native is the sole engine;
   the `wfb.engine` flag is gone), without starting it.
"""

from __future__ import annotations

import dataclasses
import socket
import time
from types import SimpleNamespace

import pytest

import fpvdgs.dynlink.controller as controller_mod
from fpvdgs.dynlink.controller import DynamicLinkController
from fpvdgs.runner_supervisor import RunnerSupervisor
from fpvdgs.wfb.children import WfbChild
from fpvdgs.wfb.engine import WfbEngine
from tests.unit.test_wfb_children import make_spec, write_fake_rx, write_fake_tx


@pytest.fixture(autouse=True)
def _isolate_dl_disk(tmp_path, monkeypatch):
    """Same isolation as test_dl_controller.py: keep the real Policy's
    learned-prior/flightlog persistence off the shared system paths
    (/etc/fpvd/learned, /media/dvr/log/dynamic-link/) which are unwritable
    on dev/CI and must never be touched by a test run."""
    real = controller_mod.build_policy_config

    def _to_tmp(block):
        cfg = real(block)
        return dataclasses.replace(
            cfg,
            learned_prior=dataclasses.replace(
                cfg.learned_prior, persist_dir=str(tmp_path / "learned")
            ),
            flightlog=dataclasses.replace(cfg.flightlog, dir=str(tmp_path / "fl")),
        )

    monkeypatch.setattr(controller_mod, "build_policy_config", _to_tmp)


def _free_udp_port() -> int:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("127.0.0.1", 0))
    port = s.getsockname()[1]
    s.close()
    return port


class _FakeService:
    """Stand-in for MavlinkService/TunnelService — the mavlink/tunnel legs
    play no part in the video-stats path this test exercises, and the real
    services would need a real unix rx socket / tun device to construct."""

    def __init__(self, cfg):
        self.cfg = cfg

    async def start(self, loop, rx_unix_path, *args, **kwargs):
        pass

    async def stop(self):
        pass

    def set_tx_socket(self, name):
        pass

    def set_all_tx_sockets(self, names):
        pass

    def rssi_cb(self, *args):
        pass


def _make_graph_builder(tmp_path):
    """Tiny graph_builder pointing every leg's argv at Task 8's fake
    wfb_rx/wfb_tx scripts, so the engine drives REAL WfbChild subprocesses
    end-to-end instead of a stub."""
    rx_argv = [write_fake_rx(tmp_path)]
    tx_argv = [write_fake_tx(tmp_path)]

    def _build(effective, wlans, *, rand_suffix):
        return SimpleNamespace(
            video_rx=make_spec("video_rx", "rx", list(rx_argv)),
            mavlink_rx=make_spec("mavlink_rx", "rx", list(rx_argv)),
            tunnel_rx=make_spec("tunnel_rx", "rx", list(rx_argv)),
            mavlink_tx=make_spec("mavlink_tx", "tx", list(tx_argv)),
            tunnel_tx=make_spec("tunnel_tx", "tx", list(tx_argv)),
            mav_rx_sock=f"mavlink-rx-{rand_suffix()}",
            mav_peer=SimpleNamespace(peer=None),
            tun_rx_sock=f"tunnel-rx-{rand_suffix()}",
            tun_cfg=SimpleNamespace(ifname="gs-wfb-test"),
        )

    return _build


def _make_engine(tmp_path):
    return WfbEngine(
        config_provider=lambda: {"link": {"width": 20}, "wfb": {}},
        wlans_resolver=lambda cfg: ["wlan0"],
        graph_builder=_make_graph_builder(tmp_path),
        child_cls=WfbChild,
        radio_init=lambda wlans, link: True,
        stats_port=0,
        mav_service_cls=_FakeService,
        tunnel_service_cls=_FakeService,
    )


def _wait_until(predicate, timeout=10.0, interval=0.05):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        time.sleep(interval)
    return predicate()


def test_native_engine_feeds_real_dynlink_controller_end_to_end(tmp_path):
    engine = _make_engine(tmp_path)
    assert engine.start() is True
    try:
        snapshot = {
            "enabled": True,
            "maxMcs": 5,
            "droneAddr": "127.0.0.1",
            "dronePort": _free_udp_port(),
            "tap": {"enabled": False},  # exercise the :8103-equivalent (hub) path only
        }
        controller = DynamicLinkController(snapshot, stats_client_factory=engine.client_factory())
        controller.start()
        try:
            assert _wait_until(lambda: controller.status()["statsConnected"] is True)
            # Real video_rx fake-child windows must actually reach the
            # policy through the in-process hub -- not just a "connected"
            # flag -- so also wait for a real emitted decision.
            assert _wait_until(lambda: controller.status()["emitSeq"] > 0)
            st = controller.status()
            assert st["decision"] is not None
            assert st["decision"]["mcs"] is not None
        finally:
            controller.stop()
        assert controller.status()["statsConnected"] is False
    finally:
        engine.shutdown()
    assert engine.state()["running"] is False


def _write_config(tmp_path, extra=""):
    config = tmp_path / "config.json"
    config.write_text(
        '{"link": {"region": "US", "channel": 132, "width": 20, "wlans": ["wlan0"]}' + extra + "}"
    )
    return str(config)


def test_build_app_always_constructs_wfb_engine_without_starting(tmp_path, monkeypatch):
    """Native is the sole engine (the `wfb.engine` flag is gone): build_app
    always constructs a WfbEngine, never a RunnerSupervisor."""
    import fpvdgs.supervisor as sup

    monkeypatch.setattr(sup.render_mod, "write_cfg", lambda *a, **k: None)
    monkeypatch.setattr(sup.render_mod, "render_cfg", lambda eff: "")

    config = _write_config(tmp_path)
    app = sup.build_app(config, str(tmp_path / "out.cfg"), "127.0.0.1", 0)
    assert isinstance(app.runner, WfbEngine)
    assert not isinstance(app.runner, RunnerSupervisor)
    # Never started: no thread, no children.
    assert app.runner.state()["running"] is False
