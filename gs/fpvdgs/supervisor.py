"""fpvd supervisor: owns config + HTTP API + runner supervision. Pure stdlib."""

import argparse
import json
import logging
import signal
import sys
import time

from . import __version__, radio, schema
from . import render as render_mod
from . import status as status_mod
from .api import Api, make_http_server
from .beamforming import BeamformingController
from .beamforming_armer import BeamformingArmer
from .config import ConfigStore
from .config_defaults import default_config
from .connection_monitor import ConnectionMonitor, ConnectionMonitorConfig
from .drone_client import DroneClient
from .dynlink.config_build import make_dl_snapshot
from .dynlink.controller import DynamicLinkController
from .events import EventBus
from .idr_relay import IdrRelay
from .pixelpilot import render_pixelpilot_argv, render_pixelpilot_env
from .probe.config_build import make_probe_snapshot
from .probe.controller import ProbeController
from .runner_supervisor import ProcessSupervisor, RunnerSupervisor, resolve_wlans

log = logging.getLogger(__name__)


def adapter_matches_profile(adapter_id, radio_profile) -> bool:
    """Loose match: the drone's radio-up.sh adapter_id (e.g. 'bl-m8812eu2')
    should contain the configured radioProfile (e.g. 'm8812eu2'). Unknown
    adapter_id (None / "") → treated as a match (no warning)."""
    if not adapter_id:
        return True
    return str(radio_profile) in str(adapter_id)


class App:
    def __init__(
        self,
        store,
        runner,
        http_server,
        api,
        dynlink,
        pixelpilot=None,
        probe=None,
        armer=None,
        idr_relay=None,
        connection_monitor=None,
        bus=None,
    ):
        self.store = store
        self.runner = runner
        self.http = http_server
        self.api = api
        self.dynlink = dynlink
        self.pixelpilot = pixelpilot
        self.probe = probe
        self.armer = armer
        self.idr_relay = idr_relay
        self.connection_monitor = connection_monitor
        self.bus = bus

    def start(self):
        self.runner.start()
        if self.connection_monitor is not None:
            self.connection_monitor.start()
        if self.armer is not None:
            self.armer.start()  # boot re-arm: keeps the GS beamformee armed to config
        if self.pixelpilot is not None and self.store.effective().get("pixelpilot", {}).get(
            "enabled", True
        ):
            self.pixelpilot.start()
        if self.idr_relay is not None and self.store.effective().get("idrForward", {}).get(
            "enabled", True
        ):
            self.idr_relay.start()
        if self.store.effective().get("dynamicLink", {}).get("enabled"):
            self.dynlink.start()
        if self.probe is not None and self.store.effective().get("dynamicLink", {}).get("enabled"):
            self.probe.start()

    def serve_forever(self):
        self.http.serve_forever()

    def shutdown(self):
        self.http.shutdown()
        if self.connection_monitor is not None:
            self.connection_monitor.stop()
        if self.armer is not None:
            self.armer.stop()
        self.dynlink.stop()
        if self.pixelpilot is not None:
            self.pixelpilot.shutdown()
        if self.idr_relay is not None:
            self.idr_relay.stop()
        if self.probe is not None:
            self.probe.stop()
        self.runner.shutdown()


def build_app(
    config_path,
    cfg_out,
    host,
    port,
    runner_cmd,
    ready_port=8103,
    ready_timeout=10.0,
    log_path=None,
    probe_spawn=None,
):
    store = ConfigStore.load(config_path)
    effective = store.effective()
    schema.validate_effective(effective)

    # Render the cfg the runner will read.
    render_mod.write_cfg(cfg_out, render_mod.render_cfg(effective))

    profile = effective.get("wfb", {}).get("profile", "gs")
    wlans = resolve_wlans(effective)
    runner = RunnerSupervisor(
        runner_cmd,
        cfg_out=cfg_out,
        profile=profile,
        wlans=wlans,
        ready_port=ready_port,
        ready_timeout=ready_timeout,
        log_path=log_path,
    )

    drone_cfg = effective.get("drone", {})
    drone_host = drone_cfg.get("host", "10.5.0.10")
    drone = DroneClient(f"http://{drone_host}:{int(drone_cfg.get('apiPort', 8080))}")

    bus = EventBus()
    cm_cfg = effective.get("connectionMonitor") or {}  # tolerate an explicit null
    dcm = ConnectionMonitorConfig()
    mon_cfg = ConnectionMonitorConfig(
        enabled=bool(cm_cfg.get("enabled", dcm.enabled)),
        tunnel_stale_s=float(cm_cfg.get("tunnelStaleS", dcm.tunnel_stale_s)),
        http_poll_s=float(cm_cfg.get("httpPollS", dcm.http_poll_s)),
        http_timeout_s=float(cm_cfg.get("httpTimeoutS", dcm.http_timeout_s)),
        http_fail_count=int(cm_cfg.get("httpFailCount", dcm.http_fail_count)),
        eval_interval_s=float(cm_cfg.get("evalIntervalS", dcm.eval_interval_s)),
    )
    mon_drone = DroneClient(
        f"http://{drone_host}:{int(drone_cfg.get('apiPort', 8080))}", timeout=mon_cfg.http_timeout_s
    )
    connection_monitor = ConnectionMonitor(bus, mon_drone, mon_cfg)

    idr_cfg = effective.get("idrForward", {})
    idr_relay = IdrRelay(drone_host, port=int(idr_cfg.get("port", 11223)))

    probe_ctrl = ProbeController(make_probe_snapshot(effective), spawn=probe_spawn)

    dynlink = DynamicLinkController(
        make_dl_snapshot(effective), probe_status=probe_ctrl.status, bus=bus
    )

    pixelpilot = ProcessSupervisor(
        argv=render_pixelpilot_argv(effective),
        env=render_pixelpilot_env(effective),
        ready_timeout=1.5,
        ready_on_timeout=True,  # settle: alive through the window
        log_path="/tmp/pixelpilot.log",
    )

    beamforming = BeamformingController()

    def _bf_capable(cfg):
        wlans = resolve_wlans(cfg)
        primary = wlans[0] if wlans else None
        return bool(primary and beamforming.supported(primary))

    schema.set_bf_capable(_bf_capable)

    armer = BeamformingArmer(beamforming, drone, resolve_wlans, lambda: store.effective())

    started = time.monotonic()

    def _dynamic_link_status():
        # GS-local controller state only — no drone round-trip. Clients read
        # /air/status for the drone's own dynamic-link/adapter state.
        eff_dl = store.effective().get("dynamicLink", {})
        st = dynlink.status()
        st["enabled"] = bool(eff_dl.get("enabled"))
        return st

    def _pixelpilot_status():
        pp_cfg = store.effective().get("pixelpilot", {})
        if not bool(pp_cfg.get("enabled", True)):
            return {"enabled": False, "running": False}
        return {"enabled": True, **pixelpilot.state()}

    def _probe_status():
        if not store.effective().get("dynamicLink", {}).get("enabled"):
            return {"enabled": False, "running": False}
        return {"enabled": True, **probe_ctrl.status()}

    def status_fn():
        wlans = resolve_wlans(store.effective())
        wlan_info = {w: status_mod.iw_info(w) for w in wlans}
        eff_link = store.effective().get("link", {})
        link_info = {"linkId": eff_link.get("linkId")}
        uptime_ms = int((time.monotonic() - started) * 1000)
        primary = wlans[0] if wlans else None
        return status_mod.build_status(
            __version__,
            runner.state(),
            wlan_info,
            link_info,
            uptime_ms=uptime_ms,
            dynamic_link=_dynamic_link_status(),
            pixelpilot=_pixelpilot_status(),
            probe=_probe_status(),
            beamforming=beamforming.status_with_primary(primary),
            connection=connection_monitor.status(),
        )

    api = Api(
        store=store,
        schema=schema,
        render_mod=render_mod,
        runner=runner,
        drone=drone,
        status_fn=status_fn,
        cfg_out=cfg_out,
        dynlink=dynlink,
        pixelpilot=pixelpilot,
        probe=probe_ctrl,
        retune=lambda lnk: radio.retune(resolve_wlans(store.effective()), lnk),
        wlans_resolver=resolve_wlans,
        armer_tick=armer.tick,
        idr_relay=idr_relay,
    )

    http_server = make_http_server(api, host, port)
    return App(
        store,
        runner,
        http_server,
        api,
        dynlink,
        pixelpilot=pixelpilot,
        probe=probe_ctrl,
        armer=armer,
        idr_relay=idr_relay,
        connection_monitor=connection_monitor,
        bus=bus,
    )


def main(argv=None):
    p = argparse.ArgumentParser(prog="fpvd")
    p.add_argument("--config", default="/etc/fpvd/config.json")
    p.add_argument(
        "--dump-config", action="store_true", help="print the full default config as JSON and exit"
    )
    p.add_argument("--cfg-out", default="/etc/wifibroadcast.cfg")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--log", default=None)
    p.add_argument(
        "--runner", default=None, help="runner command (default: this python -m fpvdgs.runner)"
    )
    args = p.parse_args(argv)

    if args.dump_config:
        print(json.dumps(default_config(), indent=2))
        return

    runner_cmd = args.runner.split() if args.runner else [sys.executable, "-m", "fpvdgs.runner"]
    app = build_app(args.config, args.cfg_out, args.host, args.port, runner_cmd, log_path=args.log)

    def _on_sigterm(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, _on_sigterm)

    app.start()
    sys.stderr.write(f"fpvd: listening on {args.host}:{args.port}\n")
    try:
        app.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        app.shutdown()


if __name__ == "__main__":
    main()
