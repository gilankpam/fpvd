"""fpvd supervisor: owns config + HTTP API + runner supervision. Pure stdlib."""

import argparse
import sys

from . import __version__, render as render_mod, schema, status as status_mod
from .api import Api, make_http_server
from .config import ConfigStore
from .drone_client import DroneClient
from .link import LinkCoordinator
from .runner_supervisor import RunnerSupervisor, resolve_wlans


class App:
    def __init__(self, store, runner, http_server):
        self.store = store
        self.runner = runner
        self.http = http_server

    def start(self):
        self.runner.start()

    def serve_forever(self):
        self.http.serve_forever()

    def shutdown(self):
        self.http.shutdown()
        self.runner.stop()


def build_app(defaults_path, overlay_path, cfg_out, host, port,
              runner_cmd, ready_port=8103, ready_timeout=10.0, log_path=None):
    store = ConfigStore.load(defaults_path, overlay_path)
    effective = store.effective()
    schema.validate_effective(effective)

    # Render the cfg the runner will read.
    render_mod.write_cfg(cfg_out, render_mod.render_cfg(effective))

    profile = effective.get("wfb", {}).get("profile", "gs")
    wlans = resolve_wlans(effective)
    runner = RunnerSupervisor(runner_cmd, cfg_out=cfg_out, profile=profile,
                              wlans=wlans, ready_port=ready_port,
                              ready_timeout=ready_timeout, log_path=log_path)

    drone = DroneClient(effective.get("drone", {}).get("endpoint", "http://10.5.0.10:8080"))

    def renderer_write(eff):
        render_mod.write_cfg(cfg_out, render_mod.render_cfg(eff))

    link = LinkCoordinator(store, renderer_write, runner, drone)

    def status_fn():
        wlan_info = {w: status_mod.iw_info(w) for w in resolve_wlans(store.effective())}
        reachable = drone.healthz()
        eff_link = store.effective().get("link", {})
        probe = {"reachable": reachable, "linkId": eff_link.get("linkId"),
                 "inSync": None}
        return status_mod.build_status(__version__, runner.state(), wlan_info, probe)

    api = Api(store=store, schema=schema, render_mod=render_mod, runner=runner,
              drone=drone, link=link, status_fn=status_fn, cfg_out=cfg_out)

    http_server = make_http_server(api, host, port)
    return App(store, runner, http_server)


def main(argv=None):
    p = argparse.ArgumentParser(prog="fpvd")
    p.add_argument("--defaults", default="/etc/fpvd/defaults.json")
    p.add_argument("--config", default="/etc/fpvd/config.json")
    p.add_argument("--cfg-out", default="/etc/wifibroadcast.cfg")
    p.add_argument("--host", default="0.0.0.0")
    p.add_argument("--port", type=int, default=8080)
    p.add_argument("--log", default=None)
    p.add_argument("--runner", default=None,
                   help="runner command (default: this python -m fpvdgs.runner)")
    args = p.parse_args(argv)

    runner_cmd = (args.runner.split() if args.runner
                  else [sys.executable, "-m", "fpvdgs.runner"])
    app = build_app(args.defaults, args.config, args.cfg_out, args.host, args.port,
                    runner_cmd, log_path=args.log)
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
