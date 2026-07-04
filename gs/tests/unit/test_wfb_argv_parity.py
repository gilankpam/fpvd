"""Argv-parity golden test: `fpvdgs.wfb.graph.build_graph` vs REAL wfb-ng.

This is the keystone regression net for the native wfb data-plane port: it
renders an fpvd effective config to a scratch `wifibroadcast.cfg` via the
test-only `_wfb_ng_cfg_render.render_cfg` helper (fpvd's own `render.py` was
deleted -- the native `WfbEngine` builds its own child argv and never reads
this file at runtime; this helper is kept alive purely so this test can
still hand wfb-ng a config to parse), feeds it to the actual `wfb_ng`
package (the fpvd fork, `swfec` branch) via `WIFIBROADCAST_CFG`, and
compares the five child argv lists `build_graph` produces against argvs
re-rendered from `wfb_ng.services`'s *own* format strings, filled from
`parse_services`'s real per-service cfg (all profile inheritance applied
exactly as wfb-ng resolves it). This catches argv drift between the native
port and upstream that hand-transcribed golden tests (`test_wfb_graph.py`)
cannot.

Run (needs the wfb-ng checkout; deps `twisted`/`msgpack`/`pyserial`/
`pyroute2` are test-time only, never shipped by fpvdgs):

    WFB_NG_SRC=~/Projects/poc/wfb-ng .venv/bin/python -m pytest tests/unit/test_wfb_argv_parity.py -v

Without `WFB_NG_SRC` set, this whole file SKIPS (verified as part of the
full `pytest tests/ -q` run staying green).

What this proves vs what it assumes
------------------------------------
Proves: for the "gs" profile, `build_graph`'s five argvs are byte-for-byte
identical to what `wfb_ng.services.init_udp_direct_rx` / `init_mavlink` /
`init_tunnel` render from the *same* rendered `wifibroadcast.cfg`, across
two scenarios (default 20 MHz width with the dynlink tap on; 10 MHz width
with the tap off, exercising the `-B min(width, 20)` uplink cap and the
tap flag toggle).

Assumes / does not prove:
  - The reference re-renders services.py's format strings inline (copied
    from `wfb_ng/services.py`, branch `swfec`) rather than shelling out to
    wfb-ng's own process-spawning code (`init_udp_direct_rx` et al. spawn
    real subprocesses via Twisted's reactor and are not synchronously
    callable) -- so a future upstream edit to those format strings needs a
    human to re-sync this file's copies. `parse_services` itself IS the
    real function, which is where the profile-inheritance fidelity comes
    from.
  - `render_cfg`'s uplink bandwidth cap (`min(link.width, 20)` on
    `gs_mavlink`/`gs_tunnel`) is an intentional fpvd override, documented
    in `_wfb_ng_cfg_render.py`; the reference cfg is rendered by `render_cfg`
    too, so `parse_services` already resolves the capped value and this test
    cannot distinguish "fpvd's cap is correct" from "fpvd's cap is
    self-consistent" -- the cap's *rationale* is validated elsewhere
    (bench/flight logs), not here.
  - `wfb_ng/conf/local.cfg` is a dev-only convenience file in the wfb-ng
    checkout (parsed unconditionally by `conf/__init__.py`'s hardcoded
    `_cfg_files` list) that overrides `path.bin_dir`/`path.conf_dir` to
    `.`, for running against locally-built binaries. It is NOT packaged
    for distribution (`setup.py`'s `package_data` ships only
    `master.cfg`/`site.cfg`), so a real GS install never sees it. This
    test neutralizes it after import so the reference matches what a real
    deployed install resolves -- `master.cfg`'s `/usr/bin`/`/etc`, the
    same values `graph.py` hardcodes as `WFB_BIN_DIR`/`GS_KEY`. Without
    this, every argv would spuriously mismatch on the binary/key path.
"""

from __future__ import annotations

import os
import sys

import pytest

from fpvdgs.config import deep_merge
from fpvdgs.config_defaults import default_config
from fpvdgs.wfb.graph import build_graph

from ._wfb_ng_cfg_render import render_cfg, write_cfg

WFB_NG_SRC = os.environ.get("WFB_NG_SRC")
pytestmark = pytest.mark.skipif(
    not WFB_NG_SRC, reason="set WFB_NG_SRC to the wfb-ng checkout to run parity"
)

WLANS = ["wlan1", "wlan2"]
SFX = "aaaaaaaa"  # 8 hex-shaped chars, matching os.urandom(4).hex()'s length


def _load_wfb_ng(cfg_path: str):
    """(Re)import `wfb_ng` against `cfg_path`, forcing a fresh config parse.

    `wfb_ng.conf` parses `WIFIBROADCAST_CFG` at *module import time* (a
    module-level statement in `conf/__init__.py`), so each scenario needs
    a clean re-import after the env var changes -- purge any previously
    imported `wfb_ng.*` modules first (safe: we don't touch `twisted.*`,
    so its reactor singleton is untouched across reimports).
    """
    os.environ["WIFIBROADCAST_CFG"] = cfg_path
    if WFB_NG_SRC not in sys.path:
        sys.path.insert(0, WFB_NG_SRC)
    for name in list(sys.modules):
        if name == "wfb_ng" or name.startswith("wfb_ng."):
            del sys.modules[name]

    import wfb_ng.services as services
    from wfb_ng.conf import settings

    settings.path.bin_dir = "/usr/bin"
    settings.path.conf_dir = "/etc"
    return services, settings


def _render(tmp_path, overlay: dict):
    eff = deep_merge(default_config(), overlay)
    cfg_path = tmp_path / "wifibroadcast.cfg"
    write_cfg(str(cfg_path), render_cfg(eff))
    return eff, str(cfg_path)


# ---- reference argv construction ---------------------------------------
# Transcribed from wfb_ng/services.py's init_udp_direct_rx / init_mavlink /
# init_tunnel format strings (fpvd fork, branch `swfec`). Filled from
# `parse_services`'s real per-service cfg objects, so profile inheritance
# (base -> gs_base -> {video,mavlink,tunnel} -> {gs_video,gs_mavlink,
# gs_tunnel}, plus our rendered overrides) is the genuine wfb-ng resolver,
# not a hand-copied guess.
# Note: %(cluster)s template branch in wfb-ng is intentionally omitted; fpvd never runs cluster mode on this path.


def _rx_argv(settings, key_arg_fn, cfg, stream, conn_str, wlans, link_id, *, tap=False):
    tap_port = int(getattr(cfg, "dynlink_tap_port", 0) or 0)
    cmd = (
        "%(cmd)s -p %(stream)d %(conn_str)s %(key_arg)s -R %(rcv_buf_size)d "
        "-s %(snd_buf_size)d -l %(log_interval)d -i %(link_id)d%(tap)s"
        % dict(
            cmd=os.path.join(settings.path.bin_dir, "wfb_rx"),
            stream=stream,
            conn_str=conn_str,
            key_arg=key_arg_fn(cfg),
            rcv_buf_size=settings.common.tx_rcv_buf_size,
            snd_buf_size=settings.common.rx_snd_buf_size,
            log_interval=settings.common.log_interval,
            link_id=link_id,
            tap=" -D %d" % tap_port if (tap and tap_port) else "",
        )
    ).split() + wlans
    return cmd


def _tx_argv(settings, key_arg_fn, cfg, stream, unix_path, wlans, link_id):
    cmd = (
        "%(cmd)s -f %(frame_type)s -p %(stream)d -U %(unix_socket)s %(key_arg)s "
        "-B %(bw)d -G %(gi)s -S %(stbc)d -L %(ldpc)d -M %(mcs)d"
        "%(mirror)s%(force_vht)s%(qdisc)s "
        "-k %(fec_k)d -n %(fec_n)d -T %(fec_timeout)d -F %(fec_delay)d -i %(link_id)d "
        "-R %(rcv_buf_size)d -s %(snd_buf_size)d -l %(log_interval)d -C %(control_port)d"
        % dict(
            cmd=os.path.join(settings.path.bin_dir, "wfb_tx"),
            frame_type=cfg.frame_type,
            stream=stream,
            unix_socket=unix_path,
            control_port=cfg.control_port,
            key_arg=key_arg_fn(cfg),
            bw=cfg.bandwidth,
            force_vht=" -V" if cfg.force_vht else "",
            qdisc=" -Q -P %d" % (cfg.fwmark,) if cfg.use_qdisc else "",
            gi="short" if cfg.short_gi else "long",
            stbc=cfg.stbc,
            ldpc=cfg.ldpc,
            mcs=cfg.mcs_index,
            mirror=" -m" if cfg.mirror else "",
            fec_k=cfg.fec_k,
            fec_n=cfg.fec_n,
            fec_timeout=cfg.fec_timeout,
            fec_delay=cfg.fec_delay,
            link_id=link_id,
            log_interval=settings.common.log_interval,
            rcv_buf_size=settings.common.tx_rcv_buf_size,
            snd_buf_size=settings.common.rx_snd_buf_size,
        )
    ).split() + wlans
    return cmd


@pytest.mark.parametrize(
    "overlay",
    [
        pytest.param({"link": {"channel": 132, "width": 20}}, id="width20-tap-on"),
        pytest.param(
            {
                "link": {"channel": 132, "width": 10},
                "dynamicLink": {"tap": {"enabled": False}},
            },
            id="width10-tap-off",
        ),
    ],
)
def test_argv_matches_real_wfb_ng(tmp_path, overlay):
    eff, cfg_path = _render(tmp_path, overlay)

    try:
        services, settings = _load_wfb_ng(cfg_path)
    except ImportError as e:
        pytest.xfail(f"wfb_ng import failed (missing dev-only dep?): {e}")

    link_id = services.hash_link_domain(settings.gs.link_domain)
    by_name = {name: cfg for name, _stype, cfg in services.parse_services("gs", None)}
    video_cfg, mav_cfg, tun_cfg = by_name["video"], by_name["mavlink"], by_name["tunnel"]

    ref_video = _rx_argv(
        settings,
        services.key_arg,
        video_cfg,
        video_cfg.stream_rx,
        "-c 127.0.0.1 -u 5600",
        WLANS,
        link_id,
        tap=True,
    )
    ref_mav_rx = _rx_argv(
        settings,
        services.key_arg,
        mav_cfg,
        mav_cfg.stream_rx,
        f"-U mavlink-rx-{SFX}",
        WLANS,
        link_id,
    )
    ref_mav_tx = _tx_argv(
        settings, services.key_arg, mav_cfg, mav_cfg.stream_tx, f"mavlink-tx-{SFX}", WLANS, link_id
    )
    ref_tun_rx = _rx_argv(
        settings,
        services.key_arg,
        tun_cfg,
        tun_cfg.stream_rx,
        f"-U tunnel-rx-{SFX}",
        WLANS,
        link_id,
    )
    ref_tun_tx = _tx_argv(
        settings, services.key_arg, tun_cfg, tun_cfg.stream_tx, f"tunnel-tx-{SFX}", WLANS, link_id
    )

    g = build_graph(eff, WLANS, rand_suffix=lambda: SFX)

    assert g.video_rx.argv == ref_video
    assert g.mavlink_rx.argv == ref_mav_rx
    assert g.mavlink_tx.argv == ref_mav_tx
    assert g.tunnel_rx.argv == ref_tun_rx
    assert g.tunnel_tx.argv == ref_tun_tx
