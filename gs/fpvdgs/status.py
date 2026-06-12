"""Assemble GET /gs/status: GS-local runner + per-wlan radio + link state.
Drone state is intentionally excluded (clients read /air/status for that)."""

import re
import subprocess


def parse_iw_info(text: str) -> dict:
    out: dict = {}
    m = re.search(r"^\s*type (\w+)", text, re.M)
    if m:
        out["type"] = m.group(1)
    m = re.search(r"channel (\d+) \((\d+) MHz\), width: (\d+) MHz", text)
    if m:
        out["channel"] = int(m.group(1))
        out["freqMhz"] = int(m.group(2))
        out["widthMhz"] = int(m.group(3))
    m = re.search(r"txpower ([\d.]+) dBm", text)
    if m:
        out["txpowerDbm"] = float(m.group(1))
    return out


def iw_info(wlan: str) -> dict:
    try:
        out = subprocess.run(["iw", "dev", wlan, "info"],
                             capture_output=True, text=True, timeout=3)
        return parse_iw_info(out.stdout)
    except (OSError, subprocess.SubprocessError):
        return {}


def build_status(version: str, runner_state: dict, wlans: dict,
                 link_info: dict, link_stats: dict | None = None,
                 uptime_ms: int | None = None,
                 dynamic_link: dict | None = None,
                 pixelpilot: dict | None = None,
                 probe: dict | None = None,
                 beamforming: dict | None = None) -> dict:
    radio = []
    for wlan, info in wlans.items():
        radio.append({"wlan": wlan, **info})
    # GS-local link view only. Drone reachability / cross-device sync are NOT
    # reported here — clients read /air/status for drone state.
    link = {"linkId": link_info.get("linkId")}
    if link_stats:
        link["stats"] = link_stats
    fpvd = {"version": version}
    if uptime_ms is not None:
        fpvd["uptimeMs"] = uptime_ms
    out = {
        "fpvd": fpvd,
        "runner": runner_state,
        "radio": radio,
        "link": link,
    }
    if dynamic_link is not None:
        out["dynamicLink"] = dynamic_link
    if pixelpilot is not None:
        out["pixelpilot"] = pixelpilot
    if probe is not None:
        out["probe"] = probe
    if beamforming is not None:
        out["beamforming"] = beamforming
    return out
