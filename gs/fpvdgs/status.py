"""Assemble GET /status: runner state + per-wlan radio state + link/drone."""

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
                 drone_probe: dict, link_stats: dict | None = None,
                 uptime_ms: int | None = None) -> dict:
    radio = []
    for wlan, info in wlans.items():
        radio.append({"wlan": wlan, **info})
    link = {
        "linkId": drone_probe.get("linkId"),
        "droneReachable": drone_probe.get("reachable", False),
        "inSync": drone_probe.get("inSync"),
    }
    if link_stats:
        link["stats"] = link_stats
    fpvd = {"version": version}
    if uptime_ms is not None:
        fpvd["uptimeMs"] = uptime_ms
    return {
        "fpvd": fpvd,
        "runner": runner_state,
        "radio": radio,
        "link": link,
    }
