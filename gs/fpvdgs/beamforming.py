"""GS beamformee responder.

Arms the rtl88x2eu monitor-BF hardware (`CONFIG_BEAMFORMING_MONITOR`) to
auto-echo the drone's downlink sounding. Config-only: once `bf_monitor_conf`
is armed with the drone's MAC, the WLAN-MAC hardware assembles and transmits
the VHT compressed beamforming report within SIFS, so there is NO sounding
loop here (unlike the drone-side beamformer). Mirrors the shape of
`drone/src/supervise/beamforming.cpp`, beamformee half only.
"""

import os

PROC_BASE = "/proc/net/rtl88x2eu"
SYS_BASE = "/sys/class/net"


def read_mac(iface: str, sys_base: str = SYS_BASE) -> str:
    try:
        with open(f"{sys_base}/{iface}/address") as f:
            return f.read().strip()
    except OSError:
        return ""


class BeamformingController:
    def __init__(self, proc_base: str = PROC_BASE, sys_base: str = SYS_BASE):
        self._proc_base = proc_base
        self._sys_base = sys_base
        self._armed = False
        self._iface = ""
        self._peer = ""
        self._state = "disabled"   # disabled | unsupported | active | error
        self._reason = ""

    def supported(self, iface: str) -> bool:
        return os.path.exists(f"{self._proc_base}/{iface}/bf_monitor_conf")

    def local_mac(self, iface: str) -> str:
        return read_mac(iface, self._sys_base)

    def _write_conf(self, iface: str, content: str) -> bool:
        try:
            with open(f"{self._proc_base}/{iface}/bf_monitor_conf", "w") as f:
                f.write(content)
            return True
        except OSError:
            return False

    def reconcile(self, enabled: bool, iface: str, peer_mac: str) -> dict:
        if not enabled:
            if self._armed and self.supported(self._iface):
                if not self._write_conf(self._iface, "0 00:00:00:00:00:00 0 0"):
                    self._armed = False
                    self._state, self._reason = "error", "bf_monitor_conf reset failed"
                    return self.status()
            self._armed, self._iface, self._peer = False, iface, ""
            self._state, self._reason = "disabled", ""
            return self.status()

        if not self.supported(iface):
            self._armed, self._iface, self._peer = False, iface, peer_mac
            self._state = "unsupported"
            self._reason = f"no bf_monitor_conf node on {iface}"
            return self.status()

        if self._armed and self._iface == iface and self._peer == peer_mac:
            return self.status()   # idempotent: no rewrite

        if self._write_conf(iface, f"1 {peer_mac} 0 0"):
            self._armed, self._iface, self._peer = True, iface, peer_mac
            self._state, self._reason = "active", ""
        else:
            self._armed, self._iface, self._peer = False, iface, peer_mac
            self._state, self._reason = "error", "bf_monitor_conf write failed"
        return self.status()

    def status(self) -> dict:
        return {
            "requested": self._state != "disabled",
            "state": self._state,
            "reason": self._reason,
            "iface": self._iface,
            "localMac": self.local_mac(self._iface) if self._iface else "",
            "peerMac": self._peer,
        }

    def status_with_primary(self, primary_iface) -> dict:
        """status(), but with iface/localMac filled from the primary card when
        the beamformee isn't armed yet. The GS card MAC is what the drone needs
        as link.beamforming.remoteMac, so a client must be able to read it from
        status BEFORE enabling BF (the armer only learns its iface on reconcile)."""
        st = self.status()
        if not st["localMac"] and primary_iface:
            st["localMac"] = self.local_mac(primary_iface)
            if not st["iface"]:
                st["iface"] = primary_iface
        return st
