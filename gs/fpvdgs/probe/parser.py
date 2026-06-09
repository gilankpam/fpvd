"""Pure parsing of wfb_rx stdout IPC lines + per-MCS EWMA aggregation.

Line formats (tab-separated), per ../wfb-ng/src/rx.cpp dump_stats():
  RX_ANT: <ts>\tRX_ANT\t<freq>:<mcs>:<bw>\t<ant>\t<count>:<rssi_min>:<rssi_avg>:<rssi_max>:<snr_min>:<snr_avg>:<snr_max>
  PKT:    <ts>\tPKT\t<all>:<all_bytes>:<dec_err>:<session>:<data>:<uniq>:<fec_rec>:<lost>:<bad>:<out>:... (>=11 fields)
The needed PKT indices (data=4, fec_rec=6, lost=7) are stable across versions
(newer wfb-ng appends fields). FEC is off on the probe, so fec_rec≈0 and the raw
on-air PER for a window is lost/(data+lost).
"""
from __future__ import annotations


def parse_line(line: str):
    """Return ('RX_ANT', {mcs,rssi,snr}) | ('PKT', {data,fec_rec,lost}) | None."""
    cols = line.rstrip("\n").split("\t")
    if len(cols) < 3:
        return None
    kind = cols[1]
    try:
        if kind == "RX_ANT" and len(cols) >= 5:
            _freq, mcs, _bw = (int(x) for x in cols[2].split(":"))
            vals = [int(x) for x in cols[4].split(":")]
            if len(vals) < 7:
                return None
            # vals = [count, rssi_min, rssi_avg, rssi_max, snr_min, snr_avg, snr_max]
            return ("RX_ANT", {"mcs": mcs, "rssi": vals[2], "snr": vals[5]})
        if kind == "PKT":
            f = [int(x) for x in cols[2].split(":")]
            if len(f) < 8:
                return None
            return ("PKT", {"data": f[4], "fec_rec": f[6], "lost": f[7]})
    except ValueError:
        return None
    return None


class McsAggregator:
    """Per-MCS EWMA of raw on-air PER + latest RSSI/SNR.

    Each probe wfb_rx receives one MCS, so RX_ANT supplies the MCS label (and
    rssi/snr) and the following PKT lines supply that MCS's window data/lost.
    Callers route on_rx_ant/on_pkt with the mcs from the latest RX_ANT.

    The probe feeder is a low-rate trickle (~1 packet per stats window), so a
    single window with no decoded packets is *sparsity*, not loss — scoring it
    as 100% loss was the dominant source of a wildly inflated PER (a healthy
    rung reading ~0.2 instead of ~0.01). An empty window therefore carries no
    PER information and is ignored. Only a *run* of `blackout_windows`
    consecutive empties — which sparsity practically never produces — is a real
    blackout and pins per=1.0, preserving the promote-blocking contract.
    """

    def __init__(self, alpha: float = 0.25, blackout_windows: int = 10):
        self.alpha = alpha
        self.blackout_windows = blackout_windows
        self._m: dict[int, dict] = {}

    def _slot(self, mcs: int) -> dict:
        return self._m.setdefault(
            mcs, {"per": None, "rssi": None, "snr": None, "windows": 0,
                  "_empty": 0})

    def on_rx_ant(self, mcs: int, rssi: int, snr: int) -> None:
        s = self._slot(mcs)
        s["rssi"], s["snr"] = rssi, snr

    def on_pkt(self, mcs: int, data: int, lost: int) -> None:
        s = self._slot(mcs)
        denom = data + lost
        if denom <= 0:
            # No decodes this window. Sparse-feeder gap, not loss — ignore it
            # unless it's part of a sustained run (a genuine blackout).
            s["_empty"] += 1
            if s["_empty"] >= self.blackout_windows:
                s["per"] = 1.0
            return
        s["_empty"] = 0
        win_per = lost / denom
        s["per"] = win_per if s["per"] is None else (
            self.alpha * win_per + (1 - self.alpha) * s["per"])
        s["windows"] += 1

    def snapshot(self) -> dict[int, dict]:
        return {mcs: {k: v for k, v in s.items() if k != "_empty"}
                for mcs, s in self._m.items()}
