#!/usr/bin/env python3
"""Offline analysis of a Phase-4 flight log (gs/fpvdgs/dynlink/flightlog.py).

Usage:
    python3 flightlog_analyze.py <flight>.jsonl [--plot out.png]

Prints summary stats. With --plot and matplotlib available, also writes a
SNR-EWMA / RSSI / MCS+flap / probe-PER timeline PNG. Dev-machine tool — not deployed."""

from __future__ import annotations

import argparse
import json
from collections import Counter


def _records(path):
    with open(path) as f:
        for line in f:
            line = line.strip()
            if line:
                try:
                    yield json.loads(line)
                except ValueError:
                    continue


def probe_target_per(rec) -> float | None:
    """PER of the probed rung (operating MCS + 1) from one record, or None
    (pre-field logs, no probe_status, or the rung was never heard)."""
    mcs = rec.get("mcs")
    if mcs is None:
        return None
    entry = (rec.get("probe") or {}).get(str(mcs + 1)) or {}
    return entry.get("per")


def summarize(path) -> dict:
    recs = list(_records(path))
    time_at_mcs = Counter()
    predictive = reactive = gated = 0
    prior_learn = promote_suppressed = 0
    max_flap_level = 0
    last_knees = None
    mcss = []
    snrs, snr_ewmas, evms = [], [], []
    min_evm = None
    for r in recs:
        time_at_mcs[r.get("mcs")] += 1
        reason = r.get("reason") or ""
        if "predict_demote" in reason:
            predictive += 1
        if "video_per_demote" in reason or "emergency" in reason:
            reactive += 1
        if r.get("predict_gated"):
            gated += 1
        if r.get("prior_learn"):
            prior_learn += 1
        if r.get("promote_suppressed"):
            promote_suppressed += 1
        fl = r.get("flap_level")
        if fl is not None:
            max_flap_level = max(max_flap_level, fl)
        if r.get("snr_knees") is not None:
            last_knees = r["snr_knees"]
        if r.get("mcs") is not None:
            mcss.append(r["mcs"])
        if r.get("snr") is not None:
            snrs.append(r["snr"])
        if r.get("snr_ewma") is not None:
            snr_ewmas.append(r["snr_ewma"])
        if r.get("evm") is not None:
            evms.append(r["evm"])
        if r.get("evm_min") is not None:
            min_evm = r["evm_min"] if min_evm is None else min(min_evm, r["evm_min"])
    return {
        "records": len(recs),
        "time_at_mcs": dict(time_at_mcs),
        "predictive_demotes": predictive,
        "reactive_demotes": reactive,
        "gated_demotes": gated,
        "prior_learn_ticks": prior_learn,
        "promote_suppressed_ticks": promote_suppressed,
        "max_flap_level": max_flap_level,
        "last_knees": last_knees,
        "mean_mcs": (sum(mcss) / len(mcss)) if mcss else None,
        "mean_snr": (sum(snrs) / len(snrs)) if snrs else None,
        "mean_snr_ewma": (sum(snr_ewmas) / len(snr_ewmas)) if snr_ewmas else None,
        "mean_evm": (sum(evms) / len(evms)) if evms else None,
        "min_evm": min_evm,
    }


def _print_summary(s: dict) -> None:
    print(f"records: {s['records']}")
    print(f"time-at-MCS (ticks): {s['time_at_mcs']}")
    print(f"predictive demotes: {s['predictive_demotes']}")
    print(f"reactive demotes:   {s['reactive_demotes']}")
    print(f"gated demotes:      {s['gated_demotes']}")
    print(f"prior-learn ticks:  {s['prior_learn_ticks']}")
    print(
        f"promote_suppressed: {s['promote_suppressed_ticks']}   max flap_level: {s['max_flap_level']}"
    )
    print(f"last snr_knees:      {s['last_knees']}")
    print(f"mean SNR / SNR-EWMA: {s['mean_snr']} / {s['mean_snr_ewma']}")
    print(f"mean EVM:            {s['mean_evm']}   min EVM: {s['min_evm']}")
    print(f"mean MCS: {s['mean_mcs']}")


def _plot(path, out):
    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        print("matplotlib not available — skipping plot")
        return
    recs = list(_records(path))
    ts = [r.get("ts") for r in recs]
    fig, ax = plt.subplots(4, 1, sharex=True)
    ax[0].plot(ts, [r.get("snr_ewma") for r in recs], label="snr_ewma")
    ax[0].plot(ts, [r.get("snr") for r in recs], label="snr_raw", alpha=0.4)
    ax[0].legend()
    ax[0].set_ylabel("SNR (dB)")
    ax[1].plot(ts, [r.get("rssi") for r in recs], label="rssi")
    ax[1].plot(ts, [r.get("rssi_raw") for r in recs], label="rssi_raw")
    ax[1].legend()
    ax[1].set_ylabel("RSSI")
    ax[2].plot(ts, [r.get("mcs") for r in recs], label="mcs")
    ax[2].plot(
        ts, [r.get("flap_level", 0) for r in recs], label="flap_level", drawstyle="steps-post"
    )
    ax[2].legend()
    ax[2].set_ylabel("MCS / flap")
    ax[3].plot(ts, [probe_target_per(r) for r in recs], label="probe per (mcs+1)")
    ax[3].legend()
    ax[3].set_ylabel("PER")
    ax[3].set_xlabel("ts")
    fig.savefig(out)
    print(f"wrote {out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("logfile")
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()
    _print_summary(summarize(args.logfile))
    if args.plot:
        _plot(args.logfile, args.plot)


if __name__ == "__main__":
    main()
