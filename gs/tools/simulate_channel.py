#!/usr/bin/env python3
"""Closed-loop channel simulation: replay a recorded flight's raw-SNR series
as the exogenous channel and let the CURRENT Policy choose MCS; loss is
generated from the SIMULATED rung (raw SNR below that rung's viability
threshold), not read from the log. A recorded loss series is open-loop — it
happened at the MCS the old selector chose — so replaying it flatters the
flown trajectory; this harness is the directional old-vs-new comparison the
2026-07-06 spec calls for. Model, not proof. Dev-machine tool — not deployed.

Usage:
  simulate_channel.py /path/to/000003.jsonl [--code-root /other/checkout/gs]

--code-root imports fpvdgs from another checkout (e.g. a main worktree) so
old and new selectors run through this same tool and channel model.
"""

import argparse
import json
import statistics
import sys
import tempfile
from pathlib import Path


def load_channel(path):
    """(ts, snr_ewma, snr_w) series + the flight's final learned knees."""
    ticks, knees = [], [None] * 8
    for line in open(path):
        r = json.loads(line)
        if r.get("snr") is None or r.get("snr_ewma") is None:
            continue
        ticks.append((r["ts"], float(r["snr_ewma"]), float(r["snr"])))
        for i, k in enumerate(r.get("snr_knees") or []):
            if k is not None:
                knees[i] = float(k)
    return ticks, knees


def fit_viability(knees):
    """Per-rung viability thresholds: the flight's learned knees; interior
    gaps linearly interpolated; ends extended at the mean learned inter-rung
    step (fallback 3.0 dB/rung)."""
    learned = [(i, k) for i, k in enumerate(knees) if k is not None]
    if not learned:
        raise SystemExit("log has no learned knees - cannot fit a channel model")
    steps = [(kb - ka) / (ib - ia) for (ia, ka), (ib, kb) in zip(learned, learned[1:]) if ib > ia]
    step = statistics.mean(steps) if steps else 3.0
    out = list(knees)
    lo_i, lo_k = learned[0]
    for i in range(lo_i - 1, -1, -1):
        out[i] = lo_k + step * (i - lo_i)
    hi_i, hi_k = learned[-1]
    for i in range(hi_i + 1, len(out)):
        out[i] = hi_k + step * (i - hi_i)
    for (ia, ka), (ib, kb) in zip(learned, learned[1:]):
        for i in range(ia + 1, ib):
            out[i] = ka + (kb - ka) * (i - ia) / (ib - ia)
    return out


LOSS_BELOW_KNEE = 0.3  # window loss emitted when raw SNR is below the rung threshold


def run(ticks, viability, Policy, PolicyConfig, Signals):
    cfg = PolicyConfig()
    cfg.flightlog.enabled = False
    cfg.learned_prior.persist_dir = tempfile.mkdtemp(prefix="simchan-")
    p = Policy(cfg, "sim")
    glitch = changes = 0
    mcs_hist = {}
    last = p.leading.state.current_mcs
    for ts, ewma, raw in ticks:
        mcs = p.leading.state.current_mcs  # loss hits the currently-applied rung
        loss = LOSS_BELOW_KNEE if raw < viability[mcs] else 0.0
        s = Signals(
            snr=ewma,
            snr_w=raw,
            residual_loss_w=loss,
            fec_work=0.0,
            link_starved_w=False,
            timestamp=ts,
        )
        s.packet_rate_w = 1000.0
        p.tick(s)
        cur = p.leading.state.current_mcs
        mcs_hist[cur] = mcs_hist.get(cur, 0) + 1
        if loss > 0.05:
            glitch += 1
        if cur != last:
            changes += 1
            last = cur
    total = len(ticks)
    dur_min = (ticks[-1][0] - ticks[0][0]) / 60.0 if total > 1 else 1.0
    return {
        "ticks": total,
        "glitch_ticks": glitch,
        "glitch_s": glitch / 10.0,
        "mcs_changes": changes,
        "changes_per_min": round(changes / dur_min, 1),
        "mean_mcs": round(sum(k * v for k, v in mcs_hist.items()) / total, 2),
        "time_at_mcs": {k: v for k, v in sorted(mcs_hist.items())},
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log")
    ap.add_argument("--code-root", default=str(Path(__file__).resolve().parents[1]))
    args = ap.parse_args()
    sys.path.insert(0, args.code_root)
    from fpvdgs.dynlink.policy import Policy, PolicyConfig
    from fpvdgs.dynlink.signals import Signals

    ticks, knees = load_channel(args.log)
    viability = fit_viability(knees)
    print(f"viability: {[round(v, 1) for v in viability]}")
    print(json.dumps(run(ticks, viability, Policy, PolicyConfig, Signals), indent=2))


if __name__ == "__main__":
    main()
