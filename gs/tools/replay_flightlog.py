#!/usr/bin/env python3
"""Replay a real flight log's signal series through the current Policy and
summarize what the probe-less selector WOULD have done (directional — decisions
change the trajectory, so downstream signals are approximations). Dev-machine
tool — not deployed.

Usage: replay_flightlog.py /path/to/000020.jsonl
"""

import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fpvdgs.dynlink.policy import Policy, PolicyConfig  # noqa: E402
from fpvdgs.dynlink.signals import Signals  # noqa: E402


def main() -> None:
    path = sys.argv[1]
    cfg = PolicyConfig()
    cfg.flightlog.enabled = False
    cfg.learned_prior.persist_dir = "/tmp/replay-learned"
    p = Policy(cfg, "replay")
    changes = suppressed = 0
    mcs_hist: dict[int, int] = {}
    last = p.leading.state.current_mcs
    for line in open(path):
        r = json.loads(line)
        s = Signals()
        s.timestamp = r["ts"]
        s.snr = r.get("snr_ewma")
        s.snr_w = r.get("snr")
        s.residual_loss_w = r.get("residual_loss_w") or 0.0
        s.fec_work = r.get("fec_work") or 0.0
        s.link_starved_w = bool(r.get("link_starved"))
        s.session = object()
        s.packet_rate_w = 0.0 if r.get("link_starved") else 1000.0
        p.tick(s)
        cur = p.leading.state.current_mcs
        mcs_hist[cur] = mcs_hist.get(cur, 0) + 1
        if cur != last:
            changes += 1
            last = cur
        if p.leading._promote_suppressed:
            suppressed += 1
    total = sum(mcs_hist.values())
    print(f"ticks: {total}  mcs-changes: {changes}")
    print(f"time-at-MCS: { {k: v for k, v in sorted(mcs_hist.items())} }")
    print(f"promote_suppressed ticks: {suppressed} ({100 * suppressed / total:.0f}%)")
    print(f"knees: {p.learned_prior.snr_knees_snapshot()}")


if __name__ == "__main__":
    main()
