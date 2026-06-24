"""Replay regression: a real 10 MHz flight (000003) where rungs 1-5 were only
ever clean. After the learn-from-failures fix, their knees must stay None (no
inflation); rung0 (2 real failures) must learn a knee. Guards against a revert
to the clean-sample-inflation bug."""

import json
import os

from fpvdgs.dynlink.learned_prior import KneeModel, LearnedPriorConfig

FIXTURE = os.path.join(os.path.dirname(__file__), "..", "fixtures", "knee_replay_000003.json")


def test_clean_only_flight_does_not_inflate_knees():
    stream = json.load(open(FIXTURE))
    m = KneeModel(LearnedPriorConfig())
    for mcs, snr, clean in stream:
        m.observe(mcs, float(snr), bool(clean))
    # rungs 1-5 were clean-only in this flight -> no failure -> knee stays None
    assert all(m._knee[r] is None for r in range(1, 6)), m.knees_snapshot()
    # rung0 genuinely failed (2 dirty samples) -> a knee is learned
    assert m._knee[0] is not None
