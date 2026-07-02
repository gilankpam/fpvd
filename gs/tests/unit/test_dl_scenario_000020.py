"""Directional replay of the flight-000020 failure pattern (2026-07-02 spec).

Synthetic reconstruction of the three poisoning patterns from the real log:
interference bursts at good steady SNR, obstruction fades, and post-promote
margin flaps. Asserts the PROPERTIES the redesign must hold, not exact paths:
no long good-SNR suppressed pins, damper immune to cascades, fast fade
recovery."""

from fpvdgs.dynlink.policy import Policy, PolicyConfig
from fpvdgs.dynlink.signals import Signals


def _mk_policy(tmp_path):
    cfg = PolicyConfig()
    cfg.learned_prior.min_samples = 1.0
    cfg.learned_prior.recency_decay = 1.0  # deterministic: no confidence fade in-test
    cfg.learned_prior.persist_dir = str(tmp_path)
    cfg.flightlog.enabled = False
    return Policy(cfg, "replay20")


def _tick(p, ts, snr, snr_w, loss):
    s = Signals()
    s.timestamp = ts
    s.snr = snr
    s.snr_w = snr_w
    s.residual_loss_w = loss
    s.session = object()
    s.packet_rate_w = 1000.0
    return p.tick(s)


def test_interference_bursts_do_not_pin_the_link(tmp_path):
    """000020 pattern 1: repeated 0.3 s loss bursts at steady SNR 24 cascaded
    the ladder and ratcheted flap levels to 8 => 30 s pins. Now: bursts charge
    nothing; between bursts the selector must re-climb, and total time spent
    >2 rungs below the SNR-supported rung must stay bounded."""
    p = _mk_policy(tmp_path)
    ts = 1_000_000.0
    low_ticks = total = 0
    for i in range(6000):  # 10 min at 10 Hz
        ts += 0.1
        burst = i % 600 in (0, 1, 2)  # 0.3 s burst every 60 s
        _tick(p, ts, 24.0, 24.0, 0.3 if burst else 0.0)
        total += 1
        if p.leading.state.current_mcs <= 2:
            low_ticks += 1
    assert max(p.leading._flap_level.values(), default=0) <= 1
    assert low_ticks / total < 0.35  # 000020 spent 37% at MCS<=2 with SNR 23


def test_fade_recovery_within_two_seconds(tmp_path):
    """000020/tree pattern: SNR collapse demotes the ladder; after recovery the
    selector must snap back to the pre-fade rung within ~2 s (spec acceptance
    #3), not ladder-climb for 15 s."""
    p = _mk_policy(tmp_path)
    ts = 1_000_000.0
    for _ in range(400):  # climb + confirm at a high rung, SNR 28
        ts += 0.1
        _tick(p, ts, 28.0, 28.0, 0.0)
    pre_fade = p.leading.state.current_mcs
    assert pre_fade >= 4
    for _ in range(20):  # 2 s obstruction: raw SNR 8, EWMA lags down, loss
        ts += 0.1
        _tick(p, ts, 20.0, 8.0, 0.4)
    assert p.leading.state.current_mcs < pre_fade
    t_rec = ts
    while p.leading.state.current_mcs < pre_fade:
        ts += 0.1
        _tick(p, ts, 28.0, 28.0, 0.0)
        assert ts - t_rec < 2.0, "fade recovery slower than 2 s"


def test_margin_flap_learns_and_stops_repeating(tmp_path):
    """000020 pattern 3: promotes at SNR ~20 flapped repeatedly (41/55). Now the
    first flap plants the knee and further promotes at the same SNR are blocked
    — at most 2 flaps of the same rung at the same SNR in 5 minutes."""
    p = _mk_policy(tmp_path)
    ts = 1_000_000.0
    flaps = 0
    for _ in range(3000):
        ts += 0.1
        cur = p.leading.state.current_mcs
        # rung 3+ is unviable at SNR 20: loss whenever operating there
        loss = 0.2 if cur >= 3 else 0.0
        _tick(p, ts, 20.0, 20.0, loss)
        if p.leading.state.current_mcs < cur and cur >= 3:
            flaps += 1
    assert flaps <= 2
