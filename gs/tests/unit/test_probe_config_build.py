from fpvdgs.probe.config_build import (
    PROBE_BLACKOUT_WINDOWS,
    PROBE_EWMA_ALPHA,
    PROBE_PORT,
    PROBE_RX_L,
    make_probe_snapshot,
)


def _eff(**dl):
    return {"link": {"linkId": 7669206, "wlans": ["wlan0"]}, "dynamicLink": {"enabled": True, **dl}}


def test_snapshot_uses_frozen_probe_constants():
    snap = make_probe_snapshot(_eff())
    assert snap["port"] == PROBE_PORT
    assert snap["rxL"] == PROBE_RX_L == 50
    assert snap["ewmaAlpha"] == PROBE_EWMA_ALPHA
    assert snap["blackoutWindows"] == PROBE_BLACKOUT_WINDOWS
    assert snap["linkId"] == 7669206
    assert snap["wlans"] == ["wlan0"]


def test_snapshot_ignores_orphaned_probe_block():
    # A stale top-level `probe` block (hand-added on an old device) is no
    # longer honored — the knobs are frozen constants now.
    eff = _eff()
    eff["probe"] = {"rxL": 800, "ewmaAlpha": 0.9, "blackoutWindows": 99}
    snap = make_probe_snapshot(eff)
    assert snap["rxL"] == 50
    assert snap["ewmaAlpha"] == PROBE_EWMA_ALPHA
    assert snap["blackoutWindows"] == PROBE_BLACKOUT_WINDOWS
