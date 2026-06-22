"""Tests for EIRP RSSI normalization math (GS RSSI-norm design §Approach)."""

from __future__ import annotations

from fpvdgs.dynlink.signals import RssiNormConfig, normalize_rssi

CURVE = (29, 28, 25, 23, 19, 19, 19, 19)


def _bound():
    return RssiNormConfig(enabled=True, p_ref_dbm=29, tx_power_dbm_by_mcs=CURVE)


def test_default_is_identity_until_bound():
    cfg = RssiNormConfig()
    assert cfg.enabled is False
    assert cfg.tx_power_dbm_by_mcs == ()
    # identity: returns raw RSSI unchanged
    assert normalize_rssi(-70.0, 5, cfg) == -70.0


def test_normalize_adds_pref_minus_curve_per_mcs():
    cfg = _bound()
    # MCS0: curve 29, offset 0 → unchanged.
    assert normalize_rssi(-60.0, 0, cfg) == -60.0
    # MCS5: curve 19, offset +10 → raised 10 dB.
    assert normalize_rssi(-70.0, 5, cfg) == -60.0
    # MCS3: curve 23, offset +6.
    assert normalize_rssi(-70.0, 3, cfg) == -64.0


def test_normalize_clamps_mcs_out_of_range():
    cfg = _bound()
    # mcs > 7 clamps to 7 (curve 19, offset +10).
    assert normalize_rssi(-70.0, 9, cfg) == -60.0
    # mcs < 0 clamps to 0 (curve 29, offset 0).
    assert normalize_rssi(-60.0, -3, cfg) == -60.0


def test_normalize_identity_when_disabled():
    cfg = RssiNormConfig(enabled=False)
    assert normalize_rssi(-70.0, 5, cfg) == -70.0


def test_normalize_none_safe():
    cfg = _bound()
    assert normalize_rssi(None, 5, cfg) is None
    assert normalize_rssi(-70.0, None, cfg) == -70.0
