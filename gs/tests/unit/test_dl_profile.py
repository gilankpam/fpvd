from pathlib import Path

import pytest

from fpvdgs.dynlink.profile import ProfileError, RadioProfile, load_profile

PROFILES = Path(__file__).resolve().parents[2] / "fpvdgs" / "dynlink" / "profiles"


def test_load_m8812eu2_from_json():
    p = load_profile("m8812eu2", [PROFILES])
    assert isinstance(p, RadioProfile)
    assert p.name == "BL-M8812EU2"
    assert p.chipset == "RTL8812EU"
    assert p.bandwidth_supported == (20, 40)
    # JSON string keys must have been coerced back to ints.
    assert p.snr_floor_dB[20][0] == 5.0
    assert p.data_rate_Mbps_LGI[40][5] == 108.0


def test_missing_profile_raises():
    with pytest.raises(ProfileError):
        load_profile("does_not_exist", [PROFILES])
