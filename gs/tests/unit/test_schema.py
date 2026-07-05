import pytest

from fpvdgs import schema
from fpvdgs.schema import (
    SchemaError,
    validate_config_patch,
    validate_effective,
)


def test_config_patch_accepts_link():
    schema.validate_config_patch({"link": {"channel": 100}})  # no raise


def test_config_patch_rejects_unknown_link_key():
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"link": {"bogus": 1}})


def test_config_patch_allows_non_link():
    validate_config_patch({"wfb": {"mavlink": {"peer": "connect://127.0.0.1:14550"}}})


def test_config_patch_rejects_unknown_top_level():
    with pytest.raises(SchemaError):
        validate_config_patch({"bogus": 1})


def test_validate_effective_checks_width_domain():
    with pytest.raises(SchemaError):
        validate_effective({"link": {"channel": 132, "width": 80, "region": "US"}})


def test_validate_effective_accepts_10mhz():
    # 10 MHz is supported (underclocked); must not raise.
    validate_effective({"link": {"channel": 132, "width": 10, "region": "US"}})


def test_validate_effective_rejects_40mhz_with_dynamic_link():
    with pytest.raises(SchemaError):
        validate_effective(
            {
                "link": {"channel": 132, "width": 40, "region": "US"},
                "dynamicLink": {"enabled": True},
            }
        )


def test_validate_effective_accepts_40mhz_when_dynamic_link_off():
    validate_effective(
        {
            "link": {"channel": 132, "width": 40, "region": "US"},
            "dynamicLink": {"enabled": False},
        }
    )


def test_validate_effective_accepts_10mhz_with_dynamic_link():
    validate_effective(
        {
            "link": {"channel": 132, "width": 10, "region": "US"},
            "dynamicLink": {"enabled": True},
        }
    )


def test_validate_effective_ok():
    validate_effective(
        {
            "link": {
                "channel": 132,
                "width": 40,
                "txPowerDbm": 19,
                "region": "US",
                "linkId": 7669206,
                "beamforming": {"enabled": False},
                "wlans": "auto",
            },
            "wfb": {"profile": "gs", "mavlink": {"peer": "connect://127.0.0.1:14550"}, "raw": {}},
            "drone": {"endpoint": "http://10.5.0.10:8080"},
        }
    )


def _eff(**dl):
    base = {
        "link": {"channel": 132, "width": 40, "region": "US"},
        "dynamicLink": {
            "enabled": False,
            "maxMcs": 5,
            "dronePort": 9999,
        },
    }
    base["dynamicLink"].update(dl)
    return base


def test_config_patch_allows_dynamiclink():
    schema.validate_config_patch({"dynamicLink": {"enabled": True}})  # no raise


def test_effective_accepts_valid_dynamiclink():
    schema.validate_effective(_eff())  # no raise


def test_effective_rejects_bad_max_mcs():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(maxMcs=9))


def test_effective_accepts_snr_margins():
    schema.validate_effective(
        _eff(selector={"snrPromoteMarginDb": 1.0, "snrDemoteMarginDb": 2.0})
    )  # no raise


def test_effective_rejects_negative_snr_margin():
    with pytest.raises(SchemaError):
        schema.validate_effective(_eff(selector={"snrPromoteMarginDb": -1.0}))


def test_effective_rejects_demote_margin_not_above_promote():
    # demote <= promote collapses the dead-band -> re-creates the knife-edge.
    with pytest.raises(SchemaError):
        schema.validate_effective(
            _eff(selector={"snrPromoteMarginDb": 1.5, "snrDemoteMarginDb": 1.5})
        )


def test_idr_forward_validates():
    schema.validate_effective(
        {"link": {"channel": 1, "region": "US"}, "idrForward": {"enabled": True, "port": 11223}}
    )
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(
            {"link": {"channel": 1, "region": "US"}, "idrForward": {"enabled": True, "port": 0}}
        )


def test_config_patch_accepts_pixelpilot():
    # should not raise
    schema.validate_config_patch({"pixelpilot": {"screenMode": "1280x720@60"}})


def test_validate_effective_accepts_pixelpilot_block():
    cfg = {
        "link": {"channel": 132, "width": 40, "region": "US"},
        "pixelpilot": {
            "enabled": True,
            "screenMode": "1920x1080@60",
            "rtpPort": 5600,
            "codec": "h265",
            "env": {},
            "dvr": {"dir": "/media/dvr"},
            "extraArgs": [],
        },
    }
    schema.validate_effective(cfg)  # no raise


def test_validate_effective_rejects_bad_pixelpilot():
    base = {"link": {"channel": 132, "width": 40, "region": "US"}}
    for bad in (
        {"enabled": "yes"},
        {"screenMode": ""},
        {"extraArgs": "not-a-list"},
        {"extraArgs": [1, 2]},
        {"rtpPort": 0},
        {"rtpPort": 70000},
        {"codec": ""},
        {"dvr": {"dir": ""}},
        {"dvr": {"fmp4": "yes"}},
        {"env": {"A": 1}},
        {"env": "x"},
    ):
        with pytest.raises(schema.SchemaError):
            schema.validate_effective({**base, "pixelpilot": bad})


def test_shipped_defaults_include_pixelpilot_and_validate():
    from fpvdgs.config_defaults import default_config

    cfg = default_config()
    assert "pixelpilot" in cfg
    assert cfg["pixelpilot"]["enabled"] is True
    schema.validate_effective(cfg)  # no raise


def test_beamforming_enabled_bool_ok():
    from fpvdgs import schema

    # Shape-only check: pin the capability hook to "unknown" so a probe left
    # registered by a prior build_app test doesn't shell out / reject here.
    schema.set_bf_capable(None)
    cfg = {"link": {"region": "US", "channel": 132, "beamforming": {"enabled": True}}}
    schema.validate_effective(cfg)  # must not raise


def test_beamforming_enabled_must_be_bool():
    from fpvdgs import schema

    cfg = {"link": {"region": "US", "channel": 132, "beamforming": {"enabled": "yes"}}}
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)


def test_beamforming_rejects_unknown_subkey():
    from fpvdgs import schema

    cfg = {
        "link": {"region": "US", "channel": 132, "beamforming": {"enabled": True, "remoteMac": "x"}}
    }
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(cfg)


def test_validate_effective_accepts_drone_block():
    schema.validate_effective(
        {"link": {"channel": 132, "region": "US"}, "drone": {"host": "10.5.0.10", "apiPort": 8080}}
    )


def test_drone_host_empty_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective({"link": {"channel": 132, "region": "US"}, "drone": {"host": ""}})


def test_drone_apiport_out_of_range_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(
            {"link": {"channel": 132, "region": "US"}, "drone": {"apiPort": 0}}
        )


def test_patch_rejects_unknown_drone_key():
    # the old drone.endpoint key is now unknown -> rejected on PATCH
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"drone": {"endpoint": "http://x:8080"}})


def test_patch_accepts_known_drone_keys():
    schema.validate_config_patch({"drone": {"host": "10.5.0.10", "apiPort": 8080}})


def test_enable_bf_on_incapable_card_rejected():
    schema.set_bf_capable(lambda cfg: False)
    try:
        with pytest.raises(schema.SchemaError):
            schema.validate_effective(
                {"link": {"channel": 1, "region": "US", "beamforming": {"enabled": True}}}
            )
    finally:
        schema.set_bf_capable(lambda cfg: True)


def test_enable_bf_on_capable_card_ok():
    schema.set_bf_capable(lambda cfg: True)
    try:
        schema.validate_effective(
            {"link": {"channel": 1, "region": "US", "beamforming": {"enabled": True}}}
        )
    finally:
        schema.set_bf_capable(None)


def _dl(**over):
    base = {"enabled": True, "maxMcs": 5, "dronePort": 9999}
    base.update(over)
    return {"link": {"channel": 132, "region": "US", "width": 20}, "dynamicLink": base}


def test_validate_effective_accepts_flat_dynamic_link():
    schema.validate_effective(
        _dl(
            selector={"videoDemotePer": 0.05},
            smoothing={"ewmaAlphaRssi": 0.3},
            flightlog={"enabled": True},
        )
    )  # no raise


def test_selector_probability_out_of_range_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(selector={"videoDemotePer": 1.5}))


def test_smoothing_alpha_out_of_range_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(smoothing={"ewmaAlphaRssi": 0}))


def test_patch_rejects_unknown_dynamic_link_subkey():
    with pytest.raises(schema.SchemaError):
        schema.validate_config_patch({"dynamicLink": {"bogusKnob": 1}})


def test_patch_accepts_known_dynamic_link_keys():
    schema.validate_config_patch({"dynamicLink": {"selector": {}, "maxMcs": 4}})


def test_smoothing_alpha_above_one_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(smoothing={"ewmaAlphaRssi": 1.5}))


def test_selector_non_dict_rejected():
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(selector=0))


def test_selector_loss_windows_is_unknown_key_rejected():
    # lossWindows was removed (loss-demote is always single-window); it is now
    # an unknown selector key.
    with pytest.raises(schema.SchemaError):
        schema.validate_effective(_dl(selector={"lossWindows": 2}))


def test_learned_prior_block_accepted_in_patch():
    validate_config_patch(
        {"dynamicLink": {"learnedPrior": {"settleTicks": 8, "alphaTighten": 0.4}}}
    )  # no raise


def test_learned_prior_unknown_key_rejected():
    with pytest.raises(SchemaError, match="learnedPrior"):
        validate_effective(_dl(learnedPrior={"bogus": 1}))


def test_learned_prior_value_ranges_validated():
    # all valid defaults — must not raise
    validate_effective(
        _dl(
            learnedPrior={
                "settleTicks": 5,
                "viableLoss": 0.05,
                "alphaTighten": 0.25,
                "alphaRelax": 0.05,
                "minSamples": 8,
                "recencyDecay": 0.9995,
            }
        )
    )
    with pytest.raises(SchemaError):
        validate_effective(_dl(learnedPrior={"settleTicks": 0}))  # pos int required
    with pytest.raises(SchemaError):
        validate_effective(_dl(learnedPrior={"alphaTighten": 1.5}))  # (0,1]
    with pytest.raises(SchemaError):
        validate_effective(_dl(learnedPrior={"viableLoss": 2.0}))  # 0..1


def test_connection_monitor_accepts_shipped_defaults():
    from fpvdgs.config_defaults import default_config

    validate_effective(default_config())  # includes connectionMonitor; must pass


def test_connection_monitor_invariant_rejects_stale_le_poll():
    from fpvdgs.config_defaults import default_config

    cfg = default_config()
    cfg["connectionMonitor"]["tunnelStaleS"] = 1.0
    cfg["connectionMonitor"]["httpPollS"] = 1.5  # stale must be > poll
    with pytest.raises(SchemaError):
        validate_effective(cfg)


def test_connection_monitor_rejects_bad_fail_count():
    from fpvdgs.config_defaults import default_config

    cfg = default_config()
    cfg["connectionMonitor"]["httpFailCount"] = 0  # must be a positive int
    with pytest.raises(SchemaError):
        validate_effective(cfg)


def test_config_patch_accepts_connection_monitor():
    validate_config_patch({"connectionMonitor": {"enabled": False}})  # no raise


def test_connection_monitor_tolerates_unknown_keys():
    # Leniency is load-bearing: a stale/removed knob in an on-disk config must
    # NOT brick boot (the loader doesn't deep-strip connectionMonitor subkeys).
    validate_effective(
        {
            "link": {"channel": 132, "region": "US"},
            "connectionMonitor": {"enabled": True, "futureKnob": 99},
        }
    )  # must not raise


def test_config_patch_accepts_video_encryption():
    schema.validate_config_patch({"link": {"videoEncryption": False}})  # no raise


def test_default_config_video_encryption_true():
    from fpvdgs.config_defaults import default_config

    assert default_config()["link"]["videoEncryption"] is True


def test_connection_monitor_rejects_bad_grace():
    from fpvdgs.config_defaults import default_config

    cfg = default_config()
    cfg["connectionMonitor"]["disconnectGraceS"] = 0  # must be a positive number
    with pytest.raises(SchemaError):
        validate_effective(cfg)


def test_selector_accepts_new_knobs_rejects_probe_knobs():
    schema.validate_effective(
        _dl(selector={"trialWindowMs": 8000, "collapseDeltaDb": 3.0})
    )  # must not raise
    with pytest.raises(SchemaError):
        schema.validate_effective(_dl(selector={"probeViableThreshold": 0.9}))


def test_tap_block_valid_patch_accepted():
    from fpvdgs import schema

    schema.validate_config_patch(
        {
            "dynamicLink": {
                "tap": {"enabled": False, "port": 9000, "staleMs": 250, "captureRaw": True}
            }
        }
    )


def test_tap_unknown_key_rejected():
    import pytest

    from fpvdgs import schema

    with pytest.raises(schema.SchemaError):
        schema._validate_dynamic_link({"tap": {"bogus": 1}})


def test_tap_port_and_stale_range_rejected():
    import pytest

    from fpvdgs import schema

    with pytest.raises(schema.SchemaError):
        schema._validate_dynamic_link({"tap": {"port": 80}})
    with pytest.raises(schema.SchemaError):
        schema._validate_dynamic_link({"tap": {"staleMs": 0}})


def test_tap_bool_fields_type_checked():
    import pytest

    from fpvdgs import schema

    with pytest.raises(schema.SchemaError):
        schema._validate_dynamic_link({"tap": {"enabled": "false"}})
    with pytest.raises(schema.SchemaError):
        schema._validate_dynamic_link({"tap": {"captureRaw": 1}})


def test_wfb_engine_key_is_gone_from_defaults_and_not_required():
    from fpvdgs.config_defaults import default_config
    from fpvdgs.schema import validate_effective

    cfg = default_config()
    assert "engine" not in cfg["wfb"]
    validate_effective(cfg)  # no engine key, still valid


def test_wfb_tx_selector_valid_accepted():
    schema.validate_effective(
        {
            "link": {"channel": 132, "region": "US"},
            "wfb": {
                "txSelector": {
                    "rssiDeltaDb": 3,
                    "counterRelDelta": 0.1,
                    "counterAbsDelta": 3,
                }
            },
        }
    )


def test_wfb_tx_selector_rejects_bad_rssi_delta():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {"channel": 132, "region": "US"},
                "wfb": {"txSelector": {"rssiDeltaDb": -1}},
            }
        )


def test_wfb_tx_selector_rejects_out_of_range_counter_rel_delta():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {"channel": 132, "region": "US"},
                "wfb": {"txSelector": {"counterRelDelta": 1.5}},
            }
        )


def test_wfb_tx_selector_rejects_negative_counter_abs_delta():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {"channel": 132, "region": "US"},
                "wfb": {"txSelector": {"counterAbsDelta": -1}},
            }
        )


def test_wfb_tx_selector_rejects_unknown_key():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {"channel": 132, "region": "US"},
                "wfb": {"txSelector": {"bogus": 1}},
            }
        )


def test_wfb_mavlink_peer_null_rejected():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {"channel": 132, "region": "US"},
                "wfb": {"mavlink": {"peer": None}},
            }
        )


def test_wfb_mavlink_peer_missing_rejected():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {"channel": 132, "region": "US"},
                "wfb": {"mavlink": {}},
            }
        )


def test_wfb_mavlink_peer_malformed_rejected():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {"channel": 132, "region": "US"},
                "wfb": {"mavlink": {"peer": "http://127.0.0.1:14550"}},
            }
        )
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {"channel": 132, "region": "US"},
                "wfb": {"mavlink": {"peer": "connect://127.0.0.1"}},
            }
        )


def test_wfb_mavlink_peer_valid_schemes_accepted():
    schema.validate_effective(
        {
            "link": {"channel": 132, "region": "US"},
            "wfb": {"mavlink": {"peer": "connect://127.0.0.1:14550"}},
        }
    )
    schema.validate_effective(
        {
            "link": {"channel": 132, "region": "US"},
            "wfb": {"mavlink": {"peer": "listen://0.0.0.0:14550"}},
        }
    )


def test_wfb_no_mavlink_block_is_allowed():
    # wfb.mavlink is optional at the block level; only required WHEN present.
    schema.validate_effective({"link": {"channel": 132, "region": "US"}, "wfb": {}})


# ---- link.cards / link.serverAddress (wfb.cards migration) ----------------


def test_config_patch_accepts_cards_and_server_address():
    schema.validate_config_patch({"link": {"cards": ["wlan0"]}})  # no raise
    schema.validate_config_patch({"link": {"serverAddress": "10.0.0.5"}})  # no raise


def test_config_patch_still_accepts_legacy_wlans():
    # Deprecated but must keep working — old clients/overlays only know it.
    schema.validate_config_patch({"link": {"wlans": ["wlan0"]}})  # no raise


def test_validate_effective_accepts_string_and_object_cards():
    schema.validate_effective(
        {
            "link": {
                "channel": 132,
                "region": "US",
                "cards": [
                    "wlan0",
                    {"host": "192.168.1.10", "iface": "wlan1", "txPowerDbm": 20},
                ],
            }
        }
    )


def test_validate_effective_accepts_cards_auto():
    schema.validate_effective({"link": {"channel": 132, "region": "US", "cards": "auto"}})


def test_validate_effective_rejects_card_object_without_iface():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {"link": {"channel": 132, "region": "US", "cards": [{"host": "x"}]}}
        )


def test_validate_effective_rejects_unknown_card_key():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {"link": {"channel": 132, "region": "US", "cards": [{"iface": "w", "bogus": 1}]}}
        )


def test_validate_effective_rejects_non_string_server_address():
    with pytest.raises(SchemaError):
        schema.validate_effective({"link": {"channel": 132, "region": "US", "serverAddress": 5}})


def test_validate_effective_accepts_null_server_address():
    schema.validate_effective({"link": {"channel": 132, "region": "US", "serverAddress": None}})


def test_defaults_have_cards_and_server_address_not_wlans():
    from fpvdgs.config_defaults import default_config

    link = default_config()["link"]
    assert link["cards"] == "auto"
    assert link["serverAddress"] is None
    assert "wlans" not in link


# ---- link.cards remote-field hardening (Task 1 review fold-in) ------------


def test_validate_effective_accepts_full_remote_card():
    schema.validate_effective(
        {
            "link": {
                "channel": 132,
                "region": "US",
                "cards": [
                    {
                        "host": "192.168.1.10",
                        "iface": "wlan0",
                        "sshUser": "root",
                        "sshPort": 2222,
                        "sshKey": "/root/.ssh/id_ed25519",
                        "txPowerDbm": 20,
                    }
                ],
            }
        }
    )


def test_validate_effective_accepts_txpower_off_and_null():
    schema.validate_effective(
        {
            "link": {
                "channel": 132,
                "region": "US",
                "cards": [
                    {"host": "192.168.1.10", "iface": "wlan0", "txPowerDbm": "off"},
                    {"host": "192.168.1.10", "iface": "wlan1", "txPowerDbm": None},
                ],
            }
        }
    )


def test_validate_effective_rejects_sshport_out_of_range():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {
                    "channel": 132,
                    "region": "US",
                    "cards": [{"host": "x", "iface": "w", "sshPort": 99999}],
                }
            }
        )


def test_validate_effective_rejects_sshport_non_int():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {
                    "channel": 132,
                    "region": "US",
                    "cards": [{"host": "x", "iface": "w", "sshPort": "x"}],
                }
            }
        )


def test_validate_effective_rejects_empty_host():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {"link": {"channel": 132, "region": "US", "cards": [{"host": "", "iface": "w"}]}}
        )


def test_validate_effective_rejects_non_string_host():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {"link": {"channel": 132, "region": "US", "cards": [{"host": 5, "iface": "w"}]}}
        )


def test_validate_effective_rejects_empty_ssh_user():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {
                    "channel": 132,
                    "region": "US",
                    "cards": [{"host": "x", "iface": "w", "sshUser": ""}],
                }
            }
        )


def test_validate_effective_rejects_non_string_ssh_key():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {
                    "channel": 132,
                    "region": "US",
                    "cards": [{"host": "x", "iface": "w", "sshKey": 5}],
                }
            }
        )


def test_validate_effective_rejects_bad_txpower_string():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {
                    "channel": 132,
                    "region": "US",
                    "cards": [{"host": "x", "iface": "w", "txPowerDbm": "on"}],
                }
            }
        )


def test_validate_effective_rejects_bool_txpower():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {
                    "channel": 132,
                    "region": "US",
                    "cards": [{"host": "x", "iface": "w", "txPowerDbm": True}],
                }
            }
        )


def test_validate_effective_accepts_string_init_script():
    schema.validate_effective(
        {
            "link": {
                "channel": 132,
                "region": "US",
                "cards": [
                    {
                        "host": "x",
                        "iface": "w",
                        "initScript": "iw phy phy0 interface add wlan0 type monitor || true",
                    }
                ],
            }
        }
    )


def test_validate_effective_accepts_null_init_script():
    schema.validate_effective(
        {
            "link": {
                "channel": 132,
                "region": "US",
                "cards": [{"host": "x", "iface": "w", "initScript": None}],
            }
        }
    )


def test_validate_effective_rejects_non_string_init_script():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {
                    "channel": 132,
                    "region": "US",
                    "cards": [{"host": "x", "iface": "w", "initScript": 123}],
                }
            }
        )


# ---- link.cards single-remote-host guard (final-review fold-in) -----------
#
# The engine derives ONE server_address from the first remote card's host and
# uses it for every remote node. Correct for a single remote host; silently
# wrong for 2+ DISTINCT remote hosts (a second node gets told to send video
# to the wrong GS source address -> video loss -> GS reboots on sustained
# video loss). Per-node server_address is a future enhancement; until then,
# reject multi-remote-host configs outright.


def test_validate_effective_accepts_two_remote_cards_same_host():
    schema.validate_effective(
        {
            "link": {
                "channel": 132,
                "region": "US",
                "cards": [
                    {"host": "192.168.1.10", "iface": "wlan0"},
                    {"host": "192.168.1.10", "iface": "wlan1"},
                ],
            }
        }
    )


def test_validate_effective_rejects_two_remote_cards_different_hosts():
    with pytest.raises(SchemaError):
        schema.validate_effective(
            {
                "link": {
                    "channel": 132,
                    "region": "US",
                    "cards": [
                        {"host": "192.168.1.10", "iface": "wlan0"},
                        {"host": "192.168.1.11", "iface": "wlan1"},
                    ],
                }
            }
        )


def test_validate_effective_accepts_one_remote_host_with_local_cards():
    schema.validate_effective(
        {
            "link": {
                "channel": 132,
                "region": "US",
                "cards": [
                    "wlan0",
                    "wlan1",
                    {"host": "192.168.1.10", "iface": "wlan2"},
                ],
            }
        }
    )


def test_validate_effective_accepts_cards_auto_with_remote_host_guard():
    # "auto" only ever auto-detects local NICs, so zero remote hosts.
    schema.validate_effective({"link": {"channel": 132, "region": "US", "cards": "auto"}})


def test_validate_effective_accepts_legacy_wlans_with_remote_host_guard():
    # Legacy wlans is local-only (parse_cards raises if it ever isn't), so
    # zero remote hosts; the guard must not trip on it.
    schema.validate_effective(
        {"link": {"channel": 132, "region": "US", "wlans": ["wlan0", "wlan1"]}}
    )
