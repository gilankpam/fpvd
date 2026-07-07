"""Validation rules. `link` is a normal mutable block in /config."""

from .wfb.cards import parse_cards

LINK_KEYS = {
    "channel",
    "width",
    "txPowerDbm",
    "region",
    "linkId",
    "beamforming",
    "cards",
    "serverAddress",
    # legacy overlay key, pre-dates `cards` (Phase 2 remote cards): a plain
    # list of local iface name strings, or "auto". `wfb.cards.parse_cards`
    # still consumes it as a back-compat fallback — keep accepting it here
    # (and never strip it in the tolerant loader) so old overlays/PATCHes
    # that only set `wlans` keep working. Deprecated; prefer `cards`.
    "wlans",
    "videoEncryption",
}
CONFIG_TOP_KEYS = {
    "link",
    "wfb",
    "drone",
    "dynamicLink",
    "pixelpilot",
    "idrForward",
    "connectionMonitor",
}
DYNAMIC_LINK_KEYS = {
    "enabled",
    "maxMcs",
    "dronePort",
    "selector",
    "smoothing",
    "flightlog",
    "learnedPrior",
    "tap",
    "probe",
}
DRONE_KEYS = {"host", "apiPort"}  # the drone's address; reused by HTTP/IDR/DL
SELECTOR_KEYS = {
    "videoDemotePer",
    "emergencyFecPressure",
    "holdModesDownMs",
    "minBetweenChangesMs",
    "starvationWindows",
    "snrPromoteMarginDb",
    "snrDemoteMarginDb",
    "flapBaseBackoffMs",
    "flapBackoffMult",
    "flapBackoffCapMs",
    "flapResetCleanDwellTicks",
    "trialWindowMs",
    "promoteDwellTicks",
    "promoteSlopeMin",
    "collapseDeltaDb",
    "snapbackRecoverMarginDb",
    "confirmTtlMs",
    "flapSnrReleaseDb",
    "flapDecayMs",
}
SMOOTHING_KEYS = {"ewmaAlphaRssi", "ewmaAlphaFec", "ewmaAlphaBurst", "starvationThresholdPps"}
LEARNED_PRIOR_KEYS = {
    "settleTicks",
    "viableLoss",
    "alphaTighten",
    "alphaRelax",
    "minSamples",
    "recencyDecay",
}
TAP_KEYS = frozenset({"enabled", "port", "staleMs", "captureRaw"})
PROBE_KEYS = frozenset({"enabled"})
CARD_KEYS = frozenset({"host", "iface", "sshUser", "sshPort", "sshKey", "txPowerDbm", "initScript"})
VALID_WIDTHS = {
    5,
    10,
    20,
    40,
}  # 5/10 MHz = underclocked baseband (20 MHz modulation); matches the drone
TX_SELECTOR_KEYS = frozenset({"rssiDeltaDb", "counterRelDelta", "counterAbsDelta"})


_bf_capable = None  # callable(cfg) -> bool; None => unknown => allow


def set_bf_capable(fn) -> None:
    global _bf_capable
    _bf_capable = fn


class SchemaError(ValueError):
    pass


def validate_config_patch(sparse: dict) -> None:
    """A /gs/config PATCH: any known top-level key, including `link`."""
    unknown = set(sparse) - CONFIG_TOP_KEYS
    if unknown:
        raise SchemaError(f"unknown config keys: {sorted(unknown)}")
    link = sparse.get("link")
    if link is not None:
        if not isinstance(link, dict):
            raise SchemaError("link must be an object")
        unknown_link = set(link) - LINK_KEYS
        if unknown_link:
            raise SchemaError(f"unknown link keys: {sorted(unknown_link)}")
    dl = sparse.get("dynamicLink")
    if dl is not None:
        # PATCH rejects unknown top-level dynamicLink keys; sub-block contents
        # (selector/smoothing ranges) are deferred to validate_effective on apply.
        if not isinstance(dl, dict):
            raise SchemaError("dynamicLink must be an object")
        unknown_dl = set(dl) - DYNAMIC_LINK_KEYS
        if unknown_dl:
            raise SchemaError(f"unknown dynamicLink keys: {sorted(unknown_dl)}")
    dr = sparse.get("drone")
    if dr is not None:
        if not isinstance(dr, dict):
            raise SchemaError("drone must be an object")
        unknown_dr = set(dr) - DRONE_KEYS
        if unknown_dr:
            raise SchemaError(f"unknown drone keys: {sorted(unknown_dr)}")


def validate_effective(cfg: dict) -> None:
    """Sanity-check the full effective config before rendering/applying."""
    link = cfg.get("link", {})
    width = link.get("width")
    if width is not None and width not in VALID_WIDTHS:
        raise SchemaError(f"link.width must be one of {sorted(VALID_WIDTHS)}")
    # 5 and 40 MHz are static widths: rejected under DL (5 MHz has too little
    # dynamic range; 40 MHz TX-power backoff is unvalidated at true 40 MHz
    # modulation). Mirrors the drone (drone/src/config/validate.cpp).
    if width in (5, 40) and bool((cfg.get("dynamicLink") or {}).get("enabled", False)):
        raise SchemaError("link.width 5/40 MHz requires dynamicLink.enabled=false")
    if not link.get("region"):
        raise SchemaError("link.region is required")
    if not link.get("channel"):
        raise SchemaError("link.channel is required")
    bf = link.get("beamforming")
    if bf is not None:
        _validate_beamforming(bf)
    cards = link.get("cards")
    if cards is not None and cards != "auto":
        _validate_cards(cards)
    _validate_single_remote_host(link)
    server_addr = link.get("serverAddress")
    if server_addr is not None and not isinstance(server_addr, str):
        raise SchemaError("link.serverAddress must be a string or null")
    if bf is not None and bf.get("enabled") and _bf_capable is not None:
        if not _bf_capable(cfg):
            raise SchemaError(
                "beamforming requires a card with a bf_monitor_conf node "
                "(GS driver lacks CONFIG_BEAMFORMING_MONITOR)"
            )
    dl = cfg.get("dynamicLink")
    if dl is not None:
        _validate_dynamic_link(dl)
    pp = cfg.get("pixelpilot")
    if pp is not None:
        _validate_pixelpilot(pp)
    idr = cfg.get("idrForward")
    if idr is not None:
        _validate_idr_forward(idr)
    cm = cfg.get("connectionMonitor")
    if cm is not None:
        _validate_connection_monitor(cm)
    dr = cfg.get("drone")
    if dr is not None:
        _validate_drone(dr)
    wfb = cfg.get("wfb")
    if wfb is not None:
        _validate_wfb(wfb)


def _validate_drone(dr: dict) -> None:
    if not isinstance(dr, dict):
        raise SchemaError("drone must be an object")
    host = dr.get("host", "10.5.0.10")
    if not isinstance(host, str) or not host:
        raise SchemaError("drone.host must be a non-empty string")
    port = dr.get("apiPort", 8080)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("drone.apiPort must be an int in 1..65535")


def _validate_wfb(wfb: dict) -> None:
    if not isinstance(wfb, dict):
        raise SchemaError("wfb must be an object")
    txsel = wfb.get("txSelector")
    if txsel is not None:
        _validate_block_keys("wfb.txSelector", txsel, TX_SELECTOR_KEYS)
        _validate_non_neg_num("wfb.txSelector.rssiDeltaDb", txsel.get("rssiDeltaDb"))
        _validate_prob("wfb.txSelector.counterRelDelta", txsel.get("counterRelDelta"))
        abs_delta = txsel.get("counterAbsDelta")
        if abs_delta is not None and (
            isinstance(abs_delta, bool) or not isinstance(abs_delta, int) or abs_delta < 0
        ):
            raise SchemaError("wfb.txSelector.counterAbsDelta must be a non-negative int")
    mav = wfb.get("mavlink")
    if mav is not None:
        if not isinstance(mav, dict):
            raise SchemaError("wfb.mavlink must be an object")
        _validate_mavlink_peer("wfb.mavlink.peer", mav.get("peer"))


def _validate_mavlink_peer(name: str, v) -> None:
    if not isinstance(v, str) or not v:
        raise SchemaError(f"{name} must be a non-empty string")
    scheme, sep, rest = v.partition("://")
    if not sep or scheme not in ("connect", "listen"):
        raise SchemaError(f"{name} must match connect://host:port or listen://host:port")
    host, sep2, port = rest.rpartition(":")
    if not sep2 or not host or not port.isdigit():
        raise SchemaError(f"{name} must match connect://host:port or listen://host:port")


def _validate_beamforming(bf: dict) -> None:
    if not isinstance(bf, dict):
        raise SchemaError("link.beamforming must be an object")
    unknown = set(bf) - {"enabled"}
    if unknown:
        raise SchemaError(f"unknown link.beamforming keys: {sorted(unknown)}")
    if not isinstance(bf.get("enabled", False), bool):
        raise SchemaError("link.beamforming.enabled must be a bool")


def _validate_cards(cards) -> None:
    if not isinstance(cards, list):
        raise SchemaError("link.cards must be a list or 'auto'")
    for c in cards:
        if isinstance(c, str):
            continue
        if not isinstance(c, dict):
            raise SchemaError("link.cards entries must be a string or an object")
        if "iface" not in c:
            raise SchemaError("link.cards object entries require 'iface'")
        unknown = set(c) - CARD_KEYS
        if unknown:
            raise SchemaError(f"unknown link.cards keys: {sorted(unknown)}")
        if "host" in c:
            host = c["host"]
            if not isinstance(host, str) or not host:
                raise SchemaError("link.cards.host must be a non-empty string")
        port = c.get("sshPort", 22)
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise SchemaError("link.cards.sshPort must be an int in 1..65535")
        ssh_user = c.get("sshUser", "root")
        if not isinstance(ssh_user, str) or not ssh_user:
            raise SchemaError("link.cards.sshUser must be a non-empty string")
        ssh_key = c.get("sshKey")
        if ssh_key is not None and not isinstance(ssh_key, str):
            raise SchemaError("link.cards.sshKey must be a string or absent")
        if "txPowerDbm" in c:
            txp = c["txPowerDbm"]
            if (
                txp is not None
                and txp != "off"
                and (isinstance(txp, bool) or not isinstance(txp, (int, float)))
            ):
                raise SchemaError("link.cards.txPowerDbm must be a number, 'off', or null")
        if "initScript" in c:
            s = c["initScript"]
            if s is not None and not isinstance(s, str):
                raise SchemaError("link.cards.initScript must be a string or null")


def _validate_single_remote_host(link: dict) -> None:
    # The engine derives ONE server_address from the first remote card's
    # host and uses it for every remote node — correct for a single remote
    # host, silently wrong for 2+ DISTINCT remote hosts (a second node would
    # be told to send video to the wrong GS source address -> video loss ->
    # GS reboots on sustained video loss). Per-node server_address is a
    # future enhancement; until then, reject a multi-remote-host config
    # outright rather than mis-wire it. Remote cards are always explicit
    # (never "auto"), so resolve_cards' auto-expansion isn't needed here.
    try:
        cards = parse_cards(link)
    except ValueError as exc:
        # parse_cards' own invariants (e.g. a remote entry hiding in the
        # legacy `wlans` list) — surface as a clean SchemaError rather than
        # an uncaught ValueError.
        raise SchemaError(str(exc)) from exc
    if cards == "auto":
        return
    hosts = {c.host for c in cards if c.host is not None}
    if len(hosts) > 1:
        raise SchemaError(
            "multiple remote-card hosts not yet supported (per-node server "
            "address pending); use at most one remote host"
        )


def _validate_dynamic_link(dl: dict) -> None:
    unknown = set(dl) - DYNAMIC_LINK_KEYS
    if unknown:
        raise SchemaError(f"unknown dynamicLink keys: {sorted(unknown)}")
    max_mcs = dl.get("maxMcs", 5)
    if not isinstance(max_mcs, int) or isinstance(max_mcs, bool) or not 0 <= max_mcs <= 7:
        raise SchemaError("dynamicLink.maxMcs must be an int in 0..7")
    port = dl.get("dronePort", 9999)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("dynamicLink.dronePort must be an int in 1..65535")
    tap = dl.get("tap")
    if tap is not None:
        if not isinstance(tap, dict):
            raise SchemaError("dynamicLink.tap must be an object")
        unknown_tap = set(tap) - TAP_KEYS
        if unknown_tap:
            raise SchemaError(f"unknown dynamicLink.tap keys: {sorted(unknown_tap)}")
        enabled = tap.get("enabled", True)
        if not isinstance(enabled, bool):
            raise SchemaError("dynamicLink.tap.enabled must be a bool")
        capture = tap.get("captureRaw", False)
        if not isinstance(capture, bool):
            raise SchemaError("dynamicLink.tap.captureRaw must be a bool")
        tap_port = tap.get("port", 8110)
        if (
            isinstance(tap_port, bool)
            or not isinstance(tap_port, int)
            or not 1024 <= tap_port <= 65535
        ):
            raise SchemaError("dynamicLink.tap.port must be an int in 1024..65535")
        stale = tap.get("staleMs", 500)
        if isinstance(stale, bool) or not isinstance(stale, (int, float)) or stale <= 0:
            raise SchemaError("dynamicLink.tap.staleMs must be > 0")
    probe = dl.get("probe")
    if probe is not None:
        if not isinstance(probe, dict):
            raise SchemaError("dynamicLink.probe must be an object")
        unknown_probe = set(probe) - PROBE_KEYS
        if unknown_probe:
            raise SchemaError(f"unknown dynamicLink.probe keys: {sorted(unknown_probe)}")
        if "enabled" in probe and not isinstance(probe["enabled"], bool):
            raise SchemaError("dynamicLink.probe.enabled must be a bool")
    if not isinstance(dl.get("enabled", False), bool):
        raise SchemaError("dynamicLink.enabled must be a bool")
    sel = dl.get("selector")
    if sel is not None:
        _validate_block_keys("dynamicLink.selector", sel, SELECTOR_KEYS)
        for k in ("videoDemotePer", "emergencyFecPressure"):
            _validate_prob(f"dynamicLink.selector.{k}", sel.get(k))
        for k in ("starvationWindows", "promoteDwellTicks"):
            _validate_pos_int(f"dynamicLink.selector.{k}", sel.get(k))
        for k in (
            "holdModesDownMs",
            "minBetweenChangesMs",
            "snrPromoteMarginDb",
            "snrDemoteMarginDb",
            "trialWindowMs",
            "collapseDeltaDb",
            "snapbackRecoverMarginDb",
            "confirmTtlMs",
            "flapSnrReleaseDb",
            "flapDecayMs",
        ):
            _validate_non_neg_num(f"dynamicLink.selector.{k}", sel.get(k))
        v = sel.get("promoteSlopeMin")
        if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))):
            raise SchemaError("dynamicLink.selector.promoteSlopeMin must be a number")
        # Invariant: the proactive-demote margin must exceed the promote margin or
        # the SNR-knee gates collapse to a single oscillating edge (no dead-band).
        # Fallbacks track SelectorConfig defaults; both are validated numeric above.
        pm = sel.get("snrPromoteMarginDb", 1.0)
        dm = sel.get("snrDemoteMarginDb", 1.5)
        if dm <= pm:
            raise SchemaError(
                "dynamicLink.selector.snrDemoteMarginDb must be > "
                "snrPromoteMarginDb (hysteresis dead-band)"
            )
    sm = dl.get("smoothing")
    if sm is not None:
        _validate_block_keys("dynamicLink.smoothing", sm, SMOOTHING_KEYS)
        for k in ("ewmaAlphaRssi", "ewmaAlphaFec", "ewmaAlphaBurst"):
            _validate_alpha(f"dynamicLink.smoothing.{k}", sm.get(k))
        _validate_non_neg_num(
            "dynamicLink.smoothing.starvationThresholdPps", sm.get("starvationThresholdPps")
        )
    lp = dl.get("learnedPrior")
    if lp is not None:
        _validate_block_keys("dynamicLink.learnedPrior", lp, LEARNED_PRIOR_KEYS)
        _validate_pos_int("dynamicLink.learnedPrior.settleTicks", lp.get("settleTicks"))
        _validate_non_neg_num("dynamicLink.learnedPrior.minSamples", lp.get("minSamples"))
        _validate_prob("dynamicLink.learnedPrior.viableLoss", lp.get("viableLoss"))
        for k in ("alphaTighten", "alphaRelax", "recencyDecay"):
            _validate_alpha(f"dynamicLink.learnedPrior.{k}", lp.get(k))
    fl = dl.get("flightlog")
    if fl is not None:
        _validate_block_keys("dynamicLink.flightlog", fl, {"enabled"})
        if not isinstance(fl.get("enabled", True), bool):
            raise SchemaError("dynamicLink.flightlog.enabled must be a bool")


def _validate_block_keys(name: str, blk: dict, known: set) -> None:
    if not isinstance(blk, dict):
        raise SchemaError(f"{name} must be an object")
    unknown = set(blk) - known
    if unknown:
        raise SchemaError(f"unknown {name} keys: {sorted(unknown)}")


def _validate_prob(name: str, v) -> None:
    if v is not None and (
        isinstance(v, bool) or not isinstance(v, (int, float)) or not 0.0 <= v <= 1.0
    ):
        raise SchemaError(f"{name} must be a number in 0..1")


def _validate_alpha(name: str, v) -> None:
    if v is not None and (
        isinstance(v, bool) or not isinstance(v, (int, float)) or not 0.0 < v <= 1.0
    ):
        raise SchemaError(f"{name} must be a number in (0,1]")


def _validate_pos_int(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, int) or v <= 0):
        raise SchemaError(f"{name} must be a positive int")


def _validate_non_neg_num(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float)) or v < 0):
        raise SchemaError(f"{name} must be a non-negative number")


def _validate_idr_forward(idr: dict) -> None:
    if not isinstance(idr.get("enabled", True), bool):
        raise SchemaError("idrForward.enabled must be a bool")
    port = idr.get("port", 11223)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("idrForward.port must be an int in 1..65535")


def _validate_connection_monitor(cm: dict) -> None:
    if not isinstance(cm, dict):
        raise SchemaError("connectionMonitor must be an object")
    if not isinstance(cm.get("enabled", True), bool):
        raise SchemaError("connectionMonitor.enabled must be a bool")
    for k in ("tunnelStaleS", "httpPollS", "httpTimeoutS", "evalIntervalS", "disconnectGraceS"):
        v = cm.get(k)
        if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float)) or v <= 0):
            raise SchemaError(f"connectionMonitor.{k} must be a positive number")
    _validate_pos_int("connectionMonitor.httpFailCount", cm.get("httpFailCount"))
    # Invariant: the heartbeat's own HTTP return traffic keeps the tunnel 'fresh',
    # so tunnelStaleS must exceed httpPollS or a quiet healthy link false-disconnects.
    # Fallbacks must track ConnectionMonitorConfig defaults; both values are
    # already validated numeric above, so a plain comparison is safe.
    stale = cm.get("tunnelStaleS", 4.0)
    poll = cm.get("httpPollS", 1.5)
    if stale <= poll:
        raise SchemaError("connectionMonitor.tunnelStaleS must be > httpPollS")


def _validate_pixelpilot(pp: dict) -> None:
    if not isinstance(pp.get("enabled", True), bool):
        raise SchemaError("pixelpilot.enabled must be a bool")
    port = pp.get("rtpPort", 5600)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("pixelpilot.rtpPort must be an int in 1..65535")
    for key in ("screenMode", "bin", "configPath", "osdConfigPath", "codec"):
        val = pp.get(key)
        if val is not None and (not isinstance(val, str) or not val):
            raise SchemaError(f"pixelpilot.{key} must be a non-empty string")
    dvr = pp.get("dvr", {})
    if not isinstance(dvr, dict):
        raise SchemaError("pixelpilot.dvr must be an object")
    for key in ("dir", "template"):
        val = dvr.get(key)
        if val is not None and (not isinstance(val, str) or not val):
            raise SchemaError(f"pixelpilot.dvr.{key} must be a non-empty string")
    for key in ("fmp4", "sequencedFiles"):
        if key in dvr and not isinstance(dvr[key], bool):
            raise SchemaError(f"pixelpilot.dvr.{key} must be a bool")
    env = pp.get("env", {})
    if not isinstance(env, dict) or not all(
        isinstance(k, str) and isinstance(v, str) for k, v in env.items()
    ):
        raise SchemaError("pixelpilot.env must be a map of string to string")
    extra = pp.get("extraArgs", [])
    if not isinstance(extra, list) or not all(isinstance(a, str) for a in extra):
        raise SchemaError("pixelpilot.extraArgs must be a list of strings")
