"""Validation rules. `link` is a normal mutable block in /config."""

LINK_KEYS = {"channel", "width", "txPowerDbm", "region", "linkId",
             "beamforming", "wlans"}
CONFIG_TOP_KEYS = {"link", "wfb", "drone", "dynamicLink", "pixelpilot",
                   "idrForward", "connectionMonitor"}
DYNAMIC_LINK_KEYS = {"enabled", "maxMcs", "radioProfile", "dronePort",
                     "selector", "smoothing", "flightlog", "rssiNorm",
                     "learnedPrior"}
DRONE_KEYS = {"host", "apiPort"}   # the drone's address; reused by HTTP/IDR/DL
SELECTOR_KEYS = {"probeViableThreshold", "probeFreshnessMs",
                 "promoteDebounceWindows", "videoDemotePer",
                 "emergencyFecPressure", "holdModesDownMs", "minBetweenChangesMs",
                 "starvationWindows", "lossWindows"}
SMOOTHING_KEYS = {"ewmaAlphaRssi", "ewmaAlphaFec", "ewmaAlphaBurst",
                  "starvationThresholdPps"}
LEARNED_PRIOR_KEYS = {"settleTicks", "viableLoss", "alphaTighten",
                      "alphaRelax", "minSamples", "recencyDecay"}
VALID_WIDTHS = {10, 20, 40}              # 10 MHz = underclocked baseband (20 MHz modulation); matches the drone


_bf_capable = None   # callable(cfg) -> bool; None => unknown => allow


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
    if not link.get("region"):
        raise SchemaError("link.region is required")
    if not link.get("channel"):
        raise SchemaError("link.channel is required")
    bf = link.get("beamforming")
    if bf is not None:
        _validate_beamforming(bf)
    if bf is not None and bf.get("enabled") and _bf_capable is not None:
        if not _bf_capable(cfg):
            raise SchemaError(
                "beamforming requires a card with a bf_monitor_conf node "
                "(GS driver lacks CONFIG_BEAMFORMING_MONITOR)")
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


def _validate_drone(dr: dict) -> None:
    if not isinstance(dr, dict):
        raise SchemaError("drone must be an object")
    host = dr.get("host", "10.5.0.10")
    if not isinstance(host, str) or not host:
        raise SchemaError("drone.host must be a non-empty string")
    port = dr.get("apiPort", 8080)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("drone.apiPort must be an int in 1..65535")


def _validate_beamforming(bf: dict) -> None:
    if not isinstance(bf, dict):
        raise SchemaError("link.beamforming must be an object")
    unknown = set(bf) - {"enabled"}
    if unknown:
        raise SchemaError(f"unknown link.beamforming keys: {sorted(unknown)}")
    if not isinstance(bf.get("enabled", False), bool):
        raise SchemaError("link.beamforming.enabled must be a bool")


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
    # radioProfile is a free identifier: it keys the learned-prior persistence
    # and the drone adapter-match warning.
    profile = dl.get("radioProfile", "m8812eu2")
    if not isinstance(profile, str) or not profile:
        raise SchemaError("dynamicLink.radioProfile must be a non-empty string")
    if not isinstance(dl.get("enabled", False), bool):
        raise SchemaError("dynamicLink.enabled must be a bool")
    sel = dl.get("selector")
    if sel is not None:
        _validate_block_keys("dynamicLink.selector", sel, SELECTOR_KEYS)
        for k in ("probeViableThreshold", "videoDemotePer",
                  "emergencyFecPressure"):
            _validate_prob(f"dynamicLink.selector.{k}", sel.get(k))
        for k in ("promoteDebounceWindows", "starvationWindows", "lossWindows"):
            _validate_pos_int(f"dynamicLink.selector.{k}", sel.get(k))
        for k in ("probeFreshnessMs", "holdModesDownMs", "minBetweenChangesMs"):
            _validate_non_neg_num(f"dynamicLink.selector.{k}", sel.get(k))
    sm = dl.get("smoothing")
    if sm is not None:
        _validate_block_keys("dynamicLink.smoothing", sm, SMOOTHING_KEYS)
        for k in ("ewmaAlphaRssi", "ewmaAlphaFec", "ewmaAlphaBurst"):
            _validate_alpha(f"dynamicLink.smoothing.{k}", sm.get(k))
        _validate_non_neg_num("dynamicLink.smoothing.starvationThresholdPps",
                              sm.get("starvationThresholdPps"))
    lp = dl.get("learnedPrior")
    if lp is not None:
        _validate_block_keys("dynamicLink.learnedPrior", lp, LEARNED_PRIOR_KEYS)
        _validate_pos_int("dynamicLink.learnedPrior.settleTicks", lp.get("settleTicks"))
        _validate_non_neg_num("dynamicLink.learnedPrior.minSamples", lp.get("minSamples"))
        _validate_prob("dynamicLink.learnedPrior.viableLoss", lp.get("viableLoss"))
        for k in ("alphaTighten", "alphaRelax", "recencyDecay"):
            _validate_alpha(f"dynamicLink.learnedPrior.{k}", lp.get(k))
    for sub in ("flightlog", "rssiNorm"):
        blk = dl.get(sub)
        if blk is not None:
            _validate_block_keys(f"dynamicLink.{sub}", blk, {"enabled"})
            if not isinstance(blk.get("enabled", True), bool):
                raise SchemaError(f"dynamicLink.{sub}.enabled must be a bool")


def _validate_block_keys(name: str, blk: dict, known: set) -> None:
    if not isinstance(blk, dict):
        raise SchemaError(f"{name} must be an object")
    unknown = set(blk) - known
    if unknown:
        raise SchemaError(f"unknown {name} keys: {sorted(unknown)}")


def _validate_prob(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))
                          or not 0.0 <= v <= 1.0):
        raise SchemaError(f"{name} must be a number in 0..1")


def _validate_alpha(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))
                          or not 0.0 < v <= 1.0):
        raise SchemaError(f"{name} must be a number in (0,1]")


def _validate_pos_int(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, int) or v <= 0):
        raise SchemaError(f"{name} must be a positive int")


def _validate_non_neg_num(name: str, v) -> None:
    if v is not None and (isinstance(v, bool) or not isinstance(v, (int, float))
                          or v < 0):
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
    for k in ("tunnelStaleS", "httpPollS", "httpTimeoutS", "evalIntervalS"):
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
    vs = pp.get("videoScale", 1.0)
    if isinstance(vs, bool) or not isinstance(vs, (int, float)) or vs <= 0:
        raise SchemaError("pixelpilot.videoScale must be a positive number")
    port = pp.get("rtpPort", 5600)
    if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("pixelpilot.rtpPort must be an int in 1..65535")
    jit = pp.get("rtpJitterMs", 1)
    if isinstance(jit, bool) or not isinstance(jit, int) or jit < 0:
        raise SchemaError("pixelpilot.rtpJitterMs must be a non-negative int")
    for key in ("screenMode", "bin", "configPath", "osdConfigPath", "codec"):
        val = pp.get(key)
        if val is not None and (not isinstance(val, str) or not val):
            raise SchemaError(f"pixelpilot.{key} must be a non-empty string")
    dvr = pp.get("dvr", {})
    if not isinstance(dvr, dict):
        raise SchemaError("pixelpilot.dvr must be an object")
    for key in ("framerate", "maxSizeMb", "reencBitrate", "reencFps"):
        v = dvr.get(key)
        if v is not None and (isinstance(v, bool) or not isinstance(v, int) or v <= 0):
            raise SchemaError(f"pixelpilot.dvr.{key} must be a positive int")
    for key in ("dir", "template", "mode", "reencCodec", "reencResolution"):
        val = dvr.get(key)
        if val is not None and (not isinstance(val, str) or not val):
            raise SchemaError(f"pixelpilot.dvr.{key} must be a non-empty string")
    for key in ("fmp4", "sequencedFiles", "osd"):
        if key in dvr and not isinstance(dvr[key], bool):
            raise SchemaError(f"pixelpilot.dvr.{key} must be a bool")
    env = pp.get("env", {})
    if not isinstance(env, dict) or not all(
            isinstance(k, str) and isinstance(v, str) for k, v in env.items()):
        raise SchemaError("pixelpilot.env must be a map of string to string")
    extra = pp.get("extraArgs", [])
    if not isinstance(extra, list) or not all(isinstance(a, str) for a in extra):
        raise SchemaError("pixelpilot.extraArgs must be a list of strings")


