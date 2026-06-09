"""Validation rules. Link/overlap params are mutated ONLY via /link."""

from pathlib import Path

LINK_KEYS = {"channel", "width", "txpower", "region", "linkId", "beamforming", "wlans"}
CONFIG_TOP_KEYS = {"wfb", "drone", "dynamicLink", "pixelpilot"}   # link is excluded on purpose
DL_BANDWIDTHS = {20, 40}
DL_PROFILES_DIR = Path(__file__).resolve().parent / "dynlink" / "profiles"
ALL_TOP_KEYS = {"link"} | CONFIG_TOP_KEYS
VALID_WIDTHS = {10, 20, 40}              # 10 MHz = underclocked baseband (20 MHz modulation); matches the drone


class SchemaError(ValueError):
    pass


def validate_config_patch(sparse: dict) -> None:
    """A /config PATCH: any top-level key except `link`."""
    if "link" in sparse:
        raise SchemaError("link.* is read-only via /config; use /link")
    unknown = set(sparse) - CONFIG_TOP_KEYS
    if unknown:
        raise SchemaError(f"unknown config keys: {sorted(unknown)}")


def validate_link_patch(sparse: dict) -> None:
    """A /link PATCH: only `link.*`, only known link keys."""
    if set(sparse) - {"link"}:
        raise SchemaError("only link.* allowed via /link")
    link = sparse.get("link", {})
    if not isinstance(link, dict) or not link:
        raise SchemaError("link patch must be a non-empty object")
    unknown = set(link) - LINK_KEYS
    if unknown:
        raise SchemaError(f"unknown link keys: {sorted(unknown)}")


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
    dl = cfg.get("dynamicLink")
    if dl is not None:
        _validate_dynamic_link(dl)
    pp = cfg.get("pixelpilot")
    if pp is not None:
        _validate_pixelpilot(pp)


def _validate_beamforming(bf: dict) -> None:
    if not isinstance(bf, dict):
        raise SchemaError("link.beamforming must be an object")
    unknown = set(bf) - {"enabled"}
    if unknown:
        raise SchemaError(f"unknown link.beamforming keys: {sorted(unknown)}")
    if not isinstance(bf.get("enabled", False), bool):
        raise SchemaError("link.beamforming.enabled must be a bool")


def _validate_dynamic_link(dl: dict) -> None:
    max_mcs = dl.get("maxMcs", 5)
    if not isinstance(max_mcs, int) or not 0 <= max_mcs <= 7:
        raise SchemaError("dynamicLink.maxMcs must be an int in 0..7")
    bw = dl.get("bandwidth", 20)
    if bw not in DL_BANDWIDTHS:
        raise SchemaError(f"dynamicLink.bandwidth must be one of {sorted(DL_BANDWIDTHS)}")
    tx = dl.get("txpower", {}) or {}
    lo, hi = tx.get("min", 0), tx.get("max", 30)
    if lo > hi:
        raise SchemaError("dynamicLink.txpower.min must be <= max")
    port = dl.get("dronePort", 9999)
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("dynamicLink.dronePort must be an int in 1..65535")
    idr_port = dl.get("idrPort", 11223)
    if not isinstance(idr_port, int) or not 1 <= idr_port <= 65535:
        raise SchemaError("dynamicLink.idrPort must be an int in 1..65535")
    vid = dl.get("videoStreamId", "video")
    if not isinstance(vid, str) or not vid:
        raise SchemaError("dynamicLink.videoStreamId must be a non-empty string")
    profile = dl.get("radioProfile", "m8812eu2")
    if not (DL_PROFILES_DIR / f"{profile}.json").is_file():
        available = sorted(p.stem for p in DL_PROFILES_DIR.glob("*.json"))
        raise SchemaError(
            f"dynamicLink.radioProfile {profile!r} not found; available: {available}"
        )


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
