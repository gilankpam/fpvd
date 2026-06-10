"""Validation rules for the unified config tree (validated at /apply time)."""

from pathlib import Path

DL_PROFILES_DIR = Path(__file__).resolve().parent / "dynlink" / "profiles"
VALID_WIDTHS = {10, 20, 40}              # 10 MHz = underclocked baseband (20 MHz modulation); matches the drone


class SchemaError(ValueError):
    pass


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
    rx = link.get("rxpower")
    if rx is not None and (isinstance(rx, bool) or not isinstance(rx, (int, float))
                           or not 0 <= rx <= 30):
        # dBm, mirrors the drone's link.txpower (0..30); null => driver auto.
        raise SchemaError("link.rxpower must be a number 0..30 (dBm) or null")
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
    if not isinstance(dl.get("enabled", False), bool):
        raise SchemaError("dynamicLink.enabled must be a bool")
    ctl = dl.get("controller", {}) or {}
    max_mcs = ctl.get("maxMcs", 5)
    if not isinstance(max_mcs, int) or not 0 <= max_mcs <= 7:
        raise SchemaError("dynamicLink.controller.maxMcs must be an int in 0..7")
    port = ctl.get("dronePort", 9999)
    if not isinstance(port, int) or not 1 <= port <= 65535:
        raise SchemaError("dynamicLink.controller.dronePort must be an int in 1..65535")
    profile = ctl.get("radioProfile", "m8812eu2")
    if not (DL_PROFILES_DIR / f"{profile}.json").is_file():
        available = sorted(p.stem for p in DL_PROFILES_DIR.glob("*.json"))
        raise SchemaError(
            f"dynamicLink.controller.radioProfile {profile!r} not found; available: {available}")


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


