"""Validation rules. Link/overlap params are mutated ONLY via /link."""

LINK_KEYS = {"channel", "width", "txpower", "region", "linkId", "beamforming", "wlans"}
CONFIG_TOP_KEYS = {"wfb", "drone"}      # link is excluded on purpose
ALL_TOP_KEYS = {"link"} | CONFIG_TOP_KEYS
VALID_WIDTHS = {20, 40}                  # 10 reserved for future


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
