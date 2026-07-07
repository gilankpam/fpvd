"""Live radio retune via `iw` on the running monitor interfaces.

Changing the card's regulatory domain / channel / width / txpower with `iw` on
an already-up monitor interface is transparent to the running wfb_rx/wfb_tx
(they inject/capture on whatever the card is tuned to), so it avoids a process
restart. Mirrors the drone's radio-tune approach.
"""

import subprocess

# width (MHz) -> iw HT/channel-width mode argument
WIDTH_HTMODE = {5: "5MHz", 10: "10MHz", 20: "HT20", 40: "HT40+"}


def htmode(width: int) -> str:
    return WIDTH_HTMODE.get(width, "HT20")


def iw_args(wlan: str, channel: int, width: int) -> list[str]:
    """The `iw` command to retune one interface (mirrors wfb_ng init_wlans:
    values > 2000 are a frequency in MHz, otherwise a channel number)."""
    mode = htmode(width)
    if channel > 2000:
        return ["iw", "dev", wlan, "set", "freq", str(channel), mode]
    return ["iw", "dev", wlan, "set", "channel", str(channel), mode]


def retune_commands(wlans, link: dict) -> list[list[str]]:
    """Ordered `iw` commands to apply the live-settable radio params from `link`:
    regulatory domain first (it gates the channel), then per-interface
    channel/width, then txpower. txPowerDbm is dBm; it is converted to mBm
    (wfb-ng's wifi_txpower units) before passing to `iw`. None means 'auto'
    (driver default), so setting null reverts live power to the driver default."""
    cmds: list[list[str]] = []
    region = link.get("region")
    if region:
        cmds.append(["iw", "reg", "set", region])
    channel = link.get("channel")
    width = link.get("width", 20)
    txpower_dbm = link.get("txPowerDbm")
    for wlan in wlans:
        if channel is not None:
            cmds.append(iw_args(wlan, channel, width))
        # None => 'auto' (driver default), so lowering back to null reverts live;
        # a value is dBm, converted to fixed mBm (wfb-ng's wifi_txpower units).
        if txpower_dbm is None:
            cmds.append(["iw", "dev", wlan, "set", "txpower", "auto"])
        else:
            cmds.append(
                ["iw", "dev", wlan, "set", "txpower", "fixed", str(round(txpower_dbm * 100))]
            )
    return cmds


def init_commands(wlans, link: dict) -> list[list[str]]:
    """Ordered `iw` commands to initialize monitor-mode cards, matching wfb_ng's
    init_wlans sequence: regulatory domain first, then per-interface down/monitor/up,
    then channel/width and txpower (reused from retune_commands).
    """
    cmds: list[list[str]] = []
    region = link.get("region")
    if region:
        cmds.append(["iw", "reg", "set", region])

    # Per-interface monitor-mode initialization (down/monitor/up)
    for wlan in wlans:
        cmds.append(["ip", "link", "set", wlan, "down"])
        cmds.append(["iw", "dev", wlan, "set", "monitor", "otherbss"])
        cmds.append(["ip", "link", "set", wlan, "up"])

    # Append retune commands (channel/txpower), but skip the leading reg set
    # to avoid emitting it twice.
    retune_cmds = retune_commands(wlans, link)
    for cmd in retune_cmds:
        # Skip the first reg set command if it exists
        if cmd == ["iw", "reg", "set", region]:
            continue
        cmds.append(cmd)

    return cmds


def init_cards(wlans, link: dict) -> bool:
    """Apply every init command sequentially. Returns True only if all succeed."""
    ok = True
    for cmd in init_commands(wlans, link):
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            ok = False
    return ok


def retune(wlans, link: dict) -> bool:
    """Apply every retune command live. Returns True only if all succeed."""
    ok = True
    for cmd in retune_commands(wlans, link):
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            ok = False
    return ok
