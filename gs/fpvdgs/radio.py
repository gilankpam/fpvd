"""Live radio retune via `iw` on the running monitor interfaces.

Changing the card's regulatory domain / channel / width / txpower with `iw` on
an already-up monitor interface is transparent to the running wfb_rx/wfb_tx
(they inject/capture on whatever the card is tuned to), so it avoids a process
restart. Mirrors the drone's radio-tune approach.
"""

import subprocess

# width (MHz) -> iw HT/channel-width mode argument
WIDTH_HTMODE = {10: "10MHz", 20: "HT20", 40: "HT40+"}


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
    channel/width, then txpower. txpower is mBm (wfb-ng's wifi_txpower units);
    None means 'leave at the driver default' (we don't touch it), matching the
    rendered cfg which omits wifi_txpower when null."""
    cmds: list[list[str]] = []
    region = link.get("region")
    if region:
        cmds.append(["iw", "reg", "set", region])
    channel = link.get("channel")
    width = link.get("width", 20)
    # rxpower (GS card power) — the iw verb is still 'txpower', but the config
    # key is rxpower since the GS card is a receiver.
    txpower = link.get("rxpower")
    for wlan in wlans:
        if channel is not None:
            cmds.append(iw_args(wlan, channel, width))
        # None => 'auto' (driver default), so lowering back to null reverts live;
        # a value is fixed mBm (wfb-ng's wifi_txpower units).
        if txpower is None:
            cmds.append(["iw", "dev", wlan, "set", "txpower", "auto"])
        else:
            cmds.append(["iw", "dev", wlan, "set", "txpower", "fixed", str(txpower)])
    return cmds


def retune(wlans, link: dict) -> bool:
    """Apply every retune command live. Returns True only if all succeed."""
    ok = True
    for cmd in retune_commands(wlans, link):
        try:
            subprocess.run(cmd, check=True, capture_output=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            ok = False
    return ok
