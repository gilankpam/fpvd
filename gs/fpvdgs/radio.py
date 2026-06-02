"""Live radio retune via `iw` on the running monitor interfaces.

Changing the card's channel/width with `iw` on an already-up monitor interface
is transparent to the running wfb_rx/wfb_tx (they inject/capture on whatever the
card is tuned to), so it avoids a process restart. Mirrors the drone's
radio-tune approach.
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


def retune(wlans, channel: int, width: int) -> bool:
    """Retune every interface live. Returns True only if all succeed."""
    ok = True
    for wlan in wlans:
        try:
            subprocess.run(iw_args(wlan, channel, width), check=True,
                           capture_output=True, timeout=5)
        except (OSError, subprocess.SubprocessError):
            ok = False
    return ok
