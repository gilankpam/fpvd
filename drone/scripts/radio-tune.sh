#!/bin/sh
# radio-tune.sh — apply ONE live radio change without restarting wfb.
# Usage: radio-tune.sh <channel|txpower|mtu>
# Inputs (env): FPVD_IFACE, FPVD_DRIVER, FPVD_CHANNEL, FPVD_WIDTH,
#               FPVD_TXPOWER_DBM, FPVD_MTU
set -eu

action="${1:-}"
iface="${FPVD_IFACE:-wlan0}"

case "$action" in
    channel)
        # 5/10 MHz use dedicated tokens (baseband underclock, 20 MHz modulation);
        # 40 => HT40+; everything else => HT20. Mirrors radio-up.sh.
        case "${FPVD_WIDTH:-20}" in
            5)  iw "$iface" set channel "${FPVD_CHANNEL:-161}" 5MHz ;;
            10) iw "$iface" set channel "${FPVD_CHANNEL:-161}" 10MHz ;;
            40) iw "$iface" set channel "${FPVD_CHANNEL:-161}" HT40+ ;;
            *)  iw "$iface" set channel "${FPVD_CHANNEL:-161}" HT20 ;;
        esac
        ;;
    txpower)
        # FPVD_TXPOWER_DBM is dBm, or the literal "auto" to hand power to the
        # driver's per-rate TXAGC table (iw clears the user target). Otherwise
        # iw wants fixed mBm (dBm * 100). Matches radio-up.sh + radio_txpower.cpp.
        if [ "${FPVD_TXPOWER_DBM:-20}" = "auto" ]; then
            iw "$iface" set txpower auto
        else
            iw "$iface" set txpower fixed $(( ${FPVD_TXPOWER_DBM:-20} * 100 ))
        fi
        ;;
    mtu)
        ip link set "$iface" mtu "${FPVD_MTU:-1500}"
        ;;
    *)
        echo "radio-tune.sh: unknown action '$action'" >&2
        exit 2
        ;;
esac
