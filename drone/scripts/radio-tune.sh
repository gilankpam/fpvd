#!/bin/sh
# radio-tune.sh — apply ONE live radio change without restarting wfb.
# Usage: radio-tune.sh <channel|txpower|mtu>
# Inputs (env): FPVD_IFACE, FPVD_DRIVER, FPVD_CHANNEL, FPVD_WIDTH,
#               FPVD_TXPOWER, FPVD_MTU
set -eu

action="${1:-}"
iface="${FPVD_IFACE:-wlan0}"

case "$action" in
    channel)
        # 10MHz uses a dedicated token (baseband underclock, 20MHz modulation);
        # 40 => HT40+; everything else => HT20. Mirrors radio-up.sh.
        case "${FPVD_WIDTH:-20}" in
            10) iw "$iface" set channel "${FPVD_CHANNEL:-161}" 10MHz ;;
            40) iw "$iface" set channel "${FPVD_CHANNEL:-161}" HT40+ ;;
            *)  iw "$iface" set channel "${FPVD_CHANNEL:-161}" HT20 ;;
        esac
        ;;
    txpower)
        if [ "${FPVD_DRIVER:-}" = "88XXau" ]; then
            iw "$iface" set txpower fixed $(( ${FPVD_TXPOWER:-1} * -100 ))
        else
            iw "$iface" set txpower fixed $(( ${FPVD_TXPOWER:-1} *  50 ))
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
