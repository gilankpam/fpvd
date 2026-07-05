#!/bin/sh
# radio-up.sh — bring up the USB radio for fpvd.
# Inputs (env): FPVD_CHANNEL, FPVD_WIDTH, FPVD_TXPOWER_DBM, FPVD_MTU
# Optional:     FPVD_WLAN_ADAPTER (forces an adapter id)
# Outputs (stdout, key=value lines): driver=, iface=, adapter_id=
set -eu

WLAN_DEV=wlan0
driver=""
adapter_id="${FPVD_WLAN_ADAPTER:-}"

for card in $(lsusb 2>/dev/null | awk '{print $6}' | uniq); do
    case "$card" in
        "0bda:8812"|"0bda:881a"|"0b05:17d2"|"2357:0101"|"2604:0012")
            driver=88XXau ;;
        "0bda:a81a")
            driver=8812eu; [ -z "$adapter_id" ] && adapter_id="bl-m8812eu2" ;;
        "0bda:f72b"|"0bda:b733")
            driver=8733bu; [ -z "$adapter_id" ] && adapter_id="bl-m8731bu4" ;;
    esac
done

if [ -z "$driver" ]; then
    echo "no supported USB wifi adapter detected" >&2
    exit 1
fi

if [ ! -e /sys/class/net/$WLAN_DEV ]; then
    if [ "$driver" != "88XXau" ]; then
        modprobe "$driver" rtw_tx_pwr_by_rate=0 rtw_tx_pwr_lmt_enable=0 MaxTxBufLen=32
    else
        modprobe "$driver" MaxTxBufLen=32
    fi
    sleep 3
fi
ifconfig $WLAN_DEV up
ifconfig $WLAN_DEV mtu "${FPVD_MTU:-1500}"
iw $WLAN_DEV set monitor none
# 10MHz uses a dedicated channel-width token (baseband underclocked, 20MHz
# modulation); 40 => HT40+; everything else => HT20.
case "${FPVD_WIDTH:-20}" in
    10) iw $WLAN_DEV set channel "${FPVD_CHANNEL:-161}" 10MHz ;;
    40) iw $WLAN_DEV set channel "${FPVD_CHANNEL:-161}" HT40+ ;;
    *)  iw $WLAN_DEV set channel "${FPVD_CHANNEL:-161}" HT20 ;;
esac
iw reg set 00
if [ "${FPVD_TXPOWER_DBM:-20}" = "auto" ]; then
    iw $WLAN_DEV set txpower auto
else
    iw $WLAN_DEV set txpower fixed $(( ${FPVD_TXPOWER_DBM:-20} * 100 ))
fi

# Chipset poke for ssc33x boards (telemetry serial enablement).
if command -v ipcinfo >/dev/null 2>&1; then
    chipset=$(ipcinfo -c 2>/dev/null || true)
    [ "$chipset" = "ssc33x" ] && devmem 0x1F207890 16 0x8 || true
fi

echo "driver=$driver"
echo "iface=$WLAN_DEV"
[ -n "$adapter_id" ] && echo "adapter_id=$adapter_id"
