#!/bin/sh
echo "action=$1 iface=${FPVD_IFACE} channel=${FPVD_CHANNEL} width=${FPVD_WIDTH} txpower=${FPVD_TXPOWER_DBM} mtu=${FPVD_MTU}" >> "$FPVD_TEST_RECORD"
exit 0
