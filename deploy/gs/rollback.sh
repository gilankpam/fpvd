#!/usr/bin/env bash
# deploy/gs/rollback.sh — restore the stock S98wifibroadcast on the GS.
set -euo pipefail
GS_HOST="${GS_HOST:-10.18.0.1}"; GS_USER="${GS_USER:-root}"
while [ $# -gt 0 ]; do case "$1" in
    --host) GS_HOST="$2"; shift 2 ;; --user) GS_USER="$2"; shift 2 ;;
    *) echo "unknown arg: $1" >&2; exit 2 ;; esac; done
TARGET="${GS_USER}@${GS_HOST}"
ssh -o StrictHostKeyChecking=accept-new "$TARGET" '
    set -e
    [ -x /etc/init.d/S99fpvd ] && /etc/init.d/S99fpvd stop >/dev/null 2>&1 || true
    rm -f /etc/init.d/S99fpvd /usr/bin/fpvd
    SITE="$(python3 -c "import site; print(site.getsitepackages()[0])")"
    rm -rf "$SITE/fpvdgs"
    if [ -f /root/fpvd-gs-rollback/S98wifibroadcast ]; then
        cp -a /root/fpvd-gs-rollback/S98wifibroadcast /etc/init.d/S98wifibroadcast
        cp -a /root/fpvd-gs-rollback/wifibroadcast.cfg.orig /etc/wifibroadcast.cfg 2>/dev/null || true
        chmod +x /etc/init.d/S98wifibroadcast
        /etc/init.d/S98wifibroadcast start
    fi
    echo rollback-done
'
