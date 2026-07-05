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
        chmod +x /etc/init.d/S98wifibroadcast
        /etc/init.d/S98wifibroadcast start
    fi
    # restore the standalone dynamic-link-gs service (retired by deploy.sh) so a
    # full rollback returns the GS to its complete pre-fpvd state.
    if [ -f /root/fpvd-gs-rollback/S99dynamic-link-gs ]; then
        mv /root/fpvd-gs-rollback/S99dynamic-link-gs /etc/init.d/S99dynamic-link-gs
        chmod +x /etc/init.d/S99dynamic-link-gs
        /etc/init.d/S99dynamic-link-gs start >/dev/null 2>&1 || true
    fi
    # restore the stock PixelPilot init script retired by deploy.sh (fpvd
    # shutdown already stopped its pixelpilot child when S99fpvd stopped).
    for pp in /root/fpvd-gs-rollback/S*pixelpilot*; do
        [ -e "$pp" ] || continue
        name="$(basename "$pp")"
        mv "$pp" "/etc/init.d/$name"
        chmod +x "/etc/init.d/$name"
        "/etc/init.d/$name" start >/dev/null 2>&1 || true
    done
    echo rollback-done
'
