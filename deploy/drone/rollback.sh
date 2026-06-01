#!/usr/bin/env bash
# deploy/drone/rollback.sh — revert a drone from fpvd back to the original
# OpenIPC stack (S95waybeam / S98wifibroadcast / S99dynamic-link-applier),
# using the backup deploy.sh left in /root/fpvd-rollback on first install.
#
# Usage: ./deploy/drone/rollback.sh [--host IP] [--user USER] [--reboot]
# Env overrides: DRONE_HOST, DRONE_USER.
set -euo pipefail

DRONE_HOST="${DRONE_HOST:-192.168.10.152}"
DRONE_USER="${DRONE_USER:-root}"
REBOOT=0
while [ $# -gt 0 ]; do
    case "$1" in
        --host) DRONE_HOST="$2"; shift 2 ;;
        --user) DRONE_USER="$2"; shift 2 ;;
        --reboot) REBOOT=1; shift ;;
        -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done
TARGET="${DRONE_USER}@${DRONE_HOST}"
SSH_OPTS=(-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o LogLevel=error)

echo "[rollback] restoring original stack on $TARGET …"
ssh "${SSH_OPTS[@]}" "$TARGET" '
    set -e
    B=/root/fpvd-rollback
    [ -d "$B/init.d" ] || { echo "no backup at $B — cannot roll back"; exit 1; }
    /etc/init.d/S99fpvd stop >/dev/null 2>&1 || true
    sleep 1
    cp -a "$B"/init.d/* /etc/init.d/ 2>/dev/null || true
    [ -f "$B/waybeam.json.orig" ] && cp -a "$B/waybeam.json.orig" /etc/waybeam.json || true
    [ -f "$B/wfb.yaml.orig" ]     && cp -a "$B/wfb.yaml.orig"     /etc/wfb.yaml     || true
    rm -f /etc/init.d/S99fpvd
    echo "restored: $(ls /etc/init.d | grep -E "waybeam|wifibroadcast|dynamic-link" | paste -sd" " -)"
'

if [ "$REBOOT" -eq 1 ]; then
    echo "[rollback] rebooting drone to start the original stack…"
    ssh "${SSH_OPTS[@]}" "$TARGET" 'reboot' || true
else
    echo "[rollback] files restored. Reboot the drone to start the original stack:"
    echo "           ssh $TARGET reboot     (or re-run with --reboot)"
fi
