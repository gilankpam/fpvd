#!/usr/bin/env bash
# Deploy swfec-fork wfb-ng binaries to the drone.
#
#   WFB_BIN=~/Projects/poc/wfb-ng TARGET=root@192.168.10.152 ./deploy-drone.sh
#
# Expects wfb_tx/wfb_rx/wfb_tun cross-built for the drone in $WFB_BIN
# (build them in the fork: make wfb_tx wfb_rx wfb_tun with the cross
# toolchain env). Originals are preserved once in /root/fpvd-rollback/wfb/.
# fpvd supervises the wfb_* processes, so we stop fpvd around the swap.
set -euo pipefail

TARGET="${TARGET:-root@192.168.10.152}"
WFB_BIN="${WFB_BIN:?set WFB_BIN to the wfb-ng build dir containing wfb_tx/wfb_rx/wfb_tun}"
# Drone uses busybox/dropbear (no sftp) → scp -O; accept-new for first contact.
SSH_OPTS=(-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o LogLevel=error)

remote() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }
copy()   { scp -O "${SSH_OPTS[@]}" "$1" "$TARGET:$2"; }

for b in wfb_tx wfb_rx wfb_tun; do
    [ -x "$WFB_BIN/$b" ] || { echo "missing $WFB_BIN/$b" >&2; exit 1; }
done

echo "==> staging binaries"
for b in wfb_tx wfb_rx wfb_tun; do
    copy "$WFB_BIN/$b" "/usr/bin/$b.new"
done

echo "==> swapping (with one-time rollback snapshot)"
remote '
    set -e
    mkdir -p /root/fpvd-rollback/wfb
    for b in wfb_tx wfb_rx wfb_tun; do
        [ -f /root/fpvd-rollback/wfb/$b.orig ] || cp -a /usr/bin/$b /root/fpvd-rollback/wfb/$b.orig
    done
    /etc/init.d/S99fpvd stop >/dev/null 2>&1 || true
    sleep 2
    for b in wfb_tx wfb_rx wfb_tun; do
        chmod +x /usr/bin/$b.new
        mv -f /usr/bin/$b.new /usr/bin/$b
    done
    # start-stop-daemon -K signals but does not remove the pidfile; a stale
    # /var/run/fpvd.pid makes the subsequent -S fail ("Starting fpvd: FAIL").
    # Clear it after the stop+settle so the start is clean (the documented race).
    rm -f /var/run/fpvd.pid
    /etc/init.d/S99fpvd start
'

echo "==> verify"
sleep 5
remote '
    printf "  procs: "; for p in fpvd wfb_tx wfb_rx wfb_tun; do
        printf "%s=%s " "$p" "$(pidof $p 2>/dev/null | cut -d" " -f1 || echo -)"; done; echo
    pidof fpvd >/dev/null 2>&1
' || { echo "VERIFY FAILED: fpvd not running" >&2; exit 1; }
echo "done. rollback: restore /root/fpvd-rollback/wfb/*.orig and restart S99fpvd"
