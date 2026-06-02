#!/usr/bin/env bash
# deploy/gs/deploy.sh — install fpvd (GS) onto an OpenIPC SBC ground station.
#
# Pure Python: no build. Copies the fpvdgs package + init script, backs up and
# disables the stock S98wifibroadcast (wfb-server), then starts fpvd.
#
# Usage: ./deploy/gs/deploy.sh [--host IP] [--user USER]
# Env overrides: GS_HOST, GS_USER.
set -euo pipefail

GS_HOST="${GS_HOST:-10.18.0.1}"
GS_USER="${GS_USER:-root}"
while [ $# -gt 0 ]; do
    case "$1" in
        --host) GS_HOST="$2"; shift 2 ;;
        --user) GS_USER="$2"; shift 2 ;;
        -h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
GS="$REPO/gs"
TARGET="${GS_USER}@${GS_HOST}"
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o LogLevel=error)
remote() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }

# Install into the GS's real site-packages (where wfb_ng already lives) so that
# both `python3 -m fpvdgs.supervisor` and the spawned `python3 -m fpvdgs.runner`
# import without any sys.path/PYTHONPATH hacks.
SITE="$(remote 'python3 -c "import site; print(site.getsitepackages()[0])"')"
echo "[push] fpvdgs -> $TARGET:$SITE/fpvdgs  (+ init + defaults)"
remote "mkdir -p /etc/fpvd '$SITE/fpvdgs' '$SITE/fpvdgs/dynlink/profiles'"
scp -O "${SSH_OPTS[@]}" "$GS/fpvdgs"/*.py "$TARGET:$SITE/fpvdgs/"
# dynlink subpackage (in-process GS dynamic-link controller) + JSON radio profiles
scp -O "${SSH_OPTS[@]}" "$GS/fpvdgs/dynlink"/*.py "$TARGET:$SITE/fpvdgs/dynlink/"
scp -O "${SSH_OPTS[@]}" "$GS/fpvdgs/dynlink/profiles"/*.json "$TARGET:$SITE/fpvdgs/dynlink/profiles/"
# defaults (do not clobber an existing user overlay /etc/fpvd/config.json)
scp -O "${SSH_OPTS[@]}" "$GS/etc/defaults.json" "$TARGET:/etc/fpvd/defaults.json"
scp -O "${SSH_OPTS[@]}" "$GS/scripts/S99fpvd"  "$TARGET:/etc/init.d/S99fpvd"
# initial dynamic-link overlay (production tuning translated from the standalone's
# gs.yaml) — installed ONLY on first deploy; never clobbers operator edits.
if remote 'test -e /etc/fpvd/config.json'; then
    echo "[skip] /etc/fpvd/config.json exists — operator overlay preserved"
else
    echo "[push] initial config.json -> $TARGET:/etc/fpvd/config.json"
    scp -O "${SSH_OPTS[@]}" "$REPO/deploy/gs/config.json" "$TARGET:/etc/fpvd/config.json"
fi

echo "[install] fpvd launcher + backup/disable S98wifibroadcast"
remote '
    set -e
    # launcher: `fpvd` runs the now-importable package entrypoint
    printf "#!/bin/sh\nexec python3 -m fpvdgs.supervisor \"\$@\"\n" > /usr/bin/fpvd
    chmod +x /usr/bin/fpvd /etc/init.d/S99fpvd

    mkdir -p /root/fpvd-gs-rollback
    # back up the stock cfg + init script ONCE; never clobber on re-deploy
    # (the live /etc/wifibroadcast.cfg is fpvd-generated after the first install).
    if [ ! -e /root/fpvd-gs-rollback/wifibroadcast.cfg.orig ]; then
        cp -a /etc/wifibroadcast.cfg /root/fpvd-gs-rollback/wifibroadcast.cfg.orig 2>/dev/null || true
    fi
    if [ -f /etc/init.d/S98wifibroadcast ] && [ ! -e /root/fpvd-gs-rollback/S98wifibroadcast ]; then
        cp -a /etc/init.d/S98wifibroadcast /root/fpvd-gs-rollback/
    fi
    [ -x /etc/init.d/S98wifibroadcast ] && /etc/init.d/S98wifibroadcast stop >/dev/null 2>&1 || true
    sleep 2
    rm -f /etc/init.d/S98wifibroadcast

    # Retire the standalone dynamic-link-gs service — now folded into fpvd as the
    # in-process dynlink controller + IDR relay. Stop it (also stops its bundled
    # idr-forwarder socat) and disable boot autostart, keeping the init script in
    # the rollback dir so the GS dynamic-link role is fully owned by fpvd. fpvd
    # then binds the HELLO listener on UDP 5801 and the IDR relay on 127.0.0.1:11223.
    if [ -f /etc/init.d/S99dynamic-link-gs ]; then
        [ -x /etc/init.d/S99dynamic-link-gs ] && /etc/init.d/S99dynamic-link-gs stop >/dev/null 2>&1 || true
        mv /etc/init.d/S99dynamic-link-gs /root/fpvd-gs-rollback/S99dynamic-link-gs
    fi
    : > /tmp/fpvd.log
    /etc/init.d/S99fpvd restart   # restart: reloads code on re-deploy; starts on first install
'

echo "[verify]"
sleep 5
remote '
    printf "  fpvd:  "; ps w | grep -q "[f]pvdgs.supervisor" && echo running || echo DOWN
    printf "  dlstd: "; ps w | grep -q "[d]ynamic_link.service" && echo "STILL RUNNING (!)" || echo retired
    printf "  procs: "; for p in wfb_rx wfb_tx; do
        printf "%s=%s " "$p" "$(pidof $p 2>/dev/null | cut -d" " -f1 || echo -)"; done; echo
    printf "  api:   "; curl -s http://127.0.0.1:8080/status | head -c 200; echo
    printf "  8103:  "; ss -tln 2>/dev/null | grep -q ":8103" && echo listening || echo down
'
echo "[done] fpvd (GS) deployed to $GS_HOST. Rollback: deploy/gs/rollback.sh --host $GS_HOST"
