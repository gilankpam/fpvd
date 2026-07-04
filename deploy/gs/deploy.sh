#!/usr/bin/env bash
# deploy/gs/deploy.sh — install fpvd (GS) onto an OpenIPC SBC ground station.
#
# Pure Python: no build. Copies the fpvdgs package + init script, backs up and
# disables the stock S98wifibroadcast (wfb-server), then starts fpvd.
#
# Ordering: dynamicLink.tap.enabled defaults true, so this deploy renders
# `wfb_rx -D`; a GS still running the pre-tap wfb_rx binary exits on the
# unknown flag (crash-loop -> video down -> GS reboots on sustained video
# loss). Deploy the forked wfb binaries BEFORE running this script.
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
echo "[push] fpvdgs -> $TARGET:$SITE/fpvdgs  (+ init)"
remote "mkdir -p /etc/fpvd '$SITE/fpvdgs' '$SITE/fpvdgs/dynlink' '$SITE/fpvdgs/probe' '$SITE/fpvdgs/wfb'"
scp -O "${SSH_OPTS[@]}" "$GS/fpvdgs"/*.py "$TARGET:$SITE/fpvdgs/"
# dynlink subpackage (in-process GS dynamic-link controller)
scp -O "${SSH_OPTS[@]}" "$GS/fpvdgs/dynlink"/*.py "$TARGET:$SITE/fpvdgs/dynlink/"
# wfb subpackage (native wfb orchestration engine — spawns wfb_rx/wfb_tx)
scp -O "${SSH_OPTS[@]}" "$GS/fpvdgs/wfb"/*.py "$TARGET:$SITE/fpvdgs/wfb/"
# stale modules from older deploys (profile.py + JSON radio profiles) are
# harmless leftovers; remove the profiles dir so imports can't resolve it.
remote "rm -rf '$SITE/fpvdgs/dynlink/profiles' '$SITE/fpvdgs/dynlink/profile.py'* '$SITE/fpvdgs/dynlink/__pycache__/profile.'*"
# learned-prior key migration: the prior was keyed by the profile's display
# name (BL-M8812EU2); it is now keyed by dynamicLink.radioProfile (m8812eu2).
# One-time rename preserves the learned curve; -n never clobbers a newer file.
remote "[ -f /etc/fpvd/learned/BL-M8812EU2.json ] && mv -n /etc/fpvd/learned/BL-M8812EU2.json /etc/fpvd/learned/m8812eu2.json || true"
# probe subpackage (in-process GS probe-link measurement: spawns FEC-off wfb_rx
# per probe radio_port, parses stdout for per-MCS PER/RSSI — observe-only)
scp -O "${SSH_OPTS[@]}" "$GS/fpvdgs/probe"/*.py "$TARGET:$SITE/fpvdgs/probe/"
scp -O "${SSH_OPTS[@]}" "$GS/scripts/S99fpvd"  "$TARGET:/etc/init.d/S99fpvd"

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
    # then binds the HELLO listener on UDP 5801 and the IDR relay on 0.0.0.0:11223.
    if [ -f /etc/init.d/S99dynamic-link-gs ]; then
        [ -x /etc/init.d/S99dynamic-link-gs ] && /etc/init.d/S99dynamic-link-gs stop >/dev/null 2>&1 || true
        mv /etc/init.d/S99dynamic-link-gs /root/fpvd-gs-rollback/S99dynamic-link-gs
    fi
    # Retire the stock PixelPilot init script — fpvd now spawns/supervises the
    # pixelpilot binary directly (single owner of process + config). Stop it,
    # move it to the rollback dir. Idempotent: never clobber on re-deploy.
    for pp in /etc/init.d/S*pixelpilot*; do
        [ -e "$pp" ] || continue
        [ -x "$pp" ] && "$pp" stop >/dev/null 2>&1 || true
        mv "$pp" /root/fpvd-gs-rollback/
    done
    # the init stop runs the wrapper in the foreground and may orphan its
    # pixelpilot child (which keeps DRM + the RTP UDP port); reap any stragglers
    # so fpvd can spawn its own cleanly.
    killall -q pixelpilot pixelpilot.sh 2>/dev/null || true
    sleep 1
    : > /tmp/fpvd.log
    # Seed /etc/fpvd/config.json from code defaults on first deploy only;
    # never clobbers operator edits. Must run after /usr/bin/fpvd is installed
    # (above) and before the daemon starts (below) — mirrors the drone deploy.
    [ -f /etc/fpvd/config.json ] || { /usr/bin/fpvd --dump-config > /etc/fpvd/config.json.tmp && mv /etc/fpvd/config.json.tmp /etc/fpvd/config.json; }
    # Explicit stop + settle + clear stale pidfile, then start — NOT `restart`.
    # `restart` is stop;sleep 1;start: the 1s settle is too short, so the new fpvd
    # races the old one for its ports and dies just after "Starting fpvd: OK",
    # leaving a stale /var/run/fpvd.pid (the documented restart race — observed
    # live 2026-06-07 deploying the probe-driven selector: video dropped until a
    # manual `rm pidfile; S99fpvd start`). Mirrors the drone deploy fix (935424d).
    /etc/init.d/S99fpvd stop >/dev/null 2>&1 || true
    sleep 2
    rm -f /var/run/fpvd.pid
    /etc/init.d/S99fpvd start
'

echo "[verify]"
sleep 5
remote '
    printf "  fpvd:  "; ps w | grep -q "[f]pvdgs.supervisor" && echo running || echo DOWN
    printf "  dlstd: "; ps w | grep -q "[d]ynamic_link.service" && echo "STILL RUNNING (!)" || echo retired
    printf "  procs: "; for p in wfb_rx wfb_tx; do
        printf "%s=%s " "$p" "$(pidof $p 2>/dev/null | cut -d" " -f1 || echo -)"; done; echo
    printf "  pp:    "; pidof pixelpilot >/dev/null 2>&1 && echo running || echo DOWN
    printf "  api:   "; curl -s http://127.0.0.1:8080/status | head -c 200; echo
    printf "  8103:  "; ss -tln 2>/dev/null | grep -q ":8103" && echo listening || echo down
'
echo "[done] fpvd (GS) deployed to $GS_HOST. Rollback: deploy/gs/rollback.sh --host $GS_HOST"
