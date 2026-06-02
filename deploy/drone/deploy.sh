#!/usr/bin/env bash
# deploy/drone/deploy.sh — build + deploy fpvd to an OpenIPC drone.
#
# Auto-detects the situation:
#   * fresh install  -> backs up & removes the old S95waybeam / S98wifibroadcast /
#                       S99dynamic-link-applier init scripts, then starts fpvd.
#   * update         -> pushes the new binary and restarts fpvd.
#
# fpvd then supervises wfb_* + waybeam + msposd in-process (the old dl-applier
# binary is gone; adaptive link runs inside fpvd).
#
# Usage:
#   ./deploy/drone/deploy.sh [--host IP] [--user USER] [--skip-build]
# Env overrides: DRONE_HOST, DRONE_USER.
#
# Requires (on the dev machine): nix (shell.nix provides the armv7l/musl cross
# toolchain), ssh/scp. The drone uses busybox/dropbear (no sftp), so scp -O.
set -euo pipefail

DRONE_HOST="${DRONE_HOST:-192.168.10.152}"
DRONE_USER="${DRONE_USER:-root}"
SKIP_BUILD=0

while [ $# -gt 0 ]; do
    case "$1" in
        --host) DRONE_HOST="$2"; shift 2 ;;
        --user) DRONE_USER="$2"; shift 2 ;;
        --skip-build) SKIP_BUILD=1; shift ;;
        -h|--help)
            grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "unknown arg: $1" >&2; exit 2 ;;
    esac
done

REPO="$(cd "$(dirname "$0")/../.." && pwd)"
BIN="$REPO/build/ssc338q/fpvd"
TARGET="${DRONE_USER}@${DRONE_HOST}"
SSH_OPTS=(-o BatchMode=yes -o StrictHostKeyChecking=accept-new -o LogLevel=error)

remote() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }
copy()   { scp -O "${SSH_OPTS[@]}" "$1" "$TARGET:$2"; }

STRIPPED="$(mktemp)"; INIT="$(mktemp)"
trap 'rm -f "$STRIPPED" "$INIT"' EXIT

# ---- 1. build (cross, Release) + strip ----------------------------------
if [ "$SKIP_BUILD" -eq 0 ]; then
    echo "[build] cross-compiling fpvd for ssc338q (armv7l/musl/static) via nix-shell…"
    ( cd "$REPO" && nix-shell --run "cmake -S . -B build/ssc338q \
        -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-ssc338q.cmake -DCMAKE_BUILD_TYPE=Release \
        && cmake --build build/ssc338q --target fpvd -j" )
fi
[ -f "$BIN" ] || { echo "[error] no binary at $BIN (build failed, or --skip-build with no prior build)"; exit 1; }
( cd "$REPO" && nix-shell --run "armv7l-unknown-linux-musleabihf-strip -s -o '$STRIPPED' '$BIN'" )
echo "[build] stripped binary: $(stat -c %s "$STRIPPED") bytes"

# ---- 2. detect install vs update ----------------------------------------
if remote 'test -f /etc/init.d/S99fpvd'; then MODE=update; else MODE=install; fi
echo "[mode]  $MODE  ($TARGET)"

# ---- 3. push artifacts (binary to a .new staging name to dodge ETXTBSY) --
echo "[push]  binary, radio scripts, defaults, init script…"
remote 'mkdir -p /etc/fpvd /usr/libexec/fpvd'
copy "$STRIPPED"                   /usr/bin/fpvd.new
copy "$REPO/scripts/radio-up.sh"   /usr/libexec/fpvd/radio-up.sh
copy "$REPO/scripts/radio-tune.sh" /usr/libexec/fpvd/radio-tune.sh
copy "$REPO/etc/defaults.json"     /etc/fpvd/defaults.json   # baseline; overlay /etc/fpvd/config.json (user edits) is untouched

# init script: manual-deploy variant — /rom is read-only on a live system, so
# point --defaults at the writable /etc/fpvd/defaults.json.
cat > "$INIT" <<'EOF'
#!/bin/sh
DAEMON=fpvd
DAEMON_PATH=/usr/bin/fpvd
PIDFILE=/var/run/fpvd.pid
LOG=/tmp/fpvd.log
DAEMON_ARGS="--log $LOG --defaults /etc/fpvd/defaults.json"

start() {
    printf 'Starting %s: ' "$DAEMON"
    start-stop-daemon -S -q -b -m -p "$PIDFILE" -x "$DAEMON_PATH" -- $DAEMON_ARGS
    [ $? = 0 ] && echo "OK" || echo "FAIL"
    # (fpvd now writes the base OSD line itself when dynamic-link isn't feeding.)
}
stop() {
    printf 'Stopping %s: ' "$DAEMON"
    start-stop-daemon -K -q -p "$PIDFILE"
    [ $? = 0 ] && echo "OK" || echo "FAIL"
}
case "$1" in
    start)   start ;;
    stop)    stop ;;
    restart) stop; sleep 1; start ;;
    *) echo "Usage: $0 {start|stop|restart}"; exit 1 ;;
esac
EOF
copy "$INIT" /etc/init.d/S99fpvd
remote 'chmod +x /usr/bin/fpvd.new /usr/libexec/fpvd/radio-up.sh /usr/libexec/fpvd/radio-tune.sh /etc/init.d/S99fpvd'

# ---- 4. switch over / restart -------------------------------------------
if [ "$MODE" = install ]; then
    echo "[install] backing up + removing old stack, starting fpvd…"
    remote '
        set -e
        mkdir -p /root/fpvd-rollback/init.d
        cp -a /etc/waybeam.json /root/fpvd-rollback/waybeam.json.orig 2>/dev/null || true
        cp -a /etc/wfb.yaml     /root/fpvd-rollback/wfb.yaml.orig     2>/dev/null || true
        for s in S95waybeam S98wifibroadcast S99dynamic-link-applier; do
            [ -f /etc/init.d/$s ] && cp -a /etc/init.d/$s /root/fpvd-rollback/init.d/ || true
        done
        [ -x /etc/init.d/S95waybeam ]              && /etc/init.d/S95waybeam stop              >/dev/null 2>&1 || true
        [ -x /etc/init.d/S98wifibroadcast ]        && /etc/init.d/S98wifibroadcast stop        >/dev/null 2>&1 || true
        [ -x /etc/init.d/S99dynamic-link-applier ] && /etc/init.d/S99dynamic-link-applier stop >/dev/null 2>&1 || true
        sleep 2
        rm -f /etc/init.d/S95waybeam /etc/init.d/S98wifibroadcast /etc/init.d/S99dynamic-link-applier
        mv -f /usr/bin/fpvd.new /usr/bin/fpvd
        : > /tmp/fpvd.log
        /etc/init.d/S99fpvd start
    '
else
    echo "[update] restarting fpvd with the new binary…"
    remote '
        /etc/init.d/S99fpvd stop >/dev/null 2>&1 || true
        sleep 1
        mv -f /usr/bin/fpvd.new /usr/bin/fpvd
        : > /tmp/fpvd.log
        /etc/init.d/S99fpvd start
    '
fi

# ---- 5. verify ----------------------------------------------------------
sleep 5
echo "[verify]"
remote '
    printf "  procs: "; for p in fpvd wfb_tx wfb_rx wfb_tun waybeam msposd; do
        printf "%s=%s " "$p" "$(pidof $p 2>/dev/null | cut -d" " -f1 || echo -)"; done; echo
    printf "  radio: "; iw dev wlan0 info 2>/dev/null | grep -iE "channel|txpower" | tr -d "\t" | paste -sd" " - || echo "(no wlan)"
    printf "  dl:    "; curl -s http://127.0.0.1:8080/status | tr "," "\n" | grep -iE "\"enabled\"|\"running\"" | head -2 | paste -sd" " -
'
echo "[done]  fpvd deployed to $DRONE_HOST  (mode=$MODE).  Rollback: deploy/drone/rollback.sh --host $DRONE_HOST"
