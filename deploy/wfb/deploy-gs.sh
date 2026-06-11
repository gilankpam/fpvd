#!/usr/bin/env bash
# Deploy swfec-fork wfb-ng to the GS: wfb_rx/wfb_tx binaries + the patched
# wfb_ng python (protocols.py — stats contract v3).
#
#   WFB_BIN=~/Projects/poc/wfb-ng WFB_SRC=~/Projects/poc/wfb-ng \
#       TARGET=root@10.18.0.1 ./deploy-gs.sh
set -euo pipefail

TARGET="${TARGET:-root@10.18.0.1}"
WFB_BIN="${WFB_BIN:?set WFB_BIN to the GS-arch wfb-ng build dir}"
WFB_SRC="${WFB_SRC:?set WFB_SRC to the wfb-ng fork checkout (for wfb_ng/*.py)}"
# No BatchMode on the GS (may not have keys pre-provisioned on first contact).
SSH_OPTS=(-o StrictHostKeyChecking=accept-new -o LogLevel=error)

remote() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }

for b in wfb_rx wfb_tx; do
    [ -x "$WFB_BIN/$b" ] || { echo "missing $WFB_BIN/$b" >&2; exit 1; }
done
[ -f "$WFB_SRC/wfb_ng/protocols.py" ] || { echo "missing $WFB_SRC/wfb_ng/protocols.py" >&2; exit 1; }

# Mirror gs/deploy.sh: locate the real site-packages via Python's site module
# (not via wfb_ng.__file__) so the path is canonical and matches how fpvdgs imports it.
SITE="$(remote 'python3 -c "import site; print(site.getsitepackages()[0])"')"
echo "==> GS wfb_ng package at: $SITE"

echo "==> rollback snapshot (one-time)"
remote "
    set -e
    mkdir -p /root/fpvd-gs-rollback/wfb
    for b in wfb_rx wfb_tx; do
        [ -f /root/fpvd-gs-rollback/wfb/\$b.orig ] || cp -a /usr/bin/\$b /root/fpvd-gs-rollback/wfb/\$b.orig
    done
    [ -f /root/fpvd-gs-rollback/wfb/protocols.py.orig ] || cp -a $SITE/wfb_ng/protocols.py /root/fpvd-gs-rollback/wfb/protocols.py.orig
"

echo "==> staging + swap binaries + protocols.py"
for b in wfb_rx wfb_tx; do
    scp -O "${SSH_OPTS[@]}" "$WFB_BIN/$b" "$TARGET:/usr/bin/$b.new"
done
scp -O "${SSH_OPTS[@]}" "$WFB_SRC/wfb_ng/protocols.py" "$TARGET:$SITE/wfb_ng/protocols.py.new"

remote "
    set -e
    for b in wfb_rx wfb_tx; do
        chmod +x /usr/bin/\$b.new && mv -f /usr/bin/\$b.new /usr/bin/\$b
    done
    mv -f $SITE/wfb_ng/protocols.py.new $SITE/wfb_ng/protocols.py
    # Stale interleav-fork config keys: the new python ignores unknown keys,
    # but warn so the operator can clean /etc/wifibroadcast.cfg.
    grep -n \"interleave_depth\" /etc/wifibroadcast.cfg 2>/dev/null \
        && echo \"NOTE: stale interleave_depth keys in /etc/wifibroadcast.cfg (harmless, consider removing)\" || true
    # Explicit stop + settle + clear stale pidfile, then start — NOT \`restart\`.
    # \`restart\` is stop;sleep 1;start: the 1s settle is too short and races.
    # Mirrors gs/deploy.sh fix (the documented restart race — observed live 2026-06-07).
    /etc/init.d/S99fpvd stop >/dev/null 2>&1 || true
    sleep 2
    rm -f /var/run/fpvd.pid
    /etc/init.d/S99fpvd start
"

echo "==> verify"
sleep 5
remote '
    printf "  procs: "; for p in wfb_rx wfb_tx; do
        printf "%s=%s " "$p" "$(pidof $p 2>/dev/null | cut -d" " -f1 || echo -)"; done; echo
    printf "  8103:  "; ss -tln 2>/dev/null | grep -q ":8103" && echo listening || echo down
'
echo "done. rollback: restore /root/fpvd-gs-rollback/wfb/* and restart S99fpvd"
