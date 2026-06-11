#!/usr/bin/env bash
# Deploy swfec-fork wfb-ng to the GS: wfb_rx/wfb_tx binaries + the patched
# wfb_ng python (full package — the GS's previous package passes -X to
# wfb_tx, which the swfec-fork binary rejects).
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
for p in protocols.py services.py; do
    [ -f "$WFB_SRC/wfb_ng/$p" ] || { echo "missing $WFB_SRC/wfb_ng/$p" >&2; exit 1; }
done

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
    [ -d /root/fpvd-gs-rollback/wfb/wfb_ng.orig ] || cp -a $SITE/wfb_ng /root/fpvd-gs-rollback/wfb/wfb_ng.orig
"

echo "==> staging + swap binaries + wfb_ng python package"
for b in wfb_rx wfb_tx; do
    scp -O "${SSH_OPTS[@]}" "$WFB_BIN/$b" "$TARGET:/usr/bin/$b.new"
done
for f in "$WFB_SRC"/wfb_ng/*.py; do
    scp -O "${SSH_OPTS[@]}" "$f" "$TARGET:$SITE/wfb_ng/$(basename "$f").new"
done

remote "
    set -e
    for b in wfb_rx wfb_tx; do
        chmod +x /usr/bin/\$b.new && mv -f /usr/bin/\$b.new /usr/bin/\$b
    done
    for f in $SITE/wfb_ng/*.py.new; do
        mv -f \"\$f\" \"\${f%.new}\"
    done
    rm -rf $SITE/wfb_ng/__pycache__
    # Stale interleav-fork config keys: the new package's config parser sets
    # unknown keys as plain (unused) attributes, so they are ignored — warn so
    # the operator can clean /etc/wifibroadcast.cfg.
    grep -n \"interleave_depth\" /etc/wifibroadcast.cfg 2>/dev/null \
        && echo \"NOTE: stale interleave_depth keys in /etc/wifibroadcast.cfg (ignored by the new package, consider removing)\" || true
    # Explicit stop + settle + clear stale pidfile, then start — NOT \`restart\`.
    # \`restart\` is stop;sleep 1;start: the 1s settle is too short and races.
    # Mirrors gs/deploy.sh fix (the documented restart race — observed live 2026-06-07).
    /etc/init.d/S99fpvd stop >/dev/null 2>&1 || true
    sleep 2
    rm -f /var/run/fpvd.pid
    /etc/init.d/S99fpvd start
"

echo "==> verify"
# Poll up to ~30s: a full-package swap makes fpvdgs re-import wfb_ng and
# rebind sockets, which takes well over 5s — a single short sleep+check
# false-negatives even on a healthy start (observed live 2026-06-11).
remote '
    for i in $(seq 1 15); do
        ss -tln 2>/dev/null | grep -q ":8103" && break
        sleep 2
    done
    printf "  procs: "; for p in wfb_rx wfb_tx; do
        printf "%s=%s " "$p" "$(pidof $p 2>/dev/null | cut -d" " -f1 || echo -)"; done; echo
    printf "  8103:  "; ss -tln 2>/dev/null | grep -q ":8103" && echo listening || echo down
    ss -tln 2>/dev/null | grep -q ":8103"
' || { echo "VERIFY FAILED: GS :8103 not listening after ~30s" >&2; exit 1; }
echo "done. rollback: restore /root/fpvd-gs-rollback/wfb/* and restart S99fpvd"
