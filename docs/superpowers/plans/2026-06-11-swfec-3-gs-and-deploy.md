# swfec Adoption Plan 3/3 — fpvd GS + wfb-ng Deploy Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Accept the v3 stats contract in fpvdgs, add `deploy/wfb/` scripts that ship the swfec-fork wfb-ng binaries + python patch to both ends (nothing ships them today), and document the staged cutover.

**Architecture:** GS changes are tiny by design — Phase 3b made FEC drone-owned, the GS only observes it: `stats_client` accepts contract v3 and the `'swfec'` fec_type; signals/policy/probe are untouched. Deployment follows the existing `deploy/{drone,gs}/deploy.sh` conventions (env-var target, `scp -O` for dropbear, rollback dirs, verify step). Spec: `docs/superpowers/specs/2026-06-11-swfec-adoption-design.md`.

**Tech Stack:** Python 3 + pytest (GS), POSIX shell (deploy).

**Repo/branch:** `/home/gilankpam/Projects/drone/fpvd`, branch `feat/swfec-adoption`. Prereqs: Plan 1 (fork) done; Plan 2 (drone) done.

---

### Task 1: stats_client — contract v3 + swfec session semantics

**Files:**
- Modify: `gs/fpvdgs/dynlink/stats_client.py:23` (`CONTRACT_VERSIONS_SUPPORTED`), `:54-62` (SessionInfo docstring)
- Test: `gs/tests/unit/test_dl_stats_client.py` (create if it does not exist; tests run with `cd gs && pytest tests/`)

- [ ] **Step 1: Write the failing tests**

Add to `gs/tests/unit/test_dl_stats_client.py`:

```python
import pytest

from fpvdgs.dynlink.stats_client import (
    ContractVersionError,
    SessionEvent,
    parse_record,
)


def _session_raw(contract_version, fec_type="swfec", fec_k=50, fec_n=30):
    return {
        "type": "new_session", "timestamp": 1.0, "id": "video rx",
        "fec_type": fec_type, "fec_k": fec_k, "fec_n": fec_n,
        "epoch": 1, "interleave_depth": 1,
        "contract_version": contract_version,
    }


def test_contract_v3_swfec_session_accepted():
    ev = parse_record(_session_raw(3))
    assert isinstance(ev, SessionEvent)
    assert ev.session.fec_type == "swfec"
    assert ev.session.fec_k == 50      # overhead_pct in swfec sessions
    assert ev.session.fec_n == 30      # deadline_ms in swfec sessions
    assert ev.session.contract_version == 3


def test_contract_v3_rx_record_accepted():
    raw = {
        "type": "rx", "timestamp": 1.0, "id": "video rx",
        "packets": {"out": [100, 100], "lost": [0, 0]},
        "rx_ant_stats": [],
        "session": _session_raw(3),
    }
    ev = parse_record(raw)
    assert ev.session.contract_version == 3


def test_unknown_contract_version_still_rejected():
    with pytest.raises(ContractVersionError):
        parse_record(_session_raw(4))
```

- [ ] **Step 2: Run to verify failure**

Run: `cd gs && pytest tests/unit/test_dl_stats_client.py -v`
Expected: the two v3 tests FAIL with `ContractVersionError` (supported set is `{1,2}`); the rejection test passes.

- [ ] **Step 3: Implement**

In `gs/fpvdgs/dynlink/stats_client.py` line 23:

```python
# v3: wfb-ng swfec fork — fec_type may be 'swfec', in which case the
# session's fec_k/fec_n slots carry overhead_pct/deadline_ms.
CONTRACT_VERSIONS_SUPPORTED = frozenset({1, 2, 3})
```

And extend the `SessionInfo` dataclass docstring (line 54-62) so the field reinterpretation is documented at the type:

```python
@dataclass
class SessionInfo:
    """Session parameters from the wfb_rx SESSION record.

    For fec_type 'swfec' (contract v3), fec_k/fec_n carry
    overhead_pct/deadline_ms — the sliding-window codec has no block
    geometry. interleave_depth is a legacy field, always 1 on v3 feeds.
    """
    fec_type: str
    fec_k: int
    fec_n: int
    epoch: int
    interleave_depth: int
    contract_version: int
```

- [ ] **Step 4: Run the full GS suite**

Run: `cd gs && pytest tests/`
Expected: ALL PASS (signals/controller/policy consume `lost`/`fec_rec`/`out`, which the swfec RX populates compatibly — no other change needed).

- [ ] **Step 5: Commit**

```bash
git add gs/fpvdgs/dynlink/stats_client.py gs/tests/unit/test_dl_stats_client.py
git commit -m "gs/dynlink: accept stats contract v3 (swfec sessions)"
```

---

### Task 2: deploy/wfb — drone binary deployment script

**Files:**
- Create: `deploy/wfb/deploy-drone.sh`

Conventions copied from `deploy/drone/deploy.sh`: `TARGET` env (default `root@192.168.10.152`), `scp -O` (dropbear has no sftp), rollback dir `/root/fpvd-rollback/wfb/`, verify step. The script ships **prebuilt** binaries — cross-compilation stays in the wfb-ng repo (its Makefile already carries the static-cross tweaks); pass the build dir via `WFB_BIN`.

- [ ] **Step 1: Write the script**

```bash
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
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5)

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
    /etc/init.d/S99fpvd stop || true
    sleep 1
    for b in wfb_tx wfb_rx wfb_tun; do
        chmod +x /usr/bin/$b.new
        mv -f /usr/bin/$b.new /usr/bin/$b
    done
    # deploy restart race (see memory): clear a stale pidfile before start
    rm -f /var/run/fpvd.pid
    /etc/init.d/S99fpvd start
'

echo "==> verify"
remote '
    sleep 3
    printf "procs: "; for p in fpvd wfb_tx wfb_rx wfb_tun; do
        pidof $p >/dev/null && printf "%s:up " $p || printf "%s:DOWN " $p
    done; echo
'
echo "done. rollback: restore /root/fpvd-rollback/wfb/*.orig and restart S99fpvd"
```

- [ ] **Step 2: Syntax check**

Run: `bash -n deploy/wfb/deploy-drone.sh && chmod +x deploy/wfb/deploy-drone.sh`
Expected: no output (clean parse).

- [ ] **Step 3: Commit**

```bash
git add deploy/wfb/deploy-drone.sh
git commit -m "deploy/wfb: drone binary deployment for the swfec wfb-ng fork"
```

---

### Task 3: deploy/wfb — GS binary + python deployment script

**Files:**
- Create: `deploy/wfb/deploy-gs.sh`

Conventions from `deploy/gs/deploy.sh`: target `root@10.18.0.1`, the patched `wfb_ng` python files go into the GS's real site-packages (located dynamically, same trick as fpvdgs deploy), rollback dir `/root/fpvd-gs-rollback/wfb/`. GS wfb binaries are built natively for the GS arch in the fork repo.

- [ ] **Step 1: Write the script**

```bash
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
SSH_OPTS=(-o BatchMode=yes -o ConnectTimeout=5)

remote() { ssh "${SSH_OPTS[@]}" "$TARGET" "$@"; }
copy()   { scp -O "${SSH_OPTS[@]}" "$1" "$TARGET:$2"; }

for b in wfb_rx wfb_tx; do
    [ -x "$WFB_BIN/$b" ] || { echo "missing $WFB_BIN/$b" >&2; exit 1; }
done
[ -f "$WFB_SRC/wfb_ng/protocols.py" ] || { echo "missing $WFB_SRC/wfb_ng/protocols.py" >&2; exit 1; }

SITE=$(remote 'python3 -c "import wfb_ng, os; print(os.path.dirname(wfb_ng.__file__))"')
echo "==> GS wfb_ng package at: $SITE"

echo "==> rollback snapshot (one-time)"
remote "
    set -e
    mkdir -p /root/fpvd-gs-rollback/wfb
    for b in wfb_rx wfb_tx; do
        [ -f /root/fpvd-gs-rollback/wfb/\$b.orig ] || cp -a /usr/bin/\$b /root/fpvd-gs-rollback/wfb/\$b.orig
    done
    [ -f /root/fpvd-gs-rollback/wfb/protocols.py.orig ] || cp -a $SITE/protocols.py /root/fpvd-gs-rollback/wfb/protocols.py.orig
"

echo "==> staging + swap"
for b in wfb_rx wfb_tx; do
    copy "$WFB_BIN/$b" "/usr/bin/$b.new"
done
copy "$WFB_SRC/wfb_ng/protocols.py" "$SITE/protocols.py"
remote '
    set -e
    for b in wfb_rx wfb_tx; do
        chmod +x /usr/bin/$b.new && mv -f /usr/bin/$b.new /usr/bin/$b
    done
    # Stale interleav-fork config keys: the new python ignores unknown keys,
    # but warn so the operator can clean /etc/wifibroadcast.cfg.
    grep -n "interleave_depth" /etc/wifibroadcast.cfg 2>/dev/null \
        && echo "NOTE: stale interleave_depth keys in /etc/wifibroadcast.cfg (harmless, consider removing)" || true
    /etc/init.d/S99fpvd restart
'

echo "==> verify"
remote '
    sleep 3
    printf "procs: "; for p in wfb_rx wfb_tx; do
        pidof $p >/dev/null && printf "%s:up " $p || printf "%s:DOWN " $p
    done; echo
    # stats feed up?
    (exec 3<>/dev/tcp/127.0.0.1/8103) >/dev/null 2>&1 && echo "stats :8103 open" || echo "stats :8103 CLOSED"
'
echo "done. rollback: restore /root/fpvd-gs-rollback/wfb/* and restart S99fpvd"
```

- [ ] **Step 2: Syntax check**

Run: `bash -n deploy/wfb/deploy-gs.sh && chmod +x deploy/wfb/deploy-gs.sh`
Expected: clean parse.

- [ ] **Step 3: Commit**

```bash
git add deploy/wfb/deploy-gs.sh
git commit -m "deploy/wfb: GS binary + wfb_ng python deployment for the swfec fork"
```

---

### Task 4: Cutover runbook

**Files:**
- Create: `deploy/wfb/README.md`

- [ ] **Step 1: Write the runbook**

```markdown
# wfb-ng swfec deployment + cutover

Builds come from `~/Projects/poc/wfb-ng` branch `swfec`:
- drone: cross-build `make wfb_tx wfb_rx wfb_tun` with the drone toolchain env
- GS: native `make wfb_rx wfb_tx`

## Staged cutover (order matters)

1. **Binaries first, behavior unchanged.** With `link.fec.mode` still `"rs"`:
   `./deploy-drone.sh` then `./deploy-gs.sh`. New code, old RS behavior,
   contract v3 live on the stats feed.
   - Verify: video up, GS `:8103` JSON shows `"contract_version": 3`,
     probe + dynamic link still driving MCS.
2. **fpvd both ends.** `deploy/drone/deploy.sh` (new fpvd: swfec schema,
   interleaver removed) and `deploy/gs/deploy.sh` (fpvdgs accepts v3).
   - NOTE: deploy fpvd only AFTER the wfb binaries — old wfb_tx would
     reject nothing, but new fpvd never sends CMD 5, while OLD fpvd with
     interleavingSupported=true against NEW binaries would error on CMD 5.
     The wfb_tx control socket rejects unknown cmds with an error response;
     fpvd logs it on every dynlink dispatch — noisy but not fatal. Keep the
     window short.
3. **Flip the mode.** Via the GS proxy to the drone config API:

       curl -X PATCH http://10.18.0.1:8080/air/config \
            -H 'Content-Type: application/json' \
            -d '{"link":{"fec":{"mode":"swfec"}}}'
       curl -X POST http://10.18.0.1:8080/air/apply

   (Mode flip = wfb_video_tx restart; expect a brief video drop.
   With dynamicLink enabled, `link.fec` is locked — disable DL, flip,
   re-enable, or flip before arming DL.)
   - Verify: GS stats show `"fec_type": "swfec"`, `"fec_k": 50`,
     `"fec_n": 30`; OSD shows fec_rec activity under induced loss.

## Rollback

- Mode-level: PATCH `{"link":{"fec":{"mode":"rs"}}}` + `/air/apply` (config only).
- Binary-level: restore `/root/fpvd-rollback/wfb/*.orig` (drone) /
  `/root/fpvd-gs-rollback/wfb/*` (GS), restart `S99fpvd`.
- fpvd-level: existing `deploy/{drone,gs}/rollback.sh`.

## Bench A/B before flight

Flip mode rs↔swfec at fixed MCS/power on the bench; compare
`residual_loss_w` / `fec_rec` under induced loss (see
`docs/superpowers/specs/2026-06-11-swfec-adoption-design.md` §5).
```

- [ ] **Step 2: Commit**

```bash
git add deploy/wfb/README.md
git commit -m "deploy/wfb: staged swfec cutover runbook"
```

---

### Task 5: Full verification

- [ ] **Step 1: GS suite** — `cd gs && pytest tests/` → ALL PASS.
- [ ] **Step 2: Drone suite** — `cd drone && ./build/fpvd_tests` → ALL PASS (unchanged by this plan; sanity).
- [ ] **Step 3: Shell checks** — `bash -n deploy/wfb/*.sh` → clean.
- [ ] **Step 4: Push** — `git push origin feat/swfec-adoption`.
