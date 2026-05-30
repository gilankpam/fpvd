# Link Hot-Apply — Design

**Date:** 2026-05-30
**Branch:** `feat/link-hot-apply`
**Status:** Approved design, pending implementation plan

## Goal

Change `link.*` parameters at runtime **without restarting the wfb stack**. Today
`/apply` tears down and rebuilds the whole orchestrator (all `wfb_tx`/`wfb_rx`
processes) for any link change, interrupting video/tunnel/telemetry. This refactor
applies link changes in place:

- **NIC-level params** (channel, width, txpower, mtu) → `iw`/`ip` commands.
- **Radiotap/FEC params** (mcs, fec, stbc, ldpc, bandwidth) → pushed to the video
  `wfb_tx` over its localhost UDP control socket (`CMD_SET_RADIO` / `CMD_SET_FEC`).
- **tun/tlm** become boot-once processes with fixed, robust params, fully decoupled
  from `link.*` (so a link change never needs to touch them).

Only `linkId` and `wlanAdapter` still require a full restart (rare).

## Background / verified facts

- **wfb_tx control socket:** `wfb_tx -C <port>` binds a UDP socket on `127.0.0.1`.
  Today only `wfb_video_tx` has one (`-C 8000`); `wfb_tun_tx`/`wfb_tlm_tx` have none.
  This matches upstream OpenIPC `wifibroadcast` exactly (only the video tx gets `-C`).
- **Wire format** (`wfb-ng/src/tx_cmd.h`, packed structs):
  - `cmd_req_t`: `uint32_t req_id` (network byte order, htonl/ntohl) + `uint8_t cmd_id`
    + union. `CMD_SET_RADIO=2` payload `{uint8 stbc, bool ldpc, bool short_gi,
    uint8 bandwidth, uint8 mcs_index, bool vht_mode, uint8 vht_nss}`.
    `CMD_SET_FEC=1` payload `{uint8 k, uint8 n}`. All payload fields are single-byte,
    byte-order independent.
  - Reference client: `wfbng-dynamic-link/drone/src/dl_backend_tx.c`
    (`send_radio`/`send_fec`): connected UDP, `req_id` htonl + match-on-recv,
    500 ms `SO_RCVTIMEO`, drains stale replies before send.
- **`CMD_SET_RADIO` hot-swaps the radiotap header** at runtime
  (`tx.cpp` `init_radiotap_header` + `update_radiotap_header`) — no restart, no link
  drop. This is exactly what `dl_applier` does at ~10 Hz.
- **10 MHz and `-B`:** `init_radiotap_header` treats `bandwidth=10` identically to
  `bandwidth=20` (both → `IEEE80211_RADIOTAP_MCS_BW_20` / `VHT_BW_20M`). There is no
  distinct 10 MHz radiotap flag. The genuine 10 MHz narrowing comes from the NIC
  baseband underclock via `iw … set channel <C> 10MHz`. fpvd already maps
  `modulationWidth(10) → 20` for `-B`, so the radiotap value is correct; the
  10-vs-20 distinction is delivered solely by the `iw` channel token.
- **FEC k/n is self-describing:** wfb embeds `fec_k`/`fec_n` in its session packets,
  so the ground `wfb_rx` learns them automatically. No ground-side reconfig is needed
  for any FEC change (video or tun/tlm).
- **DL coordination is already enforced.** `checkDynamicLinkLock` (`src/config/lock.cpp`)
  rejects PATCHes that write `link.mcs`, `link.txpower`, `link.fec`, `link.width`
  (plus `video.bitrate`/`qpDelta`/`roi`) while `dynamicLink.enabled`. So the hot path
  never arbitrates with `dl_applier`. `channel` and `mtu` are *not* locked and remain
  changeable under DL. `stbc`/`ldpc` are not locked (DL pins them off on the video tx
  it controls — pre-existing nuance, out of scope here).

## Field → mechanism matrix

| Link field | Mechanism on `/apply` | Affects | Drops air link? |
|---|---|---|---|
| `channel` | `iw set channel` (NIC) | all (PHY) | yes → defer ~200 ms |
| `width` | `iw set channel <HTmode>` (NIC) **+** `CMD_SET_RADIO` bandwidth → video | NIC + video radiotap | yes → defer ~200 ms |
| `txpower` | `iw set txpower` (NIC, driver-scaled) | all (PHY) | no → immediate |
| `mtu` | `ip link set … mtu` (NIC) | all | no → immediate |
| `mcs` | `CMD_SET_RADIO` → video :8000 | video only | no |
| `fec.k/n` | `CMD_SET_FEC` → video :8000 | video only | no |
| `stbc`/`ldpc` | `CMD_SET_RADIO` → video :8000 | video only | no |
| `linkId` | **full restart** (shared `-i` across all instances) | all | yes |
| `wlanAdapter` | **full restart** (different NIC) | all | yes |
| tun/tlm params | **never changed** — boot-once constants | — | — |

## Components

Four new/changed units, each independently testable.

### 1. `WfbControlClient` (new — `src/translate/wfb_control.{hpp,cpp}`)

Owns the UDP wire protocol to a `wfb_tx` control socket. Vendors `cmd_req_t`/`cmd_resp_t`
(copied from `wfb-ng/src/tx_cmd.h`).

- Constructed with `(addr = "127.0.0.1", port)`.
- `Result setRadio(stbc, ldpc, shortGi, bandwidth, mcs, vhtMode, vhtNss)`
- `Result setFec(k, n)`
- `Result { bool ok; std::string error; }`
- Internals mirror `dl_backend_tx.c`: connected UDP socket, `req_id = htonl(counter++)`,
  500 ms `SO_RCVTIMEO`, drain stale replies, match `req_id` on recv, return rc.
- Depends on: nothing in fpvd (pure socket + structs).

### 2. `radio-tune.sh` (new — `scripts/radio-tune.sh`)

Runs **only** the `iw`/`ip` line for one action. Invoked as
`radio-tune.sh <channel|txpower|mtu>`.

- Inputs (env): `FPVD_IFACE`, `FPVD_DRIVER` (passed in from boot `RadioInfo` — no
  re-detection), plus `FPVD_CHANNEL`/`FPVD_WIDTH`/`FPVD_TXPOWER`/`FPVD_MTU`.
- `channel` → `iw "$FPVD_IFACE" set channel <C> <HT20|HT40+|10MHz>` (covers
  channel + width NIC side, same switch as `radio-up.sh`).
- `txpower` → driver-scaled `iw … set txpower fixed` (`88XXau → *-100`, else `*50` —
  lifted from `radio-up.sh`).
- `mtu` → `ip link set "$FPVD_IFACE" mtu <M>`.
- The driver-specific txpower scaling lives here, kept in sync in spirit with
  `radio-up.sh` (boot path).

### 3. `tuneRadio()` (new in `src/supervise/radio.cpp`)

Sibling to `bringUpRadio()`:
`Result tuneRadio(scriptPath, action, const Config&, const RadioInfo&)`. Forks/execs
`radio-tune.sh <action>` with the `FPVD_*` env vars (including `FPVD_IFACE`/`FPVD_DRIVER`
from the stored `RadioInfo`), captures stderr, returns `ok`/exit/stderr. Reuses the
existing fork/pipe pattern in `radio.cpp`.

### 4. `classifyLinkChange()` (new in `src/config/diff.{hpp,cpp}`)

Pure function `LinkChange classifyLinkChange(const Config& old, const Config& neu)`:

```
struct LinkChange {
    bool nicChannel;    // channel || width  (air-link-dropping)
    bool nicTxpower;    // txpower
    bool nicMtu;        // mtu
    bool videoRadiotap; // mcs || stbc || ldpc || width
    bool videoFec;      // fec.k || fec.n
    bool fullRestart;   // linkId || wlanAdapter
};
```

No I/O — trivially unit-testable.

### tun/tlm constant change (`src/translate/wfb.cpp`)

`commonTx` stops reading `c.link.*` for the tun/tlm roles. Those become hardcoded
constants, still sharing `-i linkId`:

- `mcs = 0`, `fec k = 3 / n = 5`, `-B 20` (HT20), `stbc = 0`, `ldpc = 0`.

Video tx keeps reading `c.link.*` and keeps `-C 8000`.

> Note: `mcs=0` + 3/5 FEC is more robust than today's tun/tlm (mcs=1, k8/n12) — better
> for the low-rate control/telemetry links.

## `apply()` flow

`apply()` (`src/daemon.cpp`) keeps its validate → diff → persist front matter, then
gates dispatch:

```
apply(reallyRestart):
  lock(mu_)
  validate(pending_)                              # 400 on error          [unchanged]
  subs = diffSubsystems(effective_, pending_)     # encoder/telemetry/dl/services [unchanged]
  link = classifyLinkChange(effective_, pending_) # [new]

  persist overlay; effective_ = pending_; rewriteWaybeamJson()   [unchanged]
  version_++

  needsRebuild = subs.encoder || subs.telemetry || subs.dynamicLink
              || !subs.servicesAffected.empty() || link.fullRestart

  if reallyRestart && needsRebuild:
      # EXISTING full-restart path — unchanged.
      # stopAll; orch_ = {}; if subs.radio: bringUpRadio(); seedOrchestrator();
      # startAll; reconcileBeamforming(). Applies link params via re-exec + bringUpRadio.
      ...
      return

  if reallyRestart:   # NEW hot path — change is purely hot-applicable link fields
      # (A) immediate, non-link-dropping
      if link.nicTxpower: tuneRadio("txpower")
      if link.nicMtu:     tuneRadio("mtu")
      if link.videoFec:   wfbCtl(8000).setFec(fec.k, fec.n)
      if link.videoRadiotap && !link.nicChannel:   # mcs/stbc/ldpc, no width change
          wfbCtl(8000).setRadio(stbc, ldpc, 0, modWidth(width), mcs, false, 1)

      # (B) link-dropping (channel and/or width) — deferred ~200 ms
      if link.nicChannel:
          lastApply_ = ok (optimistic); version already bumped
          detach thread:
              sleep 200ms                       # let HTTP response flush
              lock(mu_)
              tuneRadio("channel")              # NIC retune first
              if width changed:                 # THEN video radiotap
                  wfbCtl(8000).setRadio(stbc, ldpc, 0, modWidth(width), mcs, false, 1)
              on failure → lastApply_.ok=false, .error=...
          return ok                              # immediate response

      lastApply_ = ok; return

  else:   # reallyRestart == false
      # reseed orchestrator specs only — unchanged (dry config load path)
```

Properties:

1. **Hot path is a fast lane** taken only when the change is purely hot-applicable link
   fields. Anything bundled with an encoder/telemetry/dl/service change — or a
   `linkId`/`wlanAdapter` change — falls through to today's untouched full-restart path.
   Subsystem restart behavior is literally unchanged (explicitly out of scope).
2. **No wfb/tun/tlm restart** on channel/width/txpower/mtu/mcs/fec/stbc/ldpc.
3. **Defer scoped to NIC channel retune only** (channel/width). The 200 ms flush window
   is preserved because the air link still drops when the NIC channel changes; the
   worker now runs `tuneRadio` + video `setRadio` instead of `stopAll`/`bringUpRadio`/
   `startAll`.
4. **Width ordering:** NIC retune (`iw … HT40+`) happens *before* the video radiotap
   `setRadio(bandwidth=40)` in the deferred worker, so video never injects 40 MHz frames
   on a 20 MHz NIC.
5. mcs/fec/stbc/ldpc apply inline with zero link drop (wfb is built for live rate
   changes — the dl_applier path proves this).

## Error handling & semantics

**Best-effort, sequential, reported — not transactional.** `iw`/control changes can't be
rolled back atomically; apply each in order and surface the first failure.

1. **Synchronous hot path:** apply each changed field in sequence. On the first failure,
   **stop**, set `lastApply_ = {ok:false, error:"<field>: <stderr/rc>"}`, return
   `ApplyResult{ok:false, radioError:...}` (mirrors today's `bringUpRadio`-failure return
   at `daemon.cpp:224`). Config is already persisted — consistent with current behavior
   where persist precedes restart.
2. **Deferred channel/width worker:** can't affect the already-sent HTTP 200. Mirrors the
   current deferred-retune contract (`daemon.cpp:204-207`): on failure sets
   `lastApply_.ok=false` + `.error`, visible on next `GET /status`. Optimistic
   `lastApply_.ok=true` set before detaching.
3. **`WfbControlClient` failure modes** (→ `Result{ok,error}`):
   - socket/connect error → `ok:false`.
   - recv timeout (500 ms, no reply) → `ok:false, error:"set_radio: timeout"` (usually
     video `wfb_tx` not running or `-C` port mismatch).
   - non-zero `rc` echoed by wfb_tx → `ok:false, error:"set_radio: rc=<errno>"`.
   - stale-reply drain + `req_id` match prevents mismatching a late reply.
4. **Partial-apply visibility:** non-atomic by design. If txpower succeeds but video
   `setRadio` times out, effective_ already reflects new config but the radio is
   half-updated. Reported in `lastApply_.error`; operator re-`/apply`s. Re-applying the
   same values is safe — `iw`/`setRadio` are level-triggered, not edge. **This
   non-atomicity is documented, not engineered around.**
5. **Common case** (txpower/mtu) either succeeds or prints to stderr, captured via the
   `radio.cpp` fork/pipe like `bringUpRadio`.

## Testing

1. **`classifyLinkChange()`** — pure table-driven unit tests
   (`tests/unit/test_link_classify.cpp`): each field toggled in isolation asserts the
   right flags; no change → all false.
2. **`WfbControlClient`** — against a localhost fake UDP server
   (`tests/unit/test_wfb_control.cpp`): assert exact wire bytes (`req_id` htonl'd,
   `cmd_id`, packed payload offsets/values, total length); echo `rc=0` → ok; `rc=EINVAL`
   → fail; silence → 500 ms timeout → fail; stale-then-correct `req_id` → drains + matches.
3. **`tuneRadio()`** — stub script records argv + `FPVD_*` env to a temp file
   (`tests/unit/test_tune_radio.cpp`): assert correct action arg + env; non-zero exit →
   `ok:false` + captured stderr. Reuses the `bringUpRadio` test pattern.
4. **`apply()` dispatch gate** — with fake orchestrator/radio: txpower-only apply calls
   `tuneRadio("txpower")` and does **not** rebuild the orchestrator; width apply schedules
   the deferred worker; linkId apply takes full-restart; encoder+txpower apply takes
   full-restart (gate). Assert no wfb bounce on the hot path.
5. **`radio-tune.sh`** — shellcheck + dry-run harness stubbing `iw`/`ip` on `PATH`:
   assert exact command line per action, including txpower scaling and
   `10MHz`/`HT20`/`HT40+` tokens.
6. **Manual on-hardware checklist** (not automated): txpower change reflected in
   `iw … get txpower`, video uninterrupted, `pidof` stable; mcs change via `CMD_GET_RADIO`
   echo / wfb_tx log, link up; channel change both ends retune, HTTP response returned
   before drop, `lastApply` ok on `/status`.

**Out of scope:** simulating RF, the rtl88x2eu driver, the ground station, and
granular subsystem (encoder/telemetry) restarts (those keep today's full-rebuild behavior).

## Open items / follow-ups

- The non-wfb subsystem restart still rebuilds the whole orchestrator (bouncing wfb as
  collateral) for an encoder/telemetry-only change. Intentionally left as-is; a future
  refactor could make those granular.
