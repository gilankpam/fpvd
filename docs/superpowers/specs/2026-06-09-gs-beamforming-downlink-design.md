# GS Beamforming (Downlink) — Design

**Date:** 2026-06-09
**Status:** Approved design, pending implementation plan
**Scope:** Add a ground-station (GS) beamformee responder so the drone's existing
downlink TX-beamforming actually works, and wire `link.beamforming` through the
GS `/link/apply` coordinator so a single apply configures both sides.

## Problem

The drone already runs a beamformer sounding loop aimed at the GS
(`drone/src/supervise/beamforming.cpp`): it writes `bf_monitor_conf` + a
periodic `bf_monitor_trig` to sound the GS and applies a steering matrix via
`bf_monitor_en`. But explicit SU beamforming is a two-sided handshake:

```
drone ──NDPA──▶ GS      (announce sounding, addressed to GS MAC)
drone ──NDP───▶ GS      (sounding waveform; GS measures channel H)
GS ──Compressed BF Report──▶ drone   (the beamformee "echo")
drone: computes steering matrix Q from the report
drone ──beamformed video──▶ GS       (now with array gain)
```

The GS never sends the report, so the drone receives no channel info and
`bf_monitor_en` applies an identity matrix — **no gain** (the live drone's
`TXBF_CTRL` registers are all zero, consistent with "never received a report").

This design adds the missing GS half.

## Key facts established during design

1. **Same driver both sides.** Drone and GS both run the `8812eu`
   (`rtl88x2eu`, internally RTL8822E) driver. The monitor-mode beamforming
   feature lives in `hal/rtl8822e/rtl8822e_bf_monitor.c`, gated by
   `CONFIG_BEAMFORMING_MONITOR`, exposed via procfs nodes
   `bf_monitor_{conf,trig,en,rfinfo,rty_cnt}` under
   `/proc/net/rtl88x2eu/<iface>/`.

2. **The beamformee echo is hardware-automatic.** Arming `bf_monitor_conf`
   writes the remote (beamformer) MAC into `REG_ASSOCIATED_BFMER0_INFO_8822E`
   and sets `REG_SND_PTCL_CTRL_8822E = 0xDB` (bit 0 `R_WMAC_VHT_NDPA_EN`; the
   register's upper half holds `R_WMAC_VHT_CATEGORY`, `CSI_RPT_OFFSET`,
   `CSI_CHKSUM_DIS`, `NDP_RX_STANDBY_TIMER`). The WLAN-MAC hardware then
   assembles and transmits the VHT Compressed Beamforming report itself,
   within SIFS (~16 µs) of each NDP. Verified three ways: (a) no software path
   anywhere in the driver builds/sends a report — only NDPA-send and
   report-RX-parse exist; (b) the register fields are owned by the WMAC; (c)
   SIFS timing makes a software response physically impossible over USB.
   Therefore the GS responder needs **no sounding loop** — pure config.

3. **The VHT capability bits are the wrong availability signal.** In monitor
   mode `iw phy` reports all VHT beamformer/beamformee bits clear (observed
   `0x03c001b2`), because there is no association to negotiate them. The
   monitor-BF path bypasses capability negotiation and programs the TXBF
   registers directly. The correct availability check is the **presence of the
   `bf_monitor_conf` proc node**, which simultaneously proves the right driver,
   the feature compiled in, and the supporting chip.

4. **GS driver currently lacks the feature.** The GS's deployed `8812eu` was
   built *without* `CONFIG_BEAMFORMING_MONITOR` — no `bf_monitor_conf` node.
   Rebuilding/redeploying the GS driver with that flag is a **separate
   prerequisite handled out-of-band**; this design covers only the fpvd code.

5. **GS has two diversity RX cards.** The GS runs two monitor cards plus an AP
   `wlan0`. Beamforming is point-to-point to one MAC (SU TXBF index 0), so a
   single card is the beamformee peer.

## Decisions

| Decision | Choice |
|---|---|
| Direction | **Downlink only** (drone→GS). GS is a pure beamformee; no GS sounding loop. |
| Beamformee peer card | **Primary = `wlans[0]`** (first resolved card). The other card stays plain diversity RX. |
| MAC exchange | **Auto-exchange at `/link/apply`.** Each side learns the other's MAC at apply time; no manual MAC entry. |
| GS driver rebuild | **Separate prereq** (operator handles). fpvd code is correct regardless. |
| Capability handling | **Hard-reject** at apply: enabling BF on a card without the `bf_monitor_conf` node aborts the apply. |
| Approach | **A** — symmetric beamformee controller + coordinator-orchestrated exchange. |

### RF caveat (co-located diversity)

Diversity cards on an FPV GS are normally co-located (same mast;
multipath/polarization diversity), so steering the drone's beam toward the
primary card's MAC also benefits the secondary card. If the two antennas were
spatially separated and aimed differently, beamforming toward one could weaken
the other — out of scope for this design.

## Architecture

Four units. The drone side already exists and is unchanged except that it now
receives a pushed `link.beamforming` from the coordinator.

### 1. GS beamformee controller — `gs/fpvdgs/beamforming.py`

Mirrors the *shape* of the drone's `BeamformingController`, but beamformee-only:
no thread, no loop, no `bf_monitor_en`, no `ack_timeout`. Pure synchronous proc
writes.

```
class BeamformingController:
    __init__(proc_base="/proc/net/rtl88x2eu")

    supported(iface) -> bool
        # {proc_base}/{iface}/bf_monitor_conf exists

    reconcile(enabled: bool, iface: str, peer_mac: str) -> dict
        # enabled=False → if previously armed, write "0 00:00:00:00:00:00 0 0"; state=disabled
        # enabled=True  → not supported(iface): state=unsupported (+reason)
        #                 supported: write f"1 {peer_mac} 0 0"; state=active
        # idempotent: no-op if (enabled, iface, peer_mac) unchanged since last reconcile
        # any write failure → state=error (never raises into the apply path)

    status() -> dict
        # {requested, state, reason, iface, localMac, peerMac}
        # state ∈ {disabled, unsupported, active, error}
        # localMac = GS primary card MAC (what the drone sounds)
        # peerMac  = drone card MAC (whom the GS responds to)
```

Writing `bf_monitor_conf` with the drone's MAC arms the hardware auto-echo; the
controller does no per-sounding work.

### 2. Schema / config — `gs/fpvdgs/schema.py`

`beamforming` is already in `LINK_KEYS`, so `/link` PATCH already accepts it.
Add shape validation; keep the GS config minimal:

```
link.beamforming = { "enabled": bool }     # default: absent ⇒ disabled
```

- `remoteMac` is **not persisted on the GS** — it is volatile (changes if an
  adapter is swapped) and auto-resolved at apply time, surfaced only in
  `/status`. The *drone's* config still carries `remoteMac` because the
  coordinator pushes it there each apply; asymmetric per-side configs are fine.
- `validate_effective` gains: if `link.beamforming` present → must be a dict,
  `enabled` must be a bool, reject unknown sub-keys. **Pure schema, no I/O** —
  the capability check lives in the coordinator (it needs filesystem access).

### 3. Coordinator — `gs/fpvdgs/link.py`

`LinkCoordinator` gains a `beamforming` controller dependency and resolves the
primary iface via the existing `resolve_wlans` (`wlans[0]`). New `apply_link`
flow:

1. **Validate (existing, pure schema).** Shape/keys only.

2. **BF capability preflight — hard-reject.** If
   `pending.link.beamforming.enabled`:
   ```
   primary = resolve_wlans(pending)[0]
   if not beamforming.supported(primary):
       raise SchemaError(
         f"beamforming unavailable on {primary}: no bf_monitor_conf node "
         f"(GS driver lacks CONFIG_BEAMFORMING_MONITOR)")
   ```
   Apply aborts; nothing committed, nothing pushed.

3. **MAC exchange** (when BF enabled):
   - `gs_mac` = read `/sys/class/net/<primary>/address` (always available)
   - `drone_mac` = `drone.get_status()["beamforming"]["localMac"]` (needs the
     drone reachable; the HW matches the NDPA TA against this specific MAC, so a
     concrete peer MAC is required to arm the GS).

4. **Drone push** (`apply_to=="both"` and reachable). Pushed as its own
   category alongside the existing `DRONE_PUSH_KEYS` deltas, with the **MAC
   transformed, not echoed** — the drone receives the GS's MAC as its
   `remoteMac`:
   ```
   if beamforming changed:
       push["beamforming"] = {"enabled": enabled, "remoteMac": gs_mac}
   ```
   This is the one place the prior "beamforming is never pushed" rule changes —
   by design, and only with the cross-referenced MAC.

5. **GS-side reconcile — orthogonal to retune/bounce.**
   `beamforming.reconcile(enabled, primary, drone_mac)` is a proc write and must
   **not** bounce the video pipeline. `beamforming` is excluded from the
   change-set that drives the retune-vs-bounce decision:
   - `non_bf_changed = changed_link_fields − {beamforming}`
   - `non_bf_changed` empty → no retune, no bounce (render + commit only)
   - live-eligible → live `iw` retune
   - else → runner bounce

6. **Failure semantics** (capability = hard, everything else = best-effort,
   matching the existing coordinator):

   | Condition | Behavior |
   |---|---|
   | GS primary lacks node, `enabled=true` | **Hard reject** at step 2 (apply aborts) |
   | Drone unreachable | Can't learn `drone_mac` ⇒ can't arm GS beamformee, can't push. **Link still applies**; BF reported not-established. |
   | GS reconcile write fails | `state=error` in status/result; link apply unaffected |
   | GS RF apply (bounce) fails | Existing rollback: restore last-good cfg **and** reconcile BF back to last-good `enabled` |

7. **Result object** extends with a `beamforming` block:
   `{gsState, peerMac, localMac, dronePushed}` so `/link/apply` callers see
   exactly what happened.

The hard-reject is **GS-primary only** (local, deterministic). The drone is
verified to have the node; its reported BF state is merely surfaced in the
result, not used as a reject gate (that would couple the reject to drone
reachability).

### 4. Status — `gs/fpvdgs/status.py`

The `/status` assembler calls `beamforming.status()` and includes a block
mirroring the drone's (`drone/src/status.cpp:94`):

```
"beamforming": { "requested", "state", "iface", "localMac", "peerMac", "reason" }
```

## Data flow (enable, happy path)

```
operator ── PATCH /link {beamforming:{enabled:true}} ──▶ GS (pending updated)
operator ── POST /link/apply {apply_to:"both"} ──▶ GS coordinator
  ├─ validate (schema)
  ├─ capability preflight: supported(wlans[0])  ── fail ⇒ 4xx, abort
  ├─ gs_mac  = sysfs address of wlans[0]
  ├─ drone_mac = drone GET /status → beamforming.localMac
  ├─ drone PATCH /config {link:{beamforming:{enabled:true, remoteMac:gs_mac}}} + apply
  │     └─ drone reconcileBeamforming(): sounds gs_mac, harvests report, applies Q
  ├─ GS reconcile(enabled=true, wlans[0], drone_mac): write bf_monitor_conf "1 <drone_mac> 0 0"
  │     └─ GS HW auto-echoes compressed beamforming report to the drone
  ├─ RF retune/bounce only if non-BF link fields changed
  └─ commit; return {gsApplied, droneApplied, beamforming:{...}}
```

## Testing

TDD, matching repo conventions (`gs/tests/unit/`, pytest; the drone side already
has `drone/tests/integration/test_beamforming.cpp`).

**Controller** (`gs/tests/unit/test_beamforming.py`, tmp proc dir):
- `supported()` true/false by node presence
- enable → writes exactly `"1 <droneMac> 0 0"`, `state=active`
- disable → writes reset `"0 00:00:00:00:00:00 0 0"`, `state=disabled`
- no node → `state=unsupported`
- idempotent: unchanged params ⇒ no rewrite
- write failure ⇒ `state=error`

**Coordinator** (extend existing link-coordinator tests; stub drone client + tmp
proc dir):
- hard-reject: enable with unsupported primary ⇒ apply errors, no commit, no push
- MAC exchange: enable ⇒ reads `gs_mac`, fetches `drone_mac` from stub `/status`,
  pushes `{beamforming:{enabled,remoteMac:gs_mac}}`, reconciles GS controller
  with `drone_mac`
- orthogonality: BF-only change ⇒ `runner.restart` **not** called
- drone unreachable ⇒ link still applies, BF reported not-established
- disable ⇒ resets both sides

**Schema**: `beamforming` shape (enabled must be bool; reject unknown sub-keys).

**Hardware verification (empirical gate).** Unit tests prove the wiring; only
hardware proves the link. After the GS driver is deployed with
`CONFIG_BEAMFORMING_MONITOR` and BF enabled, confirm end-to-end: the drone's
`/status` `beamforming` goes `active` **and** `bf_monitor_rfinfo` populates a
non-zero report (proving the GS HW auto-echo fires), optionally sniffing the VHT
compressed-beamforming action frame transmitted by the GS.

## Out of scope

- Uplink / bidirectional beamforming (GS sounding the drone).
- GS driver build/deploy with `CONFIG_BEAMFORMING_MONITOR` (separate prereq).
- Arming the secondary diversity card as a second beamformee.
- Spatially-separated-antenna steering tradeoffs.
- Changes to the drone-side beamforming implementation (tracked separately;
  known issues there — premature CBR read timing, error-path leaving
  `running_=true` — are not addressed here).

## Prerequisite checklist (operator, out-of-band)

1. Enable `CONFIG_BEAMFORMING_MONITOR` in the GS `8812eu` build.
2. Rebuild + redeploy the GS driver.
3. Confirm `/proc/net/rtl88x2eu/<primary-monitor-iface>/bf_monitor_conf` exists
   on the GS (the primary is `wlans[0]`, a monitor card such as `wlx…` — **not**
   the `wlan0` AP).

Until step 3, the hard-reject correctly refuses to enable BF.
