# Unified Config API — Design

**Status:** Draft for review
**Date:** 2026-06-09
**Branch:** `feat/unified-config`

## Problem

Today a config client on the ground station juggles **three surfaces**:

- `GET/PATCH /config` — GS-local config (Python schema).
- `GET/PATCH /air/config`, `POST /air/apply` — an opaque proxy to the drone fpvd (C++ schema).
- `PATCH /link` + `POST /link/apply` — the shared radio params, coordinated to both ends.

The primary consumer is **PixelPilot** (the GS video player + its config menu). It does
single-field edits then apply, and should not have to know *where* a setting lives or which
of three endpoints to call. Splitting the surface across three endpoints — each with its own
schema, error shape, and apply semantics — pushes routing logic into every client.

## Goal

One unified config tree on the GS. PixelPilot sees only **`/config`**, **`/apply`**, and
**`/status`**. The GS owns the routing logic — for each field it knows whether the change
applies to itself (GS), to the drone, or to both — and the apply mechanics are hidden behind
the single `/apply`.

### Non-goals

- The drone fpvd remains a fully independent daemon with its own `/config`, `/apply`,
  `/status`, schema, and persisted overlay. It is **authoritative** over its own config and
  **self-restores** on reboot. We are unifying the *front door*, not collapsing the drone into
  a dumb applier.
- `/status` is **not** unified. Each daemon keeps reporting its own runtime; the GS `/status`
  carries a `drone` summary sub-block (see below).
- Primarily a config-surface refactor. It also bundles one drone-side capability — making the
  per-MCS TX power curve config-driven (per-radio default + operator override) and exposing it in
  `/status.radio` (see [Drone per-MCS TX power curve](#drone-per-mcs-tx-power-curve)) — plus the
  schema clarifications that fall out of the merge. No other new control features.

## Consumer-facing surface

| Endpoint | Drone daemon | GS daemon | PixelPilot uses |
|---|---|---|---|
| `GET/PATCH /config`, `POST /apply` | still exist (drone-local, authoritative) | **unified front door** (routes to both) | **GS only** |
| `GET /status` | kept (drone-local runtime) | kept (GS runtime + drone summary) | GS `/status` |
| `/link`, `/link/apply`, `/air/*` | — | **removed** (folded into `/config`+`/apply`) | — |

PixelPilot's entire vocabulary becomes `/config`, `/apply`, `/status` — all on the GS. The
drone's own `/config` / `/status` remain for direct/CLI/debug access, but PP never needs `/air`.

## Tree shape — Option C (feature tree, side-split inside split features)

The merge cannot use "section = side" because a single feature now spans both ends. We keep a
**feature-organized** tree, and where a feature is genuinely split-ownership we expose the
split **structurally** (a `gs` / `drone` sub-bucket, with shared fields at the node's top
level). Wholly-owned sections need no sub-buckets.

Routing convention PixelPilot can rely on:

- `link.*` (top), `adaptiveLink.enabled` → **BOTH**
- `link.gs.*`, `adaptiveLink.controller.*`, `wfb.*`, `pixelpilot.*`, `droneLink.*` → **GS**
- `link.drone.*`, `adaptiveLink.applier.*`, `video.*`, `image.*`, `telemetry.*`,
  `recording.*`, `services.*` → **DRONE** (stale/grayed when the drone is unreachable)

### Full `GET /config`

```jsonc
{
  // ---- response metadata, read-only (not settable) ----
  "_meta": {
    "droneReachable": true,
    "droneLastSeen": "2026-06-09T10:31:04Z",
    "droneStale": false           // true => link.drone / applier / video / … are last-seen
  },

  // ===== RADIO LINK (split feature) =====
  "link": {
    // shared  →  BOTH  (coordinator: GS-first, push drone when reachable; GS-only if not)
    "channel": 132,
    "width": 20,
    "linkId": 7669206,
    "beamforming": { "enabled": false },

    "gs": {                       // → GS  (the GS = receiver station)
      "region": "US",
      "rxpower": 22,              // dBm, the GS card power. 0..30 per radioProfile
      "wlans": "auto"
    },
    "drone": {                    // → DRONE  (the drone = transmitter station; stale when unreachable)
      "mcs": 3,
      "txpower": 25,              // dBm, static baseline (adaptive-off). 0..30 per radioProfile
      "txpowerCurve": null,       // dBm[8] per-MCS curve (adaptive-on), or null => detected-radio default
      "fec": { "k": 8, "n": 12 },
      "stbc": false,
      "ldpc": false,
      "mtu": 1500,
      "wlanAdapter": null
    }
  },

  // ===== ADAPTIVE LINK (split feature) =====
  "adaptiveLink": {
    "enabled": false,             // → BOTH  (hard-gated on drone reachability, on AND off)

    "controller": {               // → GS  (the brain that DECIDES — operating envelope)
      "maxMcs": 5,
      "radioProfile": "m8812eu2",
      "droneAddr": null,          // null => host from droneLink.endpoint
      "dronePort": 9999,
      "tuning": {}                // opaque advanced policy passthrough
      // bandwidth is read from link.width (10/20 → BW20, 40 → BW40); videoStreamId is the
      // internal constant "video"; txpower is gone (v3 wire is MCS-only — power is drone-owned).
    },
    "applier": {                  // → DRONE  (OBEYS + GS-lost failsafe; stale when unreachable)
      "healthTimeoutMs": 10000,
      "interleavingSupported": true,
      "minIdrIntervalMs": 500,
      "applyStaggerMs": 50,
      "applySubPaceMs": 5,
      "osd": { "enabled": true, "debugLatency": false },
      "roiQp": { "thresholdKbps": 6000, "lowAnchorKbps": 2000, "floor": -24, "step": 3 },
      "failsafe": {               // applied when the drone loses the GS for healthTimeoutMs
        "mcs": 1, "k": 8, "n": 12, "depth": 1,
        "bandwidth": 20, "bitrateKbps": 2000
        // note: no txPowerDbm — failsafe power derives from txpowerCurve[failsafe.mcs]
      },
      "bitrate": { "minBitrateKbps": 1000, "maxBitrateKbps": 24000 },  // drone-local compute (Phase 3a)
      "fec":     { "baseRedundancyRatio": 0.5, "blocksPerFrame": 2.0, "kMin": 2, "kMax": 50 }  // drone-local
    }
  },

  // ===== VIDEO PIPELINE (wholly drone)  →  DRONE =====
  "video": {
    "codec": "h265",              // must match pixelpilot.codec (operator-maintained, not enforced)
    "resolution": "1920x1080",
    "fps": 60,
    "bitrate": 8192,
    "rcMode": "cbr",
    "gopSize": 1.0,
    "qpDelta": -4,
    "roi": { "enabled": true, "qp": 0, "center": 0.4, "steps": 2 }
  },
  "image":     { "mirror": false, "flip": false, "rotate": 0 },
  "telemetry": { "router": "msposd", "serial": "ttyS2", "osdFps": 20, "baud": 115200 },
  "recording": { "enabled": false, "format": "ts", "mode": "mirror", "maxSeconds": 300, "maxMB": 500 },
  "services":  {},

  // ===== GROUND STATION (wholly GS)  →  GS =====
  "wfb": {
    "profile": "gs",
    "mavlink": { "peer": "connect://127.0.0.1:14550" },
    "raw": {}
  },
  "pixelpilot": {
    "enabled": true,
    "bin": "/usr/bin/pixelpilot",
    "env": {},
    "configPath": "/etc/pixelpilot.yaml",
    "osdConfigPath": "/etc/pixelpilot/osd.json",
    "screenMode": "1920x1080@60",
    "videoScale": 1.0,
    "codec": "h265",              // must match video.codec (operator-maintained, not enforced)
    "rtpPort": 5600,
    "rtpJitterMs": 1,
    "dvr": {
      "framerate": 60, "dir": "/media/dvr", "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
      "fmp4": true, "sequencedFiles": true, "osd": false, "mode": "raw",
      "maxSizeMb": 4000, "reencCodec": "h264", "reencBitrate": 8000,
      "reencFps": 30, "reencResolution": "1080p"
    },
    "extraArgs": []
  },
  "droneLink": {                  // renamed from drone.endpoint — GS infra (how the GS reaches the drone)
    "endpoint": "http://10.5.0.10:8080"
  }
}
```

The **IDR relay** (PixelPilot keyframe-request forwarding) is **not** in the config tree — see
"IDR relay" below.

## Routing is structural — no schema endpoint

Because Option C exposes the side structurally, PixelPilot derives routing from the tree itself.
There is **no** `/config/schema` (or any descriptor) endpoint.

- **Split features** — `link.gs.*` / `adaptiveLink.controller.*` → GS; `link.drone.*` /
  `adaptiveLink.applier.*` → DRONE; node-top-level (`link.channel`, `adaptiveLink.enabled`) →
  BOTH.
- **Wholly-owned sections** — a small, stable, documented section→side map:
  `video` / `image` / `telemetry` / `recording` / `services` → DRONE;
  `wfb` / `pixelpilot` / `droneLink` → GS. PixelPilot, being our own consumer, knows this map by
  convention. Combined with `_meta.droneStale`, it tells PP which subtrees to gray out during a
  battery swap.
- **Validation is server-side** — `PATCH /config` is the source of truth: it validates each leaf
  (range, unit, enum) and returns `400` with the offending field, which PP surfaces. Reactive
  validation is sufficient for a config menu; PP does not pre-validate. Units and ranges (e.g.
  txpower dBm 0–30) are documented for display hints, and `mW` may be shown as a label beside the
  dBm value (dBm is the stored/validated unit).
- **Apply policy** (gate vs degrade) is enforced by the GS at apply time. The one gated field
  (`adaptiveLink.enabled`) is known by convention and also reported in the `/apply` result, so PP
  can warn proactively and react to the outcome.

## Storage & apply model — C-1 composed facade

The drone stays **authoritative** over its own config and **self-restores** from its persisted
overlay on reboot. The GS unifies only the front door:

- **Read** — `GET /config` merges GS-local config with a live `GET /air/config` fetch from the
  drone. The GS keeps a **thin read-only last-seen snapshot** of the drone subtree; on every
  successful drone read it refreshes the snapshot. When the drone is unreachable it serves the
  snapshot with `_meta.droneStale: true` (drone subtrees are last-seen, not blank). Shared
  fields (`link.channel/width/linkId/beamforming`) and `adaptiveLink.enabled` are always
  **live** because the GS holds its own copy.
- **Write** — `PATCH /config` routes each leaf to the right side's pending store
  (GS-local pending, or the drone's pending via the proxy).
- **Apply** — `POST /apply` diffs pending vs effective and fires only the lane(s) the changed
  leaves belong to.

### Apply lanes

| Lane | Fields | Mechanism |
|---|---|---|
| **GS-local** | `wfb`, `pixelpilot`, `adaptiveLink.controller` | GS `/apply` (bounce runner / restart PixelPilot) |
| **Shared link** | `link.channel/width/linkId/beamforming` | coordinator — GS-first, best-effort drone push, live-retune-vs-bounce |
| **Drone** | `link.drone.*`, `adaptiveLink.applier.*`, `video`, `image`, `telemetry`, `recording`, `services` | proxied to drone `/apply` |

The old `409 "use /link/apply"` guard and the `applyTo: gs|both` flag are **gone** — the
coordinator lane runs inside `/apply`, and the shared-link change auto-degrades to GS-only when
the drone is unreachable (no separate endpoint needed).

### Per-field apply policy

Apply policy is **per-field**, enforced by `/apply`:

| Field group | Drone unreachable behavior |
|---|---|
| Shared link (`channel`/`width`/`linkId`) | **soft-degrade** → apply GS-only, report `droneApplied:false` |
| `adaptiveLink.enabled` (on *and* off) | **hard-gate** → reject the apply; change nothing |
| Other drone-only fields | require the drone (the change is drone-routed) |

`POST /apply` returns a per-lane result so PixelPilot can show exactly what happened
(e.g. "applied on GS, drone offline").

## Overlap resolutions

Only two top-level keys existed in both schemas — `link` and `dynamicLink`. Their fields fall
into a few overlap types; the resolutions:

### Type 1 — truly shared link fields (`channel`, `width`, `linkId`)

One logical value, lives on both, must match to connect. Handled by the **shared-link
coordinator** lane: GS-local-first, push the changed shared keys to the drone when reachable.
**Drone unreachable → apply GS-only.** This is coherent because you cannot push a new channel
to a drone you cannot reach, and the drone self-restores its own channel on reboot. No staging,
no reconnect-push.

### Type 2 — `link.txpower` collision → unit unification, named by station role

In a *flat* merge the two `txpower` fields would collide, but **Option C already disambiguates
them structurally** (`link.gs.*` vs `link.drone.*`), so no rename is needed to resolve the
collision. Both *are* TX power; we name them by **station role** for operator clarity:

- `link.gs.rxpower` — the **GS** card power (the GS is the *receiving* station — its job is to
  receive the video downlink).
- `link.drone.txpower` — the **drone** radio power, static baseline (the drone is the
  *transmitting* station). Adaptive-off.

Both in **dBm**, validated against the `radioProfile`'s `tx_power_min/max_dBm` (0–30 for
BL-M8812EU2). This retires the legacy `1..63` driver-units representation and the `radio-tune.sh`
`×50 / ×-100` scaling — the static drone txpower renders via the same `iw … set txpower fixed
<dBm×100>` the adaptive path already uses. When adaptive link is on, the per-MCS
[`txpowerCurve`](#drone-per-mcs-tx-power-curve) drives the drone power instead of the static
baseline.

### Type 3 — `dynamicLink` collision → `adaptiveLink.controller` / `.applier`

The name `dynamicLink` meant two different subsystems. Restructured into:

- `adaptiveLink.controller` (GS) — the brain that decides (operating envelope + policy).
- `adaptiveLink.applier` (drone) — obeys decisions + the GS-lost failsafe.
- `adaptiveLink.enabled` — **one unified toggle** that arms both halves.

**Arming requires the drone reachable, on *and* off** (hard-gate both directions). Rationale:
the dangerous state is half-armed (a GS controller streaming decisions to an unarmed drone), so
toggling is gated. It self-heals across a battery swap: the GS controller keeps running
(streaming into dropped UDP, harmless); the drone reboots, self-restores its armed state, and
simply resumes acting on the GS's incoming MCS decisions.

**No arm-order constraint.** There is no HELLO handshake (removed post-3b). Order is still
irrelevant because: decisions are UDP, so any sent before the applier is up are dropped (no
listener); and the drone boots on its **static link config**, switching only when a decision
arrives, with the applier computing **per-MCS power locally from the curve** for every decision
— so even the first decision it receives applies a sane power. `/apply` can fire both lanes in
any order.

**Controller is a pure MCS selector (Phase 3a/3b).** The v3 decision wire carries **MCS only**;
the drone computes bitrate / FEC / depth / TX-power locally from that MCS. The field set is
trimmed to match what the GS still actually uses:

| Field | Disposition |
|---|---|
| `maxMcs`, `radioProfile`, `droneAddr`, `dronePort`, `tuning` | **kept** — the GS selector + UDP target |
| `bandwidth` | **removed** — read from the shared `link.width` (`10/20 → BW20`, `40 → BW40`); one source of truth |
| `videoStreamId` | **removed** — internal constant `"video"` (the stats-stream selector) |
| `txpower.{min,max}` | **removed** — commanded nothing on the MCS-only wire; per-MCS power is the drone's `txpowerCurve` |

The GS still uses per-MCS power for RSSI/EIRP normalization, but **for this session it keeps its
existing `tuning.rssi_norm.tx_power_dbm_by_mcs` mirror unchanged** — `controller.txpower` is
removed only because it commanded nothing on the MCS-only wire. Sourcing the GS normalization
curve from the drone's `/status.radio.txpowerCurve` (retiring the hand-maintained mirror) is a
**future convergence, out of scope here** — the drone still publishes the curve, the GS just
doesn't consume it yet.

On the **applier**, the current drone schema adds `bitrate` (`minBitrateKbps`, `maxBitrateKbps`)
and `fec` (`baseRedundancyRatio`, `blocksPerFrame`, `kMin`, `kMax`) — the Phase-3a drone-local
compute — and drops the obsolete `mavlinkEnable`.

**`tuning` stays opaque this session.** `controller.tuning` remains the untyped gs.yaml-shaped
passthrough it is today (the curated keys overlay it). The live subset is now small enough that
promoting the operator-facing knobs to typed `controller.*` fields — keeping an opaque escape
hatch only for the deep `learned_prior` / `smoothing` internals — is a sensible **follow-up**,
but out of scope here. Note: the `api.md` `tuning` table is stale (it documents deleted
SNR/hysteresis/FEC keys) and should be rewritten to the live subset when that promotion happens.

`adaptiveLink.controller` (operating envelope the brain selects within) and
`adaptiveLink.applier.failsafe` (the GS-lost fallback) are **independent by design**. The
failsafe is *not* a per-decision clamp — it engages only on the watchdog trip (below). It is
*meant* to be low/conservative, so a guard requiring `failsafe ≥ controller bound` would defeat
its purpose. No cross-check.

**Verified behavior** (`drone/src/dynlink/watchdog.cpp`, `controller.cpp`):

- Every decision packet from the GS calls `notifyDecision()` (resets the timer, clears the trip).
- `tick()` trips when `now − lastDecision ≥ healthTimeoutMs` — i.e. the drone has gone deaf to
  the GS for `healthTimeoutMs`.
- A trip applies the failsafe (`dispatchTxSafe` + `enc.applySafe(bitrateKbps)`) and emits the
  OSD `"WATCHDOG safe_defaults"` event.
- The `everSeen_` guard means it does **not** trip before the first decision is ever received —
  it is "heard the GS, then lost it," not a startup state. The next decision clears the trip.

### Type 5 — codec coupling: **skipped**

`video.codec` and `pixelpilot.codec` must match (decoder must match encoder) but stay as **two
independent fields**. Keeping them in sync is the operator's responsibility — no validate-equal,
no unification.

## Drone per-MCS TX power curve

The drone's per-MCS TX power (the PA anti-overdrive curve, today hardcoded as `kTxPowerDbmByMcs`
in `drone/src/dynlink/txpower_curve.hpp`) becomes **hardware-defaulted but operator-overridable**,
and is surfaced in `/status`.

### Config — `link.drone.txpowerCurve`

An 8-element dBm array (MCS 0–7), or `null`. It reuses the codebase's `null = auto` convention
(cf. `link.drone.wlanAdapter`):

- `null` (the default in `defaults.json`) → use the **detected radio's built-in default curve**.
- 8-element array → explicit operator override.

**Resolution at boot/apply:** `override (if set) → detected-radio default → conservative fallback`.

The drone keeps a **baked-in registry of per-radio default curves**, keyed by the radio it already
detects (`adapterId` `0bda:8812` first, then `driver`), seeded with the current `m8812eu2` curve
`{29,28,25,23,19,19,19,19}`. New radios are characterized by adding a registry entry; the curve
stays overridable for any radio meanwhile. These per-radio defaults are **baked-in, not in
`defaults.json`** (they are hardware-dependent — `defaults.json` just carries `null`).

The resolved curve threads into `DlRuntimeConfig`, so `local_compute.cpp` (per-decision power)
and the failsafe path (`controller.cpp` → `applySafe(txpowerDbmForMcs(failsafe.mcs))`) read it
instead of the `constexpr` global.

**Validation:** exactly 8 entries, each within the radio's `tx_power_min/max_dBm` (0–30).
Monotonicity is *not* enforced — a custom or flat curve is allowed.

### Status — `/status.radio.txpowerCurve`

The drone `/status.radio` block gains the **effective resolved** curve plus its source, so the
operator and the GS can read the drone's real per-MCS power (directly useful for the pending GS
RSSI/EIRP-normalization work):

```jsonc
"radio": {
  "driver": "88XXau", "iface": "wlan0", "adapterId": "0bda:8812",
  "txpowerCurve": [29,28,25,23,19,19,19,19],   // effective (resolved)
  "txpowerCurveSource": "m8812eu2"             // "override" | "<radio>" | "fallback"
}
```

Per-bandwidth curves (separate 20/40 MHz tables) are **out of scope** — a single 8-element curve,
as today.

## IDR relay — invisible infrastructure

The IDR/keyframe relay (PixelPilot → drone encoder, replacing the standalone `socat`
idr-forwarder) is **removed from config** and becomes always-on infrastructure:

- Listens on `0.0.0.0:11223`, forwards to `droneLink.endpoint` host:`11223` (hardcoded port).
- Runs as a standing part of the GS data plane, **decoupled from the adaptive-link controller**,
  so keyframe forwarding works on static *and* adaptive links.
- Harmless when the drone is unreachable (the `sendto` failure is already swallowed).

`idrForward` and `idrPort` are dropped from the schema entirely (previously under the GS
`dynamicLink` block). The relay must be lifted out of the controller's run loop
(`gs/fpvdgs/dynlink/controller.py`) into a standalone always-on relay.

## `safe` → `failsafe` rename

The drone config key `dynamicLink.safe` is renamed to `failsafe` (and surfaced as
`adaptiveLink.applier.failsafe` in the unified tree). The name `safe` reads like a ceiling/cap;
`failsafe` is the domain-standard RC/FPV term for lost-link fallback and matches the verified
behavior.

- **Scope:** JSON key only. The drone schema (`schema.hpp` `NLOHMANN_DEFINE` key,
  `validate.cpp`, `runtime_config.cpp` references), docs, and tests. Internal C++ symbols
  (`SafeDefaults`, `applySafe()`, `dispatchTxSafe()`) may stay — they still read fine.
- **Backward-compat:** a drone overlay persisted with `"safe": {…}` would silently fall back to
  defaults after the rename. Handled at the redesign cutover; alternatively the drone reads
  `failsafe` with a `safe` fallback for one release.

## `/status` (kept per-daemon)

Each daemon keeps its own `/status`. The GS `/status` extends its existing `drone` sub-block
(`reachable`, `dynamicLinkActive`, …) into a summary digest of whatever drone runtime
PixelPilot's menu needs (e.g. applier running, drone `lastApply.ok`), so PP reads a single
`/status` without resurrecting an `/air`-style proxy.

The drone `/status.radio` block also gains the resolved per-MCS `txpowerCurve` (+ source) — see
[Drone per-MCS TX power curve](#drone-per-mcs-tx-power-curve).

## Naming nit

`drone` is otherwise overloaded — `_meta.drone*` (reachability), `link.drone.*` (drone-side
link), and the GS infra endpoint. The GS infra field is renamed `drone.endpoint` →
`droneLink.endpoint` to disambiguate.

## Open questions

- Exact field set of the GS `/status` drone summary (driven by PixelPilot's menu needs).
- Whether `_meta` should carry an explicit `droneOwned` section list so PixelPilot's graying is
  data-driven instead of relying on the documented section→side convention.
- Migration mechanics for the `safe` → `failsafe` overlay key at cutover.
```