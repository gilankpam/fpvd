# fpvd-GS — Design Spec

**Date:** 2026-06-02
**Status:** Draft for review
**Target:** OpenIPC SBC ground stations (aarch64, busybox init, Python 3.13 + Twisted on board)

## 1. Purpose

`fpvd` already owns config and runtime supervision of the FPV stack on the
**drone**. This spec defines its **ground-station peer**: a daemon — also named
`fpvd` — that owns the wfb radio config and supervises the wfb processes on the
GS, exposes the same HTTP+JSON management surface, and acts as the **single
front door** for all link/radio configuration (GS-local, drone, and the shared
overlap between them).

It replaces `/etc/init.d/S98wifibroadcast` (which launches the stock
`wfb-server`) with one init script and one supervisor process, mirroring how
drone-side fpvd replaced `S95waybeam`/`S98wifibroadcast`.

The name is deliberately shared with the drone daemon: `fpvd` is the FPV
supervisor *role*, which runs on both ends. The GS build is a separate,
Python implementation living in this same repo.

## 2. Problems being solved

1. **No GS-side management plane.** The GS radio is configured by editing
   `/etc/wifibroadcast.cfg` + restarting an init script. There is no
   programmatic, networked API and no selective lifecycle control — unlike the
   drone, which now has one via fpvd.
2. **Stale, fragmented config client.** `gsmenu.sh` (rendered by PixelPilot,
   GPIO-driven) configures the drone over SSH with `wifibroadcast cli` — a path
   that assumes the old OpenIPC `wfb.yaml` stack and **no longer matches an
   fpvd drone**. It also `sed`s `/etc/wifibroadcast.cfg` locally. Two transports,
   two formats, and hand-rolled cross-box coordination.
3. **Unmanaged overlap.** Channel/width/region must agree on **both** ends or
   the link breaks. Today `gsmenu` fakes this by doing two things at once (SSH
   the drone *and* sed+restart the GS) with no atomicity, and exposes two
   separate channel knobs (`air_channel`, `gs_channel`) that can drift.
4. **No home for future link orchestration.** Coordinated channel switch,
   bandwidth switch, and beamforming have nowhere to live on the GS.

## 3. Scope

### In scope (v1)

- A new Python GS daemon, `fpvd`, an independent peer to the drone fpvd.
- Owns the GS wfb radio config (sparse user overlay over a baked default) and
  supervises the wfb data plane.
- **Replaces** `wfb-server` — but builds on the `wfb_ng` library rather than
  reimplementing it (see §4).
- HTTP+JSON API as the **single front door** with three groups: GS-local config,
  opaque drone passthrough (`/air/*`), and a link coordinator (`/link`) that
  owns the shared overlap params.
- `/apply` and `/link/apply` are **bounce-based** in v1 (a brief RX drop is
  acceptable for manual changes).

### Out of scope (v1)

- **Coordinated/seamless switching** — timed channel/bandwidth/beamforming
  handshake with auto-revert, no link teardown. Designed for; not built.
  `/link` is its home.
- Live `iw` retune without bouncing the runner.
- Managing any GS service other than wfb (msposd, gstreamer-tee, pixelpilot,
  dvrui, dynamic-link-gs keep their own init scripts).
- A generic/arbitrary user-services framework (explicitly dropped).
- Modeling the drone's config schema in the GS daemon (the proxy is opaque).

## 4. Architectural decisions (recap of brainstorming)

1. **Implementation: new Python service.** The GS ships only Python 3.13 (no C
   toolchain), and `wfb-server`/dynamic-link are already Python/Twisted. Mirrors
   fpvd's design, not its C++ code.
2. **Replaces `wfb-server`, built on the `wfb_ng` library.** No `wfb-server`
   process and no `S98wifibroadcast`. A small runner we ship imports `wfb_ng`
   (`init_wlans`, `parse_services`, `AntStatsAndSelector`, the JSON/MsgPack stats
   factories) and runs the orchestration under its own Twisted reactor, driven by
   the cfg we render. TX antenna diversity and the `:8103`/`:8003` stats APIs keep
   working unchanged, so **dynamic-link-gs and wfb-cli need zero changes**.
3. **Runtime model: supervisor + runner child (two processes).**
   - *Supervisor* (`fpvd`): the new code and source of truth — config store, HTTP
     API, process supervision, cfg rendering, `/link` coordination, `/air` proxy.
     Never imports `wfb_ng`; unit-testable without a radio.
   - *Runner*: the only thing that imports `wfb_ng`. Isolates the Twisted reactor
     lifecycle and sidesteps `wfb_ng`'s import-time global `settings`.
4. **Config feeds `wfb_ng` by rendering a cfg file** (`WIFIBROADCAST_CFG`),
   reusing wfb_ng's exact, tested config path — not by mutating `settings`.
5. **`/etc/wifibroadcast.cfg` is a generated artifact** ("do not edit" header);
   source of truth is `/etc/fpvd/{defaults,config}.json`. Keeping the stock path
   keeps `wfb-cli` and tooling consistent.
6. **Single front door (front-door topology "B").** The client (gsmenu/
   PixelPilot) talks only to `fpvd-GS:8080`. Drone-only config is forwarded
   **opaquely** (no schema coupling); the **overlap is owned by `/link`**.
7. **FEC/LDPC/STBC/MCS are NOT GS video config.** Validated against
   `wfb_ng/services.py`: the video receiver builds `wfb_rx` with no PHY params
   (auto-detected from the stream; drone-owned). Those flags appear only on the
   GS **uplink TX** (`mavlink-tx`/`tunnel-tx`) and use wfb-ng defaults. `link.width`
   stays — it sets the **card** width (HT20/HT40) the GS must match to receive.

## 5. Repository layout

Drone (C++) and GS (Python) are **symmetric top-level components** under one
repo, with shared `docs/` and `deploy/` trees. The existing C++ is relocated
into `drone/` as a unit (see relocation note below).

```
fpvd/
├── drone/                       # C++ (relocated from repo root, as a unit)
│   ├── CMakeLists.txt
│   ├── cmake/  src/  tests/
│   ├── etc/  scripts/  third_party/
│   └── shell.nix
├── gs/                          # Python (new)
│   ├── fpvdgs/
│   │   ├── __init__.py
│   │   ├── supervisor.py        # entrypoint: config store + HTTP API + supervision (invoked as `fpvd`)
│   │   ├── runner.py            # entrypoint: imports wfb_ng, runs the wfb data plane
│   │   ├── config.py            # ConfigStore: defaults + overlay → effective; pending edits
│   │   ├── schema.py            # validation rules
│   │   ├── render.py            # CfgRenderer: effective config → /etc/wifibroadcast.cfg
│   │   ├── link.py              # /link coordinator (overlap params, both-ends apply)
│   │   ├── drone_client.py      # HTTP client to drone fpvd (used by /link and /air proxy)
│   │   ├── api.py               # HTTP routes: /config /apply /reset /defaults /status /healthz /air/* /link
│   │   └── status.py            # StatusProbe: iw dev info + child health + link stats
│   ├── tests/{unit,integration}/  # pytest
│   ├── etc/defaults.json        # GS baseline (seeded from current live values)
│   ├── scripts/S99fpvd          # init script (replaces S98wifibroadcast)
│   └── pyproject.toml
├── deploy/
│   ├── drone/                   # deploy.sh, rollback.sh (existing)
│   └── gs/                      # deploy.sh (rsync pkg + init, restart, no build), rollback.sh (restore S98wifibroadcast)
├── docs/                        # shared specs + plans
└── README.md
```

**C++ relocation (first step of the implementation plan).** Move all existing
top-level C++ artifacts (`CMakeLists.txt`, `cmake/`, `src/`, `tests/`, `etc/`,
`scripts/`, `third_party/`, `shell.nix`, `build/`) into `drone/` via `git mv`
(preserves history). Fixups: `deploy/drone/deploy.sh` `REPO=` now points at the
repo root but the C++ build lives under `drone/` — repoint the build paths
(`cmake -S drone -B drone/build/ssc338q`, `BIN="$REPO/drone/build/ssc338q/fpvd"`);
update the `nix-shell` working dir to `drone/`; update README build commands.
No CI to update (none exists). `${CMAKE_SOURCE_DIR}`-relative includes inside
`CMakeLists.txt` are unaffected by the move.

## 6. Config schema (v1)

Source of truth: `/etc/fpvd/defaults.json` (shipped baseline) deep-merged with
`/etc/fpvd/config.json` (sparse user overlay) → effective config. PATCH edits a
pending copy; `/apply` commits it.

```json
{
  "link": {
    "channel": 132,
    "width": 40,
    "txpower": 19,
    "region": "US",
    "linkId": 7669206,
    "beamforming": { "enabled": false },
    "wlans": "auto"
  },
  "wfb": {
    "profile": "gs",
    "mavlink": { "peer": "connect://127.0.0.1:14550" },
    "raw": {}
  },
  "drone": {
    "endpoint": "http://10.5.0.10:8080"
  }
}
```

- **`link.*`** — purely card/radio-level. `width` → wfb_ng `bandwidth`/`ht_mode`
  (the card must sit at the drone's video TX width to receive). `linkId` matches
  the drone's `link.linkId` (`7669206`). `beamforming` is parsed but **inert in
  v1**. `wlans: "auto"` → `wfb-nics`; or an explicit list.
- **No `fec`/`ldpc`/`stbc`/`mcs`** — drone-owned for video, wfb-default for the
  GS uplink (see §4.7). If uplink tuning is ever wanted, it belongs under
  `wfb.mavlink`/`wfb.tunnel`, not `link`.
- **`wfb.raw`** — passthrough escape hatch: keys merged verbatim into the rendered
  cfg, so we are never blocked on schema coverage.
- **`drone.endpoint`** — where the proxy and `/link` reach the drone fpvd
  (binds `0.0.0.0:8080`, reachable over the wfb tunnel). Replaces gsmenu's
  hardcoded `sshpass`.

### Rendering (`render.py`)

Effective config → `/etc/wifibroadcast.cfg`, atomic (temp+rename), with a
generated-file header and previous kept as `.bak`. Core mapping:

| config | wifibroadcast.cfg |
|---|---|
| `link.channel` | `[common] wifi_channel` |
| `link.region` | `[common] wifi_region` |
| `link.txpower` | `[common] wifi_txpower` |
| `link.width` | service `bandwidth` (→ `ht_mode`) |
| `link.linkId` | `link_id` |
| `wfb.mavlink.peer` | `[gs_mavlink] peer` |
| `wfb.raw.*` | verbatim |

### Validation rules (non-exhaustive)

- `link.channel` valid for region; `link.width ∈ {20, 40}` (10 reserved for
  future); `link.region` a known CRDA code; `link.txpower` in range.
- **Overlap guard:** `/config` and `/air/config` **reject** any `link.*`/overlap
  key. Overlap is mutated **only** through `/link` (prevents radio desync).
- Unknown top-level keys rejected (except under `wfb.raw`).

## 7. Runtime lifecycle

### Boot

1. `S99fpvd start` → `fpvd` (supervisor).
2. ConfigStore loads `defaults.json` + `config.json` → effective.
3. CfgRenderer writes `/etc/wifibroadcast.cfg`.
4. RunnerSupervisor spawns the runner (`WIFIBROADCAST_CFG=/etc/wifibroadcast.cfg`).
5. Runner: `init_wlans` (radio bring-up via `iw`) → spawn `wfb_rx`/`wfb_tx` +
   antenna selection + `:8103`/`:8003` APIs.
6. dynamic-link-gs (own init script) connects to `:8103`.

### `POST /apply` (GS-local, bounce)

1. Validate pending → on failure `400` with reason, nothing touched.
2. Render new cfg atomically; keep `.bak`.
3. Restart the runner (brief RX drop ~1–2 s).
4. Readiness wait: child alive **and** `:8103` accepting, within a timeout.
5. Success → commit pending→effective, `200`. Runner fails → roll cfg to `.bak`,
   restart on old cfg, `500`; effective config unchanged.

### `POST /link/apply` (GS-local-first, best-effort drone)

**Principle: a GS link change always applies locally — drone or no drone.** The
GS-local change is the primary action and the mechanism for *establishing* a
link (e.g. drone on ch100, GS on ch132 → no link; setting the GS to ch100
connects them). Pushing the matching change to the drone is a best-effort
add-on when the drone is reachable, **never a precondition** — gating the GS
change on drone reachability would deadlock (you can't reach the drone until
the GS moves, and the GS wouldn't move until the drone is reachable).

Request carries `applyTo: "gs" | "both"` (default `"both"`):

1. Validate the link change.
2. **If `applyTo: "both"` and the drone is reachable**, push first:
   `PATCH /config {link…}` + `POST /apply` to `drone.endpoint`. The drone fpvd
   ACKs, then defers its retune until after the HTTP response flushes (existing
   drone behaviour, commit `1f250d7`) — so it doesn't drop before we follow.
   Drone unreachable → skip this step (do **not** fail).
3. Apply the **GS side unconditionally** (render cfg + bounce runner), unless
   `applyTo` was drone-only.
4. Report `{ gsApplied, droneApplied, droneReachable, inSync }`. GS-side failure
   rolls back to last-good cfg. The drone-first ordering in step 2 is an
   optimization for the reachable `"both"` path only — never a gate.

`applyTo: "gs"` is the explicit "tune the GS to a drone state I know
out-of-band" / recovery path. Because the link section is a persisted overlay,
a GS link value set now also survives reboot and is applied on next boot
regardless of the drone (pre-staging the channel the drone will come up on).

### `/air/*` (opaque proxy)

Forward the request body to `drone.endpoint` and relay the response verbatim;
`502/504` if the drone is unreachable. No local state, no schema parsing.

### Supervision & shutdown

RunnerSupervisor watches the child pid; on unexpected exit it restarts with
backoff, recorded in `/status`. Crash-loop guard: after N failures in a window it
stops restarting and reports the fault rather than thrashing the radio. On
`S99fpvd stop`: SIGTERM → wait → SIGKILL the runner, which tears down its
`wfb_rx`/`wfb_tx`.

## 8. HTTP API

Front door on `:8080` (bind `0.0.0.0`, reachable over the wfb tunnel/LAN — same
posture as the drone).

| Group | Endpoints | Behaviour |
|---|---|---|
| GS-local | `GET /config[?pending=true]` | effective or pending GS config |
| | `PATCH /config` | deep-merge sparse JSON into pending (overlap rejected) |
| | `POST /apply` | commit pending → effective; render cfg; bounce runner |
| | `POST /reset` | drop overlay |
| | `GET /defaults` | baseline |
| | `GET /status` | daemon + runner/wfb state + radio state + link stats |
| | `GET /healthz` | 200 |
| Drone proxy | `GET/PATCH /air/config`, `POST /air/apply`, `GET /air/status` | opaque forward to `drone.endpoint`; `502/504` if off |
| Link | `GET /link` | overlap params + per-end state (`gsApplied`/`droneApplied`/`droneReachable`/`inSync`) |
| | `PATCH /link`, `POST /link/apply` | GS-local-first link change; `applyTo: "gs"\|"both"` (default `"both"`) pushes to the drone best-effort when reachable |

### Client mapping (gsmenu)

gsmenu's `{get|set} {air|gs} <domain> <key>` dispatch maps mechanically:
- `air` video/encoder/etc. → `/air/*` (proxy)
- `air_channel`/`gs_channel`/width/region → unified `/link` (collapses the two
  channel knobs into one)
- `gs` wfb-local bits → `/config`
- the `sshpass`/`wifibroadcast cli` path is removed.

## 9. `GET /status` response shape

```json
{
  "fpvd":   { "version": "…", "uptimeMs": 0 },
  "runner": { "running": true, "pid": 591, "restarts": 0, "lastExit": null },
  "radio":  [
    { "wlan": "wlx…e6", "channel": 132, "freqMhz": 5660, "widthMhz": 40, "txpowerDbm": 19, "type": "monitor" }
  ],
  "link":   { "linkId": 7669206, "droneReachable": true, "inSync": true,
              "stats": { "video": { "pktS": 0, "rssiAvg": 0, "snrAvg": 0 } } }
}
```

`radio` from `iw dev <wlan> info`; `link.stats` best-effort from the runner's
read-only stats API (omitted if unavailable); `link.droneReachable` from a cheap
cached drone probe; `link.inSync` compares GS link params with the drone's
last-known (via `/air/status` or cached push result, `null` if never reached).
These let gsmenu/PixelPilot show "in sync / drone offline" proactively on a
status poll — without gating any GS change.

## 10. Error handling

- Never tear down a working radio for a bad config: validate → render → bounce
  only on success.
- Atomic cfg writes with `.bak` rollback.
- PATCH is in-memory pending only; nothing touches the radio until apply.
- `/link/apply` partial failure is **reported, never silent**; GS reverts its own
  side on local failure. A drone-unreachable on `/link/apply` is **not** an
  error — the GS side still applies and the response carries
  `droneApplied:false` / `droneReachable:false`.
- `/air/*` drone errors surface as `502/504` (unreachable) or relay the drone's
  own status; the GS side remains operable.

## 11. Security posture

Plain HTTP, no auth, on the private wfb tunnel / GS LAN — identical posture to
the drone fpvd. `drone.endpoint` is HTTP over the tunnel, removing the
plaintext `sshpass` password baked into gsmenu today.

## 12. Testing strategy

### Unit (pytest, no radio/network)

- ConfigStore: defaults+overlay merge, pending vs effective, reset.
- Validator: overlap rejected on `/config` and `/air`; unknown-key rejection;
  `wfb.raw` passthrough allowed.
- CfgRenderer: golden-file rendering; `raw` passthrough; atomic write + `.bak`.
- `/link` coordinator: GS-local-first apply; `applyTo: "gs"` and
  drone-unreachable both still apply the GS side (no gate); `"both"` ordering
  (drone push before GS bounce) and rollback against a **mock drone client**.
- `/air` proxy: forwarding + error mapping against a **mock HTTP server**.

### Integration

fpvd-GS + a **fake runner** (stub that opens `:8103` and stays alive) + a **fake
drone fpvd** (HTTP stub). Exercise PATCH/apply/link/air; assert cfg rendered,
runner restarted, drone calls issued, and error/rollback paths.

### On-device smoke (GS)

Deploy → `wfb_rx`/`wfb_tx` up; `:8103` serving; dynamic-link-gs connected; video
flowing; `/status` correct; `/link` channel change works (with a real or stub
drone); re-pointed gsmenu works.

## 13. Deploy, init & migration

- `deploy/gs/deploy.sh`: rsync the `fpvdgs` package + `S99fpvd` to the GS and
  restart. **No build step** — pure Python on the GS's existing `wfb_ng` +
  Twisted.
- Install backs up + disables `S98wifibroadcast` (and saves the stock
  `/etc/wifibroadcast.cfg`), then starts `fpvd`, mirroring the drone deploy's
  treatment of the old stack.
- `deploy/gs/rollback.sh` restores `S98wifibroadcast` and the saved cfg.
- `defaults.json` seeded from the GS's current live values (channel 132, region
  US, width 40, txpower 19, linkId 7669206, two wlans / auto).

## 14. Future work (designed for, not built)

- **Coordinated switching:** timed channel/bandwidth/beamforming handshake with
  auto-revert — both ends switch at an agreed tick; if the link doesn't
  re-establish within a window, both revert. Lives behind `/link`. Removes the
  bounce.
- **Live `iw` retune** of the runner's monitor interfaces without a child
  restart (the supervisor runs `iw` directly + signals the runner).
- **Beamforming** activation on the GS card (`link.beamforming`).
- **10 MHz width** support (`link.width: 10`), matching the drone's capability.

## 15. Open questions

1. Package/entrypoint naming: package `fpvdgs`, daemon invoked as `fpvd`, init
   `S99fpvd`, config `/etc/fpvd/` — confirm or rename.
2. `link.inSync` source: poll `/air/status` on demand vs. cache the last push
   result. (Leaning: cache + lazy refresh.)
3. **Resolved (see §7).** No drone-reachability *preflight gate*. A GS link
   change is always applied locally first (it's how a link is established);
   the drone push is best-effort when reachable and never a precondition.
   `droneReachable`/`inSync` are surfaced in `/status` + `/link` so the UI can
   warn proactively without blocking any change.

## 16. Success criteria

1. `fpvd` boots on the GS, brings up the radio, and `wfb_rx`/`wfb_tx` run with no
   `wfb-server`/`S98wifibroadcast` present.
2. dynamic-link-gs and wfb-cli work **unchanged** (`:8103`/`:8003` intact).
3. `PATCH /config` + `POST /apply` changes a GS-local param and bounces only the
   runner.
4. `POST /link/apply` with `applyTo:"both"` changes channel/width on **both**
   ends and reports per-end status; the link re-establishes.
5. With the drone unreachable, `POST /link/apply` still applies the GS side
   (`droneApplied:false`) — setting the GS to the drone's channel **establishes**
   the link (bootstrap/recovery), and the value survives reboot.
6. `/air/*` round-trips opaque drone config to the drone fpvd.
7. gsmenu, re-pointed at the three endpoint groups, drives all of the above with
   the two channel knobs collapsed into one.
