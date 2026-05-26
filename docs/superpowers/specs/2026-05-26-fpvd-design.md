# fpvd — Design Spec

**Date:** 2026-05-26
**Status:** Draft for review
**Target:** OpenIPC drones on armhf 32-bit (SoC class: ssc338q and similar)

## 1. Purpose

`fpvd` is a single C++ daemon that owns pre-flight configuration and runtime supervision of the OpenIPC FPV stack: the **waybeam** video encoder, the **wfb-ng** radio link (`wfb_tx`, `wfb_rx`, `wfb_tun`), and the telemetry router (**msposd** or **mavfwd**). It replaces `/etc/init.d/S95waybeam` and `/etc/init.d/S98wifibroadcast` with one init script and one supervisor process.

It exposes a unified, domain-modeled configuration over HTTP+JSON, reachable from a ground station over both the wfb tunnel (`10.5.0.10/24`) and any LAN interface.

## 2. Problems being solved

1. **Fragmented configuration.** Encoder and radio are operationally one system but configured through two files (`/etc/waybeam.json`, `/etc/wfb.yaml`) with different formats and cadences.
2. **No remote configuration path.** Updating either file in the field requires SSH + `sed`/`yaml-cli`. There is no programmatic, networked API.
3. **Split startup.** `S95waybeam` and `S98wifibroadcast` are separate init scripts with no shared lifecycle, despite the processes being interdependent.

## 3. Scope

### In scope (v1)

- Unified domain-modeled config, owned by fpvd, persisted as a sparse user overlay over a firmware-baked default.
- HTTP+JSON REST API: read effective config, stage partial edits, explicit apply, factory reset, status.
- Supervision of first-class subsystems: `waybeam`, `wfb_tx`, `wfb_rx`, `wfb_tun`, `msposd`/`mavfwd`.
- Extension point for additional user-defined services (e.g. adaptive-link) via a `services` config section.
- One init script (`S99fpvd`) replacing both existing scripts.

### Out of scope (v1)

- Flight-time hot reload of config (config changes happen pre-flight on the ground).
- Live telemetry streaming (RSSI/FEC/CPU) — already covered by msposd OSD and mavlink.
- Log streaming/aggregation.
- Embedded UI — API only; clients are external (`curl`, future ground-station app).
- Authentication — see Threat model (§13).

## 4. Architectural decisions (recap of brainstorming)

| Decision | Choice | Rationale |
|---|---|---|
| Type | Standalone supervisor daemon | Cleanest boundary; keeps both upstreams (waybeam, wfb-ng) unmodified. |
| Transport | HTTP+JSON over wfb tunnel and LAN | Curl-able, browsable, transport-agnostic. |
| Runtime scope | Config + supervision + status, request/reply only | No streaming layer; live telemetry stays with existing channels. |
| UI | None on device | Clients are external. Smaller binary, cleaner separation. |
| Language | C++ | Ecosystem fit (waybeam, wfb-ng, msposd are C++). ~200–400 KB binary with header-only deps. |
| Config schema | Domain-modeled (fpvd defines the schema) | Stable public API even as upstreams rename keys. Cost: must enumerate tunables. |
| Radio bring-up | Thin shell helper (`radio-up.sh`), invoked by daemon | USB detect + `modprobe` + `iw`/`ifconfig` is shell-shaped; keeping it in shell avoids dragging libusb/libnl into the C++ binary. |
| Auth | None | Pre-flight on private network; wfb tunnel members already share a key. |
| Apply model | Stage + explicit `POST /apply` | Pre-flight workflow is fiddle-many-then-commit; auto-apply on PATCH would cause repeated video blips. |
| Persistence | `/rom/etc/fpvd/defaults.json` baseline + `/etc/fpvd/config.json` sparse user overlay | OpenIPC convention; makes factory reset obviously correct (`rm` overlay). |

**Hard assumption:** Config changes happen with the drone on the ground. A multi-second video/link interruption during apply is acceptable.

## 5. Repository layout

```
fpvd/
├── CMakeLists.txt
├── docs/
│   └── superpowers/specs/2026-05-26-fpvd-design.md
├── src/
│   ├── main.cpp                # init, signal handling, lifecycle
│   ├── config/
│   │   ├── schema.hpp          # domain types (Link, Video, Telemetry, Services, ...)
│   │   ├── store.cpp           # load defaults, load overlay, merge, atomic write
│   │   └── validate.cpp        # range checks, cross-field constraints
│   ├── translate/
│   │   ├── waybeam.cpp         # domain → /etc/waybeam.json
│   │   ├── wfb.cpp             # domain → wfb_tx/wfb_rx/wfb_tun argv
│   │   └── telemetry.cpp       # domain → msposd/mavfwd argv
│   ├── supervise/
│   │   ├── process.cpp         # fork/exec/waitpid, restart backoff
│   │   ├── radio.cpp           # invokes scripts/radio-up.sh
│   │   └── orchestrator.cpp    # ordered start/stop, dependency resolution
│   ├── http/
│   │   ├── server.cpp          # cpp-httplib bindings
│   │   └── handlers.cpp        # GET/PATCH/POST endpoints
│   └── status.cpp              # aggregates child process state
├── scripts/
│   ├── radio-up.sh             # USB detect + modprobe + iw/ifconfig (lifted from current wifibroadcast)
│   └── S99fpvd                 # single init script
├── etc/
│   └── defaults.json           # installed to /rom/etc/fpvd/defaults.json
├── third_party/
│   ├── cpp-httplib/            # vendored, header-only
│   └── nlohmann_json/          # vendored, header-only
├── tests/
│   ├── unit/
│   └── integration/
├── LICENSE
└── README.md
```

## 6. Config schema (v1 domain model)

The schema is fpvd's public contract. All keys are mapped at apply time onto `/etc/waybeam.json` or `wfb_*`/`msposd`/`mavfwd` argv. Fields not modeled here are not user-tunable in v1; they live as fixed values inside `defaults.json` and the translation layer.

```jsonc
{
  "link": {
    "channel": 161,           // wifi channel
    "width": 20,              // 20 | 40 (MHz)
    "txpower": 1,             // 1..63, driver-translated (88XXau uses negative scaling, others positive)
    "mcs": 2,                 // 0..7
    "fec": { "k": 8, "n": 12 },
    "stbc": false,
    "ldpc": false,
    "linkId": 7669206,        // 24-bit
    "mtu": 1500,              // applied as ifconfig mtu
    "wlanAdapter": null       // optional adapter id passed to wfb (auto-detected if null)
  },
  "video": {
    "codec": "h265",          // h264 | h265
    "resolution": "1920x1080",
    "fps": 60,
    "bitrate": 8192,          // kbps
    "rcMode": "cbr",          // cbr | vbr
    "gopSize": 1.0,
    "qpDelta": -4,
    "roi": {
      "enabled": true,
      "qp": 0,
      "center": 0.4,
      "steps": 2
    }
  },
  "image": {
    "mirror": false,
    "flip": false,
    "rotate": 0               // 0 | 90 | 180 | 270
  },
  "telemetry": {
    "router": "msposd",       // msposd | mavfwd | none
    "serial": "ttyS2",
    "osdFps": 20,
    "baud": 115200
  },
  "recording": {
    "enabled": false,
    "dir": "/mnt/mmcblk0p1",
    "format": "ts",
    "mode": "mirror",
    "maxSeconds": 300,
    "maxMB": 500
  },
  "snapshot": {
    "enabled": true,
    "quality": 80
  },
  "services": {
    // user-defined supervised processes, see §10
  }
}
```

### Validation rules (non-exhaustive)

- `link.channel` ∈ valid set per driver (validation deferred to apply time when driver is known; basic range check on PATCH).
- `link.width` ∈ {20, 40}.
- `link.fec.k < link.fec.n`, both ≥ 1, ≤ 32.
- `video.resolution` parses as `WxH`, both > 0.
- `video.fps` > 0, ≤ 120.
- `telemetry.router == "none"` disables the telemetry subsystem entirely.
- Unknown top-level keys: rejected with 400. Unknown keys inside `services.<name>`: also rejected (typo protection).

## 7. Runtime lifecycle

```
boot
 └─ /etc/init.d/S99fpvd start
     └─ fpvd
         1. load /rom/etc/fpvd/defaults.json          (required, fail-fast if missing/invalid)
         2. overlay /etc/fpvd/config.json if present  (sparse JSON, deep-merged)
         3. validate merged config
         4. exec scripts/radio-up.sh                  (USB detect, modprobe, iw, ifconfig, devmem chipset poke)
         5. translate config → write /etc/waybeam.json
         6. spawn first-class subsystems in dependency order:
              radio subsystem → encoder → telemetry router
            The radio subsystem itself is a set of wfb-ng processes filling three roles
            (see §10): a video-egress wfb_tx (unix-socket input from waybeam), a tunnel
            pair (wfb_rx + wfb_tx + wfb_tun), and a telemetry pair (wfb_rx + wfb_tx)
            feeding the router. fpvd starts them in the order they appear here.
         7. spawn user services in topo-sorted order (by startAfter)
         8. bind HTTP on tunnel IP + configured LAN ifaces, port 8080
            (8080 chosen so it does not collide with waybeam's existing :80; configurable via system.httpPort in defaults.json)
         9. event loop: HTTP, SIGCHLD reaping, supervision
```

### Apply flow (`POST /apply`)

1. Validate pending config; reject with 400 + structured error list if invalid.
2. Compute diff between current effective and pending; categorize each changed path:
   - `link.*` → radio restart required.
   - `video.*`, `image.*`, `recording.*`, `snapshot.*` → waybeam restart.
   - `telemetry.*` → telemetry router restart.
   - `services.<name>.*` → that service restart.
   - `services.<name>` added/removed → start/stop that service.
3. Atomic-write `/etc/fpvd/config.json` (write `.tmp` → `fsync` → `rename`).
4. Stop affected children in reverse-dependency order. SIGTERM, wait 5s, SIGKILL stragglers.
5. If `link.*` changed: re-run `radio-up.sh`.
6. If `video.*`/`image.*`/`recording.*`/`snapshot.*` changed: rewrite `/etc/waybeam.json`.
7. Respawn stopped children in dependency order.
8. Return `200 { "applied": true, "version": <monotonic int>, "restarted": [...] }`.

**Note:** v1 implements selective restart at the *subsystem granularity* described above (radio / encoder / telemetry / per-service). This is more selective than a blunt "restart everything" but less than per-field selectivity, which is a future optimization.

## 8. HTTP API

All responses are JSON. All non-2xx responses carry `{ "error": "<machine code>", "message": "<human>", "details": {...} }`.

| Method | Path | Behavior |
|---|---|---|
| `GET` | `/config` | Effective config (defaults ⊕ overlay). |
| `GET` | `/config?pending=true` | Staged-not-applied config; falls back to effective if nothing staged. |
| `PATCH` | `/config` | Body is sparse JSON, deep-merged into pending. Returns full pending view. Validates types/ranges but does not require full validity. |
| `POST` | `/apply` | Commit pending → effective. Restart affected subsystems. 200 on success with applied version. |
| `POST` | `/reset` | Delete `/etc/fpvd/config.json`. Pending becomes equal to defaults. Client must `POST /apply` to take effect on running processes. |
| `GET` | `/defaults` | Read-only baseline. Useful for clients that want to compute diffs. |
| `GET` | `/status` | See §11. |
| `GET` | `/healthz` | `200 OK` if daemon event loop is alive. For external watchdog use. |

### Example: change channel and bitrate, then apply

```sh
curl -sX PATCH http://10.5.0.10:8080/config \
  -H 'content-type: application/json' \
  -d '{"link":{"channel":165},"video":{"bitrate":10000}}'
curl -sX POST  http://10.5.0.10:8080/apply
```

## 9. Process supervision

### Per-child model

Each managed process is represented by a `Process` value with:

- `name` (string, unique)
- `argv` (vector<string>)
- `env` (map<string,string>)
- `restartPolicy` (`"always"` | `"on-failure"` | `"never"`)
- `startAfter` (list of dependencies, for ordering only)
- runtime fields: `pid`, `restartCount`, `lastExitCode`, `lastExitAt`, `state` (`stopped` | `starting` | `running` | `failed`)

### Restart policy

- Exponential backoff: `1s → 2s → 4s → 8s → 16s → 30s` cap.
- Reset to `1s` after 60s of continuous uptime.
- If a child crashes ≥ 5 times in 60s without crossing the uptime reset, mark it `failed` and stop restarting. Failure is surfaced in `/status`. The daemon stays up so the operator can PATCH a fix.

### Shutdown

- Daemon SIGTERM → graceful stop of all children in reverse-dependency order → exit.
- Per-child stop: SIGTERM, wait 5s, SIGKILL.
- All children inherit a process group; daemon sends signals to the group to catch stubborn forks. Waybeam's `waybeam-resp` (respawn helper) and `waybeam-wd` (watchdog) — see the current `S95waybeam` script for context — are children of the waybeam parent, which is itself a child of fpvd. Because fpvd is the real parent (not a shell launching `&`), it gets `SIGCHLD` for the parent process and can signal the group; the per-name `pidof` dance the current init script does is unnecessary here.

## 10. Extensibility — first-class subsystems vs. user services

### First-class subsystems

Hard-coded in fpvd because they have coupling fpvd has to understand:

| Subsystem | Processes | Coupling reason |
|---|---|---|
| **Radio** | `wfb_tx` (video), `wfb_rx`+`wfb_tx`+`wfb_tun` (tunnel), `wfb_rx`+`wfb_tx` (telemetry) | Requires `radio-up.sh` (driver, iw, ifconfig) before start. Owns `link.*`. Three distinct roles, each with its own argv; named in `/status` as `wfb_video_tx`, `wfb_tun_rx`, `wfb_tun_tx`, `wfb_tun`, `wfb_tlm_rx`, `wfb_tlm_tx`. |
| **Encoder** | `waybeam` | Reads `/etc/waybeam.json` at fixed path; daemon owns that file. Owns `video.*`, `image.*`, `recording.*`, `snapshot.*`. |
| **Telemetry router** | `msposd` or `mavfwd` | Argv depends on `video.resolution` + `telemetry.*`. The chipset poke `devmem 0x1F207890 16 0x8` happens in `radio-up.sh`. |

Adding a new first-class subsystem is a schema bump + code change. Reserved for things with cross-cutting coupling.

### User services

Anything else is a `services.<name>` entry:

```jsonc
{
  "services": {
    "adaptive-link": {
      "enabled": true,
      "exec": "/usr/bin/adaptive-link",
      "args": ["--stats", "udp://127.0.0.1:9601", "--target", "http://127.0.0.1:8080"],
      "env": { "LOG_LEVEL": "info" },
      "startAfter": ["wfb_rx"],
      "restart": "always"
    }
  }
}
```

Rules:

- fpvd supervises services identically to first-class processes (restart backoff, status, shutdown).
- `startAfter` may reference first-class process names (`wfb_rx`, `waybeam`, `msposd`) or other service names. Cycles rejected at validation.
- Modifying `services.<name>` triggers restart of *that* service only on `POST /apply`. Adding/removing entries starts/stops accordingly.
- Services are surfaced in `GET /status` the same way as first-class children.

**Deliberate non-features:**

- No plugin SDK, no shared library, no in-process plugins.
- No IPC framework. If a service needs to change config (e.g. adaptive-link adjusting bitrate), it calls `PATCH /config` + `POST /apply` against fpvd's own HTTP API — same surface a human uses.
- No `${var}` argv templating in v1. If a service needs values from config, the build/integration step that drops the service entry into `defaults.json` substitutes them. Templating can be added later without breaking existing entries.

## 11. `GET /status` response shape

```jsonc
{
  "uptime": 3421,
  "version": 7,                   // monotonic apply counter
  "lastApply": {
    "at": "2026-05-26T12:34:56Z",
    "ok": true,
    "restarted": ["wfb_tx", "wfb_rx", "wfb_tun"],
    "error": null
  },
  "radio": {
    "driver": "8812eu",
    "iface": "wlan0",
    "adapterId": "bl-m8812eu2"
  },
  "processes": [
    { "name": "wfb_video_tx", "pid": 412, "state": "running", "restarts": 0, "lastExitCode": null, "uptime": 3410 },
    { "name": "wfb_tun_rx",   "pid": 413, "state": "running", "restarts": 0, "lastExitCode": null, "uptime": 3410 },
    { "name": "wfb_tun_tx",   "pid": 414, "state": "running", "restarts": 0, "lastExitCode": null, "uptime": 3410 },
    { "name": "wfb_tun",      "pid": 415, "state": "running", "restarts": 0, "lastExitCode": null, "uptime": 3410 },
    { "name": "wfb_tlm_rx",   "pid": 416, "state": "running", "restarts": 0, "lastExitCode": null, "uptime": 3410 },
    { "name": "wfb_tlm_tx",   "pid": 417, "state": "running", "restarts": 0, "lastExitCode": null, "uptime": 3410 },
    { "name": "waybeam",      "pid": 421, "state": "running", "restarts": 1, "lastExitCode": 0,    "uptime": 3200 },
    { "name": "msposd",       "pid": 432, "state": "running", "restarts": 0, "lastExitCode": null, "uptime": 3410 }
  ]
}
```

## 12. Error handling

| Failure | Response |
|---|---|
| PATCH body malformed JSON | `400 { error: "bad_json" }`. Pending unchanged. |
| PATCH violates schema (unknown key, wrong type, out-of-range) | `400 { error: "validation", details: [...] }`. Pending unchanged. |
| POST /apply with invalid pending | `400 { error: "validation", details: [...] }`. Pending unchanged, no restart. |
| `radio-up.sh` exits non-zero | `500 { error: "radio_bringup_failed", details: { exitCode, stderr } }`. Restore previous waybeam.json and wfb argv; respawn previous children. |
| Child fails to start within 3s of fork | Mark `failed`, log stderr, continue applying others. Surface in `/status`. Apply still returns 200 if at least one child started; response includes `failed` list. |
| Daemon's own HTTP bind fails on all interfaces | Fatal: exit with code 2 so initscript can restart. |
| Daemon's own HTTP bind fails on *some* but not all interfaces | Log and continue; surface in `/status`. |

## 13. Threat model and security posture

`fpvd` runs no authentication in v1. Justifications:

- Reachable only via the wfb tunnel (members already share `drone.key`) or LAN interfaces the operator has explicitly configured.
- Pre-flight workflow only; not internet-facing.
- Drone is operator-controlled hardware; trust boundary is at the radio link, not the API.

Non-goals:

- No mTLS, no token, no CSRF protection (no browser session model).
- No rate limiting beyond what cpp-httplib provides out of the box.

If a future deployment needs auth (e.g. sharing a flying field), a shared-bearer-token mode can be added as a `system.auth` config section without breaking existing clients.

## 14. Testing strategy

### Unit tests

- Schema validation: every documented constraint has a positive and negative test.
- Config merge: defaults ⊕ overlay produces expected effective config, including partial overrides.
- Translation: golden-file tests for `domain → waybeam.json` and `domain → wfb_tx argv` and `domain → msposd argv`.
- Diff categorization: which subsystems are flagged for restart given a specific pending vs. effective.

### Integration tests

- Process supervision against a fake child (`/bin/sh -c 'trap "echo TERM; exit 0" TERM; sleep 9999'` and a crashy variant). Verify start, graceful stop, SIGKILL escalation, restart backoff, failure cap.
- HTTP layer: in-process server against a fixture config dir, full round-trip per endpoint, error cases.

### On-device smoke

- A script in `tests/smoke/` that, against a running fpvd on a drone:
  1. `GET /defaults` and `GET /config`; assert they parse.
  2. `PATCH /config` with a known-safe change (e.g. `video.bitrate`).
  3. `POST /apply`; poll `/status` until `version` increments.
  4. Assert `processes[waybeam].pid` changed (proves real restart happened).
  5. Revert via `POST /reset` + `POST /apply`.
- Runs in CI under qemu-arm when feasible; otherwise documented as manual.

## 15. Buildroot integration

A new `package/fpvd/` is added to the `openipc-builder` fork.

- `WAYBEAM_VENC_INSTALL_TARGET_CMDS` is amended: stop installing `init.d/S95waybeam`. The waybeam binary install is unchanged.
- `WIFIBROADCAST_NG_INSTALL_TARGET_CMDS` is amended: stop installing `init.d/S98wifibroadcast` and the `wifibroadcast` shell script and `wfb.yaml`. The four wfb binaries continue to be installed unchanged.
- `package/fpvd/fpvd.mk` installs:
  - `/usr/bin/fpvd`
  - `/etc/init.d/S99fpvd`
  - `/usr/libexec/fpvd/radio-up.sh`
  - `/rom/etc/fpvd/defaults.json` (via the overlay mechanism)

No changes to the upstream `OpenIPC/waybeam_venc` or to `gilankpam/wfb-ng` sources are required.

## 16. Migration

Existing devices in the field that already have `/etc/waybeam.json` and `/etc/wfb.yaml`:

- `/etc/waybeam.json` is now a *daemon-managed output*. fpvd overwrites it on the first apply (or the first boot, since startup rewrites it from current config). Any pre-existing user edits to `/etc/waybeam.json` will be lost — they need to be re-expressed through fpvd's API.
- `/etc/wfb.yaml` is unused. Nothing in the new stack reads it. It can be deleted by the firmware update script or left as harmless detritus.
- fpvd does not read either legacy file. The fresh `/rom/etc/fpvd/defaults.json` is the baseline; `/etc/fpvd/config.json` does not yet exist, so the effective config = defaults.
- A one-time migration helper (`/usr/libexec/fpvd/migrate.sh`) can read the legacy files and produce a starting `/etc/fpvd/config.json`. **Optional v1**; if cut, the user reconfigures via the API on first boot.

## 17. Open questions

None blocking. Items intentionally deferred to v2+:

- Per-field selective restart (v1 is per-subsystem).
- Authentication (`system.auth` token mode).
- Live telemetry stream endpoint (WebSocket or SSE).
- Log tail endpoint.
- Argv templating in `services` entries.
- First-class promotion for adaptive-link if it becomes ubiquitous.

## 18. Success criteria

The v1 release is complete when:

1. A drone firmware build that includes fpvd boots cleanly with no `S95waybeam` or `S98wifibroadcast` present.
2. `curl http://10.5.0.10:8080/status` from a ground station returns all five first-class processes as `running`.
3. `PATCH /config` + `POST /apply` with a `video.bitrate` change observably changes the encoded stream's bitrate without manual intervention on the drone.
4. `POST /reset` + `POST /apply` returns the running configuration to the firmware baseline.
5. Killing any first-class child manually (`kill -9 $(pidof waybeam)`) results in automatic restart visible in `/status` within 2 seconds.
6. An entry added under `services` (e.g. a placeholder `echo` loop) starts under fpvd supervision after `POST /apply` and is restarted on crash.
