# fpvd HTTP API

`fpvd` is a supervisor daemon for OpenIPC FPV drone stacks. It owns a unified configuration object covering the radio link, video encoder, telemetry router, on-board recording, and optional adaptive-link control, and it supervises those subsystems as child processes. All configuration is managed through this HTTP+JSON API: stage changes incrementally, validate them, then commit in one `POST /apply`.

The server binds on `0.0.0.0:8080` by default (overridable with `--port`). There is no authentication — the API is reachable over the wfb-ng tunnel (`10.5.0.10/24`) and any LAN interface, all of which are assumed to be private networks. Clients include ground-station apps, `curl` scripts, and the `wfbng-dynamic-link` ground-station component.

> **Two daemons.** This repo ships two `fpvd` builds: the **drone** daemon (C++, `drone/`) documented in the sections below, and the **ground-station** daemon (Python, `gs/`) documented in [Ground-station API (fpvd-GS)](#ground-station-api-fpvd-gs). They share the HTTP surface (`/config`, `/apply`, `/reset`, `/defaults`, `/status`, `/healthz`) and the stage→apply lifecycle. The drone is an independent daemon with the flat config schema documented below. The GS is a **unified config facade**: its `GET/PATCH /config` returns ONE merged tree (GS-local config + the drone's config, fetched over the link) and `POST /apply` is a multi-lane router that fans changes out to the GS, a shared-link coordinator, and the drone. There are no separate GS `/link` or `/air/*` routes anymore — the unified `/config` + `/apply` is the single front door. Unless a section is under that heading, it describes the **drone** daemon (which a GS client reaches through the unified tree, not directly).

---

## Conventions

### Content-type

All request bodies must be `application/json`. All responses are `application/json`.

### Error response shape

Every error response uses HTTP 400 and a JSON body with this structure:

```jsonc
{
  "error": "error_code",       // string — machine-readable code
  "message": "human text",     // string — description
  "details": ...               // present on some errors; shape varies by code
}
```

See [Error codes](#error-codes) for the full enumeration.

### Config lifecycle

The daemon maintains two views of the configuration:

- **effective** — the config that running processes were started with. Updated only by `POST /apply`.
- **pending** — a staging area. Starts equal to effective; mutated by `PATCH /config`; reset to defaults by `POST /reset`.

`GET /config` returns effective by default and pending with `?pending=true`.

`POST /apply` validates pending, persists it as a sparse overlay over the firmware baseline, and restarts only the affected subsystems.

---

## Endpoints

### GET /healthz

Returns 200 when the daemon is alive. Carries no body other than an empty JSON object. Use this for liveness probes.

**Response body**

```jsonc
{}
```

**Status codes**

| Code | Meaning |
|------|---------|
| 200  | Daemon is alive. |

**Example**

```bash
curl http://127.0.0.1:8080/healthz
```

```json
{}
```

---

### GET /defaults

Returns the firmware baseline configuration — the values burned into `/rom/etc/fpvd/defaults.json` at build time. This is the starting point before any user overlay is applied. Use it to show "reset to defaults" previews or to build a diff display.

**Request body:** none

**Response body:** a complete [configuration object](#config-schema).

**Status codes**

| Code | Meaning |
|------|---------|
| 200  | OK. |

**Example**

```bash
curl http://127.0.0.1:8080/defaults
```

```json
{
  "link": {"channel": 161, "width": 20, "txpower": 20, "mcs": 2,
           "fec": {"k": 8, "n": 12}, "stbc": false, "ldpc": false,
           "linkId": 7669206, "mtu": 1500, "wlanAdapter": null,
           "txpowerCurve": null},
  "video": {"codec": "h265", "resolution": "1920x1080", "fps": 60,
            "bitrate": 8192, "rcMode": "cbr", "gopSize": 1.0, "qpDelta": -4,
            "roi": {"enabled": true, "qp": 0, "center": 0.4, "steps": 2}},
  "image": {"mirror": false, "flip": false, "rotate": 0},
  "telemetry": {"router": "msposd", "serial": "ttyS2", "osdFps": 20, "baud": 115200},
  "recording": {"enabled": false, "format": "ts",
                "mode": "mirror", "maxSeconds": 300, "maxMB": 500},
  "dynamicLink": {
    "enabled": false, "healthTimeoutMs": 10000,
    "interleavingSupported": true, "minIdrIntervalMs": 500,
    "applyStaggerMs": 50, "applySubPaceMs": 5, "mavlinkEnable": true,
    "osd": {"enabled": true, "debugLatency": false},
    "roiQp": {"thresholdKbps": 6000, "lowAnchorKbps": 2000, "floor": -24, "step": 3},
    "failsafe": {"mcs": 1, "k": 8, "n": 12, "depth": 1,
                 "bandwidth": 20, "bitrateKbps": 2000}
  },
  "services": {}
}
```

---

### GET /config

Returns either the **effective** configuration (what running processes are using) or the **pending** configuration (staged changes not yet applied).

**Query parameters**

| Parameter | Type | Description |
|-----------|------|-------------|
| `pending` | `"true"` | If present and equal to `"true"`, returns pending instead of effective. |

**Request body:** none

**Response body:** a complete [configuration object](#config-schema).

**Status codes**

| Code | Meaning |
|------|---------|
| 200  | OK. |

**Examples**

```bash
# Read effective config
curl http://127.0.0.1:8080/config

# Read pending (staged but not yet applied) config
curl 'http://127.0.0.1:8080/config?pending=true'
```

---

### PATCH /config

Deep-merges a partial configuration object into the pending config. Only the keys present in the request body are changed; all other fields are left as-is. Validates the merged result before accepting it.

The request body can be as sparse as a single field or as complete as the full configuration object.

**Request body:** a partial [configuration object](#config-schema). All fields are optional.

```jsonc
// Example: change channel and bitrate only
{
  "link": {"channel": 100},
  "video": {"bitrate": 12000}
}
```

**Response body (200):** the full pending configuration object after the merge.

**Response body (400):** an [error object](#error-codes). Possible error codes: `bad_json`, `validation`, `dynamic_link_locked`.

**Status codes**

| Code | Meaning |
|------|---------|
| 200  | Patch accepted. Response body is the updated pending config. |
| 400  | Invalid request. See `error` field for code. |

**Example**

```bash
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"video":{"bitrate":12000,"fps":90}}'
```

```jsonc
{
  "link": {"channel": 161, "width": 20, "txpower": 20, "mcs": 2,
           "fec": {"k": 8, "n": 12}, "stbc": false, "ldpc": false,
           "linkId": 7669206, "mtu": 1500, "wlanAdapter": null,
           "txpowerCurve": null},
  "video": {"codec": "h265", "resolution": "1920x1080", "fps": 90,
            "bitrate": 12000, "rcMode": "cbr", "gopSize": 1.0, "qpDelta": -4,
            "roi": {"enabled": true, "qp": 0, "center": 0.4, "steps": 2}},
  // ... remaining fields unchanged
}
```

**Validation error example**

```bash
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"link":{"mcs":99}}'
```

```json
{
  "error": "validation",
  "message": "schema validation failed",
  "details": [
    {"path": "link.mcs", "message": "must be 0..7"}
  ]
}
```

---

### POST /apply

Commits the pending configuration to effective, persists the user overlay, and restarts only the subsystems affected by the change. This is the single action that makes staged changes take effect on the drone.

Affected subsystems are determined by which fields changed:

- `link.*` changes restart the radio (re-runs `radio-up.sh`) and all wfb processes.
- `video.*`, `image.*`, `recording.*` changes restart the encoder (waybeam).
- `telemetry.*` changes restart the telemetry router.
- `dynamicLink.*` changes restart the adaptive-link controller (`dl_applier`).
- `services.<name>.*` changes restart that named service.

**Request body:** none

**Response body (200)**

```jsonc
{
  "applied": true,
  "version": 2,         // integer — increments on every successful apply
  "restarted": [        // array of subsystem names that were restarted
    "radio",
    "encoder"
  ]
}
```

**Response body (400):** a `validation` error object (only if pending somehow contains invalid data; normally prevented by PATCH validation).

**Status codes**

| Code | Meaning |
|------|---------|
| 200  | Changes committed; `restarted` lists affected subsystems. |
| 400  | Pending config is invalid (rare; see `validation` error). |
| 400  | Radio bring-up failed; response is `validation` with an empty `details` array. Check `GET /status` for the error text. |

**Example**

```bash
curl -X POST http://127.0.0.1:8080/apply
```

```json
{
  "applied": true,
  "version": 3,
  "restarted": ["encoder"]
}
```

---

### POST /reset

Discards the user overlay file and resets pending to the firmware baseline (same as `GET /defaults`). Does **not** call `POST /apply` — the effective configuration and running processes are unchanged until the client calls apply.

**Request body:** none

**Response body**

```jsonc
{"reset": true}
```

**Status codes**

| Code | Meaning |
|------|---------|
| 200  | Overlay discarded; pending is now the firmware baseline. |

**Example**

```bash
curl -X POST http://127.0.0.1:8080/reset
# Then inspect what will be applied after reset:
curl 'http://127.0.0.1:8080/config?pending=true'
# Commit the reset:
curl -X POST http://127.0.0.1:8080/apply
```

---

### GET /status

Returns daemon runtime state: uptime, config version, last apply outcome, radio hardware info, and the state of every supervised process.

**Request body:** none

**Response body**

```jsonc
{
  "uptime": 3600,              // integer — seconds since daemon start
  "version": 3,               // integer — config version (increments on apply)
  "lastApply": {              // null if no apply has been attempted yet
    "at": "2026-05-27T14:23:01Z",  // ISO-8601 UTC timestamp
    "ok": true,                     // boolean — whether apply succeeded
    "restarted": ["encoder"],       // array of subsystem names restarted
    "error": null                   // string or null — error message on failure
  },
  "radio": {
    "driver": "88XXau",       // string — kernel module name
    "iface": "wlan0",         // string — network interface name
    "adapterId": "0bda:8812", // string or null — USB vendor:product id
    "txpowerCurve": [29, 28, 25, 23, 19, 19, 19, 19],
                              // array[8] — resolved effective TX power curve in dBm (one per MCS 0..7)
    "txpowerCurveSource": "bl-m8812eu2"
                              // string — source of the curve: "override" (from link.txpowerCurve),
                              //   a radio name (e.g. "bl-m8812eu2") when using the per-radio default,
                              //   or "fallback" when no radio-specific curve was found
  },
  "processes": [              // array — one entry per supervised process
    {
      "name": "wfb_video_tx",
      "pid": 234,             // integer — current PID
      "state": "running",     // "stopped" | "starting" | "running" | "exited" | "failed"
      "restarts": 0,          // integer — lifetime restart count
      "lastExitCode": null    // integer or null — exit code of last termination
    }
  ]
}
```

**Status codes**

| Code | Meaning |
|------|---------|
| 200  | OK. |

**Example**

```bash
curl http://127.0.0.1:8080/status
```

```json
{
  "uptime": 182,
  "version": 1,
  "lastApply": {
    "at": "2026-05-27T10:00:05Z",
    "ok": true,
    "restarted": ["radio", "encoder", "telemetry"],
    "error": null
  },
  "radio": {
    "driver": "88XXau",
    "iface": "wlan0",
    "adapterId": "0bda:8812",
    "txpowerCurve": [29, 28, 25, 23, 19, 19, 19, 19],
    "txpowerCurveSource": "bl-m8812eu2"
  },
  "processes": [
    {"name": "wfb_video_tx", "pid": 234, "state": "running", "restarts": 0, "lastExitCode": null},
    {"name": "wfb_tun_rx",   "pid": 235, "state": "running", "restarts": 0, "lastExitCode": null},
    {"name": "wfb_tun_tx",   "pid": 236, "state": "running", "restarts": 0, "lastExitCode": null},
    {"name": "wfb_tun",      "pid": 237, "state": "running", "restarts": 0, "lastExitCode": null},
    {"name": "wfb_tlm_rx",   "pid": 238, "state": "running", "restarts": 0, "lastExitCode": null},
    {"name": "wfb_tlm_tx",   "pid": 239, "state": "running", "restarts": 0, "lastExitCode": null},
    {"name": "waybeam",      "pid": 240, "state": "running", "restarts": 0, "lastExitCode": null},
    {"name": "msposd",       "pid": 241, "state": "running", "restarts": 0, "lastExitCode": null}
  ]
}
```

---

## Config schema

The complete shape of the configuration object returned by `GET /config`, `GET /defaults`, and `PATCH /config`. Every `PATCH /config` body is validated against these rules before being accepted.

### `link` — radio link parameters

```jsonc
"link": {
  "channel": 161,       // integer, 1..200 — Wi-Fi channel number
  "width": 20,          // integer, 20 or 40 — channel width in MHz
  "txpower": 20,        // integer, 0..30 — TX power in dBm
  "mcs": 2,             // integer, 0..7 — MCS index
  "fec": {
    "k": 8,             // integer, 1..31 — FEC data shards (k < n, n ≤ 32)
    "n": 12             // integer, 2..32 — FEC total shards (n > k, n ≤ 32)
  },
  "stbc": false,        // boolean — enable STBC
  "ldpc": false,        // boolean — enable LDPC
  "linkId": 7669206,    // integer — wfb-ng link ID (must match GS)
  "mtu": 1500,          // integer — packet MTU in bytes
  "wlanAdapter": null,  // string or null — force a specific wlan interface name;
                        //                  null = auto-detect via radio-up.sh
  "txpowerCurve": null  // array[8] | null — per-MCS TX power in dBm (MCS 0..7);
                        //   null = use the per-radio default curve;
                        //   an 8-entry array overrides it. Each entry 0..30.
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `channel` | integer | `161` | 1 – 200 |
| `width` | integer | `20` | `20` or `40` |
| `txpower` | integer | `20` | 0 – 30 (dBm) |
| `mcs` | integer | `2` | 0 – 7 |
| `fec.k` | integer | `8` | 1 – 31, must be < `fec.n` |
| `fec.n` | integer | `12` | 2 – 32, must be > `fec.k` |
| `stbc` | boolean | `false` | — |
| `ldpc` | boolean | `false` | — |
| `linkId` | integer | `7669206` | — |
| `mtu` | integer | `1500` | — |
| `wlanAdapter` | string \| null | `null` | interface name or `null` |
| `txpowerCurve` | array\[8\] \| null | `null` | `null` = per-radio default curve; 8-entry array overrides it, each entry 0 – 30 (dBm, one per MCS 0..7) |

### `video` — encoder parameters

```jsonc
"video": {
  "codec": "h265",          // string — "h264" or "h265"
  "resolution": "1920x1080",// string — "WxH" format, both dimensions > 0
  "fps": 60,                // integer, 1..120 — frames per second
  "bitrate": 8192,          // integer, > 0 — target bitrate in kbps
  "rcMode": "cbr",          // string — "cbr" or "vbr"
  "gopSize": 1.0,           // number — GOP size in seconds
  "qpDelta": -4,            // integer — QP delta applied to the encoder baseline
  "roi": {
    "enabled": true,        // boolean — enable ROI (region of interest) encoding
    "qp": 0,                // integer — ROI quantization parameter
    "center": 0.4,          // number — fractional vertical center of ROI (0.0–1.0)
    "steps": 2              // integer — number of ROI concentric steps
  }
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `codec` | string | `"h265"` | `"h264"` or `"h265"` |
| `resolution` | string | `"1920x1080"` | `"WxH"` where W > 0 and H > 0 |
| `fps` | integer | `60` | 1 – 120 |
| `bitrate` | integer | `8192` | > 0 (kbps) |
| `rcMode` | string | `"cbr"` | `"cbr"` or `"vbr"` |
| `gopSize` | number | `1.0` | — |
| `qpDelta` | integer | `-4` | — |
| `roi.enabled` | boolean | `true` | — |
| `roi.qp` | integer | `0` | — |
| `roi.center` | number | `0.4` | — |
| `roi.steps` | integer | `2` | — |

### `image` — sensor orientation

```jsonc
"image": {
  "mirror": false,  // boolean — horizontal mirror
  "flip": false,    // boolean — vertical flip
  "rotate": 0       // integer — rotation in degrees: 0, 90, 180, or 270
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `mirror` | boolean | `false` | — |
| `flip` | boolean | `false` | — |
| `rotate` | integer | `0` | `0`, `90`, `180`, or `270` |

### `telemetry` — telemetry router

```jsonc
"telemetry": {
  "router": "msposd",  // string — telemetry router process: "msposd", "mavfwd", or "none"
  "serial": "ttyS2",   // string — serial device name (without /dev/)
  "osdFps": 20,        // integer — OSD update rate in frames per second
  "baud": 115200        // integer — serial baud rate
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `router` | string | `"msposd"` | `"msposd"`, `"mavfwd"`, or `"none"` |
| `serial` | string | `"ttyS2"` | — |
| `osdFps` | integer | `20` | — |
| `baud` | integer | `115200` | — |

### `recording` — on-board video recording

```jsonc
"recording": {
  "enabled": false,            // boolean — enable SD card recording
  "format": "ts",              // string — container format
  "mode": "mirror",            // string — recording mode
  "maxSeconds": 300,           // integer — maximum clip length in seconds
  "maxMB": 500                 // integer — maximum clip size in megabytes
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `false` | — |
| `format` | string | `"ts"` | — |
| `mode` | string | `"mirror"` | — |
| `maxSeconds` | integer | `300` | — |
| `maxMB` | integer | `500` | — |

### `dynamicLink` — adaptive link controller

Controls the on-drone `dl-applier` process from the `wfbng-dynamic-link` project. When `enabled` is `false` (the default), `dl-applier` is not started and does not appear in `/status`. When `enabled` is `true`, fpvd supervises it as a first-class process and certain link and video fields become read-only (see [Adaptive-link lock](#adaptive-link-lock)).

```jsonc
"dynamicLink": {
  "enabled": false,              // boolean — start the dl-applier process
  "healthTimeoutMs": 10000,      // integer, >= 1000 — watchdog timeout in ms
  "interleavingSupported": true, // boolean — whether the GS supports interleaving
  "minIdrIntervalMs": 500,       // integer, >= 16 — minimum IDR request interval in ms
  "applyStaggerMs": 50,          // integer, 0..500 — stagger between command batches in ms
  "applySubPaceMs": 5,           // integer, 0..50 — pacing between sub-commands in ms
  "mavlinkEnable": true,         // boolean — send MAVLink STATUSTEXT updates
  "osd": {
    "enabled": true,             // boolean — push link-quality data to OSD
    "debugLatency": false        // boolean — include latency debug info in OSD messages
  },
  "roiQp": {
    // ROI-QP curve: maps current bitrate onto a QP delta for the center ROI region.
    // thresholdKbps is the high-bitrate anchor; lowAnchorKbps is the low-bitrate anchor.
    // Require: thresholdKbps > lowAnchorKbps > 0
    "thresholdKbps": 6000,       // integer, > lowAnchorKbps — high-bitrate anchor (kbps)
    "lowAnchorKbps": 2000,       // integer, > 0 — low-bitrate anchor (kbps)
    "floor": -24,                // integer, <= 0 — minimum (most negative) QP delta
    "step": 3                    // integer, >= 1 — QP step per curve segment
  },
  "failsafe": {
    // Per-airframe failsafe ceilings: dl-applier will not exceed these values
    // regardless of what the ground-station controller requests.
    // Note: a legacy "safe" overlay key is migrated to "failsafe" on load.
    "mcs": 1,           // integer, 0..7 — maximum MCS index dl-applier may set
    "k": 8,             // integer, 1..31 — maximum FEC k shard count (k < n, n ≤ 32)
    "n": 12,            // integer, 2..32 — maximum FEC n shard count (n > k, n ≤ 32)
    "depth": 1,         // integer, 1..8 — wfb-ng block depth
    "bandwidth": 20,    // integer, 20 or 40 — maximum channel width in MHz
    "bitrateKbps": 2000 // integer, > 0 — maximum video bitrate in kbps
  }
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `false` | — |
| `healthTimeoutMs` | integer | `10000` | >= 1000 |
| `interleavingSupported` | boolean | `true` | — |
| `minIdrIntervalMs` | integer | `500` | >= 16 |
| `applyStaggerMs` | integer | `50` | 0 – 500 |
| `applySubPaceMs` | integer | `5` | 0 – 50 |
| `mavlinkEnable` | boolean | `true` | — |
| `osd.enabled` | boolean | `true` | — |
| `osd.debugLatency` | boolean | `false` | — |
| `roiQp.thresholdKbps` | integer | `6000` | > `roiQp.lowAnchorKbps` |
| `roiQp.lowAnchorKbps` | integer | `2000` | > 0 |
| `roiQp.floor` | integer | `-24` | <= 0 |
| `roiQp.step` | integer | `3` | >= 1 |
| `failsafe.mcs` | integer | `1` | 0 – 7 |
| `failsafe.k` | integer | `8` | 1 – 31, must be < `failsafe.n` |
| `failsafe.n` | integer | `12` | 2 – 32, must be > `failsafe.k` |
| `failsafe.depth` | integer | `1` | 1 – 8 |
| `failsafe.bandwidth` | integer | `20` | `20` or `40` |
| `failsafe.bitrateKbps` | integer | `2000` | > 0 |

### `services` — user-defined services

An object whose keys are service names and whose values are service definitions. fpvd supervises these processes alongside the built-in subsystems.

```jsonc
"services": {
  "my-service": {
    "enabled": true,            // boolean — start this service
    "exec": "/usr/bin/my-app",  // string, required — path to executable
    "args": ["--port", "9000"], // array of strings — command-line arguments
    "env": {                    // object — additional environment variables
      "LOG_LEVEL": "info"
    },
    "startAfter": ["waybeam"],  // array of strings — service names to wait for
    "restart": "always"         // string — "always", "on-failure", or "never"
  }
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `true` | — |
| `exec` | string | — | required; must not be empty |
| `args` | array of strings | `[]` | — |
| `env` | object | `{}` | string keys and string values |
| `startAfter` | array of strings | `[]` | service names; must not form a cycle |
| `restart` | string | `"always"` | `"always"`, `"on-failure"`, or `"never"` |

**Note:** `startAfter` lists are checked for dependency cycles during validation. A cycle (e.g. service A waits for service B, which waits for A) is rejected with a `validation` error at path `services`.

---

## Ground-station adaptive link controller (`fpvdgs`)

The adaptive link has two halves. The drone runs the **applier** — the [`dynamicLink`](#dynamiclink--adaptive-link-controller) block in the Config schema above, surfaced in the GS unified tree as `dynamicLink.applier` — which receives decisions and applies them to the radio and encoder. The **controller** (the brain that *decides*) runs on the ground station as part of the GS `fpvdgs` daemon, reached through the GS unified [`GET`/`PATCH /config`](#patch-config-gs) + [`POST /apply`](#post-apply-gs) (no separate `/air` proxy — the GS facade fronts both sides).

The controller is an in-process thread that subscribes to wfb-ng's link stats at 10 Hz, runs the dual-gate MCS selector + trailing FEC/bitrate loop, and emits decision packets over UDP to the drone applier at the `droneLink.endpoint` host on `controller.dronePort` (default `9999`). In the unified tree it is the **`dynamicLink.controller`** block (a GS-owned section), with the shared `dynamicLink.enabled` arm toggle alongside it — **distinct from, and differently shaped than, the drone-side `dynamicLink.applier`**:

```jsonc
"dynamicLink": {
  "enabled": false,              // boolean — arm the control loop (SHARED → BOTH; hard-gated on drone reachability)
  "controller": {                // GS-owned
    "maxMcs": 5,                 // integer, 0..7 — upper MCS bound the controller may select
    "radioProfile": "m8812eu2",  // string — packaged radio profile (fpvdgs/dynlink/profiles/<name>.json)
    "dronePort": 9999,           // integer, 1..65535 — drone dynamic-link UDP listener port (host comes from droneLink.endpoint)
    "tuning": {}                 // object — opaque passthrough of advanced policy knobs (see below)
  }
  // (drone-owned dynamicLink.applier.* lives in the same node — see the unified tree)
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `false` | — |
| `controller.maxMcs` | integer | `5` | 0 – 7 |
| `controller.radioProfile` | string | `"m8812eu2"` | a packaged profile name (`fpvdgs/dynlink/profiles/<name>.json`) |
| `controller.dronePort` | integer | `9999` | 1 – 65535 (UDP host is always the `droneLink.endpoint` host) |
| `controller.tuning` | object | `{}` | see [Tuning passthrough](#tuning-passthrough) |

The RF bandwidth the controller targets is **derived from `link.width`** — it is no longer a separate `bandwidth` field. The video rx stream is selected by an internal constant (`videoStreamId = "video"`), no longer a config field. The IDR-token relay config (`idrForward`/`idrPort`) has been removed from the config — the relay **is** always-on GS infrastructure: it listens on `0.0.0.0:11223` and forwards keyframe tokens to the drone at `droneLink.endpoint` host:`11223`, independent of `dynamicLink.enabled` (so it serves static links too).

**Operating model.** Enabling, disabling, or tuning is applied at runtime via `PATCH /config` + `POST /apply` with **no wfb restart** — the GS runner is never bounced for `dynamicLink`-only changes. The controller reads wfb-ng stats on `:8103` (fpvd renders `log_interval = 100` so the feed is 10 Hz). `dynamicLink.enabled` is a **shared** toggle that arms both the GS controller and the drone applier together; the apply lane **hard-gates** it on drone reachability (a toggle while the drone is unreachable returns `409` and changes nothing). `GET /status.dynamicLink` reports the controller state plus a `drone` sub-object (`reachable`, `dynamicLinkActive`, `hello`) so a GS-armed/drone-not mismatch is visible.

#### Tuning passthrough

`tuning` is an opaque object deep-merged over the controller's built-in defaults; the **curated keys above always win** over the same field inside `tuning`. It mirrors the section layout of the standalone `dynamic-link` `gs.yaml` and accepts these sub-objects:

| Sub-object | Selected keys |
|---|---|
| `gate` | `snr_safety_margin`, `hysteresis_up_db` / `hysteresis_down_db`, `emergency_loss_rate`, `emergency_fec_pressure`, `loss_margin_weight`, `fec_margin_weight`, `snr_ema_alpha`, `snr_slope_alpha`, `snr_predict_horizon_ticks` |
| `profile_selection` | `upward_confidence_loops`, `hold_modes_down_ms`, `min_between_changes_ms`, `fast_downgrade` |
| `fec` | `k_bounds.{min,max}`, `base_redundancy_ratio`, `max_redundancy_ratio`, `blocks_per_frame`, `n_loss_threshold`/`n_loss_windows`/`n_loss_step`, `n_recover_windows`/`n_recover_step`, `max_n_escalation`, `depth_max` |
| `smoothing` | `ewma_alpha_rssi`, `ewma_alpha_fec`, `ewma_alpha_burst`, `starvation_threshold_pps` |
| `cooldown` | `min_change_interval_ms_{fec,depth,radio,cross}` |
| `policy` | `starvation_windows`, `bitrate.{utilization_factor,min_bitrate_kbps,max_bitrate_kbps}` |
| `video` | `per_packet_airtime_us`, `max_latency_ms` (predictor latency budget) |
| `safe_defaults` | `video.{k,n}`, `depth`, `mcs` — emitted until the drone HELLO handshake completes |

A complete, production-tuned example for the BL-M8812EU2 airframe ships at **`deploy/gs/config.json`** (installed as the GS overlay on first deploy). Unknown or legacy keys are ignored with a log warning.

> **Tip:** the controller computes the FEC block size `k` from the target bitrate, so a high-bitrate (high-MCS) link uses larger FEC blocks, which raises block-fill latency. If steady latency matters more than FEC granularity, raise `tuning.fec.blocks_per_frame` and/or lower `tuning.fec.k_bounds.max`. Also ensure the link runs at the intended channel **width** — a 10 MHz channel has half the airtime of 20 MHz, so a bitrate sized for 20 MHz will saturate and bufferbloat at 10 MHz.

---

## Error codes

Every error response body has an `"error"` field containing one of the codes below.

### `bad_json`

The request body could not be parsed as JSON.

| | |
|--|--|
| **HTTP status** | 400 |
| **`details`** | not present |

```json
{
  "error": "bad_json",
  "message": "request body not valid JSON"
}
```

---

### `validation`

One or more fields in the configuration object failed a range or constraint check. This error can come from `PATCH /config` (when the merged pending config is invalid) or from `POST /apply` (rare; normally prevented at PATCH time).

| | |
|--|--|
| **HTTP status** | 400 |
| **`details`** | array of `{path, message}` objects |

```jsonc
{
  "error": "validation",
  "message": "schema validation failed",
  "details": [
    {
      "path": "link.mcs",      // dotted path to the invalid field
      "message": "must be 0..7" // human-readable constraint description
    },
    {
      "path": "link.fec",
      "message": "require 1<=k<n<=32"
    }
  ]
}
```

All failing fields are reported in a single response — the array may contain multiple entries.

---

### `dynamic_link_locked`

The PATCH body attempted to write one or more fields that are owned by the adaptive-link controller (`dl-applier`) at runtime, while the merged pending config would have `dynamicLink.enabled == true`. See [Adaptive-link lock](#adaptive-link-lock).

| | |
|--|--|
| **HTTP status** | 400 |
| **`details`** | object with a `locked` array of dotted-path strings |

```json
{
  "error": "dynamic_link_locked",
  "message": "fields owned by dl-applier while dynamicLink.enabled",
  "details": {
    "locked": ["link.mcs", "video.bitrate"]
  }
}
```

`details.locked` lists the specific paths the rejected body tried to write, so a UI can highlight exactly which fields are blocked.

---

### `radio_bringup_failed`

Radio bring-up failed during `POST /apply`. The `radio-up.sh` script returned a non-zero exit code.

| | |
|--|--|
| **HTTP status** | 400 |
| **`details`** | empty array (the error detail is in `GET /status`, not in this response) |

```json
{
  "error": "validation",
  "message": "cannot apply invalid config",
  "details": []
}
```

The radio error text (e.g. `"radio: modprobe 88XXau failed: exit code 1"`) is stored internally and surfaced through `GET /status` in the `lastApply.error` field, not in this HTTP response body. To see the failure reason, call `GET /status` after the 400.

> **Note:** Radio bring-up failures are reported using the `validation` error code because they go through the same `POST /apply` failure path. An empty `details` array combined with a `lastApply.ok == false` status is the distinguishing signal. The pending config is not rolled back — re-attempt apply after correcting the underlying hardware or link settings.

---

## Adaptive-link lock

When `dynamicLink.enabled` is `true`, the `dl-applier` process takes runtime ownership of certain link and video parameters, adjusting them continuously based on link quality. Writing to those same fields through `PATCH /config` while the controller is running would create two writers to the same hardware state, with dl-applier's next decision silently overwriting the operator's value within milliseconds.

To prevent this, `PATCH /config` rejects any body that touches the following paths when the *merged* pending config (after the patch is applied) would have `dynamicLink.enabled == true`:

| Locked path | Owned by |
|-------------|----------|
| `link.mcs` | MCS index sent via `CMD_SET_RADIO` |
| `link.txpower` | TX power via `iw set txpower` |
| `link.fec` | FEC shards `k` and `n` via `CMD_SET_FEC` (entire subtree) |
| `link.width` | Channel width via `CMD_SET_RADIO` |
| `video.bitrate` | Encoder bitrate via encoder HTTP API |
| `video.qpDelta` | Encoder QP delta via encoder HTTP API |
| `video.roi` | Encoder ROI settings via encoder HTTP API (entire subtree) |

Note: `link.channel` is **not** locked — dl-applier never changes frequency.

### Lock evaluation rule

The lock check runs against the *pending* config **after** the incoming patch body has been deep-merged in, not against the current effective config. This has two practical consequences:

- A body that simultaneously sets `dynamicLink.enabled = false` and modifies a locked field is **allowed** — the merged pending has `enabled = false`, so the lock does not apply.
- A body that simultaneously sets `dynamicLink.enabled = true` and modifies a locked field is **rejected** — the merged pending would have both `enabled = true` and a write to a locked field.

### Error response

```json
{
  "error": "dynamic_link_locked",
  "message": "fields owned by dl-applier while dynamicLink.enabled",
  "details": {
    "locked": ["link.mcs", "link.fec.k"]
  }
}
```

### How to safely edit a locked field

Use this three-step sequence:

```bash
# Step 1: disable dynamic link (and optionally change the baseline in the same patch)
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"dynamicLink":{"enabled":false},"link":{"mcs":3}}'

# Step 2: re-enable adaptive link
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"dynamicLink":{"enabled":true}}'

# Step 3: apply
curl -X POST http://127.0.0.1:8080/apply
```

Alternatively, steps 1 and 2 can be two separate PATCH calls if that is clearer in your UI flow. The apply in step 3 is the single commit point for all staged changes.

---

## Worked client flows

### Change channel and video bitrate

A typical pre-flight adjustment: move to a less congested channel and lower the bitrate for a longer-range flight.

```bash
# Stage the changes
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"link":{"channel":100},"video":{"bitrate":4096}}'

# Verify what will be committed
curl 'http://127.0.0.1:8080/config?pending=true' | jq '{link:.link.channel, bitrate:.video.bitrate}'
# {"link": 100, "bitrate": 4096}

# Commit — restarts radio and encoder
curl -X POST http://127.0.0.1:8080/apply
# {"applied":true,"version":2,"restarted":["radio","encoder"]}
```

---

### Enable adaptive link

Enable `dl-applier` supervision with custom failsafe ceilings for a long-range airframe.

```bash
# Stage: configure failsafe ceilings and enable adaptive link
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{
    "dynamicLink": {
      "enabled": true,
      "failsafe": {
        "mcs": 2,
        "bitrateKbps": 6000
      }
    }
  }'

# Apply — starts dl_applier after wfb_video_tx, wfb_tun, waybeam, msposd
curl -X POST http://127.0.0.1:8080/apply
# {"applied":true,"version":3,"restarted":["dl_applier"]}

# Verify dl_applier is running
curl http://127.0.0.1:8080/status | jq '.processes[] | select(.name=="dl_applier")'
# {"name":"dl_applier","pid":471,"state":"running","restarts":0,"lastExitCode":null}

# Now attempting to patch a locked field is rejected:
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"link":{"mcs":5}}'
# HTTP 400
# {"error":"dynamic_link_locked","message":"fields owned by dl-applier while dynamicLink.enabled",
#  "details":{"locked":["link.mcs"]}}
```

---

# Ground-station API (fpvd-GS)

The ground-station `fpvd` (Python, `gs/fpvdgs/`) owns the **GS** wfb radio config and supervises the GS wfb data plane, replacing the stock `wfb-server`/`S98wifibroadcast`. A supervisor process owns the config + this HTTP API; a runner child imports the `wfb_ng` library to run `wfb_rx`/`wfb_tx` and continues to serve the wfb stats APIs on `:8103` (JSON) / `:8003` (MsgPack), so `wfb-cli` and the adaptive-link controller are unaffected.

Binds `0.0.0.0:8080` (same posture as the drone). Source of truth for GS-local config: `/etc/fpvd/{defaults,config}.json`; the daemon renders these to `/etc/wifibroadcast.cfg` (a generated artifact — do not edit). The stage→apply lifecycle (effective vs pending, `?pending=true`) is identical to the drone's.

**The GS is a unified config facade.** Unlike the drone, the GS does not expose its own narrow schema as the front door. `GET /config` returns ONE merged tree composed of the GS-local config (radio/wfb/pixelpilot/controller) **and** the drone's config (video/image/telemetry/recording/applier), fetched live over the link. `PATCH /config` routes each leaf to wherever it lives (GS pending, or proxied to the drone's pending). `POST /apply` is a multi-lane router. There are **no** separate `/link`, `/link/apply`, or `/air/*` routes — they have been retired into the unified `/config` + `/apply`.

The GS surface is therefore: `GET/PATCH /config`, `POST /apply`, `GET /status`, `GET /healthz`, `POST /reset`, `GET /defaults`.

### Differences from the drone API at a glance

| | Drone | Ground station |
|--|--|--|
| Config schema | flat: link + video + image + telemetry + recording + dynamicLink + services | **unified Option-C tree** merged from GS-local + drone (see below) |
| Front door for *all* config | `PATCH /config` (its own flat schema) | `PATCH /config` (the unified tree — routes each leaf to GS or drone) |
| `GET /healthz` body | `{}` | `{"ok": true}` |
| Error body | `{error, message, details}` | `{"error": "<message>"}` |
| Extra endpoints | — | none (drone reached *through* the unified `/config`, not a proxy) |

> The drone remains an independent daemon with its own flat schema (documented in the sections above); it is still directly reachable for CLI/diagnostic use. The GS merely **presents** the drone's fields inside the unified tree and proxies the relevant PATCH/apply to it.

## The unified config tree

`GET /config` returns this single Option-C tree. Each section is owned by one side; `PATCH` and `apply` route accordingly (see the routing table). Drone-owned subtrees are **last-seen** snapshots — when the drone is unreachable they reflect the last successful read and a UI should gray them out (see `_meta`).

```jsonc
{
  "_meta": {                                                 // READ-ONLY (PATCHing it → 400)
    "droneReachable": true,
    "droneLastSeen": "2026-06-10T…Z",
    "droneStale": false
  },
  "link": {
    "channel": 132, "width": 20, "linkId": 7669206,          // SHARED → BOTH GS + drone
    "beamforming": { "enabled": false },                     // SHARED → BOTH
    "gs":    { "region": "US", "rxpower": null,              // → GS (the receiver)
               "wlans": "auto" },
    "drone": { "mcs": 3, "txpower": 25, "txpowerCurve": null,// → DRONE (last-seen; stale when unreachable)
               "fec": {…}, "stbc": false, "ldpc": false,
               "mtu": 1500, "wlanAdapter": null }
  },
  "dynamicLink": {
    "enabled": false,                                        // SHARED → BOTH (hard-gated on drone reachability)
    "controller": { "maxMcs": 5, "radioProfile": "m8812eu2", // → GS
                    "dronePort": 9999, "tuning": {} },
    "applier": { "healthTimeoutMs": 10000,                   // → DRONE (last-seen)
                 "interleavingSupported": true, "minIdrIntervalMs": 500,
                 "applyStaggerMs": 50, "applySubPaceMs": 5,
                 "osd": {…}, "roiQp": {…}, "failsafe": {…},
                 "bitrate": {…}, "fec": {…} }
  },
  "video": {…}, "image": {…}, "telemetry": {…},               // → DRONE passthrough (last-seen)
  "recording": {…}, "services": {…},
  "wfb": {…}, "pixelpilot": {…},                              // → GS passthrough
  "droneLink": { "endpoint": "http://10.5.0.10:8080" }       // → GS (where the GS reaches the drone fpvd)
}
```

The drone-side sections (`link.drone`, `dynamicLink.applier`, `video`, `image`, `telemetry`, `recording`, `services`) keep the same field shapes as the drone's flat schema documented earlier — `link.drone.*` is the drone `link` minus the shared keys, and `dynamicLink.applier.*` is the drone's `dynamicLink` block. The GS-side `wfb`, `droneLink`, and `pixelpilot` sections, and the `link.gs.*` fields, are the GS-local config (rxpower/region/wlans, the wfb data-plane render config, the drone endpoint, and the PixelPilot launch knobs).

### Section → side routing

| Section / path | Side | Notes |
|---|---|---|
| `link.channel`, `link.width`, `link.linkId`, `link.beamforming` | **SHARED → BOTH** | The shared-link lane: GS-first retune/bounce + best-effort drone push. |
| `dynamicLink.enabled` | **SHARED → BOTH** | Hard-gated on drone reachability (see policy). |
| `link.gs.*` (`region`, `rxpower`, `wlans`) | **GS** | Receiver-only. |
| `dynamicLink.controller.*` | **GS** | The in-process MCS/FEC selector. |
| `wfb`, `pixelpilot`, `droneLink` | **GS** | wfb data-plane render config, PixelPilot launch knobs, drone endpoint. |
| `link.drone.*` (`mcs`, `txpower`, `fec`, …) | **DRONE** | Last-seen; grayed when `_meta.droneStale`. |
| `dynamicLink.applier.*` | **DRONE** | The on-drone applier block. |
| `video`, `image`, `telemetry`, `recording`, `services` | **DRONE** | Drone passthrough; last-seen + grayed when stale. |

### `_meta`

`_meta` is a read-only window into the GS's view of the drone:

- `droneReachable` — whether the most recent drone probe succeeded.
- `droneLastSeen` — ISO-8601 UTC timestamp of the last successful drone read (the snapshot the drone subtrees reflect); `null` if never seen.
- `droneStale` — `true` when the drone is currently unreachable but a prior snapshot is being shown. A UI should gray out every DRONE-owned subtree when this is `true`.

`PATCH`ing `_meta` (or any unknown top-level section) is rejected with `400`.

## Shared endpoints (GS behavior)

`GET /healthz` → `200 {"ok": true}`. `GET /defaults` → the GS-local baseline. `POST /reset` → `{"reset": true}` (drops the GS overlay, re-renders, bounces the runner). `GET /config[?pending=true]` → the unified effective/pending tree.

### PATCH /config (GS)

Deep-merges a partial unified tree into pending and **routes each leaf** to the side that owns it: GS-owned leaves land in the GS pending; drone-owned leaves are proxied to the drone's pending (`PATCH /config` over `droneLink.endpoint`). Shared link leaves are staged for the coordinator.

```bash
# GS-local leaf (controller):
curl -X PATCH http://127.0.0.1:8080/config -H 'content-type: application/json' \
  -d '{"dynamicLink":{"controller":{"maxMcs":4}}}'

# drone-owned leaf (routed to the drone's pending):
curl -X PATCH http://127.0.0.1:8080/config -H 'content-type: application/json' \
  -d '{"video":{"bitrate":9000}}'
```

| Code | Meaning |
|------|---------|
| 200 | Patch accepted; body is the updated pending unified tree. |
| 400 | `_meta` or an unknown section was written, or a drone-leaf failed the **drone's** validation (surfaced as `400` naming the offending field). |

Validation of drone-owned leaves is performed by the drone (the GS does not re-model the drone schema); a drone-side rejection is relayed back as a `400` identifying the field.

### POST /apply (GS)

`POST /apply` is a **3-lane router**. It dispatches the staged changes to the lanes that own them and returns a per-lane result. No body is required.

- **GS-local lane** — `wfb`, `pixelpilot`, `dynamicLink.controller`, and the GS side of `dynamicLink.enabled`. Renders the cfg + bounces the runner (only when wfb config actually changed), restarts PixelPilot for `pixelpilot.*`, and routes the in-process controller for `dynamicLink.*`.
- **Shared-link lane** — `link.{channel,width,linkId,beamforming}`. The coordinator: GS-first (retune-or-bounce the GS radio) then a best-effort push of the shared subset to the drone (`PATCH /config` + `POST /apply` over `droneLink.endpoint`).
- **Drone lane** — `link.drone.*`, `dynamicLink.applier.*`, `video`/`image`/`telemetry`/`recording`/`services`. The drone-routed PATCHes already landed in the drone's pending; this lane fires the drone's `POST /apply`.

#### Per-field apply policy

| Situation | Behavior |
|---|---|
| Shared link (`channel`/`width`/`linkId`/`beamforming`) with drone **unreachable** | **Soft-degrade** — apply GS-only, return `200` with `sharedLink.droneApplied:false`. The GS still moves (this is how you tune the GS to a drone state you already know / re-establish a link). |
| `dynamicLink.enabled` toggle with drone **unreachable** | **Hard-gate** — `409`, nothing changes (the GS controller and drone applier must arm together). |
| Other drone-only fields (`video`, `link.drone`, `dynamicLink.applier`, …) with drone **unreachable** | Require the drone — the drone lane cannot fire; surfaced as an error. |

#### Per-lane result shape

```jsonc
{
  "applied": true,
  "sharedLink": { "gsApplied": true, "droneApplied": false, "droneReachable": false, "inSync": false },
  "gs":        { "applied": true, "wfbBounced": false },
  "drone":     { "fired": true, "applied": true }
}
```

Each lane sub-object is present only when that lane had staged changes. `sharedLink.droneApplied:false` on an otherwise-`200` response is the soft-degrade signal; `inSync` is best-effort.

| Code | Meaning |
|------|---------|
| 200 | Lanes dispatched; body reports per-lane outcome (incl. soft-degraded shared link). |
| 409 | `dynamicLink.enabled` toggle staged while the drone is unreachable (hard-gate); nothing changed. |
| 500 | A lane failed (e.g. the GS runner failed and was rolled back to last-good cfg). |

### GET /status (GS)

```jsonc
{
  "fpvd":   {"version": "0.1.0", "uptimeMs": 41717},
  "runner": {"running": true, "pid": 10447, "restarts": 0,
             "autoRestarts": 0, "lastExit": null, "fault": false},
  "radio":  [
    {"wlan": "wlx84fc146c36e6", "type": "monitor", "channel": 132,
     "freqMhz": 5660, "widthMhz": 40, "txpowerDbm": 19.0}
  ],
  "link":   {"linkId": 7669206, "droneReachable": true, "inSync": null},
  "idrRelay": {"running": true, "listen": "0.0.0.0:11223"}
}
```

- `runner` — the supervised wfb runner: `restarts` counts operator bounces, `autoRestarts` counts crash auto-restarts, `fault` is the crash-loop guard.
- `radio` — one entry per wlan, parsed from `iw dev <wlan> info`.
- `link.droneReachable` — cached probe of the drone fpvd. `link.inSync` — best-effort: set after a shared-link apply reached the drone; `null`/`false` after a GS-only (soft-degraded) apply even when the widths happen to match (compare against the `link.drone` subtree of `GET /config` to confirm).
- `idrRelay` — the always-on keyframe relay: `{ "running": bool, "listen": "0.0.0.0:11223" | null }`. `listen` is `null` (and `running` `false`) only if the `:11223` bind failed. Independent of `dynamicLink.enabled`; serves static and adaptive links alike.

The `dynamicLink` and `pixelpilot` status blocks are documented under their config sections below.

## PixelPilot managed service

fpvd-GS spawns and supervises the `pixelpilot` display binary (PixelPilot FPV Decoder for Rockchip ≥1.3) as a first-class managed child alongside the wfb data plane. The `pixelpilot` config block is GS-owned and surfaces under the unified `/config` tree (PixelPilot reaches any drone-side fields it needs through that same unified `/config`, not a `/air` proxy). It flows through the standard `PATCH /config` + `POST /apply` lifecycle with **granular apply**: a `pixelpilot.*` change restarts PixelPilot only (the wfb runner and radio are untouched), and conversely a `link.*` or `wfb.*` change leaves PixelPilot running. Flag order in the rendered argv is canonical (stable); the getopt-style parser accepts any order.

### Config block

```json
"pixelpilot": {
  "enabled": true,
  "bin": "/usr/bin/pixelpilot",
  "env": {},
  "configPath": "/etc/pixelpilot.yaml",
  "osdConfigPath": "/etc/pixelpilot/osd.json",
  "screenMode": "1920x1080@60",
  "videoScale": 1.0,
  "codec": "h265",
  "rtpPort": 5600,
  "rtpJitterMs": 1,
  "dvr": {
    "framerate": 60,
    "dir": "/media/dvr",
    "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
    "fmp4": true,
    "sequencedFiles": true,
    "osd": false,
    "mode": "raw",
    "maxSizeMb": 4000,
    "reencCodec": "h264",
    "reencBitrate": 8000,
    "reencFps": 30,
    "reencResolution": "1080p"
  },
  "extraArgs": []
}
```

| Key | CLI flag | Default | Notes |
|---|---|---|---|
| `enabled` | — | `true` | Toggle supervision via `PATCH /config` + `POST /apply`. |
| `bin` | — | `/usr/bin/pixelpilot` | Path to the binary. |
| `env` | — | `{}` | Extra child-process environment (e.g. `{"LD_LIBRARY_PATH":"/usr/lib/pixelpilot95"}`). Merged over `os.environ` by the supervisor. |
| `configPath` | `--config` | `/etc/pixelpilot.yaml` | Main pixelpilot config file. |
| `osdConfigPath` | `--osd-config` | `/etc/pixelpilot/osd.json` | OSD layout config. |
| `screenMode` | `--screen-mode` | `1920x1080@60` | HDMI output mode. |
| `videoScale` | `--video-scale` | `1.0` | Display scale factor. |
| `codec` | `--codec` | `h265` | Video codec (`h264` or `h265`). |
| `rtpPort` | `-p` | `5600` | RTP video input port. |
| `rtpJitterMs` | `--rtp-jitter-ms` | `1` | RTP jitter buffer (ms). |
| `dvr.framerate` | `--dvr-framerate` | `60` | DVR frame rate. |
| `dvr.dir` | — | `/media/dvr` | DVR output directory (joined with `dvr.template`). |
| `dvr.template` | `--dvr-template` | `record_%Y-%m-%d_%H-%M-%S.mp4` | DVR filename template (joined with `dvr.dir`). |
| `dvr.fmp4` | `--dvr-fmp4` | `true` | Fragmented MP4 output. |
| `dvr.sequencedFiles` | `--dvr-sequenced-files` | `true` | Sequenced output files. |
| `dvr.osd` | `--dvr-osd` | `false` | Burn OSD into DVR. |
| `dvr.mode` | `--dvr-mode` | `raw` | DVR mode (`raw` or `reencode`). |
| `dvr.maxSizeMb` | `--dvr-max-size` | `4000` | Max DVR clip size (MB). |
| `dvr.reencCodec` | `--dvr-reenc-codec` | `h264` | Re-encode codec. |
| `dvr.reencBitrate` | `--dvr-reenc-bitrate` | `8000` | Re-encode bitrate (kbps). |
| `dvr.reencFps` | `--dvr-reenc-fps` | `30` | Re-encode frame rate. |
| `dvr.reencResolution` | `--dvr-reenc-resolution` | `1080p` | Re-encode resolution. |
| `extraArgs` | — | `[]` | Verbatim-appended flags (escape hatch for un-modeled options). |

```bash
# Change display scale and apply (restarts PixelPilot only):
curl -X PATCH http://10.18.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"pixelpilot":{"videoScale":1.5}}'
curl -X POST http://10.18.0.1:8080/apply

# Set LD_LIBRARY_PATH for a perf-build lib dir:
curl -X PATCH http://10.18.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{"pixelpilot":{"env":{"LD_LIBRARY_PATH":"/usr/lib/pixelpilot95"}}}'
curl -X POST http://10.18.0.1:8080/apply
```

### Status

`GET /status` includes a `pixelpilot` block alongside `runner`, `radio`, and `link`:

```jsonc
{
  "pixelpilot": {
    "enabled": true,
    "running": true,
    "pid": 1842,
    "restarts": 0,
    "autoRestarts": 0,
    "lastExit": null,
    "fault": false
  }
}
```

When `enabled` is `false` the block is `{"enabled": false, "running": false}`. `restarts` counts operator-initiated bounces (via apply); `autoRestarts` counts crash auto-restarts; `fault` is the crash-loop guard (trips after too many rapid auto-restarts).

### Deploy takeover

`deploy/gs/deploy.sh` stops and moves the stock `S*pixelpilot*` init script to `/root/fpvd-gs-rollback/` so fpvd-GS owns the PixelPilot lifecycle. `deploy/gs/rollback.sh` restores it for a full revert. The device-provisioned files — `/etc/pixelpilot/pixelpilot.yaml`, `/etc/pixelpilot/config_osd.json`, and the `/usr/bin/pixelpilot` binary — are left in place; fpvd points at them, it does not create or manage them.

## Errors (GS)

Error responses are a single field — `{"error": "<human message>"}` — with HTTP `400` (validation/schema, `_meta`/unknown section, or a drone-relayed field rejection), `404` (no route), `409` (`dynamicLink.enabled` hard-gate: toggled while the drone is unreachable), or `500` (apply/runner/lane failure).
