# fpvd HTTP API

`fpvd` is a supervisor daemon for OpenIPC FPV drone stacks. It owns a unified configuration object covering the radio link, video encoder, telemetry router, on-board recording, and optional adaptive-link control, and it supervises those subsystems as child processes. All configuration is managed through this HTTP+JSON API: stage changes incrementally, validate them, then commit in one `POST /apply`.

The server binds on `0.0.0.0:8080` by default (overridable with `--port`). There is no authentication — the API is reachable over the wfb-ng tunnel (`10.5.0.10/24`) and any LAN interface, all of which are assumed to be private networks. Clients include ground-station apps, `curl` scripts, and the `wfbng-dynamic-link` ground-station component.

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
  "link": {"channel": 161, "width": 20, "txpower": 1, "mcs": 2,
           "fec": {"k": 8, "n": 12}, "stbc": false, "ldpc": false,
           "linkId": 7669206, "mtu": 1500, "wlanAdapter": null},
  "video": {"codec": "h265", "resolution": "1920x1080", "fps": 60,
            "bitrate": 8192, "rcMode": "cbr", "gopSize": 1.0, "qpDelta": -4,
            "roi": {"enabled": true, "qp": 0, "center": 0.4, "steps": 2}},
  "image": {"mirror": false, "flip": false, "rotate": 0},
  "telemetry": {"router": "msposd", "serial": "ttyS2", "osdFps": 20, "baud": 115200},
  "recording": {"enabled": false, "dir": "/mnt/mmcblk0p1", "format": "ts",
                "mode": "mirror", "maxSeconds": 300, "maxMB": 500},
  "snapshot": {"enabled": true, "quality": 80},
  "dynamicLink": {
    "enabled": false, "healthTimeoutMs": 10000,
    "interleavingSupported": true, "minIdrIntervalMs": 500,
    "applyStaggerMs": 50, "applySubPaceMs": 5, "mavlinkEnable": true,
    "osd": {"enabled": true, "debugLatency": false},
    "roiQp": {"thresholdKbps": 6000, "lowAnchorKbps": 2000, "floor": -24, "step": 3},
    "safe": {"mcs": 1, "k": 8, "n": 12, "depth": 1,
             "bandwidth": 20, "txPowerDbm": 20, "bitrateKbps": 2000}
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
  "link": {"channel": 161, "width": 20, "txpower": 1, "mcs": 2,
           "fec": {"k": 8, "n": 12}, "stbc": false, "ldpc": false,
           "linkId": 7669206, "mtu": 1500, "wlanAdapter": null},
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
- `video.*`, `image.*`, `snapshot.*`, `recording.*` changes restart the encoder (waybeam).
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
    "adapterId": "0bda:8812"  // string or null — USB vendor:product id
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
    "adapterId": "0bda:8812"
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
  "txpower": 1,         // integer, 1..63 — TX power (driver units)
  "mcs": 2,             // integer, 0..7 — MCS index
  "fec": {
    "k": 8,             // integer, 1..31 — FEC data shards (k < n, n ≤ 32)
    "n": 12             // integer, 2..32 — FEC total shards (n > k, n ≤ 32)
  },
  "stbc": false,        // boolean — enable STBC
  "ldpc": false,        // boolean — enable LDPC
  "linkId": 7669206,    // integer — wfb-ng link ID (must match GS)
  "mtu": 1500,          // integer — packet MTU in bytes
  "wlanAdapter": null   // string or null — force a specific wlan interface name;
                        //                  null = auto-detect via radio-up.sh
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `channel` | integer | `161` | 1 – 200 |
| `width` | integer | `20` | `20` or `40` |
| `txpower` | integer | `1` | 1 – 63 |
| `mcs` | integer | `2` | 0 – 7 |
| `fec.k` | integer | `8` | 1 – 31, must be < `fec.n` |
| `fec.n` | integer | `12` | 2 – 32, must be > `fec.k` |
| `stbc` | boolean | `false` | — |
| `ldpc` | boolean | `false` | — |
| `linkId` | integer | `7669206` | — |
| `mtu` | integer | `1500` | — |
| `wlanAdapter` | string \| null | `null` | interface name or `null` |

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
  "dir": "/mnt/mmcblk0p1",    // string — destination directory
  "format": "ts",              // string — container format
  "mode": "mirror",            // string — recording mode
  "maxSeconds": 300,           // integer — maximum clip length in seconds
  "maxMB": 500                 // integer — maximum clip size in megabytes
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `false` | — |
| `dir` | string | `"/mnt/mmcblk0p1"` | — |
| `format` | string | `"ts"` | — |
| `mode` | string | `"mirror"` | — |
| `maxSeconds` | integer | `300` | — |
| `maxMB` | integer | `500` | — |

### `snapshot` — JPEG snapshot service

```jsonc
"snapshot": {
  "enabled": true,  // boolean — enable snapshot endpoint in waybeam
  "quality": 80     // integer — JPEG quality (0–100)
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `true` | — |
| `quality` | integer | `80` | — |

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
  "safe": {
    // Per-airframe failsafe ceilings: dl-applier will not exceed these values
    // regardless of what the ground-station controller requests.
    "mcs": 1,           // integer, 0..7 — maximum MCS index dl-applier may set
    "k": 8,             // integer, 1..31 — maximum FEC k shard count (k < n, n ≤ 32)
    "n": 12,            // integer, 2..32 — maximum FEC n shard count (n > k, n ≤ 32)
    "depth": 1,         // integer, 1..8 — wfb-ng block depth
    "bandwidth": 20,    // integer, 20 or 40 — maximum channel width in MHz
    "txPowerDbm": 20,   // integer, -10..30 — maximum TX power in dBm
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
| `safe.mcs` | integer | `1` | 0 – 7 |
| `safe.k` | integer | `8` | 1 – 31, must be < `safe.n` |
| `safe.n` | integer | `12` | 2 – 32, must be > `safe.k` |
| `safe.depth` | integer | `1` | 1 – 8 |
| `safe.bandwidth` | integer | `20` | `20` or `40` |
| `safe.txPowerDbm` | integer | `20` | -10 – 30 |
| `safe.bitrateKbps` | integer | `2000` | > 0 |

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

Enable `dl-applier` supervision with custom safe ceilings for a long-range airframe.

```bash
# Stage: configure safe ceilings and enable adaptive link
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{
    "dynamicLink": {
      "enabled": true,
      "safe": {
        "mcs": 2,
        "bitrateKbps": 6000,
        "txPowerDbm": 25
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
