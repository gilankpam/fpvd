# fpvd HTTP API

`fpvd` is a supervisor daemon for OpenIPC FPV drone stacks. It owns a unified configuration object covering the radio link, video encoder, telemetry router, on-board recording, and optional adaptive-link control, and it supervises those subsystems as child processes. All configuration is managed through this HTTP+JSON API: stage changes incrementally, validate them, then commit in one `POST /apply`.

The server binds on `0.0.0.0:8080` by default (overridable with `--port`). There is no authentication — the API is reachable over the wfb-ng tunnel (`10.5.0.10/24`) and any LAN interface, all of which are assumed to be private networks. Clients include ground-station apps, `curl` scripts, and the `wfbng-dynamic-link` ground-station component.

> **Two daemons.** This repo ships two `fpvd` builds: the **drone** daemon (C++, `drone/`) documented in the sections below, and the **ground-station** daemon (Python, `gs/`) documented in [Ground-station API (fpvd-GS)](#ground-station-api-fpvd-gs). They share the stage→apply lifecycle and config-object conventions, but differ in routing: the drone serves its config tree at the root (`/config`, `/apply`, `/reset`, `/defaults`, `/status`), while the GS serves its own config tree under `/gs/*` (`/gs/config`, `/gs/apply`, `/gs/reset`, `/gs/defaults`, `/gs/status`) and adds an opaque `/air/*` proxy that forwards to the drone daemon. `GET /healthz` stays at the root on both. The GS also has a different config schema (radio-only, no encoder/telemetry/recording). Unless a section is under that heading, it describes the **drone** daemon.

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

`POST /apply` validates pending, persists the full config to `/etc/fpvd/config.json`, and restarts only the affected subsystems.

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

Returns the code-default configuration — the values baked into the `Config{}` struct at compile time. There is no `/rom/etc/fpvd/defaults.json` file; the daemon serialises `Config{}` directly and returns it. This is the starting point before any user overlay is applied. Use it to show "reset to defaults" previews or to build a diff display.

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
  "link": {"channel": 132, "width": 20, "txPowerDbm": 20, "mcs": 2,
           "fec": {"mode": "swfec", "k": 8, "n": 12, "overheadPct": 50, "deadlineMs": 30},
           "stbc": true, "ldpc": true,
           "linkId": 7669206, "mtu": 1500, "wlanAdapter": null},
  "video": {"codec": "h265", "resolution": "1920x1080", "fps": 60,
            "bitrate": 8192, "rcMode": "cbr", "gopSize": 1.0, "qpDelta": -4,
            "sensorBin": "",
            "roi": {"enabled": true, "qp": 0, "center": 0.4, "steps": 2}},
  "image": {"mirror": false, "flip": false, "rotate": 0},
  "telemetry": {"router": "msposd", "serial": "ttyS2", "osdFps": 20, "baud": 115200},
  "recording": {"enabled": false, "format": "ts",
                "mode": "mirror", "maxSeconds": 300, "maxMB": 500},
  "osd": {"enabled": true},
  "dynamicLink": {
    "enabled": false, "healthTimeoutMs": 10000,
    "applyStaggerMs": 50, "applySubPaceMs": 5,
    "roiQp": {"thresholdKbps": 6000, "lowAnchorKbps": 2000, "floor": -24, "step": 3},
    "safe": {"mcs": 1, "k": 8, "n": 12, "overheadPct": 100, "deadlineMs": 30, "bitrateKbps": 2000},
    "compute": {"minBitrateKbps": 1000, "maxBitrateKbps": 24000,
                "baseRedundancyRatio": 0.5, "blocksPerFrame": 2.0, "kMin": 2, "kMax": 50}
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
  "link": {"channel": 132, "width": 20, "txPowerDbm": 20, "mcs": 2,
           "fec": {"mode": "swfec", "k": 8, "n": 12, "overheadPct": 50, "deadlineMs": 30},
           "stbc": true, "ldpc": true,
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

Commits the pending configuration to effective, persists the full config to `/etc/fpvd/config.json`, and restarts only the subsystems affected by the change. This is the single action that makes staged changes take effect on the drone.

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

Removes `/etc/fpvd/config.json` and resets pending to the code defaults (same as `GET /defaults`). Does **not** call `POST /apply` — the effective configuration and running processes are unchanged until the client calls apply.

**Request body:** none

**Response body**

```jsonc
{"reset": true}
```

**Status codes**

| Code | Meaning |
|------|---------|
| 200  | `/etc/fpvd/config.json` removed; pending is now the code defaults. |

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
  "channel": 132,       // integer, 1..200 — Wi-Fi channel number
  "width": 20,          // integer, 10, 20, or 40 — channel width in MHz. Operator-settable
               // while dynamicLink.enabled (ground change; the GS 10<->20 retune is
               // live). 40 MHz requires dynamicLink.enabled=false.
  "txPowerDbm": 20,     // integer, -10..30 — TX power in dBm (converted x100 to mBm at the radio edge)
  "mcs": 2,             // integer, 0..7 — MCS index
  "fec": {
    "mode": "swfec",    // string — "rs" or "swfec"
    "k": 8,             // integer, 1..31 — FEC data shards (k < n, n ≤ 32; rs mode)
    "n": 12,            // integer, 2..32 — FEC total shards (n > k, n ≤ 32; rs mode)
    "overheadPct": 50,  // integer, 0..255 — swfec repair budget
    "deadlineMs": 30    // integer, 1..255 — swfec recovery window
  },
  "stbc": true,         // boolean — enable STBC
  "ldpc": true,         // boolean — enable LDPC
  "linkId": 7669206,    // integer — wfb-ng link ID (must match GS)
  "mtu": 1500,          // integer — packet MTU in bytes
  "wlanAdapter": null   // string or null — force a specific wlan interface name;
                        //                  null = auto-detect via radio-up.sh
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `channel` | integer | `132` | 1 – 200 |
| `width` | integer | `20` | `10`, `20`, or `40` |
| `txPowerDbm` | integer | `20` | -10 – 30 |
| `mcs` | integer | `2` | 0 – 7 |
| `fec.mode` | string | `"swfec"` | `"rs"` or `"swfec"` |
| `fec.k` | integer | `8` | 1 – 31, must be < `fec.n` |
| `fec.n` | integer | `12` | 2 – 32, must be > `fec.k` |
| `fec.overheadPct` | integer | `50` | 0 – 255 |
| `fec.deadlineMs` | integer | `30` | 1 – 255 |
| `stbc` | boolean | `true` | — |
| `ldpc` | boolean | `true` | — |
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
  "gopSize": 1.0,           // number — GOP size in seconds (ignored when resilience != "off")
  "resilience": "off",      // string — waybeam error-resilience preset (see note below)
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
| `gopSize` | number | `1.0` | — (ignored when `resilience` ≠ `"off"`) |
| `resilience` | string | `"off"` | `"off"`, `"rescue"`, `"quality"`, `"sprint"`, `"racing"`, `"endurance"`, `"patrol"`, `"rally"`, `"range"`, `"fpv"` |
| `qpDelta` | integer | `-4` | — |
| `roi.enabled` | boolean | `true` | — |
| `roi.qp` | integer | `0` | — |
| `roi.center` | number | `0.4` | — |
| `roi.steps` | integer | `2` | — |

#### `video.resilience` — error-resilience preset

`resilience` selects a waybeam error-resilience profile. waybeam derives intra-refresh (rolling GDR stripe), the SVC-T reference pyramid, and the GOP length from the named preset — there are no separate per-feature knobs. `"off"` (the default) preserves the classic GOP behavior driven by `gopSize`; any other preset makes waybeam own intra-refresh and GOP, and `gopSize` is then ignored.

- **Validation:** unknown values are rejected by `PATCH /config` with a `validation` error on path `video.resilience`.
- **Apply class:** a change is **restart-class** — `POST /apply` rewrites `/etc/waybeam.json` and bounces the encoder (it appears as `"encoder"` in the `restarted` array), rather than being hot-pushed to the running encoder.
- **Not adaptive-link-locked:** unlike `video.bitrate`/`qpDelta`/`roi`, `resilience` is an operator-owned flight-profile choice and stays editable while `dynamicLink.enabled` (see [Adaptive-link lock](#adaptive-link-lock)).

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
  "enabled": false,              // boolean — arm the dynamic-link applier
  "healthTimeoutMs": 10000,      // integer, >= 1000 — watchdog timeout in ms
  "applyStaggerMs": 50,          // integer, 0..500 — stagger between command batches in ms
  "applySubPaceMs": 5,           // integer, 0..50 — pacing between sub-commands in ms
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
    // Safe-mode floor: applied when the GS falls back to the lowest rung.
    // bandwidth and txPowerDbm are NOT in safe — they are derived (from link.width
    // and the per-MCS TX-power curve in txpower_curve.hpp).
    "mcs": 1,              // integer, 0..7 — safe-mode MCS
    "k": 8,                // integer, 1..31 — safe-mode FEC k (k < n, n ≤ 32)
    "n": 12,               // integer, 2..32 — safe-mode FEC n (n > k, n ≤ 32)
    "overheadPct": 100,    // integer, 0..255 — swfec repair budget at safe rung
    "deadlineMs": 30,      // integer, 1..255 — swfec recovery window at safe rung
    "bitrateKbps": 2000    // integer, > 0 — safe-mode video bitrate in kbps
  },
  "compute": {
    // Bitrate and FEC geometry derivation knobs (rarely need tuning).
    "minBitrateKbps": 1000,        // integer, > 0 — floor for computed video bitrate
    "maxBitrateKbps": 24000,       // integer, > minBitrateKbps — ceiling for computed video bitrate
    "baseRedundancyRatio": 0.5,    // number, > 0 — n/k − 1 (e.g. 0.5 → k=8, n=12)
    "blocksPerFrame": 2.0,         // number, > 0 — FEC blocks per video frame
    "kMin": 2,                     // integer, >= 1 — minimum FEC k
    "kMax": 50                     // integer, >= kMin — maximum FEC k
  }
}
```

> **`osd.enabled` is a top-level key** (`"osd": {"enabled": true}`), not inside `dynamicLink`. The OSD overlay runs regardless of whether dynamic-link is armed.

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `false` | — |
| `healthTimeoutMs` | integer | `10000` | >= 1000 |
| `applyStaggerMs` | integer | `50` | 0 – 500 |
| `applySubPaceMs` | integer | `5` | 0 – 50 |
| `roiQp.thresholdKbps` | integer | `6000` | > `roiQp.lowAnchorKbps` |
| `roiQp.lowAnchorKbps` | integer | `2000` | > 0 |
| `roiQp.floor` | integer | `-24` | <= 0 |
| `roiQp.step` | integer | `3` | >= 1 |
| `safe.mcs` | integer | `1` | 0 – 7 |
| `safe.k` | integer | `8` | 1 – 31, must be < `safe.n` |
| `safe.n` | integer | `12` | 2 – 32, must be > `safe.k` |
| `safe.overheadPct` | integer | `100` | 0 – 255 |
| `safe.deadlineMs` | integer | `30` | 1 – 255 |
| `safe.bitrateKbps` | integer | `2000` | > 0 |
| `compute.minBitrateKbps` | integer | `1000` | > 0, must be < `compute.maxBitrateKbps` |
| `compute.maxBitrateKbps` | integer | `24000` | > `compute.minBitrateKbps` |
| `compute.baseRedundancyRatio` | number | `0.5` | > 0 |
| `compute.blocksPerFrame` | number | `2.0` | > 0 |
| `compute.kMin` | integer | `2` | >= 1, must be <= `compute.kMax` |
| `compute.kMax` | integer | `50` | >= `compute.kMin` |

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

The adaptive link has two halves. The drone runs the **applier** — the [`dynamicLink`](#dynamiclink--adaptive-link-controller) block in the Config schema above — which receives decisions and applies them to the radio and encoder. The **controller** (the brain that *decides*) runs on the ground station as a separate daemon, `fpvdgs`, with its own HTTP+JSON API on the GS (same shapes as this document, but rooted under `/gs/*`: `GET`/`PATCH /gs/config`, `POST /gs/apply`, `GET /gs/status`, plus an opaque `/air/*` proxy to the drone fpvd).

The controller is an in-process thread that subscribes to wfb-ng's link stats at 10 Hz, runs the probe-driven MCS selector, and emits `{mcs}`-only decision packets over UDP to the drone applier (`drone.host`:`dynamicLink.dronePort`, default `:9999`). It is configured by the GS daemon's own `dynamicLink` block — **distinct from, and differently shaped than, the drone-side `dynamicLink` above**:

```jsonc
"dynamicLink": {
  "enabled": false,            // boolean — arm the in-process control loop
  "maxMcs": 5,                 // integer, 0..7 — operator MCS ceiling
  "dronePort": 9999,           // integer, 1..65535 — drone DL UDP port (host comes from drone.host)

  // Selector: probe-driven promote + reactive demote + timing/cadence
  "selector": {
    "probeViableThreshold": 0.99,    // probability [0,1] — min EWMA probe success rate to promote
    "probeFreshnessMs": 500.0,       // ms >= 0 — max probe age to accept for a promote decision
    "promoteDebounceWindows": 3,     // positive int — consecutive clean probe windows before promote
    "videoDemotePer": 0.05,          // probability [0,1] — residual-loss threshold for a loss demote
    "emergencyFecPressure": 0.80,    // probability [0,1] — FEC work rate for emergency demote
    "holdModesDownMs": 2000,         // ms >= 0 — cooldown after a demote before next promote
    "minBetweenChangesMs": 200,      // ms >= 0 — minimum interval between any MCS changes
    "starvationWindows": 5,          // positive int — consecutive starved windows before emergency demote
    "lossWindows": 2                 // positive int — consecutive >=videoDemotePer windows before a loss demote
  },

  // Smoothing: EWMA weights for signal aggregation
  "smoothing": {
    "ewmaAlphaRssi": 0.2,            // alpha (0,1] — RSSI EWMA decay
    "ewmaAlphaFec": 0.2,             // alpha (0,1] — FEC work rate EWMA decay
    "ewmaAlphaBurst": 0.1,           // alpha (0,1] — burst rate EWMA decay
    "starvationThresholdPps": 50.0   // number >= 0 — pps below which link is considered starved
  },

  "flightlog": { "enabled": true }   // bool — write per-tick JSONL flight logs
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `false` | — |
| `maxMcs` | integer | `5` | 0 – 7 |
| `dronePort` | integer | `9999` | 1 – 65535 (drone UDP host comes from `drone.host`) |
| `selector.*` | — | see above | see [Tuning reference](gs-dynamic-link-tuning.md) |
| `smoothing.*` | — | see above | see [Tuning reference](gs-dynamic-link-tuning.md) |
| `flightlog.enabled` | boolean | `true` | — |

All other knobs — learned-prior internals, probe measurement constants, rssi-norm EIRP curve, flightlog storage settings, and the `videoStreamId` constant — are **frozen code constants** not exposed in config. See [`docs/gs-dynamic-link-tuning.md`](gs-dynamic-link-tuning.md) for the full inventory with source file references.

> **The IDR-token relay moved out of `dynamicLink`.** It is now a top-level [`idrForward`](#idrforward--idr-token-relay) block that runs **independently** of the controller. The old `dynamicLink.idrForward` (bool) and `dynamicLink.idrPort` keys are gone.

**Operating model.** Enabling, disabling, or tuning is applied at runtime via `PATCH /gs/config` + `POST /gs/apply` with **no wfb restart** — the GS runner is never bounced for `dynamicLink`-only changes. The controller reads wfb-ng stats on `:8103` (fpvd renders `log_interval = 100` so the feed is 10 Hz). The drone side must be armed **independently** (its own `dynamicLink.enabled`, applied via the GS `/air` proxy — the client orchestrates both halves). `GET /gs/status.dynamicLink` reports the GS controller state only (no drone round-trip); to check the drone's own dynamic-link/adapter state and detect a GS-armed/drone-not mismatch, the client reads `GET /air/status`.

**PATCH validation.** On `PATCH /gs/config`, unknown `dynamicLink` sub-keys are rejected immediately (typo protection). Sub-block value ranges are validated on `POST /gs/apply`. On boot/upgrade load, unknown keys are warned and ignored so a config from an older build never bricks startup.

## `idrForward` — IDR-token relay

A top-level GS config block (a **sibling** of `dynamicLink` and `pixelpilot`) that runs the IDR/keyframe-token relay. It is **independent of `dynamicLink.enabled`**: it forwards PixelPilot keyframe/IDR tokens from `0.0.0.0:<port>` to `<droneHost>:<port>`, where `droneHost` is `drone.host`, bridging PixelPilot keyframe requests to the drone's `idr_listen`.

```jsonc
"idrForward": {
  "enabled": true,   // boolean — run the relay (independent of dynamicLink)
  "port": 11223      // integer, 1..65535 — UDP port (0.0.0.0 listen + drone forward)
}
```

| Field | Type | Default | Valid values |
|-------|------|---------|--------------|
| `enabled` | boolean | `true` | — |
| `port` | integer | `11223` | 1 – 65535 |

**Runtime.** Toggle `idrForward.enabled` at runtime via `PATCH /gs/config` + `POST /gs/apply` (no wfb bounce). A `port` change takes effect on **daemon restart**.

> **Migration:** the old `dynamicLink.idrForward` (bool) and `dynamicLink.idrPort` (int) keys are removed; their behavior now lives in this top-level block and no longer depends on the controller being armed.

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
| `link.txPowerDbm` | TX power via `iw set txpower` |
| `link.fec` | FEC shards `k` and `n` via `CMD_SET_FEC` (entire subtree) |
| `link.width` | Channel width via `CMD_SET_RADIO` |
| `video.bitrate` | Encoder bitrate via encoder HTTP API |
| `video.qpDelta` | Encoder QP delta via encoder HTTP API |
| `video.roi` | Encoder ROI settings via encoder HTTP API (entire subtree) |

Note: `link.channel` is **not** locked — dl-applier never changes frequency. `video.resilience` is **not** locked either — it is an operator-owned encoder preset the controller never writes, so it stays editable while `dynamicLink.enabled`.

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
# Stage: configure safe floor and enable adaptive link
curl -X PATCH http://127.0.0.1:8080/config \
  -H 'content-type: application/json' \
  -d '{
    "dynamicLink": {
      "enabled": true,
      "safe": {
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

The ground-station `fpvd` (Python, `gs/fpvdgs/`) owns the **GS** wfb radio config and supervises the GS wfb data plane, replacing the stock `wfb-server`/`S98wifibroadcast`. A supervisor process owns the config + this HTTP API; a runner child imports the `wfb_ng` library to run `wfb_rx`/`wfb_tx` and continues to serve the wfb stats APIs on `:8103` (JSON) / `:8003` (MsgPack), so `dynamic-link-gs` and `wfb-cli` are unaffected.

Binds `0.0.0.0:8080` (same posture as the drone). Source of truth: `/etc/fpvd/config.json`; the daemon renders these to `/etc/wifibroadcast.cfg` (a generated artifact — do not edit). The stage→apply lifecycle (effective vs pending, `?pending=true`) is identical to the drone's.

### Differences from the drone API at a glance

| | Drone | Ground station |
|--|--|--|
| Config schema | link + video + image + telemetry + recording + dynamicLink + services | `link` + `wfb` + `drone` + `dynamicLink` + `idrForward` + `pixelpilot` |
| `link` fields | channel, width, txPowerDbm, **mcs, fec, stbc, ldpc**, mtu, … | channel, width, txPowerDbm, region, linkId, beamforming, wlans — **no mcs/fec/stbc/ldpc** (drone-owned for video; GS uplink uses wfb-ng defaults) |
| Config tree route | `/config`, `/apply`, … (root) | `/gs/config`, `/gs/apply`, … (`/gs/*`) |
| `GET /healthz` body | `{}` | `{"ok": true}` (both at root) |
| Error body | `{error, message, details}` | `{"error": "<message>"}` |
| Mutating link params | `PATCH /config` | `PATCH /gs/config` (`link` is a normal mutable block) |
| Extra endpoints | — | `/air/*` (opaque drone proxy) |

## Shared endpoints (GS behavior)

`GET /healthz` → `200 {"ok": true}` (root). `GET /gs/defaults` → the GS baseline. `GET /gs/config[?pending=true]` → effective/pending GS config. `POST /gs/reset` → `{"reset": true}` (drops the overlay, re-renders, bounces the runner).

### PATCH /gs/config

Deep-merges a partial GS config into pending. `link` is a **normal mutable block** — a `PATCH /gs/config {"link": {...}}` is accepted (it is no longer rejected). Unknown top-level keys are rejected.

```bash
curl -X PATCH http://127.0.0.1:8080/gs/config -H 'content-type: application/json' \
  -d '{"wfb":{"mavlink":{"peer":"connect://127.0.0.1:14550"}}}'

# link is just another block:
curl -X PATCH http://127.0.0.1:8080/gs/config -H 'content-type: application/json' \
  -d '{"link":{"channel":100,"width":20}}'
```

| Code | Meaning |
|------|---------|
| 200 | Patch accepted; body is the updated pending config. |
| 400 | `{"error":"unknown config keys: [...]"}`, or a link validation error (e.g. enabling beamforming on a GS card without a `bf_monitor_conf` node). |

### POST /gs/apply

Validates pending, renders the cfg, and applies. A link change applies with a **live `iw` retune** when possible (channel / width / `txPowerDbm` / region with no 40 MHz-class crossing); otherwise it **bounces the runner** (a brief RX drop). On failure it restores the last-good cfg and does not commit. The GS applies **only its own side** — it never pushes to the drone.

```jsonc
{"applied": true}
```

| Code | Meaning |
|------|---------|
| 200 | `{"applied": true}` — committed (live retune or runner bounce). |
| 500 | `{"applied": false, "error":"runner failed; rolled back to last-good cfg"}`. |

### GET /gs/status

```jsonc
{
  "fpvd":   {"version": "0.1.0", "uptimeMs": 41717},
  "runner": {"running": true, "pid": 10447, "restarts": 0,
             "autoRestarts": 0, "lastExit": null, "fault": false},
  "radio":  [
    {"wlan": "wlx84fc146c36e6", "type": "monitor", "channel": 132,
     "freqMhz": 5660, "widthMhz": 40, "txpowerDbm": 19.0}
  ],
  "link":   {"linkId": 7669206},
  "beamforming": {"enabled": false, "localMac": "84:fc:14:6c:36:e6"}
}
```

- **`/gs/status` is GS-local** — it makes no drone round-trip, so it stays fast even when the drone is slow/unreachable. For drone reachability and the drone's own dynamic-link/adapter state, the client calls **`GET /air/status`**.
- `runner` — the supervised wfb runner: `restarts` counts operator bounces, `autoRestarts` counts crash auto-restarts, `fault` is the crash-loop guard.
- `radio` — one entry per wlan, parsed from `iw dev <wlan> info`.
- `beamforming.localMac` — the GS card MAC, read by the client during the beamforming handshake (see [Client orchestration](#client-orchestration-of-cross-device-link-changes)).

## Client orchestration of cross-device link changes

There is **no `/link` coordinator** — it was removed. `link` is a normal mutable block on both ends: on the GS via `PATCH /gs/config`, and on the drone via the `/air/*` proxy (`PATCH /air/config`). The GS applies only its own side and never pushes to the drone, so the **client** drives any change that must land on both ends.

### Shared link change (channel / width / linkId / region)

```bash
curl -X PATCH http://127.0.0.1:8080/air/config -H 'content-type: application/json' \
  -d '{"link":{"channel":100,"width":20}}'
curl -X PATCH http://127.0.0.1:8080/gs/config  -H 'content-type: application/json' \
  -d '{"link":{"channel":100,"width":20}}'
curl -X POST  http://127.0.0.1:8080/air/apply        # drone first on a channel/width move
curl -X POST  http://127.0.0.1:8080/gs/apply         # then GS retunes onto the moved link
```

On a channel/width move, apply the **drone first** so the GS retunes onto the link the drone has already moved to. The GS applies its change with a live `iw` retune when possible (channel / width / `txPowerDbm` / region with no 40 MHz-class crossing), else a runner bounce.

### Beamforming enable (client-owned MAC handshake)

```bash
# 1. read the GS card MAC
GS_MAC=$(curl -s http://127.0.0.1:8080/gs/status | jq -r .beamforming.localMac)

# 2. drone: point BF at the GS card MAC, and disable STBC
#    (STBC and TX-beamforming are mutually exclusive on the drone)
curl -X PATCH http://127.0.0.1:8080/air/config -H 'content-type: application/json' \
  -d "{\"link\":{\"beamforming\":{\"enabled\":true,\"remoteMac\":\"$GS_MAC\"},\"stbc\":false}}"
curl -X POST  http://127.0.0.1:8080/air/apply

# 3. GS beamformee self-reconciles to link.beamforming.enabled
#    (it reads the drone MAC read-only)
curl -X PATCH http://127.0.0.1:8080/gs/config -H 'content-type: application/json' \
  -d '{"link":{"beamforming":{"enabled":true}}}'
curl -X POST  http://127.0.0.1:8080/gs/apply
```

**Disable:** set `beamforming.enabled:false` on both sides and restore the drone's `link.stbc = true`. Enabling beamforming on a GS card without a `bf_monitor_conf` node is rejected by `/gs/config` validation.

## Drone proxy — `/air/*`

`GET/PATCH /air/config`, `POST /air/apply`, `GET /air/status` forward the request **opaquely** (no schema parsing) to the drone fpvd at `drone.host`:`drone.apiPort`, relaying its response verbatim. This is the single front door for the drone's own config — both drone-only config (video bitrate, codec, ROI, …) and the drone's `link` block during a [client-orchestrated](#client-orchestration-of-cross-device-link-changes) shared-link or beamforming change. The GS daemon never models the drone schema.

| Code | Meaning |
|------|---------|
| 2xx/4xx/5xx | Relayed from the drone fpvd. |
| 502 | `{"error":"drone unreachable: ..."}` — could not reach the drone (`drone.host`:`drone.apiPort`). |

```bash
curl http://127.0.0.1:8080/air/status                       # drone's /status, proxied
curl -X PATCH http://127.0.0.1:8080/air/config \            # drone-only config
  -H 'content-type: application/json' -d '{"video":{"bitrate":9000}}'
```

## GS config schema

```jsonc
{
  "link": {
    "channel": 132,          // integer — Wi-Fi channel (must be valid for region)
    "width": 40,             // integer, 20 or 40 — card width (HT20/HT40); must match the drone's video TX width to receive
    "txPowerDbm": null,      // integer dBm, or null — TX power; null keeps the driver default.
                             //   Converted x100 to mBm at the radio edge (wfb-ng wifi_txpower / iw txpower fixed).
                             //   Useful range -10..30; the GS does not range-check it (the drone enforces -10..30).
    "region": "US",          // string — CRDA country code
    "linkId": 7669206,       // integer — informational; the actual id is derived from wfb-ng link_domain (matches the drone)
    "beamforming": {"enabled": false},  // GS beamformee; self-reconciles to enabled (reads the drone MAC read-only)
    "wlans": "auto"          // "auto" -> wfb-nics autodetect, or an explicit list ["wlx..", ...]
  },
  "wfb": {
    "profile": "gs",         // string — wfb-ng service profile
    "mavlink": {"peer": "connect://127.0.0.1:14550"},
    "raw": {}                // passthrough: {section: {key: value}} merged verbatim into the rendered cfg
  },
  "idrForward": {            // top-level IDR-token relay (sibling of dynamicLink/pixelpilot) — see idrForward section
    "enabled": true,
    "port": 11223
  },
  "drone": {
    "host": "10.5.0.10",     // the drone's address on the wfb tunnel; reused by the /air proxy,
    "apiPort": 8080          //   the IDR relay, and the dynamic-link decision UDP
  }
}
```

| Field | Type | Default | Notes |
|-------|------|---------|-------|
| `link.channel` | integer | `132` | required |
| `link.width` | integer | `40` | `20` or `40` |
| `link.txPowerDbm` | integer \| null | `null` | dBm; `null` = driver default. Useful range -10..30 (not GS-range-checked; the drone enforces -10..30). Converted ×100 to mBm at the radio edge. |
| `link.region` | string | `"US"` | required |
| `link.linkId` | integer | `7669206` | informational (link_domain-derived) |
| `link.beamforming.enabled` | boolean | `false` | GS beamformee; rejected if the card has no `bf_monitor_conf` node |
| `link.wlans` | `"auto"` \| array | `"auto"` | `wfb-nics` autodetect or explicit list |
| `wfb.profile` | string | `"gs"` | — |
| `wfb.mavlink.peer` | string | `connect://127.0.0.1:14550` | — |
| `wfb.raw` | object | `{}` | passthrough escape hatch |
| `idrForward.enabled` | boolean | `true` | run the relay — see [`idrForward`](#idrforward--idr-token-relay) |
| `idrForward.port` | integer | `11223` | UDP relay port |
| `drone.host` | string | `10.5.0.10` | the drone's address; reused by the /air proxy, IDR relay, and DL decision UDP |
| `drone.apiPort` | integer | `8080` | the drone fpvd HTTP API port |

There is intentionally **no** `mcs`/`fec`/`ldpc`/`stbc` on the GS: video downlink FEC/modulation is auto-detected by `wfb_rx` (drone-owned), and the GS uplink TX uses wfb-ng's defaults. There are also no `video`/`image`/`telemetry`/`recording`/`services` sections — those are drone-only. (`dynamicLink`, `idrForward`, and `pixelpilot` are GS-side sections — see their dedicated sections.)

> **Migration:** `link.txpower` was renamed `link.txPowerDbm` (now dBm, -10..30, not the old driver-unit mBm value). The same rename applies on the drone.

## PixelPilot managed service

fpvd-GS spawns and supervises the `pixelpilot` display binary (PixelPilot FPV Decoder for Rockchip ≥1.3) as a first-class managed child alongside the wfb data plane. The `pixelpilot` config block models all GS-local launch knobs and flows through the standard `PATCH /gs/config` + `POST /gs/apply` lifecycle with **granular apply**: a `pixelpilot.*` change restarts PixelPilot only (the wfb runner and radio are untouched), and conversely a `link.*` or `wfb.*` change leaves PixelPilot running. Flag order in the rendered argv is canonical (stable); the getopt-style parser accepts any order.

### Config block

```json
"pixelpilot": {
  "enabled": true,
  "bin": "/usr/bin/pixelpilot",
  "env": {},
  "configPath": "/etc/pixelpilot.yaml",
  "osdConfigPath": "/etc/pixelpilot/osd.json",
  "screenMode": "1920x1080@60",
  "codec": "h265",
  "rtpPort": 5600,
  "dvr": {
    "dir": "/media/dvr",
    "template": "record_%Y-%m-%d_%H-%M-%S.mp4",
    "fmp4": true,
    "sequencedFiles": true
  },
  "extraArgs": []
}
```

| Key | CLI flag | Default | Notes |
|---|---|---|---|
| `enabled` | — | `true` | Toggle supervision via `PATCH /gs/config` + `POST /gs/apply`. |
| `bin` | — | `/usr/bin/pixelpilot` | Path to the binary. |
| `env` | — | `{}` | Extra child-process environment (e.g. `{"LD_LIBRARY_PATH":"/usr/lib/pixelpilot95"}`). Merged over `os.environ` by the supervisor. |
| `configPath` | `--config` | `/etc/pixelpilot.yaml` | Main pixelpilot config file. |
| `osdConfigPath` | `--osd-config` | `/etc/pixelpilot/osd.json` | OSD layout config. |
| `screenMode` | `--screen-mode` | `1920x1080@60` | HDMI output mode. |
| `codec` | `--codec` | `h265` | Video codec (`h264` or `h265`). |
| `rtpPort` | `-p` | `5600` | RTP video input port. |
| `dvr.dir` | — | `/media/dvr` | DVR output directory (joined with `dvr.template`). |
| `dvr.template` | `--dvr-template` | `record_%Y-%m-%d_%H-%M-%S.mp4` | DVR filename template (joined with `dvr.dir`). |
| `dvr.fmp4` | `--dvr-fmp4` | `true` | Fragmented MP4 output. |
| `dvr.sequencedFiles` | `--dvr-sequenced-files` | `true` | Sequenced output files. |
| `extraArgs` | — | `[]` | Verbatim-appended flags (escape hatch for un-modeled options). |

```bash
# Change display mode and apply (restarts PixelPilot only):
curl -X PATCH http://10.18.0.1:8080/gs/config \
  -H 'content-type: application/json' \
  -d '{"pixelpilot":{"screenMode":"1280x720@60"}}'
curl -X POST http://10.18.0.1:8080/gs/apply

# Set LD_LIBRARY_PATH for a perf-build lib dir:
curl -X PATCH http://10.18.0.1:8080/gs/config \
  -H 'content-type: application/json' \
  -d '{"pixelpilot":{"env":{"LD_LIBRARY_PATH":"/usr/lib/pixelpilot95"}}}'
curl -X POST http://10.18.0.1:8080/gs/apply
```

### Status

`GET /gs/status` includes a `pixelpilot` block alongside `runner`, `radio`, and `link`:

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

Error responses are a single field — `{"error": "<human message>"}` — with HTTP `400` (validation/schema), `404` (no route), `500` (apply/runner failure), or `502` (drone unreachable on `/air`).
