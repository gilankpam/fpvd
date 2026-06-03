# fpvd (ground station)

Python peer to the drone `fpvd`. Owns the GS wfb radio config, supervises
the wfb data plane (built on the `wfb_ng` library, replacing `wfb-server`),
and spawns/supervises the `pixelpilot` display process.
Single front-door HTTP API on `:8080`.

See `../docs/superpowers/specs/2026-06-02-fpvd-gs-design.md`.

## Test (dev host)

    cd gs && python -m pytest -q

## Deploy (GS)

    ./deploy/gs/deploy.sh --host 10.18.0.1
    # rollback:
    ./deploy/gs/rollback.sh --host 10.18.0.1

## API

| Method | Path | Behavior |
|---|---|---|
| GET | /config[?pending=true] | effective or pending GS config |
| PATCH | /config | merge sparse JSON into pending (link.* rejected) |
| POST | /apply | commit pending → effective; render cfg; bounce runner |
| POST | /reset | drop overlay |
| GET | /defaults | baseline |
| GET | /status | daemon + runner/radio/link state |
| GET | /healthz | 200 |
| GET/PATCH | /air/config, POST /air/apply, GET /air/status | opaque proxy to drone fpvd |
| GET | /link | overlap params + droneReachable |
| PATCH/POST | /link, /link/apply | GS-local-first link change; applyTo "gs"|"both" |

## Config reference

### `dynamicLink`

Arms the in-process GS adaptive-link control loop. Disabled by default.

```json
"dynamicLink": {
  "enabled": false,
  "maxMcs": 5,
  "bandwidth": 20,
  "txpower": { "min": 18, "max": 28 },
  "radioProfile": "m8812eu2",
  "droneAddr": null,
  "dronePort": 9999,
  "idrForward": true,
  "idrPort": 11223,
  "tuning": {}
}
```

| Key | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `false` | Arms the in-process control loop. Toggle at runtime via `PATCH /config` + `POST /apply`. |
| `maxMcs` | int 0–7 | `5` | Upper MCS bound the controller may select. |
| `bandwidth` | 20 or 40 | `20` | RF bandwidth the controller targets (MHz). |
| `txpower.min` / `.max` | int (dBm) | 18 / 28 | Tx-power range the controller may request. |
| `radioProfile` | string | `"m8812eu2"` | Packaged radio profile (JSON under `gs/fpvdgs/dynlink/profiles/`). Determines per-MCS bitrate tables. |
| `droneAddr` | string\|null | `null` | Drone's dynamic-link UDP address. Defaults to the host parsed from `drone.endpoint`. |
| `dronePort` | int | `9999` | Drone's dynamic-link UDP port. |
| `videoStreamId` | string | `"video"` | Substring matched against the wfb stats record `id` to select the **video** rx stream. The policy is driven by this stream only; the mavlink/tunnel rx records on `:8103` are ignored (their low packet rate would trip `link_starved` and pin MCS at the floor). |
| `idrForward` | bool | `true` | Run the IDR-token relay (`127.0.0.1:idrPort` → `droneAddr:idrPort`) while the controller is active. Bridges PixelPilot keyframe requests to the drone's `idr_listen`; replaces the old standalone `socat` idr-forwarder. |
| `idrPort` | int | `11223` | UDP port for the IDR relay (local listen + drone forward). |
| `tuning` | object | `{}` | Opaque passthrough of advanced policy knobs (gate/fec/smoothing/cooldown). Merged over code defaults. |

**Operating model.** The controller is an in-process daemon thread that
consumes wfb-ng stats on `:8103` at 10 Hz (fpvd renders `log_interval = 100`
unconditionally to guarantee this), runs the adaptive policy, and emits
decisions to the drone over UDP.

Enabling, disabling, or tuning is applied at runtime via `PATCH /config` +
`POST /apply` with **no wfb restart** — the runner is never bounced for
`dynamicLink`-only changes.

The drone side must be armed separately (its own `dynamicLink.enabled`,
reachable via fpvd's `/air` proxy).

**Deploy cutover.** `deploy/gs/deploy.sh` retires the standalone
`dynamic-link-gs` service (init `S99dynamic-link-gs`, which also ran a bundled
`socat` idr-forwarder): it stops the service and moves the init script to
`/root/fpvd-gs-rollback/` so fpvd owns the GS dynamic-link role — binding the
HELLO listener on UDP `5801` and the IDR relay on `127.0.0.1:11223`.
`deploy/gs/rollback.sh` restores it for a full revert to the pre-fpvd state.

**Status.** `GET /status.dynamicLink` shows the controller state plus a
`drone` sub-object with `reachable`, `dynamicLinkActive`, and `hello` — so a
GS-armed / drone-not mismatch is immediately visible:

```json
{
  "dynamicLink": {
    "enabled": true,
    "running": true,
    "drone": {
      "reachable": true,
      "dynamicLinkActive": false,
      "hello": "announcing"
    }
  }
}
```

When `enabled` is false the block is `{"enabled": false, "running": false}`.
`hello` is one of `"announcing"`, `"keepalive"`, `"none"`.

### `pixelpilot`

fpvd-GS spawns and supervises the `pixelpilot` binary as a managed child and
builds its argv from this block (reproducing the stock `ExecStart` at defaults).
Changes apply by restarting PixelPilot only — the radio link is untouched.

```json
"pixelpilot": {
  "enabled": true,
  "bin": "/usr/bin/pixelpilot",
  "configPath": "/etc/pixelpilot/pixelpilot.yaml",
  "screenMode": "1920x1080@60",
  "videoScale": 1.0,
  "osdConfigPath": "/etc/pixelpilot/config_osd.json",
  "dvrFramerate": 60,
  "dvrDir": "/var/dvr",
  "dvrTemplate": "record_%Y-%m-%d_%H-%M-%S.mp4",
  "extraArgs": []
}
```

| Key | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Arms PixelPilot supervision; toggle via `PATCH /config` + `POST /apply`. |
| `screenMode` | string | `1920x1080@60` | `--screen-mode` (HDMI output mode). |
| `videoScale` | number | `1.0` | `--video-scale`. |
| `osdConfigPath` | string | `/etc/pixelpilot/config_osd.json` | `--osd-config`. |
| `dvrFramerate` | int | `60` | `--dvr-framerate`. |
| `bin`/`configPath`/`dvrDir`/`dvrTemplate` | string | (see above) | Structural paths (rarely changed). |
| `extraArgs` | list[str] | `[]` | Verbatim-appended flags (escape hatch for un-modeled options). |

`GET /status.pixelpilot` shows `{enabled, running, pid, restarts, autoRestarts,
lastExit, fault}`; `{enabled:false, running:false}` when disabled. Changes to
`pixelpilot.*` restart only PixelPilot; a link/wfb change leaves it running.

## On-device smoke (run after deploy; needs the drone reachable for /air and /link "both")

1. `pidof fpvd wfb_rx wfb_tx` — all present; no `wfb-server`/`S98wifibroadcast`.
2. `curl -s :8080/status` — runner.running true; radio shows channel/width per wlan.
3. `(echo>/dev/tcp/127.0.0.1/8103)` — open; dynamic-link-gs still connected; video flowing.
4. GS-local: `curl -XPATCH :8080/config -d '{"wfb":{"mavlink":{"peer":"connect://127.0.0.1:14550"}}}'` then `curl -XPOST :8080/apply` — 200; only the runner bounced.
5. Link bootstrap (drone reachable): `curl -XPATCH :8080/link -d '{"link":{"channel":100}}'` then `curl -XPOST :8080/link/apply -d '{"applyTo":"both"}'` — `{gsApplied:true,droneApplied:true}`; link re-establishes on the new channel.
6. Link bootstrap (drone offline / different channel): same with `{"applyTo":"gs"}` — `droneApplied:false`, GS moves to the drone's channel and the link comes up.
7. `/air`: `curl :8080/air/status` round-trips the drone fpvd's status.
8. PixelPilot: `pidof pixelpilot` present; `curl -s :8080/status` shows `pixelpilot.running:true`. `curl -XPATCH :8080/config -d '{"pixelpilot":{"videoScale":1.5}}'` then `curl -XPOST :8080/apply` — 200; only PixelPilot restarts (wfb_rx/wfb_tx PIDs unchanged).
