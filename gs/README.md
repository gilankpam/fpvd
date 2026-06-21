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

GS-local routes live under `/gs/*`; `/air/*` is an opaque proxy to the
drone fpvd; `/healthz` stays at the root.

| Method | Path | Behavior |
|---|---|---|
| GET | /gs/config[?pending=true] | effective or pending GS config |
| PATCH | /gs/config | merge sparse JSON into pending (`link` is a normal mutable block — accepted) |
| POST | /gs/apply | commit pending → effective; render cfg; apply (live `iw` retune or runner bounce) |
| POST | /gs/reset | drop overlay |
| GET | /gs/defaults | baseline |
| GET | /gs/status | daemon + runner/radio/link/beamforming state |
| GET | /healthz | 200 (daemon liveness, root) |
| GET/PATCH | /air/config, POST /air/apply, GET /air/status | opaque proxy to drone fpvd (unchanged) |

### Client orchestration

The GS no longer coordinates cross-device link changes — the **client** does.
`link` is a normal mutable block on both ends (`/gs/config` and, via the proxy,
`/air/config`); the GS applies only its own side and never pushes to the drone.

**Shared link change** (channel / width / linkId / region):

    curl -XPATCH :8080/air/config -d '{"link":{"channel":100,"width":20}}'
    curl -XPATCH :8080/gs/config  -d '{"link":{"channel":100,"width":20}}'
    curl -XPOST  :8080/air/apply        # drone first on a channel/width move
    curl -XPOST  :8080/gs/apply         # then GS retunes onto the link the drone moved to

On a channel/width move apply the **drone first**, so the GS retunes onto the
link the drone has already moved to. The GS applies its link change with a live
`iw` retune when possible (channel / width / `txPowerDbm` / region with no
40 MHz-class crossing), else a runner bounce.

**Beamforming enable** (client-owned MAC handshake):

    # 1. read the GS card MAC
    curl -s :8080/gs/status | jq -r .beamforming.localMac        # -> <GS card MAC>
    # 2. drone: point BF at the GS card MAC, disable STBC (mutually exclusive with TX-BF)
    curl -XPATCH :8080/air/config \
      -d '{"link":{"beamforming":{"enabled":true,"remoteMac":"<GS card MAC>"},"stbc":false}}'
    curl -XPOST  :8080/air/apply
    # 3. GS beamformee self-reconciles to link.beamforming.enabled (reads the drone MAC read-only)
    curl -XPATCH :8080/gs/config -d '{"link":{"beamforming":{"enabled":true}}}'
    curl -XPOST  :8080/gs/apply

**Disable:** set `beamforming.enabled:false` on both sides and restore the
drone's `link.stbc = true`. Enabling BF on a GS card without a `bf_monitor_conf`
node is rejected by `/gs/config` validation.

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
  "tuning": {}
}
```

| Key | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `false` | Arms the in-process control loop. Toggle at runtime via `PATCH /gs/config` + `POST /gs/apply`. |
| `maxMcs` | int 0–7 | `5` | Upper MCS bound the controller may select. |
| `bandwidth` | 20 or 40 | `20` | RF bandwidth the controller targets (MHz). |
| `txpower.min` / `.max` | int (dBm) | 18 / 28 | Controller's tx-power request range, in dBm (distinct from the static `link.txPowerDbm` key). |
| `radioProfile` | string | `"m8812eu2"` | Packaged radio profile (JSON under `gs/fpvdgs/dynlink/profiles/`). Determines per-MCS bitrate tables. |
| `droneAddr` | string\|null | `null` | Drone's dynamic-link UDP address. Defaults to the host parsed from `drone.endpoint`. |
| `dronePort` | int | `9999` | Drone's dynamic-link UDP port. |
| `videoStreamId` | string | `"video"` | Substring matched against the wfb stats record `id` to select the **video** rx stream. The policy is driven by this stream only; the mavlink/tunnel rx records on `:8103` are ignored (their low packet rate would trip `link_starved` and pin MCS at the floor). |
| `tuning` | object | `{}` | Opaque passthrough of advanced policy knobs (gate/fec/smoothing/cooldown). Merged over code defaults. |

> The IDR-token relay is no longer part of `dynamicLink`. It is now a
> top-level [`idrForward`](#idrforward) block that runs independently of the
> controller — see below.

**Operating model.** The controller is an in-process daemon thread that
consumes wfb-ng stats on `:8103` at 10 Hz (fpvd renders `log_interval = 100`
unconditionally to guarantee this), runs the adaptive policy, and emits
decisions to the drone over UDP.

Enabling, disabling, or tuning is applied at runtime via `PATCH /gs/config` +
`POST /gs/apply` with **no wfb restart** — the runner is never bounced for
`dynamicLink`-only changes.

The drone side must be armed separately (its own `dynamicLink.enabled`,
reachable via fpvd's `/air` proxy).

**Deploy cutover.** `deploy/gs/deploy.sh` retires the standalone
`dynamic-link-gs` service (init `S99dynamic-link-gs`, which also ran a bundled
`socat` idr-forwarder): it stops the service and moves the init script to
`/root/fpvd-gs-rollback/` so fpvd owns the GS dynamic-link role — binding the
HELLO listener on UDP `5801` and (via the [`idrForward`](#idrforward) block)
the IDR relay. `deploy/gs/rollback.sh` restores it for a full revert to the
pre-fpvd state.

**Status.** `GET /gs/status.dynamicLink` shows the controller state plus a
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

### `idrForward`

Top-level block (a sibling of `dynamicLink` and `pixelpilot`) that runs the
IDR/keyframe-token relay. It is **independent of `dynamicLink.enabled`** — the
relay forwards PixelPilot keyframe/IDR tokens from `0.0.0.0:<port>` to
`<droneHost>:<port>`, where `droneHost` is derived from `drone.endpoint`,
bridging PixelPilot keyframe requests to the drone's `idr_listen`.

```json
"idrForward": {
  "enabled": true,
  "port": 11223
}
```

| Key | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Run the relay. Toggle at runtime via `PATCH /gs/config` + `POST /gs/apply` (no wfb bounce). |
| `port` | int | `11223` | UDP port (local `0.0.0.0` listen + drone forward). A `port` change takes effect on daemon restart. |

### `pixelpilot`

fpvd-GS spawns and supervises the `pixelpilot` binary (PixelPilot FPV Decoder
for Rockchip ≥1.3) as a managed child. Flag order in the rendered argv is
canonical (stable); the getopt-style parser accepts any order. Changes apply by
restarting PixelPilot only — the radio link is untouched.

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

| Key | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Arms PixelPilot supervision; toggle via `PATCH /gs/config` + `POST /gs/apply`. |
| `bin` | string | `/usr/bin/pixelpilot` | Path to the pixelpilot binary. |
| `env` | object | `{}` | Extra child-process environment variables (e.g. `LD_LIBRARY_PATH`). Merged over `os.environ` by the supervisor. |
| `configPath` | string | `/etc/pixelpilot.yaml` | `--config` (pixelpilot main config file). |
| `osdConfigPath` | string | `/etc/pixelpilot/osd.json` | `--osd-config`. |
| `screenMode` | string | `1920x1080@60` | `--screen-mode` (HDMI output mode). |
| `codec` | string | `h265` | `--codec` (video codec: `h264` or `h265`). |
| `rtpPort` | int | `5600` | `-p` (RTP video input port). |
| `dvr.dir` | string | `/media/dvr` | DVR output directory (joined with `dvr.template`). |
| `dvr.template` | string | `record_%Y-%m-%d_%H-%M-%S.mp4` | `--dvr-template` filename (joined with `dvr.dir`). |
| `dvr.fmp4` | bool | `true` | `--dvr-fmp4` flag (fragmented MP4). |
| `dvr.sequencedFiles` | bool | `true` | `--dvr-sequenced-files` flag. |
| `extraArgs` | list[str] | `[]` | Verbatim-appended flags (escape hatch for un-modeled options). |

`GET /gs/status.pixelpilot` shows `{enabled, running, pid, restarts, autoRestarts,
lastExit, fault}`; `{enabled:false, running:false}` when disabled. Changes to
`pixelpilot.*` restart only PixelPilot; a link/wfb change leaves it running.

## On-device smoke (run after deploy; needs the drone reachable for /air and the shared-link step)

1. `pidof fpvd wfb_rx wfb_tx` — all present; no `wfb-server`/`S98wifibroadcast`.
2. `curl -s :8080/gs/status` — runner.running true; radio shows channel/width per wlan.
3. `(echo>/dev/tcp/127.0.0.1/8103)` — open; dynamic-link-gs still connected; video flowing.
4. GS-local: `curl -XPATCH :8080/gs/config -d '{"wfb":{"mavlink":{"peer":"connect://127.0.0.1:14550"}}}'` then `curl -XPOST :8080/gs/apply` — 200; only the runner bounced.
5. Shared link change (client-orchestrated; drone reachable): `curl -XPATCH :8080/air/config -d '{"link":{"channel":100}}'` and `curl -XPATCH :8080/gs/config -d '{"link":{"channel":100}}'`, then `curl -XPOST :8080/air/apply` **then** `curl -XPOST :8080/gs/apply` — drone moves first, the GS retunes onto the new channel and the link re-establishes.
6. GS-only retune (tune the GS to a channel the drone is already on): `curl -XPATCH :8080/gs/config -d '{"link":{"channel":100}}'` then `curl -XPOST :8080/gs/apply` — the GS moves to the drone's channel and the link comes up (drone untouched).
7. `/air`: `curl :8080/air/status` round-trips the drone fpvd's status.
8. PixelPilot: `pidof pixelpilot` present; `curl -s :8080/gs/status` shows `pixelpilot.running:true`. `curl -XPATCH :8080/gs/config -d '{"pixelpilot":{"screenMode":"1280x720@60"}}'` then `curl -XPOST :8080/gs/apply` — 200; only PixelPilot restarts (wfb_rx/wfb_tx PIDs unchanged).
