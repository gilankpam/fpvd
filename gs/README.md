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

Arms the in-process GS adaptive-link control loop. Disabled by default. The
controller knobs live under a nested `controller` block; `enabled` is the
top-level arm toggle.

```json
"dynamicLink": {
  "enabled": false,
  "controller": {
    "maxMcs": 5,
    "radioProfile": "m8812eu2",
    "droneAddr": null,
    "dronePort": 9999,
    "tuning": {}
  }
}
```

| Key | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `false` | Arms the in-process control loop. Toggle at runtime via `PATCH /config` + `POST /apply`. |
| `controller.maxMcs` | int 0–7 | `5` | Upper MCS bound the controller may select. |
| `controller.radioProfile` | string | `"m8812eu2"` | Packaged radio profile (JSON under `gs/fpvdgs/dynlink/profiles/`). Determines per-MCS bitrate tables. |
| `controller.droneAddr` | string\|null | `null` | Drone's dynamic-link UDP address. Defaults to the host parsed from `droneLink.endpoint`. |
| `controller.dronePort` | int | `9999` | Drone's dynamic-link UDP port. |
| `controller.tuning` | object | `{}` | Opaque passthrough of advanced policy knobs (gate/fec/smoothing/cooldown). Merged over code defaults. |

The RF bandwidth the controller targets is **derived from `link.width`** — there is no separate `bandwidth` field. The video rx stream is selected by an internal constant (`videoStreamId = "video"`), not a config field; the policy is driven by that stream only (the mavlink/tunnel rx records on `:8103` are ignored, since their low packet rate would trip `link_starved` and pin MCS at the floor). Per-MCS tx power is owned by the drone (its tx-power curve), so the controller has no `txpower` field. The IDR-token relay is no longer controller config — see **IDR/keyframe relay** below.

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
HELLO listener on UDP `5801` and the always-on IDR relay on `0.0.0.0:11223`.
`deploy/gs/rollback.sh` restores it for a full revert to the pre-fpvd state.

**IDR/keyframe relay.** A standalone, **always-on** relay (independent of
`dynamicLink.enabled`, so it serves static *and* adaptive links) listens on
`0.0.0.0:11223` and forwards PixelPilot keyframe-request tokens to the drone's
`idr_listen` at `droneLink.endpoint` host:`11223`. It replaces the old
standalone `socat` idr-forwarder. There is no config for it — the port is
fixed; it is reported under `GET /status.idrRelay`.

**Status.** `GET /status.dynamicLink` shows the controller state plus a
`drone` sub-object with `reachable`, `dynamicLinkActive`, and `hello` — so a
GS-armed / drone-not mismatch is immediately visible; `GET /status.idrRelay`
reports the always-on relay:

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
  },
  "idrRelay": { "running": true, "listen": "0.0.0.0:11223" }
}
```

When `enabled` is false the block is `{"enabled": false, "running": false}`.
`hello` is one of `"announcing"`, `"keepalive"`, `"none"`. `idrRelay.listen`
is `null` if the relay failed to bind (e.g. the port was already taken).

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

| Key | Type | Default | Description |
|---|---|---|---|
| `enabled` | bool | `true` | Arms PixelPilot supervision; toggle via `PATCH /config` + `POST /apply`. |
| `bin` | string | `/usr/bin/pixelpilot` | Path to the pixelpilot binary. |
| `env` | object | `{}` | Extra child-process environment variables (e.g. `LD_LIBRARY_PATH`). Merged over `os.environ` by the supervisor. |
| `configPath` | string | `/etc/pixelpilot.yaml` | `--config` (pixelpilot main config file). |
| `osdConfigPath` | string | `/etc/pixelpilot/osd.json` | `--osd-config`. |
| `screenMode` | string | `1920x1080@60` | `--screen-mode` (HDMI output mode). |
| `videoScale` | number | `1.0` | `--video-scale`. |
| `codec` | string | `h265` | `--codec` (video codec: `h264` or `h265`). |
| `rtpPort` | int | `5600` | `-p` (RTP video input port). |
| `rtpJitterMs` | int | `1` | `--rtp-jitter-ms`. |
| `dvr.framerate` | int | `60` | `--dvr-framerate`. |
| `dvr.dir` | string | `/media/dvr` | DVR output directory (joined with `dvr.template`). |
| `dvr.template` | string | `record_%Y-%m-%d_%H-%M-%S.mp4` | `--dvr-template` filename (joined with `dvr.dir`). |
| `dvr.fmp4` | bool | `true` | `--dvr-fmp4` flag (fragmented MP4). |
| `dvr.sequencedFiles` | bool | `true` | `--dvr-sequenced-files` flag. |
| `dvr.osd` | bool | `false` | `--dvr-osd` flag (burn OSD into DVR). |
| `dvr.mode` | string | `raw` | `--dvr-mode` (`raw` or `reencode`). |
| `dvr.maxSizeMb` | int | `4000` | `--dvr-max-size` (MB). |
| `dvr.reencCodec` | string | `h264` | `--dvr-reenc-codec`. |
| `dvr.reencBitrate` | int | `8000` | `--dvr-reenc-bitrate` (kbps). |
| `dvr.reencFps` | int | `30` | `--dvr-reenc-fps`. |
| `dvr.reencResolution` | string | `1080p` | `--dvr-reenc-resolution`. |
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
