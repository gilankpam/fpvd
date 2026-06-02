# fpvd

A single supervisor daemon for OpenIPC FPV stacks. Owns one unified
config (waybeam encoder + wfb-ng radio + msposd/mavfwd telemetry +
arbitrary user services) and exposes it over HTTP+JSON.

See `docs/superpowers/specs/2026-05-26-fpvd-design.md` for the full
design.

## Build (host, for tests)

    cmake -S drone -B drone/build -DCMAKE_BUILD_TYPE=Debug
    cmake --build drone/build -j
    ( cd drone && ./build/fpvd_tests )

## Build (target, ssc338q — cross-compile)

    cmake -S drone -B drone/build/ssc338q -DCMAKE_TOOLCHAIN_FILE=drone/cmake/toolchain-ssc338q.cmake
    cmake --build drone/build/ssc338q --target fpvd -j

Requires `armv7l-unknown-linux-musleabihf-g++` on PATH (available in
the project `nix-shell`). Produces a statically-linked ARMv7 EABI5
executable.

See also the `package/fpvd/` recipe in the openipc-builder repo for
Buildroot integration.

## Run (locally for smoke testing)

    ./build/fpvd \
      --defaults etc/defaults.json \
      --overlay /tmp/fpvd-overlay.json \
      --radio-up /bin/true \
      --waybeam-json /tmp/fpvd-waybeam.json \
      --port 8080

Then:

    curl http://127.0.0.1:8080/config
    curl -X PATCH http://127.0.0.1:8080/config \
      -H 'content-type: application/json' \
      -d '{"video":{"bitrate":10000}}'
    curl -X POST http://127.0.0.1:8080/apply
    curl http://127.0.0.1:8080/status

## API

| Method | Path | Behavior |
|---|---|---|
| GET | /config[?pending=true] | Read effective or pending config |
| PATCH | /config | Deep-merge sparse JSON into pending |
| POST | /apply | Commit pending → effective; restart affected subsystems |
| POST | /reset | Drop user overlay |
| GET | /defaults | Read baseline |
| GET | /status | Daemon + process state |
| GET | /healthz | 200 OK |

## Adaptive link (in-process controller)

Set `dynamicLink.enabled = true` to run the adaptive-link control loop
**in-process inside fpvd** — there is no separate `dl-applier` binary
and no `/etc/dynamic-link/drone.conf` file. Configuration is driven
entirely by the fpvd config.

**Locked fields.** While enabled, these fields are read-only via the
API (`PATCH /config` returns `400 dynamic_link_locked`) because the
controller mutates them at runtime: `link.mcs`, `link.txpower`,
`link.fec`, `link.width`, `video.bitrate`, `video.qpDelta`,
`video.roi`. To edit a baseline, disable `dynamicLink.enabled`, PATCH
the field, then re-enable.

**Hot-reloadable knobs (no bounce).** All `dynamicLink.*` knobs
(`dynamicLink.safe`, `dynamicLink.roiQp`, timeouts, stagger/pacing,
OSD toggle, etc.) and `link.mtu` apply live via `POST /apply` —
re-snapshotting the in-process controller **without bouncing wfb or
waybeam** (no video blackout). Toggling `dynamicLink.enabled` via
`/apply` starts or stops the in-process loop without restarting the
rest of the stack.

**Baseline video changes (brief bounce).** `video.fps` and other
`video.*` baseline changes still trigger a full encoder rebuild (a
brief wfb/waybeam bounce, as before). When `dynamicLink.enabled` is
true the controller is stopped and restarted around the bounce
(restart-around) and re-announces HELLO with the updated parameters.
`video.fps` is **not** in the no-bounce list.

**`/status` block.** `GET /status` includes a `dynamicLink` key:

```json
{
  "dynamicLink": {
    "enabled": true,
    "running": true,
    "watchdogTripped": false,
    "lastDecisionAgeMs": 120,
    "hello": "keepalive"
  }
}
```

When `enabled` is false the block is `{"enabled":false,"running":false}`.
`hello` is one of `"announcing"`, `"keepalive"`, or `"disabled"`.

Watchdog visibility is surfaced on the video OSD; there is no MAVLink
status channel.

Per-airframe failsafe ceilings live under `dynamicLink.safe` and the
ROI-QP curve under `dynamicLink.roiQp`. See the design spec at
`docs/superpowers/specs/2026-05-27-dynamic-link-design.md` for the
full list and the lock-rule semantics.

### On-device smoke (manual)

Run these against a real drone to confirm §14 success criteria. All
`pidof wfb_video_tx waybeam` checks verify no bounce happened.

1. **Boot with `dynamicLink.enabled:false`** — `GET /status` →
   `dynamicLink:{enabled:false,running:false}`. No dl-applier process,
   no `/etc/dynamic-link/drone.conf` read.

2. **Enable at runtime** — `PATCH {"dynamicLink":{"enabled":true}}` +
   `POST /apply` → `dynamicLink.running:true`, GS sees HELLO,
   decisions apply; wfb/waybeam pids unchanged (no bounce).

3. **Hot-reload a locked-field baseline** — `PATCH
   {"dynamicLink":{"safe":{"mcs":3}}}` + `POST /apply` → applied live;
   all wfb/waybeam pids unchanged.

4. **Hot-reload `link.mtu`** — `PATCH {"link":{"mtu":1400}}` + `POST
   /apply` → HELLO re-announces `mtu=1400`; no bounce.

5. **Disable at runtime** — `PATCH {"dynamicLink":{"enabled":false}}` +
   `POST /apply` → `running:false`; pids unchanged.

6. **Stack bounce while enabled** — `PATCH {"video":{"codec":"h264"}}`
   + `POST /apply` while enabled → wfb/waybeam bounce (expected) +
   controller restart-around; GS receives re-HELLO.
