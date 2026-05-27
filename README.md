# fpvd

A single supervisor daemon for OpenIPC FPV stacks. Owns one unified
config (waybeam encoder + wfb-ng radio + msposd/mavfwd telemetry +
arbitrary user services) and exposes it over HTTP+JSON.

See `docs/superpowers/specs/2026-05-26-fpvd-design.md` for the full
design.

## Build (host, for tests)

    cmake -S . -B build -DCMAKE_BUILD_TYPE=Debug
    cmake --build build -j
    ./build/fpvd_tests

## Build (target, armhf — via Buildroot)

See the `package/fpvd/` recipe in the openipc-builder repo.

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

## Adaptive link (`dl-applier`)

Set `dynamicLink.enabled = true` to have fpvd supervise the drone-side
of `wfbng-dynamic-link` (`/usr/bin/dl-applier`). Configuration is
driven entirely by fpvd — no `/etc/dynamic-link/drone.conf` is read.

When enabled, these fields become read-only via the API
(`PATCH /config` returns `400 dynamic_link_locked`) because
`dl-applier` mutates them at runtime: `link.mcs`, `link.txpower`,
`link.fec`, `link.width`, `video.bitrate`, `video.qpDelta`,
`video.roi`. To edit a baseline, disable `dynamicLink.enabled`,
PATCH the field, then re-enable.

Per-airframe failsafe ceilings live under `dynamicLink.safe` and the
ROI-QP curve under `dynamicLink.roiQp`. See the design spec at
`docs/superpowers/specs/2026-05-27-dynamic-link-design.md` for the
full list and the lock-rule semantics.
