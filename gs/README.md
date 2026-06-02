# fpvd (ground station)

Python peer to the drone `fpvd`. Owns the GS wfb radio config and supervises
the wfb data plane (built on the `wfb_ng` library, replacing `wfb-server`).
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

## On-device smoke (run after deploy; needs the drone reachable for /air and /link "both")

1. `pidof fpvd wfb_rx wfb_tx` — all present; no `wfb-server`/`S98wifibroadcast`.
2. `curl -s :8080/status` — runner.running true; radio shows channel/width per wlan.
3. `(echo>/dev/tcp/127.0.0.1/8103)` — open; dynamic-link-gs still connected; video flowing.
4. GS-local: `curl -XPATCH :8080/config -d '{"wfb":{"mavlink":{"peer":"connect://127.0.0.1:14550"}}}'` then `curl -XPOST :8080/apply` — 200; only the runner bounced.
5. Link bootstrap (drone reachable): `curl -XPATCH :8080/link -d '{"link":{"channel":100}}'` then `curl -XPOST :8080/link/apply -d '{"applyTo":"both"}'` — `{gsApplied:true,droneApplied:true}`; link re-establishes on the new channel.
6. Link bootstrap (drone offline / different channel): same with `{"applyTo":"gs"}` — `droneApplied:false`, GS moves to the drone's channel and the link comes up.
7. `/air`: `curl :8080/air/status` round-trips the drone fpvd's status.
