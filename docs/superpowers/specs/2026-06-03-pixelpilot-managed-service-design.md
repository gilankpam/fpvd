# PixelPilot as an fpvd-GS managed service + config API

- **Date:** 2026-06-03
- **Status:** Approved (design)
- **Component:** `gs/fpvdgs` (fpvd ground-station daemon)
- **Related:** PixelPilot_rk `docs/superpowers/specs/2026-06-03-pixelpilot-settings-via-fpvd-gs-design.md`
  (the PixelPilot-side gsmenu integration that routes settings through this daemon)

## Problem

PixelPilot runs on the ground station alongside fpvd-GS. fpvd-GS already owns the GS
wfb radio config and supervises the wfb data plane (spawning a child process via
`RunnerSupervisor`), and is the single HTTP front door on `:8080`. PixelPilot, however,
is still launched by its own init script and configured out-of-band (a systemd-style
`ExecStart` parameterized by `/etc/default/pixelpilot`). The PixelPilot gsmenu's earlier
settings work explicitly **dropped** GS-local rows (HDMI/screen mode, etc.) because
"fpvd-GS does not model them."

Two gaps follow:

1. **No process ownership.** Nothing supervises PixelPilot the way fpvd-GS supervises
   wfb — no unified start/stop/restart, crash recovery, or status.
2. **No config front door.** PixelPilot's GS-local launch settings are not modeled by
   fpvd-GS, so the gsmenu has nowhere to send them.

## Goal

fpvd-GS becomes the single owner of PixelPilot on the GS:

1. **Managed service** — fpvd-GS spawns the `pixelpilot` binary directly as a second
   managed child (start / stop / restart / crash-loop guard / status), mirroring the wfb
   runner.
2. **Config API** — fpvd-GS models PixelPilot's GS-local **launch knobs**, renders them
   into PixelPilot's argv, and applies changes by bouncing the PixelPilot child. The
   settings flow through the existing `/config` + `/apply`.

The PixelPilot-side gsmenu wiring (`settings_fpvd.c`) is **out of scope** — this builds
the fpvd-GS backend those menus call.

## Non-goals

- **Settings without a CLI flag.** Color correction (`color_correction` / `cc_gain` /
  `cc_offset`) is applied live in-process by PixelPilot's own OSD menu via a GL shader
  and has no command-line flag. Owning it would require adding flags to PixelPilot
  (PixelPilot-side integration) — deferred.
- **The richer DVR/OSD menu rows** (dvr mode, max-size, re-encode codec/res/fps/bitrate,
  osd-burn) — out this round; the modeled surface is the four launch knobs below.
  `extraArgs` is the escape hatch until they are modeled first-class.
- **GS Wi-Fi hotspot / restream** — need separate daemons; not modeled.
- **Live apply.** PixelPilot config changes apply by restarting the PixelPilot child
  ("applies on restart"); no live pipeline rebuild.
- **Owning PixelPilot's provisioned files.** `/etc/pixelpilot/pixelpilot.yaml`,
  `config_osd.json`, and the binary remain device-provisioned; fpvd points at them.

## Decisions (resolved during brainstorming)

1. **Supervision model:** fpvd-GS spawns `pixelpilot` directly as a second managed
   child. Deploy disables the existing init-script launcher so fpvd is sole owner.
2. **Settings scope:** the minimal launch knobs the current `ExecStart` parameterizes —
   `screenMode`, `videoScale`, `dvrFramerate`, `osdConfigPath` — plus an `extraArgs`
   passthrough. Structural fields (binary path, config/osd paths, dvr dir/template)
   carry defaults and are overridable.
3. **Supervisor structure:** generalize `RunnerSupervisor` into a reusable
   `ProcessSupervisor`; wfb and PixelPilot are two instances. (Approach A.)
4. **Config/apply surface:** fold `pixelpilot` into the existing `/config` + `/apply`
   (not a new endpoint); `/apply` bounces **granularly** so each subsystem restarts only
   when its own slice changed.
5. **Current launcher:** an `/etc/init.d/S*pixelpilot*` script; takeover/rollback mirrors
   the `S98wifibroadcast` pattern.

## Architecture

```
                    fpvd-GS supervisor (App)
   ┌──────────────┬───────────────────┬──────────────────┐
   │ wfb runner   │ pixelpilot        │ dynlink          │
   │ supervisor   │ supervisor (NEW)  │ controller       │
   │ ProcessSup   │ ProcessSup        │ in-proc thread   │
   └──────────────┴───────────────────┴──────────────────┘
        argv: --profiles/--wlans   argv: render_pixelpilot_argv(cfg)
        env: WIFIBROADCAST_CFG      env: inherited
        readiness: :8103 open       readiness: liveness settle
```

### ProcessSupervisor (generalized from `RunnerSupervisor`)

`runner_supervisor.py`'s spawn / watcher thread / crash-loop budget / start-stop-restart
machinery is already generic. Extract it into `ProcessSupervisor`, parameterized by:

- **argv** — settable at runtime via `set_argv(argv)` (used by PixelPilot on apply; wfb
  never changes it).
- **env** — extra environment dict (wfb: `WIFIBROADCAST_CFG`; pixelpilot: none).
- **readiness strategy** — one of:
  - *probe*: ready as soon as a predicate is true before the timeout; timeout ⇒ failed
    start (wfb: `_port_open(8103)`).
  - *settle*: ready iff the process is still alive at the end of a short settle window;
    an immediate exit (bad arg, missing binary) ⇒ failed start (pixelpilot).

The crash-loop fault guard, operator-vs-crash restart accounting, and `state()` shape
are unchanged. wfb keeps byte-for-byte current behavior.

### App wiring

`App` holds `wfb` and `pixelpilot` supervisors plus `dynlink`.

- `start()`: `wfb.start()`; if `pixelpilot.enabled`, `pixelpilot.start()`; if
  `dynamicLink.enabled`, `dynlink.start()`. wfb starts before PixelPilot (PixelPilot
  consumes wfb's video stream).
- `shutdown()`: `http.shutdown()`; `dynlink.stop()`; `pixelpilot.shutdown()`;
  `wfb.shutdown()` (reverse order).

## Config block & schema

New top-level `pixelpilot` key, added to `schema.CONFIG_TOP_KEYS` (so it flows through
`/config` + `/apply`; `link` stays excluded). `gs/etc/defaults.json`:

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

| Key | Type | Default | Role |
|---|---|---|---|
| `enabled` | bool | `true` | Arms PixelPilot supervision. Toggle via `/config`+`/apply`. |
| `bin` | string | `/usr/bin/pixelpilot` | Binary path (structural). |
| `configPath` | string | `/etc/pixelpilot/pixelpilot.yaml` | `--config` (structural). |
| `screenMode` | string | `1920x1080@60` | **knob** — `--screen-mode`. |
| `videoScale` | number | `1.0` | **knob** — `--video-scale`. |
| `osdConfigPath` | string | `/etc/pixelpilot/config_osd.json` | **knob** — `--osd-config`. |
| `dvrFramerate` | int | `60` | **knob** — `--dvr-framerate`. |
| `dvrDir` | string | `/var/dvr` | dvr template dir (structural). |
| `dvrTemplate` | string | `record_%Y-%m-%d_%H-%M-%S.mp4` | dvr template (structural). |
| `extraArgs` | list[str] | `[]` | Verbatim-appended args (analogue of `EXTRA_OPTS` / `wfb.raw`). |

The **four operator knobs** the gsmenu PATCHes are `screenMode`, `videoScale`,
`osdConfigPath`, `dvrFramerate`. `extraArgs` lets new flags ship without a schema change.

`schema._validate_pixelpilot` (light): `enabled` bool; `videoScale` a positive number;
`dvrFramerate` a positive int; `screenMode` / path fields non-empty strings; `extraArgs`
a list of strings.

## Render → argv (`gs/fpvdgs/pixelpilot.py`)

`render_pixelpilot_argv(effective) -> list[str]` reproduces the current `ExecStart`
exactly when defaults are used:

```
[bin,
 "--osd", "--osd-custom-message",
 "--osd-config", osdConfigPath,
 "--screen-mode", screenMode,
 "--video-scale", str(videoScale),
 "--dvr-framerate", str(dvrFramerate),
 "--dvr-fmp4", "--dvr-sequenced-files",
 "--dvr-template", f"{dvrDir}/{dvrTemplate}",
 "--config", configPath,
 *extraArgs]
```

The always-on flags (`--osd`, `--osd-custom-message`, `--dvr-fmp4`,
`--dvr-sequenced-files`) are baked in by the renderer.

## API & granular `/apply`

- `GET /config` / `GET /config?pending=true` — now include the `pixelpilot` block.
- `PATCH /config` — accepts `pixelpilot` (validated); `link` still rejected.
- `POST /apply` — `_apply_gs` splits today's "non-`dynamicLink` ⇒ bounce runner" into
  independent domains; each subsystem bounces **only** when its own slice differs between
  `pending` and `effective`:

  | Changed slice | Action |
  |---|---|
  | `wfb` / `drone` (`link` already guarded equal by the `/link/apply` 409 guard) | re-render `wifibroadcast.cfg` + restart **wfb** runner (existing path incl. `.bak` rollback) |
  | `pixelpilot` | `pp.set_argv(render_pixelpilot_argv(pending))`, then route on `enabled`: off→on `start()`, on→on `restart()`, on→off `stop()`. Never touches the wfb cfg/runner. |
  | `dynamicLink` | reconfigure controller in place (unchanged) |

  The PixelPilot routing mirrors `_route_dynamic_link` exactly (`set_argv` is the analogue
  of `dynlink.set_config`). A PixelPilot-only change leaves the radio link untouched; a
  channel change leaves PixelPilot running (it reacquires the stream).

  Commit happens after the bounces, as today. A PixelPilot failing to come up does not
  roll back the config (unlike wfb, it is not link-critical); the failure surfaces via
  `/status.pixelpilot` (`fault` / `lastExit`).

## Status

`GET /status` grows a `pixelpilot` block from `pp.state()` + the enabled flag (mirrors
`dynamicLink`):

```json
"pixelpilot": {
  "enabled": true, "running": true, "pid": 1234,
  "restarts": 0, "autoRestarts": 0, "lastExit": null, "fault": false
}
```

When disabled: `{"enabled": false, "running": false}`.

## Deploy takeover & rollback

`deploy/gs/deploy.sh` — same pattern as `S98wifibroadcast` / `S99dynamic-link-gs`:

- Detect the PixelPilot init script (glob `/etc/init.d/S*pixelpilot*`); stop it; move it
  to `/root/fpvd-gs-rollback/`. Idempotent — never clobber on re-deploy.
- `gs/etc/defaults.json` already carries the `pixelpilot` block (pushed as defaults; the
  operator overlay `/etc/fpvd/config.json` is never clobbered).
- Verify step: `pidof pixelpilot` present, and `curl :8080/status` shows
  `pixelpilot.running: true`.

`deploy/gs/rollback.sh` — restore the init script from the rollback dir and re-enable
boot autostart (fpvd shutdown already stops its PixelPilot child).

`/etc/pixelpilot/{pixelpilot.yaml,config_osd.json}` and the binary stay
device-provisioned; fpvd points at them but does not create or own them.

## Errors & edge cases

| Case | Behavior |
|---|---|
| PixelPilot crash-loop | `fault: true` after the restart budget; wfb/link unaffected; surfaced in `/status`. |
| Bad arg / missing binary | Immediate exit caught by liveness-settle readiness as a failed start; fpvd stays up; `lastExit` surfaced. |
| Invalid `pixelpilot` config | `PATCH`/`apply` returns 400; nothing bounced. |
| wfb restart | Does **not** bounce PixelPilot (brief video reacquire acceptable). |
| PixelPilot restart | Never touches the radio. |
| `enabled: false` | Not spawned; toggling via `/apply` cleanly start/stops it. |
| DVR dir / config files missing | PixelPilot's concern (device-provisioned); fpvd does not pre-create them, matching the old unit. |

## Files

**New**
- `gs/fpvdgs/pixelpilot.py` — `render_pixelpilot_argv()`.
- `gs/tests/unit/test_pixelpilot_render.py`
- `gs/tests/fixtures/fake_pixelpilot.sh` — integration fixture (settles alive / exits per arg).

**Modify**
- `gs/fpvdgs/runner_supervisor.py` — extract `ProcessSupervisor` (argv/env/readiness
  strategy + `set_argv()`); wfb keeps current behavior.
- `gs/fpvdgs/supervisor.py` — construct the PixelPilot supervisor, wire start/shutdown
  order, status.
- `gs/fpvdgs/api.py` — granular `_apply_gs` + PixelPilot routing; `pixelpilot` in status.
- `gs/fpvdgs/schema.py` — `CONFIG_TOP_KEYS` += `pixelpilot`; `_validate_pixelpilot`.
- `gs/etc/defaults.json` — add the `pixelpilot` block.
- `deploy/gs/deploy.sh`, `deploy/gs/rollback.sh` — init-script takeover/restore + verify.
- `gs/README.md`, `docs/api.md` — document the block, status, and smoke steps.

## Tests (TDD, red → green)

- **`test_pixelpilot_render.py`** — argv byte-for-byte matches the current `ExecStart`
  for defaults; each knob reflected; `extraArgs` appended; numeric formatting
  (`videoScale 1.0` → `"1.0"`).
- **`test_schema.py`** — `pixelpilot` validation accept/reject cases.
- **`test_process_supervisor.py`** (extends current `RunnerSupervisor` tests) — generalized
  class: probe vs settle readiness, `set_argv()` swap, crash-loop guard still holds.
- **`test_api.py`** — `PATCH /config` accepts `pixelpilot`; pixelpilot-only `/apply`
  restarts PixelPilot but **not** wfb (asserted via fakes); wfb-only change doesn't touch
  PixelPilot; `enabled` toggle → start/stop; status includes the block.
- **Integration** — `fake_pixelpilot.sh`: e2e start/stop/restart/crash-recovery and that
  `/apply` rebuilds argv.

## Known limitations / out of scope

- Color correction and the richer DVR/OSD rows remain unmodeled (use `extraArgs` until
  modeled first-class, or a follow-up that adds CLI flags PixelPilot-side).
- No live apply — PixelPilot config changes require a child restart.
- Single PixelPilot instance per GS.
