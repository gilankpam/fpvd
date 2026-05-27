# fpvd — wfbng-dynamic-link integration

**Date:** 2026-05-27
**Status:** Draft for review
**Target:** OpenIPC drones on armhf 32-bit, fpvd v1 codebase
**Extends:** `docs/superpowers/specs/2026-05-26-fpvd-design.md`

## 1. Purpose

Bring the drone-side `dl-applier` from the `wfbng-dynamic-link` repo
under `fpvd` supervision as a first-class subsystem, on the same
footing as `wfb_*`, `waybeam`, and `msposd`/`mavfwd`.

`dl-applier` is the on-drone component of an adaptive radio-link
controller: it receives decision packets from the ground station over
the wfb-ng tunnel, validates them against per-airframe ceilings, and
dispatches to `wfb_tx`, `iw`, and the encoder HTTP API. Adding it to
`fpvd` means operators get adaptive link by flipping a single
`dynamicLink.enabled` boolean and applying — no extra init script,
no extra config file, no second source of truth on the drone.

## 2. Problems being solved

1. **Two configuration sources on the drone.** `dl-applier` ships
   its own `/etc/dynamic-link/drone.conf` with knobs that
   already appear in `fpvd`'s schema (link MTU, video FPS) or could be
   derived (wlan device, encoder choice). Today an operator changing
   `link.mtu` via `PATCH /config` would leave `dl-applier`'s
   `hello_mtu_bytes` stale until the conf file is manually edited.
2. **Two init scripts.** `dynamic-link-applier` ships its own systemd
   unit and OpenRC init. The previous fpvd integration work
   consolidated `S95waybeam` + `S98wifibroadcast` into one supervisor;
   leaving `dl-applier` out of that consolidation re-introduces the
   same fragmentation.
3. **No remote toggle.** Turning adaptive link on/off in the field
   currently requires SSH + `rc-update`. `fpvd` already exposes
   `PATCH /config` + `POST /apply`; `dl-applier`'s on/off switch
   should ride that path.

## 3. Scope

### In scope (v1)

- New `dynamicLink` section in the fpvd domain schema.
- Translator from the domain schema to `dl-applier` argv (no conf file).
- First-class supervised subsystem `dl_applier`, gated by
  `dynamicLink.enabled`.
- Diff categorization so `dynamicLink.*` changes restart `dl_applier`
  only; `link.mtu` and `video.fps` changes also restart `dl_applier`
  (because they feed `--hello-mtu-bytes` / `--hello-fps`).
- Cross-field lock: when `dynamicLink.enabled` is true, reject PATCH
  bodies that write to the link/video fields dl-applier owns at runtime.
- Unit tests: schema validation, translator golden-file, diff
  categorization, cross-field lock matrix.

### Out of scope (v1)

- The GS-side `dynamic-link-gs` Python service — fpvd runs only on the
  drone.
- Buildroot packaging of `dl-applier` itself. That stays in a separate
  `package/wfbng-dynamic-link/` recipe; fpvd assumes
  `/usr/bin/dl-applier` exists on the target rootfs.
- Phase-3 debug-suite tunables that `dl-applier` exposes only via the
  conf file (`dbg_log_dir`, `dbg_max_bytes`, `dbg_fsync_each`,
  `gs_tunnel_addr`, `gs_tunnel_port`). These are
  installation-time decisions; if a deployment needs them, they get
  pinned in the translator alongside the other hard-coded values.
- IDR-listener address/port (`idr_listen_addr`, `idr_listen_port`).
  Hard-coded to `0.0.0.0:11223` to match PixelPilot_rk.
- MAVLink sysid/compid, addr, port. Pinned to wfb_tlm_tx's listen
  port and dl-applier's default sysid/compid.
- Hello-handshake cadence (`hello_announce_initial_ms`,
  `hello_announce_steady_ms`, `hello_announce_initial_count`,
  `hello_keepalive_ms`). dl-applier's built-in defaults.

## 4. Architectural decisions

| Decision | Choice | Rationale |
|---|---|---|
| Subsystem class | First-class (not `services.<name>`) | dl-applier has the same cross-cutting coupling as `wfb_*` and `waybeam`: it needs `link.mtu`, `video.fps`, and the radio iface picked by `radio-up.sh`. User services can't reach those without templating, which v1 of fpvd does not have. |
| Configuration interface | CLI args only (no `--config` file) | `dl-applier` already accepts every conf-file field as `--<kebab-case>`. fpvd is the source of truth; writing a second conf file just for `dl-applier` to re-parse adds a sync hazard. Skipping it is also the smallest diff in fpvd. |
| Schema scope | Minimal + a few knobs | Expose airframe-specific safe_* ceilings (per-build), enabled, healthTimeoutMs, interleavingSupported, debug, IDR/apply pacing, OSD/MAVLink toggles, and ROI-QP curve. Everything ports-and-addresses stays in the translator. |
| Gating | `dynamicLink.enabled == false` → process not started | Mirrors `telemetry.router == "none"`. No "stopped because disabled" state in `/status` — the row simply isn't present. |
| Encoder kind | Hard-coded `--encoder-kind waybeam` | fpvd's only encoder is waybeam. `majestic` is a build choice we don't make. |
| MAVLink port | Hard-coded to `14551` (matches wfb_tlm_tx `-u 14551`) | `wfb_tlm_tx` listens on 14551 for upstream telemetry; that's the only port that actually reaches the GS. dl-applier's sample drone.conf points at 14560 because the sample assumes vanilla wfb-ng on a different deployment. |
| wfb_tx control port | Hard-coded to `8000` (matches wfb_video_tx `-C 8000`) | One source of truth. If we ever change the video_tx control port, we change it in `src/translate/wfb.cpp` and `src/translate/dynamic_link.cpp` together. |
| ROI-QP exposure | Expose the four-knob curve | The ROI-QP formula is airframe-tuned — different sensors and lenses want different curve shapes — so it has to be reachable via the API. |

## 5. Schema addition

Added to `src/config/schema.hpp`. Existing `Config` struct gains one
field.

```cpp
struct DynamicLinkSafe {
    int mcs{1};
    int k{8};
    int n{12};
    int depth{1};
    int bandwidth{20};
    int txPowerDbm{20};
    int bitrateKbps{2000};
};

struct DynamicLinkOsd {
    bool enabled{true};
    bool debugLatency{false};
};

struct DynamicLinkRoiQp {
    int thresholdKbps{6000};
    int lowAnchorKbps{2000};
    int floor{-24};
    int step{3};
};

struct DynamicLink {
    bool enabled{false};
    int healthTimeoutMs{10000};
    bool interleavingSupported{true};
    bool debug{false};
    int minIdrIntervalMs{500};
    int applyStaggerMs{50};
    int applySubPaceMs{5};
    bool mavlinkEnable{true};
    DynamicLinkOsd osd{};
    DynamicLinkRoiQp roiQp{};
    DynamicLinkSafe safe{};
};
```

Wire JSON shape:

```jsonc
"dynamicLink": {
  "enabled": false,
  "healthTimeoutMs": 10000,
  "interleavingSupported": true,
  "debug": false,
  "minIdrIntervalMs": 500,
  "applyStaggerMs": 50,
  "applySubPaceMs": 5,
  "mavlinkEnable": true,
  "osd": { "enabled": true, "debugLatency": false },
  "roiQp": { "thresholdKbps": 6000, "lowAnchorKbps": 2000,
             "floor": -24, "step": 3 },
  "safe": { "mcs": 1, "k": 8, "n": 12, "depth": 1,
            "bandwidth": 20, "txPowerDbm": 20, "bitrateKbps": 2000 }
}
```

Added to `etc/defaults.json` verbatim (with `enabled: false` so an
unupgraded ground station doesn't suddenly see dl-applier announce
itself).

### Validation rules (added to `src/config/validate.cpp`)

- `safe.mcs` ∈ [0, 7]
- `safe.k` ≥ 1, `safe.n` ≥ 1, `safe.n` ≤ 32, `safe.k` < `safe.n`
- `safe.depth` ∈ [1, 8]
- `safe.bandwidth` ∈ {20, 40}
- `safe.txPowerDbm` ∈ [0, 30]
- `safe.bitrateKbps` > 0
- `healthTimeoutMs` ≥ 1000 (sub-second watchdog never makes sense
  given decision cadence)
- `minIdrIntervalMs` ≥ 16 (one frame at 60 fps)
- `applyStaggerMs` ∈ [0, 1000]
- `applySubPaceMs` ∈ [0, 50] (matches dl-applier's own range check)
- `roiQp.thresholdKbps` > `roiQp.lowAnchorKbps` > 0
- `roiQp.floor` ≤ 0 (negative-sharpens-center convention)
- `roiQp.step` ≥ 1

Unknown keys inside `dynamicLink.*` rejected with 400 per §6 of the
parent spec (typo protection).

### Cross-field lock: dl-applier owns these at runtime

When `dynamicLink.enabled` is `true`, `dl-applier` mutates the
following fields at runtime via `wfb_tx` control commands, `iw`, and
the encoder HTTP API. Letting an operator also write to them through
`PATCH /config` creates two writers to the same physical value, with
dl-applier's next decision (typically <100 ms away) silently
overwriting whatever the operator just set.

To prevent the confusion, `PATCH /config` rejects any body that writes
to these keys when the *merged pending* config would have
`dynamicLink.enabled == true`:

| Locked path | Owner inside dl-applier |
|---|---|
| `link.mcs` | `CMD_SET_RADIO` (mcs field) |
| `link.txpower` | `iw wlan0 set txpower fixed <mBm>` |
| `link.fec` (the entire subtree, `k` and `n`) | `CMD_SET_FEC` |
| `link.width` | `CMD_SET_RADIO` (bandwidth field) |
| `video.bitrate` | encoder HTTP API |
| `video.qpDelta` | encoder HTTP API |
| `video.roi` (the entire subtree) | encoder HTTP API (notably `fpv.roiQp` is recomputed on every bitrate apply) |

`link.channel` is **not** locked — dl-applier never changes frequency,
only width.

**Evaluation rule.** The check runs against the *pending* config after
the incoming PATCH body has been deep-merged in, not against the
current effective config:

- `PATCH {dynamicLink:{enabled:false}, link:{mcs:5}}` while DL is
  enabled in effective → **allowed**. Merged pending has
  `enabled=false`, so the lock is open in the same operation.
  Operator can disable DL and edit baseline atomically.
- `PATCH {dynamicLink:{enabled:true}, link:{mcs:5}}` while DL is
  disabled in effective → **rejected**. Merged pending has
  `enabled=true` *and* the body writes a locked key. Operator must
  send two PATCHes: baseline first, then enable.
- `PATCH {link:{mcs:5}}` while DL is enabled in pending → **rejected**.

**Implementation.** The check happens in
`src/config/validate.cpp::validatePatch(body, mergedPending)` and runs
*before* the schema range checks. The body-key walk is path-based
(`link.fec` is locked → any `link.fec.k` or `link.fec.n` in the body
fails, and a body that overwrites `link.fec` wholesale also fails).
Toggling `enabled` itself is always allowed.

**Error shape.** 400 with `{ error: "dynamic_link_locked", details: {
locked: ["link.mcs", "video.bitrate"] } }`. The `locked` array lists
the specific dotted paths the rejected body tried to write, so the
client can surface a precise message.

**Apply path.** `POST /apply` does not re-run this check — pending was
already validated at each PATCH. The only way to reach a pending state
with `enabled=true` and a "modified" locked field is to PATCH the
locked field first while DL was off, then PATCH `enabled=true` in a
separate body; that's the intended flow ("set my baseline, then turn
DL on").

## 6. Translator

New module: `src/translate/dynamic_link.{hpp,cpp}`.

Header (mirroring `translate/wfb.hpp`):

```cpp
#pragma once
#include "config/schema.hpp"
#include <string>
#include <vector>

namespace fpvd {

// Build the argv (including argv[0] = binary path) for dl-applier.
// `iface` is the wlan device picked by radio-up.sh.
std::vector<std::string> dynamicLinkArgs(const Config& c,
                                          const std::string& iface);

} // namespace fpvd
```

The translator emits one flat argv:

| Source | Flags |
|---|---|
| `dynamicLink.enabled` | (drives orchestrator gating, not emitted) |
| `dynamicLink.healthTimeoutMs` | `--health-timeout-ms <N>` |
| `dynamicLink.interleavingSupported` | `--interleaving-supported <0\|1>` |
| `dynamicLink.debug` | `--debug-enable <0\|1>` |
| `dynamicLink.minIdrIntervalMs` | `--min-idr-interval-ms <N>` |
| `dynamicLink.applyStaggerMs` | `--apply-stagger-ms <N>` |
| `dynamicLink.applySubPaceMs` | `--apply-sub-pace-ms <N>` |
| `dynamicLink.mavlinkEnable` | `--mavlink-enable <0\|1>` |
| `dynamicLink.osd.enabled` | `--osd-enable <0\|1>` |
| `dynamicLink.osd.debugLatency` | `--osd-debug-latency <0\|1>` |
| `dynamicLink.roiQp.thresholdKbps` | `--roi-qp-threshold-kbps <N>` |
| `dynamicLink.roiQp.lowAnchorKbps` | `--roi-qp-low-anchor-kbps <N>` |
| `dynamicLink.roiQp.floor` | `--roi-qp-floor <N>` |
| `dynamicLink.roiQp.step` | `--roi-qp-step <N>` |
| `dynamicLink.safe.mcs` | `--safe-mcs <N>` |
| `dynamicLink.safe.k` | `--safe-k <N>` |
| `dynamicLink.safe.n` | `--safe-n <N>` |
| `dynamicLink.safe.depth` | `--safe-depth <N>` |
| `dynamicLink.safe.bandwidth` | `--safe-bandwidth <N>` |
| `dynamicLink.safe.txPowerDbm` | `--safe-tx-power-d-bm <N>` |
| `dynamicLink.safe.bitrateKbps` | `--safe-bitrate-kbps <N>` |
| `link.mtu` | `--hello-mtu-bytes <N>` |
| `video.fps` | `--hello-fps <N>` |
| `iface` (from radio detection) | `--wlan-dev <iface>` |
| (constant) | `--listen-addr 0.0.0.0 --listen-port 5800` |
| (constant) | `--wfb-tx-ctrl-addr 127.0.0.1 --wfb-tx-ctrl-port 8000` |
| (constant) | `--encoder-kind waybeam --encoder-host 127.0.0.1 --encoder-port 80` |
| (constant) | `--idr-listen-addr 0.0.0.0 --idr-listen-port 11223` |
| (constant) | `--mavlink-addr 127.0.0.1 --mavlink-port 14551` |
| (constant) | `--osd-msg-path /tmp/MSPOSD.msg --osd-update-interval-ms 1000` |

Hello-cadence, sysid/compid, debug-suite log-dir/max-bytes/fsync, and
gs-tunnel address/port are intentionally not emitted — `dl-applier`'s
built-in defaults are correct for this deployment.

Boolean fields use the explicit `<0|1>` form rather than the
zero-arg `--mavlink-enable` switch, because dl-applier's `--mavlink-enable`
sets the field to true only — there is no `--no-mavlink-enable`
counterpart for forcing a default-true field off. Setting `0`
explicitly via `--mavlink-enable 0` requires that dl-applier accept
that form; this is a small assumption documented as a build-time check
(see §10).

## 7. Process supervision

The radio table in §10 of the parent spec gains one row:

| Subsystem | Process name | argv | startAfter |
|---|---|---|---|
| Adaptive link | `dl_applier` | from `dynamicLinkArgs(cfg, iface)` | `wfb_video_tx`, `wfb_tun`, `waybeam`, telemetry router |

- `restartPolicy: always`
- Standard exponential backoff (1→2→4→8→16→30s) and failure cap (5
  crashes / 60s → `failed`), same as other first-class children.
- When `dynamicLink.enabled == false`, the orchestrator does not spawn
  `dl_applier` at all. `GET /status.processes[]` does not include it.
- When toggled from `true` → `false` via `PATCH /config` + `POST /apply`,
  the running `dl_applier` is stopped (SIGTERM, 5s grace, SIGKILL).
- When toggled from `false` → `true`, `dl_applier` starts after the
  startAfter list resolves.

### Why startAfter includes telemetry

`dl-applier` writes MAVLink STATUSTEXT frames to `127.0.0.1:14551`,
which is `wfb_tlm_tx`'s listen socket. If `dl_applier` starts before
`wfb_tlm_tx`, the first few `sendto()` calls go nowhere but no error
surfaces (UDP). That's tolerable — the next STATUSTEXT works — but
the ordering hint costs nothing.

`dl-applier` also writes encoder commands to `127.0.0.1:80`, so
`waybeam` is a real prerequisite, not just a hint: requests issued
before waybeam binds the port would return ECONNREFUSED until the
first retry.

## 8. Apply-flow diff categorization

`src/config/diff.cpp` gains entries:

| Changed path | Restarts |
|---|---|
| `dynamicLink.enabled` (any transition) | `dl_applier` start or stop |
| `dynamicLink.*` (any other field) | `dl_applier` |
| `link.mtu` | radio (existing) + `dl_applier` (re-emits `--hello-mtu-bytes`) |
| `video.fps` | `waybeam` (existing) + `dl_applier` (re-emits `--hello-fps`) |
| Radio iface re-detected after `radio-up.sh` | `dl_applier` rides the radio restart (already triggered by `link.*`) |

All other existing diff rules are untouched.

## 9. Status surface

`GET /status.processes[]` gains one entry when
`dynamicLink.enabled == true`:

```jsonc
{ "name": "dl_applier", "pid": 471, "state": "running",
  "restarts": 0, "lastExitCode": null, "uptime": 3410 }
```

No new top-level `status.dynamicLink` block in v1 — the process row is
the user-visible signal. A future iteration can surface dl-applier's
own MAVLink rejection/watchdog/apply-fail counters if the operator
asks for them; today they're already on the GS's `mavlink_status`
panel.

## 10. Testing

### Unit tests (new)

- `tests/unit/test_schema.cpp` — round-trip the new `dynamicLink`
  section through `from_json` / `to_json`; verify defaults match
  §5; verify unknown keys inside `dynamicLink.*` reject.
- `tests/unit/test_validate.cpp` — one positive + one negative case
  per validation rule in §5, plus the cross-field lock matrix:
  - DL enabled in effective, PATCH writes a locked key → 400
    `dynamic_link_locked` listing the path.
  - DL enabled in effective, PATCH disables it *and* writes a locked
    key in the same body → 200.
  - DL disabled in effective, PATCH enables it *and* writes a locked
    key in the same body → 400.
  - DL enabled, PATCH writes `dynamicLink.safe.mcs` (not a locked
    key) → 200.
  - DL enabled, PATCH writes `link.channel` (deliberately
    non-locked) → 200.
  - DL enabled, PATCH overwrites `link.fec` wholesale → 400 (subtree
    lock).
- `tests/unit/test_translate_dynamic_link.cpp` (new file, mirrors
  `test_translate_wfb.cpp`) — golden-file argv tests:
  1. Defaults config → exact expected argv.
  2. `safe.mcs = 3`, `safe.bitrateKbps = 8000` → corresponding
     `--safe-mcs 3`, `--safe-bitrate-kbps 8000`.
  3. `link.mtu = 1400`, `video.fps = 90` → `--hello-mtu-bytes 1400`,
     `--hello-fps 90`.
  4. Non-default iface → `--wlan-dev <name>`.
- `tests/unit/test_diff.cpp` — verify each of the five rows in §8
  produces the documented restart set.

### Integration tests (existing harness)

- HTTP round-trip: `PATCH /config` with a `dynamicLink.enabled = true`
  body → `POST /apply` → fake `/usr/bin/dl-applier` (test fixture, a
  shell script that records its argv) is spawned exactly once.
  Flip back to `false` → fake is reaped, `/status` no longer lists
  `dl_applier`.

### Build-time check

A small `tests/unit/test_dl_applier_cli_assumptions.cpp` (compiled
host-only) asserts the documented CLI surface against a vendored copy
of `dl-applier --help` output. If `wfbng-dynamic-link` removes or
renames a flag we depend on (`--mavlink-enable <0|1>` is the touchy
one — see §6), this test fails at build time rather than at runtime on
a drone. The vendored help text refresh procedure mirrors
`drone/src/vendored/README.md` in `wfbng-dynamic-link`.

## 11. Buildroot integration

No change to `package/fpvd/fpvd.mk`. `dl-applier` is built and
installed by a separate `package/wfbng-dynamic-link/` recipe (out of
scope for this spec). `fpvd` assumes:

- `/usr/bin/dl-applier` exists on the target rootfs.
- The deployed `dl-applier` accepts the CLI surface documented in §6.

If `dl-applier` is missing from the rootfs and `dynamicLink.enabled ==
true`, the supervisor logs the exec failure and the row appears as
`state: failed` in `/status` per the existing §9 of the parent spec.
The daemon stays up.

The `wfbng-dynamic-link` package recipe must stop installing
`packaging/dynamic-link-applier.service` and
`packaging/dynamic-link-applier.init` on systems that ship fpvd — the
same kind of amendment the parent spec makes for `S95waybeam` and
`S98wifibroadcast`.

## 12. Migration

For drones currently running `dl-applier` under
`dynamic-link-applier.service` or its OpenRC equivalent:

- The firmware update that ships fpvd-with-dynamic-link must
  `rc-update del dynamic-link-applier default` (and remove the systemd
  unit on systems that use it) before first boot of the new image, or
  fpvd and the standalone init script will race to bind
  `0.0.0.0:5800` and one will fail.
- Existing `/etc/dynamic-link/drone.conf` files are ignored. fpvd
  passes argv only; the conf file's `wfb_tx_ctrl_port = 8000`-style
  customizations either match fpvd's hard-coded values or did so
  already-by-coincidence. Operator-visible knobs (safe_*, debug,
  interleaving_supported, etc.) re-express through `PATCH /config`.

## 13. Trade-off: dl-applier × fpvd config races

Resolved by the cross-field lock in §5. While `dynamicLink.enabled ==
true`, `PATCH /config` rejects any write to the runtime-owned
fields (`link.mcs`/`txpower`/`fec`/`width`,
`video.bitrate`/`qpDelta`/`roi`), so the only writer to those fields
at runtime is dl-applier. fpvd still writes the schema-defined
*baseline* into `/etc/waybeam.json` and `wfb_tx` argv at process
start, but no PATCH can change those baselines without first disabling
dl-applier — which means no apply-cycle race between an operator edit
and an in-flight dl-applier decision.

What remains: each `POST /apply` that restarts `waybeam` for a
non-locked `video.*` change (e.g. `video.codec`, `video.resolution`,
`video.fps`, `video.gopSize`, `recording.*`, `snapshot.*`) still
resets the encoder's runtime ROI-QP and bitrate to the
freshly-translated baseline until dl-applier's next decision arrives
(typically <100 ms at the configured stats cadence). That's a brief
visual blip during a deliberate operator-initiated apply, not a
silent overwrite. Acceptable under v1's pre-flight workflow hard
assumption.

## 14. Open questions

None blocking. Items intentionally deferred to v2+:

- Surfacing dl-applier's MAVLink rejection / watchdog / apply-fail
  counters in `GET /status.dynamicLink`.
- Exposing the deferred Phase-3 debug-suite knobs (`dbg_log_dir`,
  `dbg_max_bytes`, `dbg_fsync_each`) once a deployment asks for them.
- Extending the §5 cross-field lock pattern to other future
  runtime-controllers (e.g. a hypothetical auto-channel-select
  service that would own `link.channel`).

## 15. Success criteria

The integration is complete when:

1. A drone firmware build that ships both fpvd and `dl-applier` boots
   with `dynamicLink.enabled = false` and `GET /status.processes[]`
   does **not** include `dl_applier`.
2. `PATCH /config -d '{"dynamicLink":{"enabled":true}}'` + `POST /apply`
   results in `dl_applier` appearing in `/status` as `running` within
   2 seconds.
3. `PATCH /config -d '{"link":{"mtu":1400}}'` + `POST /apply`
   restarts `dl_applier` and the new process has `--hello-mtu-bytes
   1400` in its argv (verifiable via `cat /proc/$(pidof dl-applier)/cmdline`).
4. `PATCH /config -d '{"dynamicLink":{"safe":{"mcs":3}}}'` + `POST /apply`
   restarts only `dl_applier` (radio, encoder, telemetry processes
   keep their PIDs).
5. Killing `dl_applier` manually (`kill -9 $(pidof dl-applier)`) with
   `dynamicLink.enabled = true` results in automatic restart visible
   in `/status` within 2 seconds.
6. Flipping `dynamicLink.enabled` from `true` → `false` + `POST /apply`
   stops the process; the `/status` row disappears.
