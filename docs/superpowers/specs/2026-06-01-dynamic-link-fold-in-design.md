# fpvd — Dynamic-Link Fold-In (drone side)

**Date:** 2026-06-01
**Status:** Draft for review
**Target:** OpenIPC drones on armhf 32-bit, fpvd codebase
**Extends:** `2026-05-26-fpvd-design.md`, `2026-05-27-dynamic-link-design.md`,
`2026-05-30-link-hot-apply-design.md`
**Supersedes:** the child-process integration in `2026-05-27-dynamic-link-design.md`

## 1. Purpose

Fold the drone-side adaptive-link controller (today the standalone
`dl-applier` binary in the `wfbng-dynamic-link` repo) **into the fpvd
process** as a first-class in-process module, and make every
adaptive-link config change apply **at runtime with no restart**.

Two outcomes:

1. **One process, one config source.** The adaptive-link control loop
   becomes a C++ module inside `fpvd`. `/etc/dynamic-link/drone.conf`
   is removed; the loop reads fpvd's `Config` only. The duplicated
   `wfb_tx` control client, `iw` path, and encoder client collapse
   onto fpvd's existing ones.
2. **Hot config reload.** Changing any `dynamicLink.*` knob — or
   toggling `dynamicLink.enabled` — applies to the running loop
   without bouncing `wfb_tx`/`waybeam` and without restarting fpvd.

This pass is **drone side only**. The GS-side Python controller is
untouched, and the wire protocol stays byte-identical so the GS needs
no change.

## 2. Background — current state

### 2.1 How it works today

`fpvd` supervises `dl-applier` as a first-class **child process**
(`2026-05-27` spec). On any `dynamicLink.*` change, fpvd re-derives
the child's argv and restarts it.

`dl-applier` itself is a single-threaded `poll(2)` reactor:
decision UDP socket (`:5800`), IDR-token UDP socket (`:11223`), a
watchdog/OSD tick timer, a stagger gap timer, and a HELLO timer. On
each accepted GS decision it dispatches to `wfb_tx` (FEC/radiotap),
`iw` (txpower), and the encoder HTTP API, with direction-aware
staggering and sub-command pacing. A watchdog pushes `safe_defaults`
when the GS goes silent.

### 2.2 Two problems this fold-in fixes

1. **A `dynamicLink.*` change blacks out the whole video stack.**
   In `src/config/diff.cpp`, any `dynamicLink.*` change (plus
   `link.mtu`/`video.fps`) sets `subs.dynamicLink`, which forces
   `needsRebuild` in `Daemon::apply()`. The orchestrator has **no
   selective restart** — `needsRebuild` runs `stopAll(); orch_ = {};
   seedOrchestrator(); startAll()`, bouncing **every** `wfb_*`,
   `waybeam`, telemetry, and `dl_applier` together. So nudging a
   single adaptive-link safe-ceiling causes a multi-second blackout.
   The `2026-05-27` spec's intent of "restart `dl_applier` only" was
   never realized because the mechanism is all-or-nothing.

2. **The CLI contract has already drifted.** fpvd's
   `src/translate/dynamic_link.cpp` builds a long flag list
   (`--listen-addr`, `--safe-mcs`, …), but the current
   `drone/src/dl_applier.c` only accepts `--config <path>` +
   `--debug`. The flag-based integration fpvd targets no longer
   exists, and there is now a second config file (`drone.conf`) that
   duplicates fpvd schema fields (`link.mtu`, `video.fps`) and
   deployment constants.

### 2.3 The precedent we build on

The `2026-05-30` link-hot-apply work already proves config can change
with **zero restart** by talking to `wfb_tx`'s UDP control socket
(`WfbControlClient`) and running `iw`/`ip` via `radio-tune.sh`. fpvd
already owns:

- `WfbControlClient` (`src/translate/wfb_control.*`) — vendored from
  `dl_backend_tx.c`; `CMD_SET_FEC` / `CMD_SET_RADIO`.
- `radio-tune.sh` + `tuneRadio()` — the `iw` txpower/channel/mtu path,
  i.e. the `dl_backend_radio.c` logic.
- The waybeam translator (config → `/etc/waybeam.json`).

This fold-in extends that precedent: the control loop becomes the
in-process holder of those live values, reading a config snapshot
instead of fpvd feeding it fresh argv.

## 3. Decisions

| Decision | Choice | Rationale |
|---|---|---|
| Process boundary | **Fold into the fpvd process** (one binary, one process) | Eliminates the duplicated `wfb_tx`/`iw`/encoder clients and the second config source. |
| Code strategy | **Unify on fpvd's backends** — port the C control logic to C++; drive it through fpvd's existing clients; delete `dl_backend_*` | Single source of truth, zero duplication. |
| Threading | **One dedicated controller thread** running the ported `poll(2)` loop | cpp-httplib's threads block; the soft-real-time loop must stay isolated. A unified single-reactor refactor was rejected as high blast-radius for no added value here. |
| Hot-reload scope | DL knobs apply **live**; `enabled` toggles **start/stop the loop at runtime**; no wfb/waybeam bounce | The "no restart" goal. The encoder/waybeam restart path is left alone (`video.*` still restarts waybeam). |
| Config source | **fpvd `Config` only**; `/etc/dynamic-link/drone.conf` removed | Single authority; drone stops reading `/etc/wfb.yaml`/`majestic.yaml`/`waybeam.json`. |
| MAVLink status channel | **Dropped** | Diagnostic-only, not in the control path. `watchdog` stays visible on the video OSD; `apply_fail` drops to log-only. `/status` counters are a deferred follow-up. |
| Feature scope | **Runtime core only**; debug suite + `dl-inject` deferred | Keep this pass focused on the control loop + hot reload. |
| GS side | **Untouched**; wire stays byte-identical | This pass is drone side only. |

## 4. Architecture

New module directory `src/dynlink/` in fpvd. The drone-side logic
ports C→C++ and runs as one in-process subsystem owned by `Daemon`.

```
                         fpvd process
  ┌──────────────────────────────────────────────────────────────┐
  │  HTTP thread(s)            Daemon (mu_)         DynamicLinkController
  │  cpp-httplib    ──PATCH──▶  effective_/pending_   (its own thread)
  │                            apply():               ┌─ poll(2) loop ─────┐
  │                              diff ───────────────▶│ fds:                │
  │                              publish snapshot ───▶│  • decision UDP:5800│
  │                              start()/stop()  ────▶│  • IDR UDP :11223   │
  │                                                   │  • tick (wd+OSD)    │
  │  SIGCHLD supervision                              │  • gap (stagger)    │
  │  Orchestrator (wfb/waybeam/                       │  • hello timer      │
  │   telemetry) — UNCHANGED                          │  • eventfd (reload/ │
  │                                                   │     stop)           │
  │                                                   └─────────┬──────────┘
  │                                    shared I/O clients ◀─────┘
  │                          WfbControlClient(:8000) · EncoderClient(:80)
  │                          radio/iw helper · OsdWriter
  └──────────────────────────────────────────────────────────────┘
```

`Daemon` holds one `DynamicLinkController`. When enabled, it runs a
single dedicated thread with the same reactor `dl-applier` has today,
plus one new fd — an **eventfd** for reload/stop. The HTTP threads and
SIGCHLD supervision are untouched; the control loop never blocks on
them, and they never block it.

### 4.1 Module port map (C → C++)

| Today (C, `drone/src/`) | Becomes (`src/dynlink/`) | Notes |
|---|---|---|
| `dl_wire.c` | `Wire` | byte-identical frames; golden-vector tested |
| `dl_dedup.c` | `Dedup` | sequence dedup |
| `dl_watchdog.c` | `Watchdog` | silence → `safe_defaults` |
| `dl_hello.c` | `HelloSm` | announce/keepalive state machine |
| `dl_apply` direction + `roi_qp` | `applyDirection()`, `RoiQp` | stagger direction; ROI-QP curve |
| `dl_idr_listen.c` | `IdrListener` | PixelPilot IDR tokens → encoder IDR |
| `dl_osd.c` | `OsdWriter` | `/tmp/MSPOSD.msg` status writes |
| `dl_applier.c` main loop | `DynamicLinkController::run()` | the reactor |

### 4.2 I/O unification (the merge payoff)

| Deleted C backend | Replaced by (fpvd) |
|---|---|
| `dl_backend_tx.c` | existing `WfbControlClient(:8000)` (`CMD_SET_FEC`/`CMD_SET_RADIO`) |
| `dl_backend_radio.c` (iw txpower) | existing `iw` path; a small txpower helper callable without a fork-per-decision (see §7) |
| `dl_backend_enc.c` (encoder HTTP) | **new** `EncoderClient` (waybeam HTTP `:80`) using `httplib::Client` |

`EncoderClient` is genuinely new: fpvd previously only *wrote*
`/etc/waybeam.json`; it had no runtime encoder client.

### 4.3 Deletions

- `src/translate/dynamic_link.{hpp,cpp}` (the argv builder).
- The `dl_applier` child spec in `seedOrchestrator()`.
- `tests/unit/test_translate_dynamic_link.cpp`,
  `tests/unit/test_dl_applier_cli_assumptions.cpp` (obsolete child-CLI
  tests).
- On the drone side (not built for fpvd targets): `dl_backend_*`,
  `dl_config.*`, `dl_yaml_get.*`, `dl_json_get.*`, `dl_mavlink.*`. The
  HELLO config-file readers go away because fpvd is authoritative for
  MTU/FPS.

### 4.4 Decisions reach hardware directly

GS decisions are applied at ~10 Hz via the shared clients. They do
**not** flow through `PATCH`/`apply` and do **not** persist to the
overlay — runtime values stay runtime, exactly as today. fpvd still
writes only the *baseline* into `wfb_tx` argv / `waybeam.json` at
process start.

## 5. Config snapshot & hot-reload mechanism

### 5.1 The snapshot

A plain immutable struct distilled from `Config` — the only state
shared between the HTTP thread and the loop thread:

```cpp
struct DlRuntimeConfig {            // built by Daemon from effective_
    // dynamicLink.* knobs
    int  healthTimeoutMs, minIdrIntervalMs, applyStaggerMs, applySubPaceMs;
    bool interleavingSupported, osdEnabled, osdDebugLatency, debug;
    RoiQpCurve roiQp;               // threshold/lowAnchor/floor/step
    SafeDefaults safe;              // mcs/k/n/depth/bandwidth/txPowerDbm/bitrateKbps
    // derived inputs
    int  helloMtuBytes;             // = link.mtu
    int  helloFps;                  // = video.fps
    std::string iface;              // from RadioInfo (set at start)
};
```

Pinned endpoints (`:5800`, `:8000`, `:80`, `:11223`, OSD path,
GS-tunnel `10.5.0.1:5801`, HELLO cadence) are compile-time constants
in the controller — they never change, so they are not in the
snapshot. They are exposed to tests via a defaulted `Endpoints` struct
(see §10).

### 5.2 Publish → wake → reconcile

```
PATCH …            POST /apply (HTTP thread, holds mu_)
                     effective_ = pending_
                     if dynamicLink-relevant changed && enabled:
                        snap = buildDlSnapshot(effective_, radio_.iface)
                        controller.setConfig(snap)   ──┐
                                                       │ atomic store + write(eventfd)
   DynamicLinkController thread:  poll() ◀─────────────┘
        eventfd readable → drain → load latest snap → reconcile(old,new):
           • re-arm tick timer if osd interval / healthTimeout moved
           • update watchdog timeout
           • re-announce HELLO if interleavingSupported changed (capability bit)
           • osdEnabled off→on / on→off: start/stop OSD writes
           • safe / roiQp / pacing / mtu / fps: stored, read on next use
```

The snapshot is published via an atomic
`shared_ptr<const DlRuntimeConfig>` swap. The eventfd is purely a
*wake* so a change applies immediately (not on the next ≤100 ms tick)
through one `reconcile()` path. Value-only knobs need no structural
action — the loop reads the new snapshot the next time it computes a
decision.

### 5.3 Enable / disable (runtime, no stack bounce)

`apply()` detects the `dynamicLink.enabled` transition:

- **false→true:** `controller.start(snap)` — spawn thread, bind
  `:5800`/`:11223`, arm timers, kick HELLO announce.
- **true→false:** `controller.stop()` — set stop flag, poke eventfd,
  join, close sockets. Leaves `wfb_tx`/encoder at their last runtime
  values (no baseline handback — the "leave the encoder alone"
  decision).

Neither touches the orchestrator → **no wfb/waybeam bounce.**

## 6. Concurrency — single writer by construction

The key safety property falls out of the **existing cross-field lock**
(`src/config/lock.cpp`), not new locking:

- While `enabled`, `checkDynamicLinkLock` already **rejects** operator
  PATCHes to `link.mcs/txpower/fec/width` and
  `video.bitrate/qpDelta/roi`. The operator can never drive the
  `wfb_tx` control socket or the encoder while the loop owns them.
- The only un-locked hot-apply fields under DL are `channel` (→ `iw`,
  NIC only) and `mtu` (→ `ip link`, plus it feeds the loop's HELLO
  snapshot). **Neither touches the `wfb_tx` control socket or the
  encoder.**
- ∴ the loop is the **sole writer** to `wfb_tx :8000` and the encoder
  whenever it runs — no shared-socket mutex, no runtime arbitration.
  This is the same invariant link-hot-apply already relies on.

The loop thread reads **only its own snapshot**; it never takes `mu_`,
so the 10 Hz loop cannot contend with an in-progress `apply()`. The
snapshot (forward) and a small status struct (reverse, §8) are the
entire shared surface between the two threads.

## 7. The `iw` txpower helper

`dl_backend_radio.c` set txpower per decision via direct `iw` calls.
fpvd's `tuneRadio()` forks `radio-tune.sh` per call, which is fine for
operator apply but too heavy at decision cadence. This pass adds a
lightweight txpower path the controller can call without a
fork-per-decision (e.g. a direct `iw` exec helper or netlink call),
keeping the driver-specific scaling (`88XXau → *-100`, else `*50`) in
**one** place shared with `radio-tune.sh`. Channel/width/mtu retunes
remain operator-path only (locked under DL except `channel`/`mtu`,
which don't ride the decision loop).

## 8. Schema, translator, apply() routing, `/status`

### 8.1 Schema & translator

- **Schema (`config/schema.hpp`, `etc/defaults.json`):** the
  `dynamicLink` section already exists and stays. The only change is
  **removing `mavlinkEnable`** (struct field + defaults.json line).
  Validation rules in `validate.cpp` are unchanged. `enabled` stays
  `false` by default.
- **Translator deleted** (§4.3).

### 8.2 `drone.conf` consolidation

`/etc/dynamic-link/drone.conf` is removed entirely. Every key lands in
one of five places:

| `drone.conf` key | Disposition |
|---|---|
| `min_idr_interval_ms`, `apply_stagger_ms`, `apply_sub_pace_ms`, `health_timeout_ms`, `interleaving_supported`, `debug_enable`, `osd_enable`, `osd_debug_latency`, `safe_*`, `roi_qp_*` | **Schema** — `dynamicLink.*` (now hot-reloadable) |
| `wlan_dev` | **Derived** — `RadioInfo.iface` |
| `encoder_kind` | **Derived** — pinned `waybeam` |
| MTU (`wfb.yaml`), FPS (`majestic`/`waybeam.yaml`) | **Derived** — `link.mtu`, `video.fps` (file readers deleted) |
| `listen_*`, `wfb_tx_ctrl_*`, `encoder_host/port`, `idr_listen_*`, `osd_msg_path`, `osd_update_interval_ms`, `gs_tunnel_addr/port`, hello cadence | **Controller constants** |
| `mavlink_*` | **Dropped** |
| `dbg_log_dir`, `dbg_max_bytes`, `dbg_fsync_each`, `dbg_log_enable` | **Deferred** (debug suite) |

Nothing operator-tunable is lost: the knobs that mattered are in
`dynamicLink.*` (and now hot-reloadable, which `drone.conf` never
was); the rest were deployment-fixed values fpvd already pinned.

> `dynamicLink.debug` stays in the schema but, with the debug suite
> deferred (§13), its only effect this pass is the controller's log
> verbosity (`DL_LOG_DEBUG`). The SD-card event logging it used to gate
> returns when the debug suite is ported.

### 8.3 `apply()` / diff routing — the core behavioral change

`dynamicLink` is **removed from `needsRebuild`** and routed to the
controller instead. `diffSubsystems` still computes `subs.dynamicLink`
(= `dynamicLink` subtree changed **or** `link.mtu` changed **or**
`video.fps` changed), but it now drives the *controller action*, not a
rebuild:

```
needsRebuild = subs.encoder || subs.telemetry
            || !servicesAffected.empty() || link.fullRestart;   // dynamicLink REMOVED

enabledOld = effective(pre).dynamicLink.enabled
enabledNew = pending.dynamicLink.enabled
```

| Situation | Orchestrator | Controller |
|---|---|---|
| DL knob / mtu / fps change, stays enabled, no rebuild | untouched | **hot reload** — `setConfig(snap)` + eventfd |
| `enabled` false→true, no rebuild | untouched | **start**(snap) |
| `enabled` true→false, no rebuild | untouched | **stop**() |
| Any `needsRebuild` change, DL stays enabled | full bounce (as today) | **restart-around**: `stop()` before `stopAll`, `start(freshSnap)` after `startAll` |
| `needsRebuild` change that also flips `enabled` | full bounce | start or stop accordingly, around the bounce |
| DL stays disabled | per other diffs | nothing |

**Restart-around** handles the case where an
encoder/telemetry/`linkId`/`wlanAdapter` change bounces
`wfb_video_tx`, dropping its live radiotap/FEC state and control
socket: the controller is stopped first and restarted after with a
fresh `iface`, cleared apply-caches, and a new HELLO — riding the
bounce exactly as the child process does today. The fresh snapshot is
built *after* `bringUpRadio()` so it carries the re-detected iface.

The previously-bouncing `link.mtu`-only and DL-knob-only cases now
take the no-rebuild path → pure hot reload. That is the win:
**no more blackout on adaptive-link config changes.**

`restarted[]` in the apply response gains `"dynamicLink"` whenever the
controller was started, stopped, restarted-around, or hot-reloaded
(informational).

### 8.4 `/status` surface

In-process, the control loop has no pid, so the
`status.processes[].dl_applier` row is replaced by a dedicated block.
The loop publishes a small atomic status struct the HTTP thread reads:

```jsonc
"dynamicLink": {
  "enabled": true,            // from config
  "running": true,            // control thread alive
  "watchdogTripped": false,   // true while in safe_defaults failsafe
  "lastDecisionAgeMs": 87,    // since last accepted GS decision; null if none yet
  "hello": "keepalive"        // disabled | announcing | keepalive
}
```

When `enabled` is false the block is
`{ "enabled": false, "running": false }` and `status.processes[]` no
longer lists `dl_applier`. This block is the natural home for the
deferred `apply_fail`/`watchdog` counters (the MAVLink replacement)
when wanted.

This is the **one visible API change**: a GS/client that scraped the
`dl_applier` process row reads `status.dynamicLink` instead.

## 9. Error handling

- **Startup (`start()`):** if the decision socket (`:5800`) fails to
  bind, the controller logs and reports `running:false` with an error
  surfaced in `lastApply_`/`status.dynamicLink`; fpvd stays up (the
  rest of the stack is unaffected). IDR socket (`:11223`) bind failure
  is non-fatal (IDR disabled), matching today.
- **Decision dispatch:** a backend failure on a decision is logged
  (`apply_fail` is now log-only — MAVLink dropped) and the loop
  continues, exactly as today. The watchdog still trips on GS silence
  and pushes `safe_defaults`.
- **Encoder/`wfb_tx` unreachable:** `EncoderClient`/`WfbControlClient`
  calls fail soft (timeout/`ECONNREFUSED`) and are retried on the next
  decision; level-triggered, so a transient miss self-heals.
- **`setConfig` during a bounce:** if a `needsRebuild` apply is in
  flight, the controller is stopped/restarted (restart-around) rather
  than hot-reloaded, so there is no window where the loop pushes
  commands at a half-restarted `wfb_tx`.
- **Shutdown:** `Daemon` joins the controller thread on SIGTERM before
  tearing down the orchestrator.

## 10. Testing

Wire/HELLO **byte-parity is the highest-risk item** — the GS peer is
unchanged, so the C++ port must emit byte-identical frames.

**Ported unit tests (doctest):** `Wire` (golden-byte decode/encode vs
vectors captured from the C `dl_wire`), `Dedup`, `Watchdog`,
`HelloSm`, `IdrListener`, `OsdWriter`, `RoiQp`, `applyDirection`,
`EncoderClient` (against a localhost fake HTTP server).

**Dropped tests:** `test_mavlink`, `test_config`/`test_dl_json`/
`test_dl_yaml`, `test_dl_latency`/`test_dbg`.

**New merge-specific tests:**

- `DynamicLinkController` lifecycle — `start()`/`stop()` spawn/join the
  thread, bind/close sockets; enable→disable→enable cycles are clean.
- Hot reload — `setConfig` + eventfd → loop reconciles (change
  `safe.mcs`/`healthTimeoutMs`; assert the watchdog timeout and timers
  update and the new safe ceiling is used on the next trip).
- `apply()` routing (§8.3) against the existing fake orchestrator/radio
  harness from link-hot-apply: DL-knob-only apply calls `setConfig`
  and does **not** rebuild; `enabled` toggle calls `start`/`stop`; an
  encoder change while enabled triggers restart-around.
- `status.dynamicLink` renders the documented fields.

**Test injectability:** the controller takes a defaulted `Endpoints`
struct so tests point the loop and its clients at ephemeral localhost
fakes and drive it end-to-end (inject a decision on `:5800`, assert
the resulting `wfb_tx` wire bytes + encoder HTTP call). Production
stays pinned.

## 11. Build & packaging

- **CMake:** add `src/dynlink/*.cpp` to `fpvd_core`; add the new tests;
  drop the deleted translator + obsolete tests. **No new third-party
  deps** — sockets/timerfd/eventfd are libc; `EncoderClient` uses
  `httplib::Client` (cpp-httplib already vendored).
- **Cross-compile to ssc338q (armv7l / musl / static):** the merged
  fpvd is now the binary that lands on the drone, so it must build a
  standalone static target binary the same way the
  `wfbng-dynamic-link` Makefile's `ssc338q` target does — preserving
  the dev workflow (build → `scp` to the drone, no Buildroot round-trip
  needed). Because fpvd is CMake-based, this is a toolchain file
  (`cmake/toolchain-ssc338q.cmake`) rather than a Make target:

  ```cmake
  # cmake/toolchain-ssc338q.cmake
  set(CMAKE_SYSTEM_NAME Linux)
  set(CMAKE_SYSTEM_PROCESSOR armv7l)
  set(CMAKE_C_COMPILER   armv7l-unknown-linux-musleabihf-gcc)
  set(CMAKE_CXX_COMPILER armv7l-unknown-linux-musleabihf-g++)
  set(_ssc "-march=armv7-a -mfpu=neon-vfpv4 -mfloat-abi=hard -Os")
  set(CMAKE_C_FLAGS_INIT   "${_ssc}")
  set(CMAKE_CXX_FLAGS_INIT "${_ssc}")
  set(CMAKE_EXE_LINKER_FLAGS_INIT "-static -static-libstdc++ -static-libgcc")
  ```

  Built with:

  ```sh
  cmake -S . -B build/ssc338q -DCMAKE_TOOLCHAIN_FILE=cmake/toolchain-ssc338q.cmake
  cmake --build build/ssc338q --target fpvd
  ```

  This mirrors the Makefile's `-march=armv7-a -mfpu=neon-vfpv4
  -mfloat-abi=hard -Os` + static link. The C++ static link adds
  `-static-libstdc++ -static-libgcc` (the C build did not need these).
  The cross build produces only the `fpvd` binary; `fpvd_tests` stays a
  host-build target (doctest runs on the dev machine, or under qemu-arm
  per the parent spec's smoke plan), so `enable_testing()` and the test
  executable are guarded to host builds.
- **Nix dev shell:** `fpvd/shell.nix` gains the musl cross toolchain as
  a build dep, mirroring `wfbng-dynamic-link/shell.nix`:

  ```nix
  packages = [
    pkgs.cmake pkgs.ninja pkgs.pkg-config
    pkgs.pkgsCross.armv7l-hf-multiplatform.pkgsMusl.stdenv.cc   # ssc338q gcc/g++
  ];
  ```

  which supplies the `armv7l-unknown-linux-musleabihf-{gcc,g++}` the
  toolchain file references.
- **Buildroot:** `package/wfbng-dynamic-link/` stops building/
  installing `dl-applier` and its init/unit for fpvd targets — the
  logic ships inside `/usr/bin/fpvd`. `S99fpvd` already owns startup.
- **dynamic-link repo:** the C sources under `drone/src/` are the
  porting reference and stay for history; production stops building
  them. Deleting `drone/` from that repo is a low-stakes cleanup
  follow-up — recommend leaving it until the fpvd port is proven on
  hardware.

## 12. Migration

- Ensure the old `dynamic-link-applier` init/systemd unit is gone from
  the image — there is now no separate process, so nothing can race to
  bind `:5800`.
- `/etc/dynamic-link/drone.conf` is dead — firmware update may delete
  it or leave it (fpvd ignores it). Operator knobs re-expressed via
  `dynamicLink.*`.
- **No GS-side change:** wire is byte-identical; the GS still talks to
  `:5800` and reads HELLO the same way. The MAVLink panel goes idle.
- `dynamicLink.enabled` stays `false` in `defaults.json` so an
  un-upgraded GS never sees an unexpected HELLO.

## 13. Out of scope (this pass)

- GS-side Python controller.
- Debug suite (SD failure log, latency log, timesync, RTP video tap)
  and `dl-inject`.
- MAVLink status channel (dropped).
- Encoder / `video.*` hot-apply — `video.*` still restarts waybeam.
- `/status` `apply_fail`/`watchdog` counters (natural follow-up given
  the MAVLink drop).
- Deleting `drone/` from the `wfbng-dynamic-link` repo.
- Buildroot recipe authoring details.

## 14. Success criteria

1. Boots with `enabled:false` → `status.dynamicLink =
   {enabled:false, running:false}`, no dl-applier process, no
   `drone.conf` read.
2. Enable + apply → control thread starts, HELLO reaches the GS,
   decisions apply — with **zero PID change** on any `wfb`/`waybeam`
   process (proves no bounce).
3. **Core goal:** any `dynamicLink.*` knob change + apply while enabled
   → applied live, **all `wfb`/`waybeam`/telemetry PIDs unchanged**.
4. `link.mtu` change while enabled → HELLO re-announces the new MTU, no
   stack bounce.
5. Disable + apply → thread stops, `running:false`, `wfb`/`waybeam`
   PIDs unchanged.
6. C++ `Wire`/`HelloSm` frames are byte-identical to the C
   implementation against captured vectors (GS interop preserved).
7. `video.codec` change while enabled → orchestrator bounces (as
   today) + controller restart-around (re-HELLO, caches cleared).
8. The merged fpvd cross-compiles to a static armv7l/musl ssc338q
   binary via `cmake/toolchain-ssc338q.cmake` (from the nix dev shell)
   and runs on the drone — same build → `scp` workflow the standalone
   `dl-applier` had.

## 15. Open questions / follow-ups

None blocking. Deferred:

- Surface `apply_fail`/`watchdog` counters in `status.dynamicLink`
  (the MAVLink replacement).
- Port the Phase-3 debug suite into fpvd if a deployment needs it.
- Make encoder/`video.*` changes hot (remove the waybeam bounce),
  extending the link-hot-apply pattern to the encoder.
- Delete `drone/` from `wfbng-dynamic-link` once the fpvd port is
  field-proven.
