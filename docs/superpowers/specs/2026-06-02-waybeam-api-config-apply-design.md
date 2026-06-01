# Waybeam API-driven config apply — design

**Date:** 2026-06-02
**Status:** approved, ready for implementation plan
**Target hardware:** ssc338q (SigmaStar Infinity6E → waybeam **Star6E** backend)

## Problem

fpvd manages the waybeam video encoder as a supervised process and writes its
config to `/etc/waybeam.json`. Today, **any** change to `video.*`, `image.*`, or
`recording.*` is classified as an `encoder` subsystem change
(`config/diff.cpp:11-13`) which sets `needsRebuild`
(`daemon.cpp:204-205`) and triggers a full orchestrator rebuild:

```
orch_.stopAll();          // kills waybeam AND every wfb process
orch_ = Orchestrator{};
seedOrchestrator();
orch_.startAll();
```
(`daemon.cpp:207-235`)

Because `stopAll()` bounces the wfb stack too, a purely encoder-side tweak
(flip the image, toggle recording, nudge bitrate while dynamic-link is off)
**drops the entire radio link**. waybeam already exposes an HTTP API to change
all of these in place — and fpvd already speaks it for dynamic-link
(`dynlink/encoder_client.cpp` pushes `video0.bitrate`/`fpv.roiQp`/`video0.fps`
via `GET /api/v1/set`).

## Goal

Apply encoder config through waybeam's API instead of bouncing the process,
so the radio link is never dropped for an encoder-only change. Specifically
(per design decisions below): live-apply where waybeam supports it (no glitch),
and for restart-class fields restart **only** waybeam (wfb untouched).

## Key discovery — why we can't lean on `/api/v1/set`'s auto-reinit for restart fields

waybeam classifies each settable field as `MUT_LIVE` or `MUT_RESTART`
(`venc_api.c:294-380`). A `/set` on a `MUT_RESTART` field persists and triggers
a reinit itself (`venc_api.c:103 venc_api_request_reinit()`) — no separate
`/api/v1/restart` call is needed. **But how that reinit executes is
backend-specific:**

- **Maruko** does a clean in-process reinit — same PID, HTTP server survives
  (`maruko_pipeline.c:3615`, teardown_graph + rebuild; the code comment is
  explicit: *"Star6E gets this for free via the fork+exec process boundary;
  Maruko reinit is in-process"*).
- **Star6E (our ssc338q target)** cannot re-init MI_SYS in the same PID — the
  SigmaStar driver retains per-PID "already inited" flags. So
  `star6e_runtime_handle_reinit()` calls `venc_respawn_request()` and the
  process **exits and fork+execs a fresh PID** (`venc_respawn_after_exit()`,
  `star6e_runtime.c:710-752`, `venc_respawn.c`).

fpvd's `Supervisor` polls `reapIfReady()` every 50 ms and, because waybeam is
`RestartPolicy::Always`, **restarts waybeam itself on that exit**
(`supervise/supervisor.cpp:47-72`). Meanwhile waybeam's own respawn-child is
independently waiting for the old PID to die, then `execv`-ing waybeam. On
Star6E this yields **two waybeam instances racing over MI_SYS + port 80**.

Conclusion: waybeam's self-respawn is designed for running *unsupervised*. Under
fpvd we must not trigger it. fpvd uses the API only for changes with no process
lifecycle (the LIVE fields) and **owns every restart itself**.

## Design decisions (confirmed)

1. **Optimize for both** — live-apply LIVE fields instantly (no glitch), and for
   RESTART fields restart only waybeam (radio stays up, brief glitch).
2. **On API push failure → fail the `POST /apply`, keep the link up** — return an
   error, leave waybeam as-is, operator retries. Implies the push is
   transactional: push *before* committing `effective_`.
3. **Restart-class fields → fpvd-owned waybeam restart** — fpvd rewrites
   `waybeam.json` and restarts only the waybeam process via its supervisor.
   `SIGTERM` kills waybeam cleanly without setting its reinit flag, so no
   self-respawn child is forked; fpvd is the sole restarter. Backend-agnostic.

## Config → waybeam field mapping

Mutability taken from waybeam's own field table (`venc_api.c:294-380`). camelCase
aliases (e.g. `video0.gopSize`, `fpv.roiQp`) and snake_case are both accepted by
`/api/v1/set`; the apply path uses the canonical snake_case names.

### LIVE — batched `GET /api/v1/set?…`, zero disruption

| fpvd config        | waybeam field      |
|--------------------|--------------------|
| `video.bitrate`    | `video0.bitrate`   |
| `video.fps`        | `video0.fps`       |
| `video.gopSize`    | `video0.gop_size`  |
| `video.qpDelta`    | `video0.qp_delta`  |
| `video.roi.enabled`| `fpv.roi_enabled`  |
| `video.roi.qp`     | `fpv.roi_qp`       |
| `video.roi.steps`  | `fpv.roi_steps`    |
| `video.roi.center` | `fpv.roi_center`   |

### RESTART — fpvd rewrites `waybeam.json` + restarts only waybeam

| fpvd config           | waybeam field        |
|-----------------------|----------------------|
| `video.resolution`    | `video0.size`        |
| `video.rcMode`        | `video0.rc_mode`     |
| `image.mirror`        | `image.mirror`       |
| `image.flip`          | `image.flip`         |
| `image.rotate`        | `image.rotate`       |
| `recording.enabled`   | `record.enabled`     |
| `recording.format`    | `record.format`      |
| `recording.mode`      | `record.mode`        |
| `recording.maxSeconds`| `record.max_seconds` |
| `recording.maxMB`     | `record.max_mb`      |

### Dropped

| fpvd config    | waybeam field   | handling |
|----------------|-----------------|----------|
| `video.codec`  | `video0.codec`  | **Retired** in waybeam (hardcoded H.265, `/set` returns 404). Remove from the translator; `validate()` pins `video.codec` to `"h265"`. Never pushed. |

Everything else fpvd writes into `waybeam.json` (sensor/isp/outgoing/audio/imu/
debug/system) is hardcoded in the translator and never changes at runtime, so it
is out of scope.

## Reconcile rule

Computed purely from the diff of `effective_` vs `pending_` over the mapped
fields (codec excluded; DL-owned fields excluded when dynamic-link is enabled —
see Dynamic-link coordination):

- diff contains **any RESTART** field → one **waybeam-only restart**. waybeam
  reloads the whole file on startup, so this also applies any LIVE fields changed
  in the same apply.
- else diff contains **LIVE** fields → one **batched `/api/v1/set`**.
- else → nothing.

At most **one** lifecycle event per apply, and never a wfb bounce. The full
orchestrator rebuild remains **only** for `telemetry.*`, `services.*`, and
`link.linkId`/`link.wlanAdapter` (`link.fullRestart`).

## Components

### Shared transport: `WaybeamClient` (new, `src/waybeam/client.{hpp,cpp}`)

Namespace `fpvd`. Extracts the transport that `EncoderClient::httpGet` currently
inlines, so both the dynamic-link controller and the apply path share it.

```cpp
class WaybeamClient {
public:
    WaybeamClient(std::string host, uint16_t port,
                  int connectTimeoutMs = 300, int readTimeoutMs = 500);
    // GET /api/v1/set?k1=v1&k2=v2…  (values URL-encoded). true on 2xx.
    bool setFields(const std::map<std::string,std::string>& fields);
    // Raw GET — used for /request/idr (and anything else). true on 2xx.
    bool get(const std::string& path);
};
```

Holds only immutable config and builds a **fresh `httplib::Client` per call**
(as `httpGet` does today), so it is stateless and safe for the daemon thread and
the DL control-loop thread to share one instance — no locking, independent
sockets. `setFields` owns URL-encoding and request building; callers pass
already-formatted string values.

### `EncoderClient` refactor (`src/dynlink/encoder_client.{hpp,cpp}`)

Keeps all DL **policy** (ROI-curve math, `lastBitrate_/lastRoiQp_/lastFps_`
diff-suppression, IDR throttle) but stops owning host/port/httplib. It takes a
`WaybeamClient&`; `apply`/`applySafe` build the `{video0.bitrate, fpv.roiQp,
[video0.fps]}` map → `client.setFields(...)`; `requestIdr` → `client.get(
"/request/idr")`. Query strings, timeouts, and dedup are unchanged, so existing
DL tests stay green. The controller owns a `WaybeamClient` (built from its
`Endpoints`, `encHost/encPort`) and passes it by reference to its `EncoderClient`.

### Translator + diff (`src/translate/waybeam.{hpp,cpp}`)

- Drop `video0.codec` from `toWaybeamJson`.
- Add `waybeamConfigDiff(old, new, dlEnabled) -> {live: map<string,string>,
  restart: map<string,string>}`. This module owns the single
  field→waybeam-name→mutability table (the companion to the translator), the
  value formatting (bool → `"true"/"false"`, double precision), and the
  exclusions: `video.codec` is never emitted, and the DL-owned fields are
  excluded when `dlEnabled` (see Dynamic-link coordination). The reconcile rule
  itself (any `restart` ⇒ restart, else any `live` ⇒ live, else none) is applied
  by `apply()` directly from `.restart`/`.live`.

### Orchestrator / Supervisor (`src/supervise/…`)

Add `Orchestrator::restart(const std::string& name)` (today there is only
`startAll`/`stopAll`, `orchestrator.hpp:26-28`). Implemented as a clean
`shutdown()` + `start()` of that one `Supervisor`, giving an immediate restart
(no failure-backoff path) of waybeam while every other supervised process keeps
running. waybeam's `startAfter wfb_video_tx` dependency is already satisfied
(wfb stays up), so a standalone restart is well-defined.

### Daemon (`src/daemon.cpp`)

`Daemon` owns its own `WaybeamClient` for the apply path (endpoint from
`DaemonPaths.dlEndpoints`, default `127.0.0.1:80`) — the same shared transport
class the controller uses, as two cheap stateless instances reading one endpoint
definition. `apply()` is restructured (see below). `validate()` gains the
`codec == "h265"`
check.

## `apply()` control flow (restructured)

```
validate(pending)                         // now includes codec == "h265"
subs  = diffSubsystems(effective, pending)
link  = classifyLinkChange(effective, pending)
enabledOld/New, bfChanged                 // unchanged

wbDiff = waybeamConfigDiff(effective, pending, enabledNew)
        // {live, restart}; codec excluded; DL-owned excluded when enabledNew
encRestart = !wbDiff.restart.empty()          // reconcile rule: any restart field ⇒ restart
encLive    = wbDiff.restart.empty() && !wbDiff.live.empty()
encChanged = encRestart || encLive

needsRebuild = subs.telemetry || !subs.servicesAffected.empty() || link.fullRestart
        // NOTE: subs.encoder REMOVED from this condition

// Transactional live push — BEFORE commit, only on the hot path.
// (If needsRebuild, the rebuild restarts waybeam and reads the new file, so skip.)
if (reallyRestart && !needsRebuild && encLive)
    if (!waybeam_.setFields(wbDiff.live))
        return {ok:false, …};             // nothing committed; waybeam unchanged; link up

// Commit (unchanged from today)
persist overlay; effective = pending; rewriteWaybeamJson();

restarted[] += "encoder" if encChanged   // plus existing radio/telemetry/dynamicLink/bf entries

if (reallyRestart && needsRebuild) {
    … full rebuild, unchanged (daemon.cpp:207-235) …
} else if (reallyRestart) {
    if (encRestart) orch_.restart("waybeam");   // file already rewritten above
    … existing DL routing (daemon.cpp:242-250) …
    … existing radio hot-apply (txpower/mtu/fec/radiotap/channel) …
} else {
    … existing dry re-seed (daemon.cpp:342-347) …
}
```

The LIVE `/set` is the only step moved ahead of the commit, so a failed push
aborts cleanly and a retried `POST /apply` still sees the diff. The waybeam-only
restart has no API call and is fpvd-local; if waybeam fails to come back the
existing `Supervisor` retry/backoff handles it, exactly as for any crash. The
radio hot-apply steps keep their current commit-then-act semantics (out of scope
to change).

## Dynamic-link coordination

When `dynamicLink.enabled`, the controller is the sole writer of `video.bitrate`,
`video.qpDelta`, `video.roi.*` (already PATCH-locked, `config/lock.cpp:8-22`, so
they cannot appear in a diff) and `video.fps`. So `waybeamConfigDiff` **excludes
the DL-owned fields when DL is enabled**:

- `bitrate`/`qpDelta`/`roi` — locked, never in a diff anyway.
- `fps` — not locked, but excluded from `waybeamConfigDiff` when DL is enabled.
  An fps change updates the helloFps advertised to the GS via `dl_.setConfig()`
  (fired by `subs.dynamicLink`, `diff.cpp:20`, `daemon.cpp:246-250`); the GS
  then echoes a Decision back which the controller pushes to waybeam — GS-round-
  trip-mediated, not a direct local `/set`. The new fps value is also committed
  to `waybeam.json` by `rewriteWaybeamJson()`, so it survives a waybeam restart.

A waybeam-only restart (e.g. a resolution change) while DL runs is safe: the
controller talks to waybeam over HTTP with short timeouts and simply retries
across the ~1-2 s gap; the radio (wfb_tx) is independent of waybeam, so link
decisions are unaffected. The DL controller is **not** bounced on a waybeam-only
restart (unlike the full-rebuild path's restart-around, which is needed only
because wfb bounces there).

**Accepted limitation — disable-transition clobber window.** An apply that
*both* disables dynamic-link and changes a DL-owned LIVE field in one request
(e.g. `{"dynamicLink":{"enabled":false},"video":{"bitrate":X}}`) is allowed (the
PATCH lock keys on `enabled`, so it passes once `enabled:false`). In `apply()`
the LIVE `/set` of `X` runs pre-commit while the controller is still running, and
`dl_.stop()` runs in the post-commit hot path — so a GS decision arriving in that
microsecond window can momentarily overwrite `X`. This is self-healing: the
committed `waybeam.json` holds `X`, so the next waybeam restart restores it, and
the controller is gone thereafter. We deliberately do NOT stop the controller
before the pre-commit push: doing so would, on a push failure, leave the
controller stopped while `effective_.dynamicLink.enabled` stays true (a
persistent inconsistency strictly worse than the transient, self-healing race).

## Testing

- Unit — `waybeamConfigDiff`: live/restart bucketing; codec excluded; DL-owned
  fields excluded when enabled, included when disabled; bool/double formatting.
- Unit — `validate()`: rejects `video.codec != "h265"`.
- Unit — `Orchestrator::restart`: bounces the named process, leaves the others
  running.
- Integration — `apply()` against a stub waybeam HTTP server:
  - LIVE-only change issues the expected single `/api/v1/set` and performs no
    restart;
  - RESTART change rewrites `waybeam.json` and bounces **only** waybeam (wfb
    supervisors untouched);
  - a `/set` failure returns an apply error with `effective_` unchanged (retry
    re-attempts);
  - mixed LIVE+RESTART in one apply → single waybeam-only restart, no `/set`.
- Regression — existing dynamic-link `EncoderClient` tests pass unchanged after
  the transport refactor (identical query strings/timeouts/dedup).

## Out of scope / risks

- The `EncoderClient` refactor touches recently-shipped DL code. It is a
  mechanical delegation with no behavior change; the regression test above is the
  guard.
- Radio hot-apply (txpower/mtu/fec) keeps its existing commit-then-act ordering;
  only the encoder `/set` is made transactional.
- Maruko's in-process reinit path is not used (we own restarts), but the design
  is backend-agnostic, so a future Maruko build needs no change here.
