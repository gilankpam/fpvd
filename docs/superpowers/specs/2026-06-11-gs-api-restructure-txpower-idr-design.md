# GS API restructure, txpower unification, and IDR forwarder decouple

**Date:** 2026-06-11
**Status:** Design approved, ready for implementation planning
**Branch:** fresh from `main` (supersedes the abandoned `feat/unified-config` server-orchestration design)

## Summary

Four related changes, spanning the GS daemon (`gs/fpvdgs/`, Python) and the
drone daemon (`drone/`, C++):

1. **GS endpoint restructure** — move GS-local routes under `/gs/*`, keep the
   drone proxy at `/air/*`.
2. **Remove the `/link` coordinator** — fold `link` into `/gs/config` and let
   the client orchestrate drone + GS changes.
3. **Unify txpower on dBm** — one unit at the config/API surface, edges convert.
4. **Dedicate the IDR forwarder** — its own config block and lifecycle, no
   longer coupled to `dynamicLink.enabled`.

The drone needs **no API change** — it has no `/link`, and its `link` config is
already part of its `/config` tree, so it is already client-orchestration
friendly (`PATCH /air/config` → `POST /air/apply`). The drone is touched only
for the txpower unit change.

## Motivation

The GS front door (`gs/fpvdgs/api.py`) currently mixes three concerns at the
root: GS-local config (`/config`, `/apply`, …), an opaque drone proxy
(`/air/*`), and a `/link` coordinator that pushes shared link params to *both*
sides via `/link/apply`. This makes the namespace ambiguous (is `/config` the
drone's or the GS's?) and bakes drone↔GS coordination into the server.

Separately, txpower is expressed in three different units depending on the path,
and the IDR forwarder has no independent existence — it lives inside the
dynamicLink controller and only runs when dynamicLink is enabled.

## Section 1 — GS endpoint restructure

### Route map

| Concern | Current (main) | New |
|---|---|---|
| Drone proxy | `/air/*` (opaque) | `/air/*` (unchanged) |
| GS-local config | `/config` | `/gs/config` |
| GS-local apply | `/apply` | `/gs/apply` |
| GS-local reset | `/reset` | `/gs/reset` |
| GS-local status | `/status` | `/gs/status` |
| GS-local defaults | `/defaults` | `/gs/defaults` |
| Daemon liveness | `/healthz` | `/healthz` (stays at root) |
| Link coordinator | `/link`, `/link/apply` | **removed** |

`/healthz` stays at the root because it reports liveness of the GS HTTP
process itself, which is namespace-agnostic (it is not "the GS config" nor "the
drone").

`/air/*` remains an opaque pass-through proxy to the drone's own
`/config`, `/apply`, `/reset`, `/status`, `/defaults`, `/healthz`.

### Implementation notes (`api.py`)

- Change the GS-local route keys from `/config`, `/apply`, … to `/gs/config`,
  `/gs/apply`, … in `handle()`.
- Delete the `/link`, `/link/apply` branches and the `_link_view()` helper.
- Delete the `_apply_gs` "link drift" 409 guard
  (`if pending.get("link") != effective.get("link"): return 409 …`). Link is now
  applied like any other GS-local key.

### Folding `link` into `/gs/config`

Today `link` is excluded from `/config` and is mutable only via `/link`
(`schema.py`: `CONFIG_TOP_KEYS` excludes `link`; `validate_config_patch` raises
"link.* is read-only via /config"; `validate_link_patch` gates the `/link`
path).

New behavior:

- `link` becomes a normal mutable block in `/gs/config`.
- Merge `LINK_KEYS` validation into the `/gs/config` patch path
  (`validate_config_patch`).
- Remove `validate_link_patch` and the read-only raise.
- `ALL_TOP_KEYS` / `CONFIG_TOP_KEYS` collapse — `link` is just another top key.

### Client orchestration (documentation, not enforcement)

Removing `/link/apply` moves drone↔GS coordination to the client. To change a
shared link parameter (`channel`, `width`, `linkId`, `region`), the client:

1. `PATCH /air/config { "link": { … } }`  then  `PATCH /gs/config { "link": { … } }`
2. `POST /air/apply`  **then**  `POST /gs/apply`

**Recommended ordering:** on a channel/width move, apply the **drone first**, so
the ground station retunes onto the link the drone has already moved to. The
server enforces nothing — this ordering lives in the API docs and is the
client's responsibility. This is the intended trade-off: the server stops being
a coordination point.

### GS-local link apply (replaces `LinkCoordinator`)

On `main`, `_apply_gs` **rejects** any `link` change with a 409 — the actual
link-apply mechanics live in `LinkCoordinator.apply_link`, which bundles three
concerns: (a) GS-local RF apply (live `iw` retune vs. runner bounce), (b) the
drone push of shared keys, and (c) beamforming arm/disarm coordination. With
`/link` removed, (b) becomes the client's job and (c) moves to the armer (see
below), leaving only (a) as a GS-local concern.

Decision: **delete `LinkCoordinator` entirely** and write a fresh, small
GS-local link applier inside `_apply_gs`. It preserves only the valuable part —
the live-`iw` **retune-vs-bounce** optimization:

- When `pending.link != effective.link`, decide retune vs. bounce on the
  **non-beamforming** link delta:
  - **Live `iw` retune** when every changed key is in
    `{channel, width, txpower→txPowerDbm, region}` **and** the radiotap BW class
    is unchanged (10 and 20 MHz are both `BW_20`; only crossing 40 differs).
  - **Runner bounce** otherwise (`wlans`, `linkId`, profile change, or a 40 MHz
    crossing).
  - On a failed retune, fall back to a bounce; on a failed bounce, restore the
    last-good cfg and bounce again (rollback), mirroring today's behavior.
- **No drone push, no `apply_to`, no beamforming** in this path. The retune
  helper (`radio.retune`) and the `_bw_class` rule carry over from
  `LinkCoordinator`/`radio.py` unchanged.
- `_apply_gs` integrates this alongside the existing wfb / dynamicLink /
  pixelpilot routing: render the pending cfg, apply the link delta (retune or
  bounce), then route the non-link blocks, then commit.

Keeping retune (vs. always-bounce) is deliberate: a plain channel or txpower
change must not drop the video pipeline, and it matters for the planned
auto-channel-hop feature.

### Beamforming: GS beamformee self-reconciles from config

The drone↔GS BF **MAC handshake is cross-device, so the client owns it.** To
enable downlink BF the client: reads the GS card MAC from `/gs/status`, then sets
the drone's `link.beamforming` via `/air` — `enabled:true`,
`remoteMac:<GS card MAC>`, and `stbc:false` (STBC and TX-BF are mutually
exclusive on the drone) — and applies `/air`.

On the GS side the beamformee only needs to **match `link.beamforming.enabled`**.
The `BeamformingArmer` already reconciles toward config every 5 s (reading the
drone MAC read-only) but **only ever arms**. Decision:

- **Extend `BeamformingArmer` to a full reconcile** — arm when
  `link.beamforming.enabled` is true *and* the beamformee isn't active, **and
  disarm** when it is false *and* the beamformee is still armed. This closes the
  orphaned-disarm gap (previously only `apply_link` disarmed) with less code than
  porting `reconcile` into the apply path. It still reads the drone MAC
  read-only; it never pushes to the drone.
- `/gs/apply` fires **one immediate armer tick** after a successful apply so a
  toggle isn't stuck waiting for the 5 s loop.
- The **capability hard-reject** (enabling BF requires a `bf_monitor_conf` node
  on the primary card) moves into `/gs/config` **schema validation** — fail fast
  at PATCH time. Validation needs a card-capability probe; inject the
  `BeamformingController.supported(iface)` check (and `resolve_wlans`) into the
  patch-validation path, or surface it through `validate_effective` at apply
  time if the iface list isn't available at patch time. (Plan picks the concrete
  wiring; the requirement is: enabling BF on an incapable card is rejected, not
  silently dropped.)
- **No beamforming logic remains in the apply critical path.**

Net effect: `LinkCoordinator` (188 lines) is deleted; the armer grows by a few
lines; BF becomes purely config-driven on the GS; every cross-device step lives
in the client.

## Section 2 — txpower unified on dBm

### Current units (the problem)

| Path | Field | Unit | Range |
|---|---|---|---|
| GS static | `link.txpower` | mBm (rendered to wfb-ng `wifi_txpower`) | nullable |
| GS dynamic | `dynamicLink.txpower.min/max` | dBm | — |
| Drone static | `link.txpower` | raw driver level (`×50` or `×-100` → mBm) | 1..63 |
| Drone dynamic | `dynamicLink.txPowerDbm`, `safe.txPowerDbm` | dBm | -10..30 |

### Target: dBm at every config/API surface

Both dynamic paths already use dBm, and the drone's dynamic radio path
(`drone/src/dynlink/radio_txpower.cpp`) already runs
`iw dev <iface> set txpower fixed <mBm>` where `mBm = dBm * 100`. So dBm is the
natural canonical unit; only the two *static* edges need to change, and they
converge on the same `×100` math the dynamic path already uses.

### Key rename

Rename the static key for a self-documenting unit:

- **GS:** `link.txpower` → `link.txPowerDbm`
- **Drone:** `link.txpower` → `link.txPowerDbm`

This matches `dynamicLink.txPowerDbm`. It requires a one-time migration of
`gs/etc/defaults.json`, the drone's `defaults.json`, and the deployed
`config.drone.json` / `config.gs.json`.

### GS changes

- `render.py`: read `link.txPowerDbm`; when non-null emit
  `wifi_txpower = <dBm * 100>` (mBm). Keep the "omit ⇒ driver default" behavior
  when null. Update the NOTE comment.
- `radio.py`: update the txpower comment (still mBm at the `iw` edge, dBm at the
  config edge).
- `schema.py`: `LINK_KEYS` swaps `txpower` → `txPowerDbm`; validate dBm range
  (align with the drone, e.g. `-10..30`) and allow null.

### Drone changes

- `config/schema.hpp`: `Link::txpower` (int, default 1) → `txPowerDbm`
  (int, dBm). Update `NLOHMANN_DEFINE_TYPE_…` field list.
- `config/validate.cpp`: range check → dBm (e.g. `-10..30`, matching
  `dynamicLink.safe.txPowerDbm`). Update the field name and message.
- `config/diff.cpp`, `config/lock.cpp`: rename the field references
  (`la.txpower` → `la.txPowerDbm`; the lock entry `{"link","txpower"}` →
  `{"link","txPowerDbm"}`). The lock semantics (link txpower locked while DL
  owns per-MCS power) are unchanged.
- `scripts/radio-tune.sh`, `scripts/radio-up.sh`: replace the per-driver
  `iw set txpower fixed $(( FPVD_TXPOWER * 50 ))` / `* -100` branches with a
  single `iw set txpower fixed $(( FPVD_TXPOWER_DBM * 100 ))`, matching
  `radio_txpower.cpp`. The env var the daemon exports becomes `FPVD_TXPOWER_DBM`
  (update `daemon.cpp`'s `tuneRadio(... "txpower" ...)` env wiring).

Both `dynamicLink` dBm paths are unchanged.

## Section 3 — IDR forwarder as a dedicated config + lifecycle

### Current state

The IDR relay is an inner class `_IdrRelay` inside
`gs/fpvdgs/dynlink/controller.py`, started/stopped inside the controller's
run loop, gated by `dynamicLink.idrForward` (default true) and
`dynamicLink.idrPort` (default 11223), forwarding local IDR tokens to
`(dynamicLink.droneAddr, idrPort)`. It only exists while `dynamicLink.enabled`.

### New config block (GS)

A new top-level GS block:

```json
"idrForward": {
  "enabled": true,
  "port": 11223
}
```

- The drone UDP target host is **derived from the existing top-level
  `drone.endpoint`** (parse host from e.g. `http://10.5.0.10:8080`). No
  duplicated address; one source of truth.
- `port` is the UDP port for both the local listen (`0.0.0.0:port`) and the
  drone forward target (`<droneHost>:port`), preserving today's behavior.

### Extraction + lifecycle

- Extract `_IdrRelay` into its own module `gs/fpvdgs/idr_relay.py` (salvage the
  extracted version from the abandoned `feat/unified-config` branch as a
  starting point) with an explicit start/stop lifecycle, owned by `App`.
- Remove `idrForward` / `idrPort` from the `dynamicLink` block and from
  `schema.py`'s `_validate_dynamic_link` (drop the `idrPort` validation there;
  add equivalent validation for the new `idrForward.port`).
- `controller.py` no longer creates or manages the relay.
- `/gs/apply` gains an `_route_idr_forward(old, new, pending)` router that
  starts / stops / reconfigures the relay on `idrForward` changes, mirroring
  `_route_dynamic_link` and `_route_pixelpilot`. It never bounces the wfb
  runner.
- The relay runs **independent of `dynamicLink.enabled`** — it can forward IDR
  tokens with dynamicLink off, which is the entire point of the decouple.
- `App` boot-starts the relay when `idrForward.enabled` is true (mirroring how
  pixelpilot boot-starts), since its default is enabled.

### Drone host derivation

Add a small helper (e.g. in `idr_relay.py` or a shared util) that parses the
host out of `drone.endpoint`. The relay is constructed with
`(droneHost, port)`; if `drone.endpoint` is unset/unparseable, the relay logs
and stays down (non-fatal, matching today's bind-failure tolerance).

## Section 4 — testing

### GS unit tests

- **Routing** (`test_api.py`): GS-local routes answer under `/gs/*`; `/air/*`
  still proxies; `/link` and `/link/apply` now return 404; `/healthz` still at
  root.
- **link via /gs/config**: a `link` PATCH through `/gs/config` is accepted and
  validated (was previously rejected as read-only); unknown link keys still
  rejected.
- **link apply** (`test_api.py`): a channel/txPowerDbm/region change through
  `/gs/apply` retunes live (no runner bounce); a `wlans`/`linkId`/40 MHz-crossing
  change bounces; a failed retune falls back to bounce; a failed bounce restores
  last-good. (Replaces the deleted `test_link.py`; `LinkCoordinator` is gone.)
- **beamforming** (`test_beamforming_armer.py`): the armer arms when
  `link.beamforming.enabled` flips true and disarms when it flips false (full
  reconcile); reads drone MAC read-only; `/gs/apply` triggers an immediate tick;
  enabling BF on a card with no `bf_monitor_conf` is rejected at `/gs/config`
  (or `validate_effective`) time.
- **txpower** (`test_render.py`, `test_schema.py`): `link.txPowerDbm` renders to
  `wifi_txpower = dBm*100`; null omits it; out-of-range dBm rejected.
- **IDR** (`test_idr_relay.py`, `test_api.py`): relay starts/stops on
  `idrForward.enabled`; runs with `dynamicLink.enabled=false`; drone host
  derived from `drone.endpoint`; `/gs/apply` routes idrForward changes; old
  `dynamicLink.idrForward/idrPort` keys rejected by schema.

### Drone unit tests

- **Validation** (`test_validate.cpp`): `link.txPowerDbm` accepts in-range dBm,
  rejects out-of-range; old `txpower` key handling per the migration.
- **Schema** (`test_schema.cpp`): round-trips `txPowerDbm`.
- **radio-tune script** (`test_radio_tune_script.cpp` + fake_radio_tune
  fixture): exports `FPVD_TXPOWER_DBM`, script computes `dBm*100` mBm.
- **diff/lock** (`test_diff.cpp`, `test_lock.cpp`): renamed field still diffs and
  locks correctly.

Run drone tests via `./build/fpvd_tests` from `drone/` (not ctest).

### Integration

- GS supervisor e2e (`test_supervisor_e2e.py`): exercise separate `/air` + `/gs`
  apply paths; confirm `/link` is gone; confirm IDR relay lifecycle.

## Migration / deployment notes

- Config key migration (`link.txpower` → `link.txPowerDbm`, value reinterpreted
  as dBm) touches `gs/etc/defaults.json`, the drone `defaults.json`, and the
  deployed `config.drone.json` / `config.gs.json`. Pick sane dBm defaults
  (the drone's static default was a raw level of 1 — choose an explicit dBm
  default, e.g. the dynamic `safe` value, rather than a mechanical 1→1).
- `dynamicLink.idrForward` / `idrPort` are removed; the new top-level
  `idrForward` block carries them. Deployed configs must move these keys.
- Any client / dashboard hitting `/config`, `/apply`, `/status`, `/link`,
  `/link/apply` must update to `/gs/*` and the client-orchestration flow.

## Out of scope

- No server-side drone↔GS coordination (explicitly removed — client owns it).
- No change to the drone's HTTP API surface beyond the txpower field rename.
- No change to dynamicLink / probe control logic beyond removing the IDR keys.
