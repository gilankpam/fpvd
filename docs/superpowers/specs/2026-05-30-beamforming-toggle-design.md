# fpvd beamforming toggle — design

## Summary

Add a beamforming (BF) toggle to fpvd's `link` config. When enabled, fpvd
drives the rtl88x2eu driver's **monitor-mode beamforming** mechanism — the
only BF that works on an unassociated wfb-ng link — by running an in-process
sounding loop that injects VHT NDPA/NDP frames and applies the steering matrix
the peer returns.

The feature is best-effort: if the active driver doesn't support monitor BF,
the video link comes up normally and `/status` reports why. Enabling BF also
brings along the coordinated changes needed to support **10MHz** link width,
because BF on a narrow channel is a primary use case.

## Background

### Why monitor-mode BF (not the normal capability bits)

The standard beamforming knob (`rtw_beamform_cap`, gated on
`CONFIG_BEAMFORMING`) only takes effect on an associated AP/STA link: explicit
BF requires a sounding handshake (NDPA → NDP → compressed beamforming report)
negotiated during association. fpvd runs the radio in monitor mode with raw
wfb-ng injection — there is no association, so `rtw_beamform_cap` is a no-op.

This driver fork ships a separate mechanism, `CONFIG_BEAMFORMING_MONITOR`
(RTL8812EU/8822EU only), that performs the sounding handshake manually via
injected frames. It is driven entirely through procfs under
`/proc/net/rtl88x2eu/<iface>/`:

- `bf_monitor_conf` — init (set beamformer MAC) / reset
- `bf_monitor_trig` — send one VHT NDPA+NDP sounding packet; reading it back
  returns the parsed Compressed Beamforming Report (CBR)
- `bf_monitor_en` — apply TXBF using the received CBR
- `ack_timeout` — sounding ACK timeout (driver default 33 µs)

The reference driver script `bf_mon.sh` loops: write `bf_monitor_conf` init,
set `ack_timeout`, then repeatedly write `bf_monitor_trig` with an incrementing
sounding token (0..63), sleep, and write `bf_monitor_en`. On stop it resets
`bf_monitor_conf` and restores `ack_timeout`.

`CONFIG_BEAMFORMING_MONITOR` is a **compile-time** driver build flag (it
`#undef`s normal `CONFIG_BEAMFORMING`). fpvd cannot toggle it; it can only use
it if the installed driver was built with it. When absent, the procfs nodes do
not exist.

### Constraints imposed by the driver

When monitor BF is active, data injection must use:

- **STBC disabled**
- **MCS 0–7 (HT)** or **MCS 0–9 single spatial stream (VHT)**

### 10MHz width realization

This driver realizes a 10MHz channel by underclocking the baseband: you set
`iw … set channel <ch> 10MHz`, but inject with **20MHz modulation**
(`wfb_tx -B 20`). So the *channel* is 10MHz while the *modulation width* used by
both wfb_tx and the BF sounding frame is 20.

## Goals

- A `link.beamforming` config block toggling monitor-mode BF on/off.
- In-process sounding loop with `/status` introspection.
- Graceful degradation when BF is unsupported (video link unaffected).
- Static validation of the driver's STBC/MCS constraints.
- First-class `link.width = 10` support across validation, wfb args, and radio
  bring-up.

## Non-goals

- Ground-station setup. fpvd is drone-side; the GS must run its own matching
  `bf_mon` against the drone's MAC (surfaced in `/status`), the same way the
  dl-applier GS counterpart lives outside fpvd.
- Building the driver with `CONFIG_BEAMFORMING_MONITOR` (a firmware build
  concern).
- Coordinating dl-applier's runtime MCS sweep against BF limits. dl-applier
  does not change STBC, and on a single-stream setup its MCS sweep stays within
  the NSS1 / 0–9 range BF requires; we rely on this rather than enforce it at
  runtime.
- 80MHz radio bring-up (a separate pre-existing gap in radio-up.sh).

## Design

### 1. Config schema

New struct in `src/config/schema.hpp`, added as a field on `Link`:

```cpp
struct Beamforming {
    bool enabled{false};
    std::string remoteMac{};                 // ground-station eFuse MAC, required when enabled
    std::optional<std::string> localMac{};   // override; omitted => resolve from iface
    int ackTimeout{255};                     // 33..255 µs
    int intervalMs{100};                     // sounding cadence
};
NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE_WITH_DEFAULT(Beamforming, enabled, remoteMac,
                                                localMac, ackTimeout, intervalMs)
```

`Link` gains `Beamforming beamforming{};` (appended to its
`NLOHMANN_DEFINE_TYPE_NON_INTRUSIVE` field list). `WITH_DEFAULT` on `Beamforming`
ensures overlays predating this key still parse.

Bandwidth is **not** stored here — it is derived from `link.width`.

`etc/defaults.json` gains under `link`:

```json
"beamforming": { "enabled": false, "remoteMac": "", "ackTimeout": 255, "intervalMs": 100 }
```

`link.beamforming` is **not** added to the dynamic-link lock list: it is
operator config, not a field dl-applier mutates.

`localMac` is normally left unset. At BF start fpvd resolves the effective
local MAC as `localMac` if present, else the interface hardware address
(`/sys/class/net/<iface>/address`). The resolved value is reported in
`/status` so the GS operator can read the drone reference. (Caveat: in monitor
mode this reads the current iface MAC — the eFuse original unless something
reassigned it; if a setup randomizes it, set `localMac` explicitly.)

### 2. Validation (`src/config/validate.cpp`)

When `link.beamforming.enabled` is true:

| Rule | Error message |
|---|---|
| `link.stbc == false` | `link.beamforming requires link.stbc=false` |
| `link.mcs` in `0..9` | `link.beamforming requires link.mcs in 0..9` |
| `remoteMac` non-empty, valid `aa:bb:cc:dd:ee:ff` | `link.beamforming.remoteMac must be a valid MAC` |
| `localMac` (if present) valid MAC | `link.beamforming.localMac must be a valid MAC` |
| `ackTimeout` in `33..255` | `link.beamforming.ackTimeout must be 33..255` |
| `intervalMs >= 1` | `link.beamforming.intervalMs must be >= 1` |

When `enabled` is false, none apply (a stale `remoteMac=""` is fine).

Independent of BF, extend existing width validation:

- `validate.cpp:43` — `link.width` must be `10`, `20`, or `40` (was 20/40).
- `validate.cpp:103` — `dynamicLink.safe.bandwidth` must be `10`, `20`, or `40`
  (consistency).

Validation runs in the existing `validate(Config)` path, so it is enforced on
both `PATCH /config` (returned as a 400 with the error) and `apply`. It does
not auto-correct, and it checks only the static config — dl-applier's runtime
MCS sweep is not validated here.

### 3. 10MHz width coordination

A shared helper keeps "10MHz means 20MHz modulation on an underclocked channel"
in one place:

```cpp
int modulationWidth(int width) { return width == 10 ? 20 : width; }
```

| `link.width` | `iw set channel` | `wfb_tx -B` / BF `bw` |
|---|---|---|
| 10 | `<ch> 10MHz` | 20 |
| 20 | `<ch> HT20` | 20 |
| 40 | `<ch> HT40+` | 40 |

- **`src/translate/wfb.cpp` `commonTx`** — `-B` uses
  `modulationWidth(c.link.width)` (currently passes `c.link.width` directly; the
  moment `10` is valid this would otherwise inject `-B 10`).
- **`scripts/radio-up.sh`** — replace the
  `[ "$FPVD_WIDTH" = "40" ] && width=HT40+ || width=HT20` line with a `case` on
  `FPVD_WIDTH`: `10)` runs `iw $WLAN_DEV set channel <ch> 10MHz`; `40)` →
  `HT40+`; default → `HT20`. Monitor mode is already set before the channel
  step, satisfying the driver's "must be in monitor mode" requirement for
  10MHz.
- **BeamformingController** — sounding `bw = modulationWidth(link.width)`.

The helper lives where both `wfb.cpp` and the controller can share it (e.g. a
small `link_width.hpp` or alongside the schema); exact location is an
implementation detail.

### 4. BeamformingController (`src/supervise/beamforming.{hpp,cpp}`)

A daemon-owned object — **not** registered in the `Orchestrator` (which
supervises only exec'd processes and is rebuilt from scratch on every apply).
It owns a worker thread and mirrors the `Supervisor` lifecycle shape.

```cpp
struct BfParams {
    std::string iface;
    std::string driver;       // from RadioResult
    std::string localMac;     // resolved
    std::string remoteMac;
    int width;                // controller derives modulation bw
    int ackTimeout;
    int intervalMs;
};

enum class BfState { Disabled, Unsupported, Active, Error };

struct BfStatus {
    bool requested{false};
    BfState state{BfState::Disabled};
    std::string reason;                  // populated for unsupported/error
    std::string localMac;                // resolved drone reference for the GS
    std::string remoteMac;
    int bw{0};
    long soundingCount{0};
    std::optional<std::string> lastCbr;  // last bf_monitor_trig readback
};

class BeamformingController {
public:
    explicit BeamformingController(std::string procBase = "/proc/net/rtl88x2eu");
    void reconcile(bool enabled, const BfParams& p);  // idempotent: start/stop/restart
    void stop();                                       // stop loop, reset driver
    BfStatus status() const;                           // snapshot for /status
private:
    void loop();
    // procBase_, thread, stopFlag, mutex-guarded BfStatus, current BfParams
};
```

**Support detection:** the proc node
`<procBase>/<iface>/bf_monitor_conf` exists. This single `stat` covers both
"wrong chipset" (only the rtl88x2eu driver registers it) and "driver built
without `CONFIG_BEAMFORMING_MONITOR`" (absent even on the right chip). No
driver-name allowlist is needed. The proc dir is `rtl88x2eu` regardless of the
`8812eu` module name.

**`reconcile(enabled, p)`** is idempotent:

- `enabled == false` → `stop()` (reset driver if it was running), state
  `Disabled`.
- `enabled == true`, unsupported → state `Unsupported`, reason
  `"no bf_monitor proc node on <iface>"`, loop not started.
- `enabled == true`, supported, not running → start loop, state `Active`.
- `enabled == true`, supported, running, params unchanged → no-op.
- `enabled == true`, supported, running, params changed → stop + restart.

**`loop()`** (mirrors `bf_mon.sh`):

1. *Init*: write `1 <remoteMac> 0 0` → `bf_monitor_conf`; write `ackTimeout` →
   `ack_timeout`.
2. *Repeat until stop*: write `<localMac> <remoteMac> 0 0 <token> <bw>` →
   `bf_monitor_trig`; `token = (token + 1) % 64`; increment `soundingCount`;
   read `bf_monitor_trig` into `lastCbr`; sleep `intervalMs`; write `1` →
   `bf_monitor_en`.
3. *Teardown* (`stop()`): write `0 00:00:00:00:00:00 0 0` → `bf_monitor_conf`;
   write `33` → `ack_timeout`; join thread.

A `/proc` write failure mid-loop transitions to `Error` (with the errno reason)
and stops the loop. The video link is unaffected — degrade gracefully.

### 5. Daemon integration (`src/daemon.{hpp,cpp}`)

`Daemon` gains a `BeamformingController bf_;` member, reconciled *after* the
orchestrator (re)starts in every link bring-up path, since it needs the
resolved `iface`/`driver` from radio bring-up:

```cpp
void Daemon::reconcileBeamforming() {
    const auto& bfc = effective_.link.beamforming;
    BfParams p;
    p.iface      = radio_.iface.empty() ? "wlan0" : radio_.iface;
    p.driver     = radio_.driver;
    p.localMac   = bfc.localMac.value_or(readIfaceMac(p.iface));
    p.remoteMac  = bfc.remoteMac;
    p.width      = effective_.link.width;
    p.ackTimeout = bfc.ackTimeout;
    p.intervalMs = bfc.intervalMs;
    bf_.reconcile(bfc.enabled, p);
}
```

Call sites (each immediately after `orch_.startAll()`):

1. `bootstrap()`.
2. `apply()` deferred channel/width retune worker (inside the detached thread).
3. `apply()` synchronous restart path.

Because `reconcile()` is idempotent, no dedicated diff flag is required — it is
called on every apply and only restarts the loop when its own params change.
The orchestrator teardown (`orch_ = Orchestrator{}`) does not touch `bf_`, so
the controller survives the rebuild.

**Shutdown:** `bf_.stop()` is invoked on the daemon shutdown path (alongside
`orch_.stopAll()`) so the driver is reset on clean exit.

**`apply()` `restarted` list:** add `"beamforming"` when the effective
`link.beamforming` block or `link.width` changed across the apply, so the API
response and `lastApply` reflect it.

### 6. `/status` reporting (`src/status.cpp`)

`buildStatus()` adds a `beamforming` object from `bf_.status()`:

```json
"beamforming": {
  "requested": true,
  "state": "active",
  "reason": "",
  "localMac": "00:c0:ca:aa:bb:cc",
  "remoteMac": "00:c0:ca:dd:ee:ff",
  "bw": 20,
  "soundingCount": 1432,
  "lastCbr": null
}
```

- `localMac` is present once the radio is up (even when BF is disabled), giving
  the GS operator the drone reference.
- `state: "unsupported"` with a reason is the graceful-degrade signal.
- `state: "error"` with the errno reason covers a mid-loop write failure.

## Testing

Host tests (`fpvd_tests`), no hardware/root required.

**Pure unit:**

- `modulationWidth()`: 10→20, 20→20, 40→40.
- Validation: BF-on rejects `stbc=true`, `mcs ∉ 0..9`, empty/invalid
  `remoteMac`, invalid `localMac`, `ackTimeout ∉ 33..255`, `intervalMs < 1`;
  BF-off accepts stale fields; `width` accepts 10/20/40 and rejects others.
- `wfb.cpp`: `width=10` → argv contains `-B 20`.
- Config round-trip: defaults parse; overlay lacking `beamforming` still loads.
- `apply()` reports `"beamforming"` in `restarted` when BF params or
  `link.width` change, and not otherwise.

**Controller with a faked procfs root** (constructor `procBase` points at a
temp dir):

- Absent `bf_monitor_conf` → `state=unsupported`, loop never starts.
- Present → `reconcile(true, …)` writes the init sequence and a first
  `bf_monitor_trig`; `stop()` writes the reset sequence and restores
  `ack_timeout=33`.
- Idempotency: identical params twice → no restart; changed
  `remoteMac`/`width`/`intervalMs` → restart; `enabled=false` → stop + reset.
- Write failure on a trig → `state=error`, loop stops, no crash.

## Affected files

- `src/config/schema.hpp` — `Beamforming` struct + `Link` field.
- `etc/defaults.json` — default `link.beamforming` block.
- `src/config/validate.cpp` — BF rules; `link.width` and
  `dynamicLink.safe.bandwidth` accept 10.
- `src/translate/wfb.cpp` — `-B` via `modulationWidth`.
- `scripts/radio-up.sh` — width `case` incl. 10MHz channel.
- `src/supervise/beamforming.{hpp,cpp}` — new controller.
- `src/daemon.{hpp,cpp}` — own + reconcile + shutdown + `restarted` reporting.
- `src/status.cpp` — `beamforming` status block.
- shared `modulationWidth` helper (small header).
- `tests/` — new coverage per above.
