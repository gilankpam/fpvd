# Drone BF OSD (real-state indicator) — Design

**Date:** 2026-06-09
**Status:** Approved design, follow-up to GS downlink beamforming
**Depends on:** `2026-06-09-gs-beamforming-downlink-design.md` (the GS beamformee
must exist for `cbr_rssi` to ever populate). Best implemented **after** the
GS+drone BF path is hardware-proven, so the OSD is built against a confirmed
signal.

## Goal

Show a beamforming indicator on the OSD the pilot sees, reflecting whether
downlink BF is **actually working** (the drone is receiving compressed
beamforming reports from the GS) — not merely whether it is configured.

## Why "real state" needs a specific signal

The drone is the beamformer: it sounds the GS and applies a steering matrix only
if the GS echoes a Compressed Beamforming Report (CBR). The authoritative
"working" signal is the driver's `bf_monitor_rfinfo`:

```
token : ndp_snr0 : ndp_snr1 : cbr_rssi0 : cbr_rssi1 : cbr_snr0 : cbr_snr1
```

The `cbr_*` fields are the RF info of the CBR frame **received by the drone from
the GS**. So:

- `cbr_rssi0 != 0` ⇒ the drone is receiving reports ⇒ **BF working end-to-end**.
- All-zero (`0:22:22:0:0:0:0`, the current live read) ⇒ no report ⇒ not working.

This is far more reliable than the existing `BfStatus.lastCbr`, which is a
premature read of `bf_monitor_trig` (read immediately after triggering, before
the GS could respond — see the drone-side review notes). `lastCbr` must NOT be
used as the OSD signal.

## OSD architecture (the constraint)

The OSD is produced on the drone and rendered by the GS PixelPilot
(`ExternalSurfaceWidget` named "msposd"). There are **two** writers to
`/tmp/MSPOSD.msg`:

1. **Flight (DL on):** `dynlink::OsdWriter::writeStatus()` owns the line
   (`...MCS4 9M (8,12)d1 TX22 R-60 I3 | &B T&T W&W CPU&C`). The dynamic-link
   controller calls it and has **no handle** to the BF controller.
2. **DL off (fallback):** `Daemon::writeOsdBaseLine()` writes a static line; the
   Daemon has direct BF access here.

Since DL is normally on in flight, the indicator MUST be added to the DL
`OsdWriter` path, which requires injecting BF status into the DL controller.

## Design

### 1. BF "working" signal — `drone/src/supervise/beamforming.{hpp,cpp}`

- In the sounding loop, additionally read `bf_monitor_rfinfo` and parse the
  4th colon-field (`cbr_rssi0`) as an int.
- Add `int cbrRssi{0}` to `BfStatus` (0 = no report). Derived predicate
  `reportActive = state==Active && cbrRssi != 0`.
- Keep the existing `lastCbr` field; it is not used for the OSD.

### 2. OSD token — `drone/src/dynlink/osd.{hpp,cpp}`

- `writeStatus(const Decision& d, int rssiDbm, int bfCode)` gains a `bfCode`:
  - `0` = BF off/disabled → render nothing
  - `1` = armed but no report (`cbrRssi==0`) → render a dim ` B·`
  - `2` = working (`reportActive`) → render ` B✓`
- Token appended after the `I%u` field, before the `|` divider, e.g.
  `...I3 B✓ | &B T&T W&W CPU&C`. ASCII-only fallback (`B+`/`B-`) if the
  msposd font lacks the glyphs — confirm during implementation.

### 3. Inject BF status into the DL controller — `drone/src/dynlink/controller.{hpp,cpp}` + `drone/src/daemon.cpp`

- DL controller stores `std::function<int()> bfCodeProvider_` (default returns
  `0`) with a `setBfCodeProvider(...)` setter.
- At the `osd_->writeStatus(lastApplied_, 0)` call site, pass
  `bfCodeProvider_ ? bfCodeProvider_() : 0`.
- The Daemon wires it once (e.g. in `startController()`):
  `dl_.setBfCodeProvider([this]{ return bfOsdCode(); });` where `bfOsdCode()`
  maps `beamformingStatus()` → 0/1/2.

### 4. DL-off fallback — `Daemon::writeOsdBaseLine()`

- Append the same token using `beamformingStatus()` directly (Daemon has access
  here), so the indicator is consistent whether or not DL is running.

## Testing

- **BF rfinfo parse** (`drone/tests/integration/test_beamforming.cpp`): tmp proc
  dir with a `bf_monitor_rfinfo` node returning `0:30:12:-60:-58:28:29` ⇒
  `BfStatus.cbrRssi == -60`; `0:22:22:0:0:0:0` ⇒ `cbrRssi == 0`.
- **OSD render** (`drone/tests/.../test_osd*` or a new unit): `writeStatus` with
  `bfCode=2` includes the working token; `bfCode=0` includes no BF token;
  `bfCode=1` includes the dim token.
- **Daemon wiring**: `bfOsdCode()` maps Active+cbrRssi!=0 → 2, Active+0 → 1,
  else → 0.
- **writeOsdBaseLine**: includes the token derived from `beamformingStatus()`.

Build/run drone tests: `./build/fpvd_tests` from `drone/` (NOT ctest).

## Out of scope

- GS-side OSD (PixelPilot AIOWidget / custom message).
- Fixing the drone-side `lastCbr` premature-read (tracked separately; this design
  simply avoids relying on it).
- Any change to the GS beamforming feature.
