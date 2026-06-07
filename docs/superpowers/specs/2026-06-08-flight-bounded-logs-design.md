# Flight-Bounded Flight Logs (link-gap roll) Design

**Date:** 2026-06-08
**Status:** Draft for review
**Target:** GS `fpvdgs.dynlink` (`flightlog.py` + `policy.py`)
**Refines:** Phase 4 (`2026-06-07-phase4-learned-rssi-prior-design.md` §8) — the per-session JSONL flight logger.

---

## 1. Purpose & scope

Today a flight-log file spans one **controller-run session** — from `dynamicLink` enable until disable / fpvd restart. The operator leaves `dynamicLink` enabled across an entire field session (multiple flights, landings, battery swaps), so a single file ends up spanning **many flights**. This change segments the log **per flight** by rolling a new file when the video link has been gone long enough to mean "landed / powered-off / out of range," and relocates the logs to DVR storage.

**In scope:** roll a new flight-log file when the link returns after an absence longer than `flight_gap_s`; move the log directory to `/media/dvr/log/dynamic-link/`.

**Out of scope:** merging config-restarts within a flight (the operator doesn't toggle mid-flight); any external flight-controller / MAVLink / GPS signal (the GS doesn't ingest it); the learned-prior curve (unchanged — see §5).

## 2. Locked decisions (from brainstorming)

- **Boundary = link-gap heuristic, on-device.** A new flight = the video link returns *healthy* after being absent (`link_starved`) for longer than `flight_gap_s`. Derived purely from signals the GS already has (`link_starved_w` + a monotonic clock); no new hardware/telemetry.
- **`flight_gap_s` default = 15 s** (configurable). Long enough to ride out a brief obstruction/dropout, short enough to catch a landing/battery swap. Tradeoff: a >15 s mid-air signal loss splits one flight into two files (acceptable per operator).
- **Logs → `/media/dvr/log/dynamic-link/`** (DVR SD card), not the `/etc/fpvd` overlay. More space, no flash-wear, co-located with DVR recordings. Removable: if unmounted, logging no-ops gracefully (§6).
- **Split only, not merge.** The roll happens on gap recovery; config-restarts/reboots simply start a fresh file (a reboot is a genuine discontinuity → new flight).
- **The learned curve is untouched** — it stays on `/etc/fpvd/learned/` and accumulates per-card across flights; only the *log file* segments.

## 3. Detection mechanism

The GS aggregator already computes per-window `link_starved_w` (`session is not None and packet_rate_w < starvation_threshold_pps`) — true when the drone isn't transmitting video, false on healthy video. `Policy` tracks the **monotonic** timestamp of the last *healthy* (non-starved) tick:

```
_last_healthy_mono = None        # at Policy init

on tick(signals):
    healthy = not signals.link_starved_w
    if healthy:
        now = time.monotonic()
        if _last_healthy_mono is not None and (now - _last_healthy_mono) > flight_gap_s:
            flightlog.roll()      # close current file, open a new <start_ms>.jsonl
        _last_healthy_mono = now
    # starved: leave _last_healthy_mono frozen so the gap accumulates
    ... (existing tick logic, then flightlog.write(record))
```

- Rolls **exactly once**, on the first healthy tick after a gap > `flight_gap_s`.
- `None` init means the **first** healthy tick never rolls (it just sets the baseline) — so a fresh session that starts while the link is still warming up doesn't spuriously roll; the file the controller already opened on start is the first flight.
- **Monotonic** time (not the unreliable GS wall-clock) measures the real elapsed gap.
- Robust whether, during the gap, the stats feed emits zero-count windows (starved ticks → `_last_healthy_mono` frozen) **or** goes fully silent (no ticks → on the next healthy tick the monotonic delta is still > `flight_gap_s`). Either way the first healthy tick after the gap rolls.
- A brief flicker (1–few starved windows, well under 15 s) never rolls (gap stays tiny).

The detection lives in `Policy.tick` (it already owns `link_starved` + timing); `FlightLog` stays a dumb sink that gains one method, `roll()`.

## 4. `FlightLog.roll()`

```python
def roll(self) -> None:
    """End the current flight file and begin a new one. No-op if disabled."""
    # close the current handle (flushes), then reopen a fresh <ms>.jsonl,
    # reset the byte counter, and prune to max_files (now == max flights).
```

- The new filename is a fresh monotonic-ms stamp (unique, ordered within a boot), same scheme as the constructor.
- On roll, `_prune()` runs → **rotation now retains the last `max_files` flights** (a useful side effect of per-flight files).
- If the logger is disabled (`_fh is None`), `roll()` is a no-op.
- The per-file **size cap (`max_mb`) is unchanged**: a flight file caps at `max_mb` (~50 min at 10 Hz / 4 MB) — ample for one flight; a pathologically long flight truncates rather than rolling (documented; not split mid-flight).

## 5. The learned curve is unaffected

`LearnedPrior` persistence stays at `/etc/fpvd/learned/<profile.name>.json` (the reliable overlay). The curve is **cumulative across flights** — flights don't reset or segment it; warm-start must work even with no SD card present. Only the flight *log* moves to DVR and segments per flight.

## 6. Config delta & failure modes

- **Config (under `tuning.learned_prior.flightlog`):** `dir` default changes to `/media/dvr/log/dynamic-link/`; add `flight_gap_s` (default `15.0`). `enabled`/`max_files`/`max_mb` unchanged.
- **`/media/dvr` unmounted / no SD card:** `os.makedirs` / `open` raises `OSError` → `FlightLog` logs a warning and sets `_fh = None` → all `write`/`roll` calls no-op. The link, the selector, and the learned curve are entirely unaffected (logging is observability only). This is the existing `FlightLog` open-failure path — the directory change just makes it more likely to be exercised.
- **`flight_gap_s` measured in monotonic time** → unaffected by the GS clock being wrong or jumping.

## 7. Testing

- **Unit (`FlightLog.roll`):** roll closes the current file and opens a new one (two files, second is newest); roll prunes to `max_files`; roll on a disabled logger is a no-op; the new file keeps appending.
- **Unit / integration (`Policy` gap detection):** a sequence healthy… → starved for > `flight_gap_s` (simulated via monotonic, or by injecting `link_starved_w=True` ticks with advancing timestamps) → healthy **rolls a new flight file**; a brief starved blip (< `flight_gap_s`) does **not** roll; the first healthy tick of a fresh `Policy` does **not** roll (`None` baseline). Drive monotonic deterministically (inject a clock or monkeypatch `time.monotonic`).
- **Config:** `flightlog.dir` defaults to `/media/dvr/log/dynamic-link/`; `flight_gap_s` defaults to 15.0 and parses overrides.
- **Regression:** the existing Phase-4 flightlog + policy tests stay green; with the link continuously healthy, behavior is one file per session as before (no spurious rolls).

## 8. Self-review

- **Placeholders:** none — mechanism, `roll()` contract, config defaults, and the `None`-baseline rule are all concrete.
- **Consistency:** detection in `Policy.tick` (has `link_starved_w` + monotonic) ↔ `FlightLog.roll()` sink; `flight_gap_s`/`dir` live under `tuning.learned_prior.flightlog` next to the existing flightlog knobs; the learned-curve path is explicitly *not* moved.
- **Ambiguity:** "healthy" = `not link_starved_w` (raw per-window), explicit; the roll fires on recovery (first healthy tick after gap), once; monotonic time is the measure.
- **Scope:** one small change to two files + config + tests. No drone/wire impact.
