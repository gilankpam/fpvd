# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

FPV drone link stack with two cooperating daemons, each the single supervisor + HTTP API (`:8080`) for its device:

- `drone/` — **fpvd**, C++17, runs on the OpenIPC ssc338q camera. Supervises waybeam (encoder), wfb-ng (radio), msposd/mavfwd; owns one unified config (defaults + sparse overlay, deep-merged) exposed as `GET/PATCH /config` → `POST /apply` (restarts only affected children). The encoder's error-resilience profile is a single operator knob, `video.resilience` (waybeam preset; restart-class, never DL-locked).
- `gs/` — **fpvdgs**, Python ≥3.11, runs on the ground station. Supervises the wfb data plane (built on the `wfb_ng` library), pixelpilot (display), the dynamic-link controller, the probe receiver, the beamforming armer, and the IDR relay.

## Commands

### Drone (C++, doctest)

```sh
cmake -S drone -B drone/build -DCMAKE_BUILD_TYPE=Debug
cmake --build drone/build -j
cd drone && ./build/fpvd_tests                       # run ALL tests from drone/
./build/fpvd_tests --test-case='*watchdog*'          # single test case (doctest filter)
```

**Do NOT use `ctest`** — it runs from `build/` and `test_daemon.cpp` copies a fixture via a path relative to `drone/`, producing false failures. Host builds need no nix-shell; the cross/deploy build does:

```sh
# cross-compile for the camera (armv7l musl, static) — inside drone/ nix-shell
cmake -S drone -B drone/build/ssc338q -DCMAKE_TOOLCHAIN_FILE=drone/cmake/toolchain-ssc338q.cmake
cmake --build drone/build/ssc338q --target fpvd -j
```

### GS (Python, pytest)

```sh
cd gs && .venv/bin/python -m pytest tests/ -q                          # full suite
cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy.py -q    # one file
cd gs && .venv/bin/python -m pytest tests/unit/test_dl_policy.py::test_name -q
```

### Deploy / rollback (live hardware)

```sh
./deploy/drone/deploy.sh [--host IP]    # env override: DRONE_HOST
./deploy/gs/deploy.sh    [--host IP]    # env override: GS_HOST
./deploy/gs/rollback.sh  [--host IP]
```

Deploy gotchas (both learned the hard way):
- Any **new `fpvdgs/` subpackage must be added to `deploy/gs/deploy.sh`'s scp list** or fpvd ImportError-crashes on the GS and video drops.
- The init-script restart race can leave a stale pidfile; recovery: `rm -f /var/run/fpvd.pid; /etc/init.d/S99fpvd start` on the device.
- A drone redeploy briefly drops the video source, which can make this GS reboot once (it reboots on sustained video loss). GS-only redeploys don't.

## Architecture: the dynamic link control loop

The adaptive link spans both daemons; this is the part that requires reading multiple files to understand.

**GS decides MCS; the drone derives everything else from it.** The wire (v3) is a `{mcs}`-only UDP decision packet, GS → drone `:9999` (`gs/fpvdgs/dynlink/wire.py` ↔ `drone/src/dynlink/wire.hpp`, golden-hex tested byte-identical on both sides). There is no HELLO/handshake (README's HELLO references are stale); the drone watchdog + dedup handle readiness, and on decision loss the drone falls back to a derived MCS-0 failsafe rung.

GS side (`gs/fpvdgs/dynlink/`): the wfb stats feed (`:8103`, video stream records only) → `SignalAggregator` (`signals.py`, EWMAs; **EIRP-normalizes RSSI per-window by the received MCS** before smoothing) → `Policy.tick` (`policy.py`) → wire encode → return link. The selector (`LeadingSelector`) is **probe-driven promote + reactive demote**: promote only when the `current+1` probe rung reads clean+fresh for N consecutive ticks; demote immediately on emergency (loss / FEC pressure / sustained starvation) or video-PER breach. A learned per-card RSSI→ceiling prior (`learned_prior.py`, persisted at `/etc/fpvd/learned/<profile-name>.json` on the GS) adds a one-shot warm-start and a debounced predictive demote; the probe stays authoritative for promotes.

Probe (`gs/fpvdgs/probe/` ↔ `drone/src/probe/`): one observe-only FEC-off `wfb_tx`/`wfb_rx` pair on radio_port 50, retuned live to ride one rung **above** the operating MCS. Caveat: `iw` TX power is per-netdev and follows the *operating* rung, so the probe measures rung `op+1` at rung `op`'s power — exact in the flat top of the curve, ~4 dB optimistic at the 3→4 boundary (known oscillation driver; see flight logs).

Drone side (`drone/src/dynlink/`): on each decision, `applyLocalCompute` (`local_compute.cpp`) computes bitrate (OpenIPC rate table × airtime factor × k/n), FEC k/n, and **per-MCS TX power** from the anti-overdrive curve in `txpower_curve.hpp` `{29,28,25,23,19,19,19,19}`; `dispatchTxApply` pushes wfb radiotap flags, the probe retune, and `iw` power together.

**Cross-cutting coupling:** the GS `RssiNormConfig.tx_power_dbm_by_mcs` (`gs/fpvdgs/dynlink/signals.py`) MUST mirror the drone curve in `drone/src/dynlink/txpower_curve.hpp` — both are static calibration constants for the same card.

While `dynamicLink.enabled`, the drone API **locks** the fields the controller mutates (`link.mcs`, `link.txpower`, `link.fec.k`/`link.fec.n`, `link.width`, `video.bitrate`, …) — `PATCH` returns `400 dynamic_link_locked` (`drone/src/config/lock.cpp`). All `dynamicLink.*` knobs hot-apply without bouncing wfb/waybeam.

Flight logs: one JSONL per flight at `/media/dvr/log/dynamic-link/` on the GS (rotated, size-capped, rolls on a >15 s link gap); per-tick records include MCS-change `reason`, probe per-rung PER, predictive-demote inputs. Analyze offline with `gs/tools/flightlog_analyze.py <file>.jsonl [--plot out.png]`.

## API shape (GS is the front door)

GS-local routes live under `/gs/*`; `/air/*` is an opaque proxy to the drone fpvd. The GS never pushes config to the drone — **the client orchestrates cross-device changes**, applying the drone first on channel/width moves so the GS retunes onto the link the drone already moved to. TX power is dBm everywhere (`link.txPowerDbm`).

## Conventions

- **Design-first:** features get a dated design spec in `docs/superpowers/specs/` (and often a plan in `docs/superpowers/plans/`) before implementation. Check there for the rationale behind any subsystem; specs are point-in-time and may describe superseded states.
- TDD throughout: drone tests in `drone/tests/` (doctest), GS in `gs/tests/unit/` (pytest). The GS suite must stay green as a whole — config_build/import coupling means partial refactors go red.
- `tools/probe-mvp/` is a throwaway rig (git-ignored contents); `gs/tools/` are dev-machine analysis tools, not deployed.
- Never commit local tooling dirs (`.claude/`, `.codegraph/`, `.gemini/`, `.mcp.json`, `opencode.jsonc`).
