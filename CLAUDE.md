# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

FPV drone link stack with two cooperating daemons, each the single supervisor + HTTP API (`:8080`) for its device:

- `drone/` — **fpvd**, C++17, runs on the OpenIPC ssc338q camera. Supervises waybeam (encoder), wfb-ng (radio), msposd/mavfwd; owns one unified config (defaults + sparse overlay, deep-merged) exposed as `GET/PATCH /config` → `POST /apply` (restarts only affected children). The encoder's error-resilience profile is a single operator knob, `video.resilience` (waybeam preset; restart-class, never DL-locked).
- `gs/` — **fpvdgs**, Python ≥3.11, runs on the ground station. Supervises the wfb data plane (built on the `wfb_ng` library via `wfb.engine`, default `"wfbng"` = wfb-ng Python runner; `"native"` = native asyncio orchestration spawning wfb_rx/wfb_tx directly from `gs/fpvdgs/wfb/` with in-process stats + TX-card selection), pixelpilot (display), the dynamic-link controller, the beamforming armer, and the IDR relay. (The GS probe receiver is bypassed — retained in-tree, never constructed; see the probe paragraph below.)

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

### Lint (CI-gated — run before every commit)

GitHub Actions (`.github/workflows/ci.yml`) blocks merges on these as well as the
test suites, so **run both linters and fix any findings before committing** — don't
push and let CI catch it. Tools are pinned (`ruff==0.8.4`, `clang-format==19.1.4`,
installed in `gs/.venv`); use the same versions locally so formatting matches CI.

```sh
# Python (gs) — ruff config in gs/pyproject.toml
cd gs && .venv/bin/ruff check fpvdgs tests        # lint (E/F/I; E501 ignored)
cd gs && .venv/bin/ruff format fpvdgs tests       # auto-format (drop --check to fix in place)

# C++ (drone) — .clang-format at repo root
find drone/src drone/tests \( -name '*.cpp' -o -name '*.hpp' \) -print0 \
  | xargs -0 gs/.venv/bin/clang-format -i          # auto-format in place
```

CI runs `ruff format --check` and `clang-format --dry-run --Werror` (verify, don't
mutate); the commands above with `-i` / no `--check` fix the tree to match.

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
- **Deploy the forked wfb binaries BEFORE any fpvdgs deploy that has the dynlink tap** (`dynamicLink.tap.enabled` defaults true → renders `wfb_rx -D`; a pre-tap binary exits on the unknown flag → wfb crash-loop → video down → GS reboot loop). Rollback: `PATCH dynamicLink.tap.enabled=false` + apply, or deploy the fork binaries.

## Architecture: the dynamic link control loop

The adaptive link spans both daemons; this is the part that requires reading multiple files to understand.

**GS decides MCS; the drone derives everything else from it.** The wire (v3) is a `{mcs}`-only UDP decision packet, GS → drone `:9999` (`gs/fpvdgs/dynlink/wire.py` ↔ `drone/src/dynlink/wire.hpp`, golden-hex tested byte-identical on both sides). There is no HELLO/handshake (README's HELLO references are stale); the drone watchdog + dedup handle readiness, and on decision loss the drone falls back to a derived MCS-0 failsafe rung.

GS side (`gs/fpvdgs/dynlink/`): the wfb stats feed (`:8103`, video stream records only) → `SignalAggregator` (`signals.py`, EWMAs; raw per-window SNR is the sole control axis — no EIRP normalization) → `Policy.tick` (`policy.py`) → wire encode → return link. The selector (`LeadingSelector`) uses **knee-gated promote (three routes) + reactive demote with failure-signature classification**: (1) snap-back — return to a recently-confirmed rung whose operating SNR has recovered, at the fast rate limit, bypassing dwell and the knee gate; (2) knee-gated — after a clean dwell with confident SNR headroom above the learned failure knee; (3) explore — a single promote onto a rung with no learned knee (tuition; its first loss plants the knee and converts the route to knee-gated on the next attempt). Demote is one-step and cooldown-gated; every loss-demote is classified fade/flap/burst and teaches the failure knee accordingly. A **per-rung escalating flap-damper** (`flap_level`, `suppress_until_ms`) imposes an escalating back-off on promotes after a flap; it lifts early on SNR recovery above the flap level (`flap_snr_release_db`) and decays per-rung over time. A learned per-card SNR knee prior (`learned_prior.py`, persisted at `/etc/fpvd/learned/<profile-name>.json` on the GS) drives the knee-gated route and adds a debounced predictive demote.

Probe (`gs/fpvdgs/probe/` ↔ `drone/src/probe/`): the probe subsystem is **disabled by default** (`dynamicLink.probe.enabled`, default false on both daemons). When enabled, it spawns an observe-only FEC-off `wfb_tx`/`wfb_rx` pair on radio_port 50, retuned live to ride one rung above the operating MCS. The probe is no longer consulted anywhere on the GS — the knob only spawns the drone-side probe children (TX + feeder); no probe data reaches the GS policy or flight log. Leaving it disabled (the default) reclaims its airtime for the operating video stream.

Drone side (`drone/src/dynlink/`): on each decision, `applyLocalCompute` (`local_compute.cpp`) computes bitrate (OpenIPC rate table × airtime factor × k/n), FEC k/n, and **per-MCS TX power** from the anti-overdrive curve in `txpower_curve.hpp` `{29,28,25,23,19,19,19,19}`; `dispatchTxApply` pushes wfb radiotap flags and `iw` power. Probe children are only spawned when `dynamicLink.probe.enabled=true` (default false); with the default off, no probe processes run on the drone.

While `dynamicLink.enabled`, the drone API **locks** the fields the controller mutates (`link.mcs`, `link.txpower`, `link.fec.k`/`link.fec.n`, `link.width`, `video.bitrate`, …) — `PATCH` returns `400 dynamic_link_locked` (`drone/src/config/lock.cpp`). All `dynamicLink.*` knobs hot-apply without bouncing wfb/waybeam.

Flight logs: one JSONL per flight at `/media/dvr/log/dynamic-link/` on the GS (rotated, size-capped, rolls on a >15 s link gap); per-tick records include MCS-change `reason` (promote reasons: `knee_promote`, `explore_promote`, `snapback_promote`; demote reasons carry a `class=fade|flap|burst` suffix), `fail_class` (the demote failure signature), `trial` (current probation rung), `snapback_tgt`, `clean_dwell`, and predictive-demote inputs. Analyze offline with `gs/tools/flightlog_analyze.py <file>.jsonl [--plot out.png]`; replay the selector logic offline with `gs/tools/replay_flightlog.py`.

## API shape (GS is the front door)

GS-local routes live under `/gs/*`; `/air/*` is an opaque proxy to the drone fpvd. The GS never pushes config to the drone — **the client orchestrates cross-device changes**, applying the drone first on channel/width moves so the GS retunes onto the link the drone already moved to. TX power is dBm everywhere (`link.txPowerDbm`).

## Conventions

- **Design-first:** features get a dated design spec in `docs/superpowers/specs/` (and often a plan in `docs/superpowers/plans/`) before implementation. Check there for the rationale behind any subsystem; specs are point-in-time and may describe superseded states.
- TDD throughout: drone tests in `drone/tests/` (doctest), GS in `gs/tests/unit/` (pytest). The GS suite must stay green as a whole — config_build/import coupling means partial refactors go red.
- `tools/probe-mvp/` is a throwaway rig (git-ignored contents); `gs/tools/` are dev-machine analysis tools, not deployed.
- Never commit local tooling dirs (`.claude/`, `.codegraph/`, `.gemini/`, `.mcp.json`, `opencode.jsonc`).
