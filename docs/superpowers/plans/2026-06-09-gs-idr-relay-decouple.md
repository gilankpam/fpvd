# GS IDR-Relay Decouple (Plan 2B of 3) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Lift the PixelPilot→drone IDR/keyframe relay out of the adaptive-link controller's run loop into a standalone, always-on GS component so keyframe forwarding works on **static *and* adaptive** links.

**Architecture:** This is sub-plan **2B of 3** of the unified-config GS work. Today the relay (`_IdrRelay`, an `asyncio.DatagramProtocol`) is created inside `DynamicLinkController._run()` and therefore only runs while the adaptive-link controller is running (`dynamicLink.enabled`). We extract it into a new top-level component `gs/fpvdgs/idr_relay.py` (`IdrRelay`: owns a daemon thread + asyncio loop, binds `0.0.0.0:11223`, forwards to `<droneHost>:11223`), wire it always-on in the supervisor (started unconditionally in `App.start`, stopped in `App.shutdown`), surface it in `/status.idrRelay`, and remove the relay from the controller. The port is hardcoded `11223`; `idrForward`/`idrPort` were already dropped from config in 2A — only the controller still reads them (with defaults), and that read goes away here.

**Tech Stack:** Python 3, stdlib `asyncio` + `threading` + `socket`, pytest. No mocking framework — tests use fakes and real ephemeral UDP sockets.

**Build & test (from `gs/`):** `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest`. Single test: `.venv/bin/python -m pytest tests/unit/test_idr_relay.py::<name>`. Keep the suite green at the end of every task (baseline: 317 passing).

**Spec:** `docs/superpowers/specs/2026-06-09-unified-config-design.md` — section "The IDR/keyframe relay … becomes always-on infrastructure": listen `0.0.0.0:11223`, forward to `droneLink.endpoint` host:`11223` (hardcoded port), decoupled from the controller, harmless when the drone is unreachable, `idrForward`/`idrPort` dropped.

---

## Why `0.0.0.0` (INADDR_ANY) matters — carry this comment over

The listen address MUST be `0.0.0.0`, never `127.0.0.1`: the relay reuses the *same* socket to forward each token on to the (non-loopback) drone. A socket bound to `127.0.0.1` cannot send off-loopback — the `sendto()` fails with `EINVAL`, which the relay swallows, so every IDR request gets silently dropped. `INADDR_ANY` still accepts the player's loopback tokens and lets the kernel pick the source for the drone route.

---

## Current state (what exists today)

- `gs/fpvdgs/dynlink/controller.py`:
  - `class _IdrRelay(asyncio.DatagramProtocol)` (≈ lines 25-46) — the forwarding protocol.
  - In `DynamicLinkController.__init__`, the status dict seeds `"idrListen": None` (≈ line 64).
  - In `_run()` (≈ lines 143-164): conditional `if snap.get("idrForward", True):` block that `create_datagram_endpoint`s `_IdrRelay` on `0.0.0.0:idrPort` and records `idrListen`. Teardown closes `idr_transport` (≈ lines 202-205).
- `gs/fpvdgs/dynlink/config_build.py` `make_dl_snapshot` already does NOT carry `idrForward`/`idrPort`. It resolves the drone host inline via `urlparse`.
- `gs/fpvdgs/supervisor.py`:
  - `App.__init__(self, store, runner, http_server, api, dynlink, pixelpilot=None, probe=None, armer=None)`.
  - `App.start()` / `App.shutdown()` manage runner/armer/pixelpilot/dynlink/probe.
  - `build_app(...)` constructs the components and the `status_fn`.
- `gs/fpvdgs/status.py` `build_status(...)` takes named kwargs (`dynamic_link`, `pixelpilot`, `probe`, `beamforming`) and emits the corresponding status keys.
- Tests: `gs/tests/unit/test_dl_controller.py` has `test_idr_relay_binds_inaddr_any_so_it_can_forward_off_loopback` (≈ line 224) and a `_snapshot`/`_DEFAULT` helper that passes `idrForward: False` (≈ line 15) so unrelated controller tests don't contend for `:11223`.

## File map

| File | Change |
|---|---|
| `gs/fpvdgs/idr_relay.py` | **NEW** — `_IdrRelay` protocol (moved) + `IdrRelay` always-on owner (thread + asyncio loop, bind `0.0.0.0:11223`, forward to `(droneHost, 11223)`, `start`/`stop`/`status`) |
| `gs/tests/unit/test_idr_relay.py` | **NEW** — unit-test the protocol forward/swallow + the owner lifecycle/bind |
| `gs/fpvdgs/dynlink/config_build.py` | add `drone_host_from_endpoint(endpoint, default)` helper; refactor `make_dl_snapshot` to use it |
| `gs/fpvdgs/supervisor.py` | construct + always-on wire `IdrRelay` (`build_app` gains injectable `idr_relay=None`; `App` gains `idr_relay`; `start`/`shutdown` + `status_fn`) |
| `gs/fpvdgs/status.py` | `build_status` gains `idr_relay=None` → emits `out["idrRelay"]` |
| `gs/fpvdgs/dynlink/controller.py` | REMOVE the `_IdrRelay` class, the `_run` relay block, and the `idrListen` status field |
| `gs/tests/unit/test_app_wiring.py` | add: relay starts unconditionally (even when `dynamicLink` disabled) |
| `gs/tests/unit/test_status.py` | assert `idrRelay` surfaces in status |
| `gs/tests/unit/test_dl_controller.py` | remove the in-controller IDR test + the `idrForward`/`idrListen` references |
| `gs/tests/integration/test_supervisor_e2e.py` | inject a fake relay so e2e doesn't bind `:11223`; tolerate the additive `idrRelay` status key |
| `docs/api.md` | document the always-on relay + `/status.idrRelay`; flip "is becoming" → "is" |

---

## Task 1: Standalone `IdrRelay` component

**Files:**
- Create: `gs/fpvdgs/idr_relay.py`
- Create: `gs/tests/unit/test_idr_relay.py`

- [ ] **Step 1: Write the failing tests** — create `gs/tests/unit/test_idr_relay.py`:

```python
import socket


def _free_udp_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(("127.0.0.1", 0))
        return s.getsockname()[1]
    finally:
        s.close()


def test_relay_forwards_each_datagram_to_dest():
    from fpvdgs.idr_relay import _IdrRelay
    sent = []

    class FakeTransport:
        def sendto(self, data, dest):
            sent.append((data, dest))

    r = _IdrRelay(("10.0.0.9", 11223))
    r.connection_made(FakeTransport())
    r.datagram_received(b"idr-token", ("127.0.0.1", 5000))
    assert sent == [(b"idr-token", ("10.0.0.9", 11223))]


def test_relay_swallows_oserror_when_drone_unreachable():
    from fpvdgs.idr_relay import _IdrRelay

    class BoomTransport:
        def sendto(self, data, dest):
            raise OSError("network unreachable")

    r = _IdrRelay(("10.0.0.9", 11223))
    r.connection_made(BoomTransport())
    r.datagram_received(b"idr-token", ("127.0.0.1", 5000))  # must not raise


def test_idr_relay_starts_binds_inaddr_any_and_stops():
    from fpvdgs.idr_relay import IdrRelay
    port = _free_udp_port()
    r = IdrRelay("10.255.255.1", port=port)
    r.start()
    try:
        st = r.status()
        assert st["running"] is True
        assert st["listen"] == "0.0.0.0:%d" % port
    finally:
        r.stop()
    assert r.status()["running"] is False


def test_idr_relay_start_is_idempotent():
    from fpvdgs.idr_relay import IdrRelay
    port = _free_udp_port()
    r = IdrRelay("10.255.255.1", port=port)
    r.start()
    r.start()  # second call is a no-op, must not raise or double-bind
    try:
        assert r.status()["running"] is True
    finally:
        r.stop()
```

- [ ] **Step 2: Run to verify they fail**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_idr_relay.py -q`
Expected: FAIL (`ModuleNotFoundError: fpvdgs.idr_relay`).

- [ ] **Step 3: Create `gs/fpvdgs/idr_relay.py`**

```python
"""Always-on IDR/keyframe relay: PixelPilot -> drone encoder.

PixelPilot sends IDR (keyframe) request tokens to the GS on 0.0.0.0:11223;
this relay forwards each datagram over the tunnel to the drone's idr_listen at
<droneHost>:11223. It is standing GS data-plane infrastructure, decoupled from
the adaptive-link controller, so keyframe forwarding works on static *and*
adaptive links. Replaces the standalone socat idr-forwarder that shipped with
the old dynamic-link-gs service.

The listen address MUST be 0.0.0.0 (INADDR_ANY), never 127.0.0.1: the same
socket is reused to forward each token to the (non-loopback) drone, and a socket
bound to 127.0.0.1 cannot send off-loopback (sendto fails EINVAL, which the
relay swallows -> every IDR request silently dropped). INADDR_ANY still accepts
the player's loopback tokens and lets the kernel pick the drone-route source."""
from __future__ import annotations

import asyncio
import logging
import threading

log = logging.getLogger("fpvdgs.idr_relay")

IDR_PORT = 11223


class _IdrRelay(asyncio.DatagramProtocol):
    """Forward every received datagram to a fixed drone destination."""

    def __init__(self, dest):
        self._dest = dest          # (droneHost, IDR_PORT)
        self._transport = None

    def connection_made(self, transport):
        self._transport = transport

    def datagram_received(self, data, addr):
        if self._transport is not None:
            try:
                self._transport.sendto(data, self._dest)
            except OSError:
                pass               # drone momentarily unreachable — drop, keep relaying


class IdrRelay:
    """Always-on owner of the IDR relay: a daemon thread running an asyncio loop
    that binds 0.0.0.0:<port> and forwards to (drone_host, <port>) until stopped.
    Thread-safe start/stop/status for the (thread-based) supervisor."""

    def __init__(self, drone_host, *, port=IDR_PORT):
        self._dest = (drone_host, port)
        self._port = port
        self._lock = threading.RLock()
        self._thread = None
        self._loop = None
        self._stop_event = None          # asyncio.Event, created in-loop
        self._started = threading.Event()
        self._status = {"running": False, "listen": None}

    # ---- thread-safe public API -----------------------------------------
    def start(self):
        with self._lock:
            if self._thread and self._thread.is_alive():
                return
            self._started.clear()
            self._thread = threading.Thread(target=self._thread_main,
                                            name="idr-relay", daemon=True)
            self._thread.start()
        self._started.wait(timeout=5.0)

    def stop(self):
        with self._lock:
            loop, stop, thread = self._loop, self._stop_event, self._thread
        if loop is not None and stop is not None:
            try:
                loop.call_soon_threadsafe(stop.set)
            except RuntimeError:
                pass  # loop already closed/closing
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout=5.0)
        with self._lock:
            self._thread = None

    def status(self):
        with self._lock:
            return dict(self._status)

    # ---- internals ------------------------------------------------------
    def _thread_main(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        with self._lock:
            self._loop = loop
        try:
            loop.run_until_complete(self._run())
        except Exception:
            log.exception("idr-relay loop crashed")
        finally:
            try:
                loop.close()
            finally:
                with self._lock:
                    self._loop = None
                    self._status.update(running=False, listen=None)
                self._started.set()   # unblock start() even on early failure

    async def _run(self):
        self._stop_event = asyncio.Event()
        transport = None
        try:
            transport, _ = await asyncio.get_running_loop().create_datagram_endpoint(
                lambda: _IdrRelay(self._dest), local_addr=("0.0.0.0", self._port))
            sa = transport.get_extra_info("sockname")
            with self._lock:
                self._status.update(
                    running=True,
                    listen="%s:%d" % (sa[0], sa[1]) if sa else None)
        except OSError as e:
            log.warning("idr-relay bind 0.0.0.0:%d failed: %s", self._port, e)
            with self._lock:
                self._status.update(running=False, listen=None)
        self._started.set()
        try:
            await self._stop_event.wait()
        finally:
            if transport is not None:
                transport.close()
            with self._lock:
                self._status.update(running=False, listen=None)
```

- [ ] **Step 4: Run to verify they pass**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_idr_relay.py -q`
Expected: PASS (4 tests).

- [ ] **Step 5: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/idr_relay.py gs/tests/unit/test_idr_relay.py
git commit -m "gs: standalone always-on IdrRelay component (extracted from controller)"
```

---

## Task 2: Wire the relay always-on into the supervisor + status

After this task, the standalone relay forwards keyframes unconditionally. The controller still has its own (now redundant) relay until Task 3 — that is **safe**: the always-on relay binds `0.0.0.0:11223` at app start, so the controller's later `create_datagram_endpoint` on the same port fails `EADDRINUSE`, which the controller already swallows (`idr_transport = None`). Forwarding works via the always-on relay throughout.

**Files:**
- Modify: `gs/fpvdgs/dynlink/config_build.py`
- Modify: `gs/fpvdgs/supervisor.py`
- Modify: `gs/fpvdgs/status.py`
- Test: `gs/tests/unit/test_app_wiring.py`, `gs/tests/unit/test_status.py`, `gs/tests/integration/test_supervisor_e2e.py`

- [ ] **Step 1: Add the drone-host helper** — in `gs/fpvdgs/dynlink/config_build.py`, add a small helper next to `make_dl_snapshot` (the file already imports `urlparse`):

```python
def drone_host_from_endpoint(endpoint, default="10.5.0.10"):
    """Hostname of an http://host:port endpoint, or `default` if unparseable."""
    return urlparse(endpoint).hostname or default
```

Then refactor `make_dl_snapshot` to reuse it — replace its inline host resolve:

```python
    endpoint = effective.get("droneLink", {}).get("endpoint", "http://10.5.0.10:8080")
    host = drone_host_from_endpoint(endpoint)
    block["droneAddr"] = block.get("droneAddr") or host
```

(`make_dl_snapshot`'s behavior is unchanged — same default, same result. The existing `test_dl_config_build.py` snapshot tests must still pass untouched.)

- [ ] **Step 2: Write the failing wiring test** — add to `gs/tests/unit/test_app_wiring.py` (reuses the `_Fake` + `ConfigStore` already imported at the top of the file):

```python
def test_app_starts_idr_relay_always_even_when_dynamiclink_disabled():
    # The IDR relay is standing infra: it must start regardless of
    # dynamicLink.enabled (it serves static links too) and stop on shutdown.
    store = ConfigStore({"pixelpilot": {"enabled": False},
                         "dynamicLink": {"enabled": False}})
    idr = _Fake("idr")
    app = App(store, _Fake("runner"), _Fake("http"), api=None,
              dynlink=_Fake("dynlink"), idr_relay=idr)
    app.start()
    assert "start" in idr.calls
    app.shutdown()
    assert "stop" in idr.calls
```

- [ ] **Step 3: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_app_wiring.py -k idr_relay -q`
Expected: FAIL (`App.__init__` has no `idr_relay`).

- [ ] **Step 4: Wire `idr_relay` into `App`** — in `gs/fpvdgs/supervisor.py`, extend the constructor and lifecycle. Constructor:

```python
    def __init__(self, store, runner, http_server, api, dynlink,
                 pixelpilot=None, probe=None, armer=None, idr_relay=None):
        self.store = store
        self.runner = runner
        self.http = http_server
        self.api = api
        self.dynlink = dynlink
        self.pixelpilot = pixelpilot
        self.probe = probe
        self.armer = armer
        self.idr_relay = idr_relay
```

In `start()`, start the relay unconditionally (add after `self.runner.start()`):

```python
        if self.idr_relay is not None:
            self.idr_relay.start()   # always-on: keyframe relay serves static + adaptive links
```

In `shutdown()`, stop it (add alongside the other stops, before `self.runner.shutdown()`):

```python
        if self.idr_relay is not None:
            self.idr_relay.stop()
```

- [ ] **Step 5: Run the wiring test to verify it passes**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_app_wiring.py -q`
Expected: PASS (existing wiring tests still green; new one passes).

- [ ] **Step 6: Construct + status-wire the relay in `build_app`** — in `gs/fpvdgs/supervisor.py`:

Add the import near the other dynlink imports:

```python
from .idr_relay import IdrRelay
from .dynlink.config_build import make_dl_snapshot, drone_host_from_endpoint
```

(adjust the existing `from .dynlink.config_build import make_dl_snapshot` line to also import `drone_host_from_endpoint`.)

Change `build_app`'s signature to accept an injectable relay (mirrors `probe_spawn`):

```python
def build_app(defaults_path, overlay_path, cfg_out, host, port,
              runner_cmd, ready_port=8103, ready_timeout=10.0, log_path=None,
              probe_spawn=None, idr_relay=None):
```

After the `drone = DroneClient(...)` line, construct the relay if not injected:

```python
    if idr_relay is None:
        endpoint = effective.get("droneLink", {}).get("endpoint", "http://10.5.0.10:8080")
        idr_relay = IdrRelay(drone_host_from_endpoint(endpoint))
```

In `status_fn`, pass the relay status to `build_status`:

```python
                                       beamforming=beamforming.status(),
                                       idr_relay=idr_relay.status())
```

And pass it to the `App(...)` construction at the end:

```python
    return App(store, runner, http_server, api, dynlink,
               pixelpilot=pixelpilot, probe=probe_ctrl, armer=armer,
               idr_relay=idr_relay)
```

- [ ] **Step 7: Write the failing status test** — add to `gs/tests/unit/test_status.py`:

```python
def test_build_status_emits_idr_relay():
    from fpvdgs.status import build_status
    out = build_status("v", {}, {}, {"reachable": True},
                       idr_relay={"running": True, "listen": "0.0.0.0:11223"})
    assert out["idrRelay"] == {"running": True, "listen": "0.0.0.0:11223"}
```

- [ ] **Step 8: Run to verify it fails**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_status.py -k idr_relay -q`
Expected: FAIL (`build_status` has no `idr_relay`).

- [ ] **Step 9: Emit `idrRelay` from `build_status`** — in `gs/fpvdgs/status.py`, add the kwarg to `build_status`'s signature:

```python
                 beamforming: dict | None = None,
                 idr_relay: dict | None = None) -> dict:
```

and, next to where the other optional blocks are appended (e.g. after the `beamforming` emit), add:

```python
    if idr_relay is not None:
        out["idrRelay"] = idr_relay
```

- [ ] **Step 10: Keep the e2e suite from binding :11223** — in `gs/tests/integration/test_supervisor_e2e.py`, find each `build_app(...)` call and pass a lightweight fake so the e2e never opens a real `:11223` socket. Add a fake near the top of the file:

```python
class _FakeIdrRelay:
    def __init__(self):
        self.started = self.stopped = 0
    def start(self):
        self.started += 1
    def stop(self):
        self.stopped += 1
    def status(self):
        return {"running": self.started > self.stopped, "listen": "0.0.0.0:11223"}
```

and pass `idr_relay=_FakeIdrRelay()` to each `build_app(...)` call in that file. (The `idrRelay` status key is additive — existing status assertions still hold. If a test asserts the full set of top-level status keys, add `"idrRelay"` to its expected set.)

- [ ] **Step 11: Run the full suite**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest -q`
Expected: PASS (all). The controller's own relay is still present but harmlessly loses the `:11223` bind race to the always-on relay.

- [ ] **Step 12: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/supervisor.py gs/fpvdgs/status.py gs/fpvdgs/dynlink/config_build.py gs/tests/
git commit -m "gs: wire always-on IdrRelay into supervisor + /status.idrRelay"
```

---

## Task 3: Remove the relay from the controller

**Files:**
- Modify: `gs/fpvdgs/dynlink/controller.py`
- Test: `gs/tests/unit/test_dl_controller.py`
- Modify: `docs/api.md`

- [ ] **Step 1: Update the controller tests first (TDD: they should fail because the controller still binds)** — in `gs/tests/unit/test_dl_controller.py`:
  - **Delete** `test_idr_relay_binds_inaddr_any_so_it_can_forward_off_loopback` (≈ line 224 to the end of that function) — that behavior now lives in `test_idr_relay.py`.
  - In the `_snapshot`/default-snapshot helper, **remove** the `"idrForward": False` entry (≈ line 15) and the comment above it about contending for `:11223` (the controller no longer touches `:11223` at all).
  - Remove any remaining `idrForward` / `idrPort` / `idrListen` references in this file. Verify: `grep -ni idr gs/tests/unit/test_dl_controller.py` → expect none.

- [ ] **Step 2: Run to verify the relevant tests fail or the file errors**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest tests/unit/test_dl_controller.py -q`
Expected: the remaining controller tests still PASS (removing the IDR test + the `idrForward:False` default does not change non-IDR behavior). This step confirms the test file is clean before gutting the controller. (If a non-IDR test now contends for `:11223`, that proves the controller still binds — which Step 3 fixes.)

- [ ] **Step 3: Strip the relay out of `gs/fpvdgs/dynlink/controller.py`**:
  - Delete the `class _IdrRelay(asyncio.DatagramProtocol):` definition (≈ lines 25-46).
  - In `DynamicLinkController.__init__`, remove `"idrListen": None` from the `self._status` seed dict (≈ line 64).
  - In `_run()`, delete the entire IDR block — the long `# IDR-token relay …` / `# The listen address MUST be 0.0.0.0 …` comment, `idr_transport = None`, `self._set(idrListen=None)`, and the whole `if snap.get("idrForward", True):` try/except (≈ lines 143-164).
  - In `_run()`'s `finally:` teardown, remove `if idr_transport is not None: idr_transport.close()` and drop `idrListen=None` from the final `self._set(running=False, statsConnected=False, idrListen=None)` → `self._set(running=False, statsConnected=False)` (≈ lines 202-205).
  - The module docstring (≈ lines 2-8) describes the loop as "stats client → SignalAggregator → Policy → wire encode → ReturnLink" — it does not mention IDR, so no change needed there. Verify nothing else references `_IdrRelay`/`idr`: `grep -ni idr gs/fpvdgs/dynlink/controller.py` → expect none.

- [ ] **Step 4: Run the controller + full suite**

Run: `cd /home/gilankpam/Projects/drone/fpvd/gs && .venv/bin/python -m pytest -q`
Expected: PASS (all). The controller no longer binds `:11223`; the always-on relay (Task 2) is the sole forwarder.

- [ ] **Step 5: Update `docs/api.md`** — in the GS section:
  - The line that currently reads "… The IDR-token relay config (`idrForward`/`idrPort`) has been removed from the config — the relay is **becoming** always-on GS infra." → change to past/active tense: "… the relay **is** always-on GS infrastructure: it listens on `0.0.0.0:11223` and forwards keyframe tokens to the drone at `droneLink.endpoint` host:`11223`, independent of `dynamicLink.enabled` (so it serves static links too)."
  - Where `GET /status` is documented for the GS, note the new `idrRelay` sub-object: `{ "running": bool, "listen": "0.0.0.0:11223" | null }`. If the old GS `/status.dynamicLink` documented an `idrListen` field, remove it (it moved to `/status.idrRelay.listen`).
  - Do NOT touch the drone-side `minIdrIntervalMs` field (unrelated encoder knob).

- [ ] **Step 6: Final sweep + full suite**

Run:
```bash
cd /home/gilankpam/Projects/drone/fpvd
grep -rn -i 'idrforward\|idrport\|idrlisten\|_idrrelay' gs/fpvdgs gs/tests | grep -v idr_relay.py
cd gs && .venv/bin/python -m pytest -q
```
Expected: the grep returns nothing (every IDR reference now lives in `idr_relay.py` / `test_idr_relay.py`); the suite is PASS (all).

- [ ] **Step 7: Commit**

```bash
cd /home/gilankpam/Projects/drone/fpvd
git add gs/fpvdgs/dynlink/controller.py gs/tests/unit/test_dl_controller.py docs/api.md
git commit -m "gs: remove the IDR relay from the dynamic-link controller (now always-on infra)"
```

---

## Done criteria

- `cd gs && .venv/bin/python -m pytest` is green.
- `gs/fpvdgs/idr_relay.py` owns the relay; it is constructed in `build_app` and started **unconditionally** in `App.start` / stopped in `App.shutdown`, independent of `dynamicLink.enabled`.
- `DynamicLinkController` no longer references `_IdrRelay`, `idrForward`, `idrPort`, or `idrListen`; it never binds `:11223`.
- `GET /status` carries `idrRelay: { running, listen }`; `idrListen` is gone from the dynamicLink status.
- No `idrForward`/`idrPort`/`idrListen`/`_idrrelay` references remain outside `idr_relay.py` + `test_idr_relay.py`.
- The relay forwards `0.0.0.0:11223` → `<droneHost>:11223`, harmless when the drone is unreachable.
