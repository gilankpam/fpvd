"""Tests for the asyncio wfb_rx/wfb_tx child supervisor.

The fake-child scripts + helper writers (`write_fake_rx`, `write_fake_tx`,
`make_spec`, ...) are module-level so Task 13's integration test can import
this harness instead of duplicating it.
"""

from __future__ import annotations

import asyncio
import os
import signal
import time

import pytest

from fpvdgs.wfb.children import WfbChild
from fpvdgs.wfb.graph import ServiceSpec

FAKE_RX = """#!/usr/bin/env python3
import sys, time
print("100\\tSESSION\\t1:2:60:30:3", flush=True)
while True:
    print("100\\tRX_ANT\\t5660:5:20\\t0\\t9:-50:-48:-45:26:28:30:22:24:26", flush=True)
    print("100\\tPKT\\t9:900:0:0:9:9:0:0:0:9:800", flush=True)
    time.sleep(0.05)
"""

FAKE_TX = """#!/usr/bin/env python3
import sys, time
print("100\\tLISTEN_UNIX\\tfake-tx-sock:0", flush=True)
print("100\\tLISTEN_UNIX_END", flush=True)
print("100\\tLISTEN_UDP_CONTROL\\t14100", flush=True)
while True:
    time.sleep(1)
"""

FAKE_TX_NO_HANDSHAKE = """#!/usr/bin/env python3
import time
while True:
    time.sleep(1)
"""

FAKE_INSTA_EXIT = """#!/usr/bin/env python3
import sys
sys.exit(1)
"""

# A forwarder/injector stand-in: no handshake, no IPC_MSG stats lines -- just
# noisy stdout (mirrors real wfb_rx -f / wfb_tx -I, which only ever emit
# WFB_ERR diagnostics, never a stats line a parser could recognize).
FAKE_NOISY_NO_PARSER = """#!/usr/bin/env python3
import time
while True:
    print("some diagnostic line that is not a stats record", flush=True)
    time.sleep(0.02)
"""


def run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


def _write_script(tmp_path, name: str, source: str) -> str:
    path = tmp_path / name
    path.write_text(source)
    path.chmod(0o755)
    return str(path)


def write_fake_rx(tmp_path) -> str:
    return _write_script(tmp_path, "fake_rx.py", FAKE_RX)


def write_fake_tx(tmp_path) -> str:
    return _write_script(tmp_path, "fake_tx.py", FAKE_TX)


def write_fake_tx_no_handshake(tmp_path) -> str:
    return _write_script(tmp_path, "fake_tx_no_handshake.py", FAKE_TX_NO_HANDSHAKE)


def write_fake_insta_exit(tmp_path) -> str:
    return _write_script(tmp_path, "fake_insta_exit.py", FAKE_INSTA_EXIT)


def write_fake_noisy_no_parser(tmp_path) -> str:
    return _write_script(tmp_path, "fake_noisy_no_parser.py", FAKE_NOISY_NO_PARSER)


def make_spec(name: str, kind: str, argv: list[str], unix_path: str | None = None) -> ServiceSpec:
    return ServiceSpec(name=name, kind=kind, argv=argv, parser=kind, unix_path=unix_path)


class StubHub:
    """Minimal hub double recording the calls WfbChild's parsers make."""

    def __init__(self):
        self.rx_windows = []
        self.sessions = []
        self.tx_windows = []

    def update_rx_stats(self, child_id, packets, ant, session):
        self.rx_windows.append((child_id, packets, ant, session))

    def process_new_session(self, child_id, session):
        self.sessions.append((child_id, session))

    def update_tx_stats(self, child_id, packets, ant_latency):
        self.tx_windows.append((child_id, packets, ant_latency))


# -- (a) rx child feeds a stub hub -------------------------------------------


def test_rx_child_feeds_hub(tmp_path):
    async def main():
        spec = make_spec("video_rx", "rx", [write_fake_rx(tmp_path)])
        hub = StubHub()
        child = WfbChild(spec, hub)
        try:
            assert await child.start() is True
            await asyncio.sleep(0.3)
            assert len(hub.rx_windows) > 0
            child_id, packets, ant, session = hub.rx_windows[0]
            assert child_id == "video_rx"
            assert "all" in packets
            assert len(hub.sessions) > 0
        finally:
            await child.stop()

    run(main())


# -- (b) tx start True + unix_sockets parsed ---------------------------------


def test_tx_child_handshake(tmp_path):
    async def main():
        spec = make_spec("tunnel_tx", "tx", [write_fake_tx(tmp_path)])
        hub = StubHub()
        child = WfbChild(spec, hub, ready_timeout=2.0)
        try:
            assert await child.start() is True
            assert child.tx_parser.unix_sockets == {0: "fake-tx-sock"}
        finally:
            await child.stop()

    run(main())


# -- (c) tx without LISTEN_UNIX_END -> False within ready_timeout -----------


def test_tx_child_handshake_timeout(tmp_path):
    async def main():
        spec = make_spec("tunnel_tx", "tx", [write_fake_tx_no_handshake(tmp_path)])
        hub = StubHub()
        child = WfbChild(spec, hub, ready_timeout=0.5)
        try:
            t0 = time.monotonic()
            ok = await child.start()
            elapsed = time.monotonic() - t0
            assert ok is False
            assert elapsed < 2.0
            assert child.state()["running"] is False
        finally:
            await child.stop()

    run(main())


# -- (d) external kill -> auto-restart increments autoRestarts --------------


def test_external_kill_triggers_auto_restart(tmp_path):
    async def main():
        spec = make_spec("video_rx", "rx", [write_fake_rx(tmp_path)])
        hub = StubHub()
        child = WfbChild(spec, hub, backoff=0.05)
        try:
            assert await child.start() is True
            pid1 = child.state()["pid"]
            os.killpg(pid1, signal.SIGKILL)

            deadline = time.monotonic() + 5.0
            st = child.state()
            while time.monotonic() < deadline and not (st["running"] and st["pid"] != pid1):
                await asyncio.sleep(0.05)
                st = child.state()

            assert st["running"] is True
            assert st["pid"] != pid1
            assert st["autoRestarts"] >= 1
        finally:
            await child.stop()

    run(main())


# -- (e) max_restarts=1 + insta-exit child -> fault + on_fault ---------------


def test_crash_loop_trips_fault_and_calls_on_fault(tmp_path):
    async def main():
        spec = make_spec("video_rx", "rx", [write_fake_insta_exit(tmp_path)])
        hub = StubHub()
        faulted = []
        child = WfbChild(spec, hub, max_restarts=1, backoff=0.02, on_fault=faulted.append)
        try:
            await child.start()
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not child.state()["fault"]:
                await asyncio.sleep(0.05)

            st = child.state()
            assert st["fault"] is True
            assert faulted == [child]
        finally:
            await child.stop()

    run(main())


# -- (e2) failed respawn folds into the same crash budget --------------------
#
# Review-flagged hole: if the respawn attempt itself fails (OSError from
# create_subprocess_exec, or a tx handshake timeout on the retry) _watch()
# used to set _supervise=False and return WITHOUT setting _fault or calling
# on_fault -- a zombie terminal state (running=False, fault=False, watch task
# dead) that nothing retries and nothing alarms on.


def test_failed_respawn_trips_fault_not_zombie(tmp_path, monkeypatch):
    async def main():
        spec = make_spec("video_rx", "rx", [write_fake_insta_exit(tmp_path)])
        hub = StubHub()
        faulted = []
        child = WfbChild(spec, hub, max_restarts=1, backoff=0.02, on_fault=faulted.append)

        real_create = asyncio.create_subprocess_exec
        call_count = 0

        async def flaky_create(*args, **kwargs):
            nonlocal call_count
            call_count += 1
            if call_count >= 2:
                raise OSError("respawn boom")
            return await real_create(*args, **kwargs)

        monkeypatch.setattr(asyncio, "create_subprocess_exec", flaky_create)

        try:
            await child.start()
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not child.state()["fault"]:
                await asyncio.sleep(0.05)

            st = child.state()
            # The hole: pre-fix, this ends running=False, fault=False (a
            # dead watch task supervising nothing). The fix charges the
            # failed respawn against the crash budget like a live crash.
            assert st["fault"] is True
            assert st["running"] is False
            assert faulted == [child]
        finally:
            await child.stop()

    run(main())


def test_failed_respawn_retries_under_budget_then_recovers(tmp_path, monkeypatch):
    async def main():
        spec = make_spec("video_rx", "rx", [write_fake_insta_exit(tmp_path)])
        good_argv = write_fake_rx(tmp_path)
        hub = StubHub()
        faulted = []
        child = WfbChild(spec, hub, max_restarts=3, backoff=0.02, on_fault=faulted.append)

        real_create = asyncio.create_subprocess_exec
        call_count = 0

        async def flaky_create(*args, **kwargs):
            nonlocal call_count
            call_count += 1
            if call_count == 2:
                # The one respawn attempt that fails outright.
                raise OSError("respawn boom")
            if call_count >= 3:
                # From here on, respawn a real, long-running good child.
                args = (good_argv,)
            return await real_create(*args, **kwargs)

        monkeypatch.setattr(asyncio, "create_subprocess_exec", flaky_create)

        try:
            await child.start()
            deadline = time.monotonic() + 5.0
            st = child.state()
            while time.monotonic() < deadline and not (st["running"] and call_count >= 3):
                await asyncio.sleep(0.05)
                st = child.state()

            assert st["running"] is True
            assert st["fault"] is False
            assert call_count >= 3
            assert faulted == []
        finally:
            await child.stop()

    run(main())


# -- (f) stop() terminates the process group ---------------------------------


# -- (g) spec.parser=None -> no parser built, no hub calls, settle-style
# readiness (ready as soon as alive, regardless of `kind`) --------------------
#
# Forwarders (kind="rx") and injectors (kind="tx") both carry parser=None
# (graph.py: neither role ever emits an IPC_MSG stats line). Dispatching
# parser construction on `.kind` alone (the pre-fix behavior) builds a real
# TxLineParser for an injector and then blocks in `_wait_tx_ready` for a
# LISTEN handshake that will never arrive -- a bug flagged by Task 4 review.
# The fix keys off `spec.parser is None` instead: no parser, stdout is
# drained (not fed to any parser), and readiness never waits on a handshake.


def test_parser_none_child_skips_parser_and_settles_ready(tmp_path):
    async def main():
        spec = make_spec("mavlink inj", "tx", [write_fake_noisy_no_parser(tmp_path)])
        spec.parser = None
        hub = StubHub()
        child = WfbChild(spec, hub, ready_timeout=1.0)
        try:
            t0 = time.monotonic()
            ok = await child.start()
            elapsed = time.monotonic() - t0
            assert ok is True
            # No handshake wait: readiness should land almost immediately,
            # well under ready_timeout.
            assert elapsed < 0.5
            assert child.tx_parser is None

            # Let the noisy child print a bunch of lines the (nonexistent)
            # parser could never have understood.
            await asyncio.sleep(0.2)
            assert hub.rx_windows == []
            assert hub.sessions == []
            assert hub.tx_windows == []
            assert child.state()["running"] is True
        finally:
            await child.stop()

    run(main())


def test_stop_terminates_process_group(tmp_path):
    async def main():
        spec = make_spec("video_rx", "rx", [write_fake_rx(tmp_path)])
        hub = StubHub()
        child = WfbChild(spec, hub)
        assert await child.start() is True
        pid = child.state()["pid"]

        await child.stop()

        assert child.state()["running"] is False
        with pytest.raises(ProcessLookupError):
            os.killpg(pid, 0)

    run(main())
