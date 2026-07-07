import asyncio
import socket
import time
from unittest.mock import patch

from fpvdgs.wfb.cards import Card
from fpvdgs.wfb.cluster import (
    NodeSession,
    cluster_wlan_id,
    derive_server_address,
    plan_cluster,
    ssh_argv,
)

LOCAL = [Card(host=None, iface="wlan1"), Card(host=None, iface="wlan2")]
REMOTE = [Card(host="192.168.1.10", iface="wlan0", txpower_dbm="off")]


def test_plan_ports_and_peers():
    p = plan_cluster(LOCAL + REMOTE)
    assert p.server_port == {"video": 10000, "mavlink": 10001, "tunnel": 10002}
    # local node: video 11000+11001, mavlink 11002+11003, tunnel 11004+11005
    assert p.injector_base[("127.0.0.1", "mavlink")] == 11002
    # remote node has its OWN counter: video 11000, mavlink 11001, tunnel 11002
    assert p.injector_base[("192.168.1.10", "tunnel")] == 11002
    # peers in sorted node order; local sorts first ("127..." < "192...")
    assert p.peers["mavlink"] == ["127.0.0.1:11002,11003", "192.168.1.10:11001"]


def test_rx_only_ids():
    p = plan_cluster(LOCAL + REMOTE)
    assert p.rx_only_wlan_ids == frozenset({cluster_wlan_id("192.168.1.10", 0)})


def test_cluster_wlan_id_encoding():
    assert cluster_wlan_id("127.0.0.1", 1) == ((0x7F000001) << 24) | 1


def test_plan_with_probe_appends_after_existing_services():
    """Probe allocation is append-only: every existing service's ports are
    byte-identical with and without the probe (parity preservation)."""
    cards = LOCAL + REMOTE
    base = plan_cluster(cards)
    probed = plan_cluster(cards, with_probe=True)
    assert probed.server_port["video"] == base.server_port["video"]
    assert probed.server_port["mavlink"] == base.server_port["mavlink"]
    assert probed.server_port["tunnel"] == base.server_port["tunnel"]
    assert probed.server_port["probe"] == base.server_port["tunnel"] + 1
    for key, val in base.injector_base.items():
        assert probed.injector_base[key] == val
    assert "probe" in probed.peers
    assert "probe" not in base.server_port


def test_service_order_and_streams_for():
    from fpvdgs.wfb.cluster import PROBE_STREAMS, service_order, streams_for

    assert service_order(False) == ("video", "mavlink", "tunnel")
    assert service_order(True) == ("video", "mavlink", "tunnel", "probe")
    assert "probe" not in streams_for(False)
    assert streams_for(True)["probe"] == PROBE_STREAMS
    assert PROBE_STREAMS == {"rx": 50, "tx": None}


# -- derive_server_address ----------------------------------------------------


def test_derive_server_address_override_wins():
    assert derive_server_address("127.0.0.1", "10.9.9.9") == "10.9.9.9"


def test_derive_server_address_derives_local_ip():
    addr = derive_server_address("127.0.0.1", None)
    socket.inet_aton(addr)  # raises OSError if not a dotted-quad
    assert addr == "127.0.0.1"


# -- ssh_argv -------------------------------------------------------------------


def test_ssh_argv_shape_with_key():
    card = Card(
        host="10.0.0.5",
        iface="wlan0",
        ssh_user="root",
        ssh_port=2222,
        ssh_key="/root/.ssh/id_ed25519",
    )
    assert ssh_argv(card) == [
        "ssh",
        "-o",
        "BatchMode=yes",
        "-o",
        "ServerAliveInterval=5",
        "-o",
        "ServerAliveCountMax=3",
        "-o",
        "StrictHostKeyChecking=accept-new",
        "-p",
        "2222",
        "-i",
        "/root/.ssh/id_ed25519",
        "root@10.0.0.5",
        "exec sh -s",
    ]


def test_ssh_argv_no_key_uses_defaults():
    card = Card(host="10.0.0.5", iface="wlan0")
    argv = ssh_argv(card)
    assert "-i" not in argv
    assert "-p" in argv and argv[argv.index("-p") + 1] == "22"
    assert argv[-2:] == ["root@10.0.0.5", "exec sh -s"]


# -- NodeSession ----------------------------------------------------------------


def run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


def _bash(cmd: str) -> list[str]:
    return ["bash", "-c", cmd]


def test_node_session_start_delivers_script(tmp_path):
    capture = tmp_path / "captured.txt"

    def argv_builder(_node):
        # cat writes what it reads as it arrives; it then blocks on the
        # next read() since NodeSession never closes stdin.
        return _bash(f"cat > {capture}")

    async def main():
        session = NodeSession(
            "n1",
            "hello from the node script\n",
            argv_builder=argv_builder,
            backoff=0.01,
            max_backoff=0.05,
        )
        try:
            ok = await session.start()
            assert ok is True
            assert session.alive is True

            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not capture.exists():
                await asyncio.sleep(0.02)

            assert capture.read_text() == "hello from the node script\n"
        finally:
            await session.stop()

    run(main())


def test_node_session_respawns_with_backoff_then_stays_up():
    calls: list[int] = []
    states: list[tuple[str, bool]] = []

    def argv_builder(_node):
        calls.append(1)
        if len(calls) <= 2:
            return _bash("exit 0")
        return _bash("sleep 5")

    async def main():
        session = NodeSession(
            "n1",
            "script\n",
            argv_builder=argv_builder,
            backoff=0.01,
            max_backoff=0.02,
            on_state=lambda node, up: states.append((node, up)),
        )
        try:
            await session.start()
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline and not (
                len(calls) >= 3 and states and states[-1] == ("n1", True)
            ):
                await asyncio.sleep(0.02)

            assert len(calls) >= 3
            assert states.count(("n1", False)) >= 2
            assert states[-1] == ("n1", True)
            assert session.alive is True

            # Give it a moment more to make sure the "stays up" run really
            # does stay up (no further flapping).
            settled = len(states)
            await asyncio.sleep(0.3)
            assert len(states) == settled
        finally:
            await session.stop()

    run(main())


def test_node_session_backoff_resets_after_durable_session():
    """A node that flaps a couple of times (backoff grows), then stays up
    for at least `stable_reset_s` before a later drop, should reconnect at
    the BASE backoff on that next retry — not stay pinned near
    max_backoff. We spy on `asyncio.sleep` (the only call site of it in
    NodeSession._watch) to record the exact backoff delay requested for
    each retry, and clamp the real wait so the test stays fast regardless
    of the (tiny) nominal backoff/max_backoff values.
    """
    calls: list[int] = []
    recorded_delays: list[float] = []
    real_sleep = asyncio.sleep

    def argv_builder(_node):
        calls.append(1)
        n = len(calls)
        if n <= 2:
            return _bash("exit 0")  # fast failures: backoff should grow
        if n == 3:
            return _bash("sleep 0.3")  # durable: exceeds stable_reset_s
        return _bash("sleep 5")  # settle here so the test can finish

    async def fast_sleep(delay, result=None):
        recorded_delays.append(delay)
        await real_sleep(min(delay, 0.01))
        return result

    async def main():
        with patch("asyncio.sleep", new=fast_sleep):
            session = NodeSession(
                "n1",
                "script\n",
                argv_builder=argv_builder,
                backoff=0.01,
                max_backoff=0.03,
                stable_reset_s=0.1,
            )
            try:
                await session.start()
                deadline = time.monotonic() + 3.0
                while time.monotonic() < deadline and len(calls) < 4:
                    await real_sleep(0.02)

                assert len(calls) >= 4
                # Two fast failures: backoff grows 0.01 -> 0.02 before the
                # durable (0.3s) 3rd spawn.
                assert recorded_delays[0] == 0.01
                assert recorded_delays[1] == 0.02
                # The 3rd spawn stayed up past stable_reset_s, so the
                # retry after IT exits must be back at base backoff
                # (0.01), not continuing to grow / staying capped at
                # max_backoff (0.03).
                assert recorded_delays[2] == 0.01
            finally:
                await session.stop()

    run(main())


def test_node_session_stop_terminates_and_stops_respawning():
    calls: list[int] = []

    def argv_builder(_node):
        calls.append(1)
        return _bash("sleep 5")

    async def main():
        session = NodeSession("n1", "x\n", argv_builder=argv_builder, backoff=0.01)
        assert await session.start() is True
        assert session.alive is True

        t0 = time.monotonic()
        await session.stop()
        elapsed = time.monotonic() - t0

        assert elapsed < 2.0
        assert session.alive is False
        assert session.state() == {"alive": False, "restarts": 0}

        calls_at_stop = len(calls)
        await asyncio.sleep(0.2)
        assert len(calls) == calls_at_stop  # no respawn after stop()

    run(main())
