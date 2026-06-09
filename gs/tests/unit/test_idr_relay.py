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
