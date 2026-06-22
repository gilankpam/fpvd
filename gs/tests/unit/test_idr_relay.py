import socket

from fpvdgs.idr_relay import IdrRelay


def test_relay_forwards_datagrams():
    # drone-side receiver
    dst = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dst.bind(("127.0.0.1", 0))
    dst_port = dst.getsockname()[1]
    relay = IdrRelay("127.0.0.1", port=0)
    relay._dest = ("127.0.0.1", dst_port)  # forward target
    relay.start()
    try:
        listen = relay.status()["listen"]
        assert listen
        host, port = listen.rsplit(":", 1)
        src = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        src.sendto(b"IDR", ("127.0.0.1", int(port)))
        dst.settimeout(2.0)
        data, _ = dst.recvfrom(16)
        assert data == b"IDR"
    finally:
        relay.stop()
        dst.close()


def test_relay_start_stop_status():
    relay = IdrRelay("127.0.0.1", port=0)
    relay.start()
    assert relay.status()["running"] is True
    relay.stop()
    assert relay.status()["running"] is False
