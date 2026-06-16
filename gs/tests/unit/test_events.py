from fpvdgs.events import EventBus, DRONE_CONNECTED, DRONE_DISCONNECTED


def test_subscribe_receives_published_payload():
    bus = EventBus()
    got = []
    bus.subscribe("e", got.append)
    bus.publish("e", {"x": 1})
    assert got == [{"x": 1}]


def test_publish_with_no_payload_delivers_empty_dict():
    bus = EventBus()
    got = []
    bus.subscribe("e", got.append)
    bus.publish("e")
    assert got == [{}]


def test_unsubscribe_stops_delivery():
    bus = EventBus()
    got = []

    def cb(p):
        got.append(p)

    bus.subscribe("e", cb)
    bus.publish("e", {"n": 1})
    bus.unsubscribe("e", cb)
    bus.publish("e", {"n": 2})
    assert got == [{"n": 1}]


def test_subscriber_exception_is_isolated():
    bus = EventBus()
    got = []

    def bad(p):
        raise RuntimeError("boom")

    def good(p):
        got.append(p)

    bus.subscribe("e", bad)
    bus.subscribe("e", good)
    bus.publish("e", {"n": 1})       # bad raises; good must still run
    assert got == [{"n": 1}]


def test_dispatch_order_is_subscription_order():
    bus = EventBus()
    order = []
    bus.subscribe("e", lambda p: order.append("a"))
    bus.subscribe("e", lambda p: order.append("b"))
    bus.publish("e")
    assert order == ["a", "b"]


def test_state_caches_latest_drone_payload():
    bus = EventBus()
    assert bus.state("drone") is None
    bus.publish(DRONE_CONNECTED, {"state": "connected"})
    assert bus.state("drone") == {"state": "connected"}
    bus.publish(DRONE_DISCONNECTED, {"state": "disconnected", "reason": "tunnel_lost"})
    assert bus.state("drone")["state"] == "disconnected"
