import asyncio

from fpvdgs.wfb.agg_queue import AggQueue


def run(coro):
    return asyncio.new_event_loop().run_until_complete(coro)


def test_aggregates_until_timeout():
    async def main():
        sent = []
        q = AggQueue(100, 0.01, sent.append)
        q.put(b"aa")
        q.put(b"bb")
        assert sent == []
        await asyncio.sleep(0.03)
        assert sent == [b"aabb"]
        q.close()

    run(main())


def test_overflow_flushes_first():
    async def main():
        sent = []
        q = AggQueue(4, 1.0, sent.append)
        q.put(b"aaa")
        q.put(b"bb")  # 3+2 > 4 -> flush "aaa", queue "bb"
        assert sent == [b"aaa"]
        q.flush()
        assert sent == [b"aaa", b"bb"]
        q.close()

    run(main())


def test_oversize_dropped_and_passthrough_mode():
    async def main():
        sent = []
        q = AggQueue(4, 1.0, sent.append)
        q.put(b"toolong")  # > max_size -> dropped
        assert sent == []
        p = AggQueue(None, None, sent.append)
        p.put(b"x")  # passthrough
        assert sent == [b"x"]
        q.close()
        p.close()

    run(main())
