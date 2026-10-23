import asyncio, threading
from events import EventHub


def test_publish_from_thread_drops_oldest_when_full():
    hub = EventHub(maxsize=2)

    async def main():
        sub = hub.subscribe()
        th = threading.Thread(target=lambda: [hub.publish({"n": i}) for i in range(3)])
        th.start()
        th.join()
        await asyncio.sleep(0.05)
        q = sub[1]
        got = [q.get_nowait() for _ in range(q.qsize())]
        hub.unsubscribe(sub)
        hub.publish({"n": 9})
        await asyncio.sleep(0.05)
        return got, q.qsize()

    assert asyncio.run(main()) == ([{"n": 1}, {"n": 2}], 0)
