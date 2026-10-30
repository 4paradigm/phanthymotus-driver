"""Speaker subprocess commands: stop must not queue behind the startup beep.

`start` plays an 8 s beep before subscribing. When it ran inline on the single
command loop, a `stop` sent meanwhile waited for the whole beep -- past Core's
5 s per-card stop deadline -- so Core dropped the connection, the driver logged
BrokenPipe, and the project stop was reported unconfirmed. Observed on G1:
`speaker stop` 7.0-7.8 s on every stop-project.

Run:  python3 -m unittest discover -s unitree/g1/tests -t .
"""

import queue
import threading
import time
import unittest

from test_speaker_pacing import G1


class Node:
    def __init__(self):
        self._sub = None
        self._topic = None
        self.state = "idle"
        self.calls = []
        self._buf = queue.Queue()

    def stop_play(self):
        self.calls.append("stop_play")
        self._sub, self.state = None, "idle"

    def start_play(self, topic):
        self.calls.append(("start_play", topic))
        self._sub, self._topic, self.state = object(), topic, "ready"
        return topic


class Beep:
    """Blocks like the real beep, in small blocks that honour cancellation."""

    def __init__(self, seconds=8.0):
        self.seconds, self.started, self.plays = seconds, threading.Event(), 0

    def __call__(self, cancel):
        self.plays += 1
        self.started.set()
        cancel.wait(self.seconds)


class Replies:
    def __init__(self):
        self.lock, self.items = threading.Lock(), {}

    def __call__(self, request_id, result):
        with self.lock:
            self.items[request_id] = (time.monotonic(), result)

    def wait(self, request_id, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self.lock:
                if request_id in self.items:
                    return self.items[request_id][1]
            time.sleep(0.005)
        raise AssertionError(f"no reply to {request_id}")


def commands(beep_seconds=8.0):
    node, beep, replies = Node(), Beep(beep_seconds), Replies()
    return G1._SpeakerCommands(node, beep, replies), node, beep, replies


class SpeakerCommandsTest(unittest.TestCase):
    def test_stop_during_startup_beep_answers_well_inside_core_deadline(self):
        cmds, node, beep, replies = commands()
        cmds.handle(1, "start", {"input_topic": "/perception/tts"})
        self.assertTrue(beep.started.wait(1))
        t0 = time.monotonic()
        cmds.handle(2, "stop", {})
        self.assertEqual(replies.wait(2), {"state": "idle"})
        self.assertLess(time.monotonic() - t0, 1.0)
        # The interrupted start never subscribes, and says it was cancelled.
        self.assertEqual(replies.wait(1), {"state": "idle", "cancelled": True})
        self.assertNotIn(("start_play", "/perception/tts"), node.calls)
        self.assertIsNone(node._sub)

    def test_start_replies_only_after_the_beep_and_subscription(self):
        cmds, node, beep, replies = commands(beep_seconds=0.2)
        cmds.handle(1, "start", {"input_topic": "/perception/tts"})
        self.assertEqual(replies.wait(1), {"state": "ready", "topic": "/perception/tts"})
        self.assertEqual(node.calls, ["stop_play", ("start_play", "/perception/tts")])

    def test_repeated_start_on_the_subscribed_topic_does_not_replay_the_beep(self):
        cmds, node, beep, replies = commands(beep_seconds=0.05)
        cmds.handle(1, "start", {"input_topic": "/perception/tts"})
        replies.wait(1)
        cmds.handle(2, "start", {"input_topic": "/perception/tts"})
        self.assertEqual(replies.wait(2), {"state": "ready", "topic": "/perception/tts"})
        self.assertEqual(beep.plays, 1)
        self.assertEqual(node.calls.count("stop_play"), 1)

    def test_start_while_the_same_start_is_in_flight_shares_it(self):
        cmds, node, beep, replies = commands(beep_seconds=0.3)
        cmds.handle(1, "start", {"input_topic": "/perception/tts"})
        self.assertTrue(beep.started.wait(1))
        cmds.handle(2, "start", {"input_topic": "/perception/tts"})
        self.assertEqual(replies.wait(1)["state"], "ready")
        self.assertEqual(replies.wait(2)["state"], "ready")
        self.assertEqual(beep.plays, 1)

    def test_start_on_a_new_topic_replaces_the_pending_one(self):
        cmds, node, beep, replies = commands(beep_seconds=0.3)
        cmds.handle(1, "start", {"input_topic": "/a"})
        self.assertTrue(beep.started.wait(1))
        cmds.handle(2, "start", {"input_topic": "/b"})
        self.assertEqual(replies.wait(1), {"state": "idle", "cancelled": True})
        self.assertEqual(replies.wait(2), {"state": "ready", "topic": "/b"})
        self.assertNotIn(("start_play", "/a"), node.calls)

    def test_info_is_not_blocked_by_a_start(self):
        cmds, node, beep, replies = commands()
        cmds.handle(1, "start", {"input_topic": "/perception/tts"})
        self.assertTrue(beep.started.wait(1))
        t0 = time.monotonic()
        cmds.handle(2, "info", {})
        self.assertEqual(replies.wait(2)["state"], "idle")
        self.assertLess(time.monotonic() - t0, 0.5)
        cmds.handle(3, "stop", {})
        replies.wait(3)

    def test_start_without_topic_is_rejected(self):
        cmds, _, _, replies = commands()
        cmds.handle(1, "start", {})
        self.assertEqual(replies.wait(1), {"error": "Missing input_topic"})


class ProxyRoutingTest(unittest.TestCase):
    """The proxy must not hold its IPC lock while a reply is outstanding."""

    def proxy(self):
        commands, results = queue.Queue(), queue.Queue()
        proxy = G1.SpeakerIsolatedProxy.__new__(G1.SpeakerIsolatedProxy)
        proxy._ipc_lock, proxy._request_id, proxy._waiters = threading.Lock(), 0, {}
        proxy._command_queue, proxy._result_queue = commands, results
        proxy._proc = type("Proc", (), {"is_alive": lambda self: True})()
        proxy._startup_error = None
        threading.Thread(target=proxy._route_results, daemon=True).start()
        return proxy, commands, results

    def test_stop_gets_its_own_reply_while_start_is_outstanding(self):
        proxy, commands, results = self.proxy()
        started = {}
        threading.Thread(target=lambda: started.setdefault(
            "result", proxy.dispatch("start", {"input_topic": "/t"})), daemon=True).start()
        start_cmd = commands.get(timeout=1)
        t0 = time.monotonic()
        stopper = {}
        thread = threading.Thread(target=lambda: stopper.setdefault("result", proxy.dispatch("stop", {})))
        thread.start()
        stop_cmd = commands.get(timeout=1)            # sent despite the pending start
        results.put({"id": stop_cmd["id"], "result": {"state": "idle"}})
        thread.join(2)
        self.assertEqual(stopper["result"], {"state": "idle"})
        self.assertLess(time.monotonic() - t0, 1.0)
        results.put({"id": start_cmd["id"], "result": {"state": "idle", "cancelled": True}})
        deadline = time.monotonic() + 2
        while "result" not in started and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual(started["result"], {"state": "idle", "cancelled": True})


if __name__ == "__main__":
    unittest.main()
