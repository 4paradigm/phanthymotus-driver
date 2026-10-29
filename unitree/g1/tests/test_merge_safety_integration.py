"""Offline regression checks for timed motion and proposal hand-off."""
import threading

from safety_harness import SmartMotionProxy
from timed_motion import TimedMotion


class Client:
    def __init__(self):
        self.move_reply = threading.Event()
        self.stop_reply = threading.Event()
        self.sent = []
        self.move_sent = threading.Event()

    def BeginMove(self, *args):
        self.sent.append("move")
        self.move_sent.set()
        return lambda: (self.move_reply.wait(1), 0)[1]

    def BeginStopMove(self, *args):
        self.sent.append("stop")
        return lambda: (self.stop_reply.wait(1), 0)[1]


def test_timed_motion_not_idle_until_move_and_stop_acknowledged():
    client = Client()
    motion = TimedMotion(client)
    move = threading.Thread(target=lambda: motion.start(0.1, 0, 0, 0, "a"))
    move.start()
    assert client.move_sent.wait(1)
    assert not motion.is_idle()
    stop = threading.Thread(target=lambda: motion.stop("command"))
    stop.start()
    client.stop_reply.set()
    stop.join(1)
    assert not motion.is_idle()  # late move reply must not authorize handoff
    client.move_reply.set()
    move.join(1)
    assert motion.is_idle()
    assert client.sent == ["move", "stop"]


def test_proposal_bind_still_brackets_parent_stop_with_confirmation():
    proxy = SmartMotionProxy.__new__(SmartMotionProxy)
    proxy._call_lock = threading.Lock()
    calls = []

    def call(method, **kwargs):
        calls.append(method)
        if method == "begin_velocity_proposal_stop_confirmation":
            return {"confirmation_start": {"monotonic": 1}}
        assert kwargs["external_stop_attempt"]["stop_move_ret"] == 0
        return {"connected": True, "stop_confirmed": True}

    proxy._call = call
    proxy._fallback_stop = lambda: calls.append("parent_stop") or 0
    result = proxy.bind_velocity_proposal("/proposal", "nav")
    assert result["stop_confirmed"] is True
    assert calls == ["begin_velocity_proposal_stop_confirmation", "parent_stop", "bind_velocity_proposal"]
