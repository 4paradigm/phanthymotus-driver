"""Bringing the robot's DDS link up when it is not there yet.

Every test here is about one failure on r1_sz: the bundle started three seconds
before its network interface existed, the single `ChannelFactoryInitialize`
attempt failed, and every state topic on that robot stayed empty from boot —
while the cards publishing them declared 10 Hz in `topic_out` and looked
healthy on the canvas.

Two defects, and they need separate tests because fixing one without the other
leaves the robot equally broken and equally quiet.
"""
from __future__ import annotations

import os
import sys
import threading
import time

import pytest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from common import dds_link  # noqa: E402


class FakeClock:
    """Sleeps that return instantly but still order the retries."""

    def __init__(self):
        self.slept = []

    def __call__(self, seconds):
        self.slept.append(seconds)
        time.sleep(0.001)


def _link(initialize, preferred="eth10"):
    clock = FakeClock()
    link = dds_link.DdsLink(preferred, initialize=initialize, sleep=clock)
    return link, clock


def _settle(link, attempts=60):
    for _ in range(attempts):
        if link.ready:
            return True
        time.sleep(0.01)
    return link.ready


# ── it keeps trying ──────────────────────────────────────────────────────────

def test_an_interface_that_appears_late_is_picked_up():
    """The whole failure. eth10 came up seconds after the process did and
    stayed up all day; nothing retried."""
    state = {"up": False, "calls": 0}

    def initialize(domain, interface):
        state["calls"] += 1
        if not state["up"]:
            raise RuntimeError("eth10: does not match an available interface")

    link, _ = _link(initialize)
    link.start()
    time.sleep(0.05)
    assert not link.ready, "it cannot succeed before the interface exists"

    state["up"] = True
    assert _settle(link), "the interface came up and the link did not"
    assert link.status()["state"] == "up"


def test_the_interface_list_is_recomputed_on_every_attempt():
    """Retrying a list captured at start would be pointless: the thing being
    waited for is an interface that was not in it."""
    seen = []
    original = dds_link.candidate_interfaces

    def spy(preferred=""):
        seen.append(len(seen))
        return original(preferred)

    dds_link.candidate_interfaces = spy
    try:
        link, _ = _link(lambda d, i: (_ for _ in ()).throw(RuntimeError("no")))
        link.start()
        time.sleep(0.08)
        link.stop()
    finally:
        dds_link.candidate_interfaces = original
    assert len(seen) > 2, "the candidates were enumerated once and reused"


def test_the_backoff_does_not_spin():
    delays = []

    def initialize(domain, interface):
        raise RuntimeError("no")

    clock = FakeClock()
    link = dds_link.DdsLink("eth10", initialize=initialize, sleep=clock)
    link.start()
    time.sleep(0.1)
    link.stop()
    delays = clock.slept
    assert delays, "it retried without waiting at all"
    assert delays == sorted(delays), "the backoff must not shrink"
    assert max(delays) <= dds_link._MAX_DELAY_S


# ── subscribers are told ─────────────────────────────────────────────────────

def test_a_subscriber_registered_before_the_link_is_up_fires_later():
    state = {"up": False}
    fired = threading.Event()

    def initialize(domain, interface):
        if not state["up"]:
            raise RuntimeError("not yet")

    link, _ = _link(initialize)
    link.on_ready(fired.set)
    link.start()
    assert not fired.is_set()

    state["up"] = True
    assert fired.wait(2.0), "the link came up and the subscriber was never told"


def test_a_subscriber_registered_after_the_link_is_up_fires_immediately():
    """Otherwise there is an ordering to get wrong between bundle start and
    card construction, and getting it wrong is silent."""
    link, _ = _link(lambda d, i: None)
    link.start()
    assert _settle(link)

    fired = threading.Event()
    link.on_ready(fired.set)
    assert fired.wait(1.0)


def test_one_failing_subscriber_does_not_stop_the_others():
    link, _ = _link(lambda d, i: None)
    ran = []
    link.on_ready(lambda: (_ for _ in ()).throw(RuntimeError("boom")))
    link.on_ready(lambda: ran.append(True))
    link.start()
    assert _settle(link)
    time.sleep(0.05)
    assert ran == [True]


# ── and it says so ───────────────────────────────────────────────────────────

def test_a_link_that_is_down_reports_why_rather_than_looking_healthy():
    """The second half of the r1_sz failure: one warning at boot and afterwards
    every layer said what it says when the link is fine."""
    link, _ = _link(lambda d, i: (_ for _ in ()).throw(RuntimeError("no such interface")))
    link.start()
    time.sleep(0.06)
    link.stop()

    status = link.status()
    assert status["state"] == "connecting"
    assert status["attempts"] > 0
    assert "no such interface" in status["last_error"]
    assert "没有数据" in status["message"] or "不会有数据" in status["message"]


def test_a_link_that_is_up_names_the_interface_it_got():
    link, _ = _link(lambda d, i: None, preferred="eth10")
    link.start()
    assert _settle(link)
    assert link.status()["interface"] == "eth10"


def test_the_preferred_interface_is_tried_first():
    tried = []
    link, _ = _link(lambda d, i: tried.append(i), preferred="eth10")
    link.start()
    assert _settle(link)
    assert tried[0] == "eth10"


def test_auto_detect_is_always_the_last_resort():
    assert dds_link.candidate_interfaces("eth10")[0] == "eth10"
    assert dds_link.candidate_interfaces("eth10")[-1] == ""
    assert dds_link.candidate_interfaces("")[-1] == ""
