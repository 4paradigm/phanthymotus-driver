"""Regressions for the four PR-review findings on #577.

Each test maps to one review issue so a future refactor that reintroduces it
turns exactly one named test red:

* High   — an outstanding accompany poll must not delay StopMove.
* Medium — operator input must post-date the confirmed accompany transition
  and stay within a freshness window before it becomes exit evidence.
* Medium — ``--watch`` must pair the freshest mode value, never the mode that
  occurred most often over the whole run.
* Low    — the driver catalogue description must advertise the new card.
"""
import importlib.util
import sys
import threading
import time
import types
import unittest
from pathlib import Path

ROOT = Path(__file__).parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def _load(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


class _GateChannel:
    """Stands in for _RpcChannel: blocks in call() until an event is set.

    A real _RpcChannel serialises all calls behind one lock. Simulating that
    with an Event lets us prove that an observer occupying its own channel
    cannot delay a motion call on a different channel.
    """

    def __init__(self, gate=None):
        self.gate = gate
        self.calls = []
        self.entered = threading.Event()

    def call(self, method, *args):
        self.calls.append(method)
        if self.gate is not None:
            self.entered.set()
            self.gate.wait(5.0)
        return (0, {"fsm_name": "AI_ACCOMPANY"}) if method == "GetState" else 0

    def stop(self):
        pass


class RpcChannelIsolationTests(unittest.TestCase):
    """High finding: motion RPCs cannot be blocked by an observation poll."""

    def setUp(self):
        self.rpc = _load("as2w_rpc_safety_under_test", ROOT / "rpc_proxy.py")

    def test_three_channels_are_created_with_separate_deadlines(self):
        """A real RpcProxy owns sport, audio_led and a short-lived observer."""
        seen = []

        class Recorder:
            def __init__(self, interface, kind, timeout):
                seen.append((kind, timeout))

        original = self.rpc._RpcChannel
        try:
            self.rpc._RpcChannel = Recorder
            self.rpc.RpcProxy("eth0")
        finally:
            self.rpc._RpcChannel = original
        kinds = {kind for kind, _ in seen}
        self.assertEqual({"sport", "audio_led", "sport_observe"}, kinds)
        observer_timeout = dict(seen)["sport_observe"]
        self.assertLess(observer_timeout, dict(seen)["sport"],
                        "the observer must fail faster than the motion channel")

    def test_observe_state_uses_the_isolated_channel_not_sport(self):
        proxy = self.rpc.RpcProxy.__new__(self.rpc.RpcProxy)
        proxy._sport = _GateChannel()
        proxy._observe = _GateChannel()
        proxy.observe_state()
        self.assertEqual([], proxy._sport.calls,
                         "observation must never touch the motion channel")
        self.assertEqual(["GetState"], proxy._observe.calls)

    def test_an_outstanding_observer_does_not_delay_a_motion_stop(self):
        """The exact scenario from the review: a stuck poll vs. StopMove."""
        gate = threading.Event()
        proxy = self.rpc.RpcProxy.__new__(self.rpc.RpcProxy)
        proxy._sport = _GateChannel()
        proxy._observe = _GateChannel(gate=gate)
        proxy._audio_led = _GateChannel()

        result = {}
        poller = threading.Thread(target=lambda: proxy.observe_state())
        poller.start()
        self.assertTrue(proxy._observe.entered.wait(2.0),
                        "observer never reached its blocking call")

        started = time.monotonic()
        stopper = threading.Thread(
            target=lambda: result.setdefault("stop", proxy.call("StopMove")))
        stopper.start()
        stopper.join(timeout=1.0)
        elapsed = time.monotonic() - started
        gate.set()
        poller.join(timeout=2.0)

        self.assertFalse(stopper.is_alive(),
                         "StopMove was blocked behind the observer")
        self.assertEqual(0, result["stop"])
        self.assertLess(elapsed, 1.0,
                        f"stop took {elapsed:.2f}s behind an outstanding poll")


class OperatorInputFreshnessTests(unittest.TestCase):
    """Medium finding: pre-entry or stale input must not be exit evidence."""

    @classmethod
    def setUpClass(cls):
        # Install the same minimal ROS stubs the main contract suite uses.
        std_msgs = types.ModuleType("std_msgs.msg")
        std_msgs.String = type("String", (), {})
        sys.modules.setdefault("std_msgs", types.ModuleType("std_msgs"))
        sys.modules["std_msgs.msg"] = std_msgs
        qos = types.ModuleType("rclpy.qos")
        qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
        qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
        qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
        qos.QoSProfile = lambda **kwargs: kwargs
        sys.modules.setdefault("rclpy", types.ModuleType("rclpy"))
        sys.modules["rclpy.qos"] = qos
        node = types.ModuleType("rclpy.node")
        node.Node = type("Node", (), {"__init__": lambda self, *a, **k: None})
        sys.modules["rclpy.node"] = node
        cls.module = _load("as2w_accompany_safety_under_test", ROOT / "accompany.py")

    def test_input_before_entry_is_not_evidence(self):
        fn = self.module._is_post_entry_evidence
        entered = 10000
        pre_entry = {"at_ms": 9000, "peak_axis": 0.7, "buttons": []}
        self.assertFalse(fn(pre_entry, entered, 10500))

    def test_input_after_entry_within_ttl_is_evidence(self):
        fn = self.module._is_post_entry_evidence
        entered = 10000
        post_entry = {"at_ms": 10100, "peak_axis": 0.7, "buttons": []}
        self.assertTrue(fn(post_entry, entered, 10500))

    def test_stale_input_after_entry_is_not_evidence(self):
        fn = self.module._is_post_entry_evidence
        entered = 10000
        stale = {"at_ms": 10100, "peak_axis": 0.7, "buttons": []}
        self.assertFalse(fn(stale, entered, 10000 + 11000),
                         "an 11 s old input must have expired at a 10 s TTL")

    def test_no_confirmed_entry_never_yields_evidence(self):
        fn = self.module._is_post_entry_evidence
        post_entry = {"at_ms": 10100, "peak_axis": 0.7, "buttons": []}
        self.assertFalse(fn(post_entry, None, 10200))

    def test_poll_uses_observe_state_when_available(self):
        """The card prefers the isolated channel on a real RpcProxy."""
        class Proxy:
            def __init__(self):
                self.observe_calls = 0
                self.getstate_calls = 0

            def observe_state(self):
                self.observe_calls += 1
                return 0, {"fsm_name": "AI_ACCOMPANY"}

            def GetState(self):
                self.getstate_calls += 1
                return 0, {"fsm_name": "AI_ACCOMPANY"}

        proxy = Proxy()
        plugin = self.module.AccompanyPlugin(
            {"fsm_names": ["ai_accompany"]}, "test", None, proxy=proxy,
            state_plugin=None)
        plugin._poll_fsm()
        self.assertEqual(1, proxy.observe_calls)
        self.assertEqual(0, proxy.getstate_calls)


class WatchFreshnessTests(unittest.TestCase):
    """Medium finding: --watch pairs the freshest mode, not the mode count winner."""

    @classmethod
    def setUpClass(cls):
        cls.probe = _load("as2w_probe_safety_under_test", ROOT / "accompany_probe.py")

    def test_snapshot_uses_the_value_passed_in_not_the_counts(self):
        """A brief transition (few frames) survives a dominant baseline."""
        counts = {3: 900, 17: 2}
        latest_mode = 17  # the freshest callback value, not max(counts)
        sample = self.probe.watch_snapshot(latest_mode, "AI_ACCOMPANY", 42.5,
                                           0, counts)
        self.assertEqual(17, sample["mode"])
        self.assertEqual("AI_ACCOMPANY", sample["fsm_name"])
        self.assertEqual(42.5, sample["at_s"])
        self.assertEqual(0, sample["code"])

    def test_snapshot_keeps_none_when_no_mode_frame_yet(self):
        sample = self.probe.watch_snapshot(None, "AI_ACCOMPANY", 1.0, 0, {})
        self.assertIsNone(sample["mode"],
                          "no DDS frame yet must not be guessed from counts")
        self.assertEqual("AI_ACCOMPANY", sample["fsm_name"])

    def test_transition_then_baseline_produces_two_distinct_samples(self):
        first = self.probe.watch_snapshot(17, "AI_ACCOMPANY", 10.0, 0, {17: 2})
        second = self.probe.watch_snapshot(3, "AI_BALANCE_STAND", 18.0, 0, {17: 2, 3: 900})
        self.assertEqual(17, first["mode"])
        self.assertEqual(3, second["mode"])
        self.assertNotEqual(first["mode"], second["mode"],
                            "a transition must not be smeared into the baseline")


class CatalogueDescriptionTests(unittest.TestCase):
    """Low finding: the driver metadata must mention the new capability."""

    def test_description_advertises_the_accompany_observer(self):
        text = (ROOT / "driver.yaml").read_text(encoding="utf-8")
        description = next(line for line in text.splitlines()
                           if line.strip().startswith("description:"))
        self.assertIn("伴随", description)


if __name__ == "__main__":
    unittest.main()
