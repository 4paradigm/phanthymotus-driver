"""Schema/adapter contract tests for Adam RL locomotion."""

from __future__ import annotations

import sys
import time
import types
import unittest
from pathlib import Path

sys.modules.setdefault("numpy", types.ModuleType("numpy"))
sys.path.insert(0, str(Path(__file__).parent))

import device
from device import (
    LocoStatePlugin,
    MotionPlugin,
    RlLocoPlugin,
)


class _Grpc:
    def __init__(self):
        self.mode = None
        self.state = {
            "success": True,
            "fsm_state": "STOP",
            "switchable_states": ["STOP", "STAND_WALK", "MULTI_AGENT", "MOTION_TRACK"],
            "available_actions": ["SetMotion", "SetTrackingMotion"],
        }
        self.control_state = {"success": True, "domain_id": 1}

    def set_mode(self, mode):
        self.mode = mode
        self.state["fsm_state"] = mode
        return {"success": True, "current_state": mode}

    def set_control_mode(self, domain_id):
        self.domain_id = domain_id
        return {"success": True, "domain_id": domain_id}

    def set_velocity(self, vx, vy, vyaw):
        self.velocity = (vx, vy, vyaw)
        return {"success": True}

    def set_height(self, height):
        self.height = height
        return {"success": True}

    def get_robot_state(self):
        return dict(self.state)

    def set_tracking_motion(self, motion_file):
        self.tracking_motion = motion_file
        return {"success": True, "current_tracking_motion": motion_file}

    def get_control_state(self):
        return dict(self.control_state)

    def set_motion(self, command, motion_file):
        self.motion = (command, motion_file)
        if command == "STOP":
            self.state["motion_playing"] = False
        return {"success": True, "current_motion": motion_file}

    def shutdown(self, force=False):
        self.shutdown_force = force
        return {"success": True}


class _Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message.data)


class _LocoStateNode:
    def __init__(self, namespace):
        self._topic = f"/{namespace}/loco/state"
        self._publisher = _Publisher()

    def publish_state(self, state):
        message = types.SimpleNamespace(data=__import__("json").dumps(state))
        self._publisher.publish(message)


class _Executor:
    def add_node(self, node):
        self.node = node

    def remove_node(self, node):
        self.removed = node


class LocoContractTests(unittest.TestCase):
    def setUp(self):
        self._original_loco_state_node = device._LocoStatePublisherNode
        device._LocoStatePublisherNode = _LocoStateNode

    def tearDown(self):
        device._LocoStatePublisherNode = self._original_loco_state_node

    def _wait_for_messages(self, publisher, count=1, timeout=1.0):
        deadline = time.monotonic() + timeout
        while len(publisher.messages) < count and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertGreaterEqual(
            len(publisher.messages), count, "loco_state publish timed out")

    def test_loco_state_is_read_only_and_flattens_grpc_state(self):
        grpc = _Grpc()
        executor = _Executor()
        plugin = LocoStatePlugin({"publish_rate_hz": 20}, "adam", executor, grpc)
        publisher = plugin._node._publisher

        tool = plugin.get_tool()
        self.assertEqual("loco_state", tool["name"])
        self.assertEqual("sensor", tool["type"])
        self.assertTrue(tool["readOnly"])
        self.assertFalse(tool["inputSchema"]["additionalProperties"])
        self.assertEqual(
            [{"topic": "/adam/loco/state", "format": "data/json"}],
            tool["topic_out"],
        )
        self.assertFalse(hasattr(grpc, "velocity"))

        plugin.start()
        self._wait_for_messages(publisher)
        plugin.stop()

        payload = __import__("json").loads(publisher.messages[-1])
        self.assertTrue(payload["success"])
        self.assertEqual("STOP", payload["fsm_state"])
        self.assertEqual(["SetMotion", "SetTrackingMotion"], payload["available_actions"])
        __import__("json").dumps(payload)
        self.assertIn("timestamp_ms", payload)
        self.assertFalse(hasattr(grpc, "velocity"))

    def test_loco_state_publishes_grpc_errors_without_writes(self):
        class FailingGrpc(_Grpc):
            def get_robot_state(self):
                raise RuntimeError("controller unavailable")

        grpc = FailingGrpc()
        executor = _Executor()
        plugin = LocoStatePlugin({"publish_rate_hz": 20}, "adam", executor, grpc)
        publisher = plugin._node._publisher

        plugin.start()
        self._wait_for_messages(publisher)
        plugin.stop()

        payload = __import__("json").loads(publisher.messages[-1])
        self.assertFalse(payload["success"])
        self.assertEqual("STATE_UNAVAILABLE", payload["code"])
        self.assertIn("controller unavailable", payload["message"])
        self.assertIsNone(grpc.mode)
        self.assertFalse(hasattr(grpc, "domain_id"))
        self.assertFalse(hasattr(grpc, "velocity"))

    def test_loco_state_converts_malformed_state_to_error(self):
        class MalformedGrpc(_Grpc):
            def get_robot_state(self):
                return None

        plugin = LocoStatePlugin(
            {"publish_rate_hz": 20}, "adam", _Executor(), MalformedGrpc())
        publisher = plugin._node._publisher

        plugin.start()
        self._wait_for_messages(publisher)
        plugin.stop()

        payload = __import__("json").loads(publisher.messages[-1])
        self.assertFalse(payload["success"])
        self.assertEqual("STATE_UNAVAILABLE", payload["code"])
        self.assertIn("must return an object", payload["message"])

    def test_loco_state_survives_publish_failure(self):
        grpc = _Grpc()
        plugin = LocoStatePlugin(
            {"publish_rate_hz": 20}, "adam", _Executor(), grpc)
        publisher = plugin._node._publisher
        publish = publisher.publish
        failures = [True]

        def fail_once(message):
            if failures:
                failures.pop()
                raise RuntimeError("publisher unavailable")
            publish(message)

        publisher.publish = fail_once
        plugin.start()
        self._wait_for_messages(publisher, timeout=1.5)
        plugin.stop()

        payload = __import__("json").loads(publisher.messages[-1])
        self.assertTrue(payload["success"])
        self.assertFalse(plugin._running)

    def test_loco_hides_mode_and_automatically_enters_walking_state(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)

        schema = plugin.get_tool()["inputSchema"]
        self.assertEqual(["move", "set_height", "stop"], schema["properties"]["action"]["enum"])
        self.assertNotIn("target_state", schema["properties"])
        self.assertNotIn("set_mode", schema["x-action-params"])
        self.assertEqual(0.1, schema["properties"]["duration_s"]["minimum"])
        self.assertEqual(30.0, schema["properties"]["duration_s"]["maximum"])
        self.assertEqual(-1.0, schema["properties"]["vx"]["minimum"])
        self.assertEqual(1.0, schema["properties"]["height"]["maximum"])

        result = plugin.dispatch("move", {
            "vx": 0.2, "vy": 0.0, "vyaw": -0.1, "duration_s": 1.0,
        })
        self.assertEqual(grpc.mode, "STAND_WALK")
        self.assertEqual((0.2, 0.0, -0.1), grpc.velocity)
        self.assertTrue(result["success"])
        self.assertTrue(result["auto_stop"])

    def test_loco_rejects_unbounded_move(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)
        result = plugin.dispatch("move", {"vx": 0.2, "vy": 0.0, "vyaw": 0.0})
        self.assertEqual("INVALID_ARGUMENT", result["code"])

    def test_loco_height_and_stop_use_direct_motion_requests(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)

        plugin.dispatch("set_height", {"height": 0.1})
        self.assertEqual(0.1, grpc.height)
        plugin.dispatch("stop", {})
        self.assertEqual((0.0, 0.0, 0.0), grpc.velocity)

    def test_focused_execution_cards_are_registered_contracts(self):
        grpc = _Grpc()
        cards = [
            MotionPlugin({}, "adam", None, grpc),
        ]
        self.assertEqual(
            {"motion"},
            {card.get_tool()["name"] for card in cards},
        )
        self.assertIn("上半身", cards[0].get_tool()["description"])

    def test_motion_card_uses_robot_side_files(self):
        grpc = _Grpc()
        motion = MotionPlugin({}, "adam", None, grpc)
        motion.dispatch("play", {"motion_file": "Sources/motion/Wave.txt"})
        self.assertEqual(("PLAY", "Sources/motion/Wave.txt"), grpc.motion)
        motion.dispatch("stop", {})
        self.assertEqual(("STOP", ""), grpc.motion)

    def test_motion_card_answers_the_canvas_lifecycle_verbs(self):
        grpc = _Grpc()
        motion = MotionPlugin({}, "adam", None, grpc)
        self.assertEqual({"state": "ready"}, motion.dispatch("start", {}))
        self.assertIsNotNone(motion.dispatch("info", {}))
        self.assertIsNotNone(motion.dispatch("stop", {}))
        # Only the lifecycle verbs are answered; anything else still declines
        # so the bundle reports it as an unknown action.
        self.assertIsNone(motion.dispatch("teleport", {}))

    def test_timed_move_declares_and_reports_completion(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)

        completion = plugin.get_tool()["inputSchema"]["x-completion"]
        self.assertEqual(["move"], completion["actions"])
        # Must outlast the 30s maximum duration plus the reporting round trip.
        self.assertGreaterEqual(completion["timeout"], 30)

        reported = []
        original = device._notify_action_completion
        device._notify_action_completion = (
            lambda action_id, status, result, tool:
            reported.append((action_id, status, tool)))
        try:
            result = plugin.dispatch("move", {
                "vx": 0.2, "vy": 0.0, "vyaw": 0.0, "duration_s": 0.1,
            })
            deadline = time.monotonic() + 2.0
            while not reported and time.monotonic() < deadline:
                time.sleep(0.02)
        finally:
            device._notify_action_completion = original

        self.assertTrue(result["action_id"].startswith("adam_loco_move_"))
        self.assertEqual([(result["action_id"], "completed", "loco")], reported)

    def test_superseded_timed_move_is_reported_cancelled(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)
        reported = []
        original = device._notify_action_completion
        device._notify_action_completion = (
            lambda action_id, status, result, tool:
            reported.append((action_id, status)))
        try:
            first = plugin.dispatch("move", {
                "vx": 0.2, "vy": 0.0, "vyaw": 0.0, "duration_s": 5.0,
            })
            # An explicit stop ends the move long before its timer is due;
            # Agent Core must not keep waiting on the declared timeout.
            plugin.dispatch("stop", {})
        finally:
            device._notify_action_completion = original

        self.assertEqual([(first["action_id"], "cancelled")], reported)
        self.assertEqual((0.0, 0.0, 0.0), grpc.velocity)


if __name__ == "__main__":
    unittest.main()
