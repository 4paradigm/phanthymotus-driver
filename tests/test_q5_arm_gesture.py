"""Offline arm-gesture regression tests; no ROS or robot connection is used."""

import importlib.util
import math
from pathlib import Path
import sys
import threading
from types import SimpleNamespace
import unittest
from unittest.mock import patch


DRIVER = Path(__file__).resolve().parents[1] / "robotera" / "q5_bundle"


def load_module(name, filename):
    spec = importlib.util.spec_from_file_location(name, DRIVER / filename)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


with patch.dict(sys.modules, {
    "arm_control": SimpleNamespace(ArmControlPlugin=None),
    "body_command": SimpleNamespace(get_router=lambda *_: None, BodyCommandRouter=object),
    "control_contract": load_module("q5_gesture_control_contract", "control_contract.py"),
    "joint_limits": load_module("q5_gesture_joint_limits", "joint_limits.py"),
}):
    gesture = load_module("q5_arm_gesture_test", "arm_gesture.py")


class NoWaitEvent:
    def is_set(self):
        return False

    def wait(self, _seconds):
        return False


class TestArmGesture(unittest.TestCase):
    def plugin(self):
        plugin = gesture.Plugin.__new__(gesture.Plugin)
        current = gesture._mirrored_positions("neutral", "both", gesture._NEUTRAL_DEG)
        self.published = []
        self.released = []
        plugin._client = SimpleNamespace(snapshot=lambda: {"fresh": True, "joints": current})
        plugin._router = SimpleNamespace(
            acquire=lambda _: True,
            release=self.released.append,
            publish=lambda positions: self.published.append(dict(positions)) or True,
        )
        plugin._lock = threading.Lock()
        plugin._status = {"state": "running"}
        plugin._max_step = 0.1
        plugin._publish_rate = 20
        plugin._hold_repetitions = 1
        plugin._stop_event = None
        plugin._motion_thread = None
        return plugin

    def run_frames(self, action, side="both"):
        plugin = self.plugin()
        with patch.object(gesture.time, "sleep"), patch.object(gesture, "_acp_notify") as notify:
            plugin._run_move(NoWaitEvent(), gesture._build_frames(action, 2),
                             0.4, side, action, "test-action")
        notify.assert_called_once_with(
            "test-action", "completed", {"gesture": action, "side": side}, "arm_gesture")
        self.assertEqual(self.released, ["arm_gesture"])
        self.assertEqual(plugin._status["state"], "idle")
        return self.published

    def test_arms_out_is_discoverable_with_completion_and_both_arm_resources(self):
        schema = gesture.Plugin.__new__(gesture.Plugin).get_tool()["inputSchema"]
        self.assertIn("arms_out", schema["properties"]["action"]["enum"])
        self.assertIn("arms_out", {x["const"] for x in schema["properties"]["action"]["oneOf"]})
        self.assertEqual(schema["x-action-params"]["arms_out"]["params"], ["side", "speed"])
        self.assertIn("arms_out", schema["x-completion"]["actions"])
        self.assertEqual(set(schema["x-resource"]), {"arm_l", "arm_r"})

    def test_horizontal_pose_is_mirrored_and_within_real_urdf_limits(self):
        target = gesture._mirrored_positions("arms_out", "both", gesture._GESTURES_DEG["arms_out"])
        self.assertEqual(len(target), 14)
        self.assertAlmostEqual(target["left_shoulder_roll_joint"], math.pi / 2)
        self.assertAlmostEqual(target["right_shoulder_roll_joint"], -math.pi / 2)
        self.assertEqual(target["left_elbow_pitch_joint"], 0)
        self.assertEqual(target["right_elbow_pitch_joint"], 0)
        self.assertTrue(set(target).issubset(gesture.JOINT_LIMITS))
        self.assertEqual(gesture._validate_pose_rad(target), [])

    def test_worker_finishes_at_horizontal_pose_without_returning_to_neutral(self):
        published = self.run_frames("arms_out")
        expected = gesture._mirrored_positions("arms_out", "both", gesture._GESTURES_DEG["arms_out"])
        self.assertEqual(published[-1], expected)
        neutral = gesture._mirrored_positions("neutral", "both", gesture._NEUTRAL_DEG)
        # The initial snapshot is held once; no later command may return to it.
        self.assertNotIn(neutral, published[1:])

    def test_single_arm_gesture_does_not_move_the_opposite_arm(self):
        for side in ("left", "right"):
            with self.subTest(side=side):
                published = self.run_frames("arms_out", side)
                self.assertTrue(all(name.startswith(side + "_") for name in published[-1]))
                self.assertEqual(len(published[-1]), 7)

    def test_existing_gestures_still_finish_at_neutral(self):
        for action in ("salute", "welcome", "raise", "shake_hands", "high_five"):
            with self.subTest(action=action):
                side = "right" if action == "salute" else "both"
                published = self.run_frames(action, side)
                expected = gesture._mirrored_positions("neutral", side, gesture._NEUTRAL_DEG)
                self.assertEqual(published[-1], expected)

    def test_reset_dispatch_uses_neutral_pose_for_both_arms(self):
        plugin = self.plugin()
        plugin._validate_run = lambda *_: None
        with patch.object(gesture.threading, "Thread") as worker:
            result = plugin.dispatch("reset", {"speed": 0.4})
        self.assertTrue(result["ok"])
        self.assertEqual(worker.call_args.kwargs["args"][1], [(gesture._NEUTRAL_DEG, 1.0, 1.0)])
        self.assertEqual(worker.call_args.kwargs["args"][3:5], ("both", "reset"))

    def test_cancel_holds_the_current_horizontal_pose(self):
        plugin = self.plugin()
        horizontal = gesture._mirrored_positions("arms_out", "both", gesture._GESTURES_DEG["arms_out"])
        plugin._client = SimpleNamespace(snapshot=lambda: {"fresh": True, "joints": horizontal})
        stop_event = threading.Event()
        plugin._stop_event = stop_event
        with patch.object(gesture.time, "sleep"):
            result = plugin.dispatch("cancel", {})
        self.assertTrue(stop_event.is_set())
        self.assertTrue(result["hold_command_published"])
        self.assertEqual(self.published[-1], horizontal)


if __name__ == "__main__":
    unittest.main()
