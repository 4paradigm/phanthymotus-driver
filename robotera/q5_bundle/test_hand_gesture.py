"""ROS-free regression tests for gesture-to-control integration."""

import unittest
from unittest.mock import Mock

import hand_control
import hand_gesture


class Client:
    def get_lifecycle_state(self):
        return "active"

    def snapshot(self):
        return {"fresh": True, "joints": dict.fromkeys(hand_control.HAND_JOINTS, 0.0)}

    def sensor_snapshot(self, name):
        return {"available": True, "fresh": True, "state": 3}


class HandGestureTests(unittest.TestCase):
    def setUp(self):
        self.plugin = hand_gesture.Plugin({}, "test", None, Client())
        self.control = self.plugin._control
        self.control._router.status = Mock(return_value={"ros_publisher_available": True})
        # Keep real dispatch, safety checks and target validation; intercept only
        # the final motion launch so these tests never publish robot commands.
        self.control._start_motion = Mock(return_value={"ok": True, "state": "moving"})

    def test_every_preset_and_side_reaches_validated_motion(self):
        for gesture, preset in hand_gesture.PRESETS.items():
            for side in ("left", "right", "both"):
                with self.subTest(gesture=gesture, side=side):
                    result = self.plugin.dispatch(gesture, {"side": side})
                    self.assertTrue(result["ok"])
                    self.assertEqual((result["gesture"], result["side"]), (gesture, side))
                    command, source, action_id = self.control._start_motion.call_args.args
                    expected = {}
                    for hand in ("left", "right"):
                        if side in (hand, "both"):
                            expected.update(preset[hand])
                    self.assertEqual(command["targets"], expected)
                    self.assertEqual(source, gesture)
                    self.assertTrue(action_id)

    def test_unavailable_ros_returns_structured_failure(self):
        self.control._router.status.return_value = {"ros_publisher_available": False}
        result = self.plugin.dispatch("light_grip", {})
        self.assertEqual(result["code"], "ROS_UNAVAILABLE")
        self.control._start_motion.assert_not_called()

    def test_unready_robot_blocks_gesture(self):
        self.control._client.sensor_snapshot = lambda name: {"available": True, "fresh": True, "state": 7}
        self.assertEqual(self.plugin.dispatch("light_grip", {})["code"], "Q5_FSM_NOT_READY")
        self.control._start_motion.assert_not_called()

    def test_joint_validation_still_blocks_missing_joints(self):
        self.control._client.snapshot = lambda: {"fresh": True, "joints": {}}
        self.assertEqual(self.plugin.dispatch("light_grip", {})["code"], "JOINT_UNAVAILABLE")
        self.control._start_motion.assert_not_called()

    def test_public_hand_control_still_works_without_exposing_set(self):
        self.assertTrue(self.control.dispatch("open_hand", {"side": "left"})["ok"])
        actions = self.control.get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertNotIn("set", actions)
        self.assertIsNone(self.control.dispatch("set", {}))


if __name__ == "__main__":
    unittest.main()
