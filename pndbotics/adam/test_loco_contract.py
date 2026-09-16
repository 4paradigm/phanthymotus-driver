"""Schema/adapter contract tests for Adam RL locomotion."""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

sys.modules.setdefault("numpy", types.ModuleType("numpy"))
sys.path.insert(0, str(Path(__file__).parent))

from device import (
    MotionPlugin,
    RlLocoPlugin,
    TrackingMotionPlugin,
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


class LocoContractTests(unittest.TestCase):
    def test_loco_hides_mode_and_automatically_enters_walking_state(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)

        schema = plugin.get_tool()["inputSchema"]
        self.assertEqual(["move", "set_height", "stop"], schema["properties"]["action"]["enum"])
        self.assertNotIn("target_state", schema["properties"])
        self.assertNotIn("set_mode", schema["x-action-params"])

        result = plugin.dispatch("move", {"vx": 0.2, "vy": 0.0, "vyaw": -0.1})
        self.assertEqual(grpc.mode, "STAND_WALK")
        self.assertEqual((0.2, 0.0, -0.1), grpc.velocity)
        self.assertTrue(result["success"])

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
            TrackingMotionPlugin({}, "adam", None, grpc),
        ]
        self.assertEqual(
            {"motion", "tracking_motion"},
            {card.get_tool()["name"] for card in cards},
        )

    def test_motion_and_tracking_cards_use_robot_side_files(self):
        grpc = _Grpc()
        motion = MotionPlugin({}, "adam", None, grpc)
        tracking = TrackingMotionPlugin({}, "adam", None, grpc)
        motion.dispatch("play", {"motion_file": "Sources/motion/Wave.txt"})
        tracking.dispatch("play", {"motion_file": "Sources/tracking/Walk.txt"})
        self.assertEqual(("PLAY", "Sources/motion/Wave.txt"), grpc.motion)
        self.assertEqual("Sources/tracking/Walk.txt", grpc.tracking_motion)
        motion.dispatch("stop", {})
        self.assertEqual(("STOP", ""), grpc.motion)

if __name__ == "__main__":
    unittest.main()
