"""Schema/adapter contract tests for Adam RL locomotion."""

from __future__ import annotations

import sys
import types
import unittest
from pathlib import Path

sys.modules.setdefault("numpy", types.ModuleType("numpy"))
sys.path.insert(0, str(Path(__file__).parent))

from device import (
    ControlModePlugin,
    MotionPlugin,
    PosturePlugin,
    RlLocoPlugin,
    SafetyPlugin,
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
    def test_set_mode_schema_and_dispatch_use_rl_state_names(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)

        mode_schema = plugin.get_tool()["inputSchema"]["properties"]["target_state"]
        self.assertEqual(mode_schema["type"], "string")

        result = plugin.dispatch("set_mode", {"target_state": "STAND_WALK"})
        self.assertEqual(grpc.mode, "STAND_WALK")
        self.assertEqual(result["current_state"], "STAND_WALK")

    def test_full_rl_actions_are_exposed(self):
        grpc = _Grpc()
        plugin = RlLocoPlugin({}, "adam", None, grpc)

        actions = plugin.get_tool()["inputSchema"]["properties"]["action"]["enum"]
        self.assertTrue({"motion", "tracking_motion", "set_control_mode",
                         "get_control_state", "shutdown"}.issubset(actions))
        plugin.dispatch("motion", {"command": "PLAY", "motion_file": "Sources/motion/Greeting.txt"})
        plugin.dispatch("set_control_mode", {"domain_id": 1})
        self.assertEqual(("PLAY", "Sources/motion/Greeting.txt"), grpc.motion)
        self.assertEqual(1, grpc.domain_id)

    def test_focused_execution_cards_are_registered_contracts(self):
        grpc = _Grpc()
        cards = [
            PosturePlugin({}, "adam", None, grpc),
            MotionPlugin({}, "adam", None, grpc),
            TrackingMotionPlugin({}, "adam", None, grpc),
            ControlModePlugin({}, "adam", None, grpc),
            SafetyPlugin({}, "adam", None, grpc),
        ]
        self.assertEqual(
            {"posture", "motion", "tracking_motion", "control_mode", "safety"},
            {card.get_tool()["name"] for card in cards},
        )

    def test_posture_checks_dynamic_switchable_states_and_waits(self):
        grpc = _Grpc()
        plugin = PosturePlugin({}, "adam", None, grpc)
        denied = plugin.dispatch("set_mode", {"target_state": "JOG"})
        self.assertEqual("NOT_ALLOWED", denied["code"])
        completed = plugin.dispatch("wait_mode", {
            "target_state": "STAND_WALK", "timeout_s": 1,
        })
        self.assertTrue(completed["completed"])
        self.assertEqual("STAND_WALK", grpc.mode)

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

    def test_control_mode_and_safety_cards_map_to_explicit_rpcs(self):
        grpc = _Grpc()
        control = ControlModePlugin({}, "adam", None, grpc)
        safety = SafetyPlugin({}, "adam", None, grpc)
        control.dispatch("set_traditional", {})
        self.assertEqual(0, grpc.domain_id)
        safety.dispatch("shutdown", {"force": True})
        self.assertTrue(grpc.shutdown_force)


if __name__ == "__main__":
    unittest.main()
