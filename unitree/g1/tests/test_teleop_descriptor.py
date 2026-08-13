import hashlib
import json
import os
import subprocess
import sys
import unittest
from pathlib import Path

from teleop.descriptor import (
    PROFILE_ID,
    canonical_json,
    capability_binding,
    tool_definitions,
)


class TeleopDescriptorTests(unittest.TestCase):
    def test_descriptor_import_is_hardware_free_and_mode_specific(self):
        forbidden = {"rclpy", "cyclonedds", "aiortc", "pinocchio", "casadi"}
        # Discovery imports every test module, including the aiortc scenario.
        # Prove the descriptor boundary in a fresh interpreter instead of
        # relying on suite execution order.
        probe = subprocess.run(
            [
                sys.executable,
                "-c",
                (
                    "import json,sys; import teleop.descriptor; "
                    f"print(json.dumps(sorted(set({sorted(forbidden)!r}) & set(sys.modules))))"
                ),
            ],
            cwd=Path(__file__).resolve().parents[1],
            env={**os.environ, "PYTHONDONTWRITEBYTECODE": "1"},
            check=True,
            capture_output=True,
            text=True,
        )
        self.assertEqual([], json.loads(probe.stdout))
        for mode, protocol, dispatch in (
            (
                "shadow",
                "motus.teleop.shadow.v1",
                "motus.teleop.dispatch.recording.v1",
            ),
            (
                "live",
                "motus.teleop.live.v1",
                "motus.teleop.dispatch.hardware.v1",
            ),
        ):
            session_tool, state_tool, ik_tool = tool_definitions(
                mode=mode,
                robot_id="g1-test",
            )
            self.assertEqual(
                [session_tool["name"], state_tool["name"], ik_tool["name"]],
                ["teleop_session", "teleop_state", "teleop_ik"],
            )
            self.assertFalse(session_tool["multiInstance"])
            x_teleop = session_tool["x-teleop"]
            self.assertEqual(x_teleop["protocol"], protocol)
            self.assertEqual(x_teleop["dispatch_contract"], dispatch)
            self.assertEqual(x_teleop["profile_id"], PROFILE_ID)
            self.assertEqual(x_teleop["signaling"]["audience"], "motus-teleop-rtc")
            self.assertEqual(
                "/ws/teleop-capture",
                x_teleop["signaling"]["path"],
            )
            self.assertEqual(
                "paired-capture-credential-only",
                x_teleop["signaling"]["access"],
            )
            self.assertEqual(
                set(x_teleop),
                {
                    "protocol", "mode", "profile_id", "capabilities",
                    "dispatch_contract", "signaling", "driver_id", "driver_name",
                    "robot_id", "actuation_enabled", "capability_digest",
                },
            )
            capabilities = x_teleop["capabilities"]
            self.assertEqual(set(capabilities), {"profile_id", "input_bindings", "outputs", "effectors"})
            self.assertEqual(capabilities["outputs"]["dual_arm"]["joint_count"], 10)
            self.assertFalse(capabilities["outputs"]["base"]["enabled"])
            self.assertFalse(capabilities["outputs"]["hands"]["enabled"])
            actions = session_tool["inputSchema"]["properties"]["action"]["enum"]
            self.assertIn("start", actions)
            self.assertIn("info", actions)
            self.assertIn("instance_id", session_tool["inputSchema"]["properties"])
            self.assertEqual(
                [
                    "start", "info", "stop", "pair_headset", "revoke_headset",
                    "pause", "release", "emergency_stop", "status",
                ],
                actions,
            )
            for private in ("boot_id", "session_id", "epoch", "fence"):
                self.assertNotIn(private, session_tool["inputSchema"]["properties"])
            expected_digest = hashlib.sha256(canonical_json(capability_binding(mode))).hexdigest()
            self.assertEqual(x_teleop["capability_digest"], expected_digest)
            self.assertEqual("processor", ik_tool["type"])
            self.assertEqual(
                ["start", "info", "stop", "solve", "self_test", "reset", "status"],
                ik_tool["inputSchema"]["properties"]["action"]["enum"],
            )
            self.assertEqual(
                ["start", "info", "stop", "status"],
                state_tool["inputSchema"]["properties"]["action"]["enum"],
            )
            self.assertIn("frame_json", ik_tool["inputSchema"]["properties"])
            self.assertNotIn("head", ik_tool["inputSchema"]["properties"])
            self.assertEqual(
                ["frame_json"],
                ik_tool["inputSchema"]["x-action-params"]["solve"]["params"],
            )
            self.assertEqual(
                [],
                ik_tool["inputSchema"]["x-action-params"]["self_test"]["params"],
            )
            self.assertFalse(
                ik_tool["x-teleop-diagnostic"]["diagnostic_hardware_output"]
            )
            self.assertFalse(
                ik_tool["x-teleop-diagnostic"]["diagnostic_publisher_present"]
            )
            self.assertTrue(ik_tool["x-teleop-diagnostic"]["shares_realtime_ik"])

    def test_signaling_cannot_be_advertised_when_unavailable(self):
        with self.assertRaisesRegex(ValueError, "cannot be advertised"):
            tool_definitions(mode="shadow", signaling_enabled=False)


if __name__ == "__main__":
    unittest.main()
