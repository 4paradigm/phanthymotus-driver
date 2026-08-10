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
        for mode, prepare, protocol, dispatch in (
            (
                "shadow",
                "prepare_shadow",
                "motus.teleop.shadow.v1",
                "motus.teleop.dispatch.recording.v1",
            ),
            (
                "live",
                "prepare_live",
                "motus.teleop.live.v1",
                "motus.teleop.dispatch.hardware.v1",
            ),
        ):
            session_tool, state_tool = tool_definitions(mode=mode, robot_id="g1-test")
            self.assertEqual([session_tool["name"], state_tool["name"]], ["teleop_session", "teleop_state"])
            self.assertFalse(session_tool["multiInstance"])
            x_teleop = session_tool["x-teleop"]
            self.assertEqual(x_teleop["protocol"], protocol)
            self.assertEqual(x_teleop["dispatch_contract"], dispatch)
            self.assertEqual(x_teleop["profile_id"], PROFILE_ID)
            self.assertEqual(x_teleop["signaling"]["audience"], "motus-teleop-rtc")
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
            self.assertIn(prepare, actions)
            self.assertNotIn("prepare_live" if mode == "shadow" else "prepare_shadow", actions)
            expected_digest = hashlib.sha256(canonical_json(capability_binding(mode))).hexdigest()
            self.assertEqual(x_teleop["capability_digest"], expected_digest)

    def test_signaling_cannot_be_advertised_when_unavailable(self):
        with self.assertRaisesRegex(ValueError, "cannot be advertised"):
            tool_definitions(mode="shadow", signaling_enabled=False)


if __name__ == "__main__":
    unittest.main()
