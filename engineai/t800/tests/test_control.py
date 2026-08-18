import math
import sys
import tempfile
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from control import (  # noqa: E402
    T800_JOINT_POSITION_LIMITS,
    T800_JOINT_NAMES,
    RepeatingCommand,
    action_schema,
    clamp,
    float_list,
    joint_payload,
    sensor_tool,
    validate_joint_indices,
    validate_joint_positions,
)
from native_sdk import NativeSdkManager  # noqa: E402


class ValidationTests(unittest.TestCase):
    def test_joint_layout_is_complete_and_unique(self):
        self.assertEqual(25, len(T800_JOINT_NAMES))
        self.assertEqual(25, len(set(T800_JOINT_NAMES)))
        self.assertEqual("J00_HIP_PITCH_L", T800_JOINT_NAMES[0])
        self.assertEqual("J24_HEAD_YAW", T800_JOINT_NAMES[-1])
        self.assertEqual(25, len(T800_JOINT_POSITION_LIMITS))

    def test_joint_payload_uses_official_index_mapping(self):
        payload = joint_payload([0.1, 0.2], [1.0, 2.0], [3.0, 4.0], timestamp_ms=123)
        self.assertEqual(123, payload["timestamp_ms"])
        self.assertEqual("J01_HIP_ROLL_L", payload["joints"][1]["name"])
        self.assertEqual(4.0, payload["joints"][1]["tau"])

    def test_joint_payload_rejects_mismatched_arrays(self):
        with self.assertRaisesRegex(ValueError, "same length"):
            joint_payload([0.0], [], [0.0])

    def test_joint_payload_rejects_more_than_t800_layout(self):
        values = [0.0] * 26
        with self.assertRaisesRegex(ValueError, "more than 25"):
            joint_payload(values, values, values)

    def test_clamp_and_finite_validation(self):
        self.assertEqual(1.0, clamp(4.0, -1.0, 1.0))
        self.assertEqual(-1.0, clamp(-4.0, -1.0, 1.0))
        with self.assertRaisesRegex(ValueError, "finite"):
            clamp(math.nan, -1.0, 1.0)

    def test_float_list_validates_size_and_values(self):
        self.assertEqual([1.0, 2.0], float_list([1, 2], "values", size=2))
        with self.assertRaisesRegex(ValueError, "exactly 2"):
            float_list([1], "values", size=2)
        with self.assertRaisesRegex(ValueError, "finite"):
            float_list([math.inf], "values")

    def test_joint_indices_validate_range_and_uniqueness(self):
        self.assertEqual([0, 24], validate_joint_indices([0, 24]))
        with self.assertRaisesRegex(ValueError, "unique"):
            validate_joint_indices([1, 1])
        with self.assertRaisesRegex(ValueError, "out of range"):
            validate_joint_indices([25])
        with self.assertRaisesRegex(ValueError, "integers"):
            validate_joint_indices([1.5])

    def test_joint_positions_enforce_urdf_limits_and_margin(self):
        indices, positions = validate_joint_positions(
            [13, 16], [-1.2, -1.8], limit_margin_rad=0.02
        )
        self.assertEqual([13, 16], indices)
        self.assertEqual([-1.2, -1.8], positions)
        with self.assertRaisesRegex(ValueError, "J16_ELBOW_PITCH_L"):
            validate_joint_positions([16], [-2.28], limit_margin_rad=0.02)

    def test_joint_position_limits_match_vendored_urdf(self):
        root = ET.parse(ROOT / "resource" / "serial_t800.urdf").getroot()
        urdf_limits = {
            joint.attrib["name"]: (
                float(joint.find("limit").attrib["lower"]),
                float(joint.find("limit").attrib["upper"]),
            )
            for joint in root.findall("joint")
            if joint.find("limit") is not None
        }
        for index, name in enumerate(T800_JOINT_NAMES):
            self.assertEqual(urdf_limits[name], T800_JOINT_POSITION_LIMITS[index])

    def test_action_schema_splits_action_parameters(self):
        schema = action_schema(
            {"move": (["vx"], "move"), "stop": ([], "stop")},
            {"vx": {"type": "number"}},
            "action",
        )
        self.assertEqual(["move", "stop"], schema["properties"]["action"]["enum"])
        self.assertEqual(["vx"], schema["x-action-params"]["move"]["params"])

    def test_sensor_tool_has_read_only_topic_contract(self):
        tool = sensor_tool("imu", "IMU", "/robot/state/imu", "data/json")
        self.assertTrue(tool["readOnly"])
        self.assertEqual("sensor", tool["type"])
        self.assertEqual("/robot/state/imu", tool["topic_out"][0]["topic"])


class RepeatingCommandTests(unittest.TestCase):
    def test_timed_stream_publishes_and_stops(self):
        published = []
        stopped = threading.Event()
        stream = RepeatingCommand(published.append, stopped.set, rate_hz=100)
        stream.start({"value": 1}, 0.04)
        self.assertTrue(stopped.wait(0.5))
        self.assertGreaterEqual(len(published), 2)
        self.assertFalse(stream.snapshot().active)

    def test_continuous_stream_stops_explicitly(self):
        published = []
        stops = []
        stream = RepeatingCommand(published.append, lambda: stops.append(True), rate_hz=100)
        stream.start({"value": 1}, -1)
        time.sleep(0.03)
        self.assertTrue(stream.stop())
        time.sleep(0.03)
        count = len(published)
        time.sleep(0.03)
        self.assertEqual(count, len(published))
        self.assertGreaterEqual(len(stops), 1)

    def test_zero_duration_is_stop_only(self):
        published = []
        stream = RepeatingCommand(published.append, lambda: None, rate_hz=10)
        snapshot = stream.start({"value": 1}, 0)
        self.assertFalse(snapshot.active)
        self.assertEqual([], published)

    def test_invalid_duration_is_rejected(self):
        stream = RepeatingCommand(lambda _: None, lambda: None, rate_hz=10)
        with self.assertRaisesRegex(ValueError, "duration"):
            stream.start({}, -2)


class NativeSdkManagerTests(unittest.TestCase):
    def test_external_mode_is_observation_only(self):
        manager = NativeSdkManager({"mode": "external", "source_revision": "abc"})
        self.assertEqual("external", manager.status()["state"])
        self.assertEqual(["status"], manager.tool()["inputSchema"]["properties"]["action"]["enum"])

    def test_process_mode_starts_and_stops_child_group(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = NativeSdkManager({
                "mode": "process",
                "workdir": directory,
                "command": ["/bin/sleep", "5"],
                "stop_timeout": 1,
            })
            started = manager.start()
            self.assertEqual("running", started["state"])
            self.assertIsInstance(started["pid"], int)
            stopped = manager.stop()
            self.assertEqual("stopped", stopped["state"])

    def test_invalid_process_command_fails_cleanly(self):
        with tempfile.TemporaryDirectory() as directory:
            manager = NativeSdkManager({"mode": "process", "workdir": directory, "command": []})
            with self.assertRaisesRegex(ValueError, "non-empty"):
                manager.start()


if __name__ == "__main__":
    unittest.main()
