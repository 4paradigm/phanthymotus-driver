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
    MOTION_STATES,
    WALK_MOTION_STATES,
    T800_JOINT_POSITION_LIMITS,
    T800_JOINT_VELOCITY_LIMITS,
    T800_JOINT_NAMES,
    ControlValidationError,
    RepeatingCommand,
    action_schema,
    clamp,
    float_list,
    joint_payload,
    resample_joint_trajectory,
    sensor_tool,
    validate_joint_indices,
    validate_joint_positions,
    validate_locomotion_request,
    validate_recording_document,
    validate_recording_frames,
)
from native_sdk import NativeSdkManager  # noqa: E402

SAFE_T800_POSITIONS = [
    (lower + upper) / 2.0
    for lower, upper in T800_JOINT_POSITION_LIMITS
]


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

    def test_recording_frames_are_normalized_and_bounded(self):
        frames = validate_recording_frames(
            [
                {"timestamp": 0, "positions": SAFE_T800_POSITIONS},
                {"timestamp": 50, "positions": SAFE_T800_POSITIONS},
            ],
            max_frames=2,
            minimum_frames=2,
        )
        self.assertEqual(0.0, frames[0]["timestamp"])
        self.assertEqual(SAFE_T800_POSITIONS, frames[0]["positions"])
        with self.assertRaisesRegex(ValueError, "maximum frame count"):
            validate_recording_frames(frames * 2, max_frames=2)
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            validate_recording_frames(
                [
                    {"timestamp": 0, "positions": SAFE_T800_POSITIONS},
                    {"timestamp": 0, "positions": SAFE_T800_POSITIONS},
                ],
                max_frames=2,
            )

    def test_locomotion_validation_rejects_schema_bypass_types(self):
        with self.assertRaises(ControlValidationError) as boolean_error:
            validate_locomotion_request(
                "move", {"vx": True}, limits=(1, 1, 1),
                max_timed_duration_sec=3,
            )
        self.assertEqual("INVALID_ARGUMENT", boolean_error.exception.code)
        with self.assertRaises(ControlValidationError) as limit_error:
            validate_locomotion_request(
                "move", {"vx": 2}, limits=(1, 1, 1),
                max_timed_duration_sec=3,
            )
        self.assertEqual("SAFETY_LIMIT", limit_error.exception.code)
        command = validate_locomotion_request(
            "arc",
            {"radius_m": 1, "angle_rad": -0.5, "linear_speed_m_s": -0.2},
            limits=(1, 1, 1),
            max_timed_duration_sec=3,
        )
        self.assertEqual(-0.2, command["vx"])
        self.assertEqual(-0.2, command["vyaw"])

    def test_recording_document_drops_unknown_payload_and_caps_duration(self):
        frames, metadata = validate_recording_document(
            {
                "metadata": {"label": "legacy"},
                "frames": [
                    {"timestamp": 0, "positions": SAFE_T800_POSITIONS, "unknown": {"large": "ignored"}},
                    {"timestamp": 50, "positions": SAFE_T800_POSITIONS},
                ],
            },
            max_frames=2,
            minimum_frames=2,
            max_duration_ms=100,
        )
        self.assertEqual({"timestamp", "positions"}, set(frames[0]))
        self.assertEqual("legacy", metadata["label"])
        with self.assertRaisesRegex(ValueError, "duration"):
            validate_recording_document(
                {"frames": [
                    {"timestamp": 0, "positions": SAFE_T800_POSITIONS},
                    {"timestamp": 101, "positions": SAFE_T800_POSITIONS},
                ]},
                max_frames=2,
                minimum_frames=2,
                max_duration_ms=100,
            )

    def test_resampler_rejects_excessive_sample_allocation(self):
        with self.assertRaisesRegex(ValueError, "sample count"):
            resample_joint_trajectory(
                [
                    {"timestamp": 0, "positions": [0.0] * 25},
                    {"timestamp": 10_000, "positions": [0.1] * 25},
                ],
                joint_indices=list(range(12, 25)),
                current_positions=[0.0] * 25,
                playback_rate_hz=100,
                speed_scale=1.0,
                entry_blend_sec=0.5,
                max_samples=100,
            )

    def test_resampler_rejects_derived_velocity_over_urdf_limit(self):
        start = list(SAFE_T800_POSITIONS)
        finish = list(SAFE_T800_POSITIONS)
        start[12] = -1.0
        finish[12] = 1.0
        with self.assertRaisesRegex(ValueError, "derived joint velocity"):
            resample_joint_trajectory(
                [
                    {"timestamp": 0, "positions": start},
                    {"timestamp": 10, "positions": finish},
                ],
                joint_indices=list(range(12, 25)),
                current_positions=start,
                playback_rate_hz=100,
                speed_scale=1.0,
                entry_blend_sec=0.5,
                max_samples=1000,
            )

    def test_resampler_rejects_single_sample_position_spike(self):
        frames = []
        for index, torso_yaw in enumerate((0.0, 0.0, 0.4, 0.0, 0.0)):
            positions = list(SAFE_T800_POSITIONS)
            positions[12] = torso_yaw
            frames.append({"timestamp": index * 10, "positions": positions})
        with self.assertRaisesRegex(ValueError, "target-step velocity"):
            resample_joint_trajectory(
                frames,
                joint_indices=list(range(12, 25)),
                current_positions=frames[0]["positions"],
                playback_rate_hz=100,
                speed_scale=1.0,
                entry_blend_sec=0.5,
                max_samples=1000,
            )

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
        urdf_velocity_limits = {
            joint.attrib["name"]: float(joint.find("limit").attrib["velocity"])
            for joint in root.findall("joint")
            if joint.find("limit") is not None
        }
        for index, name in enumerate(T800_JOINT_NAMES):
            self.assertEqual(
                urdf_velocity_limits[name],
                T800_JOINT_VELOCITY_LIMITS[index],
            )

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

    def test_resample_joint_trajectory_adds_smooth_entry_and_high_rate_samples(self):
        frames = [
            {"timestamp": 0, "positions": [0.0] * 25},
            {"timestamp": 100, "positions": [0.0] * 12 + [1.0] * 13},
            {"timestamp": 200, "positions": [0.0] * 25},
        ]
        samples = resample_joint_trajectory(
            frames,
            joint_indices=list(range(12, 25)),
            current_positions=[0.0] * 12 + [-0.5] * 13,
            playback_rate_hz=100.0,
            speed_scale=1.0,
            entry_blend_sec=0.5,
        )
        self.assertGreater(len(samples), len(frames))
        self.assertAlmostEqual(-0.5, samples[0][0][0], places=3)
        self.assertTrue(all(0.0 <= position[0] <= 1.0 for position, _ in samples[51:]))

    def test_resample_joint_trajectory_rejects_non_monotonic_timestamps(self):
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            resample_joint_trajectory(
                [
                    {"timestamp": 0, "positions": [0.0] * 25},
                    {"timestamp": 0, "positions": [0.1] * 25},
                ],
                joint_indices=list(range(12, 25)),
                current_positions=[0.0] * 25,
                playback_rate_hz=100.0,
                speed_scale=1.0,
                entry_blend_sec=0.5,
            )

    def test_resample_joint_trajectory_rejects_missing_current_state(self):
        with self.assertRaisesRegex(ValueError, "current joint state is unavailable"):
            resample_joint_trajectory(
                [
                    {"timestamp": 0, "positions": [0.0] * 25},
                    {"timestamp": 50, "positions": [0.1] * 25},
                ],
                joint_indices=list(range(12, 25)),
                current_positions=[],
                playback_rate_hz=100.0,
                speed_scale=1.0,
                entry_blend_sec=0.5,
            )
    def test_action_schema_with_completion(self):
        schema = action_schema(
            {"move": (["vx"], "move"), "stop": ([], "stop")},
            {"vx": {"type": "number"}},
            "action",
            completion=(["move"], 60),
        )
        self.assertEqual(["move"], schema["x-completion"]["actions"])
        self.assertEqual(60, schema["x-completion"]["timeout"])

    def test_action_schema_without_completion_omits_key(self):
        schema = action_schema(
            {"move": (["vx"], "move")},
            {"vx": {"type": "number"}},
            "action",
        )
        self.assertNotIn("x-completion", schema)

    def test_motion_states_has_17_official_values(self):
        self.assertEqual(17, len(MOTION_STATES))
        self.assertIn("rl_basic", MOTION_STATES)
        self.assertIn("lower_body_balance", MOTION_STATES)
        self.assertIn("rl_mimic_supine_to_stance", MOTION_STATES)
        self.assertIn("rl_mimic_stance_to_supine", MOTION_STATES)

    def test_walk_motion_states_matches_official_walking_modes(self):
        self.assertEqual(("rl_basic", "lower_body_balance"), WALK_MOTION_STATES)


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
        for duration in (-2, -0.5):
            with self.subTest(duration=duration):
                with self.assertRaisesRegex(ValueError, "duration"):
                    stream.start({}, duration)

    def test_background_publish_failure_is_exposed_and_stops_stream(self):
        published = []
        stopped = threading.Event()

        def publish(command):
            published.append(command)
            if len(published) > 1:
                raise RuntimeError("publisher failed")

        stream = RepeatingCommand(publish, stopped.set, rate_hz=100)
        stream.start({"value": 1}, 1.0)
        self.assertTrue(stopped.wait(0.5))
        snapshot = stream.snapshot()
        self.assertFalse(snapshot.active)
        self.assertEqual("publisher failed", snapshot.error)


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
