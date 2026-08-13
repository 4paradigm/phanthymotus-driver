import math
import sys
import tempfile
import threading
import time
import types
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from control import (  # noqa: E402
    T800_JOINT_NAMES,
    OccupancyGrid2D,
    RepeatingCommand,
    action_schema,
    clamp,
    extract_xyz_from_pointcloud2,
    float_list,
    joint_payload,
    normalize_odometry_payload,
    pack_sensor_mapping_binary,
    quaternion_to_yaw_rad,
    sensor_tool,
    validate_joint_indices,
)
from native_sdk import NativeSdkManager  # noqa: E402


class ValidationTests(unittest.TestCase):
    def test_joint_layout_is_complete_and_unique(self):
        self.assertEqual(25, len(T800_JOINT_NAMES))
        self.assertEqual(25, len(set(T800_JOINT_NAMES)))
        self.assertEqual("J00_HIP_PITCH_L", T800_JOINT_NAMES[0])
        self.assertEqual("J24_HEAD_YAW", T800_JOINT_NAMES[-1])

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

    def test_odometry_payload_preserves_frame_and_yaw(self):
        payload = normalize_odometry_payload(
            frame_id="odom",
            child_frame_id="base_link",
            position=[1.0, 2.0, 0.1],
            orientation=[0.0, 0.0, math.sin(0.5 / 2), math.cos(0.5 / 2)],
            linear_velocity=[0.2, 0.0, 0.0],
            angular_velocity=[0.0, 0.0, 0.1],
            stamp_sec=10,
            stamp_nanosec=500_000_000,
            received_monotonic=100.0,
            stale_timeout_sec=1.0,
            now_monotonic=100.2,
        )
        self.assertEqual("odom", payload["frame_id"])
        self.assertEqual("base_link", payload["child_frame_id"])
        self.assertAlmostEqual(0.5, payload["yaw_rad"], places=3)
        self.assertAlmostEqual(0.2, payload["age_sec"])
        self.assertFalse(payload["stale"])
        self.assertEqual(10500, payload["timestamp_ms"])

    def test_odometry_payload_marks_stale_after_timeout(self):
        payload = normalize_odometry_payload(
            frame_id="map",
            child_frame_id="body",
            position=[0.0, 0.0, 0.0],
            orientation=[0.0, 0.0, 0.0, 1.0],
            linear_velocity=[0.0, 0.0, 0.0],
            angular_velocity=[0.0, 0.0, 0.0],
            received_monotonic=10.0,
            stale_timeout_sec=0.5,
            now_monotonic=11.0,
        )
        self.assertTrue(payload["stale"])
        self.assertAlmostEqual(0.0, quaternion_to_yaw_rad(0.0, 0.0, 0.0, 1.0))

    def test_occupancy_grid_filters_height_and_packs_mapping_binary(self):
        import struct

        grid = OccupancyGrid2D(resolution_m=0.1, z_min_m=0.1, z_max_m=1.0, min_hits=1)
        accepted = grid.ingest_points(
            [
                (0.05, 0.05, 0.5),   # keep
                (0.05, 0.05, 2.0),   # too high
                (0.05, 0.05, 0.0),   # too low
                (1.05, 0.05, 0.4),   # keep
            ],
            frame_id="map",
        )
        self.assertEqual(2, accepted)
        snap = grid.snapshot()
        self.assertEqual("map", snap["frame_id"])
        self.assertEqual(2, snap["occupied"])
        centers = grid.occupied_cell_centers()
        self.assertEqual(2, len(centers))

        fields = [
            types.SimpleNamespace(name="x", offset=0, datatype=7),
            types.SimpleNamespace(name="y", offset=4, datatype=7),
            types.SimpleNamespace(name="z", offset=8, datatype=7),
        ]
        cloud = struct.pack("<fff", 0.2, 0.3, 0.5)
        parsed = extract_xyz_from_pointcloud2(fields, 12, 1, 1, cloud)
        self.assertEqual(1, len(parsed))
        self.assertAlmostEqual(0.2, parsed[0][0], places=5)
        self.assertAlmostEqual(0.3, parsed[0][1], places=5)
        self.assertAlmostEqual(0.5, parsed[0][2], places=5)

        packed = pack_sensor_mapping_binary(1.0, 2.0, 0.25, centers)
        rx, ry, ryaw, flags, count = struct.unpack_from("<fffBI", packed, 0)
        self.assertAlmostEqual(1.0, rx)
        self.assertAlmostEqual(2.0, ry)
        self.assertEqual(0x03, flags)
        self.assertEqual(2, count)


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
