import math
import sys
import tempfile
import threading
import time
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from control import (  # noqa: E402
    MOTION_STATES,
    T800_JOINT_NAMES,
    WALK_MOTION_STATES,
    RepeatingCommand,
    action_schema,
    clamp,
    float_list,
    joint_payload,
    ramped_step,
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

    def test_action_schema_can_declare_x_completion(self):
        schema = action_schema(
            {"move": (["vx"], "移动"), "stop": ([], "停止")},
            {"vx": {"type": "number"}},
            "运动",
            completion={"actions": ["move"], "timeout": 60},
        )
        self.assertEqual({"actions": ["move"], "timeout": 60}, schema["x-completion"])

    def test_action_schema_without_completion_has_no_x_completion(self):
        schema = action_schema({"stop": ([], "停止")}, {}, "运动")
        self.assertNotIn("x-completion", schema)

    def test_sensor_tool_has_read_only_topic_contract(self):
        tool = sensor_tool("imu", "IMU", "/robot/state/imu", "data/json")
        self.assertTrue(tool["readOnly"])
        self.assertEqual("sensor", tool["type"])
        self.assertEqual("/robot/state/imu", tool["topic_out"][0]["topic"])

    def test_motion_states_match_official_t800_list(self):
        self.assertEqual(
            ("idle", "passive", "pd_stand", "rl_basic", "lower_body_balance",
             "joint_bridge", "pd_sitground", "walk_server",
             "rl_mimic_supine_to_stance", "rl_mimic_prone_to_stance",
             "rl_mimic_stance_to_supine", "rl_mimic_sitdown_to_stance",
             "rl_mimic_stance_to_sitdown",
             "rl_amp", "rl_terrain", "rl_recover_prone", "rl_floor_sitting"),
            MOTION_STATES,
        )

    def test_walk_motion_states_exclude_invented_names(self):
        self.assertEqual(("rl_basic", "lower_body_balance"), WALK_MOTION_STATES)
        self.assertNotIn("walk", WALK_MOTION_STATES)
        self.assertNotIn("dance", WALK_MOTION_STATES)


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

    def test_renew_retargets_active_stream_without_restart(self):
        published = []
        stream = RepeatingCommand(published.append, lambda: None, rate_hz=100)
        first = stream.start({"value": 1}, 0.2)
        time.sleep(0.03)
        count_before = len(published)
        snapshot = stream.renew({"value": 2}, 0.2)
        self.assertTrue(snapshot.active)
        self.assertEqual(first.started_at, snapshot.started_at)  # 未重启：起始时刻不变
        time.sleep(0.05)
        # renew 无零发布、无重启：后续 tick 直接到达新目标
        self.assertGreater(len(published), count_before)
        for message in published[count_before:]:
            self.assertNotEqual(0, message["value"])
        self.assertIn({"value": 2}, published[count_before:])
        self.assertGreater(stream.snapshot().deadline, time.monotonic())

    def test_renew_falls_back_to_start_when_inactive(self):
        published = []
        stream = RepeatingCommand(published.append, lambda: None, rate_hz=100)
        snapshot = stream.renew({"value": 1}, 0.05)
        self.assertTrue(snapshot.active)
        self.assertEqual({"value": 1}, published[0])  # start 回退的同步首发

    def test_renew_after_expiry_still_activates_stream(self):
        published = []
        stream = RepeatingCommand(published.append, lambda: None, rate_hz=100)
        stream.start({"value": 1}, 0.03)
        time.sleep(0.035)  # 越过原截止时间：worker 到期收流
        snapshot = stream.renew({"value": 1}, 0.05)
        self.assertTrue(snapshot.active)
        self.assertGreaterEqual(snapshot.deadline, time.monotonic() + 0.04)


class RampTests(unittest.TestCase):
    def test_ramped_step_steps_toward_target(self):
        stepped = ramped_step(
            {"vx": 0.0, "vy": 0.0, "vyaw": 0.0},
            {"vx": 1.0, "vy": 0.0, "vyaw": -0.5},
            (1.0, 1.0, 1.0), 0.01)
        self.assertAlmostEqual(0.01, stepped["vx"])
        self.assertAlmostEqual(-0.01, stepped["vyaw"])

    def test_ramped_step_never_overshoots(self):
        stepped = ramped_step(
            {"vx": 0.99, "vy": 0.0, "vyaw": 0.0},
            {"vx": 1.0, "vy": 0.0, "vyaw": 0.0},
            (1.0, 1.0, 1.0), 0.05)
        self.assertAlmostEqual(1.0, stepped["vx"])

    def test_ramped_step_none_is_instant(self):
        target = {"vx": 0.2, "vy": -0.1, "vyaw": 0.3}
        self.assertEqual(target, ramped_step({"vx": 0.0, "vy": 0.0, "vyaw": 0.0}, target, None, 0.5))

    def test_ramped_step_nonpositive_dt_keeps_current(self):
        current = {"vx": 0.5, "vy": 0.0, "vyaw": 0.0}
        self.assertEqual(current, ramped_step(current, {"vx": 1.0, "vy": 0.0, "vyaw": 0.0}, (1.0, 1.0, 1.0), 0.0))


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
