from __future__ import annotations

import importlib.util
import json
import sys
import time
import types
import unittest
from pathlib import Path


class _Message:
    def __init__(self):
        self.data = ""


class _Publisher:
    def __init__(self):
        self.messages: list[dict] = []

    def publish(self, message):
        self.messages.append(json.loads(message.data))


class _Logger:
    def __init__(self):
        self.warnings: list[str] = []
        self.errors: list[str] = []

    def warn(self, message):
        self.warnings.append(message)

    def error(self, message):
        self.errors.append(message)


class _Node:
    def __init__(self, name):
        self.node_name = name
        self.publisher = _Publisher()
        self.logger = _Logger()

    def create_publisher(self, *_args):
        return self.publisher

    def get_logger(self):
        return self.logger


class _QoSProfile:
    def __init__(self, **kwargs):
        self.options = kwargs


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_executors = types.ModuleType("rclpy.executors")
    rclpy_node.Node = _Node
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = type(
        "ReliabilityPolicy", (), {"BEST_EFFORT": "best_effort"}
    )
    rclpy_qos.HistoryPolicy = type(
        "HistoryPolicy", (), {"KEEP_LAST": "keep_last"}
    )
    rclpy_qos.DurabilityPolicy = type(
        "DurabilityPolicy", (), {"VOLATILE": "volatile"}
    )
    rclpy.executors = rclpy_executors

    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    std_msgs_msg.String = _Message

    audio_msgs = types.ModuleType("audio_msgs")
    audio_msgs_msg = types.ModuleType("audio_msgs.msg")
    audio_msgs_msg.AudioChunk = _Message

    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.CompressedImage = _Message
    sensor_msgs_msg.Image = _Message

    sys.modules.update({
        "rclpy": rclpy,
        "rclpy.node": rclpy_node,
        "rclpy.qos": rclpy_qos,
        "rclpy.executors": rclpy_executors,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
        "audio_msgs": audio_msgs,
        "audio_msgs.msg": audio_msgs_msg,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
    })


def _load_device_module():
    _install_ros_stubs()
    path = Path(__file__).resolve().parents[1] / "device.py"
    spec = importlib.util.spec_from_file_location("bumi_device_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


DEVICE = _load_device_module()


def _load_main_module():
    _install_ros_stubs()
    sys.modules["device"] = DEVICE
    path = Path(__file__).resolve().parents[1] / "main.py"
    spec = importlib.util.spec_from_file_location("bumi_main_for_test", path)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


MAIN = _load_main_module()


class _Joint:
    def __init__(
        self,
        motor_id: int,
        error: int = 0,
        temperature: int = 30,
        velocity: float = 0.0,
    ):
        self.motor_id = motor_id
        self.error = error
        self.temperature = temperature
        self.vel = velocity


class _Controller:
    def __init__(self):
        self.mode = 2
        self.joints = [_Joint(index) for index in range(21)]
        self.fail = False

    def get_mode(self):
        if self.fail:
            raise RuntimeError("dds timeout")
        return self.mode

    def get_joint_state(self):
        return self.joints


class _Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class MotionEventTrackerTest(unittest.TestCase):
    def test_mode_and_protection_transitions_are_deduplicated(self):
        tracker = DEVICE._MotionEventTracker(20)

        self.assertEqual(tracker.observe_mode(2), [])
        self.assertEqual(tracker.observe_mode(2), [])
        self.assertEqual(
            [event["type"] for event in tracker.observe_mode(5)],
            ["workmode_changed"],
        )
        self.assertEqual(
            [event["type"] for event in tracker.observe_mode(26)],
            ["workmode_changed", "protection_entered"],
        )
        self.assertEqual(
            [event["type"] for event in tracker.observe_mode(2)],
            ["workmode_changed", "protection_cleared"],
        )

    def test_fault_appeared_changed_and_cleared(self):
        tracker = DEVICE._MotionEventTracker(20)
        fault_a = {
            "motor_id": 4,
            "joint": "l_leg_pitch_joint",
            "error": 0x0D,
            "error_name": "communication_lost",
            "temperature": 40,
        }
        fault_b = {**fault_a, "error": 0x0E, "error_name": "overload"}

        self.assertEqual(tracker.observe_faults({}), [])
        self.assertEqual(
            [event["type"] for event in tracker.observe_faults({4: fault_a})],
            ["motor_fault_appeared"],
        )
        self.assertEqual(tracker.observe_faults({4: fault_a}), [])
        self.assertEqual(
            [event["type"] for event in tracker.observe_faults({4: fault_b})],
            ["motor_fault_changed"],
        )
        self.assertEqual(
            [event["type"] for event in tracker.observe_faults({})],
            ["motor_fault_cleared"],
        )

    def test_freshness_and_bounded_history(self):
        tracker = DEVICE._MotionEventTracker(2)

        self.assertEqual(tracker.mark_fresh(), [])
        self.assertEqual(
            [event["type"] for event in tracker.mark_stale("timeout")],
            ["state_data_stale"],
        )
        self.assertEqual(tracker.mark_stale("timeout again"), [])
        self.assertEqual(
            [event["type"] for event in tracker.mark_fresh()],
            ["state_data_recovered"],
        )
        tracker.observe_mode(2)
        tracker.observe_mode(5)

        self.assertEqual(len(tracker.history), 2)
        self.assertEqual(tracker.dropped_events, 1)
        self.assertEqual(tracker.cursor, 3)
        self.assertTrue(all(event["timestamp"].endswith("Z") for event in tracker.history))
        self.assertEqual(
            tracker.snapshot,
            {
                "fresh": True,
                "cursor": 3,
                "events": tracker.history,
                "dropped_events": 1,
            },
        )

    def test_joint_activity_transitions_are_deduplicated(self):
        tracker = DEVICE._MotionEventTracker(10)
        stopped = {
            "active": False,
            "velocity_threshold": 0.15,
            "moving_joint_count": 0,
            "moving_joints": [],
            "max_abs_velocity": 0.01,
        }
        moving = {
            "active": True,
            "velocity_threshold": 0.15,
            "moving_joint_count": 1,
            "moving_joints": ["l_arm_pitch_joint"],
            "max_abs_velocity": 0.2,
        }

        self.assertEqual(tracker.observe_activity(stopped), [])
        self.assertEqual(
            [event["type"] for event in tracker.observe_activity(moving)],
            ["joint_activity_started"],
        )
        self.assertEqual(tracker.observe_activity(moving), [])
        self.assertEqual(
            [event["type"] for event in tracker.observe_activity(stopped)],
            ["joint_activity_stopped"],
        )


class MotionEventSnapshotTest(unittest.TestCase):
    def test_snapshot_reports_only_documented_faults(self):
        controller = _Controller()
        controller.mode = 26
        controller.joints[4] = _Joint(104, error=0x0D, temperature=47)
        controller.joints[5] = _Joint(105, error=0x55, temperature=48)

        controller.joints[0].vel = 0.2
        mode, faults, activity = DEVICE._read_motion_event_snapshot(
            controller, 0.15
        )

        self.assertEqual(mode, 26)
        self.assertEqual(set(faults), {104})
        self.assertEqual(faults[104]["joint"], "l_leg_pitch_joint")
        self.assertEqual(faults[104]["error_name"], "communication_lost")
        self.assertTrue(activity["active"])
        self.assertEqual(activity["moving_joints"], ["l_arm_pitch_joint"])

    def test_snapshot_rejects_wrong_joint_count(self):
        controller = _Controller()
        controller.joints.pop()

        with self.assertRaisesRegex(RuntimeError, "returned 20 joints, expected 21"):
            DEVICE._read_motion_event_snapshot(controller, 0.15)


class MotionEventsPluginTest(unittest.TestCase):
    def test_bundle_registers_enabled_motion_events_tool(self):
        controller = _Controller()
        executor = _Executor()
        bundle = MAIN.BumiDeviceBundle(
            {
                "plugins": {
                    "motion_events": {
                        "enabled": True,
                        "poll_interval_s": 0.5,
                        "history_size": 10,
                        "activity_velocity_threshold": 0.15,
                    }
                }
            },
            "bumi_test",
            executor,
            controller,
            None,
        )

        self.assertEqual(
            [tool["name"] for tool in bundle.get_all_tools()],
            ["motion_events"],
        )

    def test_tool_contract_and_event_publication(self):
        controller = _Controller()
        executor = _Executor()
        plugin = DEVICE.MotionEventsPlugin(
            {"poll_interval_s": 0.5, "history_size": 10},
            "bumi_test",
            executor,
            controller,
        )
        node = executor.nodes[0]

        tool = plugin.get_tool()
        self.assertEqual(tool["name"], "motion_events")
        self.assertEqual(tool["type"], "sensor")
        self.assertEqual(
            tool["topic_out"],
            [{"topic": "/bumi_test/motion/events", "format": "data/json"}],
        )

        self.assertEqual(node._poll_once(), [])
        controller.mode = 5
        self.assertEqual(
            [event["type"] for event in node._poll_once()],
            ["workmode_changed"],
        )
        controller.joints[4].error = 0x0D
        self.assertEqual(
            [event["type"] for event in node._poll_once()],
            ["motor_fault_appeared"],
        )
        controller.joints[0].vel = 0.2
        self.assertEqual(
            [event["type"] for event in node._poll_once()],
            ["joint_activity_started"],
        )

        info = plugin.dispatch("info", {})
        self.assertEqual(info["state"], "idle")
        self.assertEqual(info["cursor"], 3)
        self.assertEqual(info["history_count"], 3)
        report = plugin.dispatch("motion_events", {})
        self.assertEqual(len(report["events"]), 3)
        self.assertEqual(report["history_count"], 3)
        self.assertEqual(node.publisher.messages[-1]["cursor"], 3)

    def test_link_failure_and_recovery(self):
        controller = _Controller()
        executor = _Executor()
        plugin = DEVICE.MotionEventsPlugin({}, "bumi_test", executor, controller)
        node = executor.nodes[0]
        node._poll_once()

        controller.fail = True
        self.assertEqual(
            [event["type"] for event in node._poll_once()],
            ["state_data_stale"],
        )
        self.assertEqual(node._poll_once(), [])
        controller.fail = False
        self.assertEqual(
            [event["type"] for event in node._poll_once()],
            ["state_data_recovered"],
        )
        self.assertEqual(len(node.logger.warnings), 1)

    def test_configuration_validation(self):
        controller = _Controller()
        executor = _Executor()

        with self.assertRaisesRegex(ValueError, "poll_interval_s"):
            DEVICE.MotionEventsPlugin(
                {"poll_interval_s": 0.001}, "bumi", executor, controller
            )
        with self.assertRaisesRegex(ValueError, "history_size must be an integer"):
            DEVICE.MotionEventsPlugin(
                {"history_size": True}, "bumi", executor, controller
            )
        with self.assertRaisesRegex(ValueError, "history_size must be in"):
            DEVICE.MotionEventsPlugin(
                {"history_size": 1001}, "bumi", executor, controller
            )
        with self.assertRaisesRegex(ValueError, "activity_velocity_threshold"):
            DEVICE.MotionEventsPlugin(
                {"activity_velocity_threshold": 0}, "bumi", executor, controller
            )

    def test_start_and_stop_are_idempotent(self):
        controller = _Controller()
        executor = _Executor()
        plugin = DEVICE.MotionEventsPlugin(
            {"poll_interval_s": 0.02}, "bumi_test", executor, controller
        )

        first = plugin.dispatch("start", {})
        second = plugin.dispatch("start", {})
        self.assertTrue(first["running"])
        self.assertTrue(second["running"])
        time.sleep(0.03)
        stopped = plugin.dispatch("stop", {})
        self.assertFalse(stopped["running"])


if __name__ == "__main__":
    unittest.main()
