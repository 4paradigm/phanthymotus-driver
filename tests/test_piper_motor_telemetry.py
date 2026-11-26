"""PiPER telemetry tests with an SDK cache and a recording ROS publisher."""

import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from agilex.piper.arm import MOTOR_FEEDBACK_IDS, PiperDriver
from agilex.piper.motor_telemetry import JointMotorTelemetryPlugin, TOOL_NAME
from common.vendor_runtime import DriverBundle


class MotorTelemetryTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.driver = PiperDriver("can1")
        self.motors = SimpleNamespace(**{
            f"motor_{joint}": SimpleNamespace(
                can_id=can_id, motor_temp=20 + joint, foc_temp=30 + joint,
                vol=240 + joint, bus_current=500 + joint,
            )
            for joint, can_id in enumerate(MOTOR_FEEDBACK_IDS, 1)
        })
        self.driver.arm = SimpleNamespace(
            frame_lock=threading.Lock(),
            frame_times=dict.fromkeys(MOTOR_FEEDBACK_IDS, self.now - 0.1),
            GetArmLowSpdInfoMsgs=Mock(return_value=self.motors),
        )
        self.publisher = Mock(side_effect=lambda *args: Mock())
        self.plugin = JointMotorTelemetryPlugin(
            self.driver.get_motor_telemetry, namespace="right/piper", can_interface="can1",
            ros2=object(), clock=lambda: self.now, publisher_factory=self.publisher,
        )
        self.bundle = DriverBundle([self.plugin])

    def call(self, action, **args):
        return self.bundle.dispatch(TOOL_NAME, {"action": action, **args})

    def test_motor_units_and_per_joint_received_times(self):
        data = self.call("info", _trace_id="test")["telemetry"]
        self.assertEqual(data["feedback_status"], "fresh")
        self.assertEqual(data["can_interface"], "can1")
        self.assertEqual(data["J1"], {
            "status": "fresh", "motor_temperature_c": 21, "driver_temperature_c": 31,
            "driver_voltage_v": 24.1, "driver_current_a": 0.501,
            "received_at_unix_s": 99.9,
        })
        self.assertEqual(data["J6"]["motor_temperature_c"], 26)

    def test_missing_stale_and_future_frames_do_not_reuse_readings(self):
        times = self.driver.arm.frame_times
        del times[MOTOR_FEEDBACK_IDS[0]]
        times[MOTOR_FEEDBACK_IDS[1]] = self.now - 0.6
        times[MOTOR_FEEDBACK_IDS[2]] = self.now + 0.1
        self.motors.motor_4.can_id = 0
        data = self.call("info")["telemetry"]
        self.assertEqual(data["feedback_status"], "partial")
        for joint, status in (("J1", "unavailable"), ("J2", "stale"), ("J3", "unavailable"),
                              ("J4", "unavailable")):
            with self.subTest(joint=joint):
                self.assertEqual(data[joint]["status"], status)
                self.assertIsNone(data[joint]["driver_voltage_v"])
                self.assertIsNone(data[joint]["received_at_unix_s"])
        self.assertEqual(data["J5"]["status"], "fresh")
        self.now += 1
        self.assertEqual(self.call("info")["telemetry"]["feedback_status"], "unavailable")

    def test_sensor_lifecycle_is_idempotent_and_old_timer_cannot_publish(self):
        [definition] = self.bundle.get_all_tools()
        self.assertEqual(definition["type"], "sensor")
        self.assertEqual(definition["title"], "Joint Motor Telemetry")
        self.assertEqual(definition["topic_out"], [{
            "topic": "/right/piper/state/joint_motor_telemetry", "format": "data/json",
        }])
        self.driver.arm.GetArmLowSpdInfoMsgs.assert_not_called()
        self.assertEqual(self.call("start", instance_id="right", input_topic="")["state"], "running")
        self.call("start")
        self.publisher.assert_called_once()
        first = self.plugin._transport
        first_tick = self.publisher.call_args.args[-1]
        first_tick()
        first.publish.assert_called_once()
        self.assertEqual(first.publish.call_args.args[0], "data/json")
        self.assertEqual(self.call("stop")["state"], "idle")
        self.call("stop")
        first.close.assert_called_once()
        self.call("start")
        second = self.plugin._transport
        first_tick()
        first.publish.assert_called_once()
        second.publish.assert_not_called()
        self.publisher.call_args.args[-1]()
        second.publish.assert_called_once()
        with self.assertRaises(ValueError):
            self.call("move")
        with self.assertRaises(ValueError):
            self.call("info", input_topic="/unrelated")

    def test_disconnected_driver_reports_empty_readings(self):
        self.driver.arm = None
        data = self.call("info")["telemetry"]
        self.assertEqual(data["feedback_status"], "unavailable")
        self.assertIsNone(data["J6"]["motor_temperature_c"])
