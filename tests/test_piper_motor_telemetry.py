"""PiPER telemetry tests with an SDK cache and a recording ROS publisher."""

import threading
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from agilex.piper.arm import MOTOR_FEEDBACK_IDS, MotorTelemetry, PiperDriver
from agilex.piper.device import build_plugins
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
        self.assertNotIn("additional_arms", data)

    def test_two_arms_keep_independent_measurements_and_freshness(self):
        right_feedback = Mock(return_value=(MotorTelemetry(41, 45, 23.6, 0, 99.8),) * 6)
        plugin = JointMotorTelemetryPlugin(
            self.driver.get_motor_telemetry, namespace="aloha/piper", can_interface="can0",
            secondary_can_interface="can1",
            arm_roles={"can0": "left_follower", "can1": "right_follower"},
            ros2=object(), clock=lambda: self.now, publisher_factory=self.publisher,
        )
        with patch("agilex.piper.motor_telemetry.PiperDriver") as factory:
            factory.return_value.get_motor_telemetry = right_feedback
            plugin.start()
            plugin.start()
            factory.assert_called_once_with("can1")
            factory.return_value.connect.assert_called_once_with(read_only=True)
        self.addCleanup(plugin.stop)
        self.publisher.call_args.args[-1]()
        _, payload = plugin._transport.publish.call_args.args
        right = payload["additional_arms"]["can1"]
        self.assertEqual((payload["arm_role"], right["arm_role"]), ("left_follower", "right_follower"))
        self.assertEqual((payload["J1"]["motor_temperature_c"], right["J1"]["motor_temperature_c"]), (21, 41))
        self.assertEqual((payload["feedback_status"], right["feedback_status"]), ("fresh", "fresh"))
        self.assertEqual(right["J1"]["driver_current_a"], 0)

        right_feedback.return_value = ()
        payload = plugin.info()["telemetry"]
        self.assertEqual(payload["feedback_status"], "fresh")
        right = payload["additional_arms"]["can1"]
        self.assertEqual(right["feedback_status"], "unavailable")
        self.assertIsNone(right["J1"]["motor_temperature_c"])

    def test_telemetry_only_config_exposes_one_card_and_uses_two_passive_connections(self):
        [plugin] = build_plugins({"can_interface": "can0", "motor_telemetry": {
            "only": True, "secondary_can_interface": "can1",
        }}, "aloha", None)
        self.assertEqual([tool["name"] for tool in DriverBundle([plugin]).get_all_tools()], [TOOL_NAME])
        first, second = Mock(), Mock()
        with patch("agilex.piper.motor_telemetry.PiperDriver", side_effect=[first, second]):
            plugin.start()
            plugin.stop()
        for receiver in (first, second):
            receiver.connect.assert_called_once_with(read_only=True)
            receiver.close.assert_called_once_with()

    def test_start_cleans_up_if_secondary_receiver_fails(self):
        [plugin] = build_plugins({"motor_telemetry": {
            "only": True, "secondary_can_interface": "can1",
        }}, "aloha", None)
        first, second = Mock(), Mock()
        second.connect.side_effect = OSError("CAN interface unavailable")
        with patch("agilex.piper.motor_telemetry.PiperDriver", side_effect=[first, second]):
            with self.assertRaises(OSError):
                plugin.start()
        first.close.assert_called_once_with()
        second.close.assert_called_once_with()
        self.assertEqual(plugin.info()["state"], "idle")

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
