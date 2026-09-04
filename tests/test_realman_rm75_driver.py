import importlib.util
import math
import os
from pathlib import Path
import struct
import unittest
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "realman" / "rm75_6f_v"


def load_device():
    spec = importlib.util.spec_from_file_location("realman_rm75_device", DRIVER / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class RealManRM75ImageContractTests(unittest.TestCase):
    def test_image_contains_only_minimal_api2_runtime(self):
        dockerfile = (DRIVER / "Dockerfile").read_text()
        self.assertIn("COPY vendor/Robotic_Arm/ /work/Robotic_Arm/", dockerfile)
        self.assertIn("COPY deploy/ /deploy/", dockerfile)
        self.assertNotIn("colcon", dockerfile)
        self.assertNotIn("rm_driver", dockerfile)
        self.assertFalse((DRIVER / "entrypoint.sh").exists())

    def test_only_arm64_vendor_library_is_present(self):
        library = DRIVER / "vendor" / "Robotic_Arm" / "libs" / "linux_arm" / "libapi_c.so"
        self.assertEqual([library], list((DRIVER / "vendor").rglob("libapi_c.so")))
        self.assertEqual([], list((DRIVER / "vendor").rglob("libapi_cpp.so")))
        header = library.read_bytes()[:20]
        self.assertEqual(b"\x7fELF", header[:4])
        endian = "<" if header[5] == 1 else ">"
        self.assertEqual(183, struct.unpack(f"{endian}H", header[18:20])[0])

    def test_service_has_safe_connection_default(self):
        service = (DRIVER / "deploy" / "service.yml").read_text()
        self.assertIn("RM_DRIVER_ENABLED=0", service)
        self.assertNotIn("/opt/realman/rm_ws", service)
        self.assertNotIn("network_mode:", service)
        self.assertNotIn("ipc:", service)


class RealManRM75SDKClientTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def test_disabled_by_default_and_motion_is_independently_locked(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        client.start()
        self.assertEqual("disabled", client.status()["state"])
        self.assertFalse(client.motion_enabled)
        tools = self.device.RM75Plugin(client, {}).get_tools()
        self.assertEqual(
            {"connection", "joint_states", "model", "robot_info", "software_info", "arm_all_state", "controller_state", "joint_control"},
            {item["name"].split(".")[-1] for item in tools},
        )
        joint_control = next(item for item in tools if item["name"] == "joint_control")
        self.assertEqual("actuator", joint_control["type"])
        self.assertEqual(["set"], joint_control["inputSchema"]["x-completion"]["actions"])
        self.assertEqual(10, joint_control["inputSchema"]["properties"]["speed_percent"]["maximum"])

    def test_joint_degrees_are_converted_to_radians(self):
        class Handle:
            id = 1

        class Robot:
            def rm_get_joint_degree(self):
                return 0, [0, 90, -90, 180, -180, 45, -45]

        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client._handle = Handle()
        client._robot = Robot()
        result = client.joint_states()
        self.assertEqual(7, len(result["position"]))
        self.assertAlmostEqual(math.pi / 2, result["position"][1])
        self.assertAlmostEqual(-math.pi, result["position"][4])
        self.assertEqual("rad", result["position_unit"])

    def test_sdk_error_is_not_returned_as_sensor_data(self):
        with self.assertRaisesRegex(RuntimeError, "code 5"):
            self.device._sdk_result("rm_get_robot_info", (5, {}))

    def _motion_plugin(self, *, motion_enabled=True, current=None, all_state=None):
        current = current or [0.0] * 7
        all_state = all_state or {
            "joint_err_code": [0] * 7,
            "joint_en_flag": [1] * 7,
            "err": {"err_len": 0, "err": []},
        }

        class Handle:
            id = 1

        class Robot:
            def __init__(self):
                self.moves = []
                self.stops = 0

            def rm_get_joint_degree(self):
                return 0, list(current)

            def rm_get_arm_all_state(self):
                return 0, dict(all_state)

            def rm_get_joint_drive_min_pos(self):
                return 0, [item[0] for item in self_module.JOINT_LIMITS_DEG]

            def rm_get_joint_drive_max_pos(self):
                return 0, [item[1] for item in self_module.JOINT_LIMITS_DEG]

            def rm_movej(self, target, speed, radius, connect, block):
                self.moves.append((list(target), speed, radius, connect, block))
                return 0

            def rm_set_arm_slow_stop(self):
                self.stops += 1
                return 0

        self_module = self.device
        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client.motion_enabled = motion_enabled
        client._handle = Handle()
        client._robot = Robot()
        plugin = self.device.RM75Plugin(client, {"safety": {"poll_interval_seconds": 0.001}})
        plugin._acp_callback = mock.Mock()
        return plugin, client._robot

    def test_complete_joint_target_is_sent_as_one_movej(self):
        plugin, robot = self._motion_plugin(current=[10, 20, 30, 40, 50, 60, 70])
        result = plugin._start_motion({
            "joint1_deg": 10, "joint2_deg": 20, "joint3_deg": 31,
            "joint4_deg": 40, "joint5_deg": 50, "joint6_deg": 60,
            "joint7_deg": 70, "speed_percent": 1, "confirm_motion": True,
        })
        self.assertEqual("executing", result["status"])
        self.assertEqual(([10, 20, 31, 40, 50, 60, 70], 1, 0, 0, 0), robot.moves[0])

    @staticmethod
    def _seven_targets(**overrides):
        values = {f"joint{i}_deg": 0 for i in range(1, 8)}
        values.update(overrides)
        return values

    def test_set_rejects_any_missing_joint_field(self):
        plugin, robot = self._motion_plugin()
        args = self._seven_targets()
        del args["joint7_deg"]
        with self.assertRaisesRegex(ValueError, "missing: joint7_deg"):
            plugin._start_motion({**args, "confirm_motion": True})
        self.assertEqual([], robot.moves)

    def test_motion_requires_both_interlocks(self):
        plugin, robot = self._motion_plugin(motion_enabled=False)
        with self.assertRaisesRegex(PermissionError, "motion is locked"):
            plugin._start_motion({**self._seven_targets(joint1_deg=1), "confirm_motion": True})
        with self.assertRaisesRegex(ValueError, "confirm_motion"):
            plugin, robot = self._motion_plugin()
            plugin._start_motion(self._seven_targets(joint1_deg=1))
        self.assertEqual([], robot.moves)

    def test_motion_rejects_large_step_limit_and_robot_error(self):
        plugin, robot = self._motion_plugin()
        with self.assertRaisesRegex(ValueError, "exceeds 5.0deg"):
            plugin._start_motion({**self._seven_targets(joint1_deg=6), "confirm_motion": True})
        self.assertEqual([], robot.moves)
        plugin, robot = self._motion_plugin(all_state={
            "joint_err_code": [0, 0, 3, 0, 0, 0, 0],
            "joint_en_flag": [1] * 7,
            "err": {"err_len": 0, "err": []},
        })
        with self.assertRaisesRegex(RuntimeError, "joint error"):
            plugin._start_motion({**self._seven_targets(joint1_deg=1), "confirm_motion": True})

    def test_stopmotion_uses_vendor_slow_stop(self):
        plugin, robot = self._motion_plugin()
        result = plugin._stop_motion()
        self.assertEqual("stop_requested", result["state"])
        self.assertEqual(1, robot.stops)


if __name__ == "__main__":
    unittest.main()
