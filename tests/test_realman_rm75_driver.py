import importlib.util
import io
import json
import math
import os
from pathlib import Path
import ssl
import sys
import threading
import time
import unittest
import xml.etree.ElementTree as ET
from unittest import mock


ROOT = Path(__file__).resolve().parents[1]
DRIVER = ROOT / "realman" / "rm75_6f_v"


def load_device():
    # The driver directory goes on sys.path the way main.py puts it there at
    # runtime: build_plugins imports its siblings by bare name (`camera`,
    # `servo`), so a loader that skips this tests a module the driver never runs.
    if str(DRIVER) not in sys.path:
        sys.path.insert(0, str(DRIVER))
    spec = importlib.util.spec_from_file_location("realman_rm75_device", DRIVER / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    sys.modules.setdefault("device", module)      # servo.py imports `device`
    return module


class RealManRM75ImageContractTests(unittest.TestCase):
    def test_image_contains_api2_and_realsense_runtime(self):
        dockerfile = (DRIVER / "Dockerfile").read_text()
        self.assertIn("COPY vendor/Robotic_Arm/ /work/Robotic_Arm/", dockerfile)
        self.assertNotIn("RM_API2_LIB_URL", dockerfile)
        self.assertNotIn("ADD http", dockerfile)
        self.assertIn("COPY deploy/ /deploy/", dockerfile)
        self.assertNotIn("colcon", dockerfile)
        self.assertNotIn("rm_driver", dockerfile)
        self.assertIn("python3-pip", dockerfile)
        self.assertIn("ARG APT_MIRROR=", dockerfile)
        self.assertIn("${APT_MIRROR}", dockerfile)
        self.assertIn('test -z "$(dpkg --audit)"', dockerfile)
        self.assertEqual(1, dockerfile.count("apt-get update"))
        self.assertNotIn("dpkg-query -W", dockerfile)
        self.assertNotIn("        v4l-utils", dockerfile)
        self.assertNotIn("        libusb-1.0-0", dockerfile)
        self.assertIn("apt-get download libusb-1.0-0", dockerfile)
        self.assertIn("dpkg-deb --extract", dockerfile)
        self.assertIn("test -e /opt/realman/libusb/libusb-1.0.so.0", dockerfile)
        self.assertIn("LD_LIBRARY_PATH=/opt/realman/libusb", dockerfile)
        self.assertNotIn("||", dockerfile)
        self.assertIn("pyrealsense2==2.56.5.9235", dockerfile)
        self.assertIn("numpy==1.23.5", dockerfile)
        self.assertIn("opencv-python-headless==4.11.0.86", dockerfile)
        self.assertIn("--only-binary=:all:", dockerfile)
        self.assertIn("COPY main.py device.py servo.py camera.py realsense.py", dockerfile)
        camera = (DRIVER / "camera.py").read_text()
        self.assertNotIn("v4l2-ctl", camera)
        self.assertNotIn("import subprocess", camera)
        self.assertNotIn("ExtMicPlugin", camera)
        self.assertNotIn("TOOLS_EXT_MIC", camera)
        self.assertNotIn("_enumerate_ext_mics", camera)
        self.assertNotIn("_ExtMicNode", camera)
        self.assertFalse((DRIVER / "entrypoint.sh").exists())

    def test_vendor_shared_libraries_are_not_committed(self):
        self.assertEqual([], list((DRIVER / "vendor").rglob("libapi_c.so")))
        self.assertEqual([], list((DRIVER / "vendor").rglob("libapi_cpp.so")))

    def test_service_has_motion_capable_rm75_default(self):
        service = (DRIVER / "deploy" / "service.yml").read_text()
        self.assertIn("RM_DRIVER_ENABLED=1", service)
        self.assertIn("RM_MOTION_ENABLED=1", service)
        self.assertIn("RM_ARM_IP=${RM75_ARM_IP:-192.168.1.18}", service)
        # ACP 完成回调走 CA 校验的 TLS：必须挂载 Agent Core CA 并注入路径
        self.assertIn("AGENT_CORE_CA_CERT=${RM75_AGENT_CORE_CA_CERT:-/opt/phanthy-motus/data/certs/cert.pem}", service)
        self.assertNotIn("AGENT_CORE_TOKEN", service)
        self.assertIn("${RM75_CA_DIR:-/opt/phanthy-motus/data}:/opt/phanthy-motus/data:ro", service)
        self.assertIn("network_mode: host", service)
        self.assertNotIn("privileged:", service)
        self.assertIn("/dev:/dev:ro", service)
        self.assertIn("tmpfs:\n    - /dev/shm:rw,nosuid,nodev,noexec,size=128m,mode=1777", service)
        self.assertIn('"c 81:* rw"', service)
        self.assertIn('"c 189:* rw"', service)
        self.assertIn("cap_drop:\n    - MKNOD", service)
        self.assertNotIn("  devices:", service)
        self.assertNotIn("RM75_REALSENSE_VIDEO_DEVICE", service)
        self.assertNotIn("RM75_CAMERA_VIDEO", service)
        self.assertIn("/opt/phanthy-motus/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro", service)
        self.assertIn("FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml", service)
        self.assertIn("RM_CARTESIAN_ENABLED=${RM75_CARTESIAN_ENABLED:-1}", service)
        self.assertIn(
            "${RM_API2_LIB_DIR:-/opt/realman/rm_api2/libs/linux_arm}:/work/Robotic_Arm/libs/linux_arm:ro",
            service,
        )
        self.assertNotIn("/opt/realman/rm_ws", service)
        self.assertNotIn("ipc:", service)

        config = (DRIVER / "config.yaml").read_text()
        self.assertIn("cartesian:\n", config)
        self.assertIn("  enabled: true\n", config)

    def test_ext_camera_is_enabled_and_advertised(self):
        config = (DRIVER / "config.yaml").read_text()
        manifest = (DRIVER / "driver.yaml").read_text()
        self.assertIn("ext_camera:\n  enabled: true", config)
        self.assertIn("name: ext_camera", manifest)

    def test_build_plugins_registers_camera_only_when_enabled(self):
        device = load_device()
        # Imported before the patch, not after: mock.patch.dict restores the
        # whole of sys.modules on exit, so a module first imported inside the
        # block is discarded, and the same class ends up as two objects.
        from servo import RM75ServoPlugin

        calls = []

        class FakeCamera:
            def __init__(self, config, namespace, executor):
                calls.append((config, namespace, executor))

        ros2 = type("ROS2", (), {"executor_core": object()})()
        camera_module = type("CameraModule", (), {"ExtCameraPlugin": FakeCamera})()
        with mock.patch.dict("sys.modules", {"camera": camera_module}):
            enabled = device.build_plugins(
                {"ext_camera": {"enabled": True}}, "rm75", ros2
            )
        self.assertEqual(
            [device.RM75Plugin, device.GripperPlugin, RM75ServoPlugin,
             device.CartesianPlugin, FakeCamera],
            [type(plugin) for plugin in enabled],
        )
        self.assertIs(enabled[0].client, enabled[1].client)
        # The servo card shares the one SDK handle too — a second connection to
        # the same arm is a second thing able to move it.
        self.assertIs(enabled[0].client, enabled[2].client)
        self.assertIs(enabled[0].client, enabled[3].client)
        self.assertEqual(({"enabled": True}, "rm75", ros2.executor_core), calls[0])
        disabled = device.build_plugins({}, "rm75", ros2)
        self.assertEqual(
            [device.RM75Plugin, device.GripperPlugin, RM75ServoPlugin,
             device.CartesianPlugin],
            [type(plugin) for plugin in disabled],
        )

    def test_vision_capture_reuses_existing_camera_and_core_executor(self):
        device = load_device()
        camera = mock.Mock()
        capture = mock.Mock()
        ros2 = type("ROS2", (), {"executor_core": object()})()
        camera_factory, capture_factory = mock.Mock(return_value=camera), mock.Mock(return_value=capture)
        with mock.patch.dict("sys.modules", {
            "camera": type("Module", (), {"ExtCameraPlugin": camera_factory})(),
            "vision_capture": type("Module", (), {"VisionCapturePlugin": capture_factory})(),
        }):
            plugins = device.build_plugins({"ext_camera": {"enabled": True},
                                           "vision_capture": {"enabled": True}}, "rm75", ros2)
        self.assertEqual(plugins[-2:], [camera, capture])
        capture_factory.assert_called_once_with({"enabled": True}, "rm75", ros2.executor_core, camera)

    def test_capture_files_survive_container_replacement(self):
        service = (DRIVER / "deploy/service.yml").read_text()
        directory = "/opt/phanthy-motus/data/vision_capture/realman"
        self.assertIn(f"{directory}:{directory}", service)
        self.assertNotIn(f"{directory}:{directory}:ro", service)
        self.assertIn(f"output_dir: {directory}", (DRIVER / "config.yaml").read_text())
        dockerfile = (DRIVER / "Dockerfile").read_text()
        self.assertIn("ros-humble-rmw-fastrtps-cpp ffmpeg", dockerfile)
        self.assertIn("realsense.py vision_capture.py config.yaml", dockerfile)


class RealManRM75GripperPluginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def setUp(self):
        class FakeClient:
            def __init__(self):
                self.calls = []
                self.connected = True
                self.motion_enabled = True

            def command(self, method, *args):
                self.calls.append((method, args))
                return 0

        self.client = FakeClient()
        self.plugin = self.device.GripperPlugin(self.client, {}, namespace="rm75")
        # 单测不碰网络：ACP 回调替换为记录器
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

    def _wait_for(self, condition, timeout=2.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if condition():
                return True
            time.sleep(0.01)
        return False

    def test_plugin_prefix_contract(self):
        self.assertEqual("gripper", self.device.GripperPlugin.PREFIX)

    def test_tool_schema_exposes_1_to_1000_and_safety_contract(self):
        tools = self.plugin.get_tools()
        self.assertEqual(1, len(tools))
        self.assertEqual("gripper", tools[0]["name"])
        self.assertEqual("actuator", tools[0]["type"])
        position = tools[0]["inputSchema"]["properties"]["position"]
        self.assertEqual(1, position["minimum"])
        self.assertEqual(1000, position["maximum"])
        self.assertIs(True, tools[0]["inputSchema"]["x-is-dangerous"])
        self.assertIn("confirm_motion", tools[0]["inputSchema"]["properties"])
        self.assertIn("confirm_motion", tools[0]["inputSchema"]["x-action-params"]["set_position"]["params"])
        completion = tools[0]["inputSchema"]["x-completion"]
        self.assertEqual(["set_position"], completion["actions"])
        self.assertGreater(completion["timeout"], 0)

    def test_set_position_returns_action_id_and_calls_two_finger_api(self):
        result = self.plugin.dispatch(
            "set_position", {"position": 500, "confirm_motion": True}
        )

        self.assertEqual("running", result["state"])
        self.assertTrue(result["action_id"].startswith("rm75_gripper_"))
        # SDK 阻塞调用在 worker 线程里发生
        self.assertTrue(self._wait_for(lambda: len(self.client.calls) == 1))
        self.assertEqual(
            [("rm_set_gripper_position", (500, True, self.device.GRIPPER_COMPLETION_TIMEOUT))],
            self.client.calls,
        )
        # 完成后 ACP 上报 completed
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, payload = self.acp_events[0]
        self.assertEqual(result["action_id"], action_id)
        self.assertEqual("completed", status)
        self.assertEqual("target_reached", payload["reason"])

    def test_out_of_range_position_is_rejected(self):
        for value in (-1, 0, 1001, float("nan"), float("inf")):
            with self.subTest(value=value):
                with self.assertRaises(ValueError):
                    self.plugin.dispatch("set_position", {"position": value, "confirm_motion": True})
        self.assertEqual([], self.client.calls)

    def test_motion_requires_enabled_client(self):
        self.client.motion_enabled = False
        with self.assertRaisesRegex(PermissionError, "motion is locked"):
            self.plugin.dispatch("set_position", {"position": 500, "confirm_motion": True})
        self.assertEqual([], self.client.calls)

    def test_motion_requires_confirmation(self):
        for args in ({"position": 500}, {"position": 500, "confirm_motion": False}):
            with self.subTest(args=args):
                with self.assertRaisesRegex(ValueError, "confirm_motion must be true"):
                    self.plugin.dispatch("set_position", args)
        self.assertEqual([], self.client.calls)

    def test_concurrent_gripper_motion_is_rejected(self):
        self.plugin._gripper_lock.acquire()
        try:
            with self.assertRaisesRegex(RuntimeError, "another gripper motion is active"):
                self.plugin.dispatch("set_position", {"position": 500, "confirm_motion": True})
        finally:
            self.plugin._gripper_lock.release()

    def test_stop_waits_for_safe_terminal_state(self):
        # SDK 无夹爪停止 API：stop 必须等命令走到安全终态（夹爪走完目标位）才返回，
        # 且 ACP 如实上报 completed 而不是谎报 cancelled。
        released = threading.Event()

        class BlockingClient:
            def __init__(self):
                self.connected = True
                self.motion_enabled = True

            def command(self, method, *args):
                released.wait(5.0)
                return 0

        plugin = self.device.GripperPlugin(BlockingClient(), {}, namespace="rm75")
        plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        started = plugin.dispatch("set_position", {"position": 500, "confirm_motion": True})
        stop_thread = threading.Thread(target=lambda: plugin.dispatch("stop", {}))
        stop_thread.start()
        time.sleep(0.1)
        # 命令仍在途时 stop 不得返回
        self.assertTrue(stop_thread.is_alive())
        released.set()
        stop_thread.join(5.0)
        self.assertFalse(stop_thread.is_alive())
        action_id, status, payload = self.acp_events[0]
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("completed", status)
        self.assertTrue(payload["interrupted"])

    def test_plugin_stop_waits_for_worker_before_teardown(self):
        released = threading.Event()

        class BlockingClient:
            def __init__(self):
                self.connected = True
                self.motion_enabled = True

            def command(self, method, *args):
                released.wait(5.0)
                return 0

        plugin = self.device.GripperPlugin(BlockingClient(), {}, namespace="rm75")
        plugin._acp_callback = lambda action_id, status, result: None
        plugin.dispatch("set_position", {"position": 500, "confirm_motion": True})
        self.assertTrue(plugin._worker_thread.is_alive())

        stop_done = threading.Event()
        threading.Thread(target=lambda: (plugin.stop(), stop_done.set()), daemon=True).start()
        time.sleep(0.1)
        self.assertFalse(stop_done.is_set())
        released.set()
        self.assertTrue(stop_done.wait(5.0))
        self.assertFalse(plugin._worker_thread.is_alive())

    def test_canvas_lifecycle_actions(self):
        self.assertEqual({"state": "ready"}, self.plugin.dispatch("start", {}))
        self.assertEqual({"state": "idle"}, self.plugin.dispatch("stop", {}))
        info = self.plugin.dispatch("info", {})
        self.assertEqual("connected", info["state"])
        self.assertIn("active_action_id", info)

    def test_unknown_action_returns_none(self):
        self.assertIsNone(self.plugin.dispatch("something_else", {}))

    def test_gripper_card_is_advertised(self):
        manifest = (DRIVER / "driver.yaml").read_text()
        self.assertIn("name: gripper", manifest)


class RealManRM75CartesianPluginTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    FAST_SAFETY = {
        "start_grace_seconds": 0.05,
        "stall_timeout_seconds": 0.3,
        "poll_interval_seconds": 0.05,
        "progress_threshold_mm": 0.5,
        "max_motion_seconds": 2.0,
        "position_tolerance_mm": 5.0,
        "euler_tolerance_deg": 2.0,
        "max_speed_percent": 10,
        "default_speed_percent": 5,
    }

    class FakeClient:
        def __init__(self, pose_mm_deg=None):
            self.calls = []
            self.connected = True
            self.motion_enabled = True
            self.pose_mm_deg = list(pose_mm_deg or [0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        def command(self, method, *args):
            self.calls.append((method, args))
            return 0

        def call(self, method):
            self.calls.append((method,))
            if method == "rm_get_current_arm_state":
                x, y, z, rx, ry, rz = self.pose_mm_deg
                return {"pose": [x / 1000.0, y / 1000.0, z / 1000.0,
                                 math.radians(rx), math.radians(ry), math.radians(rz)],
                        "joint": [0.0] * 7, "err": {}}
            if method == "rm_get_arm_all_state":
                return {"joint_err_code": [0] * 7, "err": {"err": []}, "joint_en_flag": [1] * 7}
            raise RuntimeError(method)

    def setUp(self):
        self.client = self.FakeClient()
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client, {
                "safety": dict(self.FAST_SAFETY),
                "cartesian": {"enabled": True, "max_radius_mm": 640, "shoulder_height_mm": 240.5, "max_reach_mm": 650},
            },
            arm_plugin=self.arm, namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

    def _wait_for(self, condition, timeout=3.0):
        deadline = time.time() + timeout
        while time.time() < deadline:
            if condition():
                return True
            time.sleep(0.01)
        return False

    def _movel_args(self, **overrides):
        args = {"x_mm": 100, "y_mm": 0, "z_mm": 0, "rx_deg": 90, "ry_deg": 0, "rz_deg": 0,
                "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True}
        args.update(overrides)
        return args

    def _a_to_b_args(self, **overrides):
        args = {
            "b_x_mm": 150, "b_y_mm": 0, "b_z_mm": 200,
            "b_rx_deg": 0, "b_ry_deg": 0, "b_rz_deg": 0,
            "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
        }
        args.update(overrides)
        return args

    def test_plugin_prefix_contract(self):
        self.assertEqual("abs_move", self.device.CartesianPlugin.PREFIX)

    def test_tool_schema_declares_safety_contract(self):
        tools = self.plugin.get_tools()
        self.assertEqual(1, len(tools))
        self.assertEqual("abs_move", tools[0]["name"])
        self.assertEqual("actuator", tools[0]["type"])
        schema = tools[0]["inputSchema"]
        self.assertIs(True, schema["x-is-dangerous"])
        self.assertEqual(["move_a_to_b"], schema["x-completion"]["actions"])
        self.assertIn("confirm_motion", schema["properties"])
        self.assertEqual(False, schema["properties"]["cartesian_enabled"]["default"])
        self.assertNotIn("move_offset", schema["x-action-params"])
        self.assertIn("move_a_to_b", schema["x-action-params"])
        self.assertNotIn("a_x_mm", schema["properties"])
        self.assertNotIn("a_x_mm", schema["x-action-params"]["move_a_to_b"]["params"])
        self.assertIn("b_rz_deg", schema["x-action-params"]["move_a_to_b"]["params"])
        self.assertIn("motion_mode", schema["x-action-params"]["move_a_to_b"]["params"])
        self.assertEqual(["joint", "linear"], schema["properties"]["motion_mode"]["enum"])
        self.assertEqual("joint", schema["properties"]["motion_mode"]["default"])
        self.assertEqual(10, schema["properties"]["speed_percent"]["maximum"])
        self.assertNotIn("movel", schema["x-action-params"])
        self.assertNotIn("movep", schema["x-action-params"])
        self.assertNotIn("x_mm", schema["properties"])
        self.assertNotIn("waypoints", schema["properties"])
        for removed_field in (
                "dx_mm", "dy_mm", "dz_mm", "drx_deg", "dry_deg", "drz_deg", "frame_type"):
            self.assertNotIn(removed_field, schema["properties"])
        self.assertEqual(
            {
                "on_interrupt_motion": {"action": "stopmotion"},
                "on_interrupt_all": {"action": "stopmotion"},
            },
            schema["x-hooks"],
        )
        self.assertEqual("做绝对位置下的移动", tools[0]["description"])

    def test_move_offset_is_rejected_without_submitting_motion(self):
        with self.assertRaisesRegex(ValueError, "move_offset is no longer supported"):
            self.plugin.dispatch("move_offset", {
                "dx_mm": 20,
                "frame_type": "tool",
                "speed_percent": 5,
                "cartesian_enabled": True,
                "confirm_motion": True,
            })
        self.assertEqual([], self.client.calls)
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_move_a_to_b_reads_current_pose_as_a_and_submits_only_b(self):
        callbacks = []

        class EventClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                callbacks.append(completion_callback)
                return 0

            def cancel_trajectory_wait(self):
                return True

            def command_interrupt(self, method, *args):
                self.calls.append((method, args))
                return 0

        self.client = EventClient([25.0, -10.0, 200.0, 1.0, 2.0, 3.0])
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        started = self.plugin.dispatch("move_a_to_b", self._a_to_b_args())
        self.assertTrue(self._wait_for(lambda: len(callbacks) == 1))
        movej_p_calls = [call for call in self.client.calls if call[0] == "rm_movej_p"]
        self.assertEqual(1, len(movej_p_calls))
        self.assertEqual([0.15, 0.0, 0.2, 0.0, 0.0, 0.0], movej_p_calls[0][1][0])
        self.assertEqual([], self.acp_events)
        self.assertEqual("to_b", self.plugin._motion_status()["active_stage"])
        self.assertEqual("joint", self.plugin._motion_status()["active_motion_mode"])

        callbacks[0](True)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, result = self.acp_events[0]
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("completed", status)
        for actual, expected in zip(
                result["point_a_pose_mm_deg"], [25.0, -10.0, 200.0, 1.0, 2.0, 3.0]):
            self.assertAlmostEqual(expected, actual, places=9)
        self.assertEqual([150.0, 0.0, 200.0, 0.0, 0.0, 0.0], result["target_pose_mm_deg"])
        self.assertEqual("joint", result["motion_mode"])
        self.assertEqual("ready", self.plugin._motion_status()["state"])
        self.assertIsNone(self.plugin._motion_status()["active_stage"])
        self.assertIsNone(self.plugin._motion_status()["active_motion_mode"])

    def test_move_a_to_b_omitted_axes_keep_current_a_values(self):
        callbacks = []

        class EventClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                callbacks.append(completion_callback)
                return 0

            def cancel_trajectory_wait(self):
                return True

        self.client = EventClient([25.0, -10.0, 200.0, 10.0, 20.0, 30.0])
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        self.plugin.dispatch("move_a_to_b", {
            "b_x_mm": 150,
            "b_y_mm": 0,
            "b_z_mm": 250,
            "speed_percent": 5,
            "cartesian_enabled": True,
            "confirm_motion": True,
        })
        self.assertTrue(self._wait_for(lambda: len(callbacks) == 1))
        movej_p = next(call for call in self.client.calls if call[0] == "rm_movej_p")
        expected = [0.15, 0.0, 0.25, math.radians(10), math.radians(20), math.radians(30)]
        for actual, target in zip(movej_p[1][0], expected):
            self.assertAlmostEqual(target, actual, places=9)
        callbacks[0](True)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))

    def test_move_a_to_b_linear_mode_uses_movel(self):
        callbacks = []

        class EventClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                callbacks.append(completion_callback)
                return 0

            def cancel_trajectory_wait(self):
                return True

        self.client = EventClient([25.0, -10.0, 200.0, 0.0, 0.0, 0.0])
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        self.plugin.dispatch(
            "move_a_to_b",
            self._a_to_b_args(motion_mode="linear"),
        )
        self.assertTrue(self._wait_for(lambda: len(callbacks) == 1))
        self.assertEqual(1, len([call for call in self.client.calls if call[0] == "rm_movel"]))
        self.assertEqual(0, len([call for call in self.client.calls if call[0] == "rm_movej_p"]))
        callbacks[0](True)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        self.assertEqual("linear", self.acp_events[0][2]["motion_mode"])

    def test_move_a_to_b_requires_at_least_one_b_field(self):
        with self.assertRaisesRegex(ValueError, "at least one B pose field"):
            self.plugin.dispatch("move_a_to_b", {
                "speed_percent": 5,
                "cartesian_enabled": True,
                "confirm_motion": True,
            })

    def test_move_a_to_b_rejects_unknown_motion_mode(self):
        with self.assertRaisesRegex(ValueError, "motion_mode must be 'joint' or 'linear'"):
            self.plugin.dispatch(
                "move_a_to_b",
                self._a_to_b_args(motion_mode="curve"),
            )
        self.assertFalse(self.plugin._motion_lock.locked())
        self.assertEqual("ready", self.plugin._motion_status()["state"])
        self.assertFalse(self.plugin._motion_lock.locked())
        self.assertFalse(any(call[0] == "rm_get_current_arm_state" for call in self.client.calls))

    def test_move_a_to_b_releases_action_when_controller_rejects_b(self):
        callbacks = []

        class EventClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                callbacks.append(completion_callback)
                return 0

            def cancel_trajectory_wait(self):
                return True

        self.client = EventClient([0.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        self.plugin.dispatch("move_a_to_b", self._a_to_b_args())
        self.assertTrue(self._wait_for(lambda: len(callbacks) == 1))
        callbacks[0](False)

        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        _, status, result = self.acp_events[0]
        self.assertEqual("error", status)
        self.assertEqual("to_b", result["failed_segment"])
        self.assertIn("collision failure", result["reason"])
        self.assertEqual(1, len([call for call in self.client.calls if call[0] == "rm_movej_p"]))
        self.assertEqual("ready", self.plugin._motion_status()["state"])
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_move_a_to_b_timeout_releases_lock_for_next_action(self):
        class EventlessClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                return 0

            def cancel_trajectory_wait(self):
                self.calls.append(("cancel_trajectory_wait", ()))
                return True

            def command_interrupt(self, method, *args):
                self.calls.append((method, args))
                return 0

        self.client = EventlessClient([0.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {
                "safety": {**self.FAST_SAFETY, "max_motion_seconds": 0.05},
                "cartesian": {"enabled": True, "stop_finalize_seconds": 0.01},
            },
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        first = self.plugin.dispatch("move_a_to_b", self._a_to_b_args())
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        self.assertEqual((first["action_id"], "error"), self.acp_events[0][:2])
        self.assertEqual("to_b", self.acp_events[0][2]["failed_segment"])
        self.assertEqual("ready", self.plugin._motion_status()["state"])

        second = self.plugin.dispatch("move_a_to_b", self._a_to_b_args())
        self.assertNotEqual(first["action_id"], second["action_id"])
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 2))
        self.assertEqual((second["action_id"], "error"), self.acp_events[1][:2])

    def test_move_a_to_b_validates_b_before_motion(self):
        self.client.command_trajectory = mock.Mock()
        with self.assertRaisesRegex(ValueError, "exceeds"):
            self.plugin.dispatch(
                "move_a_to_b",
                self._a_to_b_args(b_x_mm=2000),
            )
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_cartesian_motion_requires_deployment_enable(self):
        plugin = self.device.CartesianPlugin(
            self.client, {"safety": dict(self.FAST_SAFETY)},
            arm_plugin=self.arm, namespace="rm75",
        )

        with self.assertRaisesRegex(PermissionError, "disabled by deployment configuration"):
            plugin.dispatch("movel", self._movel_args(cartesian_enabled=True))

        self.assertEqual([], [entry for entry in self.client.calls if entry[0] == "rm_movel"])
        self.assertFalse(plugin._motion_lock.locked())

    def test_deployment_environment_can_override_cartesian_motion(self):
        with mock.patch.dict(os.environ, {"RM_CARTESIAN_ENABLED": "1"}):
            plugin = self.device.CartesianPlugin(
                self.client, {"cartesian": {"enabled": False}}, arm_plugin=self.arm
            )
        self.assertTrue(plugin.cartesian_enabled)

        with mock.patch.dict(os.environ, {"RM_CARTESIAN_ENABLED": "0"}):
            plugin = self.device.CartesianPlugin(
                self.client, {"cartesian": {"enabled": True}}, arm_plugin=self.arm
            )
        self.assertFalse(plugin.cartesian_enabled)

        with mock.patch.dict(os.environ, {"RM_CARTESIAN_ENABLED": "invalid"}):
            with self.assertRaisesRegex(ValueError, "must be 0 or 1"):
                self.device.CartesianPlugin(
                    self.client, {"cartesian": {"enabled": False}}, arm_plugin=self.arm
                )

    def test_cartesian_motion_requires_card_enable(self):
        with self.assertRaisesRegex(PermissionError, "disabled for this request"):
            self.plugin.dispatch("movel", self._movel_args(cartesian_enabled=False))

        self.assertEqual([], [entry for entry in self.client.calls if entry[0] == "rm_movel"])
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_card_can_enable_cartesian_motion_with_workspace_guards(self):
        result = self.plugin.dispatch("movel", self._movel_args(cartesian_enabled=True))

        self.assertEqual("running", result["state"])
        self.assertIn(
            ("rm_movel", ([0.1, 0.0, 0.0, math.pi / 2, 0.0, 0.0], 5, 0, 0, 0)),
            self.client.calls,
        )
        self.client.pose_mm_deg = [100.0, 0.0, 0.0, 90.0, 0.0, 0.0]
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))

    def test_movel_converts_units_and_reports_completion(self):
        result = self.plugin.dispatch("movel", self._movel_args())

        self.assertEqual("running", result["state"])
        self.assertTrue(result["action_id"].startswith("rm75_cart_"))
        # 毫米/度 → 米/弧度转换后非阻塞下发（connect=0, block=0）
        self.assertIn(
            ("rm_movel", ([0.1, 0.0, 0.0, math.pi / 2, 0.0, 0.0], 5, 0, 0, 0)),
            self.client.calls,
        )
        # 到位后 ACP 上报 completed
        self.client.pose_mm_deg = [100.0, 0.0, 0.0, 90.0, 0.0, 0.0]
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, payload = self.acp_events[0]
        self.assertEqual(result["action_id"], action_id)
        self.assertEqual("completed", status)
        self.assertLessEqual(payload["position_error_mm"], 5.0)
        self.assertLessEqual(payload["euler_error_deg"], 2.0)

    def test_move_offset_maps_frame_and_computes_target(self):
        self.client.pose_mm_deg = [300.0, 0.0, 200.0, 0.0, 0.0, 0.0]
        result = self.plugin._start_cartesian("move_offset", {
            "dx_mm": 50, "dy_mm": 0, "dz_mm": 0,
            "drx_deg": 0, "dry_deg": 0, "drz_deg": 0,
            "frame_type": "tool", "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
        })

        self.assertEqual("running", result["state"])
        # 四代控制器直接执行工具系偏移；目标换算仅用于预检和完成监控。
        self.assertIn(
            ("rm_movel_offset", ([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 5, 0, 0, 1, 0)),
            self.client.calls,
        )
        self.assertNotIn("rm_movel", [entry[0] for entry in self.client.calls])
        self.client.pose_mm_deg = [350.0, 0.0, 200.0, 0.0, 0.0, 0.0]
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        _, status, payload = self.acp_events[0]
        self.assertEqual("completed", status)
        self.assertEqual([350.0, 0.0, 200.0, 0.0, 0.0, 0.0], payload["target_pose_mm_deg"])

    def test_move_offset_empty_fields_mean_no_offset(self):
        self.client.pose_mm_deg = [300.0, 0.0, 200.0, 0.0, 0.0, 0.0]
        result = self.plugin._start_cartesian("move_offset", {
            "dx_mm": 50, "dy_mm": "", "drz_deg": None,
            "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        })

        self.assertEqual("running", result["state"])
        self.assertIn(
            ("rm_movel_offset", ([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 5, 0, 0, 1, 0)),
            self.client.calls,
        )

    def test_native_offset_uses_controller_event_completion(self):
        class EventClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                if completion_callback is not None:
                    completion_callback(True)
                return 0

            def wait_trajectory(self, timeout_seconds):
                return True

            def discard_trajectory_wait(self):
                pass

            def command_interrupt(self, method, *args):
                self.calls.append((method, args))
                return 0

        self.client = EventClient()
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        started = self.plugin._start_cartesian("move_offset", {
            "dx_mm": 20, "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        })

        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, result = self.acp_events[0]
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("completed", status)
        self.assertEqual("controller_target_reached", result["reason"])
        self.assertEqual([20.0, 0.0, 0.0, 0.0, 0.0, 0.0], result["requested_offset_mm_deg"])
        self.assertIn(
            ("rm_movel_offset", ([0.02, 0.0, 0.0, 0.0, 0.0, 0.0], 5, 0, 0, 1, 0)),
            self.client.calls,
        )
        submit_index = next(
            index for index, call in enumerate(self.client.calls) if call[0] == "rm_movel_offset"
        )
        self.assertEqual(
            [],
            [call for call in self.client.calls[submit_index + 1:] if call[0].startswith("rm_get_")],
        )
        self.assertEqual(
            [("rm_get_arm_all_state",), ("rm_get_current_arm_state",)],
            [call for call in self.client.calls if call[0].startswith("rm_get_")],
        )

    def test_native_offset_rejects_composed_target_outside_workspace(self):
        self.client.command_trajectory = mock.Mock()
        self.client.wait_trajectory = mock.Mock()
        plugin = self._real_cartesian_plugin()
        self.client.pose_mm_deg = [850.0, 0.0, 240.5, 0.0, 0.0, 0.0]

        with self.assertRaisesRegex(ValueError, "horizontal radius 900 mm exceeds"):
            plugin._start_cartesian("move_offset", {
                "dx_mm": 50,
                "frame_type": "tool",
                "speed_percent": 5,
                "cartesian_enabled": True,
                "confirm_motion": True,
            })

        self.assertNotIn("rm_movel_offset", [entry[0] for entry in self.client.calls])
        self.assertFalse(plugin._motion_lock.locked())

    def test_native_offset_preflight_failure_submits_nothing(self):
        self.client.command_trajectory = mock.Mock()
        self.client.wait_trajectory = mock.Mock()
        self.arm._preflight = mock.Mock(side_effect=RuntimeError("joint 3 is disabled"))

        with self.assertRaisesRegex(RuntimeError, "joint 3 is disabled"):
            self.plugin._start_cartesian("move_offset", {
                "dx_mm": 20,
                "frame_type": "tool",
                "speed_percent": 5,
                "cartesian_enabled": True,
                "confirm_motion": True,
            })

        self.arm._preflight.assert_called_once_with()
        self.client.command_trajectory.assert_not_called()
        self.assertNotIn("rm_movel_offset", [entry[0] for entry in self.client.calls])
        self.assertEqual("ready", self.plugin._motion_status()["state"])
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_native_offset_rejects_pose_change_during_validation(self):
        self.client.command_trajectory = mock.Mock()
        self.client.wait_trajectory = mock.Mock()
        original = self.plugin._validated_offset_target

        def invalidate_after_validation(offset):
            target, revision = original(offset)
            self.plugin._invalidate_confirmed_pose()
            return target, revision

        self.plugin._validated_offset_target = invalidate_after_validation
        with self.assertRaisesRegex(RuntimeError, "reference pose changed"):
            self.plugin._start_cartesian("move_offset", {
                "dx_mm": 20,
                "frame_type": "tool",
                "speed_percent": 5,
                "cartesian_enabled": True,
                "confirm_motion": True,
            })

        self.assertNotIn("rm_movel_offset", [entry[0] for entry in self.client.calls])
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_sdk_minus_7_switches_to_cached_absolute_movel_compatibility(self):
        class ThirdGenerationClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                if method == "rm_movel_offset":
                    raise RuntimeError("rm_movel_offset failed with RealMan SDK code -7")
                if completion_callback is not None:
                    completion_callback(True)
                return 0

            def wait_trajectory(self, timeout_seconds):
                return True

            def discard_trajectory_wait(self):
                pass

            def command_interrupt(self, method, *args):
                self.calls.append((method, args))
                return 0

        self.client = ThirdGenerationClient([300.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )
        args = {
            "dx_mm": 50, "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        }

        first = self.plugin._start_cartesian("move_offset", args)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        self.assertEqual((first["action_id"], "completed"), self.acp_events[0][:2])
        self.assertEqual(
            [350.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            self.acp_events[0][2]["target_pose_mm_deg"],
        )

        second = self.plugin._start_cartesian("move_offset", args)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 2))
        self.assertEqual((second["action_id"], "completed"), self.acp_events[1][:2])
        self.assertEqual(
            [400.0, 0.0, 200.0, 0.0, 0.0, 0.0],
            self.acp_events[1][2]["target_pose_mm_deg"],
        )
        self.assertEqual(
            1,
            len([call for call in self.client.calls if call[0] == "rm_get_current_arm_state"]),
        )
        self.assertEqual(
            1,
            len([call for call in self.client.calls if call[0] == "rm_movel_offset"]),
        )
        movel_calls = [call for call in self.client.calls if call[0] == "rm_movel"]
        self.assertEqual(2, len(movel_calls))
        self.assertAlmostEqual(0.35, movel_calls[0][1][0][0], places=6)
        self.assertAlmostEqual(0.4, movel_calls[1][1][0][0], places=6)
        self.assertIs(False, self.plugin._native_tool_offset_supported)

    def test_controller_event_releases_action_when_sdk_wrapper_call_stays_blocked(self):
        entered = threading.Event()
        release_sdk = threading.Event()
        callbacks = []

        class BlockingEventClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                callbacks.append(completion_callback)
                entered.set()
                release_sdk.wait(5.0)
                return 0

            def wait_trajectory(self, timeout_seconds):
                return True

            def command_interrupt(self, method, *args):
                self.calls.append((method, args))
                return 0

        self.client = BlockingEventClient()
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )
        args = {
            "dx_mm": 20, "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        }

        first = self.plugin._start_cartesian("move_offset", args)
        self.assertTrue(entered.wait(1.0))
        callbacks[0](True)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        self.assertEqual((first["action_id"], "completed"), self.acp_events[0][:2])
        self.assertEqual("ready", self.plugin._motion_status()["state"])

        entered.clear()
        second = self.plugin._start_cartesian("move_offset", args)
        self.assertTrue(entered.wait(1.0))
        callbacks[1](True)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 2))
        self.assertEqual((second["action_id"], "completed"), self.acp_events[1][:2])
        release_sdk.set()

    def test_watchdog_releases_action_when_sdk_call_and_event_both_stall(self):
        entered = threading.Event()
        release_sdk = threading.Event()

        class StalledClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                entered.set()
                release_sdk.wait(5.0)
                return 0

            def wait_trajectory(self, timeout_seconds):
                return None

            def cancel_trajectory_wait(self):
                self.calls.append(("cancel_trajectory_wait", ()))
                return True

            def command_interrupt(self, method, *args):
                self.calls.append((method, args))
                return 0

        self.client = StalledClient()
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {
                "safety": {**self.FAST_SAFETY, "max_motion_seconds": 0.05},
                "cartesian": {"enabled": True, "stop_finalize_seconds": 0.01},
            },
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )
        args = {
            "dx_mm": 20, "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        }

        first = self.plugin._start_cartesian("move_offset", args)
        self.assertTrue(entered.wait(1.0))
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        self.assertEqual((first["action_id"], "error"), self.acp_events[0][:2])
        self.assertEqual("controller completion event timed out", self.acp_events[0][2]["reason"])
        self.assertEqual("ready", self.plugin._motion_status()["state"])

        entered.clear()
        second = self.plugin._start_cartesian("move_offset", args)
        self.assertTrue(entered.wait(1.0))
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 2))
        self.assertEqual((second["action_id"], "error"), self.acp_events[1][:2])
        release_sdk.set()

    def test_stopmotion_releases_missing_controller_event(self):
        gate = threading.Event()

        class EventClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                return 0

            def wait_trajectory(self, timeout_seconds):
                gate.wait(timeout_seconds)
                return False if gate.is_set() else None

            def discard_trajectory_wait(self):
                gate.set()

            def cancel_trajectory_wait(self):
                gate.set()
                return True

            def command_interrupt(self, method, *args):
                self.calls.append((method, args))
                return 0

        self.client = EventClient()
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {
                "safety": dict(self.FAST_SAFETY),
                "cartesian": {"enabled": True, "stop_finalize_seconds": 0.05},
            },
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )
        started = self.plugin._start_cartesian("move_offset", {
            "dx_mm": 20, "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        })
        self.assertTrue(self._wait_for(lambda: any(call[0] == "rm_movel_offset" for call in self.client.calls)))

        self.assertEqual(
            {"state": "stop_requested", "action_id": started["action_id"]},
            self.plugin.dispatch("stopmotion", {}),
        )
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, result = self.acp_events[0]
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("cancelled", status)
        self.assertEqual("stopmotion", result["reason"])
        self.assertEqual("ready", self.plugin._motion_status()["state"])

    def test_missing_controller_event_times_out_and_allows_next_motion(self):
        class EventlessClient(self.FakeClient):
            def command_trajectory(self, method, *args, completion_callback=None):
                self.calls.append((method, args))
                return 0

            def wait_trajectory(self, timeout_seconds):
                return None

            def discard_trajectory_wait(self):
                pass

        self.client = EventlessClient([300.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        self.arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {
                "safety": {**self.FAST_SAFETY, "max_motion_seconds": 0.05},
                "cartesian": {"enabled": True, "stop_finalize_seconds": 0.01},
            },
            arm_plugin=self.arm,
            namespace="rm75",
        )
        self.acp_events = []
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )
        args = {
            "dx_mm": 20, "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        }

        first = self.plugin._start_cartesian("move_offset", args)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        self.assertEqual((first["action_id"], "error"), self.acp_events[0][:2])
        self.assertEqual("controller completion event timed out", self.acp_events[0][2]["reason"])
        self.assertEqual("ready", self.plugin._motion_status()["state"])

        second = self.plugin._start_cartesian("move_offset", args)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 2))
        self.assertEqual((second["action_id"], "error"), self.acp_events[1][:2])

    def test_move_offset_rotated_tool_frame_transforms_target(self):
        # reviewer 示例：90° yaw 下工具系 +X 偏移应沿基系 +Y 移动，监控目标必须经旋转变换
        self.client.pose_mm_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 90.0]
        self.plugin._start_cartesian("move_offset", {
            "dx_mm": 100, "dy_mm": 0, "dz_mm": 0,
            "drx_deg": 0, "dry_deg": 0, "drz_deg": 0,
            "frame_type": "tool", "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
        })
        offset_calls = [entry[1] for entry in self.client.calls if entry[0] == "rm_movel_offset"]
        self.assertEqual(1, len(offset_calls))
        self.assertAlmostEqual(0.1, offset_calls[0][0][0], places=6)
        self.assertAlmostEqual(0.0, offset_calls[0][0][1], places=6)
        self.assertAlmostEqual(0.0, offset_calls[0][0][5], places=6)
        self.assertEqual((5, 0, 0, 1, 0), offset_calls[0][1:])
        self.client.pose_mm_deg = [0.0, 100.0, 0.0, 0.0, 0.0, 90.0]
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        _, status, payload = self.acp_events[0]
        self.assertEqual("completed", status)
        self.assertAlmostEqual(100.0, payload["target_pose_mm_deg"][1], places=3)
        self.assertAlmostEqual(0.0, payload["target_pose_mm_deg"][0], places=3)

    def test_work_frame_offset_is_rejected(self):
        with self.assertRaisesRegex(ValueError, "frame_type must be 'tool'"):
            self.plugin._start_cartesian("move_offset", {
                "dx_mm": 50, "dy_mm": 0, "dz_mm": 0,
                "drx_deg": 0, "dry_deg": 0, "drz_deg": 0,
                "frame_type": "work", "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
            })

    def test_workspace_limits_reject_unreachable_poses(self):
        # 水平半径、肩部臂展、姿态角超出配置上限时在下发前拒绝
        cases = [
            {"x_mm": 1200, "y_mm": 0, "z_mm": 0, "rx_deg": 0, "ry_deg": 0, "rz_deg": 0},
            {"x_mm": 700, "y_mm": 700, "z_mm": 700, "rx_deg": 0, "ry_deg": 0, "rz_deg": 0},
            {"x_mm": 0, "y_mm": 0, "z_mm": 0, "rx_deg": 0, "ry_deg": 0, "rz_deg": 400},
            # 竖直臂展超出肩部可达范围（肩高 240.5 + 臂长 650）
            {"x_mm": 0, "y_mm": 0, "z_mm": 1600, "rx_deg": 0, "ry_deg": 0, "rz_deg": 0},
        ]
        for pose in cases:
            with self.subTest(pose=pose):
                with self.assertRaisesRegex(ValueError, "exceeds"):
                    self.plugin.dispatch("movel", {**pose, "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True})
        self.assertEqual([], [entry for entry in self.client.calls if entry[0] == "rm_movel"])

    def _real_cartesian_plugin(self):
        # 真机配置：官方 MDH 几何（肩高 240.5、臂展 650）+ 夹爪 TCP 工具长度 222.5。
        return self.device.CartesianPlugin(
            self.client, {
                "safety": dict(self.FAST_SAFETY),
                "cartesian": {"enabled": True, "max_radius_mm": 640, "shoulder_height_mm": 240.5,
                              "max_reach_mm": 650, "tool_length_mm": 222.5},
            },
            arm_plugin=self.arm, namespace="rm75",
        )

    def test_vertical_reach_pose_is_allowed_within_shoulder_model(self):
        # 竖直朝上位姿：水平半径 0、肩部距离在臂展+工具长度包络内，
        # 不得再被旧「原点 610mm 球」误杀。
        plugin = self._real_cartesian_plugin()
        result = plugin.dispatch("movel", {
            "x_mm": 0, "y_mm": 0, "z_mm": 1100, "rx_deg": 0, "ry_deg": 0, "rz_deg": 0,
            "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
        })
        self.assertEqual("running", result["state"])
        self.assertIn(
            ("rm_movel", ([0.0, 0.0, 1.1, 0.0, 0.0, 0.0], 5, 0, 0, 0)),
            self.client.calls,
        )

    def test_vertical_offset_down_200mm_is_allowed(self):
        # 真机回归：竖直朝上当前位姿（上报欧拉角 (0,0,0)，夹爪实际竖直伸出、
        # TCP z≈1112 = 240.5+650+222.5），工具系下移 200mm、横移 50mm：
        # 目标 (50, 0, ~912)，旧代码曾报「radius 914 exceeds 610」误拒。
        plugin = self._real_cartesian_plugin()
        self.client.pose_mm_deg = [0.0, 0.0, 1112.0, 0.0, 0.0, 0.0]
        result = plugin._start_cartesian("move_offset", {
            "dx_mm": 50, "dy_mm": 0, "dz_mm": -200,
            "drx_deg": 0, "dry_deg": 0, "drz_deg": 0,
            "frame_type": "tool", "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
        })
        self.assertEqual("running", result["state"])
        self.assertIn(
            ("rm_movel_offset", ([0.05, 0.0, -0.2, 0.0, 0.0, 0.0], 5, 0, 0, 1, 0)),
            self.client.calls,
        )

    def test_move_offset_falls_back_when_controller_does_not_support_native_offset(self):
        class LegacyClient(self.FakeClient):
            def command(self, method, *args):
                self.calls.append((method, args))
                if method == "rm_movel_offset":
                    raise RuntimeError("rm_movel_offset failed with RealMan SDK code -7")
                return 0

        client = LegacyClient([300.0, 0.0, 200.0, 0.0, 0.0, 0.0])
        arm = self.device.RM75Plugin(client, {}, namespace="rm75")
        plugin = self.device.CartesianPlugin(
            client,
            {"safety": dict(self.FAST_SAFETY),
             "cartesian": {"enabled": True, "max_radius_mm": 640,
                           "shoulder_height_mm": 240.5, "max_reach_mm": 650}},
            arm_plugin=arm, namespace="rm75",
        )

        result = plugin._start_cartesian("move_offset", {
            "dx_mm": 50, "dy_mm": 0, "dz_mm": 0,
            "drx_deg": 0, "dry_deg": 0, "drz_deg": 0,
            "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        })

        self.assertEqual("running", result["state"])
        self.assertIn("rm_movel_offset", [entry[0] for entry in client.calls])
        self.assertIn(
            ("rm_movel", ([0.35, 0.0, 0.2, 0.0, 0.0, 0.0], 5, 0, 0, 0)),
            client.calls,
        )

    def test_tool_length_extends_tcp_envelope(self):
        # 校验直接针对 TCP 并把 tool_length 作为包络余量，不再从姿态反推法兰
        # （上报欧拉角不编码物理工具轴方向）。水平 TCP 700 ≤ 640+222.5 应放行。
        plugin = self._real_cartesian_plugin()
        result = plugin.dispatch("movel", {
            "x_mm": 700, "y_mm": 0, "z_mm": 340, "rx_deg": 0, "ry_deg": 0, "rz_deg": 0,
            "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
        })
        self.assertEqual("running", result["state"])

    def test_tool_length_envelope_still_rejects_overreach(self):
        # TCP 水平 900 > 640+222.5=862.5，超出包络必须拒绝。
        plugin = self._real_cartesian_plugin()
        with self.assertRaisesRegex(ValueError, "exceeds"):
            plugin.dispatch("movel", {
                "x_mm": 900, "y_mm": 0, "z_mm": 340, "rx_deg": 0, "ry_deg": 0, "rz_deg": 0,
                "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
            })
        self.assertEqual([], [entry for entry in self.client.calls if entry[0] == "rm_movel"])

    def test_movep_validates_every_waypoint(self):
        with self.assertRaisesRegex(ValueError, "exceeds"):
            self.plugin.dispatch("movep", {
                "waypoints": [[100, 0, 0, 0, 0, 0], [2000, 0, 0, 0, 0, 0]],
                "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
            })
        self.assertEqual([], [entry for entry in self.client.calls if entry[0] == "rm_movel"])

    def test_idle_lifecycle_stop_does_not_interrupt_joint_motion(self):
        self.arm._active_action_id = "rm75_joint_active"

        self.assertEqual({"state": "idle"}, self.plugin.dispatch("stop", {}))

        self.assertNotIn("rm_set_arm_slow_stop", [entry[0] for entry in self.client.calls])
        self.assertEqual("rm75_joint_active", self.arm._active_action_id)

    def test_active_lifecycle_stop_requests_slow_stop(self):
        with self.plugin._action_lock:
            self.plugin._active_action_id = "rm75_cart_active"

        self.assertEqual({"state": "idle"}, self.plugin.dispatch("stop", {}))

        self.assertIn(("rm_set_arm_slow_stop", ()), self.client.calls)
        self.assertIn("rm75_cart_active", self.plugin._cancelled)

    def test_stop_joins_monitor_before_teardown(self):
        # 容器关闭时 stop 必须等监控线程收尾（ACP 终态上报），再允许共享 SDK 销毁
        released = threading.Event()

        class SlowPoseClient(self.FakeClient):
            def call(self, method):
                if method == "rm_get_current_arm_state":
                    if not released.is_set():
                        released.wait(2.0)
                return super().call(method)

        self.client = SlowPoseClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client, {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}}, arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )
        self.plugin.dispatch("movel", self._movel_args())
        self.assertTrue(self.plugin._monitor_thread.is_alive())

        stop_done = threading.Event()
        threading.Thread(target=lambda: (self.plugin.stop(), stop_done.set()), daemon=True).start()
        time.sleep(0.1)
        self.assertFalse(stop_done.is_set())  # 监控未收尾前 stop 不得返回
        released.set()
        self.assertTrue(stop_done.wait(5.0))
        self.assertFalse(self.plugin._monitor_thread.is_alive())
        # 终态已上报（cancelled 或 completed，取决于取消与到位的先后）
        self.assertEqual(1, len(self.acp_events))

    def test_acp_complete_requires_ca_cert(self):
        with mock.patch.dict(os.environ, {"AGENT_CORE_CA_CERT": ""}), \
                mock.patch("urllib.request.urlopen") as urlopen:
            self.device._acp_complete("test-noca", "completed", {"reason": "x"}, "abs_move")
        urlopen.assert_not_called()

    def test_acp_complete_verifies_with_provided_ca(self):
        real_ctx = ssl.create_default_context()
        with mock.patch.dict(os.environ, {"AGENT_CORE_CA_CERT": "/tmp/ca.pem",
                                          "AGENT_CORE_URL": "https://phanthy-motus:15678/"}), \
                mock.patch("ssl.create_default_context") as mkctx, \
                mock.patch("urllib.request.urlopen") as urlopen:
            mkctx.return_value = real_ctx
            urlopen.return_value.__enter__.return_value.read.return_value = b'{"ok":true,"action_id":"test-ca"}'
            self.device._acp_complete("test-ca", "completed", {"reason": "x"}, "abs_move")
        mkctx.assert_called_once_with(cafile="/tmp/ca.pem")
        urlopen.assert_called_once()

    def test_acp_complete_reports_ca_context_failure(self):
        with mock.patch.dict(os.environ, {"AGENT_CORE_CA_CERT": "/tmp/empty.pem"}), \
                mock.patch("ssl.create_default_context", side_effect=ValueError("invalid CA")), \
                mock.patch("urllib.request.urlopen") as urlopen:
            result = self.device._acp_complete("test-bad-ca", "completed", {"reason": "x"}, "abs_move")
        self.assertEqual(("failed", "invalid CA"), result)
        urlopen.assert_not_called()

    def test_movep_chains_waypoints_with_connect_flags(self):
        waypoints = [
            [100, 0, 0, 0, 0, 0],
            [100, 100, 0, 0, 0, 0],
            [100, 100, 100, 0, 0, 0],
        ]
        result = self.plugin.dispatch("movep", {
            "waypoints": waypoints, "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
        })

        self.assertEqual("running", result["state"])
        movel_calls = [entry[1] for entry in self.client.calls if entry[0] == "rm_movel"]
        self.assertEqual(3, len(movel_calls))
        # rm_movel(pose, v, r, connect, block)：前 N-1 个点 connect=1（联合规划），末点 connect=0（立即执行）
        self.assertEqual(1, movel_calls[0][3])
        self.assertEqual(1, movel_calls[1][3])
        self.assertEqual(0, movel_calls[2][3])
        self.assertEqual([0.1, 0.1, 0.1, 0.0, 0.0, 0.0], movel_calls[2][0])

    def test_pure_rotation_move_is_not_misread_as_stall(self):
        # 回归：纯旋转运动位置误差恒为 0，进度检测必须跟踪欧拉角，
        # 否则旋转中途被误判 stall 而慢停（真机实锤过的 bug）。
        class RotatingClient(self.FakeClient):
            def __init__(self):
                super().__init__()
                self.rz = 0.0
                self.pose_calls = 0

            def call(self, method):
                if method == "rm_get_current_arm_state":
                    self.pose_calls += 1
                    # 第一次查询是下发前的当前位姿读取；之后每轮询转 5°（超过 stall 窗口时长）
                    if self.pose_calls > 1 and self.rz < 90.0:
                        self.rz = min(90.0, self.rz + 5.0)
                    return {"pose": [0.0, 0.0, 0.0, 0.0, 0.0, math.radians(self.rz)],
                            "joint": [0.0] * 7, "err": {}}
                return super().call(method)

        self.client = RotatingClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )
        self.plugin._start_cartesian("move_offset", {
            "dx_mm": 0, "dy_mm": 0, "dz_mm": 0,
            "drx_deg": 0, "dry_deg": 0, "drz_deg": 90,
            "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        })
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1, timeout=5.0))
        _, status, payload = self.acp_events[0]
        self.assertEqual("completed", status)
        self.assertLessEqual(payload["euler_error_deg"], 2.0)

    def test_monitor_stall_sends_slow_stop(self):
        # 位姿一直不前进 → stall 检测触发受控停止并如实上报 motion_stalled
        self.plugin.dispatch("movel", self._movel_args())
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1, timeout=5.0))
        action_id, status, payload = self.acp_events[0]
        self.assertEqual("error", status)
        self.assertEqual("motion_stalled", payload["reason"])
        self.assertIn(("rm_set_arm_slow_stop", ()), self.client.calls)

    def test_controller_reject_without_trajectory_releases_motion_lock(self):
        # 控制器接受命令后才判定无逆解：无规划、无运动时必须立即释放卡片。
        self.client.call_dict = lambda method: {"trajectory_type": 0}
        started = self.plugin._start_cartesian("move_offset", {
            "dx_mm": 20, "frame_type": "tool", "speed_percent": 5,
            "cartesian_enabled": True, "confirm_motion": True,
        })

        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, payload = self.acp_events[0]
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("error", status)
        self.assertEqual("controller_rejected_trajectory", payload["reason"])
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_stopmotion_does_not_hold_action_lock_during_sdk_call(self):
        # SDK 慢停无超时上限：stopmotion 必须立即返回，终态不能被慢停调用堵住。
        gate = threading.Event()

        class SlowStopClient(self.FakeClient):
            def command(self, method, *args):
                if method == "rm_set_arm_slow_stop" and not gate.is_set():
                    gate.wait(2.0)
                return super().command(method, *args)

        self.client = SlowStopClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: None

        self.plugin.dispatch("movel", self._movel_args())
        self.assertTrue(self._wait_for(lambda: self.plugin._active_action_id is not None))

        start = time.time()
        stop = self.plugin.dispatch("stopmotion", {})
        self.assertLess(time.time() - start, 1.0)
        self.assertEqual("stop_requested", stop["state"])
        start = time.time()
        self.plugin.dispatch("info", {})  # 慢停阻塞期间 info 应立即可达
        self.assertLess(time.time() - start, 1.0)
        gate.set()
        self.assertTrue(self._wait_for(lambda: self.plugin._active_action_id is None, timeout=5.0))

    def test_stopmotion_cancels_and_slow_stops(self):
        started = self.plugin.dispatch("movel", self._movel_args())
        stop = self.plugin.dispatch("stopmotion", {})

        self.assertEqual("stop_requested", stop["state"])
        self.assertEqual(started["action_id"], stop["action_id"])
        self.assertTrue(self._wait_for(lambda: ("rm_set_arm_slow_stop", ()) in self.client.calls))
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, payload = self.acp_events[0]
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("cancelled", status)
        self.assertEqual("stopmotion", payload["reason"])

    def test_joint_stopmotion_cancels_active_cartesian_action(self):
        started = self.plugin.dispatch("movel", self._movel_args())
        stop = self.arm.dispatch("stopmotion", {"_tool_name": "joint_control"})

        self.assertEqual("stop_requested", stop["state"])
        self.assertEqual(started["action_id"], stop["action_id"])
        self.assertIn(("rm_set_arm_slow_stop", ()), self.client.calls)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, payload = self.acp_events[0]
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("cancelled", status)
        self.assertEqual("stopmotion", payload["reason"])

    def test_cartesian_stopmotion_stops_active_joint_action(self):
        action_id = "rm75_joint_active"
        with self.arm._action_lock:
            self.arm._active_action_id = action_id
            self.arm._motion_state["active_action_id"] = action_id
        stop = self.plugin.dispatch("stopmotion", {})

        self.assertEqual({"state": "stop_requested", "action_id": action_id}, stop)
        self.assertIn(("rm_set_arm_slow_stop", ()), self.client.calls)
        self.assertIn(action_id, self.arm._cancelled)

    def test_info_remains_available_while_submission_is_blocked(self):
        gate = threading.Event()

        class GatedClient(self.FakeClient):
            def command(self, method, *args):
                if method == "rm_movel" and not gate.is_set():
                    gate.wait(2.0)
                return super().command(method, *args)

        self.client = GatedClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client, {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}}, arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: None

        mover = threading.Thread(target=lambda: self.plugin.dispatch("movel", self._movel_args()))
        mover.start()
        self.assertTrue(self._wait_for(lambda: self.plugin._active_action_id is not None))
        start = time.time()
        info = self.plugin.dispatch("info", {})
        self.assertLess(time.time() - start, 1.0)
        self.assertEqual("moving", info["state"])
        self.assertIsNotNone(info["active_action_id"])
        gate.set()
        mover.join(5.0)

    def test_submission_race_with_stopmotion_orders_slow_stop_after_submit(self):
        # 竞态回归：急停必须等下发完成；SDK 调用顺序
        # rm_movel 在前、rm_set_arm_slow_stop 在后，监控如实上报 cancelled。
        gate = threading.Event()

        class GatedClient(self.FakeClient):
            def command(self, method, *args):
                if method == "rm_movel" and not gate.is_set():
                    gate.wait(2.0)
                return super().command(method, *args)

        self.client = GatedClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client, {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}}, arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        mover = threading.Thread(target=lambda: self.plugin.dispatch("movel", self._movel_args()))
        mover.start()
        time.sleep(0.1)  # 让 movel 卡在 rm_movel 下发过程中
        stop_done = threading.Event()
        stopper = threading.Thread(target=lambda: (self.plugin.dispatch("stopmotion", {}), stop_done.set()))
        stopper.start()
        time.sleep(0.1)
        self.assertTrue(stop_done.is_set())  # 取消请求不得等待在途下发或 SDK 慢停
        gate.set()
        stopper.join(5.0)
        mover.join(5.0)
        self.assertTrue(stop_done.is_set())
        order = [entry[0] for entry in self.client.calls]
        self.assertLess(order.index("rm_movel"), order.index("rm_set_arm_slow_stop"))
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        _, status, payload = self.acp_events[0]
        self.assertEqual("cancelled", status)

    def test_movep_partial_submission_failure_slow_stops(self):
        # 回归：第 2 个路径点下发失败时，不得留下无看护的已排队轨迹 ——
        # 异常路径必须明确慢停，且不启动监控、不放任动作悬挂。
        class PartialFailClient(self.FakeClient):
            def __init__(self, *args, **kwargs):
                super().__init__(*args, **kwargs)
                self.movel_count = 0

            def command(self, method, *args):
                if method == "rm_movel":
                    self.movel_count += 1
                    if self.movel_count == 2:
                        raise RuntimeError("rm_movel failed with RealMan SDK code 1")
                return super().command(method, *args)

        self.client = PartialFailClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        with self.assertRaisesRegex(RuntimeError, "code 1"):
            self.plugin.dispatch("movep", {
                "waypoints": [[100, 0, 0, 0, 0, 0], [100, 100, 0, 0, 0, 0], [100, 100, 100, 0, 0, 0]],
                "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
            })
        order = [entry[0] for entry in self.client.calls]
        # 部分下发后失败 → 慢停在失败的 movel 之后发出
        self.assertIn("rm_set_arm_slow_stop", order)
        self.assertLess(order.index("rm_movel"), order.index("rm_set_arm_slow_stop"))
        # 没有孤儿监控、没有 ACP 事件、运动锁已释放
        self.assertEqual([], self.acp_events)
        self.assertIsNone(self.plugin._monitor_thread)
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_acp_callback_records_outcome_in_last_completion(self):
        plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            namespace="rm75",
        )
        plugin.ACP_RETRY_ATTEMPTS = 0
        plugin._last_completion = {"action_id": "a1", "status": "completed", "result": {}}
        with mock.patch.object(self.device, "_acp_complete", return_value=("failed", "boom")):
            plugin._acp_callback("a1", "completed", {"reason": "target_reached"})
        self.assertEqual("failed", plugin._last_completion["callback"])
        self.assertEqual("boom", plugin._last_completion["callback_error"])

    def test_arm_stopmotion_waits_for_cartesian_submission(self):
        # 回归（reviewer 要求）：臂的 stopmotion 必须排在笛卡尔 rm_movel 下发之后，
        # 共享提交锁对两张卡片全局生效 —— rm_movel 先于 rm_set_arm_slow_stop 到达 SDK。
        gate = threading.Event()

        class GatedClient(self.FakeClient):
            def command(self, method, *args):
                if method == "rm_movel" and not gate.is_set():
                    gate.wait(2.0)
                return super().command(method, *args)

        self.client = GatedClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client,
            {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}},
            arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: None

        mover = threading.Thread(target=lambda: self.plugin.dispatch("movel", self._movel_args()))
        mover.start()
        time.sleep(0.1)  # movel 卡在 rm_movel 下发中（持有共享提交锁）

        stop_done = threading.Event()
        stopper = threading.Thread(
            target=lambda: (arm.dispatch("stopmotion", {"_tool_name": "joint_control"}), stop_done.set())
        )
        stopper.start()
        time.sleep(0.1)
        self.assertFalse(stop_done.is_set())  # 下发完成前臂的慢停不得发出
        gate.set()
        stopper.join(5.0)
        mover.join(5.0)
        self.assertTrue(stop_done.is_set())
        order = [entry[0] for entry in self.client.calls]
        self.assertLess(order.index("rm_movel"), order.index("rm_set_arm_slow_stop"))

    def test_pose_query_failure_submits_nothing(self):
        # 位姿查询在下发之前：查询失败时不得下发任何运动命令、不得启动监控
        class NoPoseClient(self.FakeClient):
            def call(self, method):
                if method == "rm_get_current_arm_state":
                    raise RuntimeError("pose query failed")
                return super().call(method)

        self.client = NoPoseClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client, {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}}, arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        with self.assertRaisesRegex(RuntimeError, "pose query failed"):
            self.plugin.dispatch("movel", self._movel_args())
        self.assertEqual([], [entry for entry in self.client.calls if entry[0] == "rm_movel"])
        self.assertEqual([], self.acp_events)
        self.assertFalse(self.plugin._motion_lock.locked())  # 运动锁已释放

    def test_arrival_device_mismatch_error_is_actionable(self):
        class MismatchClient(self.FakeClient):
            def command(self, method, *args):
                if method in ("rm_movel", "rm_movel_offset"):
                    raise RuntimeError("rm_movel failed with RealMan SDK code -4")
                return super().command(method, *args)

        self.client = MismatchClient()
        arm = self.device.RM75Plugin(self.client, {}, namespace="rm75")
        self.plugin = self.device.CartesianPlugin(
            self.client, {"safety": dict(self.FAST_SAFETY), "cartesian": {"enabled": True}}, arm_plugin=arm, namespace="rm75",
        )
        self.plugin._acp_callback = lambda action_id, status, result: self.acp_events.append(
            (action_id, status, result)
        )

        with self.assertRaisesRegex(RuntimeError, "笛卡尔设备"):
            self.plugin.dispatch("movel", self._movel_args())
        self.assertEqual([], self.acp_events)
        self.assertFalse(self.plugin._motion_lock.locked())

    def test_motion_guards(self):
        self.client.motion_enabled = False
        with self.assertRaisesRegex(PermissionError, "motion is locked"):
            self.plugin.dispatch("movel", self._movel_args())
        self.client.motion_enabled = True

        with self.assertRaisesRegex(ValueError, "confirm_motion must be true"):
            self.plugin.dispatch("movel", self._movel_args(confirm_motion=False))

        for bad_speed in (0, 11):
            with self.subTest(speed=bad_speed):
                with self.assertRaisesRegex(ValueError, "speed_percent"):
                    self.plugin.dispatch("movel", self._movel_args(speed_percent=bad_speed))

        with self.assertRaisesRegex(ValueError, "x_mm must be a number"):
            self.plugin.dispatch("movel", self._movel_args(x_mm="bad"))

        with self.assertRaisesRegex(ValueError, "waypoint 0"):
            self.plugin.dispatch("movep", {
                "waypoints": [[1, 2, 3]], "speed_percent": 5, "cartesian_enabled": True, "confirm_motion": True,
            })

    def test_concurrent_motion_with_joint_control_is_rejected(self):
        self.arm._motion_lock.acquire()
        try:
            with self.assertRaisesRegex(RuntimeError, "another motion is active"):
                self.plugin.dispatch("movel", self._movel_args())
        finally:
            self.arm._motion_lock.release()

    def test_info_returns_motion_status(self):
        info = self.plugin.dispatch("info", {})
        self.assertEqual("ready", info["state"])
        self.assertIsNone(info["active_action_id"])
        self.assertEqual(10, info["max_speed_percent"])

    def test_cartesian_card_is_advertised(self):
        manifest = (DRIVER / "driver.yaml").read_text()
        self.assertIn("name: abs_move", manifest)


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
        plugin = self.device.RM75Plugin(client, {}, namespace="test_robot")
        tools = plugin.get_tools()
        self.assertEqual(
            {"connection", "joint_states", "model", "robot_info", "software_info", "arm_all_state", "controller_state", "joint_control"},
            {item["name"].split(".")[-1] for item in tools},
        )
        joint_control = next(item for item in tools if item["name"] == "joint_control")
        self.assertEqual("actuator", joint_control["type"])
        self.assertEqual(["set"], joint_control["inputSchema"]["x-completion"]["actions"])
        self.assertEqual(
            {"on_interrupt_motion": {"action": "stopmotion"}},
            joint_control["inputSchema"]["x-hooks"],
        )
        self.assertIs(True, joint_control["inputSchema"]["x-is-dangerous"])
        self.assertEqual(10, joint_control["inputSchema"]["properties"]["speed_percent"]["maximum"])
        self.assertNotIn("timeout_seconds", joint_control["inputSchema"]["properties"])
        self.assertEqual(305, joint_control["inputSchema"]["x-completion"]["timeout"])
        joint_states = next(item for item in tools if item["name"] == "joint_states")
        expected_topic_out = [
            {"topic": "/test_robot/state/joints", "format": "sensor/skeleton"}
        ]
        self.assertEqual(expected_topic_out, joint_states["topic_out"])
        self.assertEqual(
            expected_topic_out,
            plugin.dispatch("info", {"_tool_name": "joint_states"})["topic_out"],
        )
        descriptions = {
            name: joint_control["inputSchema"]["properties"][name]["description"]
            for name in (f"joint{i}_deg" for i in range(1, 8))
        }
        for index, (low, high) in enumerate(self.device.JOINT_LIMITS_DEG, 1):
            self.assertEqual(f"[{low:g}°, {high:g}°]", descriptions[f"joint{index}_deg"])

    def test_controller_trajectory_event_completes_nonblocking_command(self):
        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        robot = mock.Mock()
        robot.rm_movel_offset.return_value = 0
        client._robot = robot
        client._handle = mock.Mock(id=7)

        completion = mock.Mock()
        client.command_trajectory(
            "rm_movel_offset", [0.01] + [0.0] * 5, 5, 0, 0, 1, 0,
            completion_callback=completion,
        )
        client._on_arm_event(mock.Mock(
            event_type=1, device=0, handle_id=7, trajectory_state=True,
        ))

        completion.assert_called_once_with(True)
        self.assertIs(True, client.wait_trajectory(0.1))
        robot.rm_movel_offset.assert_called_once_with([0.01] + [0.0] * 5, 5, 0, 0, 1, 0)

    def test_late_event_is_drained_before_next_trajectory_is_registered(self):
        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        robot = mock.Mock()
        robot.rm_movel_offset.return_value = 0
        client._robot = robot
        client._handle = mock.Mock(id=8)
        args = ([0.01] + [0.0] * 5, 5, 0, 0, 1, 0)
        first_completion = mock.Mock()
        second_completion = mock.Mock()

        client.command_trajectory(
            "rm_movel_offset", *args, completion_callback=first_completion
        )
        self.assertIsNone(client.wait_trajectory(0.0))
        self.assertEqual("draining", client.trajectory_wait_state())
        with self.assertRaisesRegex(RuntimeError, "still draining"):
            client.command_trajectory(
                "rm_movel_offset", *args, completion_callback=second_completion
            )

        # The first command's late event only drains its retired slot.  It must
        # not call either the cancelled callback or a future callback.
        client._on_arm_event(mock.Mock(
            event_type=1, device=0, handle_id=8, trajectory_state=True,
        ))
        self.assertEqual("idle", client.trajectory_wait_state())
        first_completion.assert_not_called()
        second_completion.assert_not_called()

        client.command_trajectory(
            "rm_movel_offset", *args, completion_callback=second_completion
        )
        self.assertEqual("waiting", client.trajectory_wait_state())
        second_completion.assert_not_called()
        client._on_arm_event(mock.Mock(
            event_type=1, device=0, handle_id=8, trajectory_state=True,
        ))
        second_completion.assert_called_once_with(True)
        self.assertIs(True, client.wait_trajectory(0.1))

    def test_trajectory_command_is_not_blocked_by_stalled_state_query_lock(self):
        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        robot = mock.Mock()
        robot.rm_movel_offset.return_value = 0
        client._robot = robot
        client._handle = mock.Mock(id=10)

        client._lock.acquire()
        try:
            done = threading.Event()
            error = []

            def submit():
                try:
                    client.command_trajectory(
                        "rm_movel_offset", [0.01] + [0.0] * 5, 5, 0, 0, 1, 0
                    )
                except Exception as exc:
                    error.append(exc)
                finally:
                    done.set()

            threading.Thread(target=submit).start()
            self.assertTrue(done.wait(1.0))
            self.assertEqual([], error)
            robot.rm_movel_offset.assert_called_once()
        finally:
            client._lock.release()
            client.discard_trajectory_wait()

    def test_completed_event_allows_next_command_while_first_sdk_call_is_blocked(self):
        first_entered = threading.Event()
        release_first = threading.Event()
        call_count = 0

        class Robot:
            def rm_movel_offset(self, *args):
                nonlocal call_count
                call_count += 1
                if call_count == 1:
                    first_entered.set()
                    release_first.wait(5.0)
                    return 1
                return 0

        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        client._robot = Robot()
        client._handle = mock.Mock(id=9)
        first_completion = mock.Mock()
        first_error = []

        def submit_first():
            try:
                client.command_trajectory(
                    "rm_movel_offset", [0.01] + [0.0] * 5, 5, 0, 0, 1, 0,
                    completion_callback=first_completion,
                )
            except Exception as exc:
                first_error.append(exc)

        first_thread = threading.Thread(target=submit_first)
        first_thread.start()
        self.assertTrue(first_entered.wait(1.0))
        client._on_arm_event(mock.Mock(
            event_type=1, device=0, handle_id=9, trajectory_state=True,
        ))
        first_completion.assert_called_once_with(True)

        second_completion = mock.Mock()
        client.command_trajectory(
            "rm_movel_offset", [0.02] + [0.0] * 5, 5, 0, 0, 1, 0,
            completion_callback=second_completion,
        )
        release_first.set()
        first_thread.join(1.0)
        self.assertFalse(first_thread.is_alive())
        self.assertEqual(1, len(first_error))

        client._on_arm_event(mock.Mock(
            event_type=1, device=0, handle_id=9, trajectory_state=True,
        ))
        second_completion.assert_called_once_with(True)
        self.assertIs(True, client.wait_trajectory(0.1))

    def test_enabled_driver_reports_missing_host_sdk_mount(self):
        with mock.patch.dict(os.environ, {"RM_DRIVER_ENABLED": "1", "RM_ARM_IP": "192.0.2.1"}, clear=True):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        with mock.patch.object(self.device, "SDK_LIBRARY_PATH", Path("/definitely/missing/libapi_c.so")):
            with self.assertRaisesRegex(FileNotFoundError, "mount RM_API2_LIB_DIR"):
                client.start()

    def test_sdk_call_reconnects_after_initial_connection_failure(self):
        class Handle:
            def __init__(self, handle_id):
                self.id = handle_id

        robot = mock.Mock()
        robot.rm_get_joint_degree.return_value = (0, [0.0] * 7)
        robot.rm_create_robot_arm.side_effect = [Handle(-1), Handle(12)]
        callback_factory = mock.Mock(side_effect=lambda callback: callback)
        sdk_module = type("SDK", (), {
            "RoboticArm": mock.Mock(return_value=robot),
            "rm_event_callback_ptr": callback_factory,
            "rm_thread_mode_e": type("Mode", (), {"RM_TRIPLE_MODE_E": 3}),
        })

        with mock.patch.dict(os.environ, {
            "RM_DRIVER_ENABLED": "1", "RM_ARM_IP": "192.0.2.1",
        }, clear=True), mock.patch.object(
            self.device, "SDK_LIBRARY_PATH", Path(__file__)
        ), mock.patch.dict(
            sys.modules, {"Robotic_Arm.rm_robot_interface": sdk_module}
        ):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
            with self.assertRaisesRegex(ConnectionError, "192.0.2.1:8080"):
                client.start()
            self.assertEqual("disconnected", client.status()["state"])
            self.assertIn("handle=-1", client.status()["last_connection_error"])
            self.assertEqual([0.0] * 7, client.call("rm_get_joint_degree"))

        self.assertTrue(client.connected)
        self.assertIsNone(client.status()["last_connection_error"])
        self.assertEqual(2, robot.rm_create_robot_arm.call_count)
        robot.rm_create_robot_arm.assert_called_with("192.0.2.1", 8080)

    def test_start_is_idempotent_for_shared_sdk_handle(self):
        class Handle:
            id = 13

        robot = mock.Mock()
        robot.rm_create_robot_arm.return_value = Handle()
        sdk_module = type("SDK", (), {
            "RoboticArm": mock.Mock(return_value=robot),
            "rm_event_callback_ptr": mock.Mock(side_effect=lambda callback: callback),
            "rm_thread_mode_e": type("Mode", (), {"RM_TRIPLE_MODE_E": 3}),
        })

        with mock.patch.dict(os.environ, {
            "RM_DRIVER_ENABLED": "1", "RM_ARM_IP": "192.0.2.1",
        }, clear=True), mock.patch.object(
            self.device, "SDK_LIBRARY_PATH", Path(__file__)
        ), mock.patch.dict(
            sys.modules, {"Robotic_Arm.rm_robot_interface": sdk_module}
        ):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
            client.start()
            client.start()

        sdk_module.RoboticArm.assert_called_once_with(3)
        robot.rm_create_robot_arm.assert_called_once_with("192.0.2.1", 8080)

    def test_tool_start_returns_contract_lifecycle_state(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        plugin = self.device.RM75Plugin(client, {})
        self.assertEqual({"state": "running"}, plugin.dispatch("start", {"_tool_name": "joint_states"}))
        self.assertEqual({"state": "ready"}, plugin.dispatch("start", {"_tool_name": "joint_control"}))
        self.assertEqual({"state": "ready"}, plugin.dispatch("start", {"_tool_name": "model"}))

    def test_state_publisher_starts_when_initial_sdk_connection_fails(self):
        client = mock.Mock()
        client.start.side_effect = ConnectionError("controller not ready")
        ros2 = mock.Mock()
        plugin = self.device.RM75Plugin(client, {}, ros2=ros2)
        plugin._start_skeleton_publisher = mock.Mock()

        with self.assertRaisesRegex(ConnectionError, "controller not ready"):
            plugin.start()

        plugin._start_skeleton_publisher.assert_called_once_with()

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

    def test_skeleton_publisher_uses_urdf_joint_names_and_radians(self):
        class Handle:
            id = 1

        class Robot:
            def rm_get_joint_degree(self):
                return 0, [0, 90, -90, 45, -45, 180, -180]

        class StringMessage:
            def __init__(self):
                self.data = ""

        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client._handle = Handle()
        client._robot = Robot()
        plugin = self.device.RM75Plugin(client, {}, namespace="test_robot")
        plugin._skeleton_pub = mock.Mock()
        plugin._skeleton_message_type = StringMessage

        plugin._publish_skeleton()

        message = plugin._skeleton_pub.publish.call_args.args[0]
        payload = json.loads(message.data)
        self.assertEqual("sensor/skeleton", payload["format"])
        self.assertEqual("rad", payload["position_unit"])
        self.assertEqual(7, payload["joint_count"])
        self.assertEqual(self.device.JOINT_NAMES, [joint["name"] for joint in payload["joints"]])
        self.assertEqual(list(range(7)), [joint["idx"] for joint in payload["joints"]])
        self.assertAlmostEqual(math.pi / 2, payload["joints"][1]["q"])
        self.assertAlmostEqual(-math.pi, payload["joints"][6]["q"])
        urdf = ET.parse(DRIVER / "resource" / "rm75_6f_v.urdf").getroot()
        movable_names = [
            joint.attrib["name"]
            for joint in urdf.findall("joint")
            if joint.attrib.get("type") != "fixed"
        ]
        self.assertEqual(movable_names, [joint["name"] for joint in payload["joints"]])

    def test_sdk_error_is_not_returned_as_sensor_data(self):
        with self.assertRaisesRegex(RuntimeError, "code 5"):
            self.device._sdk_result("rm_get_robot_info", (5, {}))

    def test_all_advertised_read_only_methods_accept_their_sdk_return_shapes(self):
        class Handle:
            id = 1

        class Robot:
            def rm_get_robot_info(self):
                return 0, {"arm_dof": 7}

            def rm_get_arm_software_info(self):
                return 0, {"product_version": "test"}

            def rm_get_arm_all_state(self):
                return 0, {"joint_en_flag": [1] * 7}

            def rm_get_controller_state(self):
                return {"return_code": 0, "voltage": 48.0, "current": 1.0,
                        "temperature": 30.0, "system_error": 0}

        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client._handle = Handle()
        client._robot = Robot()
        plugin = self.device.RM75Plugin(client, {})
        for name in plugin.METHODS:
            result = plugin.dispatch("get", {"_tool_name": name})
            self.assertIsInstance(result, dict, name)
        self.assertEqual(0, plugin.dispatch("get", {"_tool_name": "controller_state"})["return_code"])

    def test_controller_state_rejects_nonzero_return_code(self):
        class Handle:
            id = 1

        class Robot:
            def rm_get_controller_state(self):
                return {"return_code": -2}

        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client._handle = Handle()
        client._robot = Robot()
        with self.assertRaisesRegex(RuntimeError, "code -2"):
            client.call_dict("rm_get_controller_state")

    def test_joint_acp_requires_ca_cert(self):
        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        plugin = self.device.RM75Plugin(client, {})
        with mock.patch.dict(os.environ, {"AGENT_CORE_CA_CERT": ""}, clear=True), \
                mock.patch("urllib.request.urlopen") as urlopen:
            plugin._acp_callback("action-noca", "completed", {"max_error_deg": 0.1})
        urlopen.assert_not_called()
        completion = plugin._motion_status()["last_completion"]
        self.assertEqual("failed", completion["callback"])
        self.assertEqual("AGENT_CORE_CA_CERT is required", completion["callback_error"])

    def test_acp_posts_standard_completion_from_worker_context(self):
        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        plugin = self.device.RM75Plugin(client, {})
        context = ssl.create_default_context()
        with mock.patch.dict(os.environ, {
                "AGENT_CORE_URL": "https://phanthy-motus:15678",
                "AGENT_CORE_CA_CERT": "/tmp/ca.pem",
            }, clear=True), \
                mock.patch("ssl.create_default_context", return_value=context) as create_context, \
                mock.patch("urllib.request.urlopen") as urlopen:
            urlopen.return_value.__enter__.return_value.read.return_value = b'{"ok":true,"action_id":"action-2"}'
            plugin._acp_callback("action-2", "completed", {"max_error_deg": 0.1})
            create_context.assert_called_once_with(cafile="/tmp/ca.pem")
            urlopen.assert_called_once()
            acp_call = urlopen.call_args
            self.assertEqual(
                "https://phanthy-motus:15678/api/acp/complete",
                acp_call.args[0].full_url,
            )
            self.assertIs(context, acp_call.kwargs["context"])
            self.assertTrue(context.check_hostname)
            self.assertEqual(ssl.CERT_REQUIRED, context.verify_mode)
            request_payload = json.loads(acp_call.args[0].data)
            self.assertEqual("action-2", request_payload["action_id"])
            self.assertEqual("completed", request_payload["status"])
            self.assertEqual(plugin.PREFIX, request_payload["tool"])
            self.assertEqual({"reason": "target_reached"}, request_payload["result"])
            completion = plugin._motion_status()["last_completion"]
            self.assertEqual("accepted", completion["callback"])
            self.assertEqual({"max_error_deg": 0.1}, completion["result"])
            self.assertIsNone(acp_call.args[0].get_header("Authorization"))

    def test_acp_rejected_mismatched_and_malformed_ack_remain_visible(self):
        for reply in (b'{"ok":false}', b'{"ok":true,"action_id":"other"}', b'not-json'):
            with self.subTest(reply=reply):
                plugin, _ = self._motion_plugin()
                callback = self.device.RM75Plugin._acp_callback
                with mock.patch.dict(os.environ, {"AGENT_CORE_CA_CERT": "/tmp/ca.pem"}), \
                        mock.patch("ssl.create_default_context"), \
                        mock.patch("urllib.request.urlopen") as urlopen:
                    urlopen.return_value.__enter__.return_value.read.return_value = reply
                    callback(plugin, "test-ack", "completed", {"actual_degree": [0]*7})
                info = plugin._motion_status()["last_completion"]
                self.assertEqual("completed", info["status"])
                self.assertEqual("failed", info["callback"])
                self.assertIn("callback_error", info)
                self.assertEqual([0]*7, info["result"]["actual_degree"])

    def test_acp_transport_error_preserves_terminal_status(self):
        plugin, _ = self._motion_plugin()
        with mock.patch.dict(os.environ, {"AGENT_CORE_CA_CERT": "/tmp/ca.pem"}), \
                mock.patch("ssl.create_default_context"), \
                mock.patch("urllib.request.urlopen", side_effect=TimeoutError("timeout")):
            self.device.RM75Plugin._acp_callback(plugin, "test-timeout", "error", {"reason": "motion_stalled"})
        last = plugin._motion_status()["last_completion"]
        self.assertEqual("error", last["status"])
        self.assertEqual("failed", last["callback"])
        self.assertEqual("motion_stalled", last["result"]["reason"])

    def test_acp_compact_errors_keep_reason_and_standard_endpoint(self):
        for status in ("error", "cancelled"):
            with self.subTest(status=status):
                plugin, _ = self._motion_plugin()
                with mock.patch.dict(os.environ, {
                        "AGENT_CORE_URL": "https://localhost:15678/",
                        "AGENT_CORE_CA_CERT": "/tmp/ca.pem",
                    }), mock.patch("ssl.create_default_context"), mock.patch("urllib.request.urlopen") as urlopen:
                    urlopen.return_value.__enter__.return_value.read.return_value = b'{"ok":true,"action_id":"test-error"}'
                    self.device.RM75Plugin._acp_callback(plugin, "test-error", status, {"reason": "stopmotion", "actual_degree": [0]*7})
                self.assertEqual(1, urlopen.call_count)
                request = urlopen.call_args.args[0]
                self.assertEqual("https://localhost:15678/api/acp/complete", request.full_url)
                self.assertEqual({"reason": "stopmotion"}, json.loads(request.data)["result"])
                self.assertEqual("accepted", plugin._motion_status()["last_completion"]["callback"])

    def _motion_plugin(self, *, motion_enabled=True, current=None, all_state=None, safety=None):
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

            def rm_delete_robot_arm(self):
                return 0

        self_module = self.device
        client = self.device.RM75SDKClient({"arm_ip": "192.0.2.1", "tcp_port": 8080})
        client.motion_enabled = motion_enabled
        client._handle = Handle()
        client._robot = Robot()
        safety_config = {"poll_interval_seconds": 0.001}
        safety_config.update(safety or {})
        plugin = self.device.RM75Plugin(client, {"safety": safety_config})
        plugin._acp_callback = mock.Mock()
        return plugin, client._robot

    def test_complete_joint_target_is_sent_as_one_movej(self):
        plugin, robot = self._motion_plugin(current=[10, 20, 30, 40, 50, 60, 70])
        result = plugin._start_motion({
            "joint1_deg": 10, "joint2_deg": 20, "joint3_deg": 31,
            "joint4_deg": 40, "joint5_deg": 50, "joint6_deg": 60,
            "joint7_deg": 70, "speed_percent": 1, "confirm_motion": True,
        })
        self.assertEqual("running", result["state"])
        self.assertEqual(([10, 20, 31, 40, 50, 60, 70], 1, 0, 0, 0), robot.moves[0])

    @staticmethod
    def _seven_targets(**overrides):
        values = {f"joint{i}_deg": 0 for i in range(1, 8)}
        values.update(overrides)
        return values

    def test_missing_joint_fields_keep_current_positions(self):
        current = [10, 20, 30, 40, 50, 60, 70]
        plugin, robot = self._motion_plugin(current=current)
        result = plugin._start_motion({"joint3_deg": 31, "confirm_motion": True})
        self.assertEqual("running", result["state"])
        self.assertEqual(([10, 20, 31, 40, 50, 60, 70], 5, 0, 0, 0), robot.moves[0])

    def test_lifecycle_stop_cancels_motion_and_reports_idle(self):
        plugin, robot = self._motion_plugin(safety={"start_grace_seconds": 60})
        started = plugin._start_motion({"joint1_deg": 1, "confirm_motion": True})
        self.assertEqual({"state": "idle"}, plugin.dispatch("stop", {"_tool_name": "joint_control"}))
        deadline = time.monotonic() + 1
        while not plugin._acp_callback.called and time.monotonic() < deadline:
            time.sleep(0.001)
        plugin._acp_callback.assert_called_once_with(started["action_id"], "cancelled", {"reason": "stopmotion"})
        self.assertEqual(1, robot.stops)

    def test_lifecycle_stop_failure_does_not_claim_idle(self):
        plugin, robot = self._motion_plugin()
        robot.rm_set_arm_slow_stop = lambda: 9
        with self.assertRaisesRegex(RuntimeError, "SDK code 9"):
            plugin.dispatch("stop", {"_tool_name": "joint_control"})
        self.assertEqual({"state": "idle"}, plugin.dispatch("stop", {"_tool_name": "joint_states"}))

    def test_skeleton_outage_backoff_and_recovery(self):
        plugin, robot = self._motion_plugin()
        plugin._skeleton_pub = mock.Mock()
        plugin._skeleton_message_type = mock.Mock
        query = mock.Mock(side_effect=[RuntimeError("first"), RuntimeError("different"), {"position": [0]*7}, RuntimeError("new outage")])
        plugin.client.joint_states = query
        with mock.patch.object(self.device.time, "monotonic", return_value=10) as clock, mock.patch("builtins.print") as log:
            plugin._publish_skeleton()
            for _ in range(20): plugin._publish_skeleton()
            self.assertEqual(1, query.call_count)
            clock.return_value = 12
            plugin._publish_skeleton()
            self.assertEqual(2, query.call_count)
            self.assertEqual(1, log.call_count)
            clock.return_value = 14
            plugin._publish_skeleton()
            plugin._skeleton_pub.publish.assert_called_once()
            clock.return_value = 14.1
            plugin._publish_skeleton()
            self.assertEqual(2, log.call_count)

    def test_skeleton_skips_disconnected_and_busy_client(self):
        plugin, robot = self._motion_plugin()
        plugin._skeleton_pub = mock.Mock()
        plugin._skeleton_message_type = mock.Mock
        plugin.client.joint_states = mock.Mock()
        plugin.client._handle = None
        plugin._publish_skeleton()
        plugin.client.joint_states.assert_not_called()
        plugin.client._handle = mock.Mock(id=1)
        plugin._skeleton_retry_at = 0
        acquired = threading.Event(); release = threading.Event()
        def hold():
            with plugin.client._lock:
                acquired.set(); release.wait(2)
        thread = threading.Thread(target=hold); thread.start()
        try:
            self.assertTrue(acquired.wait(1))
            plugin._publish_skeleton()
            plugin.client.joint_states.assert_not_called()
        finally:
            release.set(); thread.join(2)

    def test_motion_requires_both_interlocks(self):
        plugin, robot = self._motion_plugin(motion_enabled=False)
        with self.assertRaisesRegex(PermissionError, "motion is locked"):
            plugin._start_motion({**self._seven_targets(joint1_deg=1), "confirm_motion": True})
        with self.assertRaisesRegex(ValueError, "confirm_motion"):
            plugin, robot = self._motion_plugin()
            plugin._start_motion(self._seven_targets(joint1_deg=1))
        self.assertEqual([], robot.moves)

    def test_motion_rejects_robot_error(self):
        plugin, robot = self._motion_plugin(all_state={
            "joint_err_code": [0, 0, 3, 0, 0, 0, 0],
            "joint_en_flag": [1] * 7,
            "err": {"err_len": 0, "err": []},
        })
        with self.assertRaisesRegex(RuntimeError, "joint error"):
            plugin._start_motion({**self._seven_targets(joint1_deg=1), "confirm_motion": True})

    def test_absolute_target_is_not_rejected_for_distance_from_current(self):
        plugin, robot = self._motion_plugin(current=[-10, 0, 0, 0, 0, 0, 0])
        result = plugin._start_motion({**self._seven_targets(joint1_deg=20), "confirm_motion": True})
        self.assertEqual("running", result["state"])
        self.assertEqual(20, robot.moves[0][0][0])

    def test_zero_arm_error_code_is_not_treated_as_an_error(self):
        plugin, robot = self._motion_plugin(all_state={
            "joint_err_code": [0] * 7,
            "joint_en_flag": [1] * 7,
            "err": {"err_len": 1, "err": ["0"]},
        })
        result = plugin._start_motion({**self._seven_targets(), "confirm_motion": True})
        self.assertEqual("running", result["state"])
        self.assertEqual(1, len(robot.moves))

    def test_motion_reports_running_then_acp_completed(self):
        plugin, robot = self._motion_plugin(current=[0.0] * 7)
        result = plugin._start_motion({**self._seven_targets(), "confirm_motion": True})
        self.assertEqual("running", result["state"])
        self.assertTrue(result["action_id"].startswith("rm75_movej_"))
        self.assertEqual({"state", "action_id"}, set(result))
        deadline = time.monotonic() + 1.0
        while not plugin._acp_callback.called and time.monotonic() < deadline:
            time.sleep(0.01)
        plugin._acp_callback.assert_called_once()
        action_id, status, completion = plugin._acp_callback.call_args.args
        self.assertEqual(result["action_id"], action_id)
        self.assertEqual("completed", status)
        self.assertEqual([0.0] * 7, completion["target_degree"])

    def test_motion_stall_requests_slow_stop_and_reports_acp_error(self):
        plugin, robot = self._motion_plugin(
            current=[0.0] * 7,
            safety={
                "start_grace_seconds": 0,
                "stall_timeout_seconds": 0.01,
                "progress_threshold_deg": 0.05,
            },
        )
        result = plugin._start_motion({
            **self._seven_targets(joint1_deg=1),
            "speed_percent": 1,
            "confirm_motion": True,
        })
        self.assertEqual("running", result["state"])
        deadline = time.monotonic() + 1.0
        while not plugin._acp_callback.called and time.monotonic() < deadline:
            time.sleep(0.01)
        plugin._acp_callback.assert_called_once()
        action_id, status, completion = plugin._acp_callback.call_args.args
        self.assertEqual(result["action_id"], action_id)
        self.assertEqual("error", status)
        self.assertEqual("motion_stalled", completion["reason"])
        self.assertEqual(1, robot.stops)

    def test_agent_core_interrupt_hook_stops_pending_motion_through_mcp(self):
        runtime_spec = importlib.util.spec_from_file_location(
            "realman_interrupt_vendor_runtime", ROOT / "common" / "vendor_runtime.py"
        )
        runtime = importlib.util.module_from_spec(runtime_spec)
        runtime_spec.loader.exec_module(runtime)
        plugin, robot = self._motion_plugin(
            current=[0.0] * 7,
            safety={"start_grace_seconds": 60},
        )
        bundle = runtime.DriverBundle([plugin])
        handler_type = runtime.make_handler(lambda: bundle, "test", "test")

        def call_mcp(request_id, method, params):
            body = json.dumps({
                "jsonrpc": "2.0",
                "id": request_id,
                "method": method,
                "params": params,
            }).encode()
            handler = object.__new__(handler_type)
            handler.path = "/mcp"
            handler.headers = {"Content-Length": str(len(body))}
            handler.rfile = io.BytesIO(body)
            response = {}
            handler.send_json = lambda status, payload: response.update(
                status=status, payload=payload
            )
            handler.do_POST()
            self.assertEqual(200, response["status"])
            return response["payload"]

        listed = call_mcp(1, "tools/list", {})
        joint_control = next(
            item for item in listed["result"]["tools"] if item["name"] == "joint_control"
        )
        interrupt = joint_control["inputSchema"]["x-hooks"]["on_interrupt_motion"]
        self.assertEqual({"action": "stopmotion"}, interrupt)

        started_rpc = call_mcp(2, "tools/call", {
            "name": "joint_control",
            "arguments": {
                "action": "set",
                "joint1_deg": 1,
                "speed_percent": 1,
                "confirm_motion": True,
            },
        })
        started = json.loads(started_rpc["result"]["content"][0]["text"])
        self.assertEqual("running", started["state"])
        self.assertEqual(started["action_id"], plugin._active_action_id)

        stopped_rpc = call_mcp(3, "tools/call", {
            "name": "joint_control",
            "arguments": {"action": interrupt["action"]},
        })
        stopped = json.loads(stopped_rpc["result"]["content"][0]["text"])
        self.assertEqual("stop_requested", stopped["state"])
        self.assertEqual(started["action_id"], stopped["action_id"])
        self.assertEqual(1, robot.stops)

        deadline = time.monotonic() + 1.0
        while not plugin._acp_callback.called and time.monotonic() < deadline:
            time.sleep(0.01)
        plugin._acp_callback.assert_called_once()
        action_id, status, completion = plugin._acp_callback.call_args.args
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("cancelled", status)
        self.assertEqual("stopmotion", completion["reason"])

    def test_interrupt_during_move_submission_waits_and_cancels_same_action(self):
        plugin, robot = self._motion_plugin()
        submitted = threading.Event()
        release = threading.Event()
        stop_attempted = threading.Event()
        observed = {}
        errors = []
        original_move = robot.rm_movej

        def move(*args):
            observed["reserved"] = plugin._active_action_id
            acquired = plugin._action_lock.acquire(blocking=False)
            observed["submission_locked"] = not acquired
            if acquired:
                plugin._action_lock.release()
            submitted.set()
            if not release.wait(2):
                raise RuntimeError("test submission release timed out")
            return original_move(*args)

        def start():
            try:
                observed["start"] = plugin._start_motion({"confirm_motion": True})
            except Exception as exc:
                errors.append(exc)

        def stop():
            stop_attempted.set()
            try:
                observed["stop"] = plugin._stop_motion()
            except Exception as exc:
                errors.append(exc)

        robot.rm_movej = move
        # Hold the monitor out of the race; this test isolates submission vs stop.
        with mock.patch.object(plugin, "_monitor_motion"):
            starter = threading.Thread(target=start)
            stopper = threading.Thread(target=stop)
            starter.start()
            try:
                self.assertTrue(submitted.wait(1))
                stopper.start()
                self.assertTrue(stop_attempted.wait(1))
            finally:
                release.set()
                starter.join(2)
                if stopper.ident is not None:
                    stopper.join(2)
        self.assertFalse(starter.is_alive())
        self.assertFalse(stopper.is_alive())
        self.assertEqual([], errors)
        self.assertTrue(observed["submission_locked"])
        self.assertEqual(observed["start"]["action_id"], observed["reserved"])
        self.assertEqual(observed["start"]["action_id"], observed["stop"]["action_id"])
        self.assertIn(observed["reserved"], plugin._cancelled)
        self.assertEqual(1, robot.stops)

    def test_failed_submission_clears_reserved_id_and_releases_motion_lock(self):
        plugin, robot = self._motion_plugin()
        robot.rm_movej = lambda *args: 9
        with self.assertRaisesRegex(RuntimeError, "SDK code 9"):
            plugin._start_motion({"confirm_motion": True})
        self.assertIsNone(plugin._active_action_id)
        self.assertEqual(set(), plugin._cancelled)
        self.assertTrue(plugin._motion_lock.acquire(blocking=False))
        plugin._motion_lock.release()

    def test_stopmotion_marks_cancelled_before_sdk_stop_and_retains_it_on_failure(self):
        plugin, robot = self._motion_plugin(current=[0.0] * 7)
        action_id = "rm75_movej_stop_race"
        plugin._active_action_id = action_id
        observed = {}

        def failing_stop():
            acquired = plugin._submission_lock.acquire(blocking=False)
            if acquired:
                plugin._submission_lock.release()
            observed["submission_lock_held"] = not acquired
            observed["cancelled_before_sdk"] = action_id in plugin._cancelled
            return 9

        robot.rm_set_arm_slow_stop = failing_stop
        with self.assertRaisesRegex(RuntimeError, "SDK code 9"):
            plugin._stop_motion()

        self.assertTrue(observed["submission_lock_held"])
        self.assertTrue(observed["cancelled_before_sdk"])
        self.assertIn(action_id, plugin._cancelled)


if __name__ == "__main__":
    unittest.main()
