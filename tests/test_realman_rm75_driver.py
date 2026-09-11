import importlib.util
import io
import json
import math
import os
from pathlib import Path
import ssl
import threading
import time
import unittest
import xml.etree.ElementTree as ET
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
        self.assertNotIn("RM_API2_LIB_URL", dockerfile)
        self.assertNotIn("ADD http", dockerfile)
        self.assertIn("COPY deploy/ /deploy/", dockerfile)
        self.assertNotIn("colcon", dockerfile)
        self.assertNotIn("rm_driver", dockerfile)
        self.assertNotIn("python3-pip", dockerfile)
        self.assertNotIn("pip3 install", dockerfile)
        self.assertFalse((DRIVER / "entrypoint.sh").exists())

    def test_vendor_shared_libraries_are_not_committed(self):
        self.assertEqual([], list((DRIVER / "vendor").rglob("libapi_c.so")))
        self.assertEqual([], list((DRIVER / "vendor").rglob("libapi_cpp.so")))

    def test_service_has_motion_capable_rm75_default(self):
        service = (DRIVER / "deploy" / "service.yml").read_text()
        self.assertIn("RM_DRIVER_ENABLED=1", service)
        self.assertIn("RM_MOTION_ENABLED=1", service)
        self.assertIn("RM_ARM_IP=${RM75_ARM_IP:-192.168.1.18}", service)
        self.assertNotIn("AGENT_CORE_CA_CERT", service)
        self.assertNotIn("AGENT_CORE_TOKEN", service)
        self.assertNotIn("/opt/phanthy-motus/data:/opt/phanthy-motus/data:ro", service)
        self.assertIn("network_mode: host", service)
        self.assertNotIn("privileged: true", service)
        self.assertNotIn("/dev:/dev", service)
        self.assertIn("/opt/phanthy-motus/dds-local.xml:/opt/phanthy-motus/dds-local.xml:ro", service)
        self.assertIn("FASTRTPS_DEFAULT_PROFILES_FILE=/opt/phanthy-motus/dds-local.xml", service)
        self.assertIn(
            "${RM_API2_LIB_DIR:-/opt/realman/rm_api2/libs/linux_arm}:/work/Robotic_Arm/libs/linux_arm:ro",
            service,
        )
        self.assertNotIn("/opt/realman/rm_ws", service)
        self.assertNotIn("ipc:", service)


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
            self.client, {"safety": dict(self.FAST_SAFETY)},
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
                "speed_percent": 5, "confirm_motion": True}
        args.update(overrides)
        return args

    def test_plugin_prefix_contract(self):
        self.assertEqual("cartesian_control", self.device.CartesianPlugin.PREFIX)

    def test_tool_schema_declares_safety_contract(self):
        tools = self.plugin.get_tools()
        self.assertEqual(1, len(tools))
        self.assertEqual("cartesian_control", tools[0]["name"])
        self.assertEqual("actuator", tools[0]["type"])
        schema = tools[0]["inputSchema"]
        self.assertIs(True, schema["x-is-dangerous"])
        self.assertEqual(["movel", "move_offset", "movep"], schema["x-completion"]["actions"])
        self.assertIn("confirm_motion", schema["properties"])
        self.assertEqual(10, schema["properties"]["speed_percent"]["maximum"])
        self.assertIn("movel", schema["x-action-params"])
        self.assertIn("movep", schema["x-action-params"])

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
        result = self.plugin.dispatch("move_offset", {
            "dx_mm": 50, "dy_mm": 0, "dz_mm": 0,
            "drx_deg": 0, "dry_deg": 0, "drz_deg": 0,
            "frame_type": "work", "speed_percent": 5, "confirm_motion": True,
        })

        self.assertEqual("running", result["state"])
        # 工作坐标系 frame_type=0，偏移转换为米/弧度
        self.assertIn(
            ("rm_movel_offset", ([0.05, 0.0, 0.0, 0.0, 0.0, 0.0], 5, 0, 0, 0, 0)),
            self.client.calls,
        )
        self.client.pose_mm_deg = [350.0, 0.0, 200.0, 0.0, 0.0, 0.0]
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        _, status, payload = self.acp_events[0]
        self.assertEqual("completed", status)
        self.assertEqual([350.0, 0.0, 200.0, 0.0, 0.0, 0.0], payload["target_pose_mm_deg"])

    def test_movep_chains_waypoints_with_connect_flags(self):
        waypoints = [
            [100, 0, 0, 0, 0, 0],
            [100, 100, 0, 0, 0, 0],
            [100, 100, 100, 0, 0, 0],
        ]
        result = self.plugin.dispatch("movep", {
            "waypoints": waypoints, "speed_percent": 5, "confirm_motion": True,
        })

        self.assertEqual("running", result["state"])
        movel_calls = [entry[1] for entry in self.client.calls if entry[0] == "rm_movel"]
        self.assertEqual(3, len(movel_calls))
        # rm_movel(pose, v, r, connect, block)：前 N-1 个点 connect=1（联合规划），末点 connect=0（立即执行）
        self.assertEqual(1, movel_calls[0][3])
        self.assertEqual(1, movel_calls[1][3])
        self.assertEqual(0, movel_calls[2][3])
        self.assertEqual([0.1, 0.1, 0.1, 0.0, 0.0, 0.0], movel_calls[2][0])

    def test_monitor_stall_sends_slow_stop(self):
        # 位姿一直不前进 → stall 检测触发受控停止并如实上报 motion_stalled
        self.plugin.dispatch("movel", self._movel_args())
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1, timeout=5.0))
        action_id, status, payload = self.acp_events[0]
        self.assertEqual("error", status)
        self.assertEqual("motion_stalled", payload["reason"])
        self.assertIn(("rm_set_arm_slow_stop", ()), self.client.calls)

    def test_stopmotion_cancels_and_slow_stops(self):
        started = self.plugin.dispatch("movel", self._movel_args())
        stop = self.plugin.dispatch("stopmotion", {})

        self.assertEqual("stop_requested", stop["state"])
        self.assertEqual(started["action_id"], stop["action_id"])
        self.assertIn(("rm_set_arm_slow_stop", ()), self.client.calls)
        self.assertTrue(self._wait_for(lambda: len(self.acp_events) == 1))
        action_id, status, payload = self.acp_events[0]
        self.assertEqual(started["action_id"], action_id)
        self.assertEqual("cancelled", status)
        self.assertEqual("stopmotion", payload["reason"])

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
                "waypoints": [[1, 2, 3]], "speed_percent": 5, "confirm_motion": True,
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
        self.assertIn("name: cartesian_control", manifest)


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

    def test_enabled_driver_reports_missing_host_sdk_mount(self):
        with mock.patch.dict(os.environ, {"RM_DRIVER_ENABLED": "1", "RM_ARM_IP": "192.0.2.1"}, clear=True):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        with mock.patch.object(self.device, "SDK_LIBRARY_PATH", Path("/definitely/missing/libapi_c.so")):
            with self.assertRaisesRegex(FileNotFoundError, "mount RM_API2_LIB_DIR"):
                client.start()

    def test_tool_start_returns_contract_lifecycle_state(self):
        with mock.patch.dict(os.environ, {}, clear=True):
            client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        plugin = self.device.RM75Plugin(client, {})
        self.assertEqual({"state": "running"}, plugin.dispatch("start", {"_tool_name": "joint_states"}))
        self.assertEqual({"state": "ready"}, plugin.dispatch("start", {"_tool_name": "joint_control"}))
        self.assertEqual({"state": "ready"}, plugin.dispatch("start", {"_tool_name": "model"}))

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

    def test_acp_posts_standard_completion_from_worker_context(self):
        client = self.device.RM75SDKClient({"arm_ip": "", "tcp_port": 8080})
        plugin = self.device.RM75Plugin(client, {})
        context = mock.Mock()
        with mock.patch.dict(os.environ, {
                "AGENT_CORE_URL": "https://phanthy-motus:15678",
            }, clear=True), \
                mock.patch("ssl.create_default_context", return_value=context) as create_context, \
                mock.patch("urllib.request.urlopen") as urlopen:
            urlopen.return_value.__enter__.return_value.read.return_value = b'{"ok":true,"action_id":"action-2"}'
            plugin._acp_callback("action-2", "completed", {"max_error_deg": 0.1})
            create_context.assert_called_once_with()
            urlopen.assert_called_once()
            acp_call = urlopen.call_args
            self.assertEqual(
                "https://phanthy-motus:15678/api/acp/complete",
                acp_call.args[0].full_url,
            )
            self.assertIs(context, acp_call.kwargs["context"])
            self.assertIs(False, context.check_hostname)
            self.assertEqual(ssl.CERT_NONE, context.verify_mode)
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
                with mock.patch("urllib.request.urlopen") as urlopen:
                    urlopen.return_value.__enter__.return_value.read.return_value = reply
                    callback(plugin, "test-ack", "completed", {"actual_degree": [0]*7})
                info = plugin._motion_status()["last_completion"]
                self.assertEqual("completed", info["status"])
                self.assertEqual("failed", info["callback"])
                self.assertIn("callback_error", info)
                self.assertEqual([0]*7, info["result"]["actual_degree"])

    def test_acp_transport_error_preserves_terminal_status(self):
        plugin, _ = self._motion_plugin()
        with mock.patch("urllib.request.urlopen", side_effect=TimeoutError("timeout")):
            self.device.RM75Plugin._acp_callback(plugin, "test-timeout", "error", {"reason": "motion_stalled"})
        last = plugin._motion_status()["last_completion"]
        self.assertEqual("error", last["status"])
        self.assertEqual("failed", last["callback"])
        self.assertEqual("motion_stalled", last["result"]["reason"])

    def test_acp_compact_errors_keep_reason_and_standard_endpoint(self):
        for status in ("error", "cancelled"):
            with self.subTest(status=status):
                plugin, _ = self._motion_plugin()
                with mock.patch.dict(os.environ, {"AGENT_CORE_URL": "https://localhost:15678/"}), mock.patch("urllib.request.urlopen") as urlopen:
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
            acquired = plugin._action_lock.acquire(blocking=False)
            if acquired:
                plugin._action_lock.release()
            observed["action_lock_held"] = not acquired
            observed["cancelled_before_sdk"] = action_id in plugin._cancelled
            return 9

        robot.rm_set_arm_slow_stop = failing_stop
        with self.assertRaisesRegex(RuntimeError, "SDK code 9"):
            plugin._stop_motion()

        self.assertTrue(observed["action_lock_held"])
        self.assertTrue(observed["cancelled_before_sdk"])
        self.assertIn(action_id, plugin._cancelled)


if __name__ == "__main__":
    unittest.main()
