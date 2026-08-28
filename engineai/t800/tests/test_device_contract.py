import hashlib
import importlib.util
import json
import os
import ssl
import struct
import sys
import tempfile
import threading
import time
import types
import unittest
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))


class Message:
    def __init__(self):
        self.header = types.SimpleNamespace(stamp=None, frame_id="")


class JointMotionPlanRequest(Message):
    REQUEST_PLAN_EXECUTE = 0
    REQUEST_CANCEL = 1
    REQUEST_RESET = 2


class JointMotionPlanState(Message):
    STATUS_DISABLED = 0
    IDLE = 1
    EXECUTING = 2
    EXITING = 3

    def __init__(self, request_id=0, status=STATUS_DISABLED, progress=0.0):
        super().__init__()
        self.request_id = request_id
        self.status = status
        self.progress = progress


class EnableMotor:
    class Request:
        enable = False


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeAudioStdin:
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(data)

    def flush(self):
        pass

    def close(self):
        pass


class FakeAudioProcess:
    def __init__(self):
        self.stdin = FakeAudioStdin()
        self.returncode = None

    def poll(self):
        return self.returncode

    def terminate(self):
        self.returncode = 0

    def wait(self, timeout=None):
        return self.returncode

    def kill(self):
        self.returncode = -9


class DeferredThread:
    def __init__(self, target, args):
        self.target = target
        self.args = args

    def start(self):
        pass

    def join(self, timeout=None):
        pass


class FakeFuture:
    def done(self):
        return True

    def result(self):
        return types.SimpleNamespace(success=True, message="ok")


class FakeClient:
    def service_is_ready(self):
        return True

    def wait_for_service(self, timeout_sec):
        return True

    def call_async(self, request):
        return FakeFuture()


class FakeNode:
    def __init__(self, name, context=None):
        self.name = name
        self.publishers = []
        self.subscriptions = []

    def create_publisher(self, message_type, topic, qos):
        publisher = FakePublisher()
        publisher.topic = topic
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, message_type, topic, callback, qos):
        subscription = types.SimpleNamespace(topic=topic, callback=callback)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_subscription(self, subscription):
        if subscription in self.subscriptions:
            self.subscriptions.remove(subscription)
        return True

    def create_timer(self, period, callback):
        return types.SimpleNamespace(period=period, callback=callback)

    def create_client(self, service_type, name):
        return FakeClient()

    def get_clock(self):
        return types.SimpleNamespace(now=lambda: types.SimpleNamespace(to_msg=lambda: "stamp"))

    def get_topic_names_and_types(self):
        return []

    def get_service_names_and_types(self):
        return []

    def count_publishers(self, _name):
        return 0

    def count_subscribers(self, _name):
        return 0

    def count_services(self, _name):
        return 0

    def count_clients(self, _name):
        return 0


class FakeExecutor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class FakeRos:
    def __init__(self):
        self.ctx_robot = object()
        self.ctx_core = object()
        self.executor_robot = FakeExecutor()
        self.executor_core = FakeExecutor()


def install_ros_stubs():
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_node.Node = FakeNode
    rclpy_qos = types.ModuleType("rclpy.qos")
    rclpy_qos.QoSProfile = lambda **kwargs: types.SimpleNamespace(**kwargs)
    rclpy_qos.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1, RELIABLE=2)
    rclpy_qos.HistoryPolicy = types.SimpleNamespace(KEEP_LAST=1)
    rclpy_qos.DurabilityPolicy = types.SimpleNamespace(VOLATILE=1)
    sys.modules["rclpy.node"] = rclpy_node
    sys.modules["rclpy.qos"] = rclpy_qos

    std_msgs = types.ModuleType("std_msgs.msg")
    std_msgs.Header = type("Header", (Message,), {})
    std_msgs.String = type("String", (Message,), {"__init__": lambda self: setattr(self, "data", "")})
    std_msgs.UInt8MultiArray = type("UInt8MultiArray", (Message,), {})
    sys.modules["std_msgs.msg"] = std_msgs

    sensor_msgs = types.ModuleType("sensor_msgs.msg")
    sensor_msgs.PointCloud2 = type("PointCloud2", (Message,), {})
    sensor_msgs.CompressedImage = type("CompressedImage", (Message,), {})
    sensor_msgs.Image = type("Image", (Message,), {})
    sys.modules["sensor_msgs.msg"] = sensor_msgs

    nav_msgs = types.ModuleType("nav_msgs.msg")
    nav_msgs.Odometry = type("Odometry", (Message,), {})
    sys.modules["nav_msgs.msg"] = nav_msgs

    audio_msgs = types.ModuleType("audio_msgs.msg")
    audio_msgs.AudioChunk = type(
        "AudioChunk",
        (Message,),
        {"__init__": lambda self: (Message.__init__(self), setattr(self, "format", ""),
                                    setattr(self, "data", []))[-1]},
    )
    sys.modules["audio_msgs.msg"] = audio_msgs

    protocol_msg = types.ModuleType("interface_protocol.msg")
    for name in (
        "BodyVelCmd", "GamepadKeys", "ImuInfo", "JointCommand",
        "JointOverrideCommand", "JointState", "LedControl", "MotionState", "MotionStateRequest",
        "Heartbeat", "LinkInfo", "MotorDebug", "NodeControl", "PowerInfo", "Tts",
    ):
        setattr(protocol_msg, name, type(name, (Message,), {}))
    protocol_msg.JointMotionPlanRequest = JointMotionPlanRequest
    protocol_msg.JointMotionPlanState = JointMotionPlanState
    protocol_srv = types.ModuleType("interface_protocol.srv")
    protocol_srv.EnableMotor = EnableMotor
    protocol = types.ModuleType("interface_protocol")
    protocol.msg = protocol_msg
    protocol.srv = protocol_srv
    sys.modules["interface_protocol"] = protocol
    sys.modules["interface_protocol.msg"] = protocol_msg
    sys.modules["interface_protocol.srv"] = protocol_srv


def load_device():
    install_ros_stubs()
    spec = importlib.util.spec_from_file_location("t800_device_contract", ROOT / "device.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


CONFIG = {
    "ros": {"robot_domain_id": 69, "core_domain_id": 42, "source_timeout_sec": 1.0},
    "control": {"velocity_rate_hz": 100.0, "low_level_rate_hz": 200.0, "override_rate_hz": 100.0,
                "max_vx": 1.0, "max_vy": 1.0, "max_vyaw": 1.0, "mode_transition_timeout_sec": 0.1,
                "stream_watchdog_period_sec": 0.5},
    "topics": {
        "joint_state": "/hardware/joint_state", "imu": "/hardware/imu_info",
        "gamepad": "/hardware/gamepad_keys", "motor_debug": "/hardware/motor_debug",
        "motor_state": "/hardware/motor_state", "motor_command": "/hardware/motor_command",
        "joint_command_feedback": "/hardware/joint_command_feedback",
        "power": "/hardware/power_info", "motion_state": "/motion/motion_state",
        "body_velocity": "/motion/body_vel_cmd", "motion_request": "/motion/set_motion_state",
        "led": "/hardware/led_control", "joint_plan_request": "/motion/joint_motion_plan/request",
        "joint_plan_state": "/motion/joint_motion_plan/state",
        "joint_override": "/motion/joint_override_command", "joint_command": "/hardware/joint_command",
        "tts": "/hardware/tts", "native_node_control": "/motion/node_control",
        "heartbeat": "/heartbeat",
        "odometry": "/manifold/ODIN2/device0/odometry",
        "vision_cloud_raw": "/manifold/ODIN2/device0/cloud/raw",
        "vision_cloud_slam": "/manifold/ODIN2/device0/cloud/slam",
        "vision_camera_left": "/manifold/ODIN2/device0/camera0/compressed",
        "vision_camera_right": "/manifold/ODIN2/device0/camera1/compressed",
        "vision_depth": "/manifold/ODIN2/device0/depth",
    },
    "services": {"enable_motor": "/hardware/enable_motor"},
    "diagnostics": {"command_trace_capacity": 20, "motion_events_capacity": 100},
}


class DevicePluginContractTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.device = load_device()

    def setUp(self):
        self.ros = FakeRos()
        self.state = self.device.StatePlugin(CONFIG, "robot", self.ros)
        self._original_notify_acp = self.device._notify_acp_completion

    def tearDown(self):
        self.device._notify_acp_completion = self._original_notify_acp

    def test_complete_tool_surface_is_declared(self):
        from virtual_gamepad import VirtualGamepadPlugin

        motion_mode = self.device.MotionModePlugin(CONFIG, "robot", self.ros, self.state)
        joint_plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros)
        plugins = [
            self.state,
            self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state),
            motion_mode,
            self.device.DancePlugin(motion_mode, self.state),
            joint_plan,
            self.device.GesturePlugin(joint_plan),
            self.device.HeadActuatorPlugin(CONFIG, joint_plan, self.state),
            self.device.JointOverridePlugin(CONFIG, "robot", self.ros, self.state),
            self.device.JointBridgePlugin(CONFIG, "robot", self.ros, self.state),
            self.device.LedPlugin(CONFIG, "robot", self.ros),
            self.device.TtsPlugin(CONFIG, "robot", self.ros),
            self.device.MicPlugin(CONFIG, "robot", self.ros),
            self.device.SpeakerPlugin(CONFIG, "robot", self.ros),
            self.device.VisionPlugin(CONFIG, "robot", self.ros),
            self.device.MotorPowerPlugin(CONFIG, "robot", self.ros),
            self.device.NativeNodeControlPlugin(CONFIG, "robot", self.ros),
            self.device.SafetyControlPlugin(CONFIG, "robot", self.ros, self.state),
            self.device.NativeSdkPlugin({"mode": "external"}, "robot", self.ros),
            self.device.HeartbeatStatusPlugin(CONFIG, "robot", self.ros),
            self.device.MotionCommandTracePlugin(CONFIG, "robot", self.ros),
            self.device.MotionEventsPlugin(CONFIG, "robot", self.ros),
            self.device.NativeInterfaceProbePlugin(CONFIG, "robot", self.ros),
            VirtualGamepadPlugin({}, "robot", self.ros),
        ]
        names = set()
        definitions = []
        for plugin in plugins:
            tools = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            definitions.extend(tools)
            names.update(tool["name"] for tool in tools)
        self.assertEqual(
            {"joints", "imu", "battery", "motor_health", "motor_state", "motor_command", "joint_command_feedback",
             "gamepad", "motion_state", "driver_health", "model",
             "robot_snapshot", "fault_summary", "stability", "joint_groups", "capabilities", "ros_graph",
             "mainboard", "heartbeat_status", "motion_command_trace", "motion_events",
             "native_interface_probe",
             "loco", "motion_mode", "dance", "joint_plan", "joint_plan_state", "gesture", "head",
             "joint_override", "joint_bridge",
             "led", "tts", "mic", "speaker", "pointcloud", "camera", "depth",
             "motor_power", "native_node_control", "virtual_gamepad", "safety", "native_sdk"},
            names,
        )
        self.assertEqual(43, len(names))
        self.assertEqual(43, len(definitions), "tool names must be unique")
        for tool in definitions:
            schema = tool.get("inputSchema")
            self.assertEqual("object", schema.get("type"), tool["name"])
            properties = schema.get("properties", {})
            action = properties.get("action")
            if not action:
                continue
            enum = action.get("enum", [])
            self.assertEqual(len(enum), len(set(enum)), tool["name"])
            for action_name, detail in schema.get("x-action-params", {}).items():
                self.assertIn(action_name, enum, tool["name"])
                for parameter in detail.get("params", []):
                    self.assertIn(parameter, properties, f"{tool['name']}.{action_name}")

    def test_latest_head_and_four_pr_cards_are_available_together(self):
        self.assertTrue(hasattr(self.device, "GaitPlugin"))
        self.assertTrue(hasattr(self.device, "MotionRecorderPlugin"))
        motion_mode = self.device.MotionModePlugin(CONFIG, "robot", self.ros, self.state)
        gait = self.device.GaitPlugin(CONFIG, motion_mode, self.state)
        with tempfile.TemporaryDirectory() as recordings_dir:
            config = {
                **CONFIG,
                "plugins": {
                    **CONFIG.get("plugins", {}),
                    "motion_recorder": {"recordings_dir": recordings_dir},
                },
            }
            recorder = self.device.MotionRecorderPlugin(config, "robot", self.ros)
            joint_plan = self.device.JointPlanPlugin(
                CONFIG, "robot", self.ros, self.state
            )
            names = {
                self.device.SpeakerPlugin(CONFIG, "robot", self.ros).get_tool()["name"],
                self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state).get_tool()["name"],
                gait.get_tool()["name"],
                recorder.get_tool()["name"],
                self.device.HeadActuatorPlugin(
                    CONFIG, joint_plan, self.state
                ).get_tool()["name"],
            }
        self.assertEqual(
            {"speaker", "loco", "gait", "motion_recorder", "head"}, names
        )

    def test_sensor_lifecycle_schemas_and_info_topics_match_agent_core(self):
        plugins = [
            self.state,
            self.device.HeartbeatStatusPlugin(CONFIG, "robot", self.ros),
            self.device.MotionCommandTracePlugin(CONFIG, "robot", self.ros),
            self.device.MotionEventsPlugin(CONFIG, "robot", self.ros),
            self.device.NativeInterfaceProbePlugin(CONFIG, "robot", self.ros),
            self.device.VisionPlugin(CONFIG, "robot", self.ros),
            self.device.JointPlanPlugin(CONFIG, "robot", self.ros),
        ]
        for plugin in plugins:
            tools = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            for tool in tools:
                if tool["type"] != "sensor":
                    continue
                action = tool["inputSchema"]["properties"]["action"]
                self.assertTrue({"start", "info", "stop"}.issubset(action["enum"]), tool["name"])
                info = plugin.dispatch("info", {"_tool_name": tool["name"]})
                self.assertTrue(info.get("topic_out"), tool["name"])

    def test_driver_health_is_one_shot_actuator_without_topic_stream(self):
        tool = {item["name"]: item for item in self.state.get_tools()}["driver_health"]
        self.assertEqual("actuator", tool["type"])
        self.assertNotIn("topic_out", tool)
        self.assertEqual(
            ["status"], tool["inputSchema"]["properties"]["action"]["enum"]
        )
        self.assertEqual(["action"], tool["inputSchema"]["required"])
        self.assertNotIn("driver_health", self.state._publishers)

        direct = self.state.dispatch("driver_health", {})
        queried = self.state.dispatch("status", {"_tool_name": "driver_health"})
        self.assertEqual("waiting", direct["state"])
        self.assertEqual("0/9 路数据流正常", direct["health_summary"])
        self.assertEqual(direct["total_sources"], queried["total_sources"])

    def test_new_status_plugins_can_start_with_declared_ros_dependencies(self):
        plugins = [
            self.device.HeartbeatStatusPlugin(CONFIG, "robot", self.ros),
            self.device.MotionCommandTracePlugin(CONFIG, "robot", self.ros),
            self.device.MotionEventsPlugin(CONFIG, "robot", self.ros),
            self.device.NativeInterfaceProbePlugin(CONFIG, "robot", self.ros),
        ]
        for plugin in plugins:
            plugin.start()
            plugin.stop()

    def test_motion_trace_prefers_fresh_command_over_stale_odometry(self):
        plugin = self.device.MotionCommandTracePlugin(CONFIG, "robot", self.ros)
        plugin._on_odometry(types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="odom"),
            child_frame_id="base",
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.1, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )),
        ))
        plugin._odometry_updated = time.monotonic() - 5.0
        plugin._on_velocity(types.SimpleNamespace(linear_velocity=[0.8, 0.0, 0.0], yaw_velocity=0.0))
        snapshot = plugin.dispatch("status", {})
        self.assertEqual("body_velocity_command", snapshot["source"])
        self.assertEqual("0.80 m/s", snapshot["speed"])

    def test_motion_trace_rejects_implausible_odin_odometry(self):
        plugin = self.device.MotionCommandTracePlugin(CONFIG, "robot", self.ros)
        plugin._on_odometry(types.SimpleNamespace(
            header=types.SimpleNamespace(frame_id="device0/odom"),
            child_frame_id="device0/base_link",
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=-10057.9, y=-27579.0, z=-17383.9),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )),
        ))
        plugin._on_gamepad(types.SimpleNamespace(
            hardware_connected=True,
            digital_states=[],
            analog_states=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ))
        snapshot = plugin.dispatch("status", {})
        self.assertEqual("gamepad_analog", snapshot["source"])
        self.assertEqual("0.00 m/s", snapshot["speed"])
        self.assertEqual("stopped", snapshot["motion_state"])
        self.assertEqual("none", snapshot["direction"])

    def test_motion_trace_recognizes_gamepad_analog_speed(self):
        plugin = self.device.MotionCommandTracePlugin(CONFIG, "robot", self.ros)
        plugin._on_gamepad(types.SimpleNamespace(
            hardware_connected=False,
            digital_states=[],
            analog_states=[0.0, 0.0, -0.4, 0.3, 0.0, 0.0],
        ))
        snapshot = plugin.dispatch("status", {})
        self.assertEqual("gamepad_analog", snapshot["source"])
        self.assertEqual("moving", snapshot["motion_state"])
        self.assertEqual("forward_left", snapshot["direction"])
        self.assertEqual("0.50 m/s", snapshot["speed"])

    def test_motion_events_ignore_implausible_odin_odometry(self):
        plugin = self.device.MotionEventsPlugin(CONFIG, "robot", self.ros)
        plugin._on_odometry(types.SimpleNamespace(
            twist=types.SimpleNamespace(twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=-10057.9, y=-27579.0, z=-17383.9),
            )),
        ))
        snapshot = plugin.dispatch("status", {})
        self.assertEqual("no_data", snapshot["state"])

    def test_motion_events_detect_gamepad_motion_transitions(self):
        plugin = self.device.MotionEventsPlugin(CONFIG, "robot", self.ros)
        plugin._on_gamepad(types.SimpleNamespace(
            hardware_connected=False,
            digital_states=[],
            analog_states=[0.0, 0.0, 0.0, 0.5, 0.0, 0.0],
        ))
        moving = plugin.dispatch("status", {})
        self.assertEqual("running", moving["state"])
        self.assertEqual("moving", moving["motion_state"])
        self.assertEqual("gamepad", moving["speed_source"])
        self.assertEqual("gamepad_analog", moving["control_source"])
        self.assertEqual("move", moving["action"])
        self.assertEqual("forward", moving["direction"])
        self.assertEqual([], moving["buttons"])
        self.assertEqual("0.50 m/s", moving["speed"])
        self.assertEqual("motion_start", moving["event"])

        plugin._on_gamepad(types.SimpleNamespace(
            hardware_connected=False,
            digital_states=[],
            analog_states=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ))
        stopped = plugin.dispatch("status", {})
        self.assertEqual("stopped", stopped["motion_state"])
        self.assertEqual("0.00 m/s", stopped["speed"])
        self.assertEqual("motion_stop", stopped["event"])

    def test_motion_events_identify_gamepad_macro_and_motion_state(self):
        plugin = self.device.MotionEventsPlugin(CONFIG, "robot", self.ros)
        digital = [0] * 12
        digital[0] = 1  # LB
        digital[2] = 1  # A
        plugin._on_gamepad(types.SimpleNamespace(
            hardware_connected=True,
            digital_states=digital,
            analog_states=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ))
        snapshot = plugin.dispatch("status", {})
        self.assertEqual("stand", snapshot["action"])
        self.assertEqual("gamepad_analog", snapshot["control_source"])
        self.assertEqual("none", snapshot["direction"])
        self.assertEqual(["LB", "A"], snapshot["buttons"])
        self.assertEqual("gamepad_action", snapshot["event"])

        plugin._on_motion_state(types.SimpleNamespace(current_motion_task="pd_stand"))
        snapshot = plugin.dispatch("status", {})
        self.assertEqual("pd_stand", snapshot["current_motion_state"])
        self.assertEqual("stand", snapshot["action"])
        self.assertEqual("motion_state", snapshot["control_source"])
        self.assertEqual("motion_state_changed", snapshot["event"])

    def test_motion_events_normalize_screen_motion_states(self):
        plugin = self.device.MotionEventsPlugin(CONFIG, "robot", self.ros)
        for raw, action in (
            ("sit_down", "sit"),
            ("boxing_combo", "punch"),
            ("screen_punch", "punch"),
        ):
            with self.subTest(raw=raw):
                plugin._on_motion_state(types.SimpleNamespace(current_motion_task=raw))
                snapshot = plugin.dispatch("status", {})
                self.assertEqual(raw, snapshot["current_motion_state"])
                self.assertEqual(action, snapshot["action"])
                self.assertEqual("motion_state", snapshot["control_source"])
                self.assertEqual("motion_state_changed", snapshot["event"])

    def test_motion_events_keep_screen_state_visible_after_idle_gamepad(self):
        plugin = self.device.MotionEventsPlugin(CONFIG, "robot", self.ros)
        plugin._on_motion_state(types.SimpleNamespace(current_motion_task="sit_down"))
        plugin._on_gamepad(types.SimpleNamespace(
            hardware_connected=True,
            digital_states=[0] * 12,
            analog_states=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ))
        snapshot = plugin.dispatch("status", {})
        self.assertEqual("sit", snapshot["action"])
        self.assertEqual("motion_state", snapshot["control_source"])
        self.assertEqual("sit_down", snapshot["current_motion_state"])

    def test_motion_events_accept_ros_array_like_gamepad_states(self):
        class RosArrayLike:
            def __init__(self, values):
                self._values = list(values)

            def __iter__(self):
                return iter(self._values)

            def __bool__(self):
                raise ValueError("ambiguous truth value")

        plugin = self.device.MotionEventsPlugin(CONFIG, "robot", self.ros)
        digital = RosArrayLike([1, 0, 1] + [0] * 9)
        plugin._on_gamepad(types.SimpleNamespace(
            hardware_connected=True,
            digital_states=digital,
            analog_states=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        ))
        snapshot = plugin.dispatch("status", {})
        self.assertEqual("stand", snapshot["action"])
        self.assertEqual(["LB", "A"], snapshot["buttons"])

    def test_derived_diagnostics_and_capability_resources(self):
        self.state._set("imu", {
            "rpy_rad": [1.1, 0.0, 0.0],
            "angular_velocity_rad_s": [0.0, 0.0, 0.0],
        })
        self.state._set("motor_health", {
            "offline": [0, 1], "enabled": [1, 0], "error_code": [0, 7],
            "motor_temperature_c": [30.0, 80.0],
        })
        self.state._set("battery", {"error_code": 0})
        self.assertEqual("fall_risk", self.state.dispatch("stability", {})["state"])
        faults = self.state.dispatch("fault_summary", {})
        self.assertEqual([1], faults["offline_joints"])
        self.assertEqual(7, faults["motor_errors"][0]["code"])
        self.assertEqual(25, self.state.dispatch("capabilities", {})["dof"])
        self.assertEqual([23, 24], [item["index"] for item in
                                   self.state.dispatch("joint_groups", {})["groups"]["head"]])

    def test_driver_health_aggregates_audio_and_odin2_stream_freshness(self):
        mic = self.device.MicPlugin(CONFIG, "robot", self.ros)
        vision = self.device.VisionPlugin(CONFIG, "robot", self.ros)
        now = time.monotonic()
        mic._running = True
        mic._last_chunk_at = now
        mic._samples_published = 512
        mic._chunks_published = 1
        vision._running = True
        vision._enabled_tools.update(vision._TOOL_NAMES)
        vision._updated = {
            "pointcloud": now,
            "camera_left": now,
            "camera_right": now,
            "depth": now,
        }
        vision._frames = {
            "pointcloud": 3,
            "camera_left": 4,
            "camera_right": 4,
            "depth": 2,
        }
        self.state.register_health_provider("mic", mic.health_sources)
        self.state.register_health_provider("vision", vision.health_sources)
        for name in self.state._STREAMS:
            if name != "driver_health":
                self.state._set(name, {})

        health = self.state.dispatch("driver_health", {})
        self.assertEqual("running", health["state"])
        self.assertEqual((14, 14, 14), (
            health["connected_sources"], health["fresh_sources"], health["total_sources"],
        ))
        self.assertEqual("running", health["robot_state"])
        self.assertEqual("running", health["audio_state"])
        self.assertEqual("running", health["odin2_state"])
        self.assertEqual("14/14 路数据流正常", health["health_summary"])
        self.assertEqual("running · 9/9 路正常", health["robot_summary"])
        self.assertEqual("running · 1/1 路正常", health["audio_summary"])
        self.assertEqual("running · 4/4 路正常", health["odin2_summary"])
        self.assertEqual("running", health["microphone_state"])
        self.assertEqual("running", health["odin2_camera_right_state"])
        self.assertEqual([], health["issues"])
        self.assertEqual(512, health["sources"]["microphone"]["samples_published"])
        self.assertEqual(3, health["sources"]["odin2_pointcloud"]["frames"])

        vision._updated["camera_right"] = time.monotonic() - 2.0
        degraded = self.state.dispatch("driver_health", {})
        self.assertEqual("degraded", degraded["state"])
        self.assertEqual("degraded", degraded["odin2_state"])
        self.assertEqual("stale", degraded["odin2_camera_right_state"])
        self.assertEqual("degraded · 3/4 路正常", degraded["odin2_summary"])
        self.assertEqual("stale", degraded["sources"]["odin2_camera_right"]["state"])
        self.assertIn("Odin2 右相机: stale", degraded["issues"])

    def test_locomotion_allowed_state_publishes_and_stops(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        result = plugin.dispatch("move", {"vx": 0.9, "vy": -0.9, "vyaw": 0.9, "duration": 0.03})
        self.assertEqual(0.9, result["vx"])
        self.assertEqual(-0.9, result["vy"])
        time.sleep(0.08)
        self.assertGreaterEqual(len(plugin._publisher.messages), 2)
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_rejects_direct_velocity_outside_safety_limits(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        plugin.dispatch("move", {"vx": 0.1, "duration": -1})

        rejected = plugin.dispatch("move", {
            "vx": 100.0,
            "vy": 0.0,
            "vyaw": 100.0,
            "duration": 0.1,
        })

        self.assertEqual("SAFETY_LIMIT", rejected["code"])
        self.assertIn("vx", rejected["error"])
        self.assertIn("vyaw", rejected["error"])
        self.assertFalse(plugin._stream.snapshot().active)
        self.assertEqual([0.0, 0.0], plugin._publisher.messages[-1].linear_velocity)
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_treats_optional_nulls_as_omitted_defaults(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))

        result = plugin.dispatch("move", {
            "vx": 0.1,
            "vy": None,
            "vyaw": None,
            "duration": None,
        })

        self.assertEqual("running", result["state"])
        self.assertEqual(0.1, result["vx"])
        self.assertEqual(0.0, result["vy"])
        self.assertEqual(0.0, result["vyaw"])
        self.assertEqual(1.0, result["duration"])
        plugin.dispatch("stop_move", {})
        properties = plugin.get_tool()["inputSchema"]["properties"]
        self.assertEqual(0.0, properties["vy"]["default"])
        self.assertEqual(0.0, properties["vyaw"]["default"])
        self.assertEqual(1.0, properties["duration"]["default"])

    def test_locomotion_rejects_invalid_and_composite_safety_limits(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        cases = [
            ("move", {"vx": True, "duration": 0.1}, "INVALID_ARGUMENT"),
            ("move", {"vx": "0.1", "duration": 0.1}, "INVALID_ARGUMENT"),
            ("move", {"vx": 10**400, "duration": 0.1}, "INVALID_ARGUMENT"),
            ("move", {"vx": float("nan"), "duration": 0.1}, "INVALID_ARGUMENT"),
            ("move", {"vx": 0.1, "duration": -0.5}, "INVALID_ARGUMENT"),
            ("move", {"vx": 0.1, "duration": 3.1}, "SAFETY_LIMIT"),
            ("move_displacement", {"x_m": 0.1, "y_m": 0, "speed_m_s": 10}, "SAFETY_LIMIT"),
            ("move_displacement", {"x_m": 0.1, "y_m": 0, "speed_m_s": None}, "INVALID_ARGUMENT"),
            ("move_displacement", {"x_m": 0.1, "y_m": 0, "speed_m_s": 0}, "INVALID_ARGUMENT"),
            ("turn_angle", {"angle_rad": 0.1, "angular_speed_rad_s": 10}, "SAFETY_LIMIT"),
            ("turn_angle", {"angle_rad": 0.1, "angular_speed_rad_s": 0}, "INVALID_ARGUMENT"),
            ("arc", {"radius_m": 1, "angle_rad": 0.1, "linear_speed_m_s": 10}, "SAFETY_LIMIT"),
            ("arc", {"radius_m": 0.1, "angle_rad": 0.1, "linear_speed_m_s": 0.2}, "SAFETY_LIMIT"),
        ]
        for action, arguments, expected_code in cases:
            with self.subTest(action=action, arguments=arguments):
                plugin.dispatch("move", {"vx": 0.1, "duration": -1})
                rejected = plugin.dispatch(action, arguments)
                self.assertEqual(expected_code, rejected["code"])
                self.assertIn("error", rejected)
                self.assertFalse(plugin._stream.snapshot().active)
                self.assertEqual([0.0, 0.0], plugin._publisher.messages[-1].linear_velocity)
                self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

        duration_schema = plugin.get_tool()["inputSchema"]["properties"]["duration"]
        self.assertEqual(
            [
                {"type": "number", "const": -1},
                {"type": "number", "minimum": 0, "maximum": 3.0},
            ],
            duration_schema["anyOf"],
        )

    def test_locomotion_accepts_official_walk_states_without_force(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        schema = plugin.get_tool()["inputSchema"]
        self.assertNotIn("x-completion", schema)
        self.assertNotIn("force", schema["properties"])
        for motion in ("rl_basic", "walk", "lower_body_balance"):
            with self.subTest(motion=motion):
                self.state._on_motion(types.SimpleNamespace(
                    current_motion_task=motion,
                    available_transition_motions=["passive"],
                ))
                result = plugin.dispatch("move", {
                    "vx": 0.2,
                    "duration": 0.01,
                })
                self.assertEqual("running", result["state"])
                self.assertEqual(0.2, result["vx"])
                plugin.dispatch("stop_move", {})

        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="idle",
            available_transition_motions=["passive"],
        ))
        rejected = plugin.dispatch("move", {"vx": 0.1, "duration": 0.01})
        self.assertIn("rl_basic", rejected["error"])

    def test_locomotion_continuous_move_does_not_block_manual_stop(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))

        result = plugin.dispatch("move", {"vx": 0.1, "duration": -1})

        self.assertNotIn("action_id", result)
        self.assertEqual("stopped", plugin.dispatch("stop_move", {})["state"])

    def test_locomotion_timed_move_stops_and_zeroes_velocity(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))

        result = plugin.dispatch("move", {"vx": 0.1, "duration": 0.03})

        self.assertNotIn("action_id", result)
        time.sleep(0.08)
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_manual_stop_zeroes_active_timed_action(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        plugin.dispatch("move", {"vx": 0.1, "duration": 0.5})

        stopped = plugin.dispatch("stop_move", {})

        self.assertEqual("stopped", stopped["state"])
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_watchdog_zeroes_velocity(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        plugin.dispatch("move", {"vx": 0.1, "duration": 0.5})
        plugin._stream._last_publish_at = time.monotonic() - 2.0

        plugin._stream_health_check()

        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_background_publish_failure_is_visible_and_zeroes(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        real_publish = plugin._stream._publisher
        publish_count = 0

        def fail_after_first(payload):
            nonlocal publish_count
            publish_count += 1
            if publish_count > 1:
                raise RuntimeError("velocity publisher failed")
            real_publish(payload)

        plugin._stream._publisher = fail_after_first
        plugin.dispatch("move", {"vx": 0.1, "duration": 0.5})

        deadline = time.monotonic() + 0.5
        while plugin._stream.snapshot().active and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual("velocity publisher failed", plugin._stream.snapshot().error)
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_rejected_command_stops_existing_stream(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        plugin.dispatch("move", {"vx": 0.1, "duration": -1})

        rejected = plugin.dispatch("move_displacement", {"x_m": 0, "y_m": 0})

        self.assertIn("non-zero", rejected["error"])
        self.assertFalse(plugin._stream.snapshot().active)
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_motion_state_transition_stops_active_stream(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        plugin.dispatch("move", {"vx": 0.1, "duration": -1})
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="passive",
            available_transition_motions=["pd_stand"],
        ))

        plugin._stream_health_check()

        self.assertFalse(plugin._stream.snapshot().active)
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_force_cannot_bypass_motion_state_gate(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="passive",
            available_transition_motions=["pd_stand"],
        ))

        rejected = plugin.dispatch("move", {
            "vx": 0.1, "duration": 0.1, "force": True,
        })

        self.assertIn("rl_basic", rejected["error"])
        self.assertFalse(plugin._stream.snapshot().active)
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_rejects_timed_commands_beyond_safety_limit(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        rejected = plugin.dispatch("move", {
            "vx": 0.1,
            "duration": plugin._MAX_TIMED_DURATION_SEC + 0.1,
        })
        self.assertEqual("SAFETY_LIMIT", rejected["code"])
        self.assertIn("3s safety limit", rejected["error"])

    def test_locomotion_open_loop_composites(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        self.state._on_motion(types.SimpleNamespace(
            current_motion_task="rl_basic",
            available_transition_motions=["passive"],
        ))
        move = plugin.dispatch("move_displacement", {
            "x_m": 1.0, "y_m": 0.0, "speed_m_s": 0.5,
        })
        self.assertTrue(move["open_loop"])
        self.assertAlmostEqual(0.5, move["vx"])
        self.assertAlmostEqual(2.0, move["duration"])
        plugin.dispatch("stop_move", {})
        turn = plugin.dispatch("turn_angle", {
            "angle_rad": -1.0, "angular_speed_rad_s": 0.5,
        })
        self.assertAlmostEqual(-0.5, turn["vyaw"])
        plugin.dispatch("stop_move", {})
        arc = plugin.dispatch("arc", {
            "radius_m": 1.0, "angle_rad": 1.0, "linear_speed_m_s": 0.5,
        })
        self.assertAlmostEqual(arc["vx"], arc["vyaw"])
        plugin.dispatch("stop_move", {})

    def test_motion_mode_force_path_publishes_custom_state(self):
        plugin = self.device.MotionModePlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        result = plugin.dispatch("switch", {"target": "vendor_future_mode", "force": True, "wait": False})
        self.assertEqual("requested", result["state"])
        self.assertEqual("vendor_future_mode", plugin._publisher.messages[-1].target_motion_name)
        plugin.dispatch("get_up", {"force": True, "wait": False})
        self.assertEqual("rl_mimic_supine_to_stance", plugin._publisher.messages[-1].target_motion_name)

    def test_dance_facade_lists_and_plays_official_dance(self):
        mode = self.device.MotionModePlugin(CONFIG, "robot", self.ros, self.state)
        mode.start()
        dance = self.device.DancePlugin(mode, self.state)
        self.assertEqual("dance.mnn", dance.dispatch("list", {})["built_in"][0]["policy"])
        result = dance.dispatch("play", {"name": "dance", "force": True, "wait": False})
        self.assertEqual("requested", result["state"])
        self.assertEqual("dance", mode._publisher.messages[-1].target_motion_name)

    def test_joint_plan_accepts_arbitrary_valid_joint_set(self):
        plugin = self.device.JointPlanPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        result = plugin.dispatch("plan", {"joint_indices": [12, 23], "target_positions": [0.1, -0.2],
                                           "duration": 1.0})
        self.assertEqual("requested", result["state"])
        sent = plugin._publisher.messages[-1]
        self.assertEqual([12, 23], sent.joint_indices)
        self.assertEqual([0.1, -0.2], sent.target_positions)

    def test_joint_plan_rejects_urdf_limit_violations_on_every_plan_facade(self):
        plugin = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        unsafe_calls = [
            ("plan", {"joint_indices": [16], "target_positions": [-3.0]}),
            ("plan_named", {
                "joint_names": ["J16_ELBOW_PITCH_L"],
                "target_positions": [-3.0],
            }),
            ("head_pose", {"pitch_rad": 1.0, "yaw_rad": 0.0}),
            ("arm_pose", {
                "side": "left",
                "target_positions": [0.0, 0.0, 0.0, -3.0, 0.0],
            }),
        ]
        for action, arguments in unsafe_calls:
            with self.subTest(action=action):
                with self.assertRaisesRegex(ValueError, "safe position limit"):
                    plugin.dispatch(action, arguments)

        self.state._last_joint_positions[16] = -3.0
        with self.assertRaisesRegex(ValueError, "safe position limit"):
            plugin.dispatch("hold_current", {})
        self.assertEqual([], plugin._publisher.messages)

    def test_joint_plan_validation_failure_does_not_leak_head_ownership(self):
        plugin = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        # head joints [23, 24] 但 target_positions 只给 1 个，验证失败
        with self.assertRaises(ValueError):
            plugin.dispatch("plan", {
                "joint_indices": [23, 24],
                "target_positions": [0.0],
            })
        # 验证失败后 head 不应被占有，head_pose 应能正常执行
        result = plugin.dispatch("head_pose", {"pitch_rad": 0.1, "yaw_rad": 0.0})
        self.assertEqual("requested", result["state"])
        self.assertEqual([23, 24], plugin._publisher.messages[-1].joint_indices)

    def test_joint_plan_named_head_arm_and_hold_actions(self):
        plugin = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        named = plugin.dispatch("plan_named", {
            "joint_names": ["J23_HEAD_PITCH", "J24_HEAD_YAW"],
            "target_positions": [0.1, -0.2], "duration": 1.0,
        })
        self.assertEqual("requested", named["state"])
        self.assertEqual([23, 24], plugin._publisher.messages[-1].joint_indices)
        # head 请求持有锁，模拟完成后释放
        req_id = plugin._publisher.messages[-1].request_id
        plugin._on_state(JointMotionPlanState(req_id, JointMotionPlanState.EXECUTING, 0.5))
        plugin._on_state(JointMotionPlanState(req_id, JointMotionPlanState.IDLE, 1.0))

        plugin.dispatch("head_pose", {"pitch_rad": 0.2, "yaw_rad": 0.3})
        self.assertEqual([23, 24], plugin._publisher.messages[-1].joint_indices)
        req_id = plugin._publisher.messages[-1].request_id
        plugin._on_state(JointMotionPlanState(req_id, JointMotionPlanState.EXECUTING, 0.5))
        plugin._on_state(JointMotionPlanState(req_id, JointMotionPlanState.IDLE, 1.0))

        plugin.dispatch("arm_pose", {"side": "left", "target_positions": [0.0] * 5})
        self.assertEqual([13, 14, 15, 16, 17], plugin._publisher.messages[-1].joint_indices)
        plugin.dispatch("hold_current", {})
        self.assertEqual(list(range(25)), plugin._publisher.messages[-1].joint_indices)

    def test_gesture_exposes_complete_official_sequences_and_custom_queue(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_args, **_kwargs: {}
        plan.wait_for_request = lambda *_args, **_kwargs: {}
        gesture = self.device.GesturePlugin(plan)
        self.device._notify_acp_completion = lambda *_args, **_kwargs: None
        listed = {item["name"]: item["steps"] for item in gesture.dispatch("list", {})["gestures"]}
        self.assertEqual(8, listed["wave_hands"])
        self.assertEqual(2, listed["shake_hand"])
        result = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23, 24], "target_positions": [0.1, -0.1], "duration": 0.05}],
            "reset_after": False,
            "force": True,
        })
        self.assertEqual("running", result["state"])
        gesture._thread.join(timeout=1.0)
        self.assertEqual("completed", gesture.dispatch("status", {})["state"])
        self.assertEqual([23, 24], plan._publisher.messages[-1].joint_indices)

    def test_gesture_declares_async_completion_and_lifecycle_actions(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        gesture = self.device.GesturePlugin(plan)
        schema = gesture.get_tool()["inputSchema"]
        self.assertEqual(["play", "sequence"], schema["x-completion"]["actions"])
        self.assertGreaterEqual(schema["x-completion"]["timeout"], 60)
        self.assertEqual(
            {"on_interrupt_motion": {"action": "stop_gesture"}},
            schema["x-hooks"],
        )
        actions = set(schema["properties"]["action"]["enum"])
        self.assertTrue({"start", "info", "stop"}.issubset(actions))
        action_params = schema["x-action-params"]
        self.assertEqual(
            ["name", "repetitions", "reset_after", "force"],
            action_params["play"]["params"],
        )
        self.assertEqual(
            ["steps", "reset_after", "force"],
            action_params["sequence"]["params"],
        )
        self.assertEqual(["reset_after"], action_params["stop"]["params"])
        self.assertEqual(["reset_after"], action_params["stop_gesture"]["params"])
        self.assertNotIn("wait", schema["properties"])
        self.assertEqual(gesture._MAX_SEQUENCE_STEPS, schema["properties"]["steps"]["maxItems"])
        rejected = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23], "target_positions": [0.0]}],
            "wait": True,
            "force": True,
        })
        self.assertIn("wait=true is not supported", rejected["error"])
        self.assertNotIn("action_id", rejected)
        self.assertIsNone(gesture._thread)

    def test_gesture_async_acp_uses_real_planner_state_transitions(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        gesture = self.device.GesturePlugin(plan)
        completions = []
        self.device._notify_acp_completion = lambda tool, action_id, status, result, timeout: completions.append(
            (action_id, status, result)
        )

        result = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23], "target_positions": [0.1], "duration": 0.05}],
            "reset_after": False,
            "force": True,
        })
        self.assertEqual("running", result["state"])
        self.assertTrue(result["action_id"].startswith("t800_gesture_"))

        deadline = time.monotonic() + 1.0
        while not plan._publisher.messages and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(plan._publisher.messages)
        request_id = plan._publisher.messages[-1].request_id
        plan._on_state(JointMotionPlanState(request_id, JointMotionPlanState.EXECUTING, 0.5))
        plan._on_state(JointMotionPlanState(request_id, JointMotionPlanState.IDLE, 1.0))
        gesture._thread.join(timeout=1.0)

        self.assertFalse(gesture._thread.is_alive())
        self.assertEqual("completed", gesture.dispatch("status", {})["state"])
        self.assertEqual(result["action_id"], completions[0][0])
        self.assertEqual("completed", completions[0][1])
        self.assertEqual(request_id, completions[0][2]["request_id"])

    def test_gesture_reset_after_completes_successfully(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        gesture = self.device.GesturePlugin(plan)
        completions = []
        self.device._notify_acp_completion = lambda tool, action_id, status, result, timeout: completions.append(
            (action_id, status, result)
        )
        result = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23], "target_positions": [0.1], "duration": 0.05}],
            "reset_after": True,
            "force": True,
        })

        deadline = time.monotonic() + 1.0
        while not plan._publisher.messages and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertTrue(plan._publisher.messages)
        step_request_id = plan._publisher.messages[-1].request_id
        plan._on_state(JointMotionPlanState(
            step_request_id, JointMotionPlanState.EXECUTING, 0.5
        ))
        plan._on_state(JointMotionPlanState(
            step_request_id, JointMotionPlanState.IDLE, 1.0
        ))

        deadline = time.monotonic() + 1.0
        while len(plan._publisher.messages) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        self.assertGreaterEqual(len(plan._publisher.messages), 2)
        reset_request = plan._publisher.messages[-1]
        self.assertEqual(JointMotionPlanRequest.REQUEST_RESET, reset_request.request_type)
        plan._on_state(JointMotionPlanState(
            reset_request.request_id, JointMotionPlanState.EXECUTING, 0.5
        ))
        plan._on_state(JointMotionPlanState(
            reset_request.request_id, JointMotionPlanState.IDLE, 1.0
        ))
        gesture._thread.join(timeout=1.0)

        status = gesture.dispatch("status", {})
        self.assertEqual("completed", status["state"])
        self.assertIsNone(status["error"])
        self.assertEqual(reset_request.request_id, status["request_id"])
        self.assertEqual(result["action_id"], completions[0][0])
        self.assertEqual("completed", completions[0][1])
        self.assertEqual(reset_request.request_id, completions[0][2]["request_id"])

    def test_gesture_completion_releases_head_before_acp_callback(self):
        """ACP 回调触发时 head 所有权必须已释放，否则连续动作会返回 head is busy。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        gesture = self.device.GesturePlugin(plan)
        head_owner_during_callback = []

        def notify(tool, action_id, status, result, timeout):
            # 模拟 Agent Core 收到回调后立即尝试获取 head
            head_owner_during_callback.append(plan.head_status().get("owner"))

        self.device._notify_acp_completion = notify
        gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23], "target_positions": [0.1], "duration": 0.05}],
            "reset_after": True,
            "force": True,
        })

        deadline = time.monotonic() + 1.0
        while not plan._publisher.messages and time.monotonic() < deadline:
            time.sleep(0.005)
        step_request_id = plan._publisher.messages[-1].request_id
        plan._on_state(JointMotionPlanState(step_request_id, JointMotionPlanState.EXECUTING, 0.5))
        plan._on_state(JointMotionPlanState(step_request_id, JointMotionPlanState.IDLE, 1.0))

        deadline = time.monotonic() + 1.0
        while len(plan._publisher.messages) < 2 and time.monotonic() < deadline:
            time.sleep(0.005)
        reset_request = plan._publisher.messages[-1]
        plan._on_state(JointMotionPlanState(reset_request.request_id, JointMotionPlanState.EXECUTING, 0.5))
        plan._on_state(JointMotionPlanState(reset_request.request_id, JointMotionPlanState.IDLE, 1.0))
        gesture._thread.join(timeout=1.0)

        # 回调触发时 head 所有权应为 None（已释放），而非 "gesture"
        self.assertEqual(1, len(head_owner_during_callback))
        self.assertIsNone(head_owner_during_callback[0])

    def test_gesture_rejects_sequence_beyond_acp_runtime_budget(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        gesture = self.device.GesturePlugin(plan)
        long_step = {
            "joint_indices": [23],
            "target_positions": [0.1],
            "duration": 120.0,
            "hold_after_sec": 30.0,
        }
        over_budget = gesture.dispatch("sequence", {
            "steps": [long_step, long_step],
            "reset_after": False,
            "force": True,
        })
        self.assertIn("exceeds ACP completion timeout", over_budget["error"])
        self.assertNotIn("action_id", over_budget)
        self.assertIsNone(gesture._thread)

        too_many = gesture.dispatch("sequence", {
            "steps": [long_step] * (gesture._MAX_SEQUENCE_STEPS + 1),
            "reset_after": False,
            "force": True,
        })
        self.assertIn("cannot contain more than", too_many["error"])
        self.assertIsNone(gesture._thread)

    def test_gesture_stop_preserves_cancelled_and_reports_acp(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        entered_wait = threading.Event()
        release_wait = threading.Event()
        plan.wait_until_idle = lambda *_args, **_kwargs: {}

        def wait_for_request(*_args, **_kwargs):
            entered_wait.set()
            release_wait.wait(timeout=1.0)
            return {}

        plan.wait_for_request = wait_for_request
        gesture = self.device.GesturePlugin(plan)
        completions = []
        self.device._notify_acp_completion = lambda tool, action_id, status, result, timeout: completions.append(
            (action_id, status, result)
        )
        result = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23], "target_positions": [0.1], "duration": 0.05}],
            "reset_after": False,
            "force": True,
        })
        self.assertTrue(entered_wait.wait(timeout=1.0))
        busy = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23], "target_positions": [0.0]}],
            "force": True,
        })
        self.assertIn("already running", busy["error"])
        self.assertNotIn("action_id", busy)
        stop_started = time.monotonic()
        stopped = gesture.dispatch("stop_gesture", {})
        stop_elapsed = time.monotonic() - stop_started
        self.assertEqual("cancelled", stopped["state"])
        self.assertLess(stop_elapsed, 0.02)
        self.assertEqual(
            JointMotionPlanRequest.REQUEST_CANCEL,
            plan._publisher.messages[-1].request_type,
        )
        release_wait.set()
        gesture._thread.join(timeout=1.0)

        self.assertEqual("cancelled", gesture.dispatch("status", {})["state"])
        self.assertEqual(result["action_id"], completions[0][0])
        self.assertEqual("cancelled", completions[0][1])
        self.assertEqual({"state": "idle"}, gesture.dispatch("stop", {}))
        message_count = len(plan._publisher.messages)
        ignored_reset = gesture.dispatch("stop", {"reset_after": True})
        self.assertTrue(ignored_reset["reset_after_ignored"])
        self.assertEqual(message_count, len(plan._publisher.messages))

    def test_gesture_cancel_transport_failure_stays_gated_until_retry(self):
        class FlakyJointPlan:
            def __init__(self):
                self.fail_cancel = True
                self.cancel_attempts = 0
                self.entered_wait = threading.Event()
                self.release_wait = threading.Event()
                self.status = {"request_id": 41, "status": 2, "progress": 0.4}

            def current_motion(self):
                return "lower_body_balance", []

            def wait_until_idle(self, *_args, **_kwargs):
                return {}

            def wait_for_request(self, *_args, **_kwargs):
                self.entered_wait.set()
                self.release_wait.wait(timeout=1.0)
                return {}

            def acquire_head(self, _owner):
                return None

            def release_head(self, _owner):
                pass

            def _dispatch_owned(self, _owner, action, args):
                return self.dispatch(action, args)

            def dispatch(self, action, _args):
                if action in ("plan", "plan_named"):
                    return {"state": "requested", "request_id": 41}
                if action == "cancel":
                    self.cancel_attempts += 1
                    if self.fail_cancel:
                        raise RuntimeError("gesture cancel publish failed")
                    return {"state": "requested", "request_id": 41}
                if action == "status":
                    return dict(self.status)
                return {"error": f"unexpected action: {action}"}

        plan = FlakyJointPlan()
        gesture = self.device.GesturePlugin(plan)
        gesture._acp_notify = lambda *_args: None
        interrupt_group = self.device.MotionInterruptGroup()
        interrupt_group.register("gesture", gesture.halt, gesture.motion_active)
        gesture.set_interrupt_group(interrupt_group)
        gesture.dispatch("sequence", {
            "steps": [{
                "joint_indices": [23],
                "target_positions": [0.1],
                "duration": 0.05,
            }],
            "reset_after": False,
            "force": True,
        })
        self.assertTrue(plan.entered_wait.wait(timeout=1.0))

        stopped = gesture.dispatch("stop_gesture", {})
        self.assertIn("gesture cancel publish failed", str(stopped))
        plan.release_wait.set()
        gesture._thread.join(timeout=1.0)
        self.assertTrue(gesture.dispatch("status", {})["cancel_failed"])
        self.assertEqual(["gesture"], interrupt_group.blocking_outputs())

        plan.fail_cancel = False
        retried = gesture.dispatch("stop_gesture", {})
        self.assertEqual("", retried["error"])
        self.assertNotIn("error", retried["cancel_result"])
        self.assertEqual(2, plan.cancel_attempts)
        self.assertTrue(gesture.dispatch("status", {})["cancel_pending"])
        plan.status = {"request_id": 41, "status": 1, "progress": 1.0}
        deadline = time.monotonic() + 1.0
        while gesture.dispatch("status", {})["cancel_pending"] and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertFalse(gesture.dispatch("status", {})["cancel_failed"])
        self.assertFalse(gesture.dispatch("status", {})["cancel_pending"])
        self.assertEqual([], interrupt_group.blocking_outputs())

    def test_gesture_stop_returns_promptly_but_retains_head_until_worker_exit(self):
        """stop 立即返回，但 worker 退出前不能释放 head 锁。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_a, **_kw: {}

        dispatch_entered = threading.Event()
        release_dispatch = threading.Event()
        real_dispatch = plan._dispatch_owned

        def tracked_dispatch(owner, action, args):
            if action == "plan":
                # worker 已通过取消检查、正卡在 _dispatch_owned 前
                dispatch_entered.set()
                release_dispatch.wait(timeout=2.0)
            return real_dispatch(owner, action, args)

        plan._dispatch_owned = tracked_dispatch

        gesture = self.device.GesturePlugin(plan)
        result = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23], "target_positions": [0.1], "duration": 0.05}],
            "reset_after": False,
            "force": True,
        })
        self.assertTrue(dispatch_entered.wait(timeout=1.0))
        self.assertEqual("gesture", plan.head_status()["owner"])

        stop_done = threading.Event()

        def do_stop():
            gesture.dispatch("stop_gesture", {})
            stop_done.set()

        stop_thread = threading.Thread(target=do_stop)
        stop_thread.start()
        # ACP interrupt hook must return promptly, without releasing the lease.
        self.assertTrue(stop_done.wait(timeout=0.2))
        self.assertEqual("gesture", plan.head_status()["owner"])

        release_dispatch.set()
        stop_thread.join(timeout=1.0)
        gesture._thread.join(timeout=1.0)

        self.assertFalse(gesture._thread.is_alive())
        self.assertIsNone(plan.head_status()["owner"])

    def test_gesture_async_error_reports_acp(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_args, **_kwargs: {}
        plan.wait_for_request = lambda *_args, **_kwargs: (_ for _ in ()).throw(
            RuntimeError("planner failed")
        )
        gesture = self.device.GesturePlugin(plan)
        completions = []
        self.device._notify_acp_completion = lambda tool, action_id, status, result, timeout: completions.append(
            (action_id, status, result)
        )
        result = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23], "target_positions": [0.1], "duration": 0.05}],
            "reset_after": False,
            "force": True,
        })
        gesture._thread.join(timeout=1.0)

        self.assertEqual(result["action_id"], completions[0][0])
        self.assertEqual("error", completions[0][1])
        self.assertIn("planner failed", completions[0][2]["error"])

    def test_gesture_acp_notify_posts_completion_payload(self):
        received = []

        class Handler(BaseHTTPRequestHandler):
            def do_POST(self):
                length = int(self.headers.get("Content-Length", "0"))
                received.append((self.path, json.loads(self.rfile.read(length))))
                self.send_response(204)
                self.end_headers()

            def log_message(self, _format, *_args):
                pass

        server = ThreadingHTTPServer(("127.0.0.1", 0), Handler)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        previous = os.environ.get("AGENT_CORE_URL")
        previous_ca = os.environ.pop("AGENT_CORE_CA_CERT", None)
        original_create_default_context = ssl.create_default_context
        contexts = []

        def create_default_context(*args, **kwargs):
            context = original_create_default_context(*args, **kwargs)
            contexts.append(context)
            return context

        ssl.create_default_context = create_default_context
        os.environ["AGENT_CORE_URL"] = f"http://127.0.0.1:{server.server_port}"
        try:
            self.device._notify_acp_completion(
                "gesture", "t800_gesture_test", "completed",
                {"gesture": "wave_hands"},
                self.device.GesturePlugin._ACP_CALLBACK_TIMEOUT_SEC,
            )
        finally:
            if previous is None:
                os.environ.pop("AGENT_CORE_URL", None)
            else:
                os.environ["AGENT_CORE_URL"] = previous
            if previous_ca is not None:
                os.environ["AGENT_CORE_CA_CERT"] = previous_ca
            ssl.create_default_context = original_create_default_context
            server.shutdown()
            server.server_close()

        self.assertEqual("/api/acp/complete", received[0][0])
        payload = received[0][1]
        self.assertEqual("t800_gesture_test", payload["action_id"])
        self.assertEqual("completed", payload["status"])
        self.assertEqual("gesture", payload["tool"])
        self.assertEqual("wave_hands", payload["result"]["gesture"])
        # HTTP loopback does not use TLS, but the shared transport still builds
        # a secure default context rather than weakening verification globally.
        self.assertTrue(contexts[0].check_hostname)
        self.assertEqual(ssl.CERT_REQUIRED, contexts[0].verify_mode)

    def test_motion_recorder_acp_notify_verifies_tls_with_configured_ca(self):
        import urllib.request as urllib_request

        previous_url = os.environ.get("AGENT_CORE_URL")
        previous_ca = os.environ.get("AGENT_CORE_CA_CERT")
        original_create_default_context = ssl.create_default_context
        original_urlopen = urllib_request.urlopen
        contexts = []
        cafiles = []
        requests = []

        def create_default_context(*args, **kwargs):
            cafiles.append(kwargs.get("cafile"))
            context = original_create_default_context()
            contexts.append(context)
            return context

        def urlopen(request, *, timeout, context):
            requests.append((request.full_url, timeout, context))
            return types.SimpleNamespace(close=lambda: None)

        ssl.create_default_context = create_default_context
        urllib_request.urlopen = urlopen
        os.environ["AGENT_CORE_URL"] = "https://phanthy-motus:15678"
        os.environ["AGENT_CORE_CA_CERT"] = "/opt/phanthy-motus/data/certs/cert.pem"
        try:
            self.device._t800_acp_notify(
                "t800_motion_test", "completed", {"frames": 10}, "motion_recorder"
            )
        finally:
            if previous_url is None:
                os.environ.pop("AGENT_CORE_URL", None)
            else:
                os.environ["AGENT_CORE_URL"] = previous_url
            if previous_ca is None:
                os.environ.pop("AGENT_CORE_CA_CERT", None)
            else:
                os.environ["AGENT_CORE_CA_CERT"] = previous_ca
            ssl.create_default_context = original_create_default_context
            urllib_request.urlopen = original_urlopen

        self.assertEqual(
            ["/opt/phanthy-motus/data/certs/cert.pem"],
            cafiles,
        )
        self.assertTrue(contexts[0].check_hostname)
        self.assertEqual(ssl.CERT_REQUIRED, contexts[0].verify_mode)
        self.assertEqual("https://phanthy-motus:15678/api/acp/complete", requests[0][0])

    def test_acp_preflight_exposes_missing_ca_certificate(self):
        previous_url = os.environ.get("AGENT_CORE_URL")
        previous_ca = os.environ.pop("AGENT_CORE_CA_CERT", None)
        os.environ["AGENT_CORE_URL"] = "https://phanthy-motus:15678"
        try:
            status = self.device._t800_acp_preflight()
            self.assertEqual("error", status["state"])
            self.assertFalse(status["configured"])
            self.assertIn("AGENT_CORE_CA_CERT", status["last_error"])
        finally:
            if previous_url is None:
                os.environ.pop("AGENT_CORE_URL", None)
            else:
                os.environ["AGENT_CORE_URL"] = previous_url
            if previous_ca is not None:
                os.environ["AGENT_CORE_CA_CERT"] = previous_ca

    def test_acp_callback_failure_is_visible_and_success_recovers(self):
        import urllib.request as urllib_request

        previous_url = os.environ.get("AGENT_CORE_URL")
        previous_ca = os.environ.pop("AGENT_CORE_CA_CERT", None)
        original_urlopen = urllib_request.urlopen
        os.environ["AGENT_CORE_URL"] = "http://127.0.0.1:15678"
        attempts = 0

        def urlopen(*_args, **_kwargs):
            nonlocal attempts
            attempts += 1
            if attempts == 1:
                raise OSError("agent core unavailable")
            return types.SimpleNamespace(close=lambda: None)

        urllib_request.urlopen = urlopen
        try:
            self.device._t800_acp_preflight()
            self.assertFalse(self.device._t800_acp_notify(
                "failed_action", "completed", {}, "motion_recorder"
            ))
            failed = self.device._t800_acp_status()
            self.assertEqual("error", failed["state"])
            self.assertTrue(failed["configured"])
            self.assertIn("agent core unavailable", failed["last_error"])

            self.assertTrue(self.device._t800_acp_notify(
                "recovered_action", "completed", {}, "motion_recorder"
            ))
            recovered = self.device._t800_acp_status()
            self.assertEqual("ready", recovered["state"])
            self.assertIsNone(recovered["last_error"])
            self.assertIsNotNone(recovered["last_success_at"])
        finally:
            urllib_request.urlopen = original_urlopen
            if previous_url is None:
                os.environ.pop("AGENT_CORE_URL", None)
            else:
                os.environ["AGENT_CORE_URL"] = previous_url
            if previous_ca is not None:
                os.environ["AGENT_CORE_CA_CERT"] = previous_ca

    def test_gesture_uses_validated_real_device_trajectories(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        waits = []
        plan.wait_until_idle = lambda *_args, **kwargs: waits.append(("idle", kwargs)) or {}
        plan.wait_for_request = lambda request_id, *_args: waits.append(("request", request_id)) or {}
        gesture = self.device.GesturePlugin(plan)
        self.device._notify_acp_completion = lambda *_args, **_kwargs: None

        wave = gesture._prepare_steps(gesture._official_steps("wave_hands"))
        self.assertEqual("return_to_neutral", wave[-1]["name"])
        self.assertEqual(gesture._NEUTRAL, wave[-1]["target_positions"])
        self.assertTrue(all(not step["stiffness"] for step in wave))

        handshake = gesture._prepare_steps(gesture._official_steps("shake_hand"))
        self.assertEqual(2.0, handshake[0]["hold_after_sec"])
        self.assertEqual(gesture._HAND_WITHDRAWN, handshake[-1]["target_positions"])

        result = gesture.dispatch("play", {
            "name": "wave_hands", "reset_after": False, "force": True,
        })
        self.assertEqual("running", result["state"])
        gesture._thread.join(timeout=1.0)
        self.assertEqual("completed", gesture.dispatch("status", {})["state"])
        self.assertEqual(8, len([item for item in waits if item[0] == "request"]))
        self.assertEqual(gesture._NEUTRAL, plan._publisher.messages[-1].target_positions)

    def test_gesture_rejects_unsafe_state_repetition_and_joint_target(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        gesture = self.device.GesturePlugin(plan)
        self.assertIn(
            "lower_body_balance",
            gesture.dispatch("play", {"name": "wave_hands"})["error"],
        )
        self.assertIn(
            "thermal safety",
            gesture.dispatch("play", {
                "name": "wave_hands", "repetitions": 2, "force": True,
            })["error"],
        )
        result = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [16], "target_positions": [-2.28]}],
            "force": True,
        })
        self.assertIn("safe position limit", result["error"])

    def test_joint_plan_waits_for_exact_executing_then_idle_request(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan._on_state(JointMotionPlanState(7, JointMotionPlanState.EXECUTING, 0.5))
        plan._on_state(JointMotionPlanState(7, JointMotionPlanState.IDLE, 1.0))
        state = plan.wait_for_request(7, 0.1, threading.Event())
        self.assertEqual(7, state["request_id"])
        self.assertEqual(JointMotionPlanState.IDLE, state["status"])

    def test_joint_override_force_path_and_release(self):
        plugin = self.device.JointOverridePlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        plugin.dispatch("command", {"joint_indices": [14], "position": [0.3], "duration": -1, "force": True})
        time.sleep(0.03)
        plugin.dispatch("release", {})
        self.assertEqual(0.0, plugin._publisher.messages[-1].weight)

    def test_joint_bridge_force_path_and_damping_stop(self):
        plugin = self.device.JointBridgePlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        plugin.dispatch("command", {"position": [0.0] * 25, "duration": -1, "force": True})
        time.sleep(0.02)
        plugin.dispatch("stop_command", {})
        self.assertEqual([1.0] * 25, plugin._publisher.messages[-1].damping)

    def test_led_tts_and_motor_service_paths(self):
        led = self.device.LedPlugin(CONFIG, "robot", self.ros)
        led.start()
        self.assertEqual("set", led.dispatch("led", {"mode": "breathe_red"})["state"])
        self.assertEqual(9, led._publisher.messages[-1].color)

        tts = self.device.TtsPlugin(CONFIG, "robot", self.ros)
        tts.start()
        self.assertEqual("published", tts.dispatch("tts", {"text": "你好", "rate": 150})["state"])
        self.assertEqual("你好", tts._publisher.messages[-1].text)

        motor = self.device.MotorPowerPlugin(CONFIG, "robot", self.ros)
        motor.start()
        result = motor.dispatch("disable", {})
        self.assertTrue(result["success"])
        self.assertFalse(result["enabled"])

        motor._client = types.SimpleNamespace(service_is_ready=lambda: False)
        unavailable = motor.dispatch("start", {})
        self.assertEqual("error", unavailable["state"])

    def test_mic_uses_pulseaudio_capture_and_publishes_pcm_chunks(self):
        plugin = self.device.MicPlugin(CONFIG, "robot", self.ros)

        class FakeStdout:
            def __init__(self):
                self.reads = [b"\x01\x00" * 512, b""]

            def read(self, _size):
                return self.reads.pop(0) if self.reads else b""

        class FakeProcess:
            def __init__(self):
                self.stdout = FakeStdout()
                self.returncode = None

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = 0

            def wait(self, timeout=None):
                return self.returncode

            def kill(self):
                self.returncode = -9

        process = FakeProcess()
        plugin._check_pulse = lambda: None
        plugin._spawn_capture = lambda: process
        self.assertEqual("running", plugin.dispatch("start", {})["state"])
        deadline = time.monotonic() + 1
        while not plugin._publisher.messages and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual("audio/pcm-16k", plugin._publisher.messages[0].format)
        self.assertEqual(1024, len(plugin._publisher.messages[0].data))
        health = plugin.health_sources()["microphone"]
        self.assertEqual("running", health["state"])
        self.assertFalse(health["stale"])
        self.assertEqual(1, health["chunks_published"])
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        stopped_health = plugin.health_sources()["microphone"]
        self.assertEqual("idle", stopped_health["state"])
        self.assertTrue(stopped_health["stale"])

    def test_speaker_is_a_canvas_pcm_sink_per_official_audio_interface(self):
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)
        tool = plugin.get_tool()
        self.assertEqual([{"format": "audio/pcm-16k"}], tool["topic_in"])
        self.assertEqual(
            ["start", "stop", "info", "get_volume", "set_volume"],
            tool["inputSchema"]["properties"]["action"]["enum"],
        )
        self.assertIn("set_volume", tool["inputSchema"]["x-action-params"])
        missing = plugin.dispatch("start", {})
        self.assertEqual("error", missing["state"])
        self.assertEqual("Missing input_topic", missing["error"])

        class FakeStdin:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(data)

            def flush(self):
                pass

            def close(self):
                pass

        class FakeProcess:
            def __init__(self):
                self.stdin = FakeStdin()
                self.returncode = None

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = 0

            def wait(self, timeout=None):
                return self.returncode

            def kill(self):
                self.returncode = -9

        process = FakeProcess()
        commands = []
        plugin._check_pulse = lambda: None
        plugin._run_command = lambda command: commands.append(command) or ""
        plugin._spawn_player = lambda: process
        plugin._enqueue_startup_sound = lambda *args: None  # 跳过开机音
        started = plugin.dispatch("start", {"input_topic": "/perception/tts"})
        self.assertEqual("ready", started["state"])
        self.assertEqual("/perception/tts", started["topic_in"][0]["topic"])
        # 启动时按官方接口解除静音
        self.assertIn(["pactl", "set-sink-mute", "@DEFAULT_SINK@", "0"], commands)

        # 画布 PCM 块经 aplay stdin 流式播放；EOF magic 不入流
        plugin._on_chunk(types.SimpleNamespace(format="audio/pcm-16k", data=[1, 2, 3, 4]))
        plugin._on_chunk(types.SimpleNamespace(format="audio/pcm-16k",
                                               data=list(plugin._EOF_MAGIC)))
        deadline = time.monotonic() + 1
        while not process.stdin.writes and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual([b"\x01\x02\x03\x04"], process.stdin.writes)
        self.assertEqual("playing", plugin.dispatch("info", {})["state"])
        time.sleep(0.35)
        self.assertEqual("ready", plugin.dispatch("info", {})["state"])
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertEqual([], plugin._node.subscriptions)

    def test_speaker_discards_callbacks_from_previous_session(self):
        # review 反馈:stop() 后残留的 ROS 回调若在新会话播放器就绪后执行,
        # 会把旧 topic 的 PCM 入队泄漏到新连接——回调须绑定会话。
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        old_session = plugin._session
        # 旧会话回调(模拟 stop 后仍被派发)必须被丢弃
        plugin._on_chunk(types.SimpleNamespace(format="audio/pcm-16k",
                                               data=[1, 2, 3, 4]), session=old_session)
        self.assertEqual(0, plugin._queue.qsize())
        # 未绑定会话且未运行的回调也丢弃
        plugin._running = False
        plugin._on_chunk(types.SimpleNamespace(format="audio/pcm-16k", data=[1, 2, 3, 4]))
        self.assertEqual(0, plugin._queue.qsize())
        # 当前会话回调正常入队
        plugin._running = True
        plugin._on_chunk(types.SimpleNamespace(format="audio/pcm-16k",
                                               data=[1, 2, 3, 4]), session=plugin._session)
        self.assertEqual(1, plugin._queue.qsize())

    def test_speaker_sliding_window_drops_oldest_on_overflow(self):
        # 持续实时流(remote_mic)发布速率高于播放速率时,队列满应丢最旧块
        # 而非新块,保证播放跟随最新输入(否则永远滞后 6 秒+)。
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        plugin._running = True
        # 队列 maxsize=50,塞 55 块,验证尾部(最新)数据保留、头部(最旧)被丢
        for i in range(55):
            plugin._on_chunk(types.SimpleNamespace(
                format="audio/pcm-16k", data=list(bytes([i & 0xFF]) * 1024)))
        qsize = plugin._queue.qsize()
        self.assertEqual(50, qsize)
        self.assertEqual(plugin._dropped, 5)
        head = bytes(plugin._queue.queue[0])[0]
        self.assertEqual(5, head)  # 0-4 被丢,队首是最新保留块中最旧的
        tail = bytes(plugin._queue.queue[-1])[0]
        self.assertEqual(54, tail)  # 最新的 54 在队尾

    def test_speaker_volume_uses_official_pactl_interface(self):
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)
        commands = []
        plugin._run_command = lambda command: commands.append(command) or "Volume: front-left: 32768 /  50% / -18.06 dB"
        result = plugin.dispatch("get_volume", {})
        self.assertEqual(50, result["volume"])
        self.assertEqual(["pactl", "get-sink-volume", "@DEFAULT_SINK@"], commands[-1])
        result = plugin.dispatch("set_volume", {"volume": 80})
        self.assertEqual(80, result["volume"])
        self.assertEqual(["pactl", "set-sink-volume", "@DEFAULT_SINK@", "80%"], commands[-1])
        # 越界值收敛到 0-100；解析失败走 error 路径
        self.assertEqual(100, plugin.dispatch("set_volume", {"volume": 999})["volume"])
        plugin._run_command = lambda command: "unparseable"
        self.assertIn("error", plugin.dispatch("get_volume", {}))

    def test_speaker_aplay_command_flags_for_low_latency(self):
        # 真机延迟根因:aplay 默认 500ms 读前缓冲 + ALSA dmix ~341ms 缓冲。
        # aplay 需带 --buffer-time/--period-time;默认设备经 /etc/asound.conf
        # 路由到 PulseAudio(pactl 音量才能作用于播放输出)。
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)
        calls = []

        class FakeProc:
            def __init__(self):
                self.stdin = types.SimpleNamespace(
                    write=lambda data: None, flush=lambda: None, close=lambda: None)
                self.returncode = None

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = 0

            def wait(self, timeout=None):
                return self.returncode

            def kill(self):
                self.returncode = -9

        def fake_popen(argv, **kwargs):
            calls.append(argv)
            return FakeProc()

        original = self.device.subprocess.Popen
        self.device.subprocess.Popen = fake_popen
        try:
            plugin._check_pulse = lambda: None
            plugin._run_command = lambda command: ""
            result = plugin.dispatch("start", {"input_topic": "/test/pcm"})
        finally:
            self.device.subprocess.Popen = original
        self.assertEqual("ready", result["state"])
        self.assertEqual(
            ["aplay", "-q", "-t", "raw", "-f", "S16_LE", "-r", "16000", "-c", "1",
             "--buffer-time=100000", "--period-time=20000", "-"],
            calls[-1],
        )

    def test_speaker_startup_asset_is_packaged_like_g1_without_external_fetch(self):
        dockerfile = (ROOT / "Dockerfile").read_text()
        startup_beep = ROOT / "resource" / "startup_beep.pcm"
        g1_startup_beep = (
            ROOT.parents[1]
            / "unitree"
            / "g1"
            / "resource"
            / "startup_beep.pcm"
        )
        self.assertTrue(startup_beep.is_file())
        self.assertEqual(256000, startup_beep.stat().st_size)
        self.assertLess(startup_beep.stat().st_size, 500 * 1024)
        self.assertEqual(
            g1_startup_beep.read_bytes(),
            startup_beep.read_bytes(),
        )
        self.assertEqual(
            "e634d402feeead175e7a669a77fa8d6aa5770e162fbd3c867503d4897dc2f166",
            hashlib.sha256(startup_beep.read_bytes()).hexdigest(),
        )
        self.assertIn("COPY resource/ /work/resource/", dockerfile)
        self.assertIn(
            "e634d402feeead175e7a669a77fa8d6aa5770e162fbd3c867503d4897dc2f166  "
            "/work/resource/startup_beep.pcm",
            dockerfile,
        )
        self.assertIn("sha256sum -c -", dockerfile)
        self.assertNotIn("STARTUP_BEEP_URL", dockerfile)
        self.assertNotIn("raw.githubusercontent.com", dockerfile)

    def test_speaker_aplay_exit_sets_error(self):
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)

        class DeadProcess:
            def __init__(self):
                self.stdin = None
                self.returncode = 1

            def poll(self):
                return self.returncode

            def terminate(self):
                pass

            def wait(self, timeout=None):
                return self.returncode

            def kill(self):
                pass

        plugin._check_pulse = lambda: None
        plugin._run_command = lambda command: ""
        plugin._spawn_player = lambda: DeadProcess()
        plugin._enqueue_startup_sound = lambda *args: None  # 跳过开机音
        started = plugin.dispatch("start", {"input_topic": "/perception/tts"})
        self.assertEqual("error", started["state"])
        self.assertIn("aplay exited", started["message"])

    def test_speaker_stop_is_idempotent_when_player_already_exited(self):
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)

        class ClosedStdin:
            def close(self):
                raise BrokenPipeError("aplay pipe is already closed")

        class ReapedProcess:
            def __init__(self):
                self.stdin = ClosedStdin()
                self.kill_calls = 0

            def poll(self):
                return 0

            def kill(self):
                self.kill_calls += 1
                raise ProcessLookupError("aplay is already reaped")

        process = ReapedProcess()
        plugin._process = process
        plugin._running = True
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
        self.assertEqual(0, process.kill_calls)
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])

    def test_speaker_startup_sound_plays_on_dispatch_start(self):
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)

        class FakeStdin:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(data)

            def flush(self):
                pass

        class FakeProcess:
            def __init__(self):
                self.stdin = FakeStdin()
                self.returncode = None

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = 0

            def wait(self, timeout=None):
                return self.returncode

            def kill(self):
                self.returncode = -9

        import pathlib
        process = FakeProcess()
        plugin._check_pulse = lambda: None
        plugin._run_command = lambda command: ""
        plugin._spawn_player = lambda: process
        # _enqueue_startup_sound 分块推入 _beep_queue，测试用假 PCM 模拟
        real_startup = plugin._enqueue_startup_sound
        startup_queued = []
        def fake_enqueue(*args):
            for _ in range(3):
                plugin._beep_queue.put_nowait(b"\x01\x02\x03")
            startup_queued.append(True)
        plugin._enqueue_startup_sound = fake_enqueue
        started = plugin.dispatch("start", {"input_topic": "/perception/tts"})
        self.assertEqual("ready", started["state"])
        self.assertTrue(startup_queued)
        self.assertEqual(3, plugin._beep_queue.qsize())
        # dispatch(start) 先返回再异步入队，播放线程已就绪

    def test_speaker_live_audio_preempts_racing_startup_beep_block(self):
        # review 反馈:beep producer 可能在 active 检查后、put 前暂停；live
        # callback drain 完毕后它再放入一块。播放器必须丢弃该迟到 beep，
        # 让首个 live PCM 成为下一块输出。
        real_queue = self.device.queue.Queue
        real_thread = self.device.threading.Thread
        real_read_bytes = Path.read_bytes
        beep_queues = []

        class GatePutQueue(real_queue):
            def __init__(self):
                super().__init__(maxsize=256)
                self.put_started = threading.Event()
                self.release_put = threading.Event()
                self._gated = False

            def put(self, item, block=True, timeout=None):
                if not self._gated:
                    self._gated = True
                    self.put_started.set()
                    self.release_put.wait(timeout=2)
                return super().put(item, block=block, timeout=timeout)

        def queue_factory(maxsize=0):
            if maxsize == 256:
                result = GatePutQueue()
                beep_queues.append(result)
                return result
            return real_queue(maxsize=maxsize)

        deferred_player = []

        def thread_factory(*args, **kwargs):
            if kwargs.get("name") == "t800-speaker":
                thread = DeferredThread(kwargs["target"], kwargs.get("args", ()))
                deferred_player.append(thread)
                return thread
            return real_thread(*args, **kwargs)

        self.device.queue.Queue = queue_factory
        self.device.threading.Thread = thread_factory
        Path.read_bytes = lambda path: b"\x00" * 2048
        plugin = None
        player_runner = None
        try:
            plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)
            process = FakeAudioProcess()
            plugin._check_pulse = lambda: None
            plugin._run_command = lambda command: ""
            plugin._spawn_player = lambda: process
            self.assertEqual(
                "ready",
                plugin.dispatch("start", {"input_topic": "/perception/tts"})["state"],
            )
            self.assertEqual(1, len(deferred_player))
            active_beep_queue = plugin._beep_queue
            self.assertTrue(active_beep_queue.put_started.wait(timeout=1))

            live_pcm = b"\x55\x66\x77\x88"
            plugin._subscription.callback(types.SimpleNamespace(
                format="audio/pcm-16k", data=list(live_pcm)))
            active_beep_queue.release_put.set()
            deadline = time.monotonic() + 1
            while active_beep_queue.qsize() == 0 and time.monotonic() < deadline:
                time.sleep(0.01)

            deferred = deferred_player[0]
            player_runner = real_thread(target=deferred.target, args=deferred.args)
            player_runner.start()
            deadline = time.monotonic() + 0.5
            while not process.stdin.writes and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual(live_pcm, process.stdin.writes[0])
        finally:
            for beep_queue in beep_queues:
                beep_queue.release_put.set()
            if plugin is not None:
                plugin.stop()
            if player_runner is not None:
                player_runner.join(timeout=1)
            Path.read_bytes = real_read_bytes
            self.device.queue.Queue = real_queue
            self.device.threading.Thread = real_thread

    def test_speaker_stale_startup_thread_cannot_write_into_restarted_session(self):
        # review 反馈:旧会话的开机音线程若延迟到 stop/start 后才运行，不能把
        # 旧开机音写进新会话。通过延迟线程调度稳定复现该竞态，并从新 aplay
        # 进程的输出边界验证没有跨会话音频泄漏。
        plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)

        class FakeStdin:
            def __init__(self):
                self.writes = []

            def write(self, data):
                self.writes.append(data)

            def flush(self):
                pass

            def close(self):
                pass

        class FakeProcess:
            def __init__(self):
                self.stdin = FakeStdin()
                self.returncode = None

            def poll(self):
                return self.returncode

            def terminate(self):
                self.returncode = 0

            def wait(self, timeout=None):
                return self.returncode

            def kill(self):
                self.returncode = -9

        processes = []
        plugin._check_pulse = lambda: None
        plugin._run_command = lambda command: ""

        def spawn_player():
            process = FakeProcess()
            processes.append(process)
            return process

        plugin._spawn_player = spawn_player

        real_thread = self.device.threading.Thread
        real_read_bytes = Path.read_bytes
        deferred_beep_threads = []

        class DeferredThread:
            def __init__(self, target, args):
                self.target = target
                self.args = args

            def start(self):
                pass

        def controlled_thread(*args, **kwargs):
            if kwargs.get("name") == "t800-beep-enqueue":
                thread = DeferredThread(kwargs["target"], kwargs.get("args", ()))
                deferred_beep_threads.append(thread)
                return thread
            return real_thread(*args, **kwargs)

        self.device.threading.Thread = controlled_thread
        Path.read_bytes = lambda path: b"\x00" * 2048
        try:
            self.assertEqual(
                "ready",
                plugin.dispatch("start", {"input_topic": "/perception/tts/old"})["state"],
            )
            self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
            self.assertEqual(
                "ready",
                plugin.dispatch("start", {"input_topic": "/perception/tts/new"})["state"],
            )
            self.assertEqual(2, len(deferred_beep_threads))

            stale_thread = deferred_beep_threads[0]
            stale_thread.target(*stale_thread.args)
            deadline = time.monotonic() + 0.2
            while not processes[-1].stdin.writes and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual([], processes[-1].stdin.writes)
        finally:
            Path.read_bytes = real_read_bytes
            self.device.threading.Thread = real_thread
            plugin.stop()

    def test_speaker_old_player_cannot_consume_restarted_session_audio(self):
        # review 反馈:旧播放线程可能已阻塞在 queue.get() 中；stop() 的 join
        # 超时后若新会话复用同一队列，旧线程会取走并丢弃新会话首块音频。
        real_queue = self.device.queue.Queue
        real_thread = self.device.threading.Thread

        class BlockingGetQueue(real_queue):
            def __init__(self):
                super().__init__(maxsize=50)
                self.get_started = threading.Event()
                self.release_get = threading.Event()

            def get(self, block=True, timeout=None):
                self.get_started.set()
                self.release_get.wait(timeout=2)
                return super().get(block=block, timeout=timeout)

        blocking_queue = BlockingGetQueue()
        live_queue_count = 0

        def queue_factory(maxsize=0):
            nonlocal live_queue_count
            if maxsize == 50:
                live_queue_count += 1
                # 当前实现只在 __init__ 创建一次；per-session 实现会在首个
                # start 再创建一次。两种情况下旧会话都使用这个受控队列。
                if live_queue_count <= 2:
                    return blocking_queue
            return real_queue(maxsize=maxsize)

        speaker_thread_count = 0
        deferred_player = []

        def thread_factory(*args, **kwargs):
            nonlocal speaker_thread_count
            if kwargs.get("name") == "t800-speaker":
                speaker_thread_count += 1
                if speaker_thread_count == 2:
                    thread = DeferredThread(kwargs["target"], kwargs.get("args", ()))
                    deferred_player.append(thread)
                    return thread
            return real_thread(*args, **kwargs)

        self.device.queue.Queue = queue_factory
        self.device.threading.Thread = thread_factory
        plugin = None
        old_player = None
        new_player_runner = None
        try:
            plugin = self.device.SpeakerPlugin(CONFIG, "robot", self.ros)
            processes = []
            plugin._check_pulse = lambda: None
            plugin._run_command = lambda command: ""
            plugin._enqueue_startup_sound = lambda *args: None

            def spawn_player():
                process = FakeAudioProcess()
                processes.append(process)
                return process

            plugin._spawn_player = spawn_player
            self.assertEqual(
                "ready",
                plugin.dispatch("start", {"input_topic": "/perception/tts/old"})["state"],
            )
            old_player = plugin._thread
            self.assertTrue(blocking_queue.get_started.wait(timeout=1))

            self.assertEqual("idle", plugin.dispatch("stop", {})["state"])
            self.assertEqual(
                "ready",
                plugin.dispatch("start", {"input_topic": "/perception/tts/new"})["state"],
            )
            self.assertEqual(1, len(deferred_player))

            live_pcm = b"\x11\x22\x33\x44"
            plugin._subscription.callback(types.SimpleNamespace(
                format="audio/pcm-16k", data=list(live_pcm)))
            blocking_queue.release_get.set()
            old_player.join(timeout=1)

            deferred = deferred_player[0]
            new_player_runner = real_thread(target=deferred.target, args=deferred.args)
            new_player_runner.start()
            deadline = time.monotonic() + 0.3
            while not processes[-1].stdin.writes and time.monotonic() < deadline:
                time.sleep(0.01)
            self.assertEqual([live_pcm], processes[-1].stdin.writes)
        finally:
            blocking_queue.release_get.set()
            if plugin is not None:
                plugin.stop()
            if old_player is not None:
                old_player.join(timeout=1)
            if new_player_runner is not None:
                new_player_runner.join(timeout=1)
            self.device.queue.Queue = real_queue
            self.device.threading.Thread = real_thread

    def test_native_node_control_and_composed_safety(self):
        native = self.device.NativeNodeControlPlugin(CONFIG, "robot", self.ros)
        native.start()
        result = native.dispatch("start_node", {"node_name": "hardware_interface_node"})
        self.assertEqual("requested", result["state"])
        self.assertTrue(native._publisher.messages[-1].command)
        self.assertEqual("hardware_interface_node", native._publisher.messages[-1].node_name)

        class ActiveControl:
            def __init__(self):
                self.stopped = False

            def stop(self):
                self.stopped = True

        active_controls = [ActiveControl(), ActiveControl()]
        safety = self.device.SafetyControlPlugin(CONFIG, "robot", self.ros, self.state)
        safety.set_controls(active_controls)
        safety.start()
        result = safety.dispatch("emergency_passive", {})
        self.assertEqual("passive", result["target_motion"])
        self.assertEqual([0.0, 0.0], safety._body_pub.messages[-1].linear_velocity)
        self.assertEqual(0.0, safety._override_pub.messages[-1].weight)
        self.assertEqual([1.0] * 25, safety._joint_pub.messages[-1].damping)
        self.assertTrue(all(control.stopped for control in active_controls))

    def test_safety_prefers_non_destructive_halt_over_lifecycle_stop(self):
        class ActiveControl:
            def __init__(self):
                self.halted = False
                self.stopped = False

            def halt(self):
                self.halted = True

            def stop(self):
                self.stopped = True

        control = ActiveControl()
        safety = self.device.SafetyControlPlugin(CONFIG, "robot", self.ros, self.state)
        safety.set_controls([control])
        safety.start()

        safety.dispatch("emergency_passive", {})

        self.assertTrue(control.halted)
        self.assertFalse(control.stopped)

    def test_vision_pointcloud_passthrough_binary_header(self):
        import struct

    def test_vision_pointcloud_passthrough_binary_header(self):
        import struct

        plugin = self.device.VisionPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        data = bytes(range(64))  # 4 点 × 16 字节 point_step
        plugin._on_cloud_raw(types.SimpleNamespace(point_step=16, data=data))
        out = plugin._cloud_pub.messages[-1]
        self.assertEqual(struct.pack("<II", 16, 4), bytes(out.data[:8]))
        self.assertEqual(bytes(range(64)), bytes(out.data[8:]))
        self.assertEqual(1, plugin._frames["pointcloud"])
        plugin = self.device.VisionPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        data = bytes(range(64))  # 4 点 × 16 字节 point_step
        plugin._on_cloud_raw(types.SimpleNamespace(point_step=16, data=data))
        out = plugin._cloud_pub.messages[-1]
        self.assertEqual(struct.pack("<II", 16, 4), bytes(out.data[:8]))
        self.assertEqual(bytes(range(64)), bytes(out.data[8:]))
        self.assertEqual(1, plugin._frames["pointcloud"])
    def test_vision_pointcloud_passthrough_binary_header(self):
        import struct

        plugin = self.device.VisionPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        data = bytes(range(64))  # 4 点 × 16 字节 point_step
        plugin._on_cloud_raw(types.SimpleNamespace(point_step=16, data=data))
        out = plugin._cloud_pub.messages[-1]
        self.assertEqual(struct.pack("<II", 16, 4), bytes(out.data[:8]))
        self.assertEqual(bytes(range(64)), bytes(out.data[8:]))
        self.assertEqual(1, plugin._frames["pointcloud"])
        health = plugin.health_sources()["odin2_pointcloud"]
        self.assertEqual("running", health["state"])
        self.assertFalse(health["stale"])

    def test_vision_select_source_switches_cloud(self):
        plugin = self.device.VisionPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        plugin.dispatch("select_source", {"source": "slam"})
        self.assertEqual("slam", plugin._source)
        data = bytes(32)
        plugin._on_cloud_raw(types.SimpleNamespace(point_step=16, data=data))  # raw 被忽略
        self.assertEqual(0, len(plugin._cloud_pub.messages))
        plugin._on_cloud_slam(types.SimpleNamespace(point_step=16, data=data))
        self.assertEqual(1, len(plugin._cloud_pub.messages))
        info = plugin.dispatch("info", {})
        self.assertEqual("slam", info["source"])
        self.assertEqual(4, len(info["topic_out"]))
        self.assertEqual("sensor/pointcloud", info["topic_out"][0]["format"])

    def test_vision_lifecycle_resolves_and_stops_only_the_requested_card(self):
        plugin = self.device.VisionPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        tools = {tool["name"]: tool for tool in plugin.get_tools()}
        cloud_schema = tools["pointcloud"]["inputSchema"]
        self.assertIn("select_source", cloud_schema["properties"]["action"]["enum"])
        self.assertEqual(["raw", "slam"], cloud_schema["properties"]["source"]["enum"])

        for tool_name, tool in tools.items():
            info = plugin.dispatch("info", {"_tool_name": tool_name})
            self.assertEqual(tool["topic_out"], info["topic_out"], tool_name)

        plugin._updated["depth"] = time.monotonic()
        stopped = plugin.dispatch("stop", {"_tool_name": "depth"})
        camera = plugin.dispatch("info", {"_tool_name": "camera"})
        self.assertEqual("idle", stopped["state"])
        self.assertEqual("idle", stopped["health"]["odin2_depth"]["state"])
        self.assertTrue(stopped["health"]["odin2_depth"]["stale"])
        self.assertEqual("running", camera["state"])
        self.assertTrue(plugin._running)
        self.assertNotIn("depth", plugin._enabled_tools)

        plugin.stop()
        restarted = plugin.dispatch("start", {"_tool_name": "pointcloud"})
        self.assertEqual("running", restarted["state"])
        self.assertEqual({"pointcloud"}, plugin._enabled_tools)

    def test_vision_camera_passthrough_and_native_depth_conversion(self):
        plugin = self.device.VisionPlugin(CONFIG, "robot", self.ros)
        plugin.start()
        left = types.SimpleNamespace(format="jpeg", data=bytes([0xFF, 0xD8]))
        right = types.SimpleNamespace(format="jpeg", data=bytes([0xFF, 0xD9]))
        plugin._on_camera_left(left)
        plugin._on_camera_right(right)
        self.assertIs(left, plugin._cam_left_pub.messages[-1])
        self.assertIs(right, plugin._cam_right_pub.messages[-1])
        # The official pcd2depth node publishes camera optical-axis depth in
        # metres.  Include invalid values and an over-range value to exercise
        # normalization into the dashboard's uint16 millimetre contract.
        values = (
            2.0, float("nan"), -1.0, 70.0,
            1.5, 1.0, 0.0, 0.5,
            3.0, 2.5, 2.0, 1.5,
        )
        depth = types.SimpleNamespace(
            header=types.SimpleNamespace(stamp="source-stamp", frame_id="camera"),
            width=4,
            height=3,
            encoding="32FC1",
            is_bigendian=False,
            step=16,
            data=struct.pack("<12f", *values),
        )
        plugin._on_depth(depth)
        out = plugin._depth_pub.messages[-1]
        self.assertEqual("16UC1", out.encoding)
        self.assertEqual((640, 480, 1280), (out.width, out.height, out.step))
        self.assertEqual(640 * 480 * 2, len(out.data))
        self.assertEqual("source-stamp", out.header.stamp)
        self.assertEqual("camera", out.header.frame_id)
        row = struct.unpack("<640H", bytes(out.data[:1280]))
        self.assertEqual((2000, 0, 0, 65535), (row[0], row[160], row[320], row[480]))
        tools = {tool["name"]: tool for tool in plugin.get_tools()}
        self.assertEqual("image/jpeg", tools["camera"]["topic_out"][0]["format"])
        self.assertEqual(2, len(tools["camera"]["topic_out"]))
        self.assertEqual("image/depth-z16", tools["depth"]["topic_out"][0]["format"])

    def test_head_declares_lifecycle_and_completion_schema(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        schema = head.get_tool()["inputSchema"]
        self.assertEqual(
            ["nod", "shake", "look", "rotate_to", "reset"],
            schema["x-completion"]["actions"],
        )
        self.assertEqual(
            {"on_interrupt_motion": {"action": "stop"}},
            schema["x-hooks"],
        )
        actions = set(schema["properties"]["action"]["enum"])
        self.assertTrue({"start", "info", "stop", "status"}.issubset(actions))
        self.assertTrue({"nod", "shake", "look", "rotate_to", "reset"}.issubset(actions))
        params = schema["x-action-params"]
        self.assertEqual(
            ["pitch_deg", "yaw_deg", "rotation_time", "duration"],
            params["rotate_to"]["params"],
        )
        self.assertEqual(["times", "speed"], params["nod"]["params"])

    def test_head_acp_timeout_covers_max_rotate_to(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        timeout = head.get_tool()["inputSchema"]["x-completion"]["timeout"]
        grace = float(head._config.get("feedback_grace_sec", 1.0))
        budget = (
            head._READY_TIMEOUT_SEC
            + head._ROTATION_TIME_MAX_SEC + grace   # 就绪 + 目标规划
            + head._HOLD_DURATION_MAX_SEC            # 保持
            + head._ROTATION_TIME_MAX_SEC + grace   # 复位规划
            + head._ACP_CALLBACK_TIMEOUT_SEC         # 完成回调
        )
        self.assertGreaterEqual(timeout, budget)

    def test_head_lifecycle_returns_plain_dict(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        for action in ("start", "info"):
            result = head.dispatch(action, {})
            self.assertEqual("ready", result["state"])
            self.assertEqual(
                {"pitch": [-0.5, 0.5], "yaw": [-1.0, 1.0]},
                result["limits_rad"],
            )
        status = head.dispatch("status", {})
        self.assertEqual("idle", status["state"])
        self.assertIn("limits_rad", status)
        self.assertIn("joint_plan", status)

    def test_head_rotate_to_rejects_out_of_range_angles(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        rejected = head.dispatch("rotate_to", {"pitch_deg": 90.0, "yaw_deg": 0.0})
        self.assertIn("pitch_deg must be between", rejected["error"])
        self.assertIsNone(head._thread)

    def test_head_repeat_validation_rejects_non_integer_times(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        for bad in (1.9, True):
            rejected = head.dispatch("nod", {"times": bad})
            self.assertIn("times must be an integer", rejected["error"])
            self.assertIsNone(head._thread)

    def test_head_rejects_unsafe_config_at_init(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        with self.assertRaises(ValueError):
            self.device.HeadActuatorPlugin({"nod_amplitude_rad": 2.0}, plan, self.state)
        with self.assertRaises(ValueError):
            self.device.HeadActuatorPlugin(
                {"look_poses": {"forward": {"pitch_rad": 0.9, "yaw_rad": 0.0}}},
                plan,
                self.state,
            )

    def test_head_rejects_incomplete_look_poses_at_init(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        for pose in ({"pitch_rad": 0.0}, {"yaw_rad": 0.0}, {}):
            with self.assertRaises(ValueError):
                self.device.HeadActuatorPlugin(
                    {"look_poses": {"forward": pose}}, plan, self.state
                )

    def test_head_rejects_invalid_timing_config_at_init(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        for grace in (-1.0, 0.0, float("nan"), float("inf"), 100.0):
            with self.assertRaises(ValueError):
                self.device.HeadActuatorPlugin(
                    {"feedback_grace_sec": grace}, plan, self.state
                )
        for step in (-0.35, 0.0, float("nan"), float("inf"), 20.0):
            with self.assertRaises(ValueError):
                self.device.HeadActuatorPlugin(
                    {"step_duration_sec": step}, plan, self.state
                )

    def test_head_nod_shake_clamp_step_duration_to_rotation_bounds(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        head = self.device.HeadActuatorPlugin(
            {"step_duration_sec": 0.1}, plan, self.state
        )
        for steps in (
            head._nod_steps({"times": 1, "speed": 2.0}),
            head._shake_steps({"times": 1, "speed": 2.0}),
        ):
            for step in steps:
                self.assertEqual(head._ROTATION_TIME_MIN_SEC, step["duration"])
        head = self.device.HeadActuatorPlugin(
            {"step_duration_sec": 10.0}, plan, self.state
        )
        for steps in (
            head._nod_steps({"times": 1, "speed": 0.5}),
            head._shake_steps({"times": 1, "speed": 0.5}),
        ):
            for step in steps:
                self.assertEqual(head._ROTATION_TIME_MAX_SEC, step["duration"])

    def test_head_completion_timeout_grows_with_worst_case_config(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        default = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        default_timeout = default.get_tool()["inputSchema"]["x-completion"]["timeout"]
        pathological = self.device.HeadActuatorPlugin(
            {"step_duration_sec": 10.0, "feedback_grace_sec": 5.0}, plan, self.state
        )
        grown_timeout = pathological.get_tool()["inputSchema"]["x-completion"]["timeout"]
        self.assertEqual(int(default._ACP_TIMEOUT_SEC), default_timeout)
        self.assertGreater(grown_timeout, default_timeout)

    def test_head_ownership_blocks_joint_plan_head_pose(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        self.assertIsNone(plan.acquire_head("head"))
        busy = plan.dispatch("head_pose", {"pitch_rad": 0.0, "yaw_rad": 0.0})
        self.assertIn("head is busy", busy["error"])
        self.assertEqual("head", busy["owner"])
        plan.release_head("head")

    def test_head_ownership_blocks_non_head_joint_plan_during_execution(self):
        """head 执行期间，非 head 关节的 joint_plan.plan 也应被拒绝，避免 superseded。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_args, **_kwargs: {}
        entered_wait = threading.Event()

        def wait_for_request(_request_id, _timeout, cancel_event):
            entered_wait.set()
            while not cancel_event.is_set():
                time.sleep(0.005)
            raise RuntimeError("cancelled")

        plan.wait_for_request = wait_for_request
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        self.device._notify_acp_completion = lambda *_args, **_kwargs: None
        head.dispatch("nod", {"times": 1})
        self.assertTrue(entered_wait.wait(timeout=1.0))
        # head 执行中，针对手臂关节的 plan 应被拒绝
        busy = plan.dispatch("plan", {
            "joint_indices": [13, 14, 15, 16, 17],
            "target_positions": [0.0] * 5,
        })
        self.assertIn("head is busy", busy["error"])
        self.assertEqual("head", busy["owner"])
        head.dispatch("stop", {})
        head._thread.join(timeout=1.0)

    def test_direct_joint_plan_head_pose_blocks_subsequent_non_head_plan(self):
        """直接 joint_plan.head_pose 持有锁后，后续非 head plan 应被拒绝（owner=joint_plan 不重入）。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        # 直接调用 head_pose，owner="joint_plan" 获取锁
        result = plan.dispatch("head_pose", {"pitch_rad": 0.1, "yaw_rad": 0.0})
        self.assertEqual("requested", result["state"])
        self.assertEqual("joint_plan", plan.head_status()["owner"])
        # 后续非 head plan 应被拒绝（同一个外部 owner 不允许重入）
        busy = plan.dispatch("plan", {
            "joint_indices": [13, 14, 15, 16, 17],
            "target_positions": [0.0] * 5,
        })
        self.assertIn("head is busy", busy["error"])
        self.assertEqual("joint_plan", busy["owner"])

    def test_direct_head_request_not_released_by_initial_idle(self):
        """直接 head 请求在初始 IDLE 确认时不应释放锁，必须等 EXECUTING→终态。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        result = plan.dispatch("head_pose", {"pitch_rad": 0.1, "yaw_rad": 0.0})
        req_id = result["request_id"]
        # 初始 IDLE（planner 在 EXECUTING 前发布）不应释放锁
        plan._on_state(JointMotionPlanState(req_id, JointMotionPlanState.IDLE, 0.0))
        self.assertEqual("joint_plan", plan.head_status()["owner"])
        # EXECUTING 后再 IDLE 才释放
        plan._on_state(JointMotionPlanState(req_id, JointMotionPlanState.EXECUTING, 0.5))
        plan._on_state(JointMotionPlanState(req_id, JointMotionPlanState.IDLE, 1.0))
        self.assertIsNone(plan.head_status()["owner"])

    def test_direct_head_request_released_on_cancel_before_executing(self):
        """直接 head 请求在 EXECUTING 前被 cancel 时，锁应立即释放，不永久持有。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        result = plan.dispatch("head_pose", {"pitch_rad": 0.1, "yaw_rad": 0.0})
        req_id = result["request_id"]
        self.assertEqual("joint_plan", plan.head_status()["owner"])
        # 未发布 EXECUTING，直接 cancel
        cancelled = plan.dispatch("cancel", {"request_id": req_id})
        self.assertEqual("requested", cancelled["state"])
        # cancel 后锁应立即释放
        self.assertIsNone(plan.head_status()["owner"])
        # 后续请求应能正常获取锁
        again = plan.dispatch("head_pose", {"pitch_rad": 0.2, "yaw_rad": 0.0})
        self.assertEqual("requested", again["state"])

    def test_direct_head_request_released_on_exiting_before_executing(self):
        """直接 head 请求在 EXECUTING 前收到 EXITING（故障/拒绝）时应释放锁。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        result = plan.dispatch("head_pose", {"pitch_rad": 0.1, "yaw_rad": 0.0})
        req_id = result["request_id"]
        self.assertEqual("joint_plan", plan.head_status()["owner"])
        # planner 在 EXECUTING 前发布 EXITING（拒绝/故障）
        plan._on_state(JointMotionPlanState(req_id, JointMotionPlanState.EXITING, 0.0))
        self.assertIsNone(plan.head_status()["owner"])
        # 后续 head 动作不应被永久阻塞
        again = plan.dispatch("head_pose", {"pitch_rad": 0.2, "yaw_rad": 0.0})
        self.assertEqual("requested", again["state"])

    def test_direct_reset_released_on_fault_before_executing(self):
        """直接 reset 在 EXECUTING 前被 planner 拒绝/故障时应释放 head 锁。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        result = plan.dispatch("reset", {})
        req_id = result["request_id"]
        self.assertEqual("joint_plan", plan.head_status()["owner"])
        # planner 在 EXECUTING 前发布 DISABLED（故障/禁用）
        plan._on_state(JointMotionPlanState(req_id, JointMotionPlanState.STATUS_DISABLED, 0.0))
        self.assertIsNone(plan.head_status()["owner"])
        # 后续 head 动作不应被永久阻塞
        again = plan.dispatch("head_pose", {"pitch_rad": 0.1, "yaw_rad": 0.0})
        self.assertEqual("requested", again["state"])

    def test_head_nod_shake_rejects_invalid_speed_types(self):
        """speed 为 null/数组/对象时应返回 error，而非 TypeError 逃逸。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        for speed in (None, [], {}):
            with self.subTest(speed=speed):
                result = head.dispatch("nod", {"times": 1, "speed": speed})
                self.assertIn("error", result)
                result = head.dispatch("shake", {"times": 1, "speed": speed})
                self.assertIn("error", result)

    def test_head_look_works_without_configured_look_poses(self):
        """配置中没有 look_poses 时，应使用内置默认值，look 动作正常可用。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_args, **_kwargs: {}
        plan.wait_for_request = lambda *_args, **_kwargs: {}
        # 不传 look_poses 的配置
        minimal_config = {"enabled": True, "step_duration_sec": 0.35}
        head = self.device.HeadActuatorPlugin(minimal_config, plan, self.state)
        self.device._notify_acp_completion = lambda *_args, **_kwargs: None
        result = head.dispatch("look", {"direction": "forward"})
        self.assertEqual("running", result["state"])
        head._thread.join(timeout=1.0)
        self.assertEqual("completed", head.dispatch("status", {})["state"])

    def test_head_async_completion_reports_acp(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_args, **_kwargs: {}
        plan.wait_for_request = lambda *_args, **_kwargs: {}
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        completions = []
        self.device._notify_acp_completion = lambda tool, action_id, status, result, timeout: completions.append(
            (tool, action_id, status, result)
        )
        result = head.dispatch("nod", {"times": 1})
        self.assertEqual("running", result["state"])
        head._thread.join(timeout=1.0)
        self.assertEqual("completed", head.dispatch("status", {})["state"])
        self.assertEqual(1, len(completions))
        self.assertEqual("head", completions[0][0])
        self.assertEqual(result["action_id"], completions[0][1])
        self.assertEqual("completed", completions[0][2])

    def test_head_stop_cancels_and_reports_acp(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_args, **_kwargs: {}
        entered_wait = threading.Event()

        def wait_for_request(_request_id, _timeout, cancel_event):
            entered_wait.set()
            while not cancel_event.is_set():
                time.sleep(0.005)
            raise RuntimeError("cancelled")

        plan.wait_for_request = wait_for_request
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        completions = []
        self.device._notify_acp_completion = lambda tool, action_id, status, result, timeout: completions.append(
            (tool, action_id, status, result)
        )
        result = head.dispatch("nod", {"times": 1})
        self.assertTrue(entered_wait.wait(timeout=1.0))
        stopped = head.dispatch("stop", {})
        self.assertEqual("idle", stopped["state"])
        head._thread.join(timeout=1.0)
        self.assertEqual("cancelled", head.dispatch("status", {})["state"])
        self.assertEqual(1, len(completions))
        self.assertEqual("head", completions[0][0])
        self.assertEqual(result["action_id"], completions[0][1])
        self.assertEqual("cancelled", completions[0][2])

    def test_head_interrupt_stays_gated_until_worker_and_planner_are_idle(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_args, **_kwargs: {}
        entered_wait = threading.Event()
        release_wait = threading.Event()

        def wait_for_request(_request_id, _timeout, _cancel_event):
            entered_wait.set()
            release_wait.wait(timeout=1.0)
            return {}

        plan.wait_for_request = wait_for_request
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        self.device._notify_acp_completion = lambda *_args, **_kwargs: None
        group = self.device.MotionInterruptGroup()
        sibling_stops = []
        group.register("head", head.halt, head.motion_active)
        group.register(
            "motion_recorder",
            lambda: sibling_stops.append(True) or {"state": "idle"},
            lambda: False,
        )
        head.set_interrupt_group(group)

        started = head.dispatch("nod", {"times": 1})
        self.assertTrue(entered_wait.wait(timeout=1.0))
        stop_started = time.monotonic()
        stopped = head.dispatch("stop", {})
        self.assertLess(time.monotonic() - stop_started, 0.05)
        self.assertIn("motion_recorder", stopped["interrupted_outputs"])
        self.assertEqual([True], sibling_stops)
        self.assertEqual(["head"], group.blocking_outputs())

        request_id = plan._publisher.messages[0].request_id
        plan._on_state(JointMotionPlanState(
            request_id, JointMotionPlanState.IDLE, 1.0
        ))
        release_wait.set()
        head._thread.join(timeout=1.0)
        deadline = time.monotonic() + 1.0
        while group.blocking_outputs() and time.monotonic() < deadline:
            time.sleep(0.01)
        self.assertEqual([], group.blocking_outputs())
        self.assertFalse(head.dispatch("status", {})["cancel_pending"])

    def test_head_timeout_cancels_active_request_before_releasing_lease(self):
        """wait_for_request 超时时必须先 cancel 在飞请求，再释放 head 锁。"""
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        plan.wait_until_idle = lambda *_a, **_kw: {}

        def wait_for_request(_request_id, _timeout, cancel_event):
            # planner 已报告 EXECUTING 但在超时时间内未完成（反馈超时）
            raise TimeoutError("joint planner did not complete request")

        plan.wait_for_request = wait_for_request
        head = self.device.HeadActuatorPlugin(CONFIG, plan, self.state)
        self.device._notify_acp_completion = lambda *_a, **_kw: None

        result = head.dispatch("nod", {"times": 1})
        self.assertIn("action_id", result)
        head._thread.join(timeout=2.0)
        self.assertFalse(head._thread.is_alive())

        # 超时后应已发布 cancel 消息
        cancel_msgs = [
            m for m in plan._publisher.messages
            if int(m.request_type) == JointMotionPlanRequest.REQUEST_CANCEL
        ]
        self.assertTrue(cancel_msgs, "timeout path must publish a cancel request")
        # head 锁应已释放
        self.assertIsNone(plan.head_status()["owner"])
        # 下一个 head 动作应能正常获取锁
        plan.wait_for_request = lambda *_a, **_kw: {}
        again = head.dispatch("nod", {"times": 1})
        self.assertIn("action_id", again)
        head._thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
