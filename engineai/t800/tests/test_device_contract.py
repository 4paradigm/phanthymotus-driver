import importlib.util
import struct
import sys
import time
import types
import unittest
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


class EnableMotor:
    class Request:
        enable = False


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


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
        "BodyVelCmd", "GamepadKeys", "ImuInfo", "JointCommand", "JointMotionPlanState",
        "JointOverrideCommand", "JointState", "LedControl", "MotionState", "MotionStateRequest",
        "Heartbeat", "LinkInfo", "MotorDebug", "NodeControl", "PowerInfo", "Tts",
    ):
        setattr(protocol_msg, name, type(name, (Message,), {}))
    protocol_msg.JointMotionPlanRequest = JointMotionPlanRequest
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
                "max_vx": 3.0, "max_vy": 1.0, "max_vyaw": 3.14, "mode_transition_timeout_sec": 0.1},
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
            self.device.JointOverridePlugin(CONFIG, "robot", self.ros, self.state),
            self.device.JointBridgePlugin(CONFIG, "robot", self.ros, self.state),
            self.device.LedPlugin(CONFIG, "robot", self.ros),
            self.device.TtsPlugin(CONFIG, "robot", self.ros),
            self.device.MicPlugin(CONFIG, "robot", self.ros),
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
             "loco", "motion_mode", "dance", "joint_plan", "joint_plan_state", "gesture",
             "joint_override", "joint_bridge",
             "led", "tts", "mic", "pointcloud", "camera", "depth",
             "motor_power", "native_node_control", "virtual_gamepad", "safety", "native_sdk"},
            names,
        )
        self.assertEqual(41, len(names))
        self.assertEqual(41, len(definitions), "tool names must be unique")
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
        self.assertEqual("0.98 m/s", snapshot["speed"])

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
        self.assertEqual("1.50 m/s", moving["speed"])
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

    def test_locomotion_force_path_publishes_and_stops(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        result = plugin.dispatch("move", {"vx": 9, "vy": -9, "vyaw": 9, "duration": 0.03, "force": True})
        self.assertEqual(3.0, result["vx"])
        self.assertEqual(-1.0, result["vy"])
        time.sleep(0.08)
        self.assertGreaterEqual(len(plugin._publisher.messages), 2)
        self.assertEqual(0.0, plugin._publisher.messages[-1].yaw_velocity)

    def test_locomotion_open_loop_composites(self):
        plugin = self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        move = plugin.dispatch("move_displacement", {
            "x_m": 1.0, "y_m": 0.0, "speed_m_s": 0.5, "force": True,
        })
        self.assertTrue(move["open_loop"])
        self.assertAlmostEqual(0.5, move["vx"])
        self.assertAlmostEqual(2.0, move["duration"])
        plugin.dispatch("stop_move", {})
        turn = plugin.dispatch("turn_angle", {
            "angle_rad": -1.0, "angular_speed_rad_s": 0.5, "force": True,
        })
        self.assertAlmostEqual(-0.5, turn["vyaw"])
        plugin.dispatch("stop_move", {})
        arc = plugin.dispatch("arc", {
            "radius_m": 1.0, "angle_rad": 1.0, "linear_speed_m_s": 0.5, "force": True,
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
        self.assertEqual("supine_to_stance", plugin._publisher.messages[-1].target_motion_name)

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

    def test_joint_plan_named_head_arm_and_hold_actions(self):
        plugin = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plugin.start()
        named = plugin.dispatch("plan_named", {
            "joint_names": ["J23_HEAD_PITCH", "J24_HEAD_YAW"],
            "target_positions": [0.1, -0.2], "duration": 1.0,
        })
        self.assertEqual("requested", named["state"])
        self.assertEqual([23, 24], plugin._publisher.messages[-1].joint_indices)
        plugin.dispatch("head_pose", {"pitch_rad": 0.2, "yaw_rad": 0.3})
        self.assertEqual([23, 24], plugin._publisher.messages[-1].joint_indices)
        plugin.dispatch("arm_pose", {"side": "left", "target_positions": [0.0] * 5})
        self.assertEqual([13, 14, 15, 16, 17], plugin._publisher.messages[-1].joint_indices)
        plugin.dispatch("hold_current", {})
        self.assertEqual(list(range(25)), plugin._publisher.messages[-1].joint_indices)

    def test_gesture_exposes_complete_official_sequences_and_custom_queue(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        gesture = self.device.GesturePlugin(plan)
        listed = {item["name"]: item["steps"] for item in gesture.dispatch("list", {})["gestures"]}
        self.assertEqual(7, listed["wave_hands"])
        self.assertEqual(2, listed["shake_hand"])
        result = gesture.dispatch("sequence", {
            "steps": [{"joint_indices": [23, 24], "target_positions": [0.1, -0.1], "duration": 0.05}],
            "reset_after": False,
            "wait": True,
        })
        self.assertEqual("completed", result["state"])
        self.assertEqual([23, 24], plan._publisher.messages[-1].joint_indices)

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
                return self.reads.pop(0)

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
        self.assertEqual("idle", plugin.dispatch("stop", {})["state"])

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

        stopped = plugin.dispatch("stop", {"_tool_name": "depth"})
        camera = plugin.dispatch("info", {"_tool_name": "camera"})
        self.assertEqual("idle", stopped["state"])
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


if __name__ == "__main__":
    unittest.main()
