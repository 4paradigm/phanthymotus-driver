import importlib.util
import math
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
    std_msgs.String = type("String", (Message,), {"__init__": lambda self: setattr(self, "data", "")})
    std_msgs.UInt8MultiArray = type(
        "UInt8MultiArray",
        (Message,),
        {"__init__": lambda self: setattr(self, "data", [])},
    )
    sys.modules["std_msgs.msg"] = std_msgs

    protocol_msg = types.ModuleType("interface_protocol.msg")
    for name in (
        "BodyVelCmd", "GamepadKeys", "ImuInfo", "JointCommand", "JointMotionPlanState",
        "JointOverrideCommand", "JointState", "LedControl", "MotionState", "MotionStateRequest",
        "MotorDebug", "NodeControl", "PowerInfo", "Tts",
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
        "odometry": "/manifold/odin1/device0/odometry",
        "cloud_slam": "/manifold/odin1/device0/cloud/slam",
    },
    "plugins": {
        "odometry": {"enabled": True, "stale_timeout_sec": 1.0},
        "mapping": {
            "enabled": True,
            "resolution_m": 0.1,
            "z_min_m": 0.1,
            "z_max_m": 1.8,
            "publish_hz": 1.0,
            "stale_timeout_sec": 1.0,
        },
    },
    "services": {"enable_motor": "/hardware/enable_motor"},
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
        odometry = self.device.OdometryPlugin(CONFIG, "robot", self.ros)
        gesture = self.device.GesturePlugin(joint_plan)
        plugins = [
            self.state,
            odometry,
            self.device.WaypointPlugin(odometry),
            self.device.MappingPlugin(CONFIG, "robot", self.ros, odometry=odometry),
            self.device.LocomotionPlugin(CONFIG, "robot", self.ros, self.state),
            motion_mode,
            self.device.DancePlugin(motion_mode, self.state),
            joint_plan,
            gesture,
            self.device.PoseTeachPlugin(self.state, gesture),
            self.device.JointOverridePlugin(CONFIG, "robot", self.ros, self.state),
            self.device.JointBridgePlugin(CONFIG, "robot", self.ros, self.state),
            self.device.LedPlugin(CONFIG, "robot", self.ros),
            self.device.TtsPlugin(CONFIG, "robot", self.ros),
            self.device.MotorPowerPlugin(CONFIG, "robot", self.ros),
            self.device.NativeNodeControlPlugin(CONFIG, "robot", self.ros),
            self.device.SafetyControlPlugin(CONFIG, "robot", self.ros, self.state),
            self.device.NativeSdkPlugin({"mode": "external"}, "robot", self.ros),
            VirtualGamepadPlugin({}, "robot", self.ros),
        ]
        names = set()
        for plugin in plugins:
            tools = plugin.get_tools() if hasattr(plugin, "get_tools") else [plugin.get_tool()]
            names.update(tool["name"] for tool in tools)
        self.assertEqual(
            {"joints", "imu", "battery", "motor_health", "motor_state", "motor_command", "joint_command_feedback",
             "gamepad", "motion_state", "driver_health", "model",
             "robot_snapshot", "fault_summary", "stability", "joint_groups", "capabilities", "ros_graph",
             "odometry", "waypoint", "mapping",
             "loco", "motion_mode", "dance", "joint_plan", "joint_plan_state", "gesture", "pose_teach",
             "joint_override", "joint_bridge",
             "led", "tts", "motor_power", "native_node_control", "virtual_gamepad", "safety", "native_sdk"},
            names,
        )
        self.assertEqual(36, len(names))

    def test_odometry_bridge_normalizes_pose_and_stale(self):
        plugin = self.device.OdometryPlugin(CONFIG, "robot", self.ros)
        tool = plugin.get_tool()
        self.assertEqual("odometry", tool["name"])
        self.assertEqual("/robot/state/odometry", tool["topic_out"][0]["topic"])

        empty = plugin.dispatch("odometry", {})
        self.assertEqual("no_data", empty["state"])
        self.assertTrue(empty["stale"])

        msg = Message()
        msg.header = types.SimpleNamespace(
            frame_id="odom",
            stamp=types.SimpleNamespace(sec=12, nanosec=250_000_000),
        )
        msg.child_frame_id = "base_link"
        msg.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=1.5, y=-0.5, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        msg.twist = types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.3, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.05),
            )
        )
        plugin._on_odometry(msg)
        snapshot = plugin.dispatch("odometry", {})
        self.assertEqual("ok", snapshot["state"])
        self.assertFalse(snapshot["stale"])
        self.assertEqual("odom", snapshot["frame_id"])
        self.assertAlmostEqual(1.5, snapshot["x"])
        self.assertAlmostEqual(-0.5, snapshot["y"])
        self.assertAlmostEqual(0.3, snapshot["vx"])
        self.assertIsNotNone(plugin.current_pose())

        plugin._updated = time.monotonic() - 2.0
        stale = plugin.dispatch("odometry", {})
        self.assertTrue(stale["stale"])
        self.assertEqual("stale", stale["state"])
        self.assertIsNone(plugin.current_pose())

    def test_waypoint_mark_list_distance_and_overwrite_guard(self):
        odometry = self.device.OdometryPlugin(CONFIG, "robot", self.ros)
        waypoint = self.device.WaypointPlugin(odometry)

        blocked = waypoint.dispatch("mark", {"name": "充电点"})
        self.assertIn("error", blocked)

        msg = Message()
        msg.header = types.SimpleNamespace(
            frame_id="odom",
            stamp=types.SimpleNamespace(sec=1, nanosec=0),
        )
        msg.child_frame_id = "base_link"
        msg.pose = types.SimpleNamespace(
            pose=types.SimpleNamespace(
                position=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                orientation=types.SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
            )
        )
        msg.twist = types.SimpleNamespace(
            twist=types.SimpleNamespace(
                linear=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
                angular=types.SimpleNamespace(x=0.0, y=0.0, z=0.0),
            )
        )
        odometry._on_odometry(msg)

        marked = waypoint.dispatch("mark", {"name": "充电点", "note": "大厅"})
        self.assertEqual("marked", marked["state"])
        self.assertEqual("充电点", marked["waypoint"]["name"])
        self.assertEqual("大厅", marked["waypoint"]["note"])

        duplicate = waypoint.dispatch("mark", {"name": "充电点"})
        self.assertIn("overwrite=true", duplicate["error"])

        msg.pose.pose.position.x = 3.0
        msg.pose.pose.position.y = 4.0
        odometry._on_odometry(msg)
        dist = waypoint.dispatch("distance_to", {"name": "充电点"})
        self.assertEqual("ok", dist["state"])
        self.assertAlmostEqual(5.0, dist["distance_m"])
        self.assertAlmostEqual(math.atan2(0.0 - 4.0, 0.0 - 3.0), dist["bearing_rad"])

        listed = waypoint.dispatch("list", {})
        self.assertEqual(1, listed["waypoint_count"])
        overwritten = waypoint.dispatch("mark", {"name": "充电点", "overwrite": True, "note": "更新"})
        self.assertEqual("updated", overwritten["state"])
        self.assertEqual("更新", overwritten["waypoint"]["note"])
        deleted = waypoint.dispatch("delete", {"name": "充电点"})
        self.assertEqual("deleted", deleted["state"])
        self.assertEqual(0, waypoint.dispatch("list", {})["waypoint_count"])

    def test_mapping_ingests_pointcloud_and_publishes_occupancy(self):
        import struct

        odometry = self.device.OdometryPlugin(CONFIG, "robot", self.ros)
        mapping = self.device.MappingPlugin(CONFIG, "robot", self.ros, odometry=odometry)
        tool = mapping.get_tool()
        self.assertEqual("mapping", tool["name"])
        self.assertEqual("sensor/mapping", tool["topic_out"][0]["format"])

        empty = mapping.dispatch("status", {})
        self.assertEqual("no_data", empty["state"])
        self.assertTrue(empty["stale"])

        # One FLOAT32 XYZ point at (0.15, 0.25, 0.5) → occupied cell near origin.
        point = struct.pack("<fff", 0.15, 0.25, 0.5)
        msg = Message()
        msg.header = types.SimpleNamespace(frame_id="map")
        msg.fields = [
            types.SimpleNamespace(name="x", offset=0, datatype=7),
            types.SimpleNamespace(name="y", offset=4, datatype=7),
            types.SimpleNamespace(name="z", offset=8, datatype=7),
        ]
        msg.point_step = 12
        msg.width = 1
        msg.height = 1
        msg.data = point
        mapping._on_cloud(msg)

        status = mapping.dispatch("status", {})
        self.assertEqual("ok", status["state"])
        self.assertEqual("map", status["frame_id"])
        self.assertGreaterEqual(status["occupied"], 1)
        self.assertFalse(status["path_planning"])

        mapping._running = True
        mapping._publish_tick()
        self.assertGreaterEqual(len(mapping._publisher.messages), 1)
        payload = bytes(mapping._publisher.messages[-1].data)
        robot_x, robot_y, robot_yaw, flags, count = struct.unpack_from("<fffBI", payload, 0)
        self.assertEqual(0x03, flags)
        self.assertGreaterEqual(count, 1)
        self.assertAlmostEqual(0.0, robot_x)

        mapping.dispatch("clear", {})
        cleared = mapping.dispatch("status", {})
        self.assertEqual("no_data", cleared["state"])

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

    def test_pose_teach_capture_preview_and_export(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        gesture = self.device.GesturePlugin(plan)
        teach = self.device.PoseTeachPlugin(self.state, gesture)

        self.state._last_joint_positions = [i * 0.01 for i in range(25)]

        empty = teach.dispatch("preview", {})
        self.assertIn("error", empty)

        first = teach.dispatch("capture", {"label": "raised", "duration": 1.5})
        self.assertEqual("captured", first["state"])
        self.assertEqual("raised", first["frame"]["label"])
        self.assertEqual(1.5, first["frame"]["duration"])
        self.assertEqual(13, len(first["frame"]["target_positions"]))
        self.assertAlmostEqual(0.12, first["frame"]["target_positions"][0])
        self.assertAlmostEqual(0.24, first["frame"]["target_positions"][-1])

        self.state._last_joint_positions = [0.0] * 25
        second = teach.dispatch("capture", {"duration": 0.5})
        self.assertEqual(2, second["frame_count"])

        listed = teach.dispatch("list", {})
        self.assertEqual(2, listed["frame_count"])
        self.assertEqual("raised", listed["frames"][0]["label"])

        updated = teach.dispatch("update", {"index": 1, "label": "neutral", "duration": 2.0})
        self.assertEqual("updated", updated["state"])
        self.assertEqual("neutral", updated["frame"]["label"])
        self.assertEqual(2.0, updated["frame"]["duration"])

        preview = teach.dispatch("preview", {"reset_after": False, "wait": True})
        self.assertEqual("completed", preview["state"])
        self.assertGreaterEqual(len(plan._publisher.messages), 2)
        self.assertEqual(list(range(12, 25)), plan._publisher.messages[-1].joint_indices)

        exported = teach.dispatch("export", {"format": "sequence"})
        self.assertEqual("exported", exported["state"])
        self.assertEqual(2, len(exported["steps"]))
        self.assertEqual(list(range(12, 25)), exported["steps"][0]["joint_indices"])
        self.assertEqual(list(self.device.GesturePlugin._BASE_STIFFNESS), exported["steps"][0]["stiffness"])
        self.assertEqual("gesture", exported["gesture_call"]["tool"])
        self.assertEqual("sequence", exported["gesture_call"]["action"])

        yaml_export = teach.dispatch("export", {"format": "yaml", "name": "heart"})
        self.assertIn('"heart": [', yaml_export["yaml"])
        self.assertIn("_BASE_STIFFNESS", yaml_export["yaml"])

        cleared = teach.dispatch("clear", {})
        self.assertEqual(0, len(cleared["frames"]))
        self.assertEqual(0, teach.dispatch("list", {})["frame_count"])

    def test_pose_teach_release_upper_cancels_and_softens(self):
        plan = self.device.JointPlanPlugin(CONFIG, "robot", self.ros, self.state)
        plan.start()
        gesture = self.device.GesturePlugin(plan)
        teach = self.device.PoseTeachPlugin(self.state, gesture)

        schema = teach.get_tool()["inputSchema"]
        self.assertIn("release_upper", schema["properties"]["action"]["enum"])
        self.assertNotIn("restore_balance", schema["properties"]["action"]["enum"])

        self.state._current_motion = "lower_body_balance"
        self.state._available_motions = ["walk", "lower_body_balance", "pd_stand"]
        self.state._last_joint_positions = [0.01] * 25
        released = teach.dispatch("release_upper", {})
        self.assertEqual("released", released["state"])
        self.assertEqual("lower_body_balance", released["motion"])
        self.assertEqual(1, plan._publisher.messages[0].request_type)
        soften = plan._publisher.messages[-1]
        self.assertEqual(0, soften.request_type)
        self.assertEqual(list(range(12, 25)), soften.joint_indices)
        self.assertEqual([0.0] * 13, list(soften.stiffness))
        self.assertEqual([1.0] * 13, list(soften.damping))
        self.assertIsNone(released.get("target"))

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


if __name__ == "__main__":
    unittest.main()
