import importlib.util
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
             "loco", "motion_mode", "dance", "joint_plan", "joint_plan_state", "gesture",
             "joint_override", "joint_bridge",
             "led", "tts", "motor_power", "native_node_control", "virtual_gamepad", "safety", "native_sdk"},
            names,
        )
        self.assertEqual(32, len(names))

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
