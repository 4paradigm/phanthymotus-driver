#!/usr/bin/env python3
"""不依赖 ROS 安装的天轶2.0 原子传感器单元测试。"""

import importlib
import json
import sys
import types
import unittest
from pathlib import Path


class _Stamp:
    def __init__(self, sec=0, nanosec=0):
        self.sec = sec
        self.nanosec = nanosec


class _Header:
    def __init__(self, sec=0, nanosec=0, frame_id=""):
        self.stamp = _Stamp(sec, nanosec)
        self.frame_id = frame_id


class _String:
    def __init__(self):
        self.data = ""


class _Image:
    def __init__(self):
        self.header = _Header()
        self.height = 0
        self.width = 0
        self.encoding = ""
        self.is_bigendian = 0
        self.step = 0
        self.data = b""


class _Publisher:
    def __init__(self, msg_type, topic):
        self.msg_type = msg_type
        self.topic = topic
        self.messages = []

    def publish(self, msg):
        self.messages.append(msg)


class _Subscription:
    def __init__(self, msg_type, topic, callback):
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback


class _Logger:
    def warning(self, _message):
        pass


class _Node:
    def __init__(self, name, context=None):
        self.name = name
        self.context = context
        self.publishers = []
        self.subscriptions = []

    def create_publisher(self, msg_type, topic, _qos):
        publisher = _Publisher(msg_type, topic)
        self.publishers.append(publisher)
        return publisher

    def create_subscription(self, msg_type, topic, callback, _qos):
        subscription = _Subscription(msg_type, topic, callback)
        self.subscriptions.append(subscription)
        return subscription

    def destroy_publisher(self, publisher):
        self.publishers.remove(publisher)

    def destroy_subscription(self, subscription):
        self.subscriptions.remove(subscription)

    def get_logger(self):
        return _Logger()


class _QoSProfile:
    def __init__(self, **kwargs):
        self.settings = kwargs


class _Policy:
    RELIABLE = "reliable"
    BEST_EFFORT = "best_effort"
    KEEP_LAST = "keep_last"
    VOLATILE = "volatile"


class _Executor:
    def __init__(self):
        self.nodes = []

    def add_node(self, node):
        self.nodes.append(node)


class _Ros2:
    def __init__(self):
        self.ctx_tianyi = object()
        self.ctx_core = object()
        self.executor_tianyi = _Executor()
        self.executor_core = _Executor()


def _install_ros_stubs():
    rclpy = types.ModuleType("rclpy")
    rclpy_node = types.ModuleType("rclpy.node")
    rclpy_qos = types.ModuleType("rclpy.qos")
    std_msgs = types.ModuleType("std_msgs")
    std_msgs_msg = types.ModuleType("std_msgs.msg")
    sensor_msgs = types.ModuleType("sensor_msgs")
    sensor_msgs_msg = types.ModuleType("sensor_msgs.msg")

    rclpy_node.Node = _Node
    rclpy_qos.QoSProfile = _QoSProfile
    rclpy_qos.ReliabilityPolicy = _Policy
    rclpy_qos.HistoryPolicy = _Policy
    rclpy_qos.DurabilityPolicy = _Policy
    std_msgs_msg.String = _String
    sensor_msgs_msg.Image = _Image

    sys.modules.update(
        {
            "rclpy": rclpy,
            "rclpy.node": rclpy_node,
            "rclpy.qos": rclpy_qos,
            "std_msgs": std_msgs,
            "std_msgs.msg": std_msgs_msg,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
        }
    )


_install_ros_stubs()
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

camera_depth = importlib.import_module("camera_depth")
hand_state = importlib.import_module("hand_state")
remote_controller = importlib.import_module("remote_controller")


class AtomicSensorTests(unittest.TestCase):
    def test_tool_names_and_formats(self):
        ros2 = _Ros2()
        cases = [
            (
                hand_state.HandStatePlugin({}, "robot", ros2),
                "hand_state",
                "data/json",
            ),
            (
                camera_depth.CameraDepthPlugin({}, "robot", ros2),
                "camera_depth",
                "image/depth-z16",
            ),
            (
                remote_controller.RemoteControllerPlugin({}, "robot", ros2),
                "remote_controller",
                "data/json",
            ),
        ]

        for plugin, name, output_format in cases:
            with self.subTest(name=name):
                tool = plugin.get_tool()
                self.assertEqual(tool["name"], name)
                self.assertEqual(tool["type"], "sensor")
                self.assertEqual(tool["topic_out"][0]["format"], output_format)

    def test_hand_state_aggregates_both_hands(self):
        plugin = hand_state.HandStatePlugin({}, "robot", _Ros2())
        plugin._running = True
        left = types.SimpleNamespace(
            header=_Header(1, 2, "left"),
            name=["finger_1"],
            position=[0.25],
            velocity=[0.5],
            effort=[0.75],
        )
        right = types.SimpleNamespace(
            header=_Header(3, 4, "right"),
            name=["finger_2"],
            position=[0.4],
            velocity=[0.6],
            effort=[0.8],
        )

        plugin._on_hand_state("left", left)
        plugin._on_hand_state("right", right)

        payload = plugin.dispatch("hand_state", {})
        self.assertEqual(payload["left"]["position"], [0.25])
        self.assertEqual(payload["right"]["effort"], [0.8])
        published = json.loads(plugin._pub.messages[-1].data)
        self.assertEqual(published["left"]["name"], ["finger_1"])
        self.assertEqual(published["right"]["name"], ["finger_2"])

        published_count = len(plugin._pub.messages)
        plugin.stop()
        plugin._on_hand_state("left", left)
        self.assertEqual(len(plugin._pub.messages), published_count)

    def test_remote_controller_preserves_values_and_key_name(self):
        plugin = remote_controller.RemoteControllerPlugin({}, "robot", _Ros2())
        plugin._running = True
        msg = types.SimpleNamespace(
            header=_Header(10, 20, "sbus"),
            key_event_new=17,
            key_event_old=16,
            button_a=1,
            button_b=2,
            button_c=3,
            button_d=4,
            button_e=5,
            button_f=6,
            button_g=7,
            button_h=8,
            x1=0.1,
            y1=-0.2,
            x2=0.3,
            y2=-0.4,
        )

        plugin._on_remote_controller(msg)
        payload = plugin.dispatch("remote_controller", {})
        self.assertEqual(payload["key_event_new"], 17)
        self.assertEqual(payload["key_event_new_name"], "g_right")
        self.assertEqual(payload["button_h"], 8)
        self.assertAlmostEqual(payload["y2"], -0.4)

        published_count = len(plugin._pub.messages)
        plugin.stop()
        plugin._on_remote_controller(msg)
        self.assertEqual(len(plugin._pub.messages), published_count)

    def test_depth_conversion_removes_padding(self):
        plugin = camera_depth.CameraDepthPlugin({}, "robot", _Ros2())
        source = _Image()
        source.header = _Header(5, 6, "depth")
        source.width = 2
        source.height = 2
        source.encoding = "16UC1"
        source.step = 6
        source.data = b"\x01\x00\x02\x00xx\x03\x00\x04\x00yy"

        output = plugin._to_depth_z16(source)
        self.assertEqual(output.encoding, "16UC1")
        self.assertEqual(output.step, 4)
        self.assertEqual(output.data, b"\x01\x00\x02\x00\x03\x00\x04\x00")

    def test_depth_conversion_swaps_big_endian_pixels(self):
        plugin = camera_depth.CameraDepthPlugin({}, "robot", _Ros2())
        source = _Image()
        source.width = 2
        source.height = 1
        source.encoding = "mono16"
        source.is_bigendian = 1
        source.step = 4
        source.data = b"\x00\x01\x00\x02"

        output = plugin._to_depth_z16(source)
        self.assertEqual(output.is_bigendian, 0)
        self.assertEqual(output.data, b"\x01\x00\x02\x00")

    def test_depth_lifecycle_releases_resources_before_restart(self):
        plugin = camera_depth.CameraDepthPlugin({}, "robot", _Ros2())

        plugin.start()
        first_publisher = plugin._publisher
        plugin.stop()
        self.assertEqual(plugin._pub_node.publishers, [])
        self.assertEqual(plugin._sub_node.subscriptions, [])

        plugin.start()
        self.assertIsNot(plugin._publisher, first_publisher)
        self.assertEqual(len(plugin._pub_node.publishers), 1)
        self.assertEqual(len(plugin._sub_node.subscriptions), 1)
        plugin.stop()


if __name__ == "__main__":
    unittest.main()
