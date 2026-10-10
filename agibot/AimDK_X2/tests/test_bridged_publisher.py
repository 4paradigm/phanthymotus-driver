"""Unit tests for the ROS-domain Unix socket publisher."""

from __future__ import annotations

import json
import socket
import struct
import sys
import types
import unittest
from pathlib import Path
from unittest import mock

DEVICE_DIR = Path(__file__).resolve().parent.parent
if str(DEVICE_DIR) not in sys.path:
    sys.path.insert(0, str(DEVICE_DIR))

import x2_bridged_publisher as bridge  # noqa: E402


class BridgedPublisherTests(unittest.TestCase):
    def test_type_name_uses_ros_interface_module(self):
        message = type("String", (), {})
        message.__module__ = "std_msgs.msg._string"
        self.assertEqual(bridge._type_name(message), "std_msgs/msg/String")

    def test_integer_depth_preserves_default_reliable_qos_in_registration(self):
        message = type("String", (), {})
        message.__module__ = "std_msgs.msg._string"
        publisher = bridge.BridgedPublisher(message, "/test/topic", 5)
        fake_socket = mock.Mock()

        with mock.patch.object(socket, "socket", return_value=fake_socket):
            self.assertTrue(publisher._connect())

        registration = fake_socket.sendall.call_args_list[0].args[0]
        size = struct.unpack("<I", registration[:4])[0]
        metadata = json.loads(registration[4:4 + size])
        self.assertEqual(metadata["qos"], {
            "reliability": "reliable",
            "durability": "volatile",
            "history": "keep_last",
            "depth": 5,
        })

    def test_explicit_best_effort_qos_is_preserved(self):
        policy = lambda name: types.SimpleNamespace(name=name)
        qos = types.SimpleNamespace(
            reliability=policy("BEST_EFFORT"),
            durability=policy("VOLATILE"),
            history=policy("KEEP_LAST"),
            depth=7,
        )
        self.assertEqual(bridge._qos_metadata(qos), {
            "reliability": "best_effort",
            "durability": "volatile",
            "history": "keep_last",
            "depth": 7,
        })

    def test_failed_send_disconnects_for_next_publish(self):
        publisher = bridge.BridgedPublisher(types.SimpleNamespace, "/test/topic")
        publisher._connected = True
        publisher._socket = mock.Mock()
        publisher._socket.sendall.side_effect = BrokenPipeError
        serialization = types.ModuleType("rclpy.serialization")
        serialization.serialize_message = lambda message: b"payload"

        with mock.patch.dict(sys.modules, {"rclpy.serialization": serialization}):
            publisher.publish(object())

        self.assertFalse(publisher._connected)
        self.assertIsNone(publisher._socket)

    def test_publish_retries_connection_after_bridge_starts(self):
        publisher = bridge.BridgedPublisher(types.SimpleNamespace, "/test/topic", 5)
        unavailable = mock.Mock()
        unavailable.connect.side_effect = FileNotFoundError
        available = mock.Mock()
        serialization = types.ModuleType("rclpy.serialization")
        serialization.serialize_message = lambda message: b"payload"

        with mock.patch.object(socket, "socket", side_effect=[unavailable, available]), \
             mock.patch.dict(sys.modules, {"rclpy.serialization": serialization}):
            publisher.publish(object())
            publisher.publish(object())

        self.assertTrue(unavailable.close.called)
        self.assertTrue(publisher._connected)
        self.assertGreaterEqual(available.sendall.call_count, 2)


if __name__ == "__main__":
    unittest.main()
