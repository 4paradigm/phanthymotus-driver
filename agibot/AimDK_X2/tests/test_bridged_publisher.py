"""Unit tests for the ROS-domain Unix socket publisher."""

from __future__ import annotations

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


if __name__ == "__main__":
    unittest.main()
